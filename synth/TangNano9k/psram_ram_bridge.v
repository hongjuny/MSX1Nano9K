//
// Bridges the MSX core's flat, "1-cycle-latency-shaped" RAM bus (ram_addr_o /
// ram_data_i/o / ram_ce_o / ram_oe_o / ram_we_o, all synchronous to
// clock_master_s / clock_cpu_s) onto the board's embedded PSRAM, which needs
// its own ~81MHz clock domain and has multi-cycle read/write latency
// (see psram_controller.v).
//
// The MSX core (memoryctl.vhd / msx.vhd) has no native ready/valid handshake
// for RAM - it assumes memory answers within essentially one clock, like the
// spram/dpram blocks used elsewhere in this design. To use the PSRAM instead
// we must stall the Z80 (T80) core with real Z80 wait states: ram_wait_n_o
// must be wired to the_msx's bus_wait_n_i port, which the original DECA
// top-level left tied to '1' (never used) because its SDRAM was fast enough
// relative to clock_master_s not to need it.
//
// Only PSRAM die 0 is used (4MB, addr[21:0]), which comfortably covers the
// 512KB window used by memoryctl's ramsize_g=512 address map (top address
// bits of ram_addr_o are always 0 in that configuration).
//
module psram_ram_bridge #(
    parameter FREQ    = 85_909_091,
    parameter LATENCY = 4
) (
    // Shared PLL (see gowin_rpll.vhd): clk/clk_p are the PSRAM-rate clock
    // and its 90-degree companion, pll_lock is that PLL's LOCK output.
    input  wire        clk,
    input  wire        clk_p,
    input  wire        pll_lock,
    input  wire        rst_n,          // async, e.g. btn_reset_n

    // CPU-side RAM bus - clock_master_s / clock_cpu_s domain.
    // Address/data are assumed stable for as long as ram_ce_i & (we_i|oe_i)
    // stay asserted, which holds true as long as we keep ram_wait_n_o low
    // (standard Z80/T80 bus behavior).
    input  wire [22:0] ram_addr_i,
    input  wire [7:0]  ram_wdata_i,
    output reg  [7:0]  ram_rdata_o,
    input  wire        ram_ce_i,
    input  wire        ram_oe_i,
    input  wire        ram_we_i,
    output reg         ram_wait_n_o,   // 1 = ready, 0 = hold the CPU (drive bus_wait_n_i)

    // PSRAM physical pins (GW1NR-9C SiP-embedded PSRAM, die 0 only)
    output wire [1:0]  O_psram_ck,
    output wire [1:0]  O_psram_ck_n,
    output wire [1:0]  O_psram_reset_n,
    inout  wire [1:0]  IO_psram_rwds,
    inout  wire [15:0] IO_psram_dq,
    output wire [1:0]  O_psram_cs_n,

    // Debug (LED instrumentation while bringing up the flash->PSRAM boot
    // path - see top_tangnano9k.vhd).
    output wire        dbg_resetn,
    output wire        dbg_req,
    output wire [1:0]  dbg_state,
    output wire        dbg_ctrl_busy,

    // DEBUG: read back PsramController's configured CR0 register (level
    // input, clock_master_s domain - synchronized/edge-detected internally,
    // one-shot use only) to check whether the boot-time LATENCY config
    // write actually took effect on the real chip.
    input  wire        cr_read_i,
    output wire [7:0]  cr_read_data_o,
    output wire        cr_read_valid_o
);

// ---------------------------------------------------------------------
// PSRAM clock domain: HyperRAM-style controller (clock comes from the
// shared PLL in gowin_rpll.vhd, not instantiated here)
// ---------------------------------------------------------------------

// Synchronize the external async reset into the psram clock domain.
reg [1:0] resetn_sync;
wire      resetn = resetn_sync[1] & pll_lock;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        resetn_sync <= 2'b00;
    else
        resetn_sync <= {resetn_sync[0], 1'b1};
end

assign O_psram_reset_n = {2{resetn}};
assign O_psram_ck_n    = 2'b00;   // not driven by psram_controller.v either; tied to a defined level

reg         ctrl_read, ctrl_write;
reg  [21:0] addr_lat;
reg  [15:0] din_word;
wire [15:0] dout_word;
wire        ctrl_busy;

// cr_read_i level -> synchronized one-cycle pulse in the `clk` (psram) domain
reg [2:0] cr_read_sync;
always @(posedge clk or negedge resetn) begin
    if (!resetn)
        cr_read_sync <= 3'b000;
    else
        cr_read_sync <= {cr_read_sync[1:0], cr_read_i};
end
wire cr_read_pulse = cr_read_sync[1] & ~cr_read_sync[2];

PsramController #(
    .FREQ(FREQ), .LATENCY(LATENCY)
) u_psram_ctrl (
    .clk(clk), .clk_p(clk_p), .resetn(resetn),
    .read(ctrl_read), .write(ctrl_write),
    .addr(addr_lat), .din(din_word), .byte_write(1'b1),
    .dout(dout_word), .busy(ctrl_busy),
    .O_psram_ck(O_psram_ck),
    .IO_psram_rwds(IO_psram_rwds),
    .IO_psram_dq(IO_psram_dq),
    .O_psram_cs_n(O_psram_cs_n),
    .cr_read(cr_read_pulse),
    .cr_read_data(cr_read_data_o),
    .cr_read_valid(cr_read_valid_o)
);

// ---------------------------------------------------------------------
// CPU request -> PSRAM clock domain synchronizer
// ---------------------------------------------------------------------
reg [1:0] ce_sync, we_sync, oe_sync;
always @(posedge clk or negedge resetn) begin
    if (!resetn) begin
        ce_sync <= 2'b00;
        we_sync <= 2'b00;
        oe_sync <= 2'b00;
    end else begin
        ce_sync <= {ce_sync[0], ram_ce_i};
        we_sync <= {we_sync[0], ram_we_i};
        oe_sync <= {oe_sync[0], ram_oe_i};
    end
end
wire req      = ce_sync[1] & (we_sync[1] | oe_sync[1]);
wire req_wr   = we_sync[1];

localparam ST_IDLE  = 2'd0,
           ST_ISSUE = 2'd1,
           ST_BUSY  = 2'd2,
           ST_HOLD  = 2'd3;
reg [1:0] state;

// Minimum guaranteed low-pulse width for ram_wait_n_o, in `clk` (psram
// domain) cycles. PsramController's READ_ST completion is edge-triggered
// on a real RWDS toggle from the physical chip (not self-timed like
// WRITE_ST's fixed LATENCY-based cycle count), so its busy pulse can be as
// short as ~9-10 cycles - only ~2-3 cycles as seen through the CPU-side
// 2-flop synchronizer (a much slower clock domain). A pulse that narrow is
// at real risk of being missed entirely by that synchronizer (classic CDC
// pulse-width problem - see rom_loader.vhd's near-identical writeup), which
// would let the CPU read/write one cycle too early against stale data.
// Forcibly holding ST_BUSY for at least this many cycles regardless of how
// fast the underlying controller finishes removes that risk for every
// consumer (CPU and rom_loader alike) without depending on each consumer
// protecting itself.
localparam MIN_HOLD_CYCLES = 16;
reg [4:0] hold_cnt;
reg       ctrl_busy_prev;

always @(posedge clk or negedge resetn) begin
    if (!resetn) begin
        state        <= ST_IDLE;
        ram_wait_n_o <= 1'b1;
        ctrl_read    <= 1'b0;
        ctrl_write   <= 1'b0;
        hold_cnt     <= 5'd0;
    end else begin
        ctrl_read  <= 1'b0;
        ctrl_write <= 1'b0;

        case (state)
        ST_IDLE: begin
            ram_wait_n_o <= 1'b1;
            if (req) begin
                // Bus has been stable for >=2 psram clocks by construction
                // (synchronizer delay), safe to capture combinationally here.
                addr_lat     <= ram_addr_i[21:0];
                din_word     <= {ram_wdata_i, ram_wdata_i};
                ram_wait_n_o <= 1'b0;
                hold_cnt     <= 5'd0;
                state        <= ST_ISSUE;
            end
        end

        ST_ISSUE: begin
            // one-cycle start pulse into PsramController
            if (req_wr) ctrl_write <= 1'b1;
            else        ctrl_read  <= 1'b1;
            state <= ST_BUSY;
        end

        ST_BUSY: begin
            ctrl_busy_prev <= ctrl_busy;
            if (ctrl_busy_prev && !ctrl_busy && !req_wr)
                ram_rdata_o <= addr_lat[0] ? dout_word[15:8] : dout_word[7:0];
            if (hold_cnt < MIN_HOLD_CYCLES[4:0])
                hold_cnt <= hold_cnt + 5'd1;
            if (!ctrl_busy && hold_cnt >= MIN_HOLD_CYCLES[4:0]) begin
                ram_wait_n_o <= 1'b1;   // release the CPU
                state        <= ST_HOLD;
            end
        end

        ST_HOLD: begin
            // Wait for the CPU to actually see wait_n=1 and drop its strobe
            // before allowing a new transaction to start.
            if (!req)
                state <= ST_IDLE;
        end

        default: state <= ST_IDLE;
        endcase
    end
end

assign dbg_resetn    = resetn;
assign dbg_req       = req;
assign dbg_state     = state;
assign dbg_ctrl_busy = ctrl_busy;

endmodule
