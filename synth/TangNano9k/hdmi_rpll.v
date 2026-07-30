// rPLL wrapper: 27 MHz in -> 126 MHz out (= 5 x 25.2 MHz pixel clock,
// the closest integer-ratio match to the VESA 640x480@60 25.175 MHz
// pixel clock achievable from a 27 MHz reference).
// CLKOUT = FCLKIN*(FBDIV_SEL+1)/(IDIV_SEL+1) = 27*14/3 = 126 MHz
// VCO    = CLKOUT*ODIV_SEL = 126*4 = 504 MHz (valid range 400-1200 MHz
// for GW1NR-9C). Parameters computed with Apycula's own gowin_pll
// calculator (apycula.gowin_pll), which encodes the official Gowin
// rPLL formula/limits validated for this exact part.
// Renamed from Gowin_rPLL to hdmi_rpll: GHDL's default component binding is
// case-insensitive, so "Gowin_rPLL" would otherwise silently resolve to
// this project's unrelated VHDL entity "gowin_rpll" (the shared master/
// PSRAM PLL) instead of staying an unbound blackbox for yosys to bind here.
module hdmi_rpll (clkout, lock, clkin);

output clkout;
output lock;
input clkin;

wire clkoutp_o;
wire clkoutd_o;
wire clkoutd3_o;
wire gw_gnd;

assign gw_gnd = 1'b0;

rPLL rpll_inst (
    .CLKOUT(clkout),
    .LOCK(lock),
    .CLKOUTP(clkoutp_o),
    .CLKOUTD(clkoutd_o),
    .CLKOUTD3(clkoutd3_o),
    .RESET(gw_gnd),
    .RESET_P(gw_gnd),
    .CLKIN(clkin),
    .CLKFB(gw_gnd),
    .FBDSEL({gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd}),
    .IDSEL({gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd}),
    .ODSEL({gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd}),
    .PSDA({gw_gnd,gw_gnd,gw_gnd,gw_gnd}),
    .DUTYDA({gw_gnd,gw_gnd,gw_gnd,gw_gnd}),
    .FDLY({gw_gnd,gw_gnd,gw_gnd,gw_gnd})
);

defparam rpll_inst.FCLKIN = "27";
defparam rpll_inst.DYN_IDIV_SEL = "false";
defparam rpll_inst.IDIV_SEL = 2;        // /3
defparam rpll_inst.DYN_FBDIV_SEL = "false";
defparam rpll_inst.FBDIV_SEL = 13;      // x14 -> 126 MHz
defparam rpll_inst.DYN_ODIV_SEL = "false";
defparam rpll_inst.ODIV_SEL = 4;
defparam rpll_inst.PSDA_SEL = "0000";
defparam rpll_inst.DYN_DA_EN = "true";
defparam rpll_inst.DUTYDA_SEL = "1000";
defparam rpll_inst.CLKOUT_FT_DIR = 1'b1;
defparam rpll_inst.CLKOUTP_FT_DIR = 1'b1;
defparam rpll_inst.CLKOUT_DLY_STEP = 0;
defparam rpll_inst.CLKOUTP_DLY_STEP = 0;
defparam rpll_inst.CLKFB_SEL = "internal";
defparam rpll_inst.CLKOUT_BYPASS = "false";
defparam rpll_inst.CLKOUTP_BYPASS = "false";
defparam rpll_inst.CLKOUTD_BYPASS = "false";
defparam rpll_inst.DYN_SDIV_SEL = 2;
defparam rpll_inst.CLKOUTD_SRC = "CLKOUT";
defparam rpll_inst.CLKOUTD3_SRC = "CLKOUT";
defparam rpll_inst.DEVICE = "GW1NR-9C";

endmodule
