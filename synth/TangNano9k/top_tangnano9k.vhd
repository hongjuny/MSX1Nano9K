library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.msx_pack.all;

-- This top-level used to run the 32K BIOS/BASIC ROM and 512K-addressable
-- work RAM out of the board's embedded PSRAM (SiP HyperRAM). That path was
-- abandoned: extensive real-hardware debugging (a standalone quantitative
-- BIST tool, PLL frequency/phase sweeps, an nextpnr --seed sweep across 20
-- seeds - all hung) converged on near-zero PSRAM read timing margin that
-- the open-source toolchain couldn't reliably close (see git history for
-- gowin_rpll.vhd / psram_controller.v / psram_ram_bridge.v / psram_bist_top
-- .vhd for the full investigation).
--
-- Per-the-user's-direction pivot: get a minimal, old-school MSX1 actually
-- booting first (Daewoo DPC-100 style, 16K RAM, no mapper/Nextor/SD-card
-- features), using only resources that have already been proven reliable on
-- real hardware:
--   - ROM: served on-demand ("XIP") straight from SPI config flash, one
--     byte at a time (flash_rom_xip.vhd / flash_reader.vhd - the flash SPI
--     read path was verified byte-perfect against a 32K checksum earlier
--     this project). Slower than a cache (no prefetch), but correct.
--   - RAM: 16K of on-chip BRAM (spram.vhd, same primitive already used for
--     VRAM) - single-cycle synchronous, matches msx.vhd's native assumption
--     that RAM answers within one clock, so it needs no wait-states.
-- GW1NR-9C has 26 BSRAM blocks (~58.5KB) total; VRAM alone already uses ~8,
-- leaving no room for a full 32K ROM copy too (SPI-flash XIP avoids needing
-- one).
entity top_tangnano9k is
    port (
        -- 1. Clock & Reset
        clk_27m         : in  std_logic;                    -- Tang Nano 27MHz Onboard Clock
        btn_reset_n     : in  std_logic;                    -- Onboard S2 Button (Active Low Reset)

        -- 2. HDMI Output (DVI / TMDS)
        tmds_clk_p      : out std_logic;
        tmds_clk_n      : out std_logic;
        tmds_d_p        : out std_logic_vector(2 downto 0);
        tmds_d_n        : out std_logic_vector(2 downto 0);

        -- 3. PS/2 Keyboard Interface
        ps2_clk_io      : inout std_logic;
        ps2_data_io     : inout std_logic;

        -- 4. MicroSD Card Interface (SPI)
        sd_cs_n_o       : out std_logic;
        sd_sclk_o       : out std_logic;
        sd_mosi_o       : out std_logic;
        sd_miso_i       : in  std_logic;

        -- 5. Status LEDs
        leds_n_o        : out std_logic_vector(5 downto 0);

        -- 5b. External SPI config flash (spare space holds the BIOS ROM,
        -- read on-demand - see flash_rom_xip.vhd)
        flash_clk       : out std_logic;
        flash_ncs       : out std_logic;
        flash_do        : out std_logic;  -- flash MOSI
        flash_di        : in  std_logic   -- flash MISO
    );
end entity top_tangnano9k;

architecture behavior of top_tangnano9k is

    -- HDMI PLL: 27 MHz -> 126 MHz (5x TMDS serial clock)
    component hdmi_rpll
        port (
            clkout : out std_logic;
            lock   : out std_logic;
            clkin  : in  std_logic
        );
    end component;

    -- Divide the 126 MHz TMDS serial clock by 5 -> 25.2 MHz pixel clock
    component Gowin_CLKDIV
        port (
            clkout : out std_logic;
            hclkin : in  std_logic;
            resetn : in  std_logic
        );
    end component;

    component dvi_tx
        port (
            clk_pixel  : in  std_logic;
            clk_tmds   : in  std_logic;
            reset_n    : in  std_logic;
            red        : in  std_logic_vector(7 downto 0);
            green      : in  std_logic_vector(7 downto 0);
            blue       : in  std_logic_vector(7 downto 0);
            hsync      : in  std_logic;
            vsync      : in  std_logic;
            de         : in  std_logic;
            tmds_clk_p : out std_logic;
            tmds_clk_n : out std_logic;
            tmds_d_p   : out std_logic_vector(2 downto 0);
            tmds_d_n   : out std_logic_vector(2 downto 0)
        );
    end component;

    -- Clocks
    signal clock_master_s  : std_logic;  -- 21.5 MHz (MSX Master, ~0.1% off real 21.4773MHz)
    signal pll_locked_s    : std_logic;

    signal clock_vdp_s     : std_logic;
    signal clock_cpu_s     : std_logic;
    signal clock_psg_en_s  : std_logic;
    signal clock_3m_s      : std_logic;
    signal turbo_on_s      : std_logic;

    -- HDMI: 27MHz -> 126MHz (clk_tmds_s) -> /5 -> 25.2MHz (clk_pixel_s)
    signal clk_tmds_s      : std_logic;
    signal clk_pixel_s     : std_logic;
    signal hdmi_pll_lock_s : std_logic;
    signal hdmi_resetn_sync: std_logic_vector(3 downto 0) := (others => '0');
    signal hdmi_reset_n_s  : std_logic;

    -- Video signals: raw VDP output (clock_master_s domain)
    signal cnt_hor_s       : std_logic_vector(8 downto 0);
    signal cnt_ver_s       : std_logic_vector(7 downto 0);
    signal rgb_col_s       : std_logic_vector(3 downto 0);  -- palette index (video_opt_g=3)

    -- Video signals: scan-doubled VGA-timing output (clk_pixel_s domain)
    signal vga_col_s       : std_logic_vector(3 downto 0);
    signal vga_r_s         : std_logic_vector(3 downto 0);
    signal vga_g_s         : std_logic_vector(3 downto 0);
    signal vga_b_s         : std_logic_vector(3 downto 0);
    signal vga_hsync_n_s   : std_logic;
    signal vga_vsync_n_s   : std_logic;
    signal vga_blank_s     : std_logic;
    signal vga_de_s        : std_logic;
    signal dvi_red_s       : std_logic_vector(7 downto 0);
    signal dvi_green_s     : std_logic_vector(7 downto 0);
    signal dvi_blue_s      : std_logic_vector(7 downto 0);

    -- Resets
    signal reset_s         : std_logic;
    signal por_s            : std_logic;

    -- Main RAM: 16K on-chip BRAM (flat, single bank - no mapper/Nextor/
    -- megaram support, matching a minimal MSX1 like the Daewoo DPC-100).
    -- ram_addr_o is 23 bits (memoryctl's general "up to 8MB" bus width) but
    -- only the low 14 bits are used here - see top-level comment.
    signal ram_addr_s      : std_logic_vector(22 downto 0);
    signal ram_data_from_s : std_logic_vector( 7 downto 0);  -- BRAM -> msx (read)
    signal ram_data_to_s   : std_logic_vector( 7 downto 0);  -- msx -> BRAM (write)
    signal ram_ce_s        : std_logic;
    signal ram_we_s        : std_logic;

    -- VRAM (internal BRAM, 16K, same as DECA)
    signal vram_addr_s     : std_logic_vector(13 downto 0);
    signal vram_do_s       : std_logic_vector( 7 downto 0);  -- spram -> msx (read)
    signal vram_di_s       : std_logic_vector( 7 downto 0);  -- msx -> spram (write)
    signal vram_we_s       : std_logic;

    -- Main BIOS+BASIC ROM: served on-demand from SPI flash (flash_rom_xip),
    -- one byte per CPU access, stalling the CPU via bus_wait_n_i.
    signal rom_addr_s      : std_logic_vector(14 downto 0);
    signal rom_ce_s        : std_logic;
    signal rom_wait_n_s    : std_logic;
    signal rom_data_s      : std_logic_vector( 7 downto 0);  -- flash_rom_xip -> msx (read)

    -- Post-boot CPU-activity debug (sticky "did this ever happen" latches) -
    -- is the CPU actually executing/advancing through ROM, writing RAM/VRAM?
    signal dbg_rom_advanced_s     : std_logic := '0';  -- rom_ce_s seen with rom_addr_s /= 0
    signal dbg_vram_written_s     : std_logic := '0';  -- vram_we_s seen
    signal dbg_ram_write_toggle_s : std_logic := '0';  -- flips on every RAM write - liveness indicator
    signal dbg_ram_write_prev_s   : std_logic := '0';
    signal dbg_heartbeat_cnt_s    : unsigned(23 downto 0) := (others => '0');

begin

    reset_s <= not btn_reset_n;
    por_s   <= not pll_locked_s;

    -- =========================================================================
    -- 1. Gowin rPLL Module: 27MHz -> 21.5MHz MSX master clock.
    -- =========================================================================
    u_pll: entity work.gowin_rpll
        port map (
            clkout0 => clock_master_s,
            clkin   => clk_27m,
            lock    => pll_locked_s
        );

    -- =========================================================================
    -- 1b. HDMI PLL: 27MHz -> 126MHz (5x TMDS) -> /5 -> 25.2MHz pixel clock.
    -- Dedicated second (and last available) PLL on this device.
    -- =========================================================================
    u_hdmi_pll: hdmi_rpll
        port map (
            clkout => clk_tmds_s,
            lock   => hdmi_pll_lock_s,
            clkin  => clk_27m
        );

    u_hdmi_clkdiv: Gowin_CLKDIV
        port map (
            clkout => clk_pixel_s,
            hclkin => clk_tmds_s,
            resetn => hdmi_pll_lock_s
        );

    process (clk_pixel_s, hdmi_pll_lock_s)
    begin
        if hdmi_pll_lock_s = '0' then
            hdmi_resetn_sync <= (others => '0');
        elsif rising_edge(clk_pixel_s) then
            hdmi_resetn_sync <= hdmi_resetn_sync(2 downto 0) & '1';
        end if;
    end process;
    hdmi_reset_n_s <= hdmi_resetn_sync(3);

    -- =========================================================================
    -- 2. MSX Clock Divider Logic (Original clocks.vhd)
    -- =========================================================================
    clks: entity work.clocks
        port map (
            clock_i       => clock_master_s,
            por_i         => por_s,
            turbo_on_i    => turbo_on_s,
            clock_vdp_o   => clock_vdp_s,
            clock_5m_en_o => open,
            clock_cpu_o   => clock_cpu_s,
            clock_psg_en_o=> clock_psg_en_s,
            clock_3m_o    => clock_3m_s
        );

    -- =========================================================================
    -- 3. Core MSX Engine
    -- =========================================================================
    the_msx: entity work.msx
        generic map (
            hw_id_g      => 9,
            hw_txt_g     => "TangNano 9K",
            hw_version_g => actual_version,
            video_opt_g  => 3,
            ramsize_g    => 512,
            hw_hashwds_g => '0',
            use_ipl_g    => false  -- no SD card on this board; frees 4 BSRAM blocks (ipl_rom)
        )
        port map (
            clock_i        => clock_master_s,
            clock_vdp_i    => clock_vdp_s,
            clock_cpu_i    => clock_cpu_s,
            clock_psg_en_i => clock_psg_en_s,

            turbo_on_k_i   => '0',
            turbo_on_o     => turbo_on_s,

            reset_i        => reset_s,
            por_i          => por_s,
            softreset_o    => open,

            opt_nextor_i   => '1',
            opt_mr_type_i  => "00",

            -- Main RAM: 16K on-chip BRAM (u_ram below)
            ram_addr_o     => ram_addr_s,
            ram_data_i     => ram_data_from_s,
            ram_data_o     => ram_data_to_s,
            ram_ce_o       => ram_ce_s,
            ram_oe_o       => open,
            ram_we_o       => ram_we_s,
            bus_wait_n_i   => rom_wait_n_s,

            -- Main BIOS+BASIC ROM. Standard slot-0 MSX BIOS decode
            -- (brom_cs_s, addr 0000-7FFF), independent of opt_nextor_i/SD -
            -- boots straight into MSX BASIC with no SD card attached, like
            -- real MSX hardware. Served on-demand from SPI flash (u_rom_xip
            -- below), not pre-copied anywhere.
            rom_addr_o     => rom_addr_s,
            rom_ce_o       => rom_ce_s,
            rom_oe_o       => open,
            rom_data_i     => rom_data_s,

            -- VDP VRAM (internal BRAM)
            vram_addr_o    => vram_addr_s,
            vram_data_i    => vram_do_s,
            vram_data_o    => vram_di_s,
            vram_ce_o      => open,
            vram_oe_o      => open,
            vram_we_o      => vram_we_s,

            -- External bus / peripherals not yet present on this board
            bus_data_i     => X"FF",
            bus_nmi_n_i    => '1',
            bus_int_n_i    => '1',

            -- Keyboard: PS/2 decode chain not wired yet (future work)
            keyb_valid_i   => '0',
            keyb_data_i    => X"00",

            -- K7/cassette: not present on this board
            k7_audio_i     => '0',

            -- Joysticks: not wired yet, idle (matches DECA's convention)
            joy1_up_i      => '1',
            joy1_down_i    => '1',
            joy1_left_i    => '1',
            joy1_right_i   => '1',
            joy1_btn1_i    => '1',
            joy1_btn1_o    => open,
            joy1_btn2_i    => '1',
            joy1_btn2_o    => open,
            joy1_out_o     => open,
            joy2_up_i      => '1',
            joy2_down_i    => '1',
            joy2_left_i    => '1',
            joy2_right_i   => '1',
            joy2_btn1_i    => '1',
            joy2_btn1_o    => open,
            joy2_btn2_i    => '1',
            joy2_btn2_o    => open,
            joy2_out_o     => open,

            -- Video: raw VDP timing/palette-index output -> scan-doubler
            -- (video_opt_g=3: rgb_r_o carries the 4-bit palette index,
            -- rgb_g_o/rgb_b_o are unused, matching the DECA port)
            cnt_hor_o        => cnt_hor_s,
            cnt_ver_o        => cnt_ver_s,
            rgb_r_o          => rgb_col_s,
            rgb_g_o          => open,
            rgb_b_o          => open,
            hsync_n_o        => open,
            vsync_n_o        => open,
            ntsc_pal_o       => open,
            vga_on_k_i       => '0',
            vga_en_o         => open,
            scanline_on_k_i  => '0',
            scanline_en_o    => open,

            -- SD Card SPI (msx entity's generic "spi_*" ports, wired to this
            -- board's sd_*_o/sd_*_i pins)
            spi_cs_n_o     => sd_cs_n_o,
            spi_sclk_o     => sd_sclk_o,
            spi_mosi_o     => sd_mosi_o,
            spi_miso_i     => sd_miso_i,

            D_wait_o       => open,
            D_slots_o      => open,
            D_ipl_en_o     => open

            -- PS/2: the msx entity has no raw PS2_CLK/DATA ports - it wants
            -- an already-decoded row/col matrix + keyb_valid_i/keyb_data_i
            -- scancode stream (see DECA's separate "keyb: entity work.keyboard"
            -- instance). Not wired yet; ps2_clk_io/ps2_data_io on this entity
            -- are unused for now (future work, out of scope for this pass).
        );

    -- =========================================================================
    -- 4. VRAM (16K, internal BRAM - same approach as the DECA port)
    -- =========================================================================
    vram: entity work.spram
        generic map (
            addr_width_g => 14,
            data_width_g => 8
        )
        port map (
            clk_i  => clock_master_s,
            we_i   => vram_we_s,
            addr_i => vram_addr_s,
            data_i => vram_di_s,
            data_o => vram_do_s
        );

    -- =========================================================================
    -- 5. Main RAM (16K, internal BRAM - flat, single bank). Only the low 14
    -- bits of ram_addr_o are used; any higher-address accesses (megaram/
    -- Nextor/RAM-mapper bank switching, IPL bootstrap RAM windows - none of
    -- which a plain MSX1 BIOS/BASIC boot actually exercises without an SD
    -- card present) alias into the same 16K, which is harmless for this
    -- minimal bring-up.
    -- =========================================================================
    u_ram: entity work.spram
        generic map (
            addr_width_g => 14,
            data_width_g => 8
        )
        port map (
            clk_i  => clock_master_s,
            we_i   => ram_we_s,
            addr_i => ram_addr_s(13 downto 0),
            data_i => ram_data_to_s,
            data_o => ram_data_from_s
        );

    -- =========================================================================
    -- 6. Main BIOS+BASIC ROM: on-demand SPI flash reads (see flash_rom_xip.vhd
    -- for why - the embedded-PSRAM approach this replaced is documented in
    -- this file's header comment and in gowin_rpll.vhd's history).
    -- =========================================================================
    u_rom_xip: entity work.flash_rom_xip
        generic map (
            FLASH_BASE_G => 16#3A0000#,
            CLK_DIV      => 2
        )
        port map (
            CLK         => clock_master_s,
            RESET       => por_s,
            ROM_ADDR    => rom_addr_s,
            ROM_CE      => rom_ce_s,
            ROM_DATA    => rom_data_s,
            WAIT_N      => rom_wait_n_s,
            FLASH_DATA0 => flash_di,
            FLASH_NCSO  => flash_ncs,
            FLASH_DCLK  => flash_clk,
            FLASH_ASDO  => flash_do
        );

    -- =========================================================================
    -- 7. Scan doubler: VDP's native low-res timing (clock_master_s domain,
    -- cnt_hor_s/cnt_ver_s/rgb_col_s) -> fixed 640x480@60 VGA timing
    -- (clk_pixel_s domain). Line-buffered variant of the DECA port's
    -- vga.vhd, sized for this device's smaller BSRAM budget - see
    -- vga_linebuf.vhd.
    -- =========================================================================
    vga: entity work.vga_linebuf
        port map (
            I_CLK      => clock_master_s,
            I_CLK_VGA  => clk_pixel_s,
            I_COLOR    => rgb_col_s,
            I_HCNT     => cnt_hor_s,
            I_VCNT     => cnt_ver_s,
            O_HSYNC    => vga_hsync_n_s,
            O_VSYNC    => vga_vsync_n_s,
            O_COLOR    => vga_col_s,
            O_BLANK    => vga_blank_s
        );

    -- =========================================================================
    -- 8. Palette index -> RGB444. Same 16-entry TMS9918 palette table as the
    -- DECA port (msx_deca.vhd), packed "RB0G" (bits 15:12=R, 11:8=B, 3:0=G).
    -- Scanline dimming is skipped here (DECA's scanlines_en_s feature).
    -- =========================================================================
    process (clk_pixel_s)
        variable vga_col_v : integer range 0 to 15;
        variable vga_rgb_v : std_logic_vector(15 downto 0);
        type ram_t is array (natural range 0 to 15) of std_logic_vector(15 downto 0);
        constant rgb_c : ram_t := (
                --      RB0G
                0  => X"0000",
                1  => X"0000",
                2  => X"240C",
                3  => X"570D",
                4  => X"5E05",
                5  => X"7F07",
                6  => X"D405",
                7  => X"4F0E",
                8  => X"F505",
                9  => X"F707",
                10 => X"D50C",
                11 => X"E80C",
                12 => X"230B",
                13 => X"CB09",
                14 => X"CC0C",
                15 => X"FF0F"
        );
    begin
        if rising_edge(clk_pixel_s) then
            vga_col_v := to_integer(unsigned(vga_col_s));
            vga_rgb_v := rgb_c(vga_col_v);
            vga_r_s <= vga_rgb_v(15 downto 12);
            vga_b_s <= vga_rgb_v(11 downto  8);
            vga_g_s <= vga_rgb_v( 3 downto  0);
        end if;
    end process;

    -- =========================================================================
    -- 9. DVI / TMDS Transmitter (Gowin OSER10 serializer + ELVDS_OBUF pairs)
    -- =========================================================================
    dvi_red_s   <= vga_r_s & vga_r_s;
    dvi_green_s <= vga_g_s & vga_g_s;
    dvi_blue_s  <= vga_b_s & vga_b_s;
    vga_de_s    <= not vga_blank_s;

    u_dvi_tx: dvi_tx
        port map (
            clk_pixel  => clk_pixel_s,
            clk_tmds   => clk_tmds_s,
            reset_n    => hdmi_reset_n_s,
            red        => dvi_red_s,
            green      => dvi_green_s,
            blue       => dvi_blue_s,
            hsync      => vga_hsync_n_s,
            vsync      => vga_vsync_n_s,
            de         => vga_de_s,
            tmds_clk_p => tmds_clk_p,
            tmds_clk_n => tmds_clk_n,
            tmds_d_p   => tmds_d_p,
            tmds_d_n   => tmds_d_n
        );

    -- Post-boot CPU-activity debug: is the CPU actually running through the
    -- ROM (advancing past address 0), and is it writing RAM/VRAM? Sticky
    -- latches so a single occurrence is visible even if it only happens
    -- once.
    process (clock_master_s)
    begin
        if rising_edge(clock_master_s) then
            dbg_heartbeat_cnt_s <= dbg_heartbeat_cnt_s + 1;
            if por_s = '1' then
                dbg_rom_advanced_s <= '0';
                dbg_vram_written_s <= '0';
            else
                if rom_ce_s = '1' and unsigned(rom_addr_s) /= 0 then
                    dbg_rom_advanced_s <= '1';
                end if;
                if ram_ce_s = '1' and ram_we_s = '1' then
                    if dbg_ram_write_prev_s = '0' then
                        dbg_ram_write_toggle_s <= not dbg_ram_write_toggle_s;
                    end if;
                    dbg_ram_write_prev_s <= '1';
                else
                    dbg_ram_write_prev_s <= '0';
                end if;
                if vram_we_s = '1' then
                    dbg_vram_written_s <= '1';
                end if;
            end if;
        end if;
    end process;

    -- VIDEO DEBUG (temporary): all 6 LEDs show vga_linebuf's sticky maximum
    -- observed (write_lines - read_lines) gap, MSB on LED5 down to LSB on
    -- LED0, active-low so a LIT led = '1' bit. Boot-status LEDs (PLL lock
    -- etc.) are disabled for this debug pass - already confirmed working,
    -- and this needs all 6 LEDs to read out a 0-63 value directly.
    leds_n_o(0) <= not pll_locked_s;
    leds_n_o(1) <= not btn_reset_n;
    leds_n_o(2) <= not dbg_ram_write_toggle_s;
    leds_n_o(3) <= not dbg_vram_written_s;
    leds_n_o(4) <= not dbg_rom_advanced_s;
    leds_n_o(5) <= not dbg_heartbeat_cnt_s(23);

end architecture behavior;
