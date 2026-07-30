library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.msx_pack.all;

entity top_tangnano9k is
    port (
        -- 1. Clock & Reset
        clk_27m         : in  std_logic;
        btn_reset_n     : in  std_logic;

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
        leds_n_o        : out std_logic_vector(5 downto 0)
    );
end entity top_tangnano9k;

architecture behavior of top_tangnano9k is

    -- Verilog DVI TX Component Declaration
    component dvi_tx
        port (
            clk_pixel   : in  std_logic;
            clk_tmds    : in  std_logic;
            reset_n     : in  std_logic;
            red         : in  std_logic_vector(3 downto 0);
            green       : in  std_logic_vector(3 downto 0);
            blue        : in  std_logic_vector(3 downto 0);
            hsync       : in  std_logic;
            vsync       : in  std_logic;
            de          : in  std_logic;
            tmds_clk_p  : out std_logic;
            tmds_clk_n  : out std_logic;
            tmds_d_p    : out std_logic_vector(2 downto 0);
            tmds_d_n    : out std_logic_vector(2 downto 0)
        );
    end component;

    -- Clocks & Resets
    signal clock_master_s  : std_logic;
    signal clock_hdmi_s    : std_logic;
    signal pll_locked_s    : std_logic;

    signal clock_vdp_s     : std_logic;
    signal clock_cpu_s     : std_logic;
    signal clock_psg_en_s  : std_logic;
    signal clock_3m_s      : std_logic;
    signal turbo_on_s      : std_logic;

    -- Video signals
    signal rgb_r_s         : std_logic_vector(3 downto 0);
    signal rgb_g_s         : std_logic_vector(3 downto 0);
    signal rgb_b_s         : std_logic_vector(3 downto 0);
    signal hsync_n_s       : std_logic;
    signal vsync_n_s       : std_logic;

    -- Keyboard Decoding Signals
    signal keyb_valid_s    : std_logic;
    signal keyb_data_s     : std_logic_vector(7 downto 0);
    signal caps_en_s       : std_logic;
    signal rows_s          : std_logic_vector(3 downto 0);

    signal reset_s         : std_logic;
    signal por_s           : std_logic;

begin

    reset_s <= not btn_reset_n;
    por_s   <= not pll_locked_s;

    -- 1. Gowin rPLL
    u_pll: entity work.gowin_rpll
        port map (
            clkout0 => clock_master_s,
            clkout1 => clock_hdmi_s,
            clkin   => clk_27m,
            lock    => pll_locked_s
        );

    -- 2. MSX Clock Divider
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

    -- 3. PS/2 Keyboard Decoder
    u_keyboard: entity work.keyboard
        port map (
            clock_i       => clock_master_s,
            reset_i       => reset_s,
            ps2_clk_io    => ps2_clk_io,
            ps2_data_io   => ps2_data_io,
            rows_coded_i  => rows_s,
            keymap_addr_i => (others => '0'),
            keymap_data_i => (others => '0'),
            keymap_we_i   => '0',
            led_caps_i    => caps_en_s,
            keyb_valid_o  => keyb_valid_s,
            keyb_data_o   => keyb_data_s
        );

    -- 4. Core MSX Engine
    the_msx: entity work.msx
        generic map (
            hw_id_g      => 9,
            hw_txt_g     => "TangNano 9K",
            hw_version_g => X"01",
            video_opt_g  => 3,
            ramsize_g    => 8192,
            hw_hashwds_g => '0'
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
            reload_o       => open,
            
            opt_nextor_i   => '1',
            opt_mr_type_i  => "00",
            opt_vga_on_i   => '1',

            -- Internal Memory Connections
            ram_addr_o     => open,
            ram_data_i     => (others => '0'),
            ram_data_o     => open,
            ram_ce_o       => open,
            ram_oe_o       => open,
            ram_we_o       => open,

            rom_addr_o     => open,
            rom_data_i     => (others => '0'),
            rom_ce_o       => open,
            rom_oe_o       => open,

            bus_addr_o     => open,
            bus_data_i     => (others => '0'),
            bus_data_o     => open,
            bus_rd_n_o     => open,
            bus_wr_n_o     => open,
            bus_m1_n_o     => open,
            bus_iorq_n_o   => open,
            bus_mreq_n_o   => open,
            bus_sltsl1_n_o => open,
            bus_sltsl2_n_o => open,
            bus_wait_n_i   => '1',
            bus_nmi_n_i    => '1',
            bus_int_n_i    => '1',

            vram_addr_o    => open,
            vram_data_i    => (others => '0'),
            vram_data_o    => open,
            vram_ce_o      => open,
            vram_oe_o      => open,
            vram_we_o      => open,

            -- Keyboard Decoder Connection
            rows_o         => rows_s,
            cols_i         => (others => '1'),
            caps_en_o      => caps_en_s,
            keyb_valid_i   => keyb_valid_s,
            keyb_data_i    => keyb_data_s,
            keymap_addr_o  => open,
            keymap_data_o  => open,
            keymap_we_o    => open,

            -- Audio Outputs
            audio_scc_o    => open,
            audio_psg_o    => open,
            beep_o         => open,
            volumes_o      => open,

            -- Tape (K7)
            k7_motor_o     => open,
            k7_audio_o     => open,
            k7_audio_i     => '0',

            -- Joysticks
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

            -- Video Output
            cnt_hor_o      => open,
            cnt_ver_o      => open,
            rgb_r_o        => rgb_r_s,
            rgb_g_o        => rgb_g_s,
            rgb_b_o        => rgb_b_s,
            hsync_n_o      => hsync_n_s,
            vsync_n_o      => vsync_n_s,
            ntsc_pal_o     => open,
            vga_on_k_i     => '1',
            vga_en_o       => open,
            scanline_on_k_i=> '0',
            scanline_en_o  => open,
            vertfreq_on_k_i=> '0',

            -- SPI / SD Card
            flspi_cs_n_o   => open,
            spi2_cs_n_o    => open,
            spi_cs_n_o     => sd_cs_n_o,
            spi_sclk_o     => sd_sclk_o,
            spi_mosi_o     => sd_mosi_o,
            spi_miso_i     => sd_miso_i,
            sd_pres_n_i    => '0',
            sd_wp_i        => '0',

            -- Debug
            D_wait_o       => open,
            D_slots_o      => open,
            D_ipl_en_o     => open
        );

    -- 5. DVI / TMDS Transmitter Connection
    u_dvi_tx: dvi_tx
        port map (
            clk_pixel  => clock_vdp_s,
            clk_tmds   => clock_hdmi_s,
            reset_n    => pll_locked_s,
            red        => rgb_r_s,
            green      => rgb_g_s,
            blue       => rgb_b_s,
            hsync      => hsync_n_s,
            vsync      => vsync_n_s,
            de         => '1',
            tmds_clk_p => tmds_clk_p,
            tmds_clk_n => tmds_clk_n,
            tmds_d_p   => tmds_d_p,
            tmds_d_n   => tmds_d_n
        );

    -- LED Status Output
    leds_n_o <= not (turbo_on_s & pll_locked_s & "0000");

end architecture behavior;
