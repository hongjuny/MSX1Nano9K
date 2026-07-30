library ieee;
use ieee.std_logic_1164.all;

-- PLL for the MSX master clock.
--
-- This file used to also generate a shared PSRAM clock/phase output for the
-- embedded PSRAM. That path was abandoned after extensive real-hardware
-- debugging found near-zero read timing margin that no PLL frequency/phase
-- combination, nor an nextpnr --seed sweep (20 seeds total, all hung), could
-- reliably fix - see top_tangnano9k.vhd's header comment. The design now
-- uses on-chip BRAM for RAM and on-demand SPI flash reads (flash_rom_xip.vhd)
-- for ROM instead, so this PLL only needs to produce one accurate clock:
--
--   CLKOUTD -> clkout0 (MSX master clock, target 21.4773 MHz = NTSC
--              colorburst 3.579545MHz x 6)
--
-- CLKOUT = FCLKIN*(FBDIV_SEL+1)/(IDIV_SEL+1) = 27*51/4 = 344.25 MHz (internal
-- tap only, not output anywhere). PFD=27/4=6.75MHz, VCO=344.25*2=688.5MHz,
-- both in-spec.
-- CLKOUTD = CLKOUT/DYN_SDIV_SEL = 344.25/16 = 21.515625 MHz - 0.179% off the
-- real 21.4773MHz target, chosen NOT for closest absolute accuracy but to
-- jointly minimize drift against the HDMI pixel clock (hdmi_rpll.v, kept at
-- 25.2MHz) in vga_linebuf.vhd's video FIFO: the relevant ratio is
-- master_freq/pixel_freq against (VDP active-window cycles)/(VGA
-- active-window cycles) = 295488/345600 = 0.855 exactly. 21.5/25.2 (the
-- previous, closest-accuracy choice) was 0.2135% off that ratio;
-- 21.515625/25.2 is 0.141% off - the best achievable pairing found by
-- exhaustive PLL-parameter search while keeping the pixel clock at exactly
-- 25.2MHz (best HDMI/monitor compatibility - the OSER10 serializer needs
-- clk_tmds = 5x clk_pixel exactly via a fixed Gowin CLKDIV(/5), constraining
-- pixel clock options to CLKOUT/5 taps only, only ~35 of which land near
-- standard VESA pixel-clock territory).
entity gowin_rpll is
    port (
        clkout0 : out std_logic; -- 21.515625 MHz (MSX Master Clock)
        lock    : out std_logic;
        clkin   : in  std_logic  -- 27.000 MHz (Onboard)
    );
end entity;

architecture behavioral of gowin_rpll is

    -- Gowin rPLL primitive declaration (full port/parameter list, matching
    -- yosys/gowin/cells_sim.v exactly - no implicit defaults relied upon).
    component rPLL
        generic (
            FCLKIN          : string  := "27";
            DEVICE          : string  := "GW1NR-9C";
            DYN_IDIV_SEL    : string  := "false";
            IDIV_SEL        : integer := 0;
            DYN_FBDIV_SEL   : string  := "false";
            FBDIV_SEL       : integer := 0;
            DYN_ODIV_SEL    : string  := "false";
            ODIV_SEL        : integer := 8;
            PSDA_SEL        : string  := "0000";
            DYN_DA_EN       : string  := "false";
            DUTYDA_SEL      : string  := "1000";
            CLKOUT_FT_DIR   : integer := 1;
            CLKOUTP_FT_DIR  : integer := 1;
            CLKOUT_DLY_STEP : integer := 0;
            CLKOUTP_DLY_STEP: integer := 0;
            CLKFB_SEL       : string  := "internal";
            CLKOUT_BYPASS   : string  := "false";
            CLKOUTP_BYPASS  : string  := "false";
            CLKOUTD_BYPASS  : string  := "false";
            DYN_SDIV_SEL    : integer := 2;
            CLKOUTD_SRC     : string  := "CLKOUT";
            CLKOUTD3_SRC    : string  := "CLKOUT"
        );
        port (
            CLKOUT  : out std_logic;
            LOCK    : out std_logic;
            CLKOUTP : out std_logic;
            CLKOUTD : out std_logic;
            CLKOUTD3: out std_logic;
            RESET   : in  std_logic;
            RESET_P : in  std_logic;
            CLKIN   : in  std_logic;
            CLKFB   : in  std_logic;
            FBDSEL  : in  std_logic_vector(5 downto 0);
            IDSEL   : in  std_logic_vector(5 downto 0);
            ODSEL   : in  std_logic_vector(5 downto 0);
            PSDA    : in  std_logic_vector(3 downto 0);
            DUTYDA  : in  std_logic_vector(3 downto 0);
            FDLY    : in  std_logic_vector(3 downto 0)
        );
    end component;

    signal clkout_unused_s : std_logic;

begin

    rpll_inst : rPLL
        generic map (
            FCLKIN          => "27",
            DEVICE          => "GW1NR-9C",
            DYN_IDIV_SEL    => "false",
            IDIV_SEL        => 3,      -- /4, PFD = 27/4 = 6.75 MHz
            DYN_FBDIV_SEL   => "false",
            FBDIV_SEL       => 50,     -- x51 -> CLKOUT = 27*51/4 = 344.25 MHz
            DYN_ODIV_SEL    => "false",
            ODIV_SEL        => 2,      -- VCO = 344.25*2 = 688.5 MHz
            PSDA_SEL        => "0000",
            DYN_DA_EN       => "false",
            DUTYDA_SEL      => "1000",
            DYN_SDIV_SEL    => 16,     -- CLKOUTD = CLKOUT/16 = 21.515625 MHz
            CLKOUTD_SRC     => "CLKOUT"
        )
        port map (
            CLKOUT  => clkout_unused_s,
            LOCK    => lock,
            CLKOUTP => open,
            CLKOUTD => clkout0,
            CLKOUTD3=> open,
            RESET   => '0',
            RESET_P => '0',
            CLKIN   => clkin,
            CLKFB   => '0',
            FBDSEL  => "000000",
            IDSEL   => "000000",
            ODSEL   => "000000",
            PSDA    => "0000",
            DUTYDA  => "0000",
            FDLY    => "0000"
        );

end architecture;
