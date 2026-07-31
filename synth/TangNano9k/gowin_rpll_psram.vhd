library ieee;
use ieee.std_logic_1164.all;

-- Dedicated PLL for the PSRAM re-qualification effort (psram_bist_top.vhd
-- only) - kept entirely separate from gowin_rpll.vhd (the main design's
-- master-clock-only PLL, currently serving the working flash-XIP/BRAM boot
-- path) so this investigation can't destabilize it.
--
-- Per the user's qualification protocol: start well below the previous
-- 81MHz attempt. CLKOUT = 27*(FBDIV_SEL+1)/(IDIV_SEL+1) = 27*2/1 = 54 MHz
-- exactly (IDIV_SEL=0, FBDIV_SEL=1, PFD=27MHz - same high-PFD reasoning as
-- before). PSDA_SEL sweeps CLKOUTP's phase in ~22.5deg steps (0-15) for the
-- coarse phase sweep.
entity gowin_rpll_psram is
    generic (
        PSDA_SEL_G : string := "0000"  -- phase step 0-15, edited per sweep point
    );
    port (
        psram_clk_o   : out std_logic; -- 54 MHz (embedded PSRAM)
        psram_clk_p_o : out std_logic; -- 54 MHz, phase-shifted (PSRAM O_psram_ck)
        lock          : out std_logic;
        clkin         : in  std_logic  -- 27.000 MHz (Onboard)
    );
end entity;

architecture behavioral of gowin_rpll_psram is

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

begin

    rpll_inst : rPLL
        generic map (
            FCLKIN          => "27",
            DEVICE          => "GW1NR-9C",
            DYN_IDIV_SEL    => "false",
            IDIV_SEL        => 0,      -- /1, PFD = 27 MHz
            DYN_FBDIV_SEL   => "false",
            FBDIV_SEL       => 1,      -- x2 -> CLKOUT = 27*2/1 = 54 MHz
            DYN_ODIV_SEL    => "false",
            ODIV_SEL        => 8,      -- VCO = 54*8 = 432 MHz
            PSDA_SEL        => PSDA_SEL_G,
            DYN_DA_EN       => "false",
            DUTYDA_SEL      => "1000",
            DYN_SDIV_SEL    => 4,
            CLKOUTD_SRC     => "CLKOUT"
        )
        port map (
            CLKOUT  => psram_clk_o,
            LOCK    => lock,
            CLKOUTP => psram_clk_p_o,
            CLKOUTD => open,
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
