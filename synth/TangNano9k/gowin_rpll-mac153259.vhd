library ieee;
use ieee.std_logic_1164.all;

entity gowin_rpll is
    port (
        clkout0 : out std_logic; -- 21.477 MHz (MSX Master Clock)
        clkout1 : out std_logic; -- 107.385 MHz (5x TMDS Clock)
        lock    : out std_logic;
        clkin   : in  std_logic  -- 27.000 MHz (Onboard)
    );
end entity;

architecture behavioral of gowin_rpll is

    -- Gowin rPLL primitive declaration
    component rPLL
        generic (
            FCLKIN        : string := "27.0";
            DEVICE        : string := "GW1NR-9C";
            DYN_IDIV_SEL  : string := "false";
            IDIV_SEL      : integer := 8;     -- 27 / (8+1) = 3 MHz PFD
            DYN_FBDIV_SEL : string := "false";
            FBDIV_SEL     : integer := 35;    -- 3 * (35+1) = 108 MHz VCO
            DYN_ODIV_SEL  : string := "false";
            ODIV_SEL      : integer := 8;
            PSDA_SEL      : string := "0000";
            DYN_DA_EN     : string := "true";
            DUTYDA_SEL    : string := "1000";
            CLKOUT_FT_DIR : integer := 1;
            CLKOUTP_FT_DIR: integer := 1
        );
        port (
            CLKOUT   : out std_logic;
            LOCK     : out std_logic;
            CLKOUTP  : out std_logic;
            CLKOUTD  : out std_logic;
            CLKOUTD3 : out std_logic;
            RESET    : in  std_logic;
            RESET_P  : in  std_logic;
            CLKIN    : in  std_logic;
            CLKFB    : in  std_logic;
            FBDSEL   : in  std_logic_vector(5 downto 0);
            IDSEL    : in  std_logic_vector(5 downto 0);
            ODSEL    : in  std_logic_vector(5 downto 0);
            PSDA     : in  std_logic_vector(3 downto 0);
            DUTYDA   : in  std_logic_vector(3 downto 0);
            FDLY     : in  std_logic_vector(3 downto 0)
        );
    end component;

begin

    rpll_inst : rPLL
        port map (
            CLKOUT   => clkout1,     -- ~108 MHz
            LOCK     => lock,
            CLKOUTP  => open,
            CLKOUTD  => clkout0,     -- ~21.477 MHz
            CLKOUTD3 => open,
            RESET    => '0',
            RESET_P  => '0',
            CLKIN    => clkin,
            CLKFB    => '0',
            FBDSEL   => "000000",
            IDSEL    => "000000",
            ODSEL    => "000000",
            PSDA     => "0000",
            DUTYDA   => "0000",
            FDLY     => "0000"
        );

end architecture;
