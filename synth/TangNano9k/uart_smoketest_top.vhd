-------------------------------------------------------------------------------
-- uart_smoketest_top
--
-- Minimal, PSRAM-free sanity check: continuously streams "HELLO\r\n" over
-- the same UART pin/baud as psram_bist_top, to isolate whether the UART
-- link itself (pin 17, FTDI channel, host-side serial reading) is alive,
-- independent of anything PSRAM-related.
-------------------------------------------------------------------------------

library ieee;
	use ieee.std_logic_1164.all;
	use ieee.numeric_std.all;

entity uart_smoketest_top is
	port (
		clk_27m     : in  std_logic;
		btn_reset_n : in  std_logic;
		uart_tx_o   : out std_logic;
		leds_n_o    : out std_logic_vector(5 downto 0)
	);
end entity;

architecture rtl of uart_smoketest_top is
	signal reset_s      : std_logic;
	signal uart_data_s  : std_logic_vector(7 downto 0);
	signal uart_start_s : std_logic := '0';
	signal uart_busy_s  : std_logic;
	signal msg_idx      : integer range 0 to 6 := 0;
	signal div_cnt       : unsigned(23 downto 0) := (others => '0');
	signal hb            : std_logic := '0';

	type msg_t is array (0 to 6) of std_logic_vector(7 downto 0);
	constant MSG_C : msg_t := (
		std_logic_vector(to_unsigned(character'pos('H'), 8)),
		std_logic_vector(to_unsigned(character'pos('E'), 8)),
		std_logic_vector(to_unsigned(character'pos('L'), 8)),
		std_logic_vector(to_unsigned(character'pos('L'), 8)),
		std_logic_vector(to_unsigned(character'pos('O'), 8)),
		std_logic_vector(to_unsigned(13, 8)),
		std_logic_vector(to_unsigned(10, 8))
	);
begin

	reset_s <= not btn_reset_n;

	u_uart: entity work.uart_tx
		generic map (
			CLK_FREQ_G => 27_000_000,
			BAUD_G     => 115200
		)
		port map (
			CLK   => clk_27m,
			RESET => reset_s,
			DATA  => uart_data_s,
			START => uart_start_s,
			BUSY  => uart_busy_s,
			TX    => uart_tx_o
		);

	process (clk_27m)
	begin
		if rising_edge(clk_27m) then
			if reset_s = '1' then
				msg_idx      <= 0;
				uart_start_s <= '0';
				div_cnt      <= (others => '0');
				hb           <= '0';
			else
				uart_start_s <= '0';
				div_cnt <= div_cnt + 1;
				if div_cnt = to_unsigned(2_700_000, 24) then  -- ~100ms
					div_cnt <= (others => '0');
					hb <= not hb;
				end if;
				if uart_busy_s = '0' and uart_start_s = '0' then
					uart_data_s <= MSG_C(msg_idx);
					uart_start_s <= '1';
					if msg_idx = 6 then
						msg_idx <= 0;
					else
						msg_idx <= msg_idx + 1;
					end if;
				end if;
			end if;
		end if;
	end process;

	leds_n_o(0) <= '0';        -- always on: confirms bitstream loaded/running
	leds_n_o(1) <= not btn_reset_n;
	leds_n_o(2) <= hb;         -- ~5Hz blink: confirms clk_27m alive
	leds_n_o(3) <= '1';
	leds_n_o(4) <= '1';
	leds_n_o(5) <= '1';

end architecture;
