-------------------------------------------------------------------------------
-- uart_tx
--
-- Minimal 8N1 UART transmitter, byte-at-a-time, for streaming PSRAM BIST
-- results out over the Tang Nano 9K's onboard USB-UART bridge (same USB-C
-- cable used for programming - shows up as a serial port on the host PC).
-------------------------------------------------------------------------------

library ieee;
	use ieee.std_logic_1164.all;
	use ieee.numeric_std.all;

entity uart_tx is
	generic (
		CLK_FREQ_G : integer := 27_000_000;
		BAUD_G     : integer := 115200
	);
	port (
		CLK     : in  std_logic;
		RESET   : in  std_logic;  -- sync, active high
		DATA    : in  std_logic_vector(7 downto 0);
		START   : in  std_logic;  -- pulse to send DATA
		BUSY    : out std_logic;
		TX      : out std_logic
	);
end entity;

architecture rtl of uart_tx is

	constant DIV_C : integer := CLK_FREQ_G / BAUD_G;

	type state_t is (ST_IDLE, ST_START, ST_DATA, ST_STOP);
	signal state    : state_t := ST_IDLE;
	signal bit_cnt  : integer range 0 to 7 := 0;
	signal baud_cnt : integer range 0 to DIV_C - 1 := 0;
	signal shift_r  : std_logic_vector(7 downto 0);
	signal tx_r     : std_logic := '1';

begin

	TX   <= tx_r;
	BUSY <= '0' when state = ST_IDLE else '1';

	process (CLK)
	begin
		if rising_edge(CLK) then
			if RESET = '1' then
				state    <= ST_IDLE;
				tx_r     <= '1';
				baud_cnt <= 0;
				bit_cnt  <= 0;
			else
				case state is
					when ST_IDLE =>
						tx_r <= '1';
						if START = '1' then
							shift_r  <= DATA;
							baud_cnt <= 0;
							state    <= ST_START;
						end if;

					when ST_START =>
						tx_r <= '0';
						if baud_cnt = DIV_C - 1 then
							baud_cnt <= 0;
							bit_cnt  <= 0;
							state    <= ST_DATA;
						else
							baud_cnt <= baud_cnt + 1;
						end if;

					when ST_DATA =>
						tx_r <= shift_r(0);
						if baud_cnt = DIV_C - 1 then
							baud_cnt <= 0;
							shift_r  <= '0' & shift_r(7 downto 1);
							if bit_cnt = 7 then
								state <= ST_STOP;
							else
								bit_cnt <= bit_cnt + 1;
							end if;
						else
							baud_cnt <= baud_cnt + 1;
						end if;

					when ST_STOP =>
						tx_r <= '1';
						if baud_cnt = DIV_C - 1 then
							baud_cnt <= 0;
							state    <= ST_IDLE;
						else
							baud_cnt <= baud_cnt + 1;
						end if;
				end case;
			end if;
		end if;
	end process;

end architecture;
