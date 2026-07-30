-------------------------------------------------------------------------------
-- flash_reader
--
-- Minimal read-only driver for the Tang Nano 9K's external SPI config flash
-- (standard SPI NOR "READ" command 0x03 + 24-bit address). Used to load the
-- MSX BIOS/BASIC ROM image from spare flash space (past the end of the
-- FPGA bitstream) into the embedded PSRAM at boot, so the system can run
-- standalone with no SD card - see rom_loader.vhd.
--
-- State-machine structure (spi_master handshake pattern, one SPI byte
-- transaction per "count" step) follows the proven read path in
-- https://github.com/andykarpov/tang9k-speccy/blob/master/src/flash/flash.vhd
-- (same board family); this is a from-scratch, read-only rewrite of just
-- that one command sequence, not a copy of that file.
-------------------------------------------------------------------------------

library ieee;
	use ieee.std_logic_1164.all;
	use ieee.numeric_std.all;

entity flash_reader is
	generic (
		CLK_DIV : integer := 2  -- SPI SCLK = CLK / (2*CLK_DIV)
	);
	port (
		CLK    : in  std_logic;
		RESET  : in  std_logic;  -- sync, active high

		ADDR   : in  std_logic_vector(23 downto 0);
		START  : in  std_logic;  -- pulse to begin a read of ADDR
		DATA   : out std_logic_vector(7 downto 0);
		VALID  : out std_logic;  -- pulses one cycle when DATA is valid
		BUSY   : out std_logic;

		-- Physical SPI flash pins (Tang Nano 9K: pins 59/60/61/62)
		DATA0  : in  std_logic;  -- flash MISO
		NCSO   : out std_logic;  -- flash CS_n
		DCLK   : out std_logic;  -- flash SCLK
		ASDO   : out std_logic   -- flash MOSI
	);
end entity;

architecture rtl of flash_reader is

	constant SPI_CMD_READ : std_logic_vector(7 downto 0) := X"03";

	type state_t is (ST_IDLE, ST_READ);

	signal spi_di_bus, spi_do_bus : std_logic_vector(7 downto 0);
	signal spi_busy               : std_logic;
	signal spi_ena, spi_cont      : std_logic;
	signal spi_so                 : std_logic;
	signal spi_ss_n               : std_logic_vector(0 downto 0);
	signal spi_sclk_s             : std_logic;
	signal reset_n_s              : std_logic;

	signal addr_r : std_logic_vector(23 downto 0);

begin

	reset_n_s <= not RESET;

	U1: entity work.spi_master
		generic map (
			slaves  => 1,
			d_width => 8
		)
		port map (
			clock   => CLK,
			reset_n => reset_n_s,
			enable  => spi_ena,
			cpol    => '0',
			cpha    => '0',
			cont    => spi_cont,
			clk_div => CLK_DIV,
			addr    => 0,
			tx_data => spi_di_bus,
			miso    => spi_so,
			sclk    => spi_sclk_s,
			ss_n    => spi_ss_n,
			mosi    => ASDO,
			busy    => spi_busy,
			rx_data => spi_do_bus
		);

	NCSO   <= spi_ss_n(0);
	DCLK   <= spi_sclk_s;
	spi_so <= DATA0;

	process (CLK)
		variable state         : state_t := ST_IDLE;
		variable count         : integer range 0 to 7 := 0;
		variable spi_busy_prev : std_logic := '0';
	begin
		if rising_edge(CLK) then
			if RESET = '1' then
				state         := ST_IDLE;
				spi_ena       <= '0';
				spi_cont      <= '0';
				count         := 0;
				spi_busy_prev := '0';
				VALID         <= '0';
				BUSY          <= '0';
			else
				VALID <= '0';

				case state is
				when ST_IDLE =>
					BUSY <= '0';
					if START = '1' then
						addr_r <= ADDR;
						count  := 0;
						BUSY   <= '1';
						state  := ST_READ;
					end if;

				when ST_READ =>
					-- Falling-edge detector on spi_busy: compare against
					-- LAST cycle's value first, then update the variable -
					-- doing it the other way around makes the comparison
					-- tautological (spi_busy_prev would already equal
					-- spi_busy) and count would never advance past 0.
					if spi_busy_prev = '1' and spi_busy = '0' then
						count := count + 1;
					end if;
					spi_busy_prev := spi_busy;
					case count is
					when 0 =>
						if spi_busy = '0' then
							spi_cont   <= '1';
							spi_ena    <= '1';
							spi_di_bus <= SPI_CMD_READ;
						else
							spi_di_bus <= addr_r(23 downto 16);
						end if;
					when 1 =>
						spi_di_bus <= addr_r(15 downto 8);
					when 2 =>
						spi_di_bus <= addr_r(7 downto 0);
					when 3 =>
						spi_di_bus <= (others => '0');
					when 4 =>
						spi_cont <= '0';
						spi_ena  <= '0';
					when 5 =>
						count := 0;
						DATA  <= spi_do_bus;
						VALID <= '1';
						state := ST_IDLE;
					when others =>
						null;
					end case;
				end case;
			end if;
		end if;
	end process;

end architecture;
