-------------------------------------------------------------------------------
-- flash_rom_xip
--
-- On-demand ("execute in place") BIOS/BASIC ROM reader: instead of bulk-
-- copying the 32K ROM image into a separate RAM at boot (the earlier
-- flash->PSRAM rom_loader approach), this serves each CPU ROM access
-- directly from SPI flash, one byte at a time, stalling the Z80 via
-- bus_wait_n_i for the duration of each SPI transaction (same mechanism
-- previously used for PSRAM wait-states).
--
-- This exists because the embedded PSRAM link turned out to have near-zero
-- read timing margin on real hardware (see gowin_rpll.vhd's history comments
-- and the psram_bist_top.vhd diagnostic tool) - repeated nextpnr --seed
-- sweeps (20 seeds total) never found a working placement. flash_reader.vhd's
-- plain SPI read path, by contrast, was verified byte-perfect against a
-- 32K checksum on real hardware. Using it here trades ROM access speed
-- (~tens of clk_i cycles per byte, no caching) for correctness, to get a
-- working boot path first - RAM (the hot path for BASIC) is on-chip BRAM
-- (see top_tangnano9k.vhd's u_ram) and needs no wait-states at all.
--
-- A single-entry cache (last address + data) avoids re-issuing a fresh SPI
-- transaction when the CPU re-reads the same byte without the address
-- changing in between (common - e.g. an M1 opcode fetch cycle can sample
-- the bus more than once).
-------------------------------------------------------------------------------

library ieee;
	use ieee.std_logic_1164.all;
	use ieee.numeric_std.all;

entity flash_rom_xip is
	generic (
		FLASH_BASE_G : integer := 16#3A0000#;  -- flash byte offset of mainrom.bin
		CLK_DIV      : integer := 2            -- SPI SCLK = CLK / (2*CLK_DIV)
	);
	port (
		CLK          : in  std_logic;
		RESET        : in  std_logic;  -- sync, active high

		ROM_ADDR     : in  std_logic_vector(14 downto 0);  -- 32K BIOS/BASIC ROM
		ROM_CE       : in  std_logic;
		ROM_DATA     : out std_logic_vector(7 downto 0);
		WAIT_N       : out std_logic;  -- '1' = data ready / not selected, '0' = CPU must wait

		-- Physical SPI flash pins (Tang Nano 9K: pins 59/60/61/62)
		FLASH_DATA0  : in  std_logic;
		FLASH_NCSO   : out std_logic;
		FLASH_DCLK   : out std_logic;
		FLASH_ASDO   : out std_logic
	);
end entity;

architecture rtl of flash_rom_xip is

	type state_t is (ST_IDLE, ST_WAIT_VALID);
	signal state       : state_t := ST_IDLE;

	signal flash_addr_s  : std_logic_vector(23 downto 0);
	signal flash_start_s : std_logic := '0';
	signal flash_data_s  : std_logic_vector(7 downto 0);
	signal flash_valid_s : std_logic;
	signal flash_busy_s  : std_logic;

	signal cache_valid_s : std_logic := '0';
	signal cache_addr_s  : std_logic_vector(14 downto 0) := (others => '1');  -- won't match addr 0 at reset
	signal cache_data_s  : std_logic_vector(7 downto 0)  := (others => '0');

begin

	flash_addr_s <= std_logic_vector(to_unsigned(FLASH_BASE_G, 24) + resize(unsigned(ROM_ADDR), 24));

	u_flash: entity work.flash_reader
		generic map (
			CLK_DIV => CLK_DIV
		)
		port map (
			CLK    => CLK,
			RESET  => RESET,
			ADDR   => flash_addr_s,
			START  => flash_start_s,
			DATA   => flash_data_s,
			VALID  => flash_valid_s,
			BUSY   => flash_busy_s,
			DATA0  => FLASH_DATA0,
			NCSO   => FLASH_NCSO,
			DCLK   => FLASH_DCLK,
			ASDO   => FLASH_ASDO
		);

	ROM_DATA <= cache_data_s;

	process (CLK)
	begin
		if rising_edge(CLK) then
			if RESET = '1' then
				state         <= ST_IDLE;
				flash_start_s <= '0';
				cache_valid_s <= '0';
				cache_addr_s  <= (others => '1');
				WAIT_N        <= '1';
			else
				flash_start_s <= '0';

				case state is
					when ST_IDLE =>
						if ROM_CE = '1' and (cache_valid_s = '0' or cache_addr_s /= ROM_ADDR) then
							WAIT_N        <= '0';
							flash_start_s <= '1';
							state         <= ST_WAIT_VALID;
						else
							WAIT_N <= '1';
						end if;

					when ST_WAIT_VALID =>
						if flash_valid_s = '1' then
							cache_data_s  <= flash_data_s;
							cache_addr_s  <= ROM_ADDR;
							cache_valid_s <= '1';
							WAIT_N        <= '1';
							state         <= ST_IDLE;
						end if;

				end case;
			end if;
		end if;
	end process;

end architecture;
