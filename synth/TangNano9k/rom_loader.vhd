-------------------------------------------------------------------------------
-- rom_loader
--
-- One-shot boot-time copy: reads ROM_SIZE_G bytes starting at FLASH_BASE_G
-- in the external SPI config flash (spare space past the end of the FPGA
-- bitstream) and writes them into the embedded PSRAM at PSRAM_BASE_G, using
-- the exact same request/wait protocol the CPU uses against
-- psram_ram_bridge (REQ_CE/REQ_WE + WAIT_N) - see top_tangnano9k.vhd, which
-- muxes this module's requests onto the bridge's ram_* bus while DONE='0'
-- and holds the whole system in reset meanwhile.
--
-- This is what makes the MSX BIOS/BASIC ROM (mainrom.vhd, 32K) available
-- without fitting in the (already full) internal BSRAM and without an SD
-- card: its content lives in spare flash space and gets copied into the
-- much larger embedded PSRAM once at every power-up/reset.
--
-- The flash image must be written separately from the bitstream, e.g.:
--   openFPGALoader -b tangnano9k --external-flash --offset 0x3A0000 mainrom.bin
-------------------------------------------------------------------------------

library ieee;
	use ieee.std_logic_1164.all;
	use ieee.numeric_std.all;

entity rom_loader is
	generic (
		FLASH_BASE_G : integer := 16#3A0000#;  -- byte offset in SPI flash
		PSRAM_BASE_G : integer := 16#080000#;  -- byte offset in PSRAM die 0
		ROM_SIZE_G   : integer := 32768
	);
	port (
		CLK    : in  std_logic;
		RESET  : in  std_logic;  -- sync, active high

		-- Physical SPI flash pins (Tang Nano 9K: pins 59/60/61/62)
		FLASH_DATA0 : in  std_logic;
		FLASH_NCSO  : out std_logic;
		FLASH_DCLK  : out std_logic;
		FLASH_ASDO  : out std_logic;

		-- psram_ram_bridge request interface, same shape as the CPU's own
		-- ram_addr_i/ram_wdata_i/ram_ce_i/ram_we_i/ram_wait_n_o bus.
		REQ_ADDR  : out std_logic_vector(22 downto 0);
		REQ_WDATA : out std_logic_vector(7 downto 0);
		REQ_CE    : out std_logic;
		REQ_WE    : out std_logic;
		WAIT_N    : in  std_logic;

		DONE : out std_logic;

		-- Debug: byte counter + FSM state, for LED instrumentation while
		-- bringing the flash->PSRAM boot path up (see top_tangnano9k.vhd).
		DBG_IDX   : out std_logic_vector(15 downto 0);
		DBG_STATE : out std_logic_vector(2 downto 0);

		-- Debug: sticky pass/fail flags comparing the first 4 bytes actually
		-- read from flash against the known-correct MSX BIOS reset vector
		-- (F3 C3 D7 02 = DI; JP 02D7h) - isolates whether the flash-read/
		-- PSRAM-write pipeline itself is delivering correct data.
		DBG_B0_OK : out std_logic;  -- byte at idx=0 = F3h
		DBG_B1_OK : out std_logic;  -- byte at idx=1 = C3h
		DBG_B2_OK : out std_logic;  -- byte at idx=2 = D7h
		DBG_B3_OK : out std_logic;  -- byte at idx=3 = 02h

		-- Debug: running XOR checksum of every byte read from flash across
		-- the full ROM_SIZE_G transfer. Compare the final value (once
		-- DONE='1') against the checksum of the local mainrom.bin
		-- (computed offline) to check whether the ENTIRE 32K transfer is
		-- byte-correct, not just the first 4 bytes.
		DBG_XOR : out std_logic_vector(7 downto 0)
	);
end entity;

architecture rtl of rom_loader is

	signal flash_addr  : std_logic_vector(23 downto 0);
	signal flash_start : std_logic;
	signal flash_data  : std_logic_vector(7 downto 0);
	signal flash_valid : std_logic;
	signal flash_busy  : std_logic;

	type state_t is (ST_FLASH_REQ, ST_FLASH_WAIT, ST_MEM_ASSERT, ST_MEM_HOLD, ST_MEM_RELEASE, ST_DONE);
	signal state : state_t := ST_FLASH_REQ;
	signal idx   : unsigned(15 downto 0) := (others => '0');  -- byte counter, 0..ROM_SIZE_G-1

	-- WAIT_N (ram_wait_n_sync) is a 2-flop-synchronized view of the
	-- psram_ram_bridge's wait_n, which lives in the much faster psram_clk_s
	-- (129MHz) domain. The bridge only holds wait_n LOW for the duration of
	-- one PSRAM transaction, which can be narrower than one CLK (21.5MHz)
	-- sample period - a classic CDC pulse-width problem where the slow
	-- domain's synchronizer can miss the low pulse entirely and only ever
	-- observe WAIT_N=1. Edge-detecting "wait_n -> 0 -> 1" (as this FSM
	-- originally did) can then hang forever waiting for a '0' that never
	-- gets sampled. The T80 CPU doesn't have this problem because it only
	-- ever polls the LEVEL of wait_n and tolerates missing a pulse (at worst
	-- reading stale/one-cycle-early data), but this FSM needs to know for
	-- certain that a specific transaction has completed before advancing to
	-- the next byte, so instead: wait a fixed grace period (long enough to
	-- exceed the bridge's worst-case transaction latency) before starting to
	-- poll WAIT_N at all, then poll its LEVEL (not an edge) until it reads
	-- '1' - the post-completion HIGH level is stable/latched (not a narrow
	-- pulse), so the slow synchronizer can never miss it.
	constant MEM_GRACE_CYCLES_C : integer := 16;
	signal   mem_grace_cnt_s    : unsigned(4 downto 0) := (others => '0');

	signal dbg_b0_ok_s, dbg_b1_ok_s, dbg_b2_ok_s, dbg_b3_ok_s : std_logic := '0';
	signal dbg_xor_s : std_logic_vector(7 downto 0) := (others => '0');

begin

	u_flash: entity work.flash_reader
		generic map (
			CLK_DIV => 2
		)
		port map (
			CLK   => CLK,
			RESET => RESET,
			ADDR  => flash_addr,
			START => flash_start,
			DATA  => flash_data,
			VALID => flash_valid,
			BUSY  => flash_busy,
			DATA0 => FLASH_DATA0,
			NCSO  => FLASH_NCSO,
			DCLK  => FLASH_DCLK,
			ASDO  => FLASH_ASDO
		);

	flash_addr <= std_logic_vector(to_unsigned(FLASH_BASE_G, 24) + resize(idx, 24));
	REQ_ADDR   <= std_logic_vector(to_unsigned(PSRAM_BASE_G, 23) + resize(idx, 23));

	DBG_IDX <= std_logic_vector(idx);
	DBG_STATE <= "000" when state = ST_FLASH_REQ else
	             "001" when state = ST_FLASH_WAIT else
	             "010" when state = ST_MEM_ASSERT else
	             "011" when state = ST_MEM_HOLD else
	             "100" when state = ST_MEM_RELEASE else
	             "101";  -- ST_DONE

	DBG_B0_OK <= dbg_b0_ok_s;
	DBG_B1_OK <= dbg_b1_ok_s;
	DBG_B2_OK <= dbg_b2_ok_s;
	DBG_B3_OK <= dbg_b3_ok_s;
	DBG_XOR   <= dbg_xor_s;

	process (CLK)
		variable rdata_v : std_logic_vector(7 downto 0);
	begin
		if rising_edge(CLK) then
			if RESET = '1' then
				state          <= ST_FLASH_REQ;
				idx            <= (others => '0');
				flash_start    <= '0';
				REQ_CE         <= '0';
				REQ_WE         <= '0';
				REQ_WDATA      <= (others => '0');
				DONE           <= '0';
				mem_grace_cnt_s <= (others => '0');
				dbg_b0_ok_s <= '0';
				dbg_b1_ok_s <= '0';
				dbg_b2_ok_s <= '0';
				dbg_b3_ok_s <= '0';
				dbg_xor_s <= (others => '0');
			else
				flash_start <= '0';

				case state is
				when ST_FLASH_REQ =>
					flash_start <= '1';
					state       <= ST_FLASH_WAIT;

				when ST_FLASH_WAIT =>
					if flash_valid = '1' then
						rdata_v         := flash_data;
						REQ_WDATA       <= rdata_v;
						REQ_CE          <= '1';
						REQ_WE          <= '1';
						mem_grace_cnt_s <= (others => '0');
						state           <= ST_MEM_ASSERT;
						dbg_xor_s       <= dbg_xor_s xor rdata_v;

						if idx = 0 and rdata_v = X"F3" then
							dbg_b0_ok_s <= '1';
						end if;
						if idx = 1 and rdata_v = X"C3" then
							dbg_b1_ok_s <= '1';
						end if;
						if idx = 2 and rdata_v = X"D7" then
							dbg_b2_ok_s <= '1';
						end if;
						if idx = 3 and rdata_v = X"02" then
							dbg_b3_ok_s <= '1';
						end if;
					end if;

				when ST_MEM_ASSERT =>
					-- Ignore WAIT_N for a fixed grace period after asserting
					-- the request - otherwise we could read WAIT_N's stale
					-- '1' (idle) value from before the bridge has even seen
					-- the request and think we're already done. See
					-- MEM_GRACE_CYCLES_C comment above.
					if mem_grace_cnt_s = to_unsigned(MEM_GRACE_CYCLES_C - 1, mem_grace_cnt_s'length) then
						state <= ST_MEM_HOLD;
					else
						mem_grace_cnt_s <= mem_grace_cnt_s + 1;
					end if;

				when ST_MEM_HOLD =>
					-- Poll WAIT_N's level (not an edge) until the
					-- transaction has completed.
					if WAIT_N = '1' then
						REQ_CE <= '0';
						REQ_WE <= '0';
						state  <= ST_MEM_RELEASE;
					end if;

				when ST_MEM_RELEASE =>
					-- one idle cycle with the request deasserted before the
					-- next byte, matching the bridge's ST_HOLD->ST_IDLE gate
					if idx = to_unsigned(ROM_SIZE_G - 1, idx'length) then
						state <= ST_DONE;
					else
						idx   <= idx + 1;
						state <= ST_FLASH_REQ;
					end if;

				when ST_DONE =>
					DONE <= '1';

				end case;
			end if;
		end if;
	end process;

end architecture;
