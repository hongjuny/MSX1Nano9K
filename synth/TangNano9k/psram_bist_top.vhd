-------------------------------------------------------------------------------
-- psram_bist_top
--
-- Standalone PSRAM built-in-self-test top-level, INDEPENDENT of the MSX
-- core. Continuously writes then reads back a 64K-byte window of the
-- embedded PSRAM (via the exact same psram_ram_bridge.v used by the real
-- design) using an address-derived data pattern that changes every full
-- sweep, and streams a one-line report over UART after each sweep:
--
--   W=xxxxxx F=xxxxxx B=xx
--
-- W = total write/verify iterations so far (24-bit, wraps)
-- F = total mismatches so far (24-bit, sticky/cumulative - never cleared)
-- B = OR-accumulated XOR of (expected xor got) across all mismatches so far
--     (which DQ bit(s) are failing, if any)
--
-- This exists to let PLL phase (PSDA_SEL) and, later, IODELAY settings be
-- swept and compared quantitatively (pass/fail counts) instead of only
-- observing "does the whole MSX core eventually boot" - see the session
-- notes in gowin_rpll.vhd for why: many PSRAM clock/latency configurations
-- got the full MSX core partway through boot but never conclusively so;
-- this isolates the PSRAM link itself from all of that complexity.
--
-- UART: 115200 8N1 on pin 17 (Tang Nano 9K's onboard USB-UART bridge TX,
-- i.e. the FPGA's UART TX that the host PC receives).
-------------------------------------------------------------------------------

library ieee;
	use ieee.std_logic_1164.all;
	use ieee.numeric_std.all;

entity psram_bist_top is
	port (
		clk_27m         : in  std_logic;
		btn_reset_n     : in  std_logic;

		uart_tx_o       : out std_logic;

		leds_n_o        : out std_logic_vector(5 downto 0);

		O_psram_ck      : out std_logic_vector(1 downto 0);
		O_psram_ck_n    : out std_logic_vector(1 downto 0);
		O_psram_reset_n : out std_logic_vector(1 downto 0);
		IO_psram_rwds   : inout std_logic_vector(1 downto 0);
		IO_psram_dq     : inout std_logic_vector(15 downto 0);
		O_psram_cs_n    : out std_logic_vector(1 downto 0)
	);
end entity;

architecture behavior of psram_bist_top is

	component psram_ram_bridge
		generic (
			FREQ    : integer := 81_000_000;
			LATENCY : integer := 3
		);
		port (
			clk             : in  std_logic;
			clk_p           : in  std_logic;
			pll_lock        : in  std_logic;
			rst_n           : in  std_logic;
			ram_addr_i      : in  std_logic_vector(22 downto 0);
			ram_wdata_i     : in  std_logic_vector(7 downto 0);
			ram_rdata_o     : out std_logic_vector(7 downto 0);
			ram_ce_i        : in  std_logic;
			ram_oe_i        : in  std_logic;
			ram_we_i        : in  std_logic;
			ram_wait_n_o    : out std_logic;
			O_psram_ck      : out std_logic_vector(1 downto 0);
			O_psram_ck_n    : out std_logic_vector(1 downto 0);
			O_psram_reset_n : out std_logic_vector(1 downto 0);
			IO_psram_rwds   : inout std_logic_vector(1 downto 0);
			IO_psram_dq     : inout std_logic_vector(15 downto 0);
			O_psram_cs_n    : out std_logic_vector(1 downto 0);
			dbg_resetn      : out std_logic;
			dbg_req         : out std_logic;
			dbg_state       : out std_logic_vector(1 downto 0);
			dbg_ctrl_busy   : out std_logic;
			cr_read_i       : in  std_logic;
			cr_read_data_o  : out std_logic_vector(7 downto 0);
			cr_read_valid_o : out std_logic
		);
	end component;

	signal psram_clk_s   : std_logic;
	signal psram_clk_p_s : std_logic;
	signal pll_locked_s  : std_logic;
	signal unused_clkout0_s : std_logic;

	signal reset_s : std_logic;

	signal ram_addr_s      : std_logic_vector(22 downto 0);
	signal ram_wdata_s      : std_logic_vector(7 downto 0);
	signal ram_rdata_s      : std_logic_vector(7 downto 0);
	signal ram_ce_s, ram_oe_s, ram_we_s : std_logic := '0';
	signal ram_wait_n_s     : std_logic;

	constant TEST_BASE_C : unsigned(22 downto 0) := to_unsigned(0, 23);
	constant MEM_GRACE_CYCLES_C : integer := 255;  -- clk_27m cycles (~9.4us) - 10x wider than before, to definitively rule out "just needs more wait time"

	type state_t is (ST_WR_ASSERT, ST_WR_GRACE, ST_WR_POLL, ST_WR_RELEASE,
	                  ST_RD_ASSERT, ST_RD_GRACE, ST_RD_POLL, ST_RD_RELEASE,
	                  ST_CHECK,
	                  ST_RD2_ASSERT, ST_RD2_GRACE, ST_RD2_POLL, ST_RD2_RELEASE,
	                  ST_CHECK2,
	                  ST_NEXT, ST_REPORT_LATCH);
	signal state : state_t := ST_WR_ASSERT;

	signal addr_cnt   : unsigned(15 downto 0) := (others => '0');
	signal pattern_sel: unsigned(1 downto 0)  := (others => '0');
	signal grace_cnt  : unsigned(7 downto 0)  := (others => '0');
	signal expect_data: std_logic_vector(7 downto 0);
	signal write_iters : unsigned(23 downto 0) := (others => '0');
	signal fail_cnt     : unsigned(23 downto 0) := (others => '0');
	signal fail_bits    : std_logic_vector(7 downto 0) := (others => '0');
	signal first_fail_captured : std_logic := '0';
	signal first_fail_addr     : unsigned(15 downto 0) := (others => '0');
	signal first_fail_expect   : std_logic_vector(7 downto 0) := (others => '0');
	signal first_fail_got      : std_logic_vector(7 downto 0) := (others => '0');

	-- Second read-back-only check (no intervening write): does re-reading
	-- the SAME address give the SAME value as the first read? Distinguishes
	-- "writes aren't landing, reads just return stale/fixed content"
	-- (would be consistent) from "reads themselves are unstable" (would
	-- vary read to read).
	signal first_read_data      : std_logic_vector(7 downto 0) := (others => '0');
	signal read_mismatch_cnt    : unsigned(23 downto 0) := (others => '0');
	signal first_rmismatch_captured : std_logic := '0';
	signal first_rmismatch_read1 : std_logic_vector(7 downto 0) := (others => '0');
	signal first_rmismatch_read2 : std_logic_vector(7 downto 0) := (others => '0');

	-- latched values for the report line, frozen at the start of printing
	-- so they don't change mid-message
	signal rep_write_iters : unsigned(23 downto 0) := (others => '0');
	signal rep_fail_cnt    : unsigned(23 downto 0) := (others => '0');
	signal rep_fail_bits   : std_logic_vector(7 downto 0) := (others => '0');

	signal uart_data_s  : std_logic_vector(7 downto 0);
	signal uart_start_s : std_logic := '0';
	signal uart_busy_s  : std_logic;

	signal report_idx     : integer range 0 to 63 := 0;
	signal report_active  : std_logic := '0';  -- owned solely by the UART report process below
	signal report_trigger : std_logic := '0';  -- pulsed by the main FSM process

	function hex_char(nib : std_logic_vector(3 downto 0)) return std_logic_vector is
		variable v : integer;
	begin
		v := to_integer(unsigned(nib));
		if v < 10 then
			return std_logic_vector(to_unsigned(character'pos('0') + v, 8));
		else
			return std_logic_vector(to_unsigned(character'pos('A') + v - 10, 8));
		end if;
	end function;

begin

	reset_s <= not btn_reset_n;

	u_pll: entity work.gowin_rpll
		port map (
			clkout0       => unused_clkout0_s,
			psram_clk_o   => psram_clk_s,
			psram_clk_p_o => psram_clk_p_s,
			clkin         => clk_27m,
			lock          => pll_locked_s
		);

	u_bridge: psram_ram_bridge
		generic map (
			FREQ    => 81_000_000,
			LATENCY => 3
		)
		port map (
			clk             => psram_clk_s,
			clk_p           => psram_clk_p_s,
			pll_lock        => pll_locked_s,
			rst_n           => btn_reset_n,
			ram_addr_i      => ram_addr_s,
			ram_wdata_i     => ram_wdata_s,
			ram_rdata_o     => ram_rdata_s,
			ram_ce_i        => ram_ce_s,
			ram_oe_i        => ram_oe_s,
			ram_we_i        => ram_we_s,
			ram_wait_n_o    => ram_wait_n_s,
			O_psram_ck      => O_psram_ck,
			O_psram_ck_n    => O_psram_ck_n,
			O_psram_reset_n => O_psram_reset_n,
			IO_psram_rwds   => IO_psram_rwds,
			IO_psram_dq     => IO_psram_dq,
			O_psram_cs_n    => O_psram_cs_n,
			dbg_resetn      => open,
			dbg_req         => open,
			dbg_state       => open,
			dbg_ctrl_busy   => open,
			cr_read_i       => '0',
			cr_read_data_o  => open,
			cr_read_valid_o => open
		);

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

	ram_addr_s <= std_logic_vector(TEST_BASE_C + resize(addr_cnt, 23));

	expect_data <= std_logic_vector(unsigned(addr_cnt(7 downto 0)) xor
	                                 unsigned(addr_cnt(15 downto 8)) xor
	                                 (pattern_sel & pattern_sel & pattern_sel & pattern_sel));

	ram_wdata_s <= expect_data;

	-- Main BIST FSM: clk_27m domain (the bridge synchronizes ram_ce_i/we_i/
	-- oe_i internally, so any clean external clock domain is fine here -
	-- see psram_ram_bridge.v's own ce_sync/we_sync/oe_sync).
	process (clk_27m)
	begin
		if rising_edge(clk_27m) then
			if reset_s = '1' then
				state        <= ST_WR_ASSERT;
				addr_cnt     <= (others => '0');
				pattern_sel  <= (others => '0');
				ram_ce_s     <= '0';
				ram_we_s     <= '0';
				ram_oe_s     <= '0';
				write_iters  <= (others => '0');
				fail_cnt     <= (others => '0');
				fail_bits    <= (others => '0');
				first_fail_captured <= '0';
				read_mismatch_cnt <= (others => '0');
				first_rmismatch_captured <= '0';
				report_trigger <= '0';
			else
				report_trigger <= '0';
				case state is
					when ST_WR_ASSERT =>
						ram_ce_s   <= '1';
						ram_we_s   <= '1';
						grace_cnt  <= (others => '0');
						state      <= ST_WR_GRACE;

					when ST_WR_GRACE =>
						if grace_cnt = to_unsigned(MEM_GRACE_CYCLES_C - 1, grace_cnt'length) then
							state <= ST_WR_POLL;
						else
							grace_cnt <= grace_cnt + 1;
						end if;

					when ST_WR_POLL =>
						if ram_wait_n_s = '1' then
							ram_ce_s  <= '0';
							ram_we_s  <= '0';
							grace_cnt <= (others => '0');
							state     <= ST_WR_RELEASE;
						end if;

					when ST_WR_RELEASE =>
						-- Extra idle margin before the next request - give the
						-- bridge's internal ce/we/oe synchronizers (in the much
						-- faster psram_clk_s domain) plenty of time to see the
						-- request genuinely drop before we assert a new one.
						if grace_cnt = to_unsigned(MEM_GRACE_CYCLES_C - 1, grace_cnt'length) then
							state <= ST_RD_ASSERT;
						else
							grace_cnt <= grace_cnt + 1;
						end if;

					when ST_RD_ASSERT =>
						ram_ce_s  <= '1';
						ram_oe_s  <= '1';
						grace_cnt <= (others => '0');
						state     <= ST_RD_GRACE;

					when ST_RD_GRACE =>
						if grace_cnt = to_unsigned(MEM_GRACE_CYCLES_C - 1, grace_cnt'length) then
							state <= ST_RD_POLL;
						else
							grace_cnt <= grace_cnt + 1;
						end if;

					when ST_RD_POLL =>
						if ram_wait_n_s = '1' then
							ram_ce_s  <= '0';
							ram_oe_s  <= '0';
							grace_cnt <= (others => '0');
							state     <= ST_RD_RELEASE;
						end if;

					when ST_RD_RELEASE =>
						if grace_cnt = to_unsigned(MEM_GRACE_CYCLES_C - 1, grace_cnt'length) then
							state <= ST_CHECK;
						else
							grace_cnt <= grace_cnt + 1;
						end if;

					when ST_CHECK =>
						write_iters <= write_iters + 1;
						first_read_data <= ram_rdata_s;
						if ram_rdata_s /= expect_data then
							fail_cnt  <= fail_cnt + 1;
							fail_bits <= fail_bits or (ram_rdata_s xor expect_data);
							if first_fail_captured = '0' then
								first_fail_captured <= '1';
								first_fail_addr      <= addr_cnt;
								first_fail_expect     <= expect_data;
								first_fail_got        <= ram_rdata_s;
							end if;
						end if;
						state <= ST_RD2_ASSERT;

					when ST_RD2_ASSERT =>
						ram_ce_s  <= '1';
						ram_oe_s  <= '1';
						grace_cnt <= (others => '0');
						state     <= ST_RD2_GRACE;

					when ST_RD2_GRACE =>
						if grace_cnt = to_unsigned(MEM_GRACE_CYCLES_C - 1, grace_cnt'length) then
							state <= ST_RD2_POLL;
						else
							grace_cnt <= grace_cnt + 1;
						end if;

					when ST_RD2_POLL =>
						if ram_wait_n_s = '1' then
							ram_ce_s  <= '0';
							ram_oe_s  <= '0';
							grace_cnt <= (others => '0');
							state     <= ST_RD2_RELEASE;
						end if;

					when ST_RD2_RELEASE =>
						if grace_cnt = to_unsigned(MEM_GRACE_CYCLES_C - 1, grace_cnt'length) then
							state <= ST_CHECK2;
						else
							grace_cnt <= grace_cnt + 1;
						end if;

					when ST_CHECK2 =>
						if ram_rdata_s /= first_read_data then
							read_mismatch_cnt <= read_mismatch_cnt + 1;
							if first_rmismatch_captured = '0' then
								first_rmismatch_captured <= '1';
								first_rmismatch_read1     <= first_read_data;
								first_rmismatch_read2     <= ram_rdata_s;
							end if;
						end if;
						state <= ST_NEXT;

					when ST_NEXT =>
						if addr_cnt = to_unsigned(65535, addr_cnt'length) then
							addr_cnt    <= (others => '0');
							pattern_sel <= pattern_sel + 1;
							state       <= ST_REPORT_LATCH;
						else
							addr_cnt <= addr_cnt + 1;
							state    <= ST_WR_ASSERT;
						end if;

					when ST_REPORT_LATCH =>
						rep_write_iters <= write_iters;
						rep_fail_cnt    <= fail_cnt;
						rep_fail_bits   <= fail_bits;
						report_trigger  <= '1';
						state           <= ST_WR_ASSERT;

				end case;
			end if;
		end if;
	end process;

	-- UART report sequencer: "W=xxxxxx F=xxxxxx B=xx\r\n" (23 chars)
	process (clk_27m)
	begin
		if rising_edge(clk_27m) then
			if reset_s = '1' then
				report_idx    <= 0;
				uart_start_s  <= '0';
				report_active <= '0';
			else
				uart_start_s <= '0';
				if report_trigger = '1' then
					report_active <= '1';
				end if;
				if (report_active = '1' or report_trigger = '1') and uart_busy_s = '0' and uart_start_s = '0' then
					case report_idx is
						when 0 => uart_data_s <= std_logic_vector(to_unsigned(character'pos('W'), 8));
						when 1 => uart_data_s <= std_logic_vector(to_unsigned(character'pos('='), 8));
						when 2 => uart_data_s <= hex_char(std_logic_vector(rep_write_iters(23 downto 20)));
						when 3 => uart_data_s <= hex_char(std_logic_vector(rep_write_iters(19 downto 16)));
						when 4 => uart_data_s <= hex_char(std_logic_vector(rep_write_iters(15 downto 12)));
						when 5 => uart_data_s <= hex_char(std_logic_vector(rep_write_iters(11 downto 8)));
						when 6 => uart_data_s <= hex_char(std_logic_vector(rep_write_iters(7 downto 4)));
						when 7 => uart_data_s <= hex_char(std_logic_vector(rep_write_iters(3 downto 0)));
						when 8 => uart_data_s <= std_logic_vector(to_unsigned(character'pos(' '), 8));
						when 9 => uart_data_s <= std_logic_vector(to_unsigned(character'pos('F'), 8));
						when 10 => uart_data_s <= std_logic_vector(to_unsigned(character'pos('='), 8));
						when 11 => uart_data_s <= hex_char(std_logic_vector(rep_fail_cnt(23 downto 20)));
						when 12 => uart_data_s <= hex_char(std_logic_vector(rep_fail_cnt(19 downto 16)));
						when 13 => uart_data_s <= hex_char(std_logic_vector(rep_fail_cnt(15 downto 12)));
						when 14 => uart_data_s <= hex_char(std_logic_vector(rep_fail_cnt(11 downto 8)));
						when 15 => uart_data_s <= hex_char(std_logic_vector(rep_fail_cnt(7 downto 4)));
						when 16 => uart_data_s <= hex_char(std_logic_vector(rep_fail_cnt(3 downto 0)));
						when 17 => uart_data_s <= std_logic_vector(to_unsigned(character'pos(' '), 8));
						when 18 => uart_data_s <= std_logic_vector(to_unsigned(character'pos('B'), 8));
						when 19 => uart_data_s <= std_logic_vector(to_unsigned(character'pos('='), 8));
						when 20 => uart_data_s <= hex_char(rep_fail_bits(7 downto 4));
						when 21 => uart_data_s <= hex_char(rep_fail_bits(3 downto 0));
						when 22 => uart_data_s <= std_logic_vector(to_unsigned(character'pos(' '), 8));
						when 23 => uart_data_s <= std_logic_vector(to_unsigned(character'pos('A'), 8));
						when 24 => uart_data_s <= std_logic_vector(to_unsigned(character'pos('='), 8));
						when 25 => uart_data_s <= hex_char(std_logic_vector(first_fail_addr(15 downto 12)));
						when 26 => uart_data_s <= hex_char(std_logic_vector(first_fail_addr(11 downto 8)));
						when 27 => uart_data_s <= hex_char(std_logic_vector(first_fail_addr(7 downto 4)));
						when 28 => uart_data_s <= hex_char(std_logic_vector(first_fail_addr(3 downto 0)));
						when 29 => uart_data_s <= std_logic_vector(to_unsigned(character'pos(' '), 8));
						when 30 => uart_data_s <= std_logic_vector(to_unsigned(character'pos('E'), 8));
						when 31 => uart_data_s <= std_logic_vector(to_unsigned(character'pos('='), 8));
						when 32 => uart_data_s <= hex_char(first_fail_expect(7 downto 4));
						when 33 => uart_data_s <= hex_char(first_fail_expect(3 downto 0));
						when 34 => uart_data_s <= std_logic_vector(to_unsigned(character'pos(' '), 8));
						when 35 => uart_data_s <= std_logic_vector(to_unsigned(character'pos('G'), 8));
						when 36 => uart_data_s <= std_logic_vector(to_unsigned(character'pos('='), 8));
						when 37 => uart_data_s <= hex_char(first_fail_got(7 downto 4));
						when 38 => uart_data_s <= hex_char(first_fail_got(3 downto 0));
						when 39 => uart_data_s <= std_logic_vector(to_unsigned(character'pos(' '), 8));
						when 40 => uart_data_s <= std_logic_vector(to_unsigned(character'pos('R'), 8));
						when 41 => uart_data_s <= std_logic_vector(to_unsigned(character'pos('M'), 8));
						when 42 => uart_data_s <= std_logic_vector(to_unsigned(character'pos('='), 8));
						when 43 => uart_data_s <= hex_char(std_logic_vector(read_mismatch_cnt(23 downto 20)));
						when 44 => uart_data_s <= hex_char(std_logic_vector(read_mismatch_cnt(19 downto 16)));
						when 45 => uart_data_s <= hex_char(std_logic_vector(read_mismatch_cnt(15 downto 12)));
						when 46 => uart_data_s <= hex_char(std_logic_vector(read_mismatch_cnt(11 downto 8)));
						when 47 => uart_data_s <= hex_char(std_logic_vector(read_mismatch_cnt(7 downto 4)));
						when 48 => uart_data_s <= hex_char(std_logic_vector(read_mismatch_cnt(3 downto 0)));
						when 49 => uart_data_s <= std_logic_vector(to_unsigned(character'pos(' '), 8));
						when 50 => uart_data_s <= std_logic_vector(to_unsigned(character'pos('R'), 8));
						when 51 => uart_data_s <= std_logic_vector(to_unsigned(character'pos('1'), 8));
						when 52 => uart_data_s <= std_logic_vector(to_unsigned(character'pos('='), 8));
						when 53 => uart_data_s <= hex_char(first_rmismatch_read1(7 downto 4));
						when 54 => uart_data_s <= hex_char(first_rmismatch_read1(3 downto 0));
						when 55 => uart_data_s <= std_logic_vector(to_unsigned(character'pos(' '), 8));
						when 56 => uart_data_s <= std_logic_vector(to_unsigned(character'pos('R'), 8));
						when 57 => uart_data_s <= std_logic_vector(to_unsigned(character'pos('2'), 8));
						when 58 => uart_data_s <= std_logic_vector(to_unsigned(character'pos('='), 8));
						when 59 => uart_data_s <= hex_char(first_rmismatch_read2(7 downto 4));
						when 60 => uart_data_s <= hex_char(first_rmismatch_read2(3 downto 0));
						when 61 => uart_data_s <= std_logic_vector(to_unsigned(13, 8)); -- \r
						when others => uart_data_s <= std_logic_vector(to_unsigned(10, 8)); -- \n
					end case;
					uart_start_s <= '1';
					if report_idx = 62 then
						report_idx    <= 0;
						report_active <= '0';
					else
						report_idx <= report_idx + 1;
					end if;
				end if;
			end if;
		end if;
	end process;

	leds_n_o(0) <= not pll_locked_s;
	leds_n_o(1) <= not btn_reset_n;
	leds_n_o(2) <= write_iters(0);          -- toggles on every iteration - liveness
	leds_n_o(3) <= '1' when fail_cnt = 0 else '0';  -- lit (0=on) whenever any failure recorded
	leds_n_o(4) <= '1';
	leds_n_o(5) <= '1';

end architecture;
