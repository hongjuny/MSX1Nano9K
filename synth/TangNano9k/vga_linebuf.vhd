-------------------------------------------------------------------------------
-- vga_linebuf
--
-- Line-buffered replacement for src/video/vga.vhd's scan doubler, sized for
-- Tang Nano 9K's much smaller BSRAM budget. See git history for the long
-- debugging saga (buffer-depth changes, several rate-compensation attempts
-- on the READ side) that preceded this version - none of them fixed the
-- real hardware symptom (a smooth/chunky vertical roll of the displayed
-- content, confirmed independent of the base 640x480 sync generation via a
-- fixed test-grid overlay: the grid never moves, only the MSX content does).
--
-- Root cause, confirmed by that grid test plus careful re-reading of the
-- code: window_vcnt/window_hcnt (read address generation) are updated in
-- the SAME clocked process, under the SAME condition, as vcnt/hcnt (the
-- actual 640x480 output timing) - they cannot drift relative to each other,
-- full stop. The roll instead came from the WRITE side (I_CLK domain)
-- freely overwriting the small circular buffer's slots with whatever VDP
-- line it happened to be producing, with no regard for whether the READ
-- side (I_CLK_VGA domain, asynchronous to I_CLK) had actually displayed the
-- slot's PREVIOUS contents yet. Since each slot is reused vc_max/
-- BUFFER_LINES times per frame, and I_CLK/I_CLK_VGA are not exactly
-- rate-matched, which of those reuses is sitting in a slot at the moment
-- read samples it drifts over time - visible as a roll whose size exactly
-- tracks BUFFER_LINES (confirmed on hardware at 8, 32, 64) and whose total
-- cycle time does NOT shrink as BUFFER_LINES grows (also confirmed) -
-- proof that a bigger buffer only spaces out the symptom, it cannot fix it.
--
-- The actual fix: turn the circular buffer into a real single-clock-domain-
-- write / single-clock-domain-read ASYNC FIFO, decoupled from absolute VDP
-- line numbers entirely. Write pushes each completed VDP line in order
-- (0,1,2,...,vc_max-1 each frame) but SKIPS the push if the FIFO is full
-- (write has gotten BUFFER_LINES ahead of what read has consumed) - the
-- skipped line is simply not captured that pass; read's next display of
-- that slot shows the last successfully-pushed content instead (a rare,
-- single repeated scanline - not a multi-line jump). Read pops
-- unconditionally on its own fixed schedule (it can never stall - the
-- output must stay fixed-rate for the monitor), which naturally shows
-- slightly-stale content on the rare occasions write has fallen behind
-- (also just a rare repeated scanline, symmetric case). Both pointers are
-- wide (16-bit) free-running counters - standard async-FIFO Gray-code
-- pointer technique for the cross-domain full check, sized with enough
-- headroom that the write/read pointer distance can never realistically
-- wrap around and alias during normal operation.
--
-- First cut of this FIFO redesign still rolled (differently: smooth slides
-- with occasional pauses/ghosting instead of chunky jumps). Root cause: a
-- pure count-based FIFO has no concept of "frame" at all - it just pushes
-- and pops an anonymous stream. Read assumes "the Kth item I pop this VGA
-- frame is VDP source line K", which is only true if read's pop count and
-- write's push count stay aligned modulo vc_max across many frames - and
-- since neither side's stream length is *exactly* vc_max in real time
-- (that's the whole underlying rate mismatch), that alignment drifts too,
-- just as smooth "phase" drift through the anonymous stream instead of a
-- chunky slot-reuse artifact.
--
-- Fix: give the FIFO frame awareness. Write snapshots its own pointer
-- (wr_frame_ptr_s) every time the VDP starts a new frame (I_VCNT wraps to
-- 0), Gray-coded and synchronized into the read domain exactly like the
-- main pointer. Read, at the START of every VGA frame (vcnt=v_start-1 -
-- the same point window_vcnt already resets to 0, so this never touches
-- window_vcnt/vcnt/hcnt's own timing), snaps rd_ptr to that synchronized
-- value instead of just incrementing it - re-anchoring to write's most
-- recent frame start once per VGA frame. This bounds any drift to at most
-- one frame's worth (a rare single stale/repeated line at worst) instead
-- of letting it accumulate indefinitely, while keeping the FIFO's
-- within-frame flow control (fifo_full_s) intact for the rest of the
-- frame.
-------------------------------------------------------------------------------

library IEEE;
	use IEEE.std_logic_1164.all;
	use IEEE.std_logic_unsigned.all;
	use IEEE.numeric_std.all;

entity vga_linebuf is
port (
	I_CLK			: in std_logic;
	I_CLK_VGA	: in std_logic;
	I_COLOR		: in std_logic_vector(3 downto 0);
	I_HCNT		: in std_logic_vector(8 downto 0);
	I_VCNT		: in std_logic_vector(7 downto 0);
	O_HSYNC		: out std_logic;
	O_VSYNC		: out std_logic;
	O_COLOR		: out std_logic_vector(3 downto 0);
	O_BLANK		: out std_logic);
end vga_linebuf;

architecture rtl of vga_linebuf is

	constant BUFFER_LINES	: integer := 64;	-- must be a power of 2
	constant SLOT_BITS		: integer := 6;	-- log2(BUFFER_LINES)
	constant PTR_BITS			: integer := 16;	-- FIFO pointer width - see header comment

	signal pixel_out		: std_logic_vector(3 downto 0);
	signal addr_rd			: std_logic_vector(14 downto 0);
	signal addr_wr			: std_logic_vector(14 downto 0);
	signal wren				: std_logic;
	signal picture			: std_logic;
	signal window_hcnt	: std_logic_vector(9 downto 0) := (others => '0');
	signal window_vcnt	: std_logic_vector(8 downto 0) := (others => '0');
	signal hcnt				: std_logic_vector(9 downto 0) := (others => '0');
	signal h					: std_logic_vector(9 downto 0) := (others => '0');
	signal vcnt				: std_logic_vector(9 downto 0) := (others => '0');
	signal hsync			: std_logic;
	signal vsync			: std_logic;
	signal blank			: std_logic;

	-- Async FIFO pointers (see header comment). Gray-coded across the
	-- clock-domain crossing (consecutive Gray codes differ by only one
	-- bit, safe for a multi-bit CDC via a plain 2-3 flop synchronizer).
	function bin2gray(b : std_logic_vector(PTR_BITS-1 downto 0)) return std_logic_vector is
	begin
		return b xor ('0' & b(PTR_BITS-1 downto 1));
	end function;

	function gray2bin(g : std_logic_vector(PTR_BITS-1 downto 0)) return std_logic_vector is
		variable b : std_logic_vector(PTR_BITS-1 downto 0);
	begin
		b(PTR_BITS-1) := g(PTR_BITS-1);
		for i in PTR_BITS-2 downto 0 loop
			b(i) := b(i+1) xor g(i);
		end loop;
		return b;
	end function;

	signal ivcnt_prev_s		: std_logic_vector(7 downto 0) := (others => '0');  -- I_CLK domain

	-- Write side (I_CLK domain)
	signal wr_ptr_s			: unsigned(PTR_BITS-1 downto 0) := (others => '0');
	signal wr_ptr_gray_s	: std_logic_vector(PTR_BITS-1 downto 0) := (others => '0');
	signal wr_slot_s			: unsigned(SLOT_BITS-1 downto 0) := (others => '0');
	signal line_accepted_s	: std_logic := '0';
	signal rd_ptr_gray_sync1_s : std_logic_vector(PTR_BITS-1 downto 0) := (others => '0');  -- I_CLK domain
	signal rd_ptr_gray_sync2_s : std_logic_vector(PTR_BITS-1 downto 0) := (others => '0');  -- I_CLK domain
	signal rd_ptr_sync_s	: unsigned(PTR_BITS-1 downto 0);  -- combinational, I_CLK domain
	signal fifo_full_s		: std_logic;                       -- combinational, I_CLK domain

	-- Frame-boundary resync (see header comment): snapshot of wr_ptr_s taken
	-- each time the VDP starts a new frame (I_VCNT wraps to 0), synchronized
	-- into the read domain the same way as the main pointer.
	signal vdp_frame_pulse_s	: std_logic := '0';                       -- I_CLK domain
	signal wr_frame_ptr_s		: unsigned(PTR_BITS-1 downto 0) := (others => '0');  -- I_CLK domain
	signal wr_frame_gray_s		: std_logic_vector(PTR_BITS-1 downto 0) := (others => '0');  -- I_CLK domain
	signal wr_frame_gray_sync1_s : std_logic_vector(PTR_BITS-1 downto 0) := (others => '0');  -- I_CLK_VGA domain
	signal wr_frame_gray_sync2_s : std_logic_vector(PTR_BITS-1 downto 0) := (others => '0');  -- I_CLK_VGA domain
	signal wr_frame_ptr_sync_s	: unsigned(PTR_BITS-1 downto 0);  -- combinational, I_CLK_VGA domain

	-- Read side (I_CLK_VGA domain)
	signal rd_ptr_s			: unsigned(PTR_BITS-1 downto 0) := (others => '0');
	signal rd_ptr_gray_s	: std_logic_vector(PTR_BITS-1 downto 0) := (others => '0');
	signal rd_slot_s			: unsigned(SLOT_BITS-1 downto 0) := (others => '0');

-- ModeLine "640x480@60Hz"  25,175  640  656  752  800 480 490 492 525 -HSync -VSync
	-- Horizontal Timing constants
	constant h_pixels_across	: integer := 640 - 1;
	constant h_sync_on			: integer := 656 - 1;
	constant h_sync_off			: integer := 752 - 1;
	constant h_end_count			: integer := 800 - 1;
	-- Vertical Timing constants
	constant v_pixels_down		: integer := 480 - 1;
	constant v_sync_on			: integer := 490 - 1;
	constant v_sync_off			: integer := 492 - 1;
	constant v_end_count			: integer := 525 - 1;

	-- Capture window (VDP-native pixels captured into the line buffer)
	constant hc_max				: integer := 280;
	constant vc_max				: integer := 216;

	constant h_start				: integer := 40;
	constant h_end					: integer := h_start + (hc_max * 2);	-- 64 + (280 * 2) => 64 + 560 = 624
	constant v_start				: integer := 22;
	constant v_end					: integer := v_start + (vc_max * 2);

begin

	linebuf: entity work.dpram
	generic map (
		addr_width_g	=> 15,	-- BUFFER_LINES(64) * 512-wide slot = 2^15
		data_width_g	=> 4
	)
	port map(
		clk_a_i		=> I_CLK,
		data_a_i		=> I_COLOR,
		addr_a_i		=> addr_wr,
		we_i			=> wren,
		data_a_o		=> open,
		--
		clk_b_i		=> I_CLK_VGA,
		addr_b_i		=> addr_rd,
		data_b_o		=> pixel_out
	);

	-- Write side: push each completed VDP line (0..vc_max-1) into the FIFO
	-- in order, skipping the push (dropping that line's capture) if the
	-- FIFO is full - i.e. if write has already gotten BUFFER_LINES ahead of
	-- what read has consumed.
	--
	-- IMPORTANT: the frame-resync below can make rd_ptr_s momentarily land
	-- AHEAD of wr_ptr_s (e.g. right after write was throttled for most of a
	-- frame, then the next resync jumps read to write's latest snapshot+1,
	-- which can exceed write's CURRENT, still-behind position). A plain
	-- unsigned (wr_ptr_s - rd_ptr_sync_s) underflows in that case, wrapping
	-- to ~65535 and reading as permanently "full" - a real deadlock hit on
	-- hardware (write never accepts another line again, screen freezes on
	-- whatever was last buffered). Comparing as SIGNED instead treats a
	-- negative difference (read ahead of write) as correctly "not full".
	rd_ptr_sync_s <= unsigned(gray2bin(rd_ptr_gray_sync2_s));
	fifo_full_s   <= '1' when signed(std_logic_vector(wr_ptr_s)) - signed(std_logic_vector(rd_ptr_sync_s)) >= to_signed(BUFFER_LINES, PTR_BITS) else '0';

	process (I_CLK)
	begin
		if rising_edge(I_CLK) then
			rd_ptr_gray_sync1_s <= rd_ptr_gray_s;
			rd_ptr_gray_sync2_s <= rd_ptr_gray_sync1_s;

			if unsigned(I_VCNT) = 0 and unsigned(ivcnt_prev_s) /= 0 then
				-- VDP frame start: snapshot the pointer value line 0 is
				-- about to use (or would use, if not dropped below) so
				-- read can re-anchor to it once per VGA frame.
				wr_frame_ptr_s <= wr_ptr_s;
			end if;
			wr_frame_gray_s <= bin2gray(std_logic_vector(wr_frame_ptr_s));

			if I_VCNT /= ivcnt_prev_s and unsigned(I_VCNT) < vc_max then
				-- Start of a new source line: decide once whether to
				-- accept it into the FIFO, and hold that decision (and the
				-- target slot) for the whole line's pixel writes.
				if fifo_full_s = '0' then
					line_accepted_s <= '1';
					wr_slot_s       <= wr_ptr_s(SLOT_BITS-1 downto 0);
					wr_ptr_s        <= wr_ptr_s + 1;
				else
					line_accepted_s <= '0';
				end if;
			elsif unsigned(I_VCNT) >= vc_max then
				line_accepted_s <= '0';
			end if;

			wr_ptr_gray_s <= bin2gray(std_logic_vector(wr_ptr_s));
			ivcnt_prev_s  <= I_VCNT;
		end if;
	end process;

	wren    <= '1' when (I_HCNT < hc_max) and (I_VCNT < vc_max) and (line_accepted_s = '1') else '0';
	addr_wr <= std_logic_vector(wr_slot_s) & I_HCNT;

	-- Read side: pop unconditionally on the fixed 640x480 schedule (it can
	-- never stall the output), once per SOURCE line (window_vcnt odd->even,
	-- i.e. completing a 2x-doubled pair). rd_slot_s is only updated at
	-- those points and otherwise holds steady for the second output line of
	-- each pair.
	wr_frame_ptr_sync_s <= unsigned(gray2bin(wr_frame_gray_sync2_s));

	process (I_CLK_VGA)
	begin
		if I_CLK_VGA'event and I_CLK_VGA = '1' then
			rd_ptr_gray_s <= bin2gray(std_logic_vector(rd_ptr_s));
			wr_frame_gray_sync1_s <= wr_frame_gray_s;
			wr_frame_gray_sync2_s <= wr_frame_gray_sync1_s;

			if h = h_end_count then
				h <= (others => '0');
			else
				h <= h + 1;
			end if;

			if h = 7 then
				hcnt <= (others => '0');
			else
				hcnt <= hcnt + 1;
				if hcnt = (h_start-1) then
					window_hcnt <= (others => '0');
				else
					window_hcnt <= window_hcnt + 1;
				end if;
			end if;
			if hcnt = h_sync_on then
				if vcnt = v_end_count then
					vcnt <= (others => '0');
				else
					vcnt <= vcnt + 1;
					if vcnt = (v_start-1) then
						-- Start of a new VGA frame: re-anchor to write's
						-- most recent VDP-frame-start pointer (synced),
						-- instead of just continuing rd_ptr_s's own count -
						-- see header comment. window_vcnt/vcnt themselves
						-- are untouched by this, still perfectly locked.
						window_vcnt <= (others => '0');
						rd_slot_s   <= wr_frame_ptr_sync_s(SLOT_BITS-1 downto 0);
						rd_ptr_s    <= wr_frame_ptr_sync_s + 1;
					elsif window_vcnt(0) = '1' then
						window_vcnt <= window_vcnt + 1;
						if unsigned(window_vcnt(8 downto 1)) < vc_max-1 then
							rd_slot_s <= rd_ptr_s(SLOT_BITS-1 downto 0);
							rd_ptr_s  <= rd_ptr_s + 1;
						end if;
					else
						window_vcnt <= window_vcnt + 1;
					end if;
				end if;
			end if;
		end if;
	end process;

	addr_rd <= std_logic_vector(rd_slot_s) & window_hcnt(9 downto 1);

	blank		<= '1' when (hcnt > h_pixels_across) or (vcnt > v_pixels_down) else '0';
	picture	<= '1' when (blank = '0') and (hcnt > h_start and hcnt < h_end) and (vcnt > v_start and vcnt < v_end) else '0';

	O_HSYNC	<= '1' when (hcnt <= h_sync_on) or (hcnt > h_sync_off) else '0';
	O_VSYNC	<= '1' when (vcnt <= v_sync_on) or (vcnt > v_sync_off) else '0';

	-- DEBUG grid overlay kept from the previous test (see conversation) -
	-- confirms the base 640x480 output stays rock-solid regardless of what
	-- this rewrite does to the read/write FIFO logic.
	O_COLOR	<= "1111" when (blank = '0') and (hcnt(4 downto 0) = "00000" or vcnt(4 downto 0) = "00000") else
	           pixel_out when picture = '1' else
	           (others => '0');
	O_BLANK	<= blank;

end rtl;
