-------------------------------------------------------------------------------
-- vga_linebuf
--
-- Line-buffered replacement for src/video/vga.vhd's scan doubler. See git
-- history for the long saga (Gray-coded async FIFOs, a full framebuffer that
-- didn't fit BSRAM, 2-bank then 3-slot ping-pong attempts). This revision
-- fixes a third-round review finding on the previous 3-slot version - the
-- worst one yet, and the direct cause of the "vertical white bar" artifact:
--
--   THE BUG: wren_gated re-checked slot_free_s *live* on every single clock
--   cycle of a line's capture, instead of latching a single writability
--   decision once at line start. If a slot wasn't free yet when a line
--   began (VGA still technically "using" it per bookkeeping), writes were
--   correctly blocked at first - but if an unrelated ack happened to flip
--   that same free bit to '1' partway through the line (entirely possible,
--   acks arrive whenever VGA's independent clock domain gets around to
--   sending one), wren_gated's live re-evaluation would suddenly start
--   allowing writes for the *rest* of that line. The result: a slot
--   published as "one complete line" that was actually a splice - stale
--   leftover pixels (usually the previous frame's text) in the prefix
--   columns, freshly written pixels only in the suffix. Displayed
--   repeatedly while VGA hunts for the next frame's row 0 (during which it
--   keeps re-showing the last locked display_slot_s), a spliced line's
--   stale-text prefix reads as a column of vertical white bars roughly at
--   character-column spacing - exactly what showed up on hardware.
--
--   THE FIX: capture_active_s + capture_line_s are latched *once*, at the
--   exact clock edge a new line begins (I_VCNT changes), from a purely
--   combinational start_capture_s decision - and never re-evaluated against
--   slot_free_s again for the rest of that line. The slot is also claimed
--   (marked unavailable) at that same instant, not deferred to publish
--   time, closing the ownership gap the live-check bug exploited.
--
-- Other fixes in this revision (found in the same review):
--   - Initial slot_free_s used to mark every slot free while display_slot_s
--     already owned one of them from power-up - a broken invariant (a slot
--     simultaneously "free" and "owned by the display"). Now the slot
--     display_slot_s starts on is correctly marked unavailable at reset.
--   - Bumped from 3 slots to 8: with only 3, the strict "latch once at line
--     start" fix above meant a slot not yet freed by the time its line
--     started was skipped *entirely* (not partially, as the old live-check
--     bug allowed) - and 3 slots turned out not to give enough buffering
--     depth to reliably absorb the VDP<->VGA ack round-trip latency, so
--     lines were being dropped often enough to reintroduce both the scroll
--     (via expected_line_s mismatches) and flickering text. BSRAM cost is
--     irrelevant either way (a few Kbit at most), so there's no reason not
--     to give this much more slack.
--   - locked_s now clears at every frame start (alongside picture_active_s/
--     synced_this_frame_s) and only sets again once a genuine exact-match
--     row-0 adoption happens - so the picture blanks instead of repeating
--     a stale locked slot while hunting for the new frame's row 0.
--   - The publish/ack next-state computation (pub_pending_s, ready_valid_s,
--     and the registers they gate) is now resolved through process-local
--     variables so a same-I_CLK-edge coincidence (a line completing exactly
--     when an ack arrives) can't have one signal assignment silently
--     overwrite the other's intended effect (VHDL signal semantics: only
--     the last assignment in the process is visible, they don't merge).
--
-- FOURTH revision: with the buffer logic now fully correct (no more
-- corruption/flicker), what remained was a genuine ~0.05% frame-rate
-- mismatch between VDP (~60.0298 Hz, from the master clock/VDP timing
-- constants - 342 cnt/line x 262 lines/frame x 4 master cycles/clk_en_5m37
-- pulse @ 21.515625 MHz) and VGA (exactly 60.000 Hz, 800x525 @ 25.2MHz).
-- Re-tuning the HDMI pixel PLL to close that gap isn't achievable: the
-- CLKDIV feeding the TMDS serializer is hardwired to /5, and the nearest
-- achievable CLKOUT/5 tap to the needed ~25.2125MHz is exactly today's
-- 25.2MHz already (the next candidates are ~124.9MHz/5 or ~127.3MHz/5,
-- both much worse). Every prior fix in this file was logically correct but
-- had no way to keep VDP's row 0 pinned to the same VGA screen row forever
-- when the two frame periods simply don't match - re-hunting for row 0
-- every VGA frame (the old behavior) meant the top few rows visibly
-- blanked/flickered every time by a slowly drifting amount, reading as a
-- continuous scroll with a ~33.6s period (1 / (60.0298-60.000)).
--
-- Fix: absorb the drift entirely within vertical blanking, never in the
-- visible picture, and lock onto row 0 only *once* rather than every
-- frame:
--   - v_end_this_frame_s picks the current frame's total line count from
--     {524, 525, 526} instead of a fixed 525, decided once per frame at
--     the wrap point.
--   - Normally it dithers 524/525 in a fixed 553:1572 ratio (out of every
--     2125 frames) via a Bresenham-style fractional accumulator, which
--     gives an exact long-term average of 524.7397... lines/frame -
--     matching VDP's actual frame rate exactly (see the math above).
--   - A slow phase servo overrides the dither when needed: VDP's own frame
--     boundary (I_VCNT wrapping to 0) is marked with a toggle, synced into
--     the VGA domain, and the VGA line position where that edge lands
--     (vdp_phase_line_s) is compared against a small target window early
--     in the frame (chosen to leave time for row-0 capture/publish/CDC/
--     adoption to complete before v_start). Outside the window, force a
--     524-line (VDP ahead - catch up) or 526-line (VDP behind - slow down)
--     frame instead of the normal dither, nudging the two back into phase.
--   - locked_s/expected_line_s/synced_this_frame_s are no longer reset
--     every frame - row 0 is only hunted for once, at power-up. After
--     that, expected_line_s just wraps naturally at vc_max and the phase
--     servo keeps VDP's row 0 aligned with VGA's v_start indefinitely.
--
-- FIFTH revision: the dither turned out to be working correctly (confirmed
-- via LED instrumentation), but a strict-exact-match diagnostic build
-- (future-resync removed, timeout made purely observational) revealed real,
-- confirmed-by-code line-loss paths that a second-opinion review caught:
--   - The old "pending + 1-deep ready" descriptor pipe was only 2 deep
--     despite 8 physical slots - a third line completing while both were
--     occupied was explicitly dropped. Replaced with a real 8-entry
--     synchronous FIFO (fifo_slot_a/fifo_line_a), matching the slot count
--     exactly so it can never overflow by construction.
--   - start_capture_s read slot_free_s combinationally, which still showed
--     the pre-this-edge value even when an ack landing on this exact same
--     I_CLK edge was about to free precisely the slot being requested -
--     causing a spurious capture-skip once per such coincidence. Fixed via
--     effective_free_s, which also considers an ack landing this cycle.
--   - The descriptor-handshake gating used picture_active_s, which stays
--     high through the bottom vertical blanking interval (only clearing at
--     frame wrap) - asymmetric with the top blanking (where it's low from
--     wrap until v_start). As the dither shifts frame boundaries by up to
--     a line, this asymmetry could mis-consume a descriptor meant for a
--     different row. Now gated on the exact v_start..v_end vcnt range.
--
-- Also in FIFTH revision (found from a video, after the above): the FIFO
-- drain read fifo_slot_a/fifo_line_a on the same I_CLK edge a fresh
-- descriptor was pushed to them - a classic signal-array read-after-write
-- hazard (the write isn't visible until the next cycle), so an empty-to-
-- active FIFO transition republished stale leftover contents from
-- whatever was last written to that address. Fixed by draining existing
-- entries before this cycle's push, plus a direct-bypass path when both
-- the publish slot and FIFO are empty (see the process body).
--
-- SIXTH revision: with stale-descriptor republishing fixed, what remained
-- was frequency lock without phase lock - the dither correctly holds
-- VDP's and VGA's *average* frame rate equal, but never pinned *where*
-- VDP's row 0 lands relative to VGA's v_start. Two fixes:
--   - One-time boot phase acquisition: the first time VDP's own frame
--     boundary is observed, vcnt is forced to a fixed early value
--     (comfortably before v_start) so row 0 always has margin to arrive
--     before the picture starts. Fires once, ever (phase_acquired_s).
--   - Spatial matching instead of sequence matching: a descriptor is only
--     adopted if its line number equals target_line_s (derived directly
--     from window_vcnt - the row actually about to be displayed), not an
--     independently-advancing expected_line_s counter. This was the
--     deeper structural issue: once a sequence counter desyncs from
--     physical screen position (e.g. after a single dropped line), every
--     subsequent line lands at the wrong row until the numbering
--     coincidentally wraps back into alignment - reading as the whole
--     picture cycling vertically, exactly as a shared video showed. Tying
--     acceptance to physical position directly means a dropped line can
--     now only ever repeat/blank *that one row*.
--
-- SEVENTH revision: the one-time boot phase acquisition above was itself
-- still unreliable - it forced vcnt on the raw VDP frame-boundary *toggle*,
-- which only says a wrap happened, not that the descriptor sitting at the
-- FIFO head at that instant is actually row 0 (stale pre-wrap lines could
-- still be ahead of it in the pipe). Fixed by acquiring phase on the
-- genuine descriptor instead of the toggle: while phase_acquired_s = '0',
-- every arriving descriptor is inspected and, if not row 0, acked and
-- discarded as stale; the row-0 descriptor itself is left UN-acked and
-- held at the FIFO head, and only then does vcnt get forced to v_start-1.
-- Because the hold never acks it, the pre-existing (unchanged) exact-match
-- logic sees this same descriptor again the moment target_line_s reaches 0
-- and adopts it through the normal path - no separate adoption code needed.
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
	O_BLANK		: out std_logic;
	-- TEMP diagnostic (see FOURTH revision): (0) toggles each time the
	-- phase servo forces a 524-line frame, (1) toggles each time it forces
	-- a 526-line frame. Remove once the servo's behavior is confirmed.
	O_DBG			: out std_logic_vector(3 downto 0) := "0000");
end vga_linebuf;

architecture rtl of vga_linebuf is

	signal pixel_out		: std_logic_vector(3 downto 0);
	signal addr_rd			: std_logic_vector(11 downto 0);
	signal addr_wr			: std_logic_vector(11 downto 0);
	signal wren_gated	: std_logic;
	signal picture			: std_logic;
	signal window_hcnt	: std_logic_vector(9 downto 0) := (others => '0');
	signal window_vcnt	: std_logic_vector(8 downto 0) := (others => '0');
	signal hcnt				: std_logic_vector(9 downto 0) := (others => '0');
	signal h					: std_logic_vector(9 downto 0) := (others => '0');
	signal vcnt				: std_logic_vector(9 downto 0) := (others => '0');
	signal blank			: std_logic;
	signal grid_s			: std_logic;

	-- ModeLine "640x480@60Hz"  25,175  640  656  752  800 480 490 492 525 -HSync -VSync
	constant h_pixels_across	: integer := 640 - 1;
	constant h_sync_on			: integer := 656 - 1;
	constant h_sync_off			: integer := 752 - 1;
	constant h_end_count			: integer := 800 - 1;
	constant v_pixels_down		: integer := 480 - 1;
	constant v_sync_on			: integer := 490 - 1;
	constant v_sync_off			: integer := 492 - 1;
	constant v_end_count			: integer := 525 - 1;

	-- Capture window (VDP-native pixels captured into the line buffer)
	constant hc_max				: integer := 280;
	constant vc_max				: integer := 216;

	constant h_start				: integer := 40;
	constant h_end					: integer := h_start + (hc_max * 2);
	constant v_start				: integer := 22;
	constant v_end					: integer := v_start + (vc_max * 2);

	-- 8 slots (3-bit id), simple wraparound increment - see header note on
	-- why 3 slots weren\'t enough buffering depth for the ack round-trip.
	function next_slot(s : std_logic_vector(2 downto 0)) return std_logic_vector is
	begin
		if s = "111" then
			return "000";
		else
			return std_logic_vector(unsigned(s) + 1);
		end if;
	end function;

	type slot_array_t is array (0 to 7) of std_logic_vector(2 downto 0);
	type line_array_t is array (0 to 7) of std_logic_vector(7 downto 0);

	-- VDP domain (I_CLK)
	signal line_start_s			: std_logic;
	signal next_capture_slot_s	: std_logic_vector(2 downto 0);
	signal start_capture_s		: std_logic;
	-- Does an ack land on this exact I_CLK edge, and does it free precisely
	-- the slot we'd otherwise start capturing into next? Both operands are
	-- registered signals (their pre-this-edge values), so this combinational
	-- read exactly mirrors what the clocked process below will do to free_v
	-- on this same edge - closing the gap where slot_free_s (also read
	-- combinationally) would otherwise still show the old, stale "not free"
	-- state for one extra line.
	signal ack_landing_s			: std_logic;
	signal effective_free_s	: std_logic;
	signal current_slot_s		: std_logic_vector(2 downto 0) := "000";
	-- Latched once at line start, held for the whole line - see header note
	-- on THE BUG/THE FIX. Never re-derived from slot_free_s mid-line.
	signal capture_active_s	: std_logic := '0';
	signal capture_line_s		: std_logic_vector(7 downto 0) := (others => '1');
	signal slot_free_s			: std_logic_vector(7 downto 0) := "01111111";
	signal pub_pending_s		: std_logic := '0';
	signal prev_vcnt_s			: std_logic_vector(7 downto 0) := (others => '1');
	signal req_toggle_s			: std_logic := '0';
	signal ack_sync1_s			: std_logic := '0';
	signal ack_sync2_s			: std_logic := '0';
	-- Bundled with req_toggle_s - set on the same edge, held stable until
	-- acked, safe to sample directly once ack_sync2_s confirms the toggle.
	signal pub_slot_s			: std_logic_vector(2 downto 0) := "000";
	signal pub_line_s			: std_logic_vector(7 downto 0) := (others => '0');
	-- Real 8-entry descriptor FIFO (fully synchronous, I_CLK domain only -
	-- no CDC needed here, only the head entry ever crosses domains via the
	-- existing req/ack toggle). Replaces the old 1-deep "ready" queue,
	-- which - confirmed by a second-opinion review - explicitly dropped a
	-- completed line whenever both it and pub_pending_s were occupied,
	-- even though 8 physical slots existed to hold more in flight. Sized
	-- to exactly match the 8 slots, so it can never actually overflow by
	-- construction (a slot can't be captured into unless it's free, and it
	-- only becomes free again once its FIFO entry has been drained).
	signal fifo_slot_a			: slot_array_t := (others => (others => '0'));
	signal fifo_line_a			: line_array_t := (others => (others => '0'));
	signal fifo_wr_ptr_s		: unsigned(2 downto 0) := (others => '0');
	signal fifo_rd_ptr_s		: unsigned(2 downto 0) := (others => '0');
	signal fifo_count_s			: unsigned(3 downto 0) := (others => '0');
	-- Toggles once per VDP frame (I_VCNT wrapping to 0) - the phase
	-- reference the VGA-domain frame-length servo synchronizes to.
	signal vdp_frame_toggle_s	: std_logic := '0';
	-- Diagnostics for the two real line-loss paths a second-opinion review
	-- identified: capture_skip_s counts line starts where no slot was free
	-- (should now be rare/never, given effective_free_s and the 8-deep
	-- FIFO); fifo_drop_s counts a completed line finding the FIFO full
	-- (should now be structurally impossible - kept as an assurance check).
	signal dbg_skip_toggle_s	: std_logic := '0';
	signal dbg_drop_toggle_s	: std_logic := '0';

	-- VGA domain (I_CLK_VGA)
	signal display_slot_s		: std_logic_vector(2 downto 0) := "111";
	signal ack_toggle_s			: std_logic := '0';
	signal req_sync1_s			: std_logic := '0';
	signal req_sync2_s			: std_logic := '0';
	-- Bundled with ack_toggle_s the same way pub_slot_s/pub_line_s are
	-- bundled with req_toggle_s.
	signal freed_slot_s			: std_logic_vector(2 downto 0) := "111";
	signal locked_s				: std_logic := '0';
	-- True only within the actual v_start..v_end picture window.
	signal picture_active_s	: std_logic := '0';
	-- SIXTH revision (see header note): a descriptor is now accepted only
	-- if its line number matches the VGA row it would actually be shown
	-- at right now (target_line_s, derived straight from window_vcnt) -
	-- not an independently-advancing expected_line_s counter. This ties
	-- correctness to physical screen position directly: a single dropped
	-- line can now only ever blank/repeat *that one row*, never shift the
	-- whole picture's vertical mapping the way a free-running sequence
	-- counter could once it desynced from reality.
	signal target_line_s		: unsigned(7 downto 0);
	-- One-time boot phase acquisition: frequency lock (the dither) alone
	-- doesn't pin *where* VDP's row 0 lands relative to VGA's v_start -
	-- only average rate. The first time VDP's own frame boundary is
	-- observed, vcnt is forced to a fixed early value (comfortably before
	-- v_start, leaving margin for a few lines' worth of capture/publish/
	-- CDC/adopt latency) so row 0 always arrives with room to spare before
	-- the picture starts. Done only once, ever - after that the dither
	-- alone holds phase steady on its own.
	signal phase_acquired_s	: std_logic := '0';
	signal rephase_pending_s	: std_logic := '0';
	-- Purely observational now (no longer state-changing) - counts
	-- consecutive safe-boundary checks with no accepted line.
	signal no_match_count_s	: unsigned(9 downto 0) := (others => '0');
	signal dbg_reset_toggle_s	: std_logic := '0';

	-- Frame-length servo (see FOURTH revision header note): v_end_this_frame_s
	-- replaces the fixed v_end_count for the vcnt wrap comparison, chosen
	-- once per frame from {523,524,525} (=> 524/525/526-line frames).
	signal v_end_this_frame_s	: unsigned(9 downto 0) := to_unsigned(v_end_count, 10);
	-- Bresenham-style fractional accumulator driving the normal 553:1572
	-- (524-line : 525-line) dither ratio - see header math. 2125 needs 12
	-- bits (2048 < 2125 < 4096).
	signal dither_acc_s			: unsigned(11 downto 0) := (others => '0');
	-- VDP frame-boundary toggle, synchronized into this domain, plus the
	-- VGA line position it was last observed to land on.
	signal vdp_frame_sync1_s	: std_logic := '0';
	signal vdp_frame_sync2_s	: std_logic := '0';
	signal vdp_frame_seen_s	: std_logic := '0';
	signal vdp_phase_line_s	: std_logic_vector(9 downto 0) := (others => '0');

begin

	linebuf: entity work.dpram
	generic map (
		addr_width_g	=> 12,	-- 8 slots * up to 512 (I_HCNT concatenated, not tightly packed)
		data_width_g	=> 4
	)
	port map(
		clk_a_i		=> I_CLK,
		data_a_i		=> I_COLOR,
		addr_a_i		=> addr_wr,
		we_i			=> wren_gated,
		data_a_o		=> open,
		--
		clk_b_i		=> I_CLK_VGA,
		addr_b_i		=> addr_rd,
		data_b_o		=> pixel_out
	);

	-----------------------------------------------------------------------
	-- VDP domain combinational helpers - see header for why these must be
	-- latched into capture_active_s/capture_line_s rather than re-checked
	-- live for the write-enable throughout a line.
	-----------------------------------------------------------------------
	line_start_s        <= '1' when I_VCNT /= prev_vcnt_s else '0';
	next_capture_slot_s <= next_slot(current_slot_s);
	ack_landing_s       <= '1' when pub_pending_s = '1' and ack_sync2_s = req_toggle_s else '0';
	effective_free_s    <= '1' when slot_free_s(to_integer(unsigned(next_capture_slot_s))) = '1'
	                             or (ack_landing_s = '1' and freed_slot_s = next_capture_slot_s)
	                        else '0';
	start_capture_s     <= '1' when line_start_s = '1'
	                             and unsigned(I_VCNT) < to_unsigned(vc_max, 8)
	                             and effective_free_s = '1'
	                        else '0';

	-----------------------------------------------------------------------
	-- VDP domain: at each line start, push the just-finished line (if it
	-- was genuinely captured for its whole duration) into the descriptor
	-- FIFO, drain the FIFO head into the req/ack publish slot whenever
	-- it's free, and decide/latch whether the new line is capturable.
	-----------------------------------------------------------------------
	process (I_CLK)
		variable free_v        : std_logic_vector(7 downto 0);
		variable pending_v     : std_logic;
		variable wr_ptr_v      : unsigned(2 downto 0);
		variable rd_ptr_v      : unsigned(2 downto 0);
		variable count_v       : unsigned(3 downto 0);
	begin
		if rising_edge(I_CLK) then
			ack_sync1_s <= ack_toggle_s;
			ack_sync2_s <= ack_sync1_s;
			prev_vcnt_s <= I_VCNT;

			if I_VCNT /= prev_vcnt_s and unsigned(I_VCNT) = to_unsigned(0, 8) then
				vdp_frame_toggle_s <= not vdp_frame_toggle_s;
			end if;

			free_v   := slot_free_s;
			pending_v:= pub_pending_s;
			wr_ptr_v := fifo_wr_ptr_s;
			rd_ptr_v := fifo_rd_ptr_s;
			count_v  := fifo_count_s;

			-- Ack processing first, so a same-edge line-completion/FIFO-push
			-- below sees this cycle's up-to-date state.
			if ack_landing_s = '1' then
				free_v(to_integer(unsigned(freed_slot_s))) := '1';
				pending_v := '0';
			end if;

			-- Drain the FIFO head into the publish slot whenever it's free -
			-- BEFORE this cycle's line-completion push below, so this only
			-- ever reads fifo_slot_a/fifo_line_a entries written on a
			-- strictly earlier cycle (never the same edge - see FIFTH
			-- revision note 2: reading a signal array on the same edge it
			-- was written returns its pre-edge, stale contents, not what
			-- was just assigned - a classic read-after-write hazard that
			-- caused old descriptors to be republished on every empty-to-
			-- active FIFO transition).
			if pending_v = '0' and count_v /= to_unsigned(0, 4) then
				pub_slot_s   <= fifo_slot_a(to_integer(rd_ptr_v));
				pub_line_s   <= fifo_line_a(to_integer(rd_ptr_v));
				rd_ptr_v     := rd_ptr_v + 1;
				count_v      := count_v - 1;
				pending_v    := '1';
				req_toggle_s <= not req_toggle_s;
			end if;

			if line_start_s = '1' then
				-- Publish (or queue) the line that just finished, using the
				-- *latched* capture_active_s/capture_line_s from when it
				-- started - never the live slot_free_s state.
				if capture_active_s = '1' then
					if pending_v = '0' then
						-- Publish slot free (the drain above found
						-- nothing, i.e. the FIFO was truly empty) - publish
						-- directly, bypassing the FIFO array entirely so
						-- there's no same-cycle write+read of it at all.
						pub_slot_s   <= current_slot_s;
						pub_line_s   <= capture_line_s;
						pending_v    := '1';
						req_toggle_s <= not req_toggle_s;
					elsif count_v /= to_unsigned(8, 4) then
						fifo_slot_a(to_integer(wr_ptr_v)) <= current_slot_s;
						fifo_line_a(to_integer(wr_ptr_v)) <= capture_line_s;
						wr_ptr_v := wr_ptr_v + 1;
						count_v  := count_v + 1;
					else
						-- Sized to exactly match the 8 slots (1 outstanding
						-- publish + up to 7 queued), so this should never
						-- actually happen - dbg_drop_toggle_s is a pure
						-- assurance check.
						free_v(to_integer(unsigned(current_slot_s))) := '1';
						dbg_drop_toggle_s <= not dbg_drop_toggle_s;
					end if;
				end if;

				-- Latch this new line's capture decision for its entire
				-- duration, and claim the slot right now if capturing.
				current_slot_s   <= next_capture_slot_s;
				capture_line_s   <= I_VCNT;
				capture_active_s <= start_capture_s;
				if start_capture_s = '1' then
					free_v(to_integer(unsigned(next_capture_slot_s))) := '0';
				elsif unsigned(I_VCNT) < to_unsigned(vc_max, 8) then
					dbg_skip_toggle_s <= not dbg_skip_toggle_s;
				end if;
			end if;

			slot_free_s    <= free_v;
			pub_pending_s  <= pending_v;
			fifo_wr_ptr_s  <= wr_ptr_v;
			fifo_rd_ptr_s  <= rd_ptr_v;
			fifo_count_s   <= count_v;
		end if;
	end process;

	addr_wr		<= next_capture_slot_s & I_HCNT when line_start_s = '1' else current_slot_s & I_HCNT;
	wren_gated	<= '1' when (I_HCNT < hc_max) and
	               (  (line_start_s = '1' and start_capture_s = '1')
	               or (line_start_s = '0' and capture_active_s = '1' and I_VCNT = capture_line_s) )
	           else '0';

	-----------------------------------------------------------------------
	-- VGA domain: free-running output timing (unchanged/proven), plus the
	-- read-side half of the handshake at each source line's safe boundary.
	-----------------------------------------------------------------------
	process (I_CLK_VGA)
	begin
		if I_CLK_VGA'event and I_CLK_VGA = '1' then
			req_sync1_s <= req_toggle_s;
			req_sync2_s <= req_sync1_s;

			-- Purely diagnostic now (vdp_phase_line_s isn't used to drive
			-- rephasing anymore - see SEVENTH revision header note: the
			-- raw VDP frame-boundary toggle only says a wrap *happened*,
			-- not that the FIFO head is actually row 0 by the time we
			-- react to it - acquisition below instead holds the genuine
			-- pub_line_s=0 descriptor itself).
			vdp_frame_sync1_s <= vdp_frame_toggle_s;
			vdp_frame_sync2_s <= vdp_frame_sync1_s;
			if vdp_frame_sync2_s /= vdp_frame_seen_s then
				vdp_frame_seen_s <= vdp_frame_sync2_s;
				vdp_phase_line_s <= vcnt;
			end if;

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
					-- Safe swap point: first of the two doubled output lines
					-- for this source row (window_vcnt even) - never mid-row.
					-- Gated on the exact v_start..v_end picture range (not
					-- picture_active_s, which - per a second-opinion review -
					-- stays high through the whole bottom blanking interval
					-- too, consuming descriptors asymmetrically relative to
					-- the top blanking where it's low; as the dither shifts
					-- frame boundaries by up to a line, that asymmetry could
					-- mis-consume a descriptor before/after the row it was
					-- really meant for).
					if window_vcnt(0) = '0'
					   and unsigned(vcnt) >= to_unsigned(v_start, 10)
					   and unsigned(vcnt) <  to_unsigned(v_end, 10) then
						-- SIXTH revision (see header note): accept a descriptor
						-- only if its line number matches target_line_s (the
						-- row we're actually about to display, straight from
						-- window_vcnt) - not an independent sequence
						-- counter. A single dropped/mismatched line can now
						-- only ever repeat *that one row*; it can never pull
						-- the rest of the picture's vertical mapping out of
						-- alignment with physical screen position.
						-- no_match_count_s remains purely observational.
						if no_match_count_s /= to_unsigned(1023, 10) then
							no_match_count_s <= no_match_count_s + 1;
						end if;
						if no_match_count_s = to_unsigned(900, 10) then
							dbg_reset_toggle_s <= not dbg_reset_toggle_s;
						end if;
						if req_sync2_s /= ack_toggle_s then
							if phase_acquired_s = '0' then
								-- SEVENTH revision (see header note): boot phase
								-- acquisition. Inspect each arriving descriptor
								-- without consuming it. Anything before row 0
								-- is genuinely stale (published before VGA was
								-- watching) - ack it and drop it. The row-0
								-- descriptor itself is left UN-acked and held
								-- right here at the FIFO head; rephase_pending_s
								-- just requests the vcnt jump below. Because it
								-- stays un-acked, the unchanged exact-match
								-- logic below (once phase_acquired_s = '1')
								-- will see this very same descriptor again and
								-- adopt it normally the first time
								-- target_line_s reaches 0 - no separate
								-- adoption path needed here.
								if unsigned(pub_line_s) = to_unsigned(0, 8) then
									rephase_pending_s <= '1';
								else
									ack_toggle_s <= req_sync2_s;
									freed_slot_s <= pub_slot_s;
								end if;
							else
								ack_toggle_s <= req_sync2_s;
								if unsigned(pub_line_s) = target_line_s then
									freed_slot_s     <= display_slot_s;
									display_slot_s   <= pub_slot_s;
									locked_s         <= '1';
									no_match_count_s <= (others => '0');
								else
									-- wrong row - discard, keep showing the
									-- current display_slot_s for this row.
									freed_slot_s <= pub_slot_s;
								end if;
							end if;
						end if;
					end if;
				else
					window_hcnt <= window_hcnt + 1;
				end if;
			end if;
			if hcnt = h_sync_on then
				if rephase_pending_s = '1' then
					-- One-time boot phase acquisition (see SEVENTH revision
					-- header note): the genuine row-0 descriptor is already
					-- held un-acked at the FIFO head (set above). Just move
					-- VGA's own vcnt to v_start-1 so the very next tick
					-- lands it exactly on v_start/window_vcnt=0 - the point
					-- where the unchanged exact-match logic will naturally
					-- see target_line_s=0, match the held descriptor, and
					-- ack it for the first time. Never fires again after
					-- phase_acquired_s latches.
					vcnt              <= std_logic_vector(to_unsigned(v_start-1, 10));
					window_vcnt       <= (others => '0');
					picture_active_s  <= '0';
					locked_s          <= '0';
					rephase_pending_s <= '0';
					phase_acquired_s  <= '1';
				elsif unsigned(vcnt) = v_end_this_frame_s then
					vcnt <= (others => '0');
					picture_active_s <= '0';
					-- locked_s is deliberately NOT reset here - row 0 is
					-- only hunted for once (at power-up, via the phase
					-- acquisition above); after that target_line_s-based
					-- matching (see SIXTH revision) keeps every row
					-- correctly pinned to physical screen position on its
					-- own, frame after frame.

					-- Pick this next frame's length. Pure dither only now -
					-- the phase-window override (force 524/526 outside an
					-- 8-12 target) was removed: LED instrumentation showed
					-- it was firing on nearly every frame (a 60Hz toggle
					-- reads as *solid on* to the eye, not blinking - easy
					-- to misread as "rarely firing"), meaning almost every
					-- frame was being forced to 526 lines - a much bigger,
					-- one-directional rate error than the tiny original
					-- mismatch, and the likely cause of the "faster scroll"
					-- regression. The pure 553:1572 dither's average is
					-- mathematically exact (see header math), so it alone
					-- should hold phase steady without any override; the
					-- no_match_count_s safety net remains as a backstop for
					-- whatever initial phase offset exists at power-up.
					if dither_acc_s + to_unsigned(553, 12) >= to_unsigned(2125, 12) then
						dither_acc_s       <= dither_acc_s + to_unsigned(553, 12) - to_unsigned(2125, 12);
						v_end_this_frame_s <= to_unsigned(523, 10);
					else
						dither_acc_s       <= dither_acc_s + to_unsigned(553, 12);
						v_end_this_frame_s <= to_unsigned(524, 10);
					end if;
				else
					vcnt <= vcnt + 1;
					if vcnt = (v_start-1) then
						window_vcnt      <= (others => '0');
						picture_active_s <= '1';
					else
						window_vcnt <= window_vcnt + 1;
					end if;
				end if;
			end if;
		end if;
	end process;

	addr_rd			<= display_slot_s & window_hcnt(9 downto 1);
	target_line_s	<= unsigned(window_vcnt(8 downto 1));

	blank		<= '1' when (hcnt > h_pixels_across) or (vcnt > v_pixels_down) else '0';
	picture	<= '1' when (blank = '0') and (hcnt > h_start and hcnt < h_end) and (vcnt > v_start and vcnt < v_end) else '0';

	-- TEMP diagnostic overlay: a grid driven purely by raw hcnt/vcnt,
	-- independent of the line-buffer/slot/handshake logic above. Confirmed
	-- rock stable on real hardware already. Remove once done.
	grid_s	<= '1' when (hcnt(5 downto 0) = "000000") or (vcnt(5 downto 0) = "000000") else '0';

	O_HSYNC	<= '1' when (hcnt <= h_sync_on) or (hcnt > h_sync_off) else '0';
	O_VSYNC	<= '1' when (vcnt <= v_sync_on) or (vcnt > v_sync_off) else '0';
	O_COLOR	<= "1100" when (grid_s = '1' and blank = '0') else
	           pixel_out when (picture = '1' and locked_s = '1') else
	           (others => '0');
	O_BLANK	<= blank;
	-- (3)=capture skip (no free slot at line start), (2)=FIFO drop (should
	-- never fire now, 8 entries for 8 slots), (1)=reset_toggle (no_match_
	-- count_s hit 900, now purely observational), (0)=unused/spare.
	O_DBG		<= dbg_skip_toggle_s & dbg_drop_toggle_s & dbg_reset_toggle_s & '0';

end rtl;
