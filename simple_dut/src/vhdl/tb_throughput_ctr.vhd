library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

-------------------------------------------------------------------------------------------------
-- tb_throughput_ctr.vhd   (plain VHDL-93 constructs, same style as tb_simple_dut)
--
-- Needs in the simulation sources: CTR_N.vhd (the lab's counter, unmodified), throughput_ctr.vhd,
-- and this file.
--
-- Every stimulus change happens 1 ns AFTER a rising edge, so the DUT always samples stable
-- values. 'step' drives one clock cycle and, at that edge, updates an independent SHADOW model
-- (written from the definitions, not copied from the RTL). The expected numbers of the
-- hand-checked scenarios were also derived with a Python model (model_throughput_ctr.py).
--   S1  3 idle + 4 back-to-back beats, TLAST on the 4th        (hand: 7 / 3 / 4 / first 3)
--   S2  counters stay frozen after done
--   S3  enable low clears everything
--   S4  stalls and gaps before and between beats               (hand: 7 / 4 / 3 / first 2)
--   S4b TLAST without a transfer must be ignored               (hand: 4 / 2 / 2 / first 2)
--   S5  ~3000 pseudo-random cycles against the shadow model
--   S6  asynchronous reset in the middle of a measurement
--   S7  full scale: a 4-bit instance wraps at 15 and must raise overflow
-------------------------------------------------------------------------------------------------
entity tb_throughput_ctr is
end entity tb_throughput_ctr;

architecture sim of tb_throughput_ctr is

  constant C_CLK_PERIOD : time := 5 ns;   -- 200 MHz

  signal clk       : std_logic := '0';
  signal aresetn   : std_logic := '0';
  signal enable    : std_logic := '0';

  signal vld : std_logic := '0';
  signal rdy : std_logic := '0';
  signal lst : std_logic := '0';

  -- 31-bit instance (the real configuration)
  signal cycles, active, holes, first : std_logic_vector(30 downto 0);
  signal status               : std_logic_vector(2 downto 0);

  -- 4-bit instance, only used to reach full scale quickly
  signal s_cycles, s_active, s_holes, s_first : std_logic_vector(3 downto 0);
  signal s_status                   : std_logic_vector(2 downto 0);

begin

  -- holes are not an output any more: derived exactly as software will (CYCLES - ACTIVE)
  holes   <= std_logic_vector(unsigned(cycles)   - unsigned(active));
  s_holes <= std_logic_vector(unsigned(s_cycles) - unsigned(s_active));

  dut : entity work.throughput_ctr
    generic map (g_NBITS => 31)
    port map (
      clk_i => clk, aresetn_i => aresetn, enable_i => enable,
      tvalid_i => vld, tready_i => rdy, tlast_i => lst,
      cycles_o => cycles, active_o => active, first_o => first, status_o => status
    );

  dut_small : entity work.throughput_ctr
    generic map (g_NBITS => 4)
    port map (
      clk_i => clk, aresetn_i => aresetn, enable_i => enable,
      tvalid_i => vld, tready_i => rdy, tlast_i => lst,
      cycles_o => s_cycles, active_o => s_active, first_o => s_first, status_o => s_status
    );

  p_clk : process
  begin
    while true loop
      clk <= '0';
      wait for C_CLK_PERIOD / 2;
      clk <= '1';
      wait for C_CLK_PERIOD / 2;
    end loop;
  end process;

  p_stim : process
    -- shadow model of the 31-bit instance
    variable e_en   : boolean := false;
    variable e_cyc  : integer := 0;
    variable e_hol  : integer := 0;
    variable e_bea  : integer := 0;
    variable e_fir  : integer := 0;
    variable e_seen : boolean := false;
    variable e_done : boolean := false;

    variable err     : integer := 0;
    variable lfsr    : unsigned(15 downto 0) := x"ACE1";
    variable fb      : std_logic;
    variable v, r, l : std_logic;
    variable cyc_run : integer;

    procedure shadow_clear is
    begin
      e_cyc := 0; e_hol := 0; e_bea := 0; e_fir := 0;
      e_seen := false; e_done := false;
    end procedure;

    -- one clock cycle with the given probe values
    procedure step (sv : in std_logic; sr : in std_logic; sl : in std_logic) is
    begin
      vld <= sv; rdy <= sr; lst <= sl;
      wait until rising_edge(clk);            -- the DUT samples the values here
      if (not e_en) or aresetn = '0' then
        shadow_clear;
      elsif not e_done then
        if sv = '1' and sr = '1' then
          e_bea := e_bea + 1;
          if not e_seen then
            e_fir  := e_cyc;                  -- cycles elapsed before the first transfer
            e_seen := true;
          end if;
          if sl = '1' then
            e_done := true;
          end if;
        else
          e_hol := e_hol + 1;                 -- not (valid and ready)
        end if;
        e_cyc := e_cyc + 1;
      end if;
      wait for 1 ns;
    end procedure;

    procedure set_enable (b : in boolean) is
    begin
      e_en := b;
      if b then enable <= '1'; else enable <= '0'; end if;
    end procedure;

    procedure expect_int (tag : in string; got : in integer; exp : in integer) is
    begin
      if got /= exp then
        report tag & ": got " & integer'image(got) & ", expected " & integer'image(exp)
          severity error;
        err := err + 1;
      end if;
    end procedure;

    procedure expect_sl (tag : in string; got : in std_logic; exp : in std_logic) is
    begin
      if got /= exp then
        report tag & ": got " & std_logic'image(got) & ", expected " & std_logic'image(exp)
          severity error;
        err := err + 1;
      end if;
    end procedure;

    -- compare every output of the 31-bit instance against the shadow model
    procedure check_all (tag : in string) is
    begin
      expect_int(tag & " cycles", to_integer(unsigned(cycles)), e_cyc);
      expect_int(tag & " holes",  to_integer(unsigned(holes)),  e_hol);
      expect_int(tag & " first",  to_integer(unsigned(first)),  e_fir);
      expect_int(tag & " active = beats", to_integer(unsigned(active)), e_bea);
      if e_done then
        expect_sl(tag & " done", status(1), '1');
        expect_sl(tag & " running", status(0), '0');
      else
        expect_sl(tag & " done", status(1), '0');
      end if;
      expect_sl(tag & " overflow", status(2), '0');
    end procedure;

    procedure lfsr_next is
    begin
      fb   := lfsr(15) xor lfsr(13) xor lfsr(12) xor lfsr(10);
      lfsr := lfsr(14 downto 0) & fb;
    end procedure;

  begin
    -----------------------------------------------------------------------
    -- Reset
    -----------------------------------------------------------------------
    report "=== Reset ===";
    aresetn <= '0'; enable <= '0'; e_en := false;
    step('0','0','0'); step('0','0','0'); step('0','0','0');
    aresetn <= '1';
    step('0','0','0');
    check_all("reset");
    expect_sl("reset running", status(0), '0');

    -----------------------------------------------------------------------
    -- S1: 3 idle + 4 beats, TLAST on the 4th  -> cycles 7, holes 3, beats 4, first 3
    -----------------------------------------------------------------------
    report "=== S1: 3 idle + 4 back-to-back beats, TLAST on 4th ===";
    set_enable(true);
    step('0','0','0'); step('0','0','0'); step('0','0','0');
    expect_sl("S1 running during idle", status(0), '1');
    step('1','1','0'); step('1','1','0'); step('1','1','0');
    expect_sl("S1 running before TLAST", status(0), '1');
    expect_sl("S1 not done before TLAST", status(1), '0');
    step('1','1','1');
    check_all("S1");
    expect_int("S1 cycles (hand)", to_integer(unsigned(cycles)), 7);
    expect_int("S1 holes (hand)",  to_integer(unsigned(holes)),  3);
    expect_int("S1 first (hand)",  to_integer(unsigned(first)),  3);
    expect_sl("S1 done (hand)",    status(1), '1');
    expect_sl("S1 running (hand)", status(0), '0');

    -----------------------------------------------------------------------
    -- S2: frozen after done, whatever the stream does
    -----------------------------------------------------------------------
    report "=== S2: counters frozen after done ===";
    for i in 1 to 10 loop
      lfsr_next;
      step(lfsr(0), lfsr(1), lfsr(2));
    end loop;
    check_all("S2");
    expect_int("S2 cycles still 7 (hand)", to_integer(unsigned(cycles)), 7);

    -----------------------------------------------------------------------
    -- S3: enable low clears everything
    -----------------------------------------------------------------------
    report "=== S3: enable low clears ===";
    set_enable(false);
    step('0','0','0');
    check_all("S3");
    expect_int("S3 cycles zero (hand)", to_integer(unsigned(cycles)), 0);
    expect_int("S3 holes zero (hand)",  to_integer(unsigned(holes)),  0);
    expect_sl("S3 done zero (hand)",    status(1), '0');
    expect_sl("S3 running zero (hand)", status(0), '0');

    -----------------------------------------------------------------------
    -- S4: stalls and gaps -> cycles 7, holes 4, beats 3, first 2
    --     idx0 stall, idx1 stall, idx2 BEAT, idx3 idle, idx4 BEAT, idx5 stall, idx6 BEAT+TLAST
    -----------------------------------------------------------------------
    report "=== S4: stalls and gaps ===";
    set_enable(true);
    step('1','0','0'); step('1','0','0'); step('1','1','0');
    step('0','0','0'); step('1','1','0'); step('1','0','0'); step('1','1','1');
    check_all("S4");
    expect_int("S4 cycles (hand)", to_integer(unsigned(cycles)), 7);
    expect_int("S4 holes (hand)",  to_integer(unsigned(holes)),  4);
    expect_int("S4 first (hand)",  to_integer(unsigned(first)),  2);

    -----------------------------------------------------------------------
    -- S4b: TLAST without a transfer must be ignored
    --      (valid, no ready, TLAST) (ready, no valid, TLAST) BEAT BEAT+TLAST
    --      -> cycles 4, holes 2, beats 2, first 2
    -----------------------------------------------------------------------
    report "=== S4b: TLAST without a transfer is ignored ===";
    set_enable(false);
    step('0','0','0');
    set_enable(true);
    step('1','0','1'); step('0','1','1');
    expect_sl("S4b not done after spurious TLAST", status(1), '0');
    step('1','1','0'); step('1','1','1');
    check_all("S4b");
    expect_int("S4b cycles (hand)", to_integer(unsigned(cycles)), 4);
    expect_int("S4b holes (hand)",  to_integer(unsigned(holes)),  2);
    expect_int("S4b first (hand)",  to_integer(unsigned(first)),  2);

    -----------------------------------------------------------------------
    -- S5: pseudo-random run against the shadow model. TLAST is also asserted without a
    --     transfer (must be ignored); the real TLAST comes after at least 300 cycles.
    -----------------------------------------------------------------------
    report "=== S5: random run ===";
    set_enable(false);
    step('0','0','0');
    set_enable(true);
    cyc_run := 0;
    for i in 1 to 3000 loop
      lfsr_next;
      v := lfsr(0);
      r := lfsr(1) or lfsr(2);
      if cyc_run >= 300 and v = '1' and r = '1'
         and lfsr(6) = '0' and lfsr(5) = '0' and lfsr(4) = '0' then
        l := '1';                              -- the real end of frame
      elsif v = '0' or r = '0' then
        l := lfsr(7) and lfsr(8);              -- spurious TLAST, no transfer: must be ignored
      else
        l := '0';
      end if;
      step(v, r, l);
      cyc_run := cyc_run + 1;
    end loop;
    if not e_done then
      report "S5: the random stimulus never produced a TLAST transfer" severity error;
      err := err + 1;
    end if;
    check_all("S5");

    -----------------------------------------------------------------------
    -- S6: asynchronous reset in the middle of a measurement
    -----------------------------------------------------------------------
    report "=== S6: reset mid-measurement ===";
    set_enable(false);
    step('0','0','0');
    set_enable(true);
    for i in 1 to 5 loop step('0','0','0'); end loop;
    expect_int("S6 cycles before reset (hand)", to_integer(unsigned(cycles)), 5);
    aresetn <= '0';
    step('0','0','0');
    check_all("S6 in reset");
    expect_int("S6 cycles cleared by reset (hand)", to_integer(unsigned(cycles)), 0);
    aresetn <= '1';
    step('0','0','0');                          -- enable is still '1': a new window starts
    check_all("S6 restarted");
    expect_int("S6 cycles restarted (hand)", to_integer(unsigned(cycles)), 1);

    -----------------------------------------------------------------------
    -- S7: full scale. 20 idle cycles: the 4-bit instance (full scale 15) wraps to 0 at the 16th
    --     edge, so after 20 edges it shows 4 and must have raised overflow; the 31-bit instance
    --     simply counts 20.
    -----------------------------------------------------------------------
    report "=== S7: full scale / overflow ===";
    set_enable(false);
    step('0','0','0');
    set_enable(true);
    for i in 1 to 20 loop step('0','0','0'); end loop;
    check_all("S7 big");
    expect_int("S7 big cycles (hand)", to_integer(unsigned(cycles)), 20);
    expect_int("S7 small cycles after wrap (hand)", to_integer(unsigned(s_cycles)), 4);
    expect_int("S7 small holes after wrap (hand)",  to_integer(unsigned(s_holes)),  4);
    expect_sl("S7 small overflow (hand)", s_status(2), '1');
    set_enable(false);
    step('0','0','0');
    expect_sl("S7 small overflow cleared (hand)", s_status(2), '0');
    expect_int("S7 small cycles cleared (hand)", to_integer(unsigned(s_cycles)), 0);

    -----------------------------------------------------------------------
    report "----------------------------------------";
    if err = 0 then
      report "TESTBENCH OK - all checks passed" severity note;
    else
      report "TESTBENCH FAILED - " & integer'image(err) & " check(s) failed" severity error;
    end if;
    wait;
  end process;

end architecture sim;