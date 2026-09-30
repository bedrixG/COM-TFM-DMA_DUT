library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity tb_simple_dut is
end entity tb_simple_dut;

architecture sim of tb_simple_dut is

  constant C_CLK_PERIOD : time := 10 ns;
  constant C_XOR_VALUE  : std_logic_vector(31 downto 0) := x"A5A5A5A5";

  signal aclk    : std_logic := '0';
  signal aresetn : std_logic := '0';

  signal s_axis_tvalid : std_logic := '0';
  signal s_axis_tready : std_logic;
  signal s_axis_tdata  : std_logic_vector(31 downto 0) := (others => '0');
  signal s_axis_tkeep  : std_logic_vector(3 downto 0)  := (others => '0');
  signal s_axis_tlast  : std_logic := '0';

  signal m_axis_tvalid : std_logic;
  signal m_axis_tready : std_logic := '1';
  signal m_axis_tdata  : std_logic_vector(31 downto 0);
  signal m_axis_tkeep  : std_logic_vector(3 downto 0);
  signal m_axis_tlast  : std_logic;

  -- simple running tally so the transcript ends with one clear number
  signal error_count : integer := 0;

begin

  dut_inst : entity work.simple_dut
    port map (
      aclk            => aclk,
      aresetn         => aresetn,
      s_axis_tvalid => s_axis_tvalid,
      s_axis_tready => s_axis_tready,
      s_axis_tdata  => s_axis_tdata,
      s_axis_tkeep  => s_axis_tkeep,
      s_axis_tlast  => s_axis_tlast,
      m_axis_tvalid => m_axis_tvalid,
      m_axis_tready => m_axis_tready,
      m_axis_tdata  => m_axis_tdata,
      m_axis_tkeep  => m_axis_tkeep,
      m_axis_tlast  => m_axis_tlast
    );

  p_clk : process
  begin
    while true loop
      aclk <= '0';
      wait for C_CLK_PERIOD / 2;
      aclk <= '1';
      wait for C_CLK_PERIOD / 2;
    end loop;
  end process;

  p_stim : process
    variable v_expected : std_logic_vector(31 downto 0);
  begin
    -----------------------------------------------------------------------
    -- TC1: Reset behaviour
    -----------------------------------------------------------------------
    aresetn       <= '0';
    s_axis_tvalid <= '0';
    s_axis_tdata  <= (others => '0');
    s_axis_tkeep  <= (others => '0');
    s_axis_tlast  <= '0';
    wait for 3 * C_CLK_PERIOD;

    assert m_axis_tvalid = '0'
      report "TC1 FAIL: m_axis_tvalid is not low during reset" severity error;
    if m_axis_tvalid /= '0' then error_count <= error_count + 1; end if;

    assert s_axis_tready = '1'
      report "TC1 FAIL: s_axis_tready should be high (DUT empty) even in reset" severity error;
    if s_axis_tready /= '1' then error_count <= error_count + 1; end if;

    aresetn <= '1';
    wait for C_CLK_PERIOD;

    -----------------------------------------------------------------------
    -- TC2: Continuous data, 8 words back-to-back, single frame
    -----------------------------------------------------------------------
    for i in 0 to 7 loop
      s_axis_tvalid <= '1';
      s_axis_tdata  <= std_logic_vector(to_unsigned(16 + i, 32));
      s_axis_tkeep  <= "1111";
      s_axis_tlast  <= '0';
      if i = 7 then
        s_axis_tlast <= '1';
      end if;

      wait until rising_edge(aclk) and s_axis_tready = '1';

      v_expected := std_logic_vector(to_unsigned(16 + i, 32)) xor C_XOR_VALUE;

      wait until falling_edge(aclk);
      assert m_axis_tvalid = '1'
        report "TC2 FAIL: m_axis_tvalid not active on beat " & integer'image(i) severity error;
      if m_axis_tvalid /= '1' then error_count <= error_count + 1; end if;

      assert m_axis_tdata = v_expected
        report "TC2 FAIL: wrong data on beat " & integer'image(i) severity error;
      if m_axis_tdata /= v_expected then error_count <= error_count + 1; end if;

      assert m_axis_tkeep = "1111"
        report "TC2 FAIL: wrong tkeep on beat " & integer'image(i) severity error;
      if m_axis_tkeep /= "1111" then error_count <= error_count + 1; end if;

      if i = 7 then
        assert m_axis_tlast = '1' report "TC2 FAIL: TLAST not active on last beat" severity error;
        if m_axis_tlast /= '1' then error_count <= error_count + 1; end if;
      else
        assert m_axis_tlast = '0' report "TC2 FAIL: TLAST active on non-last beat " & integer'image(i) severity error;
        if m_axis_tlast /= '0' then error_count <= error_count + 1; end if;
      end if;

      wait until rising_edge(aclk);
    end loop;

    s_axis_tvalid <= '0';
    wait for C_CLK_PERIOD;

    -----------------------------------------------------------------------
    -- TC3a: Multiple frames - Frame A: 1 beat, partial TKEEP
    -----------------------------------------------------------------------
    s_axis_tvalid <= '1';
    s_axis_tdata  <= std_logic_vector(to_unsigned(100, 32));
    s_axis_tkeep  <= "0001";
    s_axis_tlast  <= '1';
    wait until rising_edge(aclk) and s_axis_tready = '1';

    v_expected := std_logic_vector(to_unsigned(100, 32)) xor C_XOR_VALUE;
    wait until falling_edge(aclk);
    assert m_axis_tdata = v_expected report "TC3a FAIL: Frame A data wrong" severity error;
    if m_axis_tdata /= v_expected then error_count <= error_count + 1; end if;
    assert m_axis_tkeep = "0001" report "TC3a FAIL: Frame A tkeep wrong" severity error;
    if m_axis_tkeep /= "0001" then error_count <= error_count + 1; end if;
    assert m_axis_tlast = '1' report "TC3a FAIL: Frame A tlast wrong" severity error;
    if m_axis_tlast /= '1' then error_count <= error_count + 1; end if;

    s_axis_tvalid <= '0';
    wait until rising_edge(aclk);

    -----------------------------------------------------------------------
    -- TC3b: Multiple frames - Frame B: 3 beats, last-beat TKEEP = "0111"
    -----------------------------------------------------------------------
    for i in 0 to 2 loop
      s_axis_tvalid <= '1';
      s_axis_tdata  <= std_logic_vector(to_unsigned(200 + i, 32));
      if i = 2 then
        s_axis_tkeep <= "0111";
        s_axis_tlast <= '1';
      else
        s_axis_tkeep <= "1111";
        s_axis_tlast <= '0';
      end if;

      wait until rising_edge(aclk) and s_axis_tready = '1';
      v_expected := std_logic_vector(to_unsigned(200 + i, 32)) xor C_XOR_VALUE;
      wait until falling_edge(aclk);

      assert m_axis_tdata = v_expected
        report "TC3b FAIL: Frame B data wrong on beat " & integer'image(i) severity error;
      if m_axis_tdata /= v_expected then error_count <= error_count + 1; end if;

      if i = 2 then
        assert m_axis_tkeep = "0111" report "TC3b FAIL: Frame B tkeep wrong" severity error;
        if m_axis_tkeep /= "0111" then error_count <= error_count + 1; end if;
        assert m_axis_tlast = '1' report "TC3b FAIL: Frame B tlast wrong" severity error;
        if m_axis_tlast /= '1' then error_count <= error_count + 1; end if;
      else
        assert m_axis_tlast = '0' report "TC3b FAIL: Frame B tlast active early" severity error;
        if m_axis_tlast /= '0' then error_count <= error_count + 1; end if;
      end if;

      wait until rising_edge(aclk);
    end loop;

    s_axis_tvalid <= '0';
    wait for C_CLK_PERIOD;

    -----------------------------------------------------------------------
    -- TC4: Backpressure - hold m_axis_tready low while a beat is pending
    -----------------------------------------------------------------------
    -- Beat 1: DUT is empty, so it is accepted immediately
    s_axis_tvalid <= '1';
    s_axis_tdata  <= std_logic_vector(to_unsigned(300, 32));
    s_axis_tkeep  <= "1111";
    s_axis_tlast  <= '0';
    wait until rising_edge(aclk) and s_axis_tready = '1';

    -- Choke the downstream BEFORE presenting beat 2, so there is no
    -- ambiguity about which cycle the stall starts on.
    m_axis_tready <= '0';
    s_axis_tdata  <= std_logic_vector(to_unsigned(301, 32));
    s_axis_tlast  <= '1';
    wait until rising_edge(aclk);

    -- Beat 1 is now parked in the DUT's output register; downstream isn't
    -- consuming it, so the DUT must also stall the input side now:
    assert s_axis_tready = '0'
      report "TC4 FAIL: s_axis_tready should drop when full and m_axis_tready is low" severity error;
    if s_axis_tready /= '0' then error_count <= error_count + 1; end if;

    v_expected := std_logic_vector(to_unsigned(300, 32)) xor C_XOR_VALUE;
    assert m_axis_tvalid = '1' and m_axis_tdata = v_expected
      report "TC4 FAIL: beat 1 did not stay correctly parked at the output" severity error;
    if not (m_axis_tvalid = '1' and m_axis_tdata = v_expected) then
      error_count <= error_count + 1;
    end if;

    -- Hold the stall a couple more cycles: nothing should change
    wait until rising_edge(aclk);
    wait until rising_edge(aclk);
    assert m_axis_tdata = v_expected
      report "TC4 FAIL: stalled output drifted during the stall" severity error;
    if m_axis_tdata /= v_expected then error_count <= error_count + 1; end if;

    -- Release backpressure and let beat 2 go through
    m_axis_tready <= '1';
    wait until rising_edge(aclk) and s_axis_tready = '1';

    v_expected := std_logic_vector(to_unsigned(301, 32)) xor C_XOR_VALUE;
    wait until falling_edge(aclk);
    assert m_axis_tdata = v_expected
      report "TC4 FAIL: beat 2 data wrong after releasing backpressure" severity error;
    if m_axis_tdata /= v_expected then error_count <= error_count + 1; end if;
    assert m_axis_tlast = '1'
      report "TC4 FAIL: beat 2 tlast wrong" severity error;
    if m_axis_tlast /= '1' then error_count <= error_count + 1; end if;

    s_axis_tvalid <= '0';
    wait until rising_edge(aclk);

    -----------------------------------------------------------------------
    report "----------------------------------------";
    if error_count = 0 then
      report "TESTBENCH OK - all checks passed" severity note;
    else
      report "TESTBENCH FAILED - " & integer'image(error_count) & " check(s) failed" severity error;
    end if;
    wait;
  end process;

end architecture sim;