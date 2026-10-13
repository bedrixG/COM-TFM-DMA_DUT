library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
-------------------------------------------------------------------------------------------------
entity throughput_ctr is
  generic (
    g_NBITS : integer range 1 to 31 := 31
  );
  port (
    clk_i     : in  std_logic;
    aresetn_i : in  std_logic;
    enable_i  : in  std_logic;

    -- AXI4-Stream link
    tvalid_i  : in  std_logic;
    tready_i  : in  std_logic;
    tlast_i   : in  std_logic;

    -- results
    cycles_o  : out std_logic_vector(g_NBITS - 1 downto 0);
    active_o  : out std_logic_vector(g_NBITS - 1 downto 0);
    first_o   : out std_logic_vector(g_NBITS - 1 downto 0);
    status_o  : out std_logic_vector(2 downto 0)
  );
end entity throughput_ctr;
------------------------------------------------------------------
architecture str of throughput_ctr is
------------------------------------------------------------------
  component CTR_N
    generic (
      g_NBITS    : positive := 8;
      g_MAXVAL   : positive := 8;
      g_RESETVAL : natural  := 0
    );
    port (
      clk_i     : in  std_logic;
      reset_a_i : in  std_logic;
      load_i    : in  std_logic;
      en_i      : in  std_logic;
      Din_i     : in  std_logic_vector(g_NBITS - 1 downto 0);
      up_nDwn_i : in  std_logic;
      Dout_o    : out std_logic_vector(g_NBITS - 1 downto 0);
      endflag_o : out std_logic
    );
  end component CTR_N;
------------------------------------------------------------------
  -- full scale = 2**g_NBITS - 1, written so that no intermediate value exceeds integer'high
  constant C_MAXVAL : positive := (2 ** (g_NBITS - 1) - 1) + 2 ** (g_NBITS - 1);
  constant C_ZERO   : std_logic_vector(g_NBITS - 1 downto 0) := (others => '0');

  signal reset_a      : std_logic;
  signal xfer         : std_logic;   -- a transfer happens on this edge
  signal clear        : std_logic;   -- window closed: load zero
  signal en_cyc       : std_logic;
  signal en_act       : std_logic;

  signal cycles_s     : std_logic_vector(g_NBITS - 1 downto 0);
  signal active_s     : std_logic_vector(g_NBITS - 1 downto 0);
  signal endflag_cyc  : std_logic;
  signal endflag_act  : std_logic;

  signal done_r       : std_logic := '0';
  signal seen_r       : std_logic := '0';
  signal ovf_r        : std_logic := '0';
  signal first_r      : std_logic_vector(g_NBITS - 1 downto 0) := (others => '0');
------------------------------------------------------------------
begin

  reset_a <= not aresetn_i;

  xfer  <= tvalid_i and tready_i;
  clear <= not enable_i;

  -- CTR_N only does anything while en_i = '1', and "load" needs en_i = '1' too:
  --   window closed (clear = 1) : en = 1, load = 1  -> counter holds 0
  --   window open               : CYCLES counts every edge; ACTIVE counts when (valid and ready)
  --   after TLAST (done_r = 1)  : en = 0            -> both counters freeze
  en_cyc  <= clear or (not done_r);
  en_act  <= clear or ((not done_r) and xfer);

  CYCLE_CTR : CTR_N
    generic map (
      g_NBITS    => g_NBITS,
      g_MAXVAL   => C_MAXVAL,
      g_RESETVAL => 0
    )
    port map (
      clk_i     => clk_i,
      reset_a_i => reset_a,
      load_i    => clear,
      en_i      => en_cyc,
      Din_i     => C_ZERO,
      up_nDwn_i => '1',
      Dout_o    => cycles_s,
      endflag_o => endflag_cyc
    );

  ACTIVE_CTR : CTR_N
    generic map (
      g_NBITS    => g_NBITS,
      g_MAXVAL   => C_MAXVAL,
      g_RESETVAL => 0
    )
    port map (
      clk_i     => clk_i,
      reset_a_i => reset_a,
      load_i    => clear,
      en_i      => en_act,
      Din_i     => C_ZERO,
      up_nDwn_i => '1',
      Dout_o    => active_s,
      endflag_o => endflag_act
    );

  -- window state: first-beat latch, end of window at the TLAST transfer, sticky overflow
  process (clk_i)
  begin
    if rising_edge(clk_i) then
      if aresetn_i = '0' or enable_i = '0' then
        done_r  <= '0';
        seen_r  <= '0';
        ovf_r   <= '0';
        first_r <= (others => '0');
      else
        if done_r = '0' and xfer = '1' then
          if seen_r = '0' then
            first_r <= cycles_s;          -- cycles elapsed BEFORE this first transfer
            seen_r  <= '1';
          end if;
          if tlast_i = '1' then           -- TLAST only counts together with a transfer
            done_r <= '1';
          end if;
        end if;
        if endflag_cyc = '1' or endflag_act = '1' then
          ovf_r <= '1';
        end if;
      end if;
    end if;
  end process;

  cycles_o <= cycles_s;
  active_o <= active_s;
  first_o  <= first_r;

  status_o(0) <= enable_i and (not done_r);   -- running
  status_o(1) <= done_r;                       -- done
  status_o(2) <= ovf_r;                        -- overflow

end architecture str;