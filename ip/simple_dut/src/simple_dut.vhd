library ieee;
use ieee.std_logic_1164.all;

entity simple_dut is
  generic (
    C_DATA_WIDTH : integer := 32   -- must be a multiple of 32 (32, 64, 128, 256, 512)
  );
  port (
    aclk    : in  std_logic;
    aresetn : in  std_logic;

    -- S_AXIS
    s_axis_tvalid : in  std_logic;
    s_axis_tready : out std_logic;
    s_axis_tdata  : in  std_logic_vector(C_DATA_WIDTH-1 downto 0);
    s_axis_tkeep  : in  std_logic_vector(C_DATA_WIDTH/8-1 downto 0);
    s_axis_tlast  : in  std_logic;

    -- M_AXIS
    m_axis_tvalid : out std_logic;
    m_axis_tready : in  std_logic;
    m_axis_tdata  : out std_logic_vector(C_DATA_WIDTH-1 downto 0);
    m_axis_tkeep  : out std_logic_vector(C_DATA_WIDTH/8-1 downto 0);
    m_axis_tlast  : out std_logic
  );
end entity simple_dut;

architecture rtl of simple_dut is

  constant C_XOR_PATTERN : std_logic_vector(31 downto 0) := x"A5A5A5A5";

  -- Repeat the 32-bit pattern in every 32-bit lane of the bus
  function replicate32(pattern : std_logic_vector(31 downto 0); width : integer)
    return std_logic_vector is
    variable r : std_logic_vector(width-1 downto 0);
  begin
    for i in 0 to width/32 - 1 loop
      r(32*i+31 downto 32*i) := pattern;
    end loop;
    return r;
  end function;

  constant C_XOR_MASK : std_logic_vector(C_DATA_WIDTH-1 downto 0) :=
    replicate32(C_XOR_PATTERN, C_DATA_WIDTH);

  signal r_tdata  : std_logic_vector(C_DATA_WIDTH-1 downto 0)   := (others => '0');
  signal r_tkeep  : std_logic_vector(C_DATA_WIDTH/8-1 downto 0) := (others => '0');
  signal r_tlast  : std_logic := '0';
  signal r_tvalid : std_logic := '0';
  signal s_ready  : std_logic;

begin

  assert (C_DATA_WIDTH mod 32) = 0
    report "simple_dut: C_DATA_WIDTH must be a multiple of 32" severity failure;

  -- Accept data if the output register is empty OR downstream is consuming it
  s_ready         <= (not r_tvalid) or m_axis_tready;
  s_axis_tready   <= s_ready;

  process(aclk)
  begin
    if rising_edge(aclk) then
      if aresetn = '0' then
        r_tvalid <= '0';
        r_tlast  <= '0';
        r_tkeep  <= (others => '0');
        r_tdata  <= (others => '0');
      else
        if s_ready = '1' then
          r_tvalid <= s_axis_tvalid;
          if s_axis_tvalid = '1' then
            r_tdata <= s_axis_tdata xor C_XOR_MASK;
            r_tkeep <= s_axis_tkeep;
            r_tlast <= s_axis_tlast;
          end if;
        end if;
      end if;
    end if;
  end process;

  m_axis_tdata  <= r_tdata;
  m_axis_tkeep  <= r_tkeep;
  m_axis_tlast  <= r_tlast;
  m_axis_tvalid <= r_tvalid;

end architecture rtl;