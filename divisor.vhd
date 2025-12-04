library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_unsigned.all;

entity Divisor is
  generic (
    N : integer := 24  -- número de bits
  );
  port (
    clk     : in  std_logic;
    div_clk : out std_logic
  );
end entity;

architecture Behavioral of Divisor is
  signal contador : std_logic_vector(N-1 downto 0) := (others => '0');
begin

  process(clk)
  begin
    if rising_edge(clk) then
      contador <= contador + 1;
    end if;
  end process;

  div_clk <= contador(N-1);

end Behavioral;