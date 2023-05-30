-- PC adder

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.archer_pkg.all;

entity add4 is
    port (
        datain : in std_logic_vector (XLEN-1 downto 0);
        result : out std_logic_vector (XLEN-1 downto 0)
    );
end add4;

architecture rtl of add4 is
begin
    result <= std_logic_vector(to_unsigned(to_integer(unsigned(datain)) + 4, XLEN));
end architecture;
