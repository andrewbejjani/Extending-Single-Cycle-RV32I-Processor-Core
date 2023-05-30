-- Branch Adder
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.archer_pkg.all;

entity branch_adder is
    port (
        imm : in std_logic_vector (XLEN-1 downto 0);
        pc : in std_logic_vector (XLEN-1 downto 0);
        result : out std_logic_vector (XLEN-1 downto 0)
    );
end branch_adder;

architecture rtl of branch_adder is
begin
    result <= std_logic_vector(signed(pc) + signed(imm));
end architecture;
