-- Test file for Divide and Remainder Instructions

RISC-V assembly

addi x1,x0,6
addi x2,x0,4
divu x3,x1,x2
remu x4,x1,x2
div x5,x4,x0
divu x6,x4,x0
rem x7,x4,x0
remu x8,x4,x0
addi x9,x0,-1
slli x10,x9,31
div x11,x10,x9
divu x12,x10,x9
rem x13,x10,x9
remu x14,x10,x9

=====
RV32I machine code

 X"93", X"00", X"60", X"00", X"13", X"01", X"40", X"00",
 X"b3", X"d1", X"20", X"02", X"33", X"f2", X"20", X"02",
 X"b3", X"42", X"02", X"02", X"33", X"53", X"02", X"02",
 X"b3", X"63", X"02", X"02", X"33", X"74", X"02", X"02",
 X"93", X"04", X"f0", X"ff", X"13", X"95", X"f4", X"01",
 X"b3", X"45", X"95", X"02", X"33", X"56", X"95", X"02",
 X"b3", X"66", X"95", X"02", X"33", X"77", X"95", X"02"

======
ROM file

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.archer_pkg.all;

entity rom is
    port (
        addr : in std_logic_vector (ADDRLEN-1 downto 0);
        dataout : out std_logic_vector (XLEN-1 downto 0)
    );
end rom;

architecture rtl of rom is
    type memory is array (0 to 2**(ADDRLEN)-1) of std_logic_vector (7 downto 0); -- memory is byte addressable
    
begin

    process (addr) is
        variable rom_array : memory := (
            X"93", X"00", X"60", X"00", X"13", X"01", X"40", X"00",
            X"b3", X"d1", X"20", X"02", X"33", X"f2", X"20", X"02",
            X"b3", X"42", X"02", X"02", X"33", X"53", X"02", X"02",
            X"b3", X"63", X"02", X"02", X"33", X"74", X"02", X"02",
            X"93", X"04", X"f0", X"ff", X"13", X"95", X"f4", X"01",
            X"b3", X"45", X"95", X"02", X"33", X"56", X"95", X"02",
            X"b3", X"66", X"95", X"02", X"33", X"77", X"95", X"02",
        others => (others=>'0'));
        variable word_addr : std_logic_vector (ADDRLEN-1 downto 0) := (others=>'0');
    begin
        word_addr := addr(ADDRLEN-1 downto 2) & "00";
        dataout <= rom_array(to_integer(unsigned(word_addr))+3) & rom_array(to_integer(unsigned(word_addr))+2) & rom_array(to_integer(unsigned(word_addr))+1) & rom_array(to_integer(unsigned(word_addr)));
    end process;

end architecture;