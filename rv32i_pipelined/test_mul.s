-- Test file for Multiply Instructions

RISC-V assembly

addi x1,x0, -0xFF
addi x2,x0, 0xFF
mul x3,x1,x2
mulh x9,x1,x2
mul x4,x3,x3
mulh x6,x3,x3
mulhsu x7,x3,x3
mulhu x8,x3,x3

=====
RV32I machine code

 X"93", X"00", X"10", X"F0", X"13", X"01", X"F0", X"0F", 
 X"B3", X"81", X"20", X"02", X"B3", X"94", X"20", X"02",
 X"33", X"82", X"31", X"02", X"33", X"93", X"31", X"02",
 X"B3", X"A3", X"31", X"02", X"33", X"B4", X"31", X"02",

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
        X"93", X"00", X"10", X"F0", X"13", X"01", X"F0", X"0F", 
        X"B3", X"81", X"20", X"02", X"B3", X"94", X"20", X"02",
        X"33", X"82", X"31", X"02", X"33", X"93", X"31", X"02",
        X"B3", X"A3", X"31", X"02", X"33", X"B4", X"31", X"02",
        others => (others=>'0'));
        variable word_addr : std_logic_vector (ADDRLEN-1 downto 0) := (others=>'0');
    begin
        word_addr := addr(ADDRLEN-1 downto 2) & "00";
        dataout <= rom_array(to_integer(unsigned(word_addr))+3) & rom_array(to_integer(unsigned(word_addr))+2) & rom_array(to_integer(unsigned(word_addr))+1) & rom_array(to_integer(unsigned(word_addr)));
    end process;

end architecture;