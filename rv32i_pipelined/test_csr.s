-- Test file for CSR Instructions

RISC-V assembly

.globl  main
.data

fib_array:                  # location of the fib array


.text
main:
    add     t0,x0,x0        # initialize iteration counter i to zero.
    addi    t1,x0,11        # initialize upper loop bound.
    la      t2,fib_array    # initialize fib_array index
loop:
    bge     t0,t1,end       # if i >= 11 exit loop.
    addi    a0,t0,0         # initialize parameter register a0 to t0 (i)
    jal     fib_rec         # call fib_rec function
    sw      a0,0(t2)        # store result in fib array
    addi    t2,t2,4         # increment fib_array index
    addi    t0,t0,1         # increment loop iteration count
    j       loop            # jump to the top of the loop
end:
    addi	a0,x0,10        # set a0 to 10 (exit code)
    csrrs   x31,t0, X"0"    # call CSRRS to get csr_cycles
    csrrc   x30,t0, X"2"    # call CSRRC to get csr_instret
    ecall       


fib_rec:
    addi    sp,sp,-12       # push ra and a0 on the stack
    sw      ra,0(sp)
    sw      a0,4(sp)

    addi    t3,x0,1         # t3 = 1
    blt     t3,a0,recursive # if a0 > 1 goto recursive
    j       return          # return a0

recursive:

    addi    a0,a0,-1        # a0 = a0 - 1
    jal     fib_rec         # call fib_rec
    sw      a0,8(sp)        # save result on the stack

    lw      a0,4(sp)        # load original a0
    addi    a0,a0,-2        # a0 = a0 - 2
    jal     fib_rec         # call fib_rec

    lw      a1,8(sp)        # load result of first fib_rec call in to a1
    add     a0,a0,a1        # a0 = a0 + a1

return:
    lw      ra,0(sp)        # pop return address form the stack
    addi    sp,sp,12        # deallocate stack space
    ret                     # return

=====
RV32I machine code

X"B3", X"02", X"00", X"00", X"13", X"03", X"B0", X"00", 
X"97", X"03", X"00", X"10", X"93", X"83", X"83", X"FF", 
X"63", X"DE", X"62", X"00", X"13", X"85", X"02", X"00", 
X"EF", X"00", X"40", X"02", X"23", X"A0", X"A3", X"00", 
X"93", X"83", X"43", X"00", X"93", X"82", X"12", X"00", 
X"6F", X"F0", X"9F", X"FE", X"13", X"05", X"A0", X"00", 
X"F3", X"AF", X"02", X"00", X"73", X"BF", X"22", X"00",         -- CSR instructions
X"73", X"00", X"00", X"00", X"13", X"01", X"41", X"FF", 
X"23", X"20", X"11", X"00", X"23", X"22", X"A1", X"00", 
X"13", X"0E", X"10", X"00", X"63", X"44", X"AE", X"00", 
X"6F", X"00", X"40", X"02", X"13", X"05", X"F5", X"FF", 
X"EF", X"F0", X"5F", X"FE", X"23", X"24", X"A1", X"00", 
X"03", X"25", X"41", X"00", X"13", X"05", X"E5", X"FF", 
X"EF", X"F0", X"5F", X"FD", X"83", X"25", X"81", X"00", 
X"33", X"05", X"B5", X"00", X"83", X"20", X"01", X"00", 
X"13", X"01", X"C1", X"00", X"67", X"80", X"00", X"00"

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
        X"B3", X"02", X"00", X"00", X"13", X"03", X"B0", X"00", 
        X"97", X"03", X"00", X"10", X"93", X"83", X"83", X"FF", 
        X"63", X"DE", X"62", X"00", X"13", X"85", X"02", X"00", 
        X"EF", X"00", X"40", X"02", X"23", X"A0", X"A3", X"00", 
        X"93", X"83", X"43", X"00", X"93", X"82", X"12", X"00", 
        X"6F", X"F0", X"9F", X"FE", X"13", X"05", X"A0", X"00", 
        X"F3", X"AF", X"02", X"00", X"73", X"BF", X"22", X"00",         -- CSR instructions
        X"73", X"00", X"00", X"00", X"13", X"01", X"41", X"FF", 
        X"23", X"20", X"11", X"00", X"23", X"22", X"A1", X"00", 
        X"13", X"0E", X"10", X"00", X"63", X"44", X"AE", X"00", 
        X"6F", X"00", X"40", X"02", X"13", X"05", X"F5", X"FF", 
        X"EF", X"F0", X"5F", X"FE", X"23", X"24", X"A1", X"00", 
        X"03", X"25", X"41", X"00", X"13", X"05", X"E5", X"FF", 
        X"EF", X"F0", X"5F", X"FD", X"83", X"25", X"81", X"00", 
        X"33", X"05", X"B5", X"00", X"83", X"20", X"01", X"00", 
        X"13", X"01", X"C1", X"00", X"67", X"80", X"00", X"00", 
        others => (others=>'0'));
        variable word_addr : std_logic_vector (ADDRLEN-1 downto 0) := (others=>'0');
    begin
        word_addr := addr(ADDRLEN-1 downto 2) & "00";
        dataout <= rom_array(to_integer(unsigned(word_addr))+3) & rom_array(to_integer(unsigned(word_addr))+2) & rom_array(to_integer(unsigned(word_addr))+1) & rom_array(to_integer(unsigned(word_addr)));
    end process;

end architecture;


-- csrrs : X"F3", X"AF", X"02", X"00", csrrc: X"73", X"BF", X"22", X"00",
--x5: 0x0000000b as simulated by venus
--rd for set: x31; rd for clear: x30

-- in the testbench file also change the for loop to 5078 to account for the 2 newly added instructions
