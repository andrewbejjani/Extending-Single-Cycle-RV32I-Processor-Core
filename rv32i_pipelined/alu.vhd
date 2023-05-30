-- Arithmetic and Logic Unit (ALU)

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.archer_pkg.all;

entity alu is
    port (
        inputA : in std_logic_vector (XLEN-1 downto 0);
        inputB : in std_logic_vector (XLEN-1 downto 0);
        ALUop : in std_logic_vector (4 downto 0);
        result : out std_logic_vector (XLEN-1 downto 0)
    );
end alu;

architecture rtl of alu is

    --Base Signals
    signal add_result : std_logic_vector (XLEN-1 downto 0);
    signal sub_result : std_logic_vector (XLEN-1 downto 0);
    signal and_result : std_logic_vector (XLEN-1 downto 0);
    signal or_result : std_logic_vector (XLEN-1 downto 0);
    signal xor_result : std_logic_vector (XLEN-1 downto 0);
    signal sll_result : std_logic_vector (XLEN-1 downto 0);
    signal srl_result : std_logic_vector (XLEN-1 downto 0);
    signal sra_result : std_logic_vector (XLEN-1 downto 0);
    signal slt_result : std_logic_vector (XLEN-1 downto 0);
    signal sltu_result : std_logic_vector (XLEN-1 downto 0);
    signal csr_clear_result : std_logic_vector (XLEN-1 downto 0);

begin
    -- Base Instruction Set
    add_result <= std_logic_vector(signed(inputA) + signed(inputB));
    sub_result <= std_logic_vector(signed(inputA) - signed(inputB));
    and_result <= inputA and inputB;
    or_result <= inputA or inputB;
    xor_result <= inputA xor inputB;
    sll_result <= std_logic_vector(shift_left(unsigned(inputA), to_integer(unsigned(inputB(4 downto 0)))));
    srl_result <= std_logic_vector(shift_right(unsigned(inputA), to_integer(unsigned(inputB(4 downto 0)))));
    sra_result <= std_logic_vector(shift_right(signed(inputA), to_integer(unsigned(inputB(4 downto 0)))));
    slt_result <= (XLEN-1 downto 1 =>'0')&'1' when signed(inputA) < signed(inputB) else (others=>'0');
    sltu_result <= (XLEN-1 downto 1 =>'0')&'1' when unsigned(inputA) < unsigned(inputB) else (others=>'0');
    csr_clear_result <= not(inputA) and inputB; 
    
    with ALUop select
    result <=   add_result when ALU_OP_ADD, -- add
                sub_result when ALU_OP_SUB, -- sub
                and_result when ALU_OP_AND, -- and
                or_result when ALU_OP_OR, -- or
                xor_result when ALU_OP_XOR, -- xor
                sll_result when ALU_OP_SLL, -- sll
                srl_result when ALU_OP_SRL, -- srl
                sra_result when ALU_OP_SRA, -- sra
                slt_result when ALU_OP_SLT, -- slt
                sltu_result when ALU_OP_SLTU, -- sltu
                csr_clear_result when ALU_OP_CSRCLEAR,
                (others=>'0') when others;

end architecture;