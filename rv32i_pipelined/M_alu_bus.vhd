-- M Extension ALU Bus

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.archer_pkg.all;

entity M_alu_bus is
    port (
        clk : in std_logic;
        M_valid_in : in std_logic_vector (1 downto 0);
        inputA_in: in std_logic_vector (XLEN-1 downto 0);
        inputB_in: in std_logic_vector (XLEN-1 downto 0);
        ALUop_in : in std_logic_vector (4 downto 0);
        instr_in : in std_logic_vector (XLEN-1 downto 0);

        M_valid_out : out std_logic_vector (1 downto 0);
        inputA_out: out std_logic_vector (XLEN-1 downto 0);
        inputB_out: out std_logic_vector (XLEN-1 downto 0);
        ALUop_out : out std_logic_vector (4 downto 0);
        instr_out : out std_logic_vector (XLEN-1 downto 0)
    );
end M_alu_bus;

architecture rtl of M_alu_bus is
begin
    process(clk, M_valid_in)
    begin
    if rising_edge(clk) then
        M_valid_out <= M_valid_in;
        inputA_out <= inputA_in;
        inputB_out <= inputB_in;
        ALUop_out <= ALUop_in;
        instr_out <= instr_in; 
    end if;
    end process;
end architecture;   











