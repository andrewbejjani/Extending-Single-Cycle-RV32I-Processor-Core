-- Instruction Decode / Execution Bus

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.archer_pkg.all;

entity ID_EX is
    port (
        clk : in std_logic;
        rst_n : in std_logic;
        instr_in: in std_logic_vector (XLEN-1 downto 0);
        instr_out: out std_logic_vector (XLEN-1 downto 0);
        pcplus4_in: in std_logic_vector (XLEN-1 downto 0);
        pcplus4_out: out std_logic_vector (XLEN-1 downto 0);
        pc_in: in std_logic_vector (XLEN-1 downto 0);
        pc_out: out std_logic_vector (XLEN-1 downto 0);
        
        regA_in : in std_logic_vector (XLEN-1 downto 0);
        regB_in : in std_logic_vector (XLEN-1 downto 0);
        regA_out : out std_logic_vector (XLEN-1 downto 0);
        regB_out : out std_logic_vector (XLEN-1 downto 0);
        imm_in : in std_logic_vector (XLEN-1 downto 0);
        imm_out : out std_logic_vector (XLEN-1 downto 0);
    
        Jump_in : in std_logic;
        Lui_in : in std_logic;
        RegWrite_in : in std_logic;
        ALUSrc1_in : in std_logic;
        ALUSrc2_in : in std_logic;
        ALUOp_in : in std_logic_vector (4 downto 0);
        MemWrite_in : in std_logic;
        MemRead_in : in std_logic;
        MemToReg_in : in std_logic;
	    CSRSrc_in : in std_logic;
        M_valid_in : in std_logic_vector (1 downto 0);
        
        Jump_out : out std_logic;
        Lui_out : out std_logic;
        RegWrite_out : out std_logic;
        ALUSrc1_out : out std_logic;
        ALUSrc2_out : out std_logic;
        ALUOp_out : out std_logic_vector (4 downto 0);
        MemWrite_out : out std_logic;
        MemRead_out : out std_logic;
        MemToReg_out : out std_logic;
        CSRSrc_out : out std_logic;
        M_valid_out : out std_logic_vector (1 downto 0);

        Stall : in std_logic;   --for multiply stalling
        NOP_load : in std_logic --for load nop

    );
end ID_EX;

architecture rtl of ID_EX is
begin
    process (clk, rst_n, Stall, NOP_load) is
    begin
        if rising_edge(clk) then
            if Stall /= '1' then 
                if rst_n = '1' then 
                    instr_out <= instr_in;
                    pcplus4_out <= pcplus4_in;
                    pc_out <= pc_in;
                    regA_out <= regA_in;
                    regB_out <= regB_in;
                    imm_out <= imm_in;

                    Jump_out <= Jump_in;
                    Lui_out <= Lui_in;
                    RegWrite_out <= RegWrite_in;
                    ALUSrc1_out <= ALUSrc1_in;
                    ALUSrc2_out <= ALUSrc2_in;
                    ALUOp_out <= ALUOp_in;
                    MemWrite_out <= MemWrite_in;
                    MemRead_out <= MemRead_in;
                    MemToReg_out <= MemToReg_in;
                    CSRSrc_out <= CSRSrc_in;
                    M_valid_out <= M_valid_in;
                else 
                    instr_out <=  (others=>'0');
                    pcplus4_out <=  (others=>'0');
                    pc_out <=  (others=>'0');
                    regA_out <=  (others=>'0');
                    regB_out <=  (others=>'0');
                    imm_out <=  (others=>'0');

                    Jump_out <= '0';
                    Lui_out <= '0';
                    RegWrite_out <= '0';
                    ALUSrc1_out <= '0';
                    ALUSrc2_out <= '0';
                    ALUOp_out <= (others=>'0');
                    MemWrite_out <= '0';
                    MemRead_out <= '0';
                    MemToReg_out <= '0';
                    CSRSrc_out <= '0';
                    M_valid_out <= (others => '0');
                end if;
            elsif NOP_load = '1' then 
                instr_out <=  (others=>'0');
                pcplus4_out <=  (others=>'0');
                pc_out <=  (others=>'0');
                regA_out <=  (others=>'0');
                regB_out <=  (others=>'0');
                imm_out <=  (others=>'0');

                Jump_out <= '0';
                Lui_out <= '0';
                RegWrite_out <= '0';
                ALUSrc1_out <= '0';
                ALUSrc2_out <= '0';
                ALUOp_out <= (others=>'0');
                MemWrite_out <= '0';
                MemRead_out <= '0';
                MemToReg_out <= '0';
                CSRSrc_out <= '0';
                M_valid_out <= (others => '0');
            end if;
        end if;
    end process;

end architecture;
