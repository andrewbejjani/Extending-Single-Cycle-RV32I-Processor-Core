-- Execution / Memory Bus

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.archer_pkg.all;

entity EX_MEM is
    port (
        clk : in std_logic;
        rst_n : in std_logic;
        instr_in: in std_logic_vector (XLEN-1 downto 0);
        instr_out: out std_logic_vector (XLEN-1 downto 0);
        pcplus4_in: in std_logic_vector (XLEN-1 downto 0);
        pcplus4_out: out std_logic_vector (XLEN-1 downto 0);
        
        ALU_res_in : in std_logic_vector (XLEN-1 downto 0);
        ALU_res_out : out std_logic_vector (XLEN-1 downto 0);
        regB_in : in std_logic_vector (XLEN-1 downto 0);
        regB_out : out std_logic_vector (XLEN-1 downto 0);
        byte_mask : out std_logic_vector (1 downto 0);
        sign_ext_n : out std_logic;
        
        Jump_in : in std_logic;
        RegWrite_in : in std_logic;
        MemWrite_in : in std_logic;
        MemRead_in : in std_logic;
        MemToReg_in : in std_logic;
	    CSRSrc_in : in std_logic;

        Jump_out : out std_logic;
        RegWrite_out : out std_logic;
        MemWrite_out : out std_logic;
        MemRead_out : out std_logic;
        MemToReg_out : out std_logic;
        CSRSrc_out : out std_logic;
        
        NOP: in std_logic
    );
end EX_MEM;

architecture rtl of EX_MEM is
begin
    process (clk, rst_n, NOP) is
    begin
        if rising_edge(clk) then
            if rst_n ='1' then
                instr_out <= instr_in;
                pcplus4_out <= pcplus4_in;
                regB_out <= regB_in;
                ALU_res_out <= ALU_res_in;
                byte_mask <= instr_in(13 downto 12);
                sign_ext_n <= instr_in(14);

                Jump_out <= Jump_in;
                RegWrite_out <= RegWrite_in;
                MemWrite_out <= MemWrite_in;
                MemRead_out <= MemRead_in;
                MemToReg_out <= MemToReg_in;
                CSRSrc_out <= CSRSrc_in;
            end if;
            if rst_n = '0' or NOP = '1' then
                instr_out <= (others=>'0');
                pcplus4_out <= (others=>'0');
                regB_out <= (others=>'0');
                ALU_res_out <= (others=>'0');
                byte_mask <= (others=>'0');
                sign_ext_n <= '0';

                Jump_out <= '0';
                RegWrite_out <= '0';
                MemWrite_out <= '0';
                MemRead_out <= '0';
                MemToReg_out <= '0';
                CSRSrc_out <= '0';
            end if;
            
    end if;
    end process;

end architecture;
