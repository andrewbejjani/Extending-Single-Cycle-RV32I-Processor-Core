-- Memory / Write-Back Bus

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.archer_pkg.all;

entity MEM_WB is
    port (
        clk : in std_logic;
        rst_n : in std_logic;
        instr_in: in std_logic_vector (XLEN-1 downto 0);
        instr_out: out std_logic_vector (XLEN-1 downto 0);
        rd: out std_logic_vector (LOG2_XRF_SIZE-1 downto 0);
        pcplus4_in: in std_logic_vector (XLEN-1 downto 0);
        pcplus4_out: out std_logic_vector (XLEN-1 downto 0);

        ALU_res_in : in std_logic_vector (XLEN-1 downto 0);
        ALU_res_out : out std_logic_vector (XLEN-1 downto 0);
        data_mem_in: in std_logic_vector (XLEN-1 downto 0);
        data_mem_out: out std_logic_vector (XLEN-1 downto 0);

        Jump_in : in std_logic;
        RegWrite_in : in std_logic;
        MemToReg_in : in std_logic;
	    CSRSrc_in : in std_logic;

        Jump_out : out std_logic;
        RegWrite_out : out std_logic;
        MemToReg_out : out std_logic;
        CSRSrc_out : out std_logic

    );
end MEM_WB;

architecture rtl of MEM_WB is
begin
    process (clk, rst_n) is
    begin
        if rising_edge(clk) and rst_n = '1' then
            instr_out <= instr_in;
            pcplus4_out <= pcplus4_in;
            ALU_res_out <= ALU_res_in;
            data_mem_out <= data_mem_in;
            rd <= instr_in(LOG2_XRF_SIZE+6 downto 7);

            Jump_out <= Jump_in;
            RegWrite_out <= RegWrite_in;
            MemToReg_out <= MemToReg_in;
            CSRSrc_out <= CSRSrc_in;
            
    end if;
    end process;

end architecture;
