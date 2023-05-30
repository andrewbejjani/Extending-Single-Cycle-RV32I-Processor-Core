-- Instruction Fetch / Instruction Decode Bus

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.archer_pkg.all;

entity IF_ID is
    port (
        clk : in std_logic;
        rst_n : in std_logic;
        instr_in: in std_logic_vector (XLEN-1 downto 0);
        instr_out: out std_logic_vector (XLEN-1 downto 0);
        pcplus4_in: in std_logic_vector (XLEN-1 downto 0);
        pcplus4_out: out std_logic_vector (XLEN-1 downto 0);
        pc_in: in std_logic_vector (XLEN-1 downto 0);
        pc_out: out std_logic_vector (XLEN-1 downto 0);
        rs1 : out std_logic_vector (LOG2_XRF_SIZE-1 downto 0);
        rs2 : out std_logic_vector (LOG2_XRF_SIZE-1 downto 0);
        funct3: out std_logic_vector (2 downto 0);
        PCSrc : in std_logic;
        Stall : in std_logic
    );
end IF_ID;

architecture rtl of IF_ID is
begin
    process (clk, rst_n, Stall, PCSrc) is
    begin
        if (rising_edge(clk) and Stall/= '1') then
            if rst_n = '1' then 
                instr_out <= instr_in;
                pcplus4_out <= pcplus4_in;
                pc_out <= pc_in;
                rs1 <= instr_in(LOG2_XRF_SIZE+14 downto 15);
                rs2 <= instr_in(LOG2_XRF_SIZE+19 downto 20);
                funct3 <= instr_in (14 downto 12);
            end if;
            if rst_n = '0' or PCSrc = '1' then 
                instr_out <= (others=>'0');
                pcplus4_out <= (others=>'0');
                pc_out <= (others=>'0');
                rs1 <= (others=>'0');
                rs2 <= (others=>'0');
                funct3 <= (others=>'0');
            end if;
    end if;
    end process;

end architecture;
