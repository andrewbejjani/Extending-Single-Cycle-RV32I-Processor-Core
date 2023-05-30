-- Forwarding Unit

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.archer_pkg.all;

entity F_Unit is
    port (
        MEM_Instr : in std_logic_vector (XLEN-1 downto 0);
        WB_Instr : in std_logic_vector (XLEN-1 downto 0);
        EX_Instr : in std_logic_vector (XLEN-1 downto 0);
        M_Instr : in std_logic_vector (XLEN-1 downto 0);
        MEM_RegWrite : in std_logic;
        WB_RegWrite : in std_logic;
        Mux1_inputA_sig : out std_logic;
        Mux2_inputA_sig : out std_logic;
        Mux3_inputB_sig : out std_logic;
        Mux4_inputB_sig : out std_logic;

        M_Mux1_inputA_sig : out std_logic;
        M_Mux2_inputA_sig : out std_logic;
        M_Mux3_inputB_sig : out std_logic;
        M_Mux4_inputB_sig : out std_logic
    );
end F_Unit;

architecture rtl of F_Unit is

    signal EX_rs1 : std_logic_vector (4 downto 0);
    signal EX_rs2 : std_logic_vector (4 downto 0);
    signal MEM_rd : std_logic_vector (4 downto 0);
    signal WB_rd : std_logic_vector (4 downto 0);

    signal M_rs1 : std_logic_vector (4 downto 0);
    signal M_rs2 : std_logic_vector (4 downto 0);   


 

begin

    EX_rs1 <= EX_Instr (LOG2_XRF_SIZE+14 downto 15);
    EX_rs2 <= EX_Instr (LOG2_XRF_SIZE+19 downto 20);
    MEM_rd <= MEM_Instr (LOG2_XRF_SIZE+6 downto 7);
    WB_rd <= WB_Instr (LOG2_XRF_SIZE+6 downto 7);

    M_rs1 <= M_Instr (LOG2_XRF_SIZE+14 downto 15);
    M_rs2 <= M_Instr (LOG2_XRF_SIZE+19 downto 20);

    Mux2_inputA_sig <= '1' when (MEM_RegWrite = '1' and MEM_rd /= "00000" and MEM_rd = EX_rs1) else
                        '0';

    Mux4_inputB_sig <= '1' when (MEM_RegWrite = '1' and MEM_rd /= "00000" and MEM_rd = EX_rs2) else
                        '0';

    Mux1_inputA_sig <= '1' when (WB_RegWrite = '1' and WB_rd /= "00000"  and MEM_rd /= EX_rs1 and WB_rd = EX_rs1) else
                        '0';

    Mux3_inputB_sig <= '1' when (WB_RegWrite = '1' and WB_rd /= "00000"  and MEM_rd /= EX_rs2 and WB_rd = EX_rs2) else
                        '0';

    M_Mux2_inputA_sig <= '1' when (MEM_RegWrite = '1' and MEM_rd /= "00000" and MEM_rd = M_rs1) else
                        '0';

    M_Mux4_inputB_sig <= '1' when (MEM_RegWrite = '1' and MEM_rd /= "00000" and MEM_rd = M_rs2) else
                        '0';

    M_Mux1_inputA_sig <= '1' when (WB_RegWrite = '1' and WB_rd /= "00000"  and MEM_rd /= M_rs1 and WB_rd = M_rs1) else
                        '0';

    M_Mux3_inputB_sig <= '1' when (WB_RegWrite = '1' and WB_rd /= "00000"  and MEM_rd /= M_rs2 and WB_rd = M_rs2) else
                        '0';

end architecture;
