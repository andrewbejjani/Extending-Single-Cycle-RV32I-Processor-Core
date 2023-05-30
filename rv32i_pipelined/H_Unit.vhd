-- Hazard Detection Unit

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.archer_pkg.all;

entity H_Unit is
    port (
        ID_Instr : in std_logic_vector (XLEN-1 downto 0);
        EX_Instr : in std_logic_vector (XLEN-1 downto 0);
        EX_MemRead: in std_logic;

        load_stall : out std_logic;
        load_nop : out std_logic
    );
end H_Unit;

architecture rtl of H_Unit is

    signal ID_rs1 : std_logic_vector (4 downto 0);
    signal ID_rs2 : std_logic_vector (4 downto 0);
    signal EX_rd : std_logic_vector (4 downto 0);

begin

    ID_rs1 <= ID_Instr (LOG2_XRF_SIZE+14 downto 15);
    ID_rs2 <= ID_Instr (LOG2_XRF_SIZE+19 downto 20);
    EX_rd <= EX_Instr (LOG2_XRF_SIZE+6 downto 7);

    load_stall <= '1' when (EX_MemRead = '1' and (EX_rd = ID_rs1 or EX_rd = ID_rs2)) else
                    '0';

    load_nop <= '1' when (EX_MemRead = '1' and (EX_rd = ID_rs1 or EX_rd = ID_rs2)) else
                    '0';

end architecture;
