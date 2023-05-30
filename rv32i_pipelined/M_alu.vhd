-- M Extension ALU

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.archer_pkg.all;

entity M_alu is
    port (
        clk : in std_logic;
        M_valid : in std_logic_vector (1 downto 0);
        inputA : in std_logic_vector (XLEN-1 downto 0);
        inputB : in std_logic_vector (XLEN-1 downto 0);
        ALUop : in std_logic_vector (4 downto 0);
        instr_in : in std_logic_vector (XLEN-1 downto 0);
        instr_out : out std_logic_vector (XLEN-1 downto 0);
        result : out std_logic_vector (XLEN-1 downto 0);
        Stall : out std_logic_vector (1 downto 0);
        NOP : out std_logic;

        M_Mux1_inputA_sig : in std_logic;
        M_Mux2_inputA_sig : in std_logic;
        M_Mux3_inputB_sig : in std_logic;
        M_Mux4_inputB_sig : in std_logic;
        WB_data : in std_logic_vector (XLEN-1 downto 0);
        MEM_data : in std_logic_vector (XLEN-1 downto 0)

    );
end M_alu;

architecture rtl of M_alu is
    component M_alu_bus is 
        port(
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
    end component;

    component mux2to1
        port (
            sel : in std_logic;
            input0 : in std_logic_vector (XLEN-1 downto 0);
            input1 : in std_logic_vector (XLEN-1 downto 0);
            output : out std_logic_vector (XLEN-1 downto 0)
        );
    end component;

    signal M_valid_EX2 : std_logic_vector (1 downto 0);
    signal inputA_EX2 : std_logic_vector (XLEN-1 downto 0);
    signal inputB_EX2 : std_logic_vector (XLEN-1 downto 0); 
    signal ALUop_EX2 : std_logic_vector (4 downto 0);
    signal instr_EX2 : std_logic_vector (XLEN-1 downto 0);

    signal M_valid_EX3 : std_logic_vector (1 downto 0);
    signal inputA_EX3 : std_logic_vector (XLEN-1 downto 0);
    signal inputB_EX3 : std_logic_vector (XLEN-1 downto 0); 
    signal ALUop_EX3 : std_logic_vector (4 downto 0);
    signal instr_EX3 : std_logic_vector (XLEN-1 downto 0);

    signal M_valid_EX4 : std_logic_vector (1 downto 0);
    signal inputA_EX4 : std_logic_vector (XLEN-1 downto 0);
    signal inputB_EX4 : std_logic_vector (XLEN-1 downto 0); 
    signal ALUop_EX4 : std_logic_vector (4 downto 0);
    signal instr_EX4 : std_logic_vector (XLEN-1 downto 0);

    signal M_valid_EX5 : std_logic_vector (1 downto 0);
    signal inputA_EX5 : std_logic_vector (XLEN-1 downto 0);
    signal inputB_EX5 : std_logic_vector (XLEN-1 downto 0); 
    signal ALUop_EX5 : std_logic_vector (4 downto 0);
    signal instr_EX5 : std_logic_vector (XLEN-1 downto 0);

    --M extension signals
    signal muls_result : std_logic_vector (2*XLEN-1 downto 0);
    signal mulu_result : std_logic_vector (2*XLEN-1 downto 0);
    signal mulsu_result : std_logic_vector (2*XLEN downto 0);
    signal div_result : std_logic_vector (XLEN-1 downto 0);
    signal divu_result : std_logic_vector (XLEN-1 downto 0);
    signal rem_result : std_logic_vector (XLEN-1 downto 0);
    signal remu_result : std_logic_vector (XLEN-1 downto 0);

    --Res of MUX
    signal Res_Mux_1: std_logic_vector (XLEN-1 downto 0);
    signal Res_Mux_2: std_logic_vector (XLEN-1 downto 0);
    signal Res_Mux_3: std_logic_vector (XLEN-1 downto 0);
    signal Res_Mux_4: std_logic_vector (XLEN-1 downto 0);


begin

    M_bus1 : M_alu_bus port map(clk=>clk, M_valid_in=>M_valid, inputA_in=>inputA, inputB_in=>inputB, ALUOp_in=>ALUop,instr_in =>instr_in, 
                                    M_valid_out=>M_valid_EX2, inputA_out=>inputA_EX2, inputB_out=>inputB_EX2,ALUOp_out=>ALUOp_EX2, instr_out => instr_EX2);
    M_bus2 : M_alu_bus port map(clk=>clk, M_valid_in=>M_valid_EX2, inputA_in=>inputA_EX2, inputB_in=>inputB_EX2, ALUOp_in=>ALUOp_EX2,instr_in =>instr_EX2, 
                                    M_valid_out=>M_valid_EX3, inputA_out=>inputA_EX3, inputB_out=>inputB_EX3,ALUOp_out=>ALUOp_EX3, instr_out => instr_EX3);
    M_bus3 : M_alu_bus port map(clk=>clk, M_valid_in=>M_valid_EX3, inputA_in=>inputA_EX3, inputB_in=>inputB_EX3, ALUOp_in=>ALUOp_EX3,instr_in =>instr_EX3, 
                                    M_valid_out=>M_valid_EX4, inputA_out=>inputA_EX4, inputB_out=>inputB_EX4,ALUOp_out=>ALUOp_EX4, instr_out => instr_EX4);
    M_bus4 : M_alu_bus port map(clk=>clk, M_valid_in=>M_valid_EX4, inputA_in=>inputA_EX4, inputB_in=>inputB_EX4, ALUOp_in=>ALUOp_EX4,instr_in =>instr_EX4,
                                    M_valid_out=>M_valid_EX5, inputA_out=>inputA_EX5, inputB_out=>inputB_EX5,ALUOp_out=>ALUOp_EX5, instr_out => instr_EX5);

    instr_out  <= instr_EX5; 


    FM_mux1 : mux2to1 port map (sel => M_Mux1_inputA_sig, input0 => inputA_EX5, input1 => WB_data, output => Res_Mux_1);
    FM_mux2 : mux2to1 port map (sel => M_Mux2_inputA_sig, input0 => Res_Mux_1, input1 => MEM_data, output => Res_Mux_2);
    FM_mux3 : mux2to1 port map (sel => M_Mux3_inputB_sig, input0 => inputB_EX5, input1 => WB_data, output => Res_Mux_3);
    FM_mux4 : mux2to1 port map (sel => M_Mux4_inputB_sig, input0 => Res_Mux_3, input1 => MEM_data, output => Res_Mux_4);

    -- M extensions
    muls_result <= std_logic_vector(signed(Res_Mux_2) * signed(Res_Mux_4));
    mulu_result <= std_logic_vector(unsigned(Res_Mux_2) * unsigned(Res_Mux_4));
    mulsu_result <= std_logic_vector(signed(Res_Mux_2) * (signed('0'& Res_Mux_4))); --we make input B as a positive number by adding the 0
    process (Res_Mux_2, Res_Mux_4) is  
    begin
        if Res_Mux_4 = X"00000000" then
            div_result <= X"FFFFFFFF";   -- (-1)
            divu_result <= X"FFFFFFFF";  -- 2**XLEN -1
            rem_result <= Res_Mux_2;
            remu_result <= Res_Mux_2;
        elsif Res_Mux_2 = X"10000000" and signed(Res_Mux_4) = X"FFFFFFFF" then
            div_result <= X"10000000";   -- (-2**XLEN -1)
            rem_result <= X"00000000";   -- 0
        else
            div_result <= std_logic_vector(signed(Res_Mux_2)/signed(Res_Mux_4));
            divu_result <= std_logic_vector(unsigned(Res_Mux_2)/unsigned(Res_Mux_4));
            rem_result <= std_logic_vector(signed(Res_Mux_2) rem signed(Res_Mux_4));
            remu_result <= std_logic_vector(unsigned(Res_Mux_2) rem unsigned(Res_Mux_4));
        end if;
    end process;   

    with ALUOp_EX5 select
    result <=   muls_result(XLEN-1 downto 0) when ALU_OP_MUL, -- mul
                muls_result(2*XLEN-1 downto XLEN) when ALU_OP_MULH, -- mulh
                mulsu_result(2*XLEN-1 downto XLEN) when ALU_OP_MULHSU, -- mulhsu (take bits 63 to 32)
                mulu_result(2*XLEN-1 downto XLEN) when ALU_OP_MULHU, --mulhu
                div_result when ALU_OP_DIV, -- div
                divu_result when ALU_OP_DIVU, -- divu
                rem_result when ALU_OP_REM, -- rem
                remu_result when ALU_OP_REMU, -- remu
                (others=>'0') when others;

    

    Stall <=   "01" when (M_valid="11" and M_valid_EX2="11" and M_valid_EX3="11" and M_valid_EX4="11" and M_valid_EX5="11" and instr_EX5 = instr_in) else --finished all 5 cycles
                "00" when (M_valid="01" and M_valid_EX2="01" and M_valid_EX3="01" and M_valid_EX4="01" and M_valid_EX5="01" and instr_EX5 = instr_EX4) else 
                "1" & M_valid(1) when (M_valid(0)='1') else --still haven't finished all 5 cycles
                "00" ;
    NOP <=     '0' when (M_valid="11" and M_valid_EX2="11" and M_valid_EX3="11" and M_valid_EX4="11" and M_valid_EX5="11") else --finished all 5 cycles
                '0' when (M_valid="01" and M_valid_EX2="01" and M_valid_EX3="01" and M_valid_EX4="01" and M_valid_EX5="01") else 
                '1' when (M_valid(0)='1') else --still haven't finished all 5 cycles
                '0' ;
end architecture;