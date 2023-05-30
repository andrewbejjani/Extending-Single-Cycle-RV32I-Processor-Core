-- Archer RV32I single-cycle datapath wrapper (top-level entity)

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.archer_pkg.all;

entity archer_rv32i_pipelined is
    port (
        clk : in std_logic;
        rst_n : in std_logic;
        -- local instruction memory bus interface
        imem_addr : out std_logic_vector (ADDRLEN-1 downto 0);
        imem_datain : out std_logic_vector (XLEN-1 downto 0);
        imem_dataout : in std_logic_vector (XLEN-1 downto 0);
        imem_wen : out std_logic; -- write enable signal
        imem_ben : out std_logic_vector (3 downto 0); -- byte enable signals
        -- local data memory bus interface
        dmem_addr : out std_logic_vector (ADDRLEN-1 downto 0);
        dmem_datain : out std_logic_vector (XLEN-1 downto 0);
        dmem_dataout : in std_logic_vector (XLEN-1 downto 0);
        dmem_wen : out std_logic; -- write enable signal
        dmem_ben : out std_logic_vector (3 downto 0) -- byte enable signals
    );
end archer_rv32i_pipelined;

architecture rtl of archer_rv32i_pipelined is
    component add4
        port (
            datain : in std_logic_vector (XLEN-1 downto 0);
            result : out std_logic_vector (XLEN-1 downto 0)
        );
    end component;

    component alu
        port (
            inputA : in std_logic_vector (XLEN-1 downto 0);
            inputB : in std_logic_vector (XLEN-1 downto 0);
            ALUop : in std_logic_vector (4 downto 0);
            result : out std_logic_vector (XLEN-1 downto 0)
        );
    end component;

    component M_alu
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
    end component;

    component branch_cmp
        port (
            inputA : in std_logic_vector(XLEN-1 downto 0);
            inputB : in std_logic_vector(XLEN-1 downto 0);
            cond : in std_logic_vector(2 downto 0);
            result : out std_logic
        );
    end component;

    component control
        port (
            instruction : in std_logic_vector (XLEN-1 downto 0);
            BranchCond : in std_logic; -- BR. COND. SATISFIED = 1; NOT SATISFIED = 0
            Stall_in : in std_logic_vector (1 downto 0);
            Load_Stall_in : in std_logic;
            NOP_in : in std_logic;
            Jump : out std_logic;
            Lui : out std_logic;
            PCSrc : out std_logic;
            RegWrite : out std_logic;
            ALUSrc1 : out std_logic;
            ALUSrc2 : out std_logic;
            ALUOp : out std_logic_vector (4 downto 0);
            MemWrite : out std_logic;
            MemRead : out std_logic;
            MemToReg : out std_logic;
            CSRSrc : out std_logic; -- CSR sig
            M_valid : out std_logic_vector (1 downto 0); -- M sig 
            Stall_out : out std_logic;
            NOP_out : out std_logic
        ) ;
      end component; 

    component lmb
        port (
            proc_addr : in std_logic_vector (XLEN-1 downto 0);
            proc_data_send : in std_logic_vector (XLEN-1 downto 0);
            proc_data_receive : out std_logic_vector (XLEN-1 downto 0);
            proc_byte_mask : in std_logic_vector (1 downto 0); -- "00" = byte; "01" = half-word; "10" = word
            proc_sign_ext_n : in std_logic;
            proc_mem_write : in std_logic;
            proc_mem_read : in std_logic;
            mem_addr : out std_logic_vector (ADDRLEN-1 downto 0);
            mem_datain : out std_logic_vector (XLEN-1 downto 0);
            mem_dataout : in std_logic_vector (XLEN-1 downto 0);
            mem_wen : out std_logic; -- write enable signal
            mem_ben : out std_logic_vector (3 downto 0) -- byte enable signals
        );
    end component;

    component immgen
        port (
            instruction : in std_logic_vector (XLEN-1 downto 0);
            immediate : out std_logic_vector (XLEN-1 downto 0)
        );
    end component;

    component branch_adder
        port (
            imm : in std_logic_vector (XLEN-1 downto 0);
            pc : in std_logic_vector (XLEN-1 downto 0);
            result : out std_logic_vector (XLEN-1 downto 0)
        );
    end component;

    component imem
        port (
            address : in std_logic_vector (ADDRLEN-1 downto 0);
            dataout : out std_logic_vector (XLEN-1 downto 0)
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
    
    component pc
        port (
            clk : in std_logic;
            rst_n : in std_logic;
            datain : in std_logic_vector(XLEN-1 downto 0);
            dataout : out std_logic_vector(XLEN-1 downto 0)
        );
    end component;

    component regfile
        port (
            clk : in std_logic;
            rst_n : in std_logic;
            RegWrite : in std_logic;
            rs1 : in std_logic_vector (LOG2_XRF_SIZE-1 downto 0);
            rs2 : in std_logic_vector (LOG2_XRF_SIZE-1 downto 0);
            rd : in std_logic_vector (LOG2_XRF_SIZE-1 downto 0);
            datain : in std_logic_vector (XLEN-1 downto 0);
            regA : out std_logic_vector (XLEN-1 downto 0);
            regB : out std_logic_vector (XLEN-1 downto 0)
        );
    end component;

    component csr
        port (
            clk : in std_logic;
            rst_n : in std_logic;
            csr_sig : in std_logic;
            csr_sig_wb: in std_logic;
            instr_first: in std_logic_vector (31 downto 0);
            instr_second: in std_logic_vector (31 downto 0);
            csr_data_in: in std_logic_vector (XLEN-1 downto 0);
            csr_out : out std_logic_vector (31 downto 0)
        );
    end component;

    component IF_ID
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
    end component;

    component ID_EX
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
end component;

component EX_MEM
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
end component;

component MEM_WB
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
end component;

component F_Unit
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
end component;

component H_Unit
    port (
        ID_Instr : in std_logic_vector (XLEN-1 downto 0);
        EX_Instr : in std_logic_vector (XLEN-1 downto 0);
        EX_MemRead: in std_logic;

        load_stall : out std_logic;
        load_nop : out std_logic
    );
end component;

    -- pc signals
    signal d_pc_in : std_logic_vector (XLEN-1 downto 0);
    signal d_pc_in1 : std_logic_vector (XLEN-1 downto 0);
    signal d_pc_out : std_logic_vector (XLEN-1 downto 0);

    -- imem signals
    signal d_imem_addr : std_logic_vector (ADDRLEN-1 downto 0);
    signal d_instr_word : std_logic_vector (XLEN-1 downto 0);

    -- add4 signals
    signal d_pcplus4 : std_logic_vector (XLEN-1 downto 0);

    -- control signals
    signal c_branch_out : std_logic;
    signal c_jump : std_logic;
    signal c_lui : std_logic;
    signal c_PCSrc : std_logic;
    signal c_reg_write : std_logic;
    signal c_alu_src1 : std_logic;
    signal c_alu_src2 : std_logic;
    signal c_alu_op : std_logic_vector (4 downto 0);
    signal c_mem_write : std_logic;
    signal c_mem_read : std_logic;
    signal c_mem_to_reg : std_logic;
    signal c_csr : std_logic;
    signal c_M_valid : std_logic_vector (1 downto 0);

    signal c_stall_in : std_logic_vector (1 downto 0);
    signal c_NOP_in : std_logic;
    signal c_stall_out : std_logic;
    signal c_NOP_out : std_logic;

    -- register file signals

    signal d_reg_file_datain : std_logic_vector (XLEN-1 downto 0);
    signal d_regA : std_logic_vector (XLEN-1 downto 0);
    signal d_regB : std_logic_vector (XLEN-1 downto 0);

    -- immgen signals

    signal d_immediate : std_logic_vector (XLEN-1 downto 0);

    -- branch adder signals

    signal d_brnch_add : std_logic_vector (XLEN-1 downto 0);

    -- lui_mux signals

    signal d_zero : std_logic_vector (XLEN-1 downto 0);
    signal d_lui_mux_out : std_logic_vector (XLEN-1 downto 0);

    -- csr_mux signal

    signal d_csr_mux_out : std_logic_vector (XLEN-1 downto 0);

    -- alu_src1_mux signals 

    signal d_alu_src1 : std_logic_vector (XLEN-1 downto 0);

    -- alu_src2_mux signals 

    signal d_alu_src2 : std_logic_vector (XLEN-1 downto 0);

    -- alu signals

    signal d_alu_out : std_logic_vector (XLEN-1 downto 0);

    -- M_alu signals

    signal d_M_alu_val_out : std_logic_vector (XLEN-1 downto 0);
    signal d_M_alu_instr_out : std_logic_vector (XLEN-1 downto 0);
    signal d_M_alu_val_mux_out : std_logic_vector (XLEN-1 downto 0);
    signal d_M_alu_inst_mux_out : std_logic_vector (XLEN-1 downto 0);

    -- mem_mux signals

    signal d_mem_mux_out : std_logic_vector (XLEN-1 downto 0);

    -- lmb signals

    signal d_data_mem_out : std_logic_vector (XLEN-1 downto 0);

    -- csr signals 

    signal d_csr_out: std_logic_vector (31 downto 0);

    -- ID signals

    signal ID_instr: std_logic_vector (XLEN-1 downto 0);
    signal ID_pcplus4: std_logic_vector (XLEN-1 downto 0);
    signal ID_pc: std_logic_vector (XLEN-1 downto 0);
    signal ID_rs1: std_logic_vector (LOG2_XRF_SIZE-1 downto 0);
    signal ID_rs2: std_logic_vector (LOG2_XRF_SIZE-1 downto 0);
    signal ID_funct3: std_logic_vector (2 downto 0);

    -- EX signals

    signal EX_instr: std_logic_vector (XLEN-1 downto 0);
    signal EX_pcplus4: std_logic_vector (XLEN-1 downto 0);
    signal EX_pc: std_logic_vector (XLEN-1 downto 0);
    signal EX_regA: std_logic_vector (XLEN-1 downto 0);
    signal EX_regB: std_logic_vector (XLEN-1 downto 0);
    signal EX_imm: std_logic_vector (XLEN-1 downto 0);
    signal EX_jump : std_logic;
    signal EX_lui : std_logic;
    signal EX_reg_write : std_logic;
    signal EX_alu_src1 : std_logic;
    signal EX_alu_src2 : std_logic;
    signal EX_alu_op : std_logic_vector (4 downto 0);
    signal EX_mem_write : std_logic;
    signal EX_mem_read : std_logic;
    signal EX_mem_to_reg : std_logic;
    signal EX_csr : std_logic;
    signal EX_M_valid : std_logic_vector (1 downto 0);

    -- MEM signals

    signal MEM_instr: std_logic_vector (XLEN-1 downto 0);
    signal MEM_pcplus4: std_logic_vector (XLEN-1 downto 0);
    signal MEM_ALU_res: std_logic_vector (XLEN-1 downto 0);
    signal MEM_regB: std_logic_vector (XLEN-1 downto 0);
    signal MEM_byte_mask: std_logic_vector (1 downto 0);
    signal MEM_sign_ext_n: std_logic;
    signal MEM_jump : std_logic;
    signal MEM_reg_write : std_logic;
    signal MEM_mem_write : std_logic;
    signal MEM_mem_read : std_logic;
    signal MEM_mem_to_reg : std_logic;
    signal MEM_csr : std_logic;

    -- WB signals

    signal WB_instr: std_logic_vector (XLEN-1 downto 0);
    signal WB_rd: std_logic_vector (LOG2_XRF_SIZE-1 downto 0);
    signal WB_pcplus4: std_logic_vector (XLEN-1 downto 0);
    signal WB_ALU_res: std_logic_vector (XLEN-1 downto 0);
    signal WB_data_mem: std_logic_vector (XLEN-1 downto 0);
    signal WB_jump : std_logic;
    signal WB_reg_write : std_logic;
    signal WB_mem_to_reg : std_logic;
    signal WB_csr : std_logic;

    -- Forwarding Unit signals

    signal F_Mux1_inputA_sel : std_logic;
    signal F_Mux2_inputA_sel : std_logic;
    signal F_Mux3_inputB_sel : std_logic;
    signal F_Mux4_inputB_sel : std_logic;

    signal M_Mux1_inputA_sel : std_logic;
    signal M_Mux2_inputA_sel : std_logic;
    signal M_Mux3_inputB_sel : std_logic;
    signal M_Mux4_inputB_sel : std_logic;

    -- Forwarding Mux signals

    signal Res_Mux_1: std_logic_vector (XLEN-1 downto 0);
    signal Res_Mux_2: std_logic_vector (XLEN-1 downto 0);
    signal Res_Mux_3: std_logic_vector (XLEN-1 downto 0);
    signal Res_Mux_4: std_logic_vector (XLEN-1 downto 0);

   -- Hazard Detecting Unit signals

   signal L_stall: std_logic;
   signal L_nop: std_logic;

begin

    pc_inst : pc port map (clk => clk, rst_n => rst_n, datain => d_pc_in, dataout => d_pc_out);

    limb_inst : lmb port map (proc_addr => d_pc_out, proc_data_send => (others=>'0'), proc_data_receive => d_instr_word,
                              proc_byte_mask => "10", proc_sign_ext_n => '1', proc_mem_write => '0',
                              proc_mem_read => '1', mem_addr => imem_addr, mem_datain => imem_datain, 
                              mem_dataout => imem_dataout, mem_wen => imem_wen, mem_ben => imem_ben);

    add4_inst : add4 port map (datain => d_pc_out, result => d_pcplus4);
    pc_mux : mux2to1 port map (sel => c_PCSrc, input0 => d_pcplus4, input1 => d_brnch_add, output => d_pc_in1);
    pc_mux2 : mux2to1 port map (sel => c_stall_out, input0 => d_pc_in1, input1 => d_pc_out, output => d_pc_in);

    IF_ID_bus : IF_ID port map (clk => clk, rst_n => rst_n, instr_in => d_instr_word, instr_out => ID_instr, pcplus4_in => d_pcplus4, pcplus4_out => ID_pcplus4, 
                                pc_in => d_pc_out, pc_out => ID_pc, rs1 => ID_rs1, rs2 => ID_rs2, funct3 => ID_funct3, PCSrc => c_PCSrc, Stall => c_stall_out);

    H_Unit_inst : H_Unit port map (ID_Instr => ID_instr, EX_Instr => EX_instr, EX_MemRead => EX_mem_read, load_stall => L_stall, load_nop => L_nop);

    control_inst : control port map (instruction => ID_instr, BranchCond => c_branch_out, Stall_in => c_stall_in, Load_Stall_in => L_stall, NOP_in => c_NOP_in,
                                    Jump => c_jump, Lui => c_lui, PCSrc => c_PCSrc, RegWrite => c_reg_write, ALUSrc1 => c_alu_src1, ALUSrc2 => c_alu_src2,
                                    ALUOp => c_alu_op, MemWrite => c_mem_write, MemRead => c_mem_read, MemToReg => c_mem_to_reg, 
                                    CSRSrc => c_csr, M_valid => c_M_valid, Stall_out => c_stall_out, NOP_out => c_NOP_out);

    RF_inst : regfile port map (clk => clk, rst_n => rst_n, RegWrite => c_reg_write, rs1 => ID_rs1, rs2 => ID_rs2, 
                                rd => WB_rd, datain => d_reg_file_datain, regA => d_regA, regB => d_regB);

    brcmp_inst : branch_cmp port map (inputA => d_regA, inputB => d_regB, cond => ID_funct3, result => c_branch_out);

    immgen_inst : immgen port map (instruction => ID_instr, immediate => d_immediate);

    brnch_add_inst: branch_adder port map (imm => d_immediate, pc => ID_pc, result => d_brnch_add);

    ID_EX_bus : ID_EX port map (clk => clk, rst_n => rst_n, instr_in => ID_instr, instr_out => EX_instr, pcplus4_in => ID_pcplus4,
                                pcplus4_out => EX_pcplus4, pc_in => ID_pc, pc_out => EX_pc, regA_in => d_regA, regB_in => d_regB, regA_out => EX_regA,
                                regB_out => EX_regB, imm_in => d_immediate, imm_out => EX_imm, Jump_in => c_jump, Lui_in => c_lui, 
                                RegWrite_in => c_reg_write, ALUSrc1_in => c_alu_src1, ALUSrc2_in => c_alu_src2, ALUOp_in => c_alu_op, MemWrite_in => c_mem_write,
                                MemRead_in => c_mem_read, MemToReg_in => c_mem_to_reg, CSRSrc_in => c_csr, M_valid_in => c_M_Valid, Jump_out => EX_jump, Lui_out => EX_lui, 
                                RegWrite_out => EX_reg_write, ALUSrc1_out => EX_alu_src1, ALUSrc2_out => EX_alu_src2, ALUOp_out => EX_alu_op, MemWrite_out => EX_mem_write,
                                MemRead_out => EX_mem_read, MemToReg_out => EX_mem_to_reg, CSRSrc_out => EX_csr, M_valid_out => EX_M_valid, Stall => c_stall_out, NOP_load => L_nop);

    F_Unit_inst : F_Unit port map (MEM_Instr => MEM_instr, WB_Instr => WB_instr, EX_Instr => EX_instr, M_Instr => d_M_alu_instr_out, MEM_RegWrite => MEM_reg_write,
                                    WB_RegWrite => WB_reg_write, Mux1_inputA_sig => F_Mux1_inputA_sel, Mux2_inputA_sig => F_Mux2_inputA_sel, 
                                    Mux3_inputB_sig => F_Mux3_inputB_sel, Mux4_inputB_sig => F_Mux4_inputB_sel, M_Mux1_inputA_sig=> M_Mux1_inputA_sel,
                                    M_Mux2_inputA_sig => M_Mux2_inputA_sel, M_Mux3_inputB_sig => M_Mux3_inputB_sel, M_Mux4_inputB_sig => M_Mux4_inputB_sel);

    F_mux1 : mux2to1 port map (sel => F_Mux1_inputA_sel, input0 => EX_regA, input1 => d_reg_file_datain, output => Res_Mux_1);
    F_mux2 : mux2to1 port map (sel => F_Mux2_inputA_sel, input0 => Res_Mux_1, input1 => MEM_ALU_res, output => Res_Mux_2);
    F_mux3 : mux2to1 port map (sel => F_Mux3_inputB_sel, input0 => EX_regB, input1 => d_reg_file_datain, output => Res_Mux_3);
    F_mux4 : mux2to1 port map (sel => F_Mux4_inputB_sel, input0 => Res_Mux_3, input1 => MEM_ALU_res, output => Res_Mux_4);

    lui_mux : mux2to1 port map (sel => EX_lui, input0 => EX_pc, input1 => d_zero, output => d_lui_mux_out);

    CSR_inst : csr port map (clk => clk, rst_n => rst_n, csr_sig => EX_csr, csr_sig_wb => WB_csr, instr_first => EX_instr, instr_second => WB_instr, 
                            csr_data_in => d_reg_file_datain, csr_out => d_csr_out);

    csr_mux : mux2to1 port map (sel => EX_csr, input0 => Res_Mux_4, input1 => d_csr_out, output => d_csr_mux_out);

    alu_src1_mux : mux2to1 port map (sel => EX_alu_src1, input0 => Res_Mux_2, input1 => d_lui_mux_out, output => d_alu_src1);
    alu_src2_mux : mux2to1 port map (sel => EX_alu_src2, input0 => d_csr_mux_out, input1 => EX_imm, output => d_alu_src2);

    alu_inst : alu port map (inputA => d_alu_src1, inputB => d_alu_src2, ALUop => EX_alu_op, result => d_alu_out);

    M_alu_inst: M_alu port map (clk => clk, M_valid => EX_M_valid, inputA => d_alu_src1, inputB => d_alu_src2, ALUop => EX_alu_op, instr_in => EX_instr, 
                                instr_out => d_M_alu_instr_out, result => d_M_alu_val_out, Stall => c_stall_in, NOP => c_NOP_in,
                                M_Mux1_inputA_sig=> M_Mux1_inputA_sel, M_Mux2_inputA_sig => M_Mux2_inputA_sel, M_Mux3_inputB_sig => M_Mux3_inputB_sel, 
                                M_Mux4_inputB_sig => M_Mux4_inputB_sel, WB_data => d_reg_file_datain, MEM_data => MEM_ALU_res);

    ALU_val_mux : mux2to1 port map (sel => EX_M_valid(0), input0 => d_alu_out, input1 => d_M_alu_val_out, output => d_M_alu_val_mux_out);
    ALU_instr_mux : mux2to1 port map (sel => EX_M_valid(0), input0 => EX_instr, input1 => d_M_alu_instr_out, output => d_M_alu_inst_mux_out);
    

    EX_MEM_bus : EX_MEM port map (clk => clk, rst_n => rst_n, instr_in => d_M_alu_inst_mux_out, instr_out => MEM_instr, pcplus4_in => EX_pcplus4,
                                pcplus4_out => MEM_pcplus4, ALU_res_in => d_M_alu_val_mux_out, ALU_res_out => MEM_ALU_res, regB_in => Res_Mux_4,
                                regB_out => MEM_regB, byte_mask => MEM_byte_mask, sign_ext_n => MEM_sign_ext_n, Jump_in => EX_jump,
                                RegWrite_in => EX_reg_write, MemWrite_in => EX_mem_write, MemRead_in => EX_mem_read, MemToReg_in => EX_mem_to_reg, 
                                CSRSrc_in => EX_csr, Jump_out => MEM_jump, RegWrite_out => MEM_reg_write, MemWrite_out => MEM_mem_write,
                                MemRead_out => MEM_mem_read, MemToReg_out => MEM_mem_to_reg, CSRSrc_out => MEM_csr, NOP => c_NOP_out);

    ldmb_inst : lmb port map (proc_addr => MEM_ALU_res, proc_data_send => MEM_regB,
                               proc_data_receive => d_data_mem_out, proc_byte_mask => MEM_byte_mask,
                               proc_sign_ext_n => MEM_sign_ext_n, proc_mem_write => MEM_mem_write, proc_mem_read => MEM_mem_read,
                               mem_addr => dmem_addr, mem_datain => dmem_datain, mem_dataout => dmem_dataout,
                               mem_wen => dmem_wen, mem_ben => dmem_ben);

    MEM_WB_bus : MEM_WB port map (clk => clk, rst_n => rst_n, instr_in => MEM_instr, instr_out => WB_instr, rd => WB_rd, pcplus4_in => MEM_pcplus4,
                                pcplus4_out => WB_pcplus4, ALU_res_in => MEM_ALU_res, ALU_res_out => WB_ALU_res, data_mem_in => d_data_mem_out,
                                data_mem_out => WB_data_mem, Jump_in => MEM_jump, RegWrite_in => MEM_reg_write, MemToReg_in => MEM_mem_to_reg, 
                                CSRSrc_in => MEM_csr, Jump_out => WB_jump, RegWrite_out => WB_reg_write,
                                MemToReg_out => WB_mem_to_reg, CSRSrc_out => WB_csr);    

    mem_mux : mux2to1 port map (sel => WB_mem_to_reg, input0 => WB_ALU_res, input1 => WB_data_mem, output => d_mem_mux_out);

    write_back_mux : mux2to1 port map (sel => WB_jump, input0 => d_mem_mux_out, input1 => WB_pcplus4, output => d_reg_file_datain);

    d_zero <= (others=>'0');


end architecture;