-- CSR Unit

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.archer_pkg.all;

entity csr is
    port (
        clk : in std_logic;
        rst_n : in std_logic;
        csr_sig : in std_logic;
        csr_sig_wb: in std_logic;
        instr_first: in std_logic_vector (XLEN-1 downto 0);
        instr_second: in std_logic_vector (XLEN-1 downto 0);
        csr_data_in: in std_logic_vector (XLEN-1 downto 0);
        csr_out : out std_logic_vector (XLEN-1 downto 0)
    );
end csr;

architecture rtl of csr is
    type csr_file is array (0 to 3) of std_logic_vector (XLEN-1 downto 0);
    
    signal CSR_mem : csr_file := (X"00000001", X"00000000",X"00000000", X"00000000");
begin

    process  is
        variable CSR_cycles : unsigned(XLEN-1 downto 0) := (others => '1');
        variable CSR_cycles_h : unsigned(XLEN-1 downto 0) := (others => '0');
        variable CSR_instret : unsigned(XLEN-1 downto 0) := (others => '0');
        variable CSR_instret_h : unsigned(XLEN-1 downto 0) := (others => '0');
    begin
    
        if rst_n = '0' then
            CSR_cycles := (others => '1');
            CSR_cycles_h := (others => '0');
            CSR_instret := (others => '0');
            CSR_instret_h := (others => '0');
            CSR_mem <= (X"00000001", X"00000000",X"00000000", X"00000000");
        elsif rising_edge(clk) then
            CSR_cycles := unsigned(CSR_mem(0)) + 1;
            CSR_mem(0) <= std_logic_vector(CSR_cycles);
            if CSR_cycles = X"00000000" then
                CSR_cycles_h := unsigned(CSR_mem(1)) + 1;
                CSR_mem(1) <= std_logic_vector(CSR_cycles_h);
            end if;
        end if;
        
        if csr_sig = '1' then 
        csr_out <= CSR_mem(to_integer(unsigned(instr_first(XLEN-1 downto 20))));
    end if;
    if falling_edge(clk) and instr_second(19 downto 15) /= "00000" and csr_sig_wb = '1' then
        CSR_mem(to_integer(unsigned(instr_second(XLEN-1 downto 20)))) <= csr_data_in;
    end if;
        if rising_edge(clk) then
        wait for 0.0001 ns;
	if (instr_second(0)='0' or instr_second(0)='1') and instr_second /= X"00000013" and instr_second /= X"00000000" then
		CSR_instret := unsigned(CSR_mem(2)) +1;
		CSR_mem(2) <= std_logic_vector(CSR_instret);
		if CSR_instret = X"00000000" then
		    CSR_instret_h := unsigned(CSR_mem(3)) +1;
		    CSR_mem(3) <= std_logic_vector(CSR_instret_h);
		end if;
	end if;
	end if;
     wait on clk, rst_n, csr_sig, csr_sig_wb, instr_first, instr_second, csr_data_in;  
    end process;
    

end architecture;
