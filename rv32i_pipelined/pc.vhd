-- Program Counter (PC) Register

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.archer_pkg.all;

entity pc is
    port (
        clk : in std_logic;
        rst_n : in std_logic;
        datain : in std_logic_vector(XLEN-1 downto 0);
        dataout : out std_logic_vector(XLEN-1 downto 0)
    );
end pc;

architecture rtl of pc is
begin
    process (clk, rst_n) is
        variable pcval : std_logic_vector(XLEN-1 downto 0);
    begin
        if rst_n = '0' then
            pcval := (others=>'0');
        elsif rising_edge(clk) then
            pcval := datain;
        end if;
        dataout <= pcval;
    end process;
end architecture;