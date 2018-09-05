-- ----------------------------------------------------------------------------
-- FILE:          vctcxo_tamer_log_tb.vhd
-- DESCRIPTION:   
-- DATE:          Feb 13, 2014
-- AUTHOR(s):     Lime Microsystems
-- REVISIONS:
-- ----------------------------------------------------------------------------

-- ----------------------------------------------------------------------------
-- NOTES:
-- ----------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

-- ----------------------------------------------------------------------------
-- Entity declaration
-- ----------------------------------------------------------------------------
entity vctcxo_tamer_log_tb is
end vctcxo_tamer_log_tb;

-- ----------------------------------------------------------------------------
-- Architecture
-- ----------------------------------------------------------------------------

architecture tb_behave of vctcxo_tamer_log_tb is
   constant clk0_period    : time := 10 ns;
   constant clk1_period    : time := 10 ns; 
   --signals
   signal clk0,clk1        : std_logic;
   signal reset_n          : std_logic; 
   
   signal dut0_pps_1s_count_v    : std_logic;
   signal dut0_uart_data_in_stb  : std_logic;
   signal dut0_uart_data_in_ack  : std_logic;
  
begin 
  
      clock0: process is
   begin
      clk0 <= '0'; wait for clk0_period/2;
      clk0 <= '1'; wait for clk0_period/2;
   end process clock0;

      clock: process is
   begin
      clk1 <= '0'; wait for clk1_period/2;
      clk1 <= '1'; wait for clk1_period/2;
   end process clock;
   
      res: process is
   begin
      reset_n <= '0'; wait for 20 ns;
      reset_n <= '1'; wait;
   end process res;
   
   
   process is 
   begin
      dut0_pps_1s_count_v <= '0'; wait until reset_n = '1';
      wait until rising_edge(clk0);
      dut0_pps_1s_count_v <= '1';
      wait until rising_edge(clk0);
      dut0_pps_1s_count_v <= '0';
      wait;
   end process;
   
   process is 
   begin  
      dut0_uart_data_in_ack <= '0';
      wait until rising_edge(dut0_uart_data_in_stb);
      wait until rising_edge(clk0);
      dut0_uart_data_in_ack <= '1';
      wait until rising_edge(clk0);
   end process;
      -- design under test  
      
   dut0 : entity work.vctcxo_tamer_log
   port map(
      clk                  => clk0,
      reset_n              => reset_n,
         
      --Data to log  
      pps_1s_error         => x"0000_0001",
      pps_10s_error        => x"0000_0002",
      pps_100s_error       => x"0000_0003",
      accuracy             => x"3",
      state                => x"1",
      dac_tuned_val        => x"00FE",
      pps_1s_count_v       => dut0_pps_1s_count_v,
      pps_10s_count_v      => dut0_pps_1s_count_v,
      pps_100s_count_v     => '0',
      fan_ctrl_in          => '1',
      
      --To uart module
      uart_data_in         => open,
      uart_data_in_stb     => dut0_uart_data_in_stb,
      uart_data_in_ack     => dut0_uart_data_in_ack
     
      );

end tb_behave;

