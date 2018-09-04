-- ----------------------------------------------------------------------------
-- FILE:          lime_gpsdo_top.vhd
-- DESCRIPTION:   Top file for Lime-GPSDO v1.0 board
-- DATE:          10:14 AM Tuesday, September 4, 2018
-- AUTHOR(s):     Lime Microsystems
-- REVISIONS:
-- ----------------------------------------------------------------------------

-- ----------------------------------------------------------------------------
--NOTES:
-- ----------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

-- ----------------------------------------------------------------------------
-- Entity declaration
-- ----------------------------------------------------------------------------
entity lime_gpsdo_top is
   port (
      -- ----------------------------------------------------------------------------
      -- External GND pin for reset
      EXT_GND           : in     std_logic;
      -- ----------------------------------------------------------------------------
      -- Clock sources
      CLK50_FPGA        : in     std_logic;
      CLK0_OUT          : in     std_logic;
      -- ----------------------------------------------------------------------------
      -- USB UART
      CP_RXD            : out    std_logic;
      CP_TXD            : in     std_logic;
      -- GNSS
      GNSS_RESET        : out    std_logic := '1';
      GNSS_TPULSE       : in     std_logic;
      GNSS_UART_TX      : in     std_logic;
      GNSS_UART_RX      : out    std_logic;
      -- ----------------------------------------------------------------------------
      -- General periphery
         -- Switch
      FPGA_SW           : in     std_logic_vector(3 downto 0);
         -- LED
      FPGA_LED1_G       : out    std_logic := '1';
      FPGA_LED1_R       : out    std_logic := '1';
      FPGA_LED2_G       : out    std_logic := '1';
      FPGA_LED2_R       : out    std_logic := '1';
      FPGA_LED3_G       : out    std_logic := '1';
      FPGA_LED3_R       : out    std_logic := '1';      
         -- Button
      FPGA_BTN          : in     std_logic
   );
end lime_gpsdo_top;

-- ----------------------------------------------------------------------------
-- Architecture
-- ----------------------------------------------------------------------------
architecture arch of lime_gpsdo_top is
--declare signals,  components here
signal inst0_beat : std_logic;
signal reset_n    : std_logic;

  
begin

-- ----------------------------------------------------------------------------
-- Reset logic
-- ---------------------------------------------------------------------------- 
   reset_n <= FPGA_BTN;

-- ----------------------------------------------------------------------------
-- General periphery
-- ----------------------------------------------------------------------------    
   inst0_alive : entity work.alive
   port map(
      clk      => CLK0_OUT,
      reset_n  => reset_n,
      beat     => inst0_beat
   );
   
-- ----------------------------------------------------------------------------
-- Output ports
-- ----------------------------------------------------------------------------  

   FPGA_LED1_G <= inst0_beat;
   FPGA_LED1_R <= '1';
   FPGA_LED2_G <= GNSS_TPULSE;
   FPGA_LED2_R <= '1';
   
   
   CP_RXD         <= GNSS_UART_TX;
   GNSS_UART_RX   <= CP_TXD;
   
   
end arch;   


