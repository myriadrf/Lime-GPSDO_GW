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
      CP_DCD            : in     std_logic;
      CP_RI_CLK         : in     std_logic;
      CP_CTS            : in     std_logic;
      CP_RTS            : in     std_logic;
      CP_RXD            : out    std_logic;
      CP_TXD            : in     std_logic;
      CP_DSR            : in     std_logic;
      CP_DTR            : in     std_logic;
      CP_GPIO           : in     std_logic_vector(6 downto 0);      
      -- GNSS
      GNSS_OFF          : in     std_logic;
      GNSS_ANT_DET      : in     std_logic;
      GNSS_ANT_OK       : in     std_logic;
      GNSS_RESET        : out    std_logic := '1';
      GNSS_TPULSE       : in     std_logic;
      GNSS_EXTINT       : in     std_logic := '0';
      GNSS_UART_TX      : in     std_logic;
      GNSS_UART_RX      : out    std_logic;
      GNSS_DDC_SDA      : inout  std_logic;
      GNSS_DDC_SCL      : inout  std_logic;
      -- ----------------------------------------------------------------------------
      -- External communication interfaces
         -- FPGA_SPI1
      FPGA_SPI1_SCLK    : out    std_logic;
      FPGA_SPI1_MOSI    : out    std_logic;    
      FPGA_SPI1_DAC_SS  : out    std_logic;
         -- FPGA_SPI2
      FPGA_SPI2_SCLK    : out    std_logic;
      FPGA_SPI2_MOSI    : out    std_logic;
      FPGA_SPI2_MISO    : in     std_logic;      
      FPGA_SPI2_FLASH_SS: out    std_logic;
         -- FPGA I2C
      FPGA_I2C_SCL      : inout  std_logic;
      FPGA_I2C_SDA      : inout  std_logic;
         -- External I2C
      EXT_I2C_SCL       : inout  std_logic;
      EXT_I2C_SDA       : inout  std_logic;
         -- External UART
      EXT_UART_RX       : in     std_logic;
      EXT_UART_TX       : out    std_logic;
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
      FPGA_BTN          : in     std_logic;
         --GPIO 
      FPGA_GPIO         : inout  std_logic_vector(7 downto 0);
         -- Time pulse
      FPGA_TPULSE       : out    std_logic;
         -- Temp sensor
      LM75_OS           : in     std_logic;
         -- Bill of material version
      BOM_VER           : in     std_logic_vector(3 downto 0)
   );
end lime_gpsdo_top;

-- ----------------------------------------------------------------------------
-- Architecture
-- ----------------------------------------------------------------------------
architecture arch of lime_gpsdo_top is
--declare signals,  components here
signal inst0_beat                : std_logic;
signal reset_n                   : std_logic;

signal inst0_fpga_spi_MISO       : std_logic;
signal inst0_fpga_spi_MOSI       : std_logic;
signal inst0_fpga_spi_SCLK       : std_logic;
signal inst0_fpga_spi_SS_n       : std_logic_vector(1 downto 0);

--inst1
signal inst1_avm_m0_address      : std_logic_vector(7 downto 0);
signal inst1_avm_m0_read         : std_logic;
signal inst1_avm_m0_write        : std_logic;
signal inst1_avm_m0_writedata    : std_logic_vector(7 downto 0);
signal inst1_avm_m0_clk_clk      : std_logic;
signal inst1_avm_m0_reset_reset  : std_logic;
signal inst1_uart_txd            : std_logic;

--inst2
signal inst2_sdout               : std_logic;
signal inst2_en                  : std_logic;
signal inst2_mm_rd_data          : std_logic_vector(7 downto 0);
signal inst2_mm_rd_datav         : std_logic;
signal inst2_mm_wait_req         : std_logic;
signal inst2_mm_irq              : std_logic;
signal inst2_uart_tx             : std_logic;
signal inst2_fpga_led_g          : std_logic;
signal inst2_fpga_led_r          : std_logic;

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
-- NIOS CPU instance
-- ----------------------------------------------------------------------------
   inst0_fpga_spi_MISO <= inst2_sdout;

   inst1_nios_cpu : entity work.nios_cpu
   port map(
      clk100               => CLK50_FPGA,
      exfifo_if_d          => (others=>'0'),
      exfifo_if_rd         => open,
      exfifo_if_rdempty    => '1',
      exfifo_of_d          => open,
      exfifo_of_wr         => open,
      exfifo_of_wrfull     => '0',
      exfifo_rst           => open,
      leds                 => open,
      lms_ctr_gpio         => open,
      fpga_spi_MISO        => inst0_fpga_spi_MISO,
      fpga_spi_MOSI        => inst0_fpga_spi_MOSI,
      fpga_spi_SCLK        => inst0_fpga_spi_SCLK,
      fpga_spi_SS_n        => inst0_fpga_spi_SS_n,
      dac_spi_SS_n         => FPGA_SPI1_DAC_SS,
      switch               => (others=>'1'),
      uart_rxd             => '1',
      uart_txd             => inst1_uart_txd,
      i2c_scl              => FPGA_I2C_SCL,
      i2c_sda              => FPGA_I2C_SDA,
      flash_spi_MISO       => FPGA_SPI2_MISO,
      flash_spi_MOSI       => FPGA_SPI2_MOSI,
      flash_spi_SCLK       => FPGA_SPI2_SCLK,
      flash_spi_SS_n       => FPGA_SPI2_FLASH_SS,
      vctcxo_tune_en       => inst2_en,
      vctcxo_irq           => inst2_mm_irq,
      avm_m0_address       => inst1_avm_m0_address,
      avm_m0_read          => inst1_avm_m0_read,
      avm_m0_waitrequest   => inst2_mm_wait_req,
      avm_m0_readdata      => inst2_mm_rd_data,
      avm_m0_readdatavalid => inst2_mm_rd_datav,
      avm_m0_write         => inst1_avm_m0_write,
      avm_m0_writedata     => inst1_avm_m0_writedata,
      avm_m0_clk_clk       => inst1_avm_m0_clk_clk,
      avm_m0_reset_reset   => inst1_avm_m0_reset_reset
   );
   
   
   inst2_limegnss_gpio_top : entity work.limegnss_gpio_top
   generic map( 
      UART_BAUD_RATE          => 9600,
      VCTCXO_CLOCK_FREQUENCY  => 30720000,
      MM_CLOCK_FREQUENCY      => 50000000
   )
   port map(
      areset_n          => reset_n,
      -- SPI interface
      -- Address and location of SPI memory module
      -- Will be hard wired at the top level
      tamercfg_maddress => "0000000111",
      gnsscfg_maddress  => "0000001000",
      -- Serial port IOs
      sdin              => inst0_fpga_spi_MOSI,    -- Data in
      sclk              => inst0_fpga_spi_SCLK,    -- Data clock
      sen               => inst0_fpga_spi_SS_n(1), -- Enable signal (active low)
      sdout             => inst2_sdout,            -- Data out 
      -- Signals coming from the pins or top level serial interface
      lreset            => reset_n,    -- Logic reset signal, resets logic cells only  (use only one reset)
      mreset            => reset_n,    -- Memory reset signal, resets configuration memory only (use only one reset)
      vctcxo_clk        => CLK0_OUT,   -- Clock from VCTCXO       
      --LimeGNSS-GPIO pins
      gnss_tx           => open,   
      gnss_rx           => GNSS_UART_TX,  
      gnss_tpulse       => GNSS_TPULSE,   
      gnss_fix          => '0',           
      fpga_led_g        => inst2_fpga_led_g,
      fpga_led_r        => inst2_fpga_led_r, 
      -- NIOS PIO
      en                => inst2_en,     
      -- NIOs  Avalon-MM Interface (External master)
      mm_clock          => inst1_avm_m0_clk_clk,
      mm_reset          => inst1_avm_m0_reset_reset,
      mm_rd_req         => inst1_avm_m0_read,
      mm_wr_req         => inst1_avm_m0_write,
      mm_addr           => inst1_avm_m0_address,
      mm_wr_data        => inst1_avm_m0_writedata,
      mm_rd_data        => inst2_mm_rd_data,
      mm_rd_datav       => inst2_mm_rd_datav,
      mm_wait_req       => inst2_mm_wait_req,
      -- Avalon Interrupts
      mm_irq            => inst2_mm_irq,
      
      -- Testing (UART logger)
      fan_ctrl_in       => '0',
      uart_tx           => inst2_uart_tx
      
   );   
-- ----------------------------------------------------------------------------
-- Output ports
-- ----------------------------------------------------------------------------  
   FPGA_SPI1_MOSI    <= inst0_fpga_spi_MOSI;
   FPGA_SPI1_SCLK    <= inst0_fpga_spi_SCLK;
   
   --FPGA_LED
   FPGA_LED1_G       <= not inst2_fpga_led_g;
   FPGA_LED1_R       <= not inst2_fpga_led_r;
   FPGA_LED2_G       <= '1';
   FPGA_LED2_R       <= '1';
   FPGA_LED3_G       <= '1';
   FPGA_LED3_R       <= '1';
   
   --USB UART
   CP_RXD            <= GNSS_UART_TX   when FPGA_SW(0) = '0' else  
                        inst1_uart_txd when FPGA_SW(1) = '0' else inst2_uart_tx;                 
   --GNSS                                           
   GNSS_UART_RX      <= CP_TXD         when FPGA_SW(0) = '0' else '1';
   
   --MISC
   FPGA_TPULSE       <= GNSS_TPULSE;
   
   
end arch;   


