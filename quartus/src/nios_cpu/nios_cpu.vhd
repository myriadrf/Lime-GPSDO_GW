-- ----------------------------------------------------------------------------	
-- FILE: 	nios_cpu.vhd
-- DESCRIPTION:	NIOS CPU top level
-- DATE:	Feb 12, 2016
-- AUTHOR(s):	Lime Microsystems
-- REVISIONS:
-- ----------------------------------------------------------------------------	
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

-- ----------------------------------------------------------------------------
-- Entity declaration
-- ----------------------------------------------------------------------------
entity nios_cpu is
   port (
      clk100               : in    std_logic;
      reset_n              : in    std_logic;
      exfifo_if_d          : in    std_logic_vector(31 downto 0);
      exfifo_if_rd         : out   std_logic;
      exfifo_if_rdempty    : in    std_logic;
      exfifo_of_d          : out   std_logic_vector(31 downto 0);
      exfifo_of_wr         : out   std_logic;
      exfifo_of_wrfull     : in    std_logic;
      exfifo_rst           : out   std_logic;
      leds                 : out   std_logic_vector(7 downto 0);
      lms_ctr_gpio         : out   std_logic_vector(3 downto 0);
      fpga_spi_MISO        : in    std_logic;
      fpga_spi_MOSI        : out   std_logic;
      fpga_spi_SCLK        : out   std_logic;
      fpga_spi_SS_n        : out   std_logic_vector(1 downto 0);
      dac_spi_SS_n         : buffer   std_logic;
      switch               : in    std_logic_vector(7 downto 0);
      uart_rxd             : in    std_logic;
      uart_txd             : out   std_logic;
      i2c_scl              : inout std_logic;
      i2c_sda              : inout std_logic;
      flash_spi_MISO       : in    std_logic;
      flash_spi_MOSI       : out   std_logic;
      flash_spi_SCLK       : out   std_logic;
      flash_spi_SS_n       : out   std_logic;
      vctcxo_tune_en       : in    std_logic;
      vctcxo_irq           : in    std_logic;
      avm_m0_address       : out   std_logic_vector(7 downto 0);                     --                           avm_m0.address
      avm_m0_read          : out   std_logic;                                        --                                 .read
      avm_m0_waitrequest   : in    std_logic                     := '0';             --                                 .waitrequest
      avm_m0_readdata      : in    std_logic_vector(7 downto 0)  := (others => '0'); --                                 .readdata
      avm_m0_readdatavalid : in    std_logic                     := '0';             --                                 .readdatavalid
      avm_m0_write         : out   std_logic;                                        --                                 .write
      avm_m0_writedata     : out   std_logic_vector(7 downto 0);                     --                                 .writedata
      avm_m0_clk_clk       : out   std_logic;                                        --                       avm_m0_clk.clk
      avm_m0_reset_reset   : out   std_logic 
      
   );
end nios_cpu;

-- ----------------------------------------------------------------------------
-- Architecture
-- ----------------------------------------------------------------------------
architecture arch of nios_cpu is
--declare signals,  components here
   
   signal vctcxo_tune_en_sync : std_logic;
   signal vctcxo_irq_sync     : std_logic;
   
   signal dac_spi_MOSI, dac_spi_SCLK: std_logic;
   signal fpga_spi_MOSI_int, fpga_spi_SCLK_int: std_logic;
   
   signal vctcxo_tamer_0_irq_out_irq   : std_logic;
   signal vctcxo_tamer_0_ctrl_export   : std_logic_vector(3 downto 0);


   
   component lms_ctr is
      port (
         clk_clk                                 : in    std_logic                    := 'X';             -- clk
         reset_reset_n                           : in    std_logic;
         exfifo_if_d_export                      : in    std_logic_vector(31 downto 0) := (others => 'X'); -- export
         exfifo_if_rd_export                     : out   std_logic;                                       -- export
         exfifo_if_rdempty_export                : in    std_logic                    := 'X';             -- export
         exfifo_of_d_export                      : out   std_logic_vector(31 downto 0);                    -- export
         exfifo_of_wr_export                     : out   std_logic;                                       -- export
         exfifo_of_wrfull_export                 : in    std_logic                    := 'X';             -- export
         exfifo_rst_export                       : out   std_logic;                                       -- export
         leds_external_connection_export         : out   std_logic_vector(7 downto 0);                    -- export
         lms_ctr_gpio_external_connection_export : out   std_logic_vector(3 downto 0);                    -- export
         dac_spi_ext_MISO                        : in    std_logic                    := 'X';             -- MISO
         dac_spi_ext_MOSI                        : out   std_logic;                                       -- MOSI
         dac_spi_ext_SCLK                        : out   std_logic;                                       -- SCLK
         dac_spi_ext_SS_n                        : out   std_logic;                                       -- SS_n
         fpga_spi_ext_MISO                       : in    std_logic                    := 'X';             -- MISO
         fpga_spi_ext_MOSI                       : out   std_logic;                                       -- MOSI
         fpga_spi_ext_SCLK                       : out   std_logic;                                       -- SCLK
         fpga_spi_ext_SS_n                       : out   std_logic_vector(1 downto 0);                    -- SS_n 
         switch_external_connection_export       : in    std_logic_vector(7 downto 0) := (others => 'X'); -- export
         uart_external_connection_rxd            : in    std_logic                    := 'X';             -- rxd
         uart_external_connection_txd            : out   std_logic;                                       -- txd
         i2c_scl_export                          : inout std_logic                    := 'X';             -- export
         i2c_sda_export                          : inout std_logic                    := 'X';             -- export
         flash_spi_MISO                          : in    std_logic                     := 'X';             -- MISO
         flash_spi_MOSI                          : out   std_logic;                                        -- MOSI
         flash_spi_SCLK                          : out   std_logic;                                        -- SCLK
         flash_spi_SS_n                          : out   std_logic;                                        -- SS_n
         vctcxo_tamer_0_ctrl_export              : in    std_logic_vector(3 downto 0)  := (others=>'0');             --            vctcxo_tamer_0_irq_in.export
         avm_m0_address                          : out   std_logic_vector(7 downto 0);                     --                           avm_m0.address
         avm_m0_read                             : out   std_logic;                                        --                                 .read
         avm_m0_waitrequest                      : in    std_logic                     := '0';             --                                 .waitrequest
         avm_m0_readdata                         : in    std_logic_vector(7 downto 0)  := (others => '0'); --                                 .readdata
         avm_m0_write                            : out   std_logic;                                        --                                 .write
         avm_m0_writedata                        : out   std_logic_vector(7 downto 0);                     --                                 .writedata
         avm_m0_readdatavalid                    : in    std_logic                     := '0';             --                                 .readdatavalid
         avm_m0_clk_clk                          : out   std_logic;                                        --                       avm_m0_clk.clk
         avm_m0_reset_reset                      : out   std_logic                                          --                     avm_m0_reset.reset 

      );
   end component lms_ctr;
  
begin

   sync_reg0 : entity work.sync_reg 
   port map(clk100, '1', vctcxo_tune_en, vctcxo_tune_en_sync);

   sync_reg1 : entity work.sync_reg 
   port map(clk100, '1', vctcxo_irq, vctcxo_irq_sync);

   u0 : component lms_ctr
      port map (
         clk_clk                                   => clk100,
         reset_reset_n                             => reset_n,
         exfifo_if_d_export                        => exfifo_if_d,
         exfifo_if_rd_export                       => exfifo_if_rd,
         exfifo_if_rdempty_export                  => exfifo_if_rdempty,
         exfifo_of_d_export                        => exfifo_of_d,
         exfifo_of_wr_export                       => exfifo_of_wr,
         exfifo_of_wrfull_export                   => exfifo_of_wrfull,
         exfifo_rst_export                         => exfifo_rst,
         leds_external_connection_export           => leds,
         lms_ctr_gpio_external_connection_export   => lms_ctr_gpio,
         dac_spi_ext_MISO                          => '0',
         dac_spi_ext_MOSI                          => dac_spi_MOSI,
         dac_spi_ext_SCLK                          => dac_spi_SCLK,
         dac_spi_ext_SS_n                          => dac_spi_SS_n,
         fpga_spi_ext_MISO                         => fpga_spi_MISO,
         fpga_spi_ext_MOSI                         => fpga_spi_MOSI_int,
         fpga_spi_ext_SCLK                         => fpga_spi_SCLK_int,
         fpga_spi_ext_SS_n                         => fpga_spi_SS_n,
         switch_external_connection_export         => switch,
         uart_external_connection_rxd              => uart_rxd,
         uart_external_connection_txd              => uart_txd,
         i2c_scl_export                            => i2c_scl,
         i2c_sda_export                            => i2c_sda,
         flash_spi_MISO                            => flash_spi_MISO,
         flash_spi_MOSI                            => flash_spi_MOSI,
         flash_spi_SCLK                            => flash_spi_SCLK,
         flash_spi_SS_n                            => flash_spi_SS_n,
         vctcxo_tamer_0_ctrl_export                => vctcxo_tamer_0_ctrl_export,
         avm_m0_address                            => avm_m0_address,
         avm_m0_read                               => avm_m0_read,
         avm_m0_waitrequest                        => avm_m0_waitrequest,
         avm_m0_readdatavalid                      => avm_m0_readdatavalid,
         avm_m0_readdata                           => avm_m0_readdata,
         avm_m0_write                              => avm_m0_write,
         avm_m0_writedata                          => avm_m0_writedata,
         avm_m0_clk_clk                            => avm_m0_clk_clk,
         avm_m0_reset_reset                        => avm_m0_reset_reset
      );
      
      
      -- SPI switch to select between AD5601 and the rest of slaves.
      -- This is neccessary, while ADF4002 CLOCK_PHASE = 0, while AD5601 CLOCK_PHASE = 1
      fpga_spi_MOSI <= fpga_spi_MOSI_int when dac_spi_SS_n = '1' else dac_spi_MOSI;
      fpga_spi_SCLK <= fpga_spi_SCLK_int when dac_spi_SS_n = '1' else dac_spi_SCLK;
      
      vctcxo_tamer_0_ctrl_export(0) <= vctcxo_tune_en_sync;
      vctcxo_tamer_0_ctrl_export(1) <= vctcxo_irq_sync;
      vctcxo_tamer_0_ctrl_export(2) <= '0';
      vctcxo_tamer_0_ctrl_export(3) <= '0';
      

end arch;   




