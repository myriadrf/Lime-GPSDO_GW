/*
 * main.c
 *
 *  Created on: Jan 22, 2016
 *      Author: zydrunas
 */


#include "io.h"
#include "system.h"
#include "unistd.h"
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <altera_avalon_spi.h>
#include "alt_types.h"
#include "math.h"

#include "LMS64C_protocol.h"
#include "lime_gpsdo_brd_v1r0.h"
#include "spi_flash_lib.h"
#include "i2c_opencores.h"
#include "vctcxo_tamer.h"


#define sbi(p,n) ((p) |= (1UL << (n)))
#define cbi(p,n) ((p) &= ~(1 << (n)))

//get info
#define FW_VER			1 //Initial version

#define SPI_NR_LMS7002M 0
#define SPI_NR_FPGA     1
#define SPI_NR_DAC      0
#define SPI_NR_FLASH    0

//CMD_PROG_MCU
#define PROG_EEPROM 1
#define PROG_SRAM	2
#define BOOT_MCU	3

///CMD_PROG_MCU
#define MCU_CONTROL_REG	0x02
#define MCU_STATUS_REG	0x03
#define MCU_FIFO_WR_REG	0x04

#define MAX_MCU_RETRIES	30
uint8_t MCU_retries;

uint8_t test, block, cmd_errors, glEp0Buffer_Rx[64], glEp0Buffer_Tx[64];
tLMS_Ctrl_Packet *LMS_Ctrl_Packet_Tx = (tLMS_Ctrl_Packet*)glEp0Buffer_Tx;
tLMS_Ctrl_Packet *LMS_Ctrl_Packet_Rx = (tLMS_Ctrl_Packet*)glEp0Buffer_Rx;

uint16_t dac_val = 30714;		//TCXO DAC value

signed short int converted_val = 300;

int flash_page = 0, flash_page_data_cnt = 0, flash_data_cnt_free = 0, flash_data_counter_to_copy = 0;
//FPGA conf
unsigned long int last_portion, current_portion, fpga_data;
unsigned char data_cnt;
unsigned char sc_brdg_data[4];
unsigned char flash_page_data[FLASH_PAGE_SIZE];
tBoard_Config_FPGA *Board_Config_FPGA = (tBoard_Config_FPGA*) flash_page_data;
unsigned long int fpga_byte;

// Used for MAX10 Flash programming
uint32_t CFM0StartAddress = 0x04A000;
uint32_t CFM0EndAddress   = 0x08BFFF;
uint32_t address = 0x0;
uint32_t byte = 0;
uint32_t byte1;
uint32_t word = 0x0;
uint8_t state, Flash = 0x0;

int boot_img_en = 0;



/**	This function checks if all blocks could fit in data field.
*	If blocks will not fit, function returns TRUE. */
unsigned char Check_many_blocks (unsigned char block_size)
{
	if (LMS_Ctrl_Packet_Rx->Header.Data_blocks > (sizeof(LMS_Ctrl_Packet_Tx->Data_field)/block_size))
	{
		LMS_Ctrl_Packet_Tx->Header.Status = STATUS_BLOCKS_ERROR_CMD;
		return 1;
	}
	else return 0;
	return 1;
}

/**
 * Gets 64 bytes packet from FIFO.
 */
void getFifoData(uint8_t *buf, uint8_t k)
{
	uint8_t cnt = 0;
	uint32_t* dest = (uint32_t*)buf;
	for(cnt=0; cnt<k/sizeof(uint32_t); ++cnt)
	{
		dest[cnt] = IORD(AV_FIFO_INT_0_BASE, 1);	// Read Data from FIFO
	};
}


/**
 * Gets led_pattern as parameter in order to write to the register.
 */
void set_led(unsigned char led_pattern)
{
    IOWR(LEDS_BASE, 0, led_pattern);               // writes register
}


/**
 * Configures LM75
 */
void Configure_LM75(void)
{
	int spirez;

	// OS polarity configuration
	spirez = I2C_start(I2C_OPENCORES_0_BASE, LM75_I2C_ADDR, 0);
	spirez = I2C_write(I2C_OPENCORES_0_BASE, 0x01, 0);				// Pointer = configuration register
	//spirez = I2C_start(I2C_OPENCORES_0_BASE, LM75_I2C_ADDR, 1);
	spirez = I2C_write(I2C_OPENCORES_0_BASE, 0x04, 1);				//Configuration value: OS polarity = 1, Comparator/int = 0, Shutdown = 0

	// THYST configuration
	spirez = I2C_start(I2C_OPENCORES_0_BASE, LM75_I2C_ADDR, 0);
	spirez = I2C_write(I2C_OPENCORES_0_BASE, 0x02, 0);				// Pointer = THYST register
	//spirez = I2C_start(I2C_OPENCORES_0_BASE, LM75_I2C_ADDR, 1);
	spirez = I2C_write(I2C_OPENCORES_0_BASE, 45, 0);				// Set THYST H
	spirez = I2C_write(I2C_OPENCORES_0_BASE,  0, 1);				// Set THYST L

	// TOS configuration
	spirez = I2C_start(I2C_OPENCORES_0_BASE, LM75_I2C_ADDR, 0);
	spirez = I2C_write(I2C_OPENCORES_0_BASE, 0x03, 0);				// Pointer = TOS register
	//spirez = I2C_start(I2C_OPENCORES_0_BASE, LM75_I2C_ADDR, 1);
	spirez = I2C_write(I2C_OPENCORES_0_BASE, 55, 0);				// Set TOS H
	spirez = I2C_write(I2C_OPENCORES_0_BASE,  0, 1);				// Set TOS L

}

//
void testEEPROM(void)
{
	int spirez;
	uint8_t converted_val;

	//EEPROM Test, RD from 0x0000
	spirez = I2C_start(I2C_OPENCORES_0_BASE, EEPROM_I2C_ADDR, 0);
	spirez = I2C_write(I2C_OPENCORES_0_BASE, 0x00, 0);
	spirez = I2C_write(I2C_OPENCORES_0_BASE, 0x00, 0);

	spirez = I2C_start(I2C_OPENCORES_0_BASE, EEPROM_I2C_ADDR, 1);
	converted_val = I2C_read(I2C_OPENCORES_0_BASE, 1);

	//WR
	spirez = I2C_start(I2C_OPENCORES_0_BASE, EEPROM_I2C_ADDR, 0);
	spirez = I2C_write(I2C_OPENCORES_0_BASE, 0x00, 0);
	spirez = I2C_write(I2C_OPENCORES_0_BASE, 0x01, 0);
	spirez = I2C_write(I2C_OPENCORES_0_BASE, 0x5A, 1);

	//EEPROM Test, RD from 0x0001
	spirez = I2C_start(I2C_OPENCORES_0_BASE, EEPROM_I2C_ADDR, 0);
	spirez = I2C_write(I2C_OPENCORES_0_BASE, 0x00, 0);
	spirez = I2C_write(I2C_OPENCORES_0_BASE, 0x01, 0);

	spirez = I2C_start(I2C_OPENCORES_0_BASE, EEPROM_I2C_ADDR, 1);
	converted_val = I2C_read(I2C_OPENCORES_0_BASE, 1);
}


void boot_from_flash(void)
{
	//set CONFIG_SEL overwrite to 1 and CONFIG_SEL to Image 0
	//IOWR(DUAL_BOOT_0_BASE, 1, 0x00000001);

	//set CONFIG_SEL overwrite to 1 and CONFIG_SEL to Image 1
	IOWR(DUAL_BOOT_0_BASE, 1, 0x00000003);

	/*wait while core is busy*/
	while(IORD(DUAL_BOOT_0_BASE, 3) == 1) {}

	//Trigger reconfiguration to selected Image
	IOWR(DUAL_BOOT_0_BASE, 0, 0x00000001);
}

/**
 *	@brief Function to control DAC for TCXO frequency control
 *	@param oe output enable control: 0 - output disabled, 1 - output enabled
 *	@param data pointer to DAC value (1 byte)
 */
void Control_TCXO_DAC (unsigned char oe, uint16_t *data) //controls DAC (AD5601)
{
	volatile int spirez;
	unsigned char DAC_data[3];

	if (oe == 0) //set DAC out to three-state
	{
		DAC_data[0] = 0x03; //POWER-DOWN MODE = THREE-STATE (PD[1:0]([17:16]) = 11)
		DAC_data[1] = 0x00;
		DAC_data[2] = 0x00; //LSB data

		spirez = alt_avalon_spi_command(DAC_SPI_BASE, SPI_NR_DAC, 3, DAC_data, 0, NULL, 0);
	}
	else //enable DAC output, set new val
	{
		DAC_data[0] = 0; //POWER-DOWN MODE = NORMAL OPERATION PD[1:0]([17:16]) = 00)
		DAC_data[1] = ((*data) >>8) & 0xFF;
		DAC_data[2] = ((*data) >>0) & 0xFF;

	    /* Update cached value of trim DAC setting */
	    vctcxo_trim_dac_value = (uint16_t) *data;
		spirez = alt_avalon_spi_command(DAC_SPI_BASE, SPI_NR_DAC, 3, DAC_data, 0, NULL, 0);


	}
}

/**
 * Main, what else? :)
 **/
int main()
{
    uint32_t* dest = (uint32_t*)glEp0Buffer_Tx;

    volatile int spirez;
    int k;

    printf("------ Lime-GPSDO v1.");
    printf("%u; ", HW_VER);
    printf("FW v1.");
    printf("%u; 30.72MHz ------\r\n\r\n", FW_VER);

    uint8_t vctcxo_tamer_irq = 0;
    uint8_t vctcxo_tamer_en=0,	vctcxo_tamer_en_old = 0;

    // Trim DAC constants
    const uint16_t trimdac_min       = 0x1938; // Decimal value = 6456
    const uint16_t trimdac_max       = 0xE2F3; // Decimal value = 58099

    // Trim DAC calibration line
    line_t trimdac_cal_line;

    // VCTCXO Tune State machine
    state_t tune_state = COARSE_TUNE_MIN;

    // Set the known/default values of the trim DAC cal line
    trimdac_cal_line.point[0].x  = 0;
    trimdac_cal_line.point[0].y  = trimdac_min;
    trimdac_cal_line.point[1].x  = 0;
    trimdac_cal_line.point[1].y  = trimdac_max;
    trimdac_cal_line.slope       = 0;
    trimdac_cal_line.y_intercept = 0;
    struct vctcxo_tamer_pkt_buf vctcxo_tamer_pkt;
    vctcxo_tamer_pkt.ready = false;

    //Reset LMS7
    IOWR(LMS_CTR_GPIO_BASE, 0, 0x06);
    IOWR(LMS_CTR_GPIO_BASE, 0, 0x07);

    // Write initial data to the DAC
	dac_val = 30714; //default DAC value
	Control_TCXO_DAC (1, &dac_val); //enable DAC output, set new val

    // I2C initialiazation
    I2C_init(I2C_OPENCORES_0_BASE, ALT_CPU_FREQ, 100000);

    // Configure LM75
    Configure_LM75();

    while (1)	// infinite loop
    {
    	vctcxo_tamer_irq = (IORD_8DIRECT(VCTCXO_TAMER_0_CTRL_BASE, 0x00) & 0x02);
	    // Clear VCTCXO tamer interrupt
	    if(vctcxo_tamer_irq != 0)
	    {	vctcxo_tamer_isr(&vctcxo_tamer_pkt);
	    	//IOWR_8DIRECT(VCTCXO_TAMER_0_BASE, 0, 0x70);
	    }

    	//Get vctcxo tamer enable bit status
    	vctcxo_tamer_en_old = vctcxo_tamer_en;
    	vctcxo_tamer_en = (IORD_8DIRECT(VCTCXO_TAMER_0_CTRL_BASE, 0x00) & 0x01);

    	if (vctcxo_tamer_en_old != vctcxo_tamer_en){
    		if (vctcxo_tamer_en == 0x01){
    			vctcxo_tamer_init();
    			vctcxo_tamer_pkt.ready = true;
    		}
    		else {
    			vctcxo_tamer_dis();
    			tune_state = COARSE_TUNE_MIN;
    			vctcxo_tamer_pkt.ready = false;
    		}
    	}


        /* Temporarily putting the VCTCXO Calibration stuff here. */
        if( vctcxo_tamer_pkt.ready ) {

            vctcxo_tamer_pkt.ready = false;

            switch(tune_state) {

            case COARSE_TUNE_MIN:

                /* Tune to the minimum DAC value */
                vctcxo_trim_dac_write( 0x08, trimdac_min );
                dac_val = (uint16_t) trimdac_min;
            	Control_TCXO_DAC (1, &dac_val); //enable DAC output, set new val

                /* State to enter upon the next interrupt */
                tune_state = COARSE_TUNE_MAX;
                printf("COARSE_TUNE_MIN: \r\n\t");
                printf("DAC value: ");
                printf("%u;\t", (unsigned int)dac_val);


                break;

            case COARSE_TUNE_MAX:

                /* We have the error from the minimum DAC setting, store it
                 * as the 'x' coordinate for the first point */
                trimdac_cal_line.point[0].x = vctcxo_tamer_pkt.pps_1s_error;

                printf("1s_error: ");
                printf("%i;\r\n", (int)vctcxo_tamer_pkt.pps_1s_error);

                /* Tune to the maximum DAC value */
                vctcxo_trim_dac_write( 0x08, trimdac_max );
                dac_val = (uint16_t) trimdac_max;
            	Control_TCXO_DAC (1, &dac_val); //enable DAC output, set new val

                /* State to enter upon the next interrupt */
                tune_state = COARSE_TUNE_DONE;
                printf("COARSE_TUNE_MAX: \r\n\t");
                printf("DAC value: ");
                printf("%u;\t", (unsigned int)dac_val);

                break;

            case COARSE_TUNE_DONE:
            	/* Write status to to state register*/
            	vctcxo_tamer_write(VT_STATE_ADDR, 0x01);

                /* We have the error from the maximum DAC setting, store it
                 * as the 'x' coordinate for the second point */
                trimdac_cal_line.point[1].x = vctcxo_tamer_pkt.pps_1s_error;

                printf("1s_error: ");
                printf("%i;\r\n", (int)vctcxo_tamer_pkt.pps_1s_error);

                /* We now have two points, so we can calculate the equation
                 * for a line plotted with DAC counts on the Y axis and
                 * error on the X axis. We want a PPM of zero, which ideally
                 * corresponds to the y-intercept of the line. */


                trimdac_cal_line.slope = ( (float) (trimdac_cal_line.point[1].y - trimdac_cal_line.point[0].y) / (float)
                                           (trimdac_cal_line.point[1].x - trimdac_cal_line.point[0].x) );

                trimdac_cal_line.y_intercept = ( trimdac_cal_line.point[0].y -
                                                 (uint16_t)(lroundf(trimdac_cal_line.slope * (float) trimdac_cal_line.point[0].x)));

                /* Set the trim DAC count to the y-intercept */
                vctcxo_trim_dac_write( 0x08, trimdac_cal_line.y_intercept );
                dac_val = (uint16_t) trimdac_cal_line.y_intercept;
            	Control_TCXO_DAC (1, &dac_val); //enable DAC output, set new val

                /* State to enter upon the next interrupt */
                tune_state = FINE_TUNE;
                printf("COARSE_TUNE_DONE: \r\n\t");
                printf("DAC value: ");
                printf("%u;\r\n\r\n", (unsigned int)dac_val);
                printf("FINE_TUNE: \r\n");
                printf("Err_Flag;DAC value;Error;\r\n");

                break;

            case FINE_TUNE:

                /* We should be extremely close to a perfectly tuned
                 * VCTCXO, but some minor adjustments need to be made */

                /* Check the magnitude of the errors starting with the
                 * one second count. If an error is greater than the maximum
                 * tolerated error, adjust the trim DAC by the error (Hz)
                 * multiplied by the slope (in counts/Hz) and scale the
                 * result by the precision interval (e.g. 1s, 10s, 100s). */

                if( vctcxo_tamer_pkt.pps_1s_error_flag ) {
                	vctcxo_trim_dac_value = (vctcxo_trim_dac_value -
                	                    		(uint16_t) (lroundf((float)vctcxo_tamer_pkt.pps_1s_error * trimdac_cal_line.slope)/1));
                	// Write tuned val to VCTCXO_tamer MM registers
                    vctcxo_trim_dac_write( 0x08, vctcxo_trim_dac_value);
                    // Change DAC value
                    dac_val = (uint16_t) vctcxo_trim_dac_value;
                	Control_TCXO_DAC (1, &dac_val); //enable DAC output, set new val
                	printf("001;");
                	printf("%u;", (unsigned int)dac_val);
                	printf("%i;\r\n", (int) vctcxo_tamer_pkt.pps_1s_error);

                } else if( vctcxo_tamer_pkt.pps_10s_error_flag ) {
                	vctcxo_trim_dac_value = (vctcxo_trim_dac_value -
                    							(uint16_t)(lroundf((float)vctcxo_tamer_pkt.pps_10s_error * trimdac_cal_line.slope)/10));
                	// Write tuned val to VCTCXO_tamer MM registers
                    vctcxo_trim_dac_write( 0x08, vctcxo_trim_dac_value);
                    // Change DAC value
                    dac_val = (uint16_t) vctcxo_trim_dac_value;
                	Control_TCXO_DAC (1, &dac_val); //enable DAC output, set new val
                	printf("010;");
                	printf("%u;", (unsigned int)dac_val);
                	printf("%i;\r\n", (int) vctcxo_tamer_pkt.pps_10s_error);

                } else if( vctcxo_tamer_pkt.pps_100s_error_flag ) {
                	vctcxo_trim_dac_value = (vctcxo_trim_dac_value -
                    							(uint16_t)(lroundf((float)vctcxo_tamer_pkt.pps_100s_error * trimdac_cal_line.slope)/100));
                	// Write tuned val to VCTCXO_tamer MM registers
                    vctcxo_trim_dac_write( 0x08, vctcxo_trim_dac_value);
                    // Change DAC value
                    dac_val = (uint16_t) vctcxo_trim_dac_value;
                	Control_TCXO_DAC (1, &dac_val); //enable DAC output, set new val
                	printf("100;");
                	printf("%u;", (unsigned int)dac_val);
                	printf("%i;\r\n", (int) vctcxo_tamer_pkt.pps_100s_error);
                }

                break;

            default:
                break;

            } /* switch */

            /* Take PPS counters out of reset */
            vctcxo_tamer_reset_counters( false );

            /* Enable interrupts */
            vctcxo_tamer_enable_isr( true );

        } /* VCTCXO Tamer interrupt */


        // FIFO
        spirez = IORD(AV_FIFO_INT_0_BASE, 2);	// Read FIFO Status
        if(!(spirez & 0x01))
        {
        	//Read packet from the FIFO
        	getFifoData(glEp0Buffer_Rx, 64);

         	memset (glEp0Buffer_Tx, 0, sizeof(glEp0Buffer_Tx)); //fill whole tx buffer with zeros
         	cmd_errors = 0;

     		LMS_Ctrl_Packet_Tx->Header.Command = LMS_Ctrl_Packet_Rx->Header.Command;
     		LMS_Ctrl_Packet_Tx->Header.Data_blocks = LMS_Ctrl_Packet_Rx->Header.Data_blocks;
     		LMS_Ctrl_Packet_Tx->Header.Periph_ID = LMS_Ctrl_Packet_Rx->Header.Periph_ID;
     		LMS_Ctrl_Packet_Tx->Header.Status = STATUS_BUSY_CMD;

     		switch(LMS_Ctrl_Packet_Rx->Header.Command)
     		{

 				case CMD_GET_INFO:

 					LMS_Ctrl_Packet_Tx->Data_field[0] = FW_VER;
 					LMS_Ctrl_Packet_Tx->Data_field[1] = DEV_TYPE;
 					LMS_Ctrl_Packet_Tx->Data_field[2] = LMS_PROTOCOL_VER;
 					LMS_Ctrl_Packet_Tx->Data_field[3] = HW_VER;
 					LMS_Ctrl_Packet_Tx->Data_field[4] = EXP_BOARD;

 					LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;
 				break;

 				case CMD_LMS_RST:

 					switch (LMS_Ctrl_Packet_Rx->Data_field[0])
 					{
 						case LMS_RST_DEACTIVATE:
 							IOWR(LMS_CTR_GPIO_BASE, 0, 0x07);
 						break;

 						case LMS_RST_ACTIVATE:
 							IOWR(LMS_CTR_GPIO_BASE, 0, 0x06);
 						break;

 						case LMS_RST_PULSE:
 							IOWR(LMS_CTR_GPIO_BASE, 0, 0x06);
 							asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
 							asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
 							IOWR(LMS_CTR_GPIO_BASE, 0, 0x07);
 						break;

 						default:
 							cmd_errors++;
 						break;
 					}

 					LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;
 				break;


 	 			case CMD_LMS7002_WR:
 	 				if(Check_many_blocks (4)) break;

 	 				for(block = 0; block < LMS_Ctrl_Packet_Rx->Header.Data_blocks; block++)
 	 				{
 	 					//write reg addr
 	 					sbi(LMS_Ctrl_Packet_Rx->Data_field[0 + (block * 4)], 7); //set write bit
 	 					spirez = alt_avalon_spi_command(FPGA_SPI_BASE, SPI_NR_LMS7002M, 4, &LMS_Ctrl_Packet_Rx->Data_field[0 + (block * 4)], 0, NULL, 0);
 	 				}

 	 				LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;
 	 			break;


 				case CMD_LMS7002_RD:
 					if(Check_many_blocks (4)) break;

 					for(block = 0; block < LMS_Ctrl_Packet_Rx->Header.Data_blocks; block++)
 					{

 						//write reg addr
 						cbi(LMS_Ctrl_Packet_Rx->Data_field[0 + (block * 2)], 7);  //clear write bit
 						spirez = alt_avalon_spi_command(FPGA_SPI_BASE, SPI_NR_LMS7002M, 2, &LMS_Ctrl_Packet_Rx->Data_field[0 + (block * 2)], 2, &LMS_Ctrl_Packet_Tx->Data_field[2 + (block * 4)], 0);
 					}

 					LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;
 				break;


 	 			case CMD_BRDSPI16_WR:
 	 				if(Check_many_blocks (4)) break;

 	 				for(block = 0; block < LMS_Ctrl_Packet_Rx->Header.Data_blocks; block++)
 	 				{
 	 					//write reg addr
 	 					sbi(LMS_Ctrl_Packet_Rx->Data_field[0 + (block * 4)], 7); //set write bit
 	 					spirez = alt_avalon_spi_command(FPGA_SPI_BASE, SPI_NR_FPGA, 4, &LMS_Ctrl_Packet_Rx->Data_field[0 + (block * 4)], 0, NULL, 0);
 	 				}

 	 				LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;
 	 			break;

 				case CMD_BRDSPI16_RD:
 					if(Check_many_blocks (4)) break;

 					for(block = 0; block < LMS_Ctrl_Packet_Rx->Header.Data_blocks; block++)
 					{
 						//write reg addr
 						cbi(LMS_Ctrl_Packet_Rx->Data_field[0 + (block * 2)], 7);  //clear write bit
 						spirez = alt_avalon_spi_command(FPGA_SPI_BASE, SPI_NR_FPGA, 2, &LMS_Ctrl_Packet_Rx->Data_field[0 + (block * 2)], 2, &LMS_Ctrl_Packet_Tx->Data_field[2 + (block * 4)], 0);
 					}

 					LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;
 				break;


				case CMD_MEMORY_WR:
					current_portion = (LMS_Ctrl_Packet_Rx->Data_field[1] << 24) | (LMS_Ctrl_Packet_Rx->Data_field[2] << 16) | (LMS_Ctrl_Packet_Rx->Data_field[3] << 8) | (LMS_Ctrl_Packet_Rx->Data_field[4]);
					data_cnt = LMS_Ctrl_Packet_Rx->Data_field[5];

					if((LMS_Ctrl_Packet_Rx->Data_field[10] == 0) && (LMS_Ctrl_Packet_Rx->Data_field[11] == 3)) //TARGET = 3 (EEPROM)
					{
						if(LMS_Ctrl_Packet_Rx->Data_field[0] == 0) //write data to EEPROM #1
						{

							cmd_errors = I2C_start(I2C_OPENCORES_0_BASE, EEPROM_I2C_ADDR, 0);
							cmd_errors += I2C_write(I2C_OPENCORES_0_BASE, LMS_Ctrl_Packet_Rx->Data_field[8], 0);
							cmd_errors += I2C_write(I2C_OPENCORES_0_BASE, LMS_Ctrl_Packet_Rx->Data_field[9], 0);

							for(k=0; k<data_cnt-1; k++)
							{
								cmd_errors += I2C_write(I2C_OPENCORES_0_BASE, LMS_Ctrl_Packet_Rx->Data_field[24+k], 0);
								usleep(5000);
							}
							cmd_errors += I2C_write(I2C_OPENCORES_0_BASE, LMS_Ctrl_Packet_Rx->Data_field[24+k], 1);
							usleep(5000);

							if(cmd_errors) LMS_Ctrl_Packet_Tx->Header.Status = STATUS_ERROR_CMD;
							else LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;
						}
						else
							LMS_Ctrl_Packet_Tx->Header.Status = STATUS_ERROR_CMD;
					}
					else

						if((LMS_Ctrl_Packet_Rx->Data_field[10] == 0) && (LMS_Ctrl_Packet_Rx->Data_field[11] == 1)) // TARGET = 1 (FX3)
						{
							switch (LMS_Ctrl_Packet_Rx->Data_field[0]) //PROG_MODE
							{

								case 2: //PROG_MODE = 2 (write FW to flash). Note please, that writes must be page after page.

									if(current_portion == 0)//beginning
									{
										flash_page = 0;
										flash_page_data_cnt = 0;
										flash_data_counter_to_copy = 0;
										fpga_byte = 0;
									}

									flash_data_cnt_free = FLASH_PAGE_SIZE - flash_page_data_cnt;

									if (flash_data_cnt_free > 0)
									{
										if (flash_data_cnt_free > data_cnt)
											flash_data_counter_to_copy = data_cnt; //copy all data if fits to free page space
										else
											flash_data_counter_to_copy = flash_data_cnt_free; //copy only amount of data that fits in to free page size

										memcpy(&flash_page_data[flash_page_data_cnt], &LMS_Ctrl_Packet_Rx->Data_field[24], flash_data_counter_to_copy);

										flash_page_data_cnt = flash_page_data_cnt + flash_data_counter_to_copy;
										flash_data_cnt_free = FLASH_PAGE_SIZE - flash_page_data_cnt;

										if (data_cnt == 0)//all bytes transmitted, end of programming
										{
											if (flash_page_data_cnt > 0)
												flash_page_data_cnt = FLASH_PAGE_SIZE; //finish page
										}

										flash_data_cnt_free = FLASH_PAGE_SIZE - flash_page_data_cnt;
									}

									if (flash_page_data_cnt >= FLASH_PAGE_SIZE) //write data to flash
									{
										if ((flash_page % (FLASH_SECTOR_SIZE/FLASH_PAGE_SIZE)) == 0) //need to erase sector? reached number of pages in block?
											if( FlashSpiEraseSector(FLASH_SPI_BASE, SPI_NR_FLASH, CyTrue, flash_page/(FLASH_SECTOR_SIZE/FLASH_PAGE_SIZE)) != CY_U3P_SUCCESS) cmd_errors++;

										if(!cmd_errors)
											if( FlashSpiTransfer(FLASH_SPI_BASE, SPI_NR_FLASH, flash_page, FLASH_PAGE_SIZE, flash_page_data, CyFalse) != CY_U3P_SUCCESS)  cmd_errors++;//write to flash

										flash_page++;
										flash_page_data_cnt = 0;
										flash_data_cnt_free = FLASH_PAGE_SIZE - flash_page_data_cnt;
									}

									//if not all bytes written to flash page
									if (data_cnt > flash_data_counter_to_copy)
									{
										flash_data_counter_to_copy = data_cnt - flash_data_counter_to_copy;

										memcpy(&flash_page_data[flash_page_data_cnt], &LMS_Ctrl_Packet_Rx->Data_field[24], data_cnt);

										flash_page_data_cnt = flash_page_data_cnt + flash_data_counter_to_copy;
										flash_data_cnt_free = FLASH_PAGE_SIZE - flash_page_data_cnt;
									}

									fpga_byte = fpga_byte + data_cnt;

									if (fpga_byte <= FLASH_PAGE_SIZE * FLASH_SECTOR_SIZE) //correct firmware size?
									{
										if (data_cnt == 0)//end of programming
										{
										}

										if(cmd_errors) LMS_Ctrl_Packet_Tx->Header.Status = STATUS_ERROR_CMD;
										else LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;
									}
									else //not correct firmware size
										LMS_Ctrl_Packet_Tx->Header.Status = STATUS_ERROR_CMD;

									break;

								default:
									LMS_Ctrl_Packet_Tx->Header.Status = STATUS_ERROR_CMD;
									break;
							}
						}
						else LMS_Ctrl_Packet_Tx->Header.Status = STATUS_ERROR_CMD;

				break;


				case CMD_MEMORY_RD:
					current_portion = (LMS_Ctrl_Packet_Rx->Data_field[1] << 24) | (LMS_Ctrl_Packet_Rx->Data_field[2] << 16) | (LMS_Ctrl_Packet_Rx->Data_field[3] << 8) | (LMS_Ctrl_Packet_Rx->Data_field[4]);
					data_cnt = LMS_Ctrl_Packet_Rx->Data_field[5];

					if((LMS_Ctrl_Packet_Rx->Data_field[10] == 0) && (LMS_Ctrl_Packet_Rx->Data_field[11] == 3)) //TARGET = 3 (EEPROM)
					{
						if(LMS_Ctrl_Packet_Rx->Data_field[0] == 0) //read data from EEPROM #1
						{

							cmd_errors = I2C_start(I2C_OPENCORES_0_BASE, EEPROM_I2C_ADDR, 0);
							cmd_errors += I2C_write(I2C_OPENCORES_0_BASE, LMS_Ctrl_Packet_Rx->Data_field[8], 0);
							cmd_errors += I2C_write(I2C_OPENCORES_0_BASE, LMS_Ctrl_Packet_Rx->Data_field[9], 0);

							cmd_errors += I2C_start(I2C_OPENCORES_0_BASE, EEPROM_I2C_ADDR, 1);
							for(k=0; k<data_cnt-1; k++)
							{
								LMS_Ctrl_Packet_Tx->Data_field[24+k] = I2C_read(I2C_OPENCORES_0_BASE, 0);
							}
							LMS_Ctrl_Packet_Tx->Data_field[24+k] = I2C_read(I2C_OPENCORES_0_BASE, 1);

							if(cmd_errors) LMS_Ctrl_Packet_Tx->Header.Status = STATUS_ERROR_CMD;
							else LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;
						}
						else
							LMS_Ctrl_Packet_Tx->Header.Status = STATUS_ERROR_CMD;
					}
					else
					{
						if((LMS_Ctrl_Packet_Rx->Data_field[10] == 0) && (LMS_Ctrl_Packet_Rx->Data_field[11] == 1)) // TARGET = 1 (FX3)
						{
							flash_page  = LMS_Ctrl_Packet_Rx->Data_field[6] << 24;
							flash_page |= LMS_Ctrl_Packet_Rx->Data_field[7] << 16;
							flash_page |= LMS_Ctrl_Packet_Rx->Data_field[8] << 8;
							flash_page |= LMS_Ctrl_Packet_Rx->Data_field[9];

							if( FlashSpiRead(FLASH_SPI_BASE, SPI_NR_FLASH, flash_page, FLASH_PAGE_SIZE, flash_page_data) != CY_U3P_SUCCESS)  cmd_errors++;//write to flash

							for(k=0; k<data_cnt; k++)
							{
								LMS_Ctrl_Packet_Tx->Data_field[24+k] = flash_page_data[k];
							}

							if(cmd_errors) LMS_Ctrl_Packet_Tx->Header.Status = STATUS_ERROR_CMD;
							else LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;
						}
						else
							LMS_Ctrl_Packet_Tx->Header.Status = STATUS_ERROR_CMD;
					}

				break;




				case CMD_ANALOG_VAL_RD:

					for(block = 0; block < LMS_Ctrl_Packet_Rx->Header.Data_blocks; block++)
					{
						switch (LMS_Ctrl_Packet_Rx->Data_field[0 + (block)])//ch
						{
							case 0://dac val

								LMS_Ctrl_Packet_Tx->Data_field[0 + (block * 4)] = LMS_Ctrl_Packet_Rx->Data_field[block]; //ch
								LMS_Ctrl_Packet_Tx->Data_field[1 + (block * 4)] = 0x00; //RAW //unit, power

								LMS_Ctrl_Packet_Tx->Data_field[2 + (block * 4)] = (dac_val >> 8) & 0xFF; //unsigned val, MSB byte
								LMS_Ctrl_Packet_Tx->Data_field[3 + (block * 4)] = dac_val & 0xFF; //unsigned val, LSB byte
								break;

							case 1: //temperature

								spirez = I2C_start(I2C_OPENCORES_0_BASE, LM75_I2C_ADDR, 0);
								spirez = I2C_write(I2C_OPENCORES_0_BASE, 0x00, 1);				// Pointer = temperature register
								spirez = I2C_start(I2C_OPENCORES_0_BASE, LM75_I2C_ADDR, 1);

								// Read temperature and recalculate
								converted_val = (signed short int)I2C_read(I2C_OPENCORES_0_BASE, 0);
								converted_val = converted_val << 8;
								converted_val = 10 * (converted_val >> 8);
								spirez = I2C_read(I2C_OPENCORES_0_BASE, 1);
								if(spirez & 0x80) converted_val = converted_val + 5;


								LMS_Ctrl_Packet_Tx->Data_field[0 + (block * 4)] = LMS_Ctrl_Packet_Rx->Data_field[block]; //ch
								LMS_Ctrl_Packet_Tx->Data_field[1 + (block * 4)] = 0x50; //mC //unit, power

								LMS_Ctrl_Packet_Tx->Data_field[2 + (block * 4)] = (converted_val >> 8); //signed val, MSB byte
								LMS_Ctrl_Packet_Tx->Data_field[3 + (block * 4)] = converted_val; //signed val, LSB byte

							break;

							default:
								cmd_errors++;
							break;
						}
					}

					LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;

				break;


				case CMD_ANALOG_VAL_WR:
					if(Check_many_blocks (4)) break;

					for(block = 0; block < LMS_Ctrl_Packet_Rx->Header.Data_blocks; block++)
					{
						switch (LMS_Ctrl_Packet_Rx->Data_field[0 + (block * 4)]) //do something according to channel
						{
							case 0:
								if (LMS_Ctrl_Packet_Rx->Data_field[1 + (block * 4)] == 0) //RAW units?
								{
									if(LMS_Ctrl_Packet_Rx->Data_field[2 + (block * 4)] == 0) //MSB byte empty?
									{
										dac_val = (LMS_Ctrl_Packet_Rx->Data_field[2 + (block * 4)] << 8 ) + LMS_Ctrl_Packet_Rx->Data_field[3 + (block * 4)];

										Control_TCXO_DAC (1, &dac_val);
									}
									else cmd_errors++;
								}
								else cmd_errors++;

							break;

							default:
								cmd_errors++;
							break;
						}
					}


					if(cmd_errors) LMS_Ctrl_Packet_Tx->Header.Status = STATUS_ERROR_CMD;
					else LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;

				break;







				case CMD_ALTERA_FPGA_GW_WR: //FPGA passive serial

					current_portion = (LMS_Ctrl_Packet_Rx->Data_field[1] << 24) | (LMS_Ctrl_Packet_Rx->Data_field[2] << 16) | (LMS_Ctrl_Packet_Rx->Data_field[3] << 8) | (LMS_Ctrl_Packet_Rx->Data_field[4]);
					data_cnt = LMS_Ctrl_Packet_Rx->Data_field[5];

					switch(LMS_Ctrl_Packet_Rx->Data_field[0])//prog_mode
					{
						/*
						Programming mode:

						0 - Bitstream to FPGA
						1 - Bitstream to Flash
						2 - Bitstream from Flash
						*/

						case 0://Bitstream to FPGA from PC
							/*
							if ( Configure_FPGA (&LMS_Ctrl_Packet_Rx->Data_field[24], current_portion, data_cnt) ) LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;
							else LMS_Ctrl_Packet_Tx->Header.Status = STATUS_ERROR_CMD;
							*/
							LMS_Ctrl_Packet_Tx->Header.Status = STATUS_ERROR_CMD;

						break;

						case 1: //write data to Flash from PC

							current_portion = (LMS_Ctrl_Packet_Rx->Data_field[1] << 24) | (LMS_Ctrl_Packet_Rx->Data_field[2] << 16) | (LMS_Ctrl_Packet_Rx->Data_field[3] << 8) | (LMS_Ctrl_Packet_Rx->Data_field[4]);
							data_cnt = LMS_Ctrl_Packet_Rx->Data_field[5];

							if (current_portion == 0) state = 10;
							if (data_cnt        == 0)
							{
								state = 30;
							}
							Flash = 1;

							while(Flash)
							{
								switch (state)
								{
									//Init
									case 10:
										//Set Flash memory addresses
										CFM0StartAddress = 0x04A000;
										CFM0EndAddress   = 0x08BFFF;

										address = CFM0StartAddress;

										//Write Control Register of On-Chip Flash IP to un-protect and erase operation
										IOWR(ONCHIP_FLASH_0_CSR_BASE, 1, 0xf7ffffff);
										IOWR(ONCHIP_FLASH_0_CSR_BASE, 1, 0xf7dfffff);

										state = 11;
										Flash = 1;

									case 11:
										//Start erase CFM0
										if((IORD(ONCHIP_FLASH_0_CSR_BASE, 0) & 0x13) == 0x10)
										{
											IOWR(ONCHIP_FLASH_0_CSR_BASE, 1, 0xf7ffffff);
											//printf("CFM0 Erased\n");
											//printf("Enter Programming file.\n");
											state = 20;
											Flash = 1;
										}
										if((IORD(ONCHIP_FLASH_0_CSR_BASE, 0) & 0x13) == 0x01)
										{
											//printf("Erasing CFM0\n");
											state = 11;
											Flash = 1;
										}
										if((IORD(ONCHIP_FLASH_0_CSR_BASE, 0) & 0x13) == 0x00)
										{
											//printf("Erase CFM0 Failed\n");
											state = 0;
										}

									break;

									//Program
									case 20:
										for(byte = 24; byte <= 52; byte += 4)
										{
											//Take word
											//word = (LMS_Ctrl_Packet_Rx->Data_field[byte+0]<<24)|(LMS_Ctrl_Packet_Rx->Data_field[byte+1]<<16)|(LMS_Ctrl_Packet_Rx->Data_field[byte+2]<<8)|(LMS_Ctrl_Packet_Rx->Data_field[byte+3]);
											word  = (ALT_CI_NIOS_CUSTOM_INSTR_BITSWAP_0(LMS_Ctrl_Packet_Rx->Data_field[byte+0]) >>  0) & 0xFF000000;
											word |= (ALT_CI_NIOS_CUSTOM_INSTR_BITSWAP_0(LMS_Ctrl_Packet_Rx->Data_field[byte+1]) >>  8) & 0x00FF0000;
											word |= (ALT_CI_NIOS_CUSTOM_INSTR_BITSWAP_0(LMS_Ctrl_Packet_Rx->Data_field[byte+2]) >> 16) & 0x0000FF00;
											word |= (ALT_CI_NIOS_CUSTOM_INSTR_BITSWAP_0(LMS_Ctrl_Packet_Rx->Data_field[byte+3]) >> 24) & 0x000000FF;

											//Command to write into On-Chip Flash IP
											if(address <= CFM0EndAddress)
											{
												IOWR_32DIRECT(ONCHIP_FLASH_0_DATA_BASE, address, word);

												address += 4;


												while((IORD(ONCHIP_FLASH_0_CSR_BASE, 0) & 0x0b) == 0x02)
												{
													//printf("Writing CFM0(%d)\n", address);
												}
												if((IORD(ONCHIP_FLASH_0_CSR_BASE, 0) & 0x0b) == 0x00)
												{
													//printf("Write to %d failed\n", address);
													state = 0;
													address = 700000;
												}
												if((IORD(ONCHIP_FLASH_0_CSR_BASE, 0) & 0x0b) == 0x08)
												{
												};
											}
											else
											{
												LMS_Ctrl_Packet_Tx->Header.Status = STATUS_ERROR_CMD;
											};
										};

										state = 20;
										Flash = 0;
										LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;

									break;

									//Finish
									case 30:
										//Re-protect the sector
										IOWR(ONCHIP_FLASH_0_CSR_BASE, 1, 0xffffffff);

										state = 0;
										Flash = 0;

										LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;

									break;

									default:
										LMS_Ctrl_Packet_Tx->Header.Status = STATUS_ERROR_CMD;
										state = 0;
										Flash = 0;
								};
							};

						break;

						case 2: //configure FPGA from flash

							//enable boot to factory image, booting is executed after response to command is sent
							boot_img_en = 1;

							LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;



						break;

						default:
							LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;
						break;

					}

				break;

				case CMD_LMS_MCU_FW_WR:

					current_portion = LMS_Ctrl_Packet_Rx->Data_field[1];

					//check if portions are send in correct order
					if(current_portion != 0) //not first portion?
					{
						if(last_portion != (current_portion - 1)) //portion number increments?
						{
							LMS_Ctrl_Packet_Tx->Header.Status = STATUS_WRONG_ORDER_CMD;
							break;
						}
					}


					if (current_portion == 0) //PORTION_NR = first fifo
					{
						//reset mcu
						sc_brdg_data[0] = (0x80); //reg addr MSB with write bit
						sc_brdg_data[1] = (MCU_CONTROL_REG); //reg addr LSB

						sc_brdg_data[2] = (0x00); //reg data MSB
						sc_brdg_data[3] = (0x00); //reg data LSB //8

						//**ZT CyU3PSpiTransmitWords (&sc_brdg_data[0], 4);
						spirez = alt_avalon_spi_command(FPGA_SPI_BASE, SPI_NR_LMS7002M, 4, &sc_brdg_data[0], 0, NULL, 0);

						//set mode
						//write reg addr - mSPI_REG2 (Controls MCU input pins)
						sc_brdg_data[0] = (0x80); //reg addr MSB with write bit
						sc_brdg_data[1] = (MCU_CONTROL_REG); //reg addr LSB

						sc_brdg_data[2] = (0x00); //reg data MSB

						//reg data LSB
						switch (LMS_Ctrl_Packet_Rx->Data_field[0]) //PROG_MODE
						{
							case PROG_EEPROM:
								sc_brdg_data[3] = (0x01); //Programming both EEPROM and SRAM  //8
								spirez = alt_avalon_spi_command(FPGA_SPI_BASE, SPI_NR_LMS7002M, 4, &sc_brdg_data[0], 0, NULL, 0);
								break;

							case PROG_SRAM:
								sc_brdg_data[3] =(0x02); //Programming only SRAM  //8
								spirez = alt_avalon_spi_command(FPGA_SPI_BASE, SPI_NR_LMS7002M, 4, &sc_brdg_data[0], 0, NULL, 0);
								break;


							case BOOT_MCU:
								sc_brdg_data[3] = (0x03); //Programming both EEPROM and SRAM  //8
								spirez = alt_avalon_spi_command(FPGA_SPI_BASE, SPI_NR_LMS7002M, 4, &sc_brdg_data[0], 0, NULL, 0);

								//spi read
								//write reg addr
								sc_brdg_data[0] = (0x00); //reg addr MSB
								sc_brdg_data[1] = (MCU_STATUS_REG); //reg addr LSB

								//read reg data
								spirez = alt_avalon_spi_command(FPGA_SPI_BASE, SPI_NR_LMS7002M, 2, &sc_brdg_data[0], 2, &sc_brdg_data[0], 0);

								goto BOOTING;

								break;
						}
					}

					MCU_retries = 0;

					//wait till EMPTY_WRITE_BUFF = 1
					while (MCU_retries < MAX_MCU_RETRIES)
					{
						//read status reg

						//spi read
						//write reg addr
						sc_brdg_data[0] = (0x00); //reg addr MSB
						sc_brdg_data[1] = (MCU_STATUS_REG); //reg addr LSB
						//**ZT CyU3PSpiTransmitWords (&sc_brdg_data[0], 2);
						//spirez = alt_avalon_spi_command(FPGA_SPI_BASE, SPI_NR_LMS7002M, 2, &sc_brdg_data[0], 0, NULL, 0);

						//read reg data
						//**ZT CyU3PSpiReceiveWords (&sc_brdg_data[0], 2); //reg data
						spirez = alt_avalon_spi_command(FPGA_SPI_BASE, SPI_NR_LMS7002M, 2, &sc_brdg_data[0], 2, &sc_brdg_data[0], 0);

						if (sc_brdg_data[1] &0x01) break; //EMPTY_WRITE_BUFF = 1

						MCU_retries++;
						usleep (30);
					}

					//write 32 bytes to FIFO
					for(block = 0; block < 32; block++)
					{

						//write reg addr - mSPI_REG4 (Writes one byte of data to MCU  )
						sc_brdg_data[0] = (0x80); //reg addr MSB with write bit
						sc_brdg_data[1] = (MCU_FIFO_WR_REG); //reg addr LSB

						sc_brdg_data[2] = (0x00); //reg data MSB
						sc_brdg_data[3] = (LMS_Ctrl_Packet_Rx->Data_field[2 + block]); //reg data LSB //8

						//**ZT CyU3PSpiTransmitWords (&sc_brdg_data[0], 4);
						spirez = alt_avalon_spi_command(FPGA_SPI_BASE, SPI_NR_LMS7002M, 4, &sc_brdg_data[0], 0, NULL, 0);

						MCU_retries = 0;
					}

					/*sbi (PORTB, SAEN); //Enable LMS's SPI
					cbi (PORTB, SAEN); //Enable LMS's SPI*/


					MCU_retries = 0;

					//wait till EMPTY_WRITE_BUFF = 1
					while (MCU_retries < 500)
					{
						//read status reg

						//spi read
						//write reg addr
						sc_brdg_data[0] = (0x00); //reg addr MSB
						sc_brdg_data[1] = (MCU_STATUS_REG); //reg addr LSB
						//**ZT CyU3PSpiTransmitWords (&sc_brdg_data[0], 2);
						//spirez = alt_avalon_spi_command(FPGA_SPI_BASE, SPI_NR_LMS7002M, 2, &sc_brdg_data[0], 0, NULL, 0);

						//read reg data
						//**ZT CyU3PSpiReceiveWords (&sc_brdg_data[0], 2); //reg data
						spirez = alt_avalon_spi_command(FPGA_SPI_BASE, SPI_NR_LMS7002M, 2, &sc_brdg_data[0], 2, &sc_brdg_data[0], 0);

						if (sc_brdg_data[1] &0x01) break; //EMPTY_WRITE_BUFF = 1

						MCU_retries++;
						usleep (30);
					}


					if (current_portion  == 255) //PORTION_NR = last fifo
					{
						//chek programmed bit

						MCU_retries = 0;

						//wait till PROGRAMMED = 1
						while (MCU_retries < MAX_MCU_RETRIES)
						{
							//read status reg

							//spi read
							//write reg addr
							sc_brdg_data[0] = (0x00); //reg addr MSB
							sc_brdg_data[1] = (MCU_STATUS_REG); //reg addr LSB
							//**ZT CyU3PSpiTransmitWords (&sc_brdg_data[0], 2);
							//spirez = alt_avalon_spi_command(FPGA_SPI_BASE, SPI_NR_LMS7002M, 2, &sc_brdg_data[0], 0, NULL, 0);

							//read reg data
							//**ZT CyU3PSpiReceiveWords (&sc_brdg_data[0], 2); //reg data
							spirez = alt_avalon_spi_command(FPGA_SPI_BASE, SPI_NR_LMS7002M, 2, &sc_brdg_data[0], 2, &sc_brdg_data[0], 0);

							if (sc_brdg_data[1] &0x40) break; //PROGRAMMED = 1

							MCU_retries++;
							usleep (30);
						}

						if (MCU_retries == MAX_MCU_RETRIES) cmd_errors++;
					}

					last_portion = current_portion; //save last portion number

					BOOTING:

					if(cmd_errors) LMS_Ctrl_Packet_Tx->Header.Status = STATUS_ERROR_CMD;
					else LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;

					//**ZT Modify_BRDSPI16_Reg_bits (FPGA_SPI_REG_LMS1_LMS2_CTRL, LMS1_SS, LMS1_SS, 1); //Disable LMS's SPI

				break;

 				default:
 					/* This is unknown request. */
 					//isHandled = CyFalse;
					LMS_Ctrl_Packet_Tx->Header.Status = STATUS_UNKNOWN_CMD;
 				break;
     		}


     		//Send response to the command
        	for(int i=0; i<64/sizeof(uint32_t); ++i)
        	{
        		IOWR(AV_FIFO_INT_0_BASE, 0, dest[i]);
        	};

        	// If boot from flash CMD is executed FPGA GW is loaded from internal FLASH (image 1)
        	if (boot_img_en==1) {

        		boot_from_flash();

        	};


        };

    }

    return 0;	//Just make compiler happy!
}
