# ----------------------------------------------------------------------------
# FILE         : 	LMS7002_timing.sdc
# DESCRIPTION  :	Timing constrains for LMS7002 IC
# DATE         :	June 2, 2017
# AUTHOR(s)    :	Lime Microsystems
# REVISIONS    :
# ----------------------------------------------------------------------------
# NOTES:
# 
# ----------------------------------------------------------------------------

# ----------------------------------------------------------------------------
# Timing parameters
# ----------------------------------------------------------------------------
# 50MHz clock 
set CLK50_FPGA_period   20.00

# Reference clock
set CLK0_OUT_period     32.55

# ----------------------------------------------------------------------------
#Base clocks
# ----------------------------------------------------------------------------
create_clock 	-name CLK50_FPGA \
					-period 	$CLK50_FPGA_period \
								[get_ports CLK50_FPGA]
							
create_clock 	-name CLK0_OUT \
					-period 	$CLK0_OUT_period \
								[get_ports CLK0_OUT]