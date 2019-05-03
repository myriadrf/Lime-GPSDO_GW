#Copy and Rename .sof file by hardware version 
file copy -force -- output_files/Lime-GPSDO.pof ../bitstream/Lime-GPSDO_HW_1.0.pof
post_message "*******************************************************************"
post_message "Generated programming file: Lime-GPSDO_HW_1.0.pof" -submsgs [list "Output file saved in ../bitstream/Lime-GPSDO_HW_1.0.pof directory"]
post_message "*******************************************************************"
