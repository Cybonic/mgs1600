


In order to read a given sensor it has to be set first. use the following table to set the sensor. 
It is possible to read all sensor by using "0x3F", read each sensor individually or read a given group of sensors by doing "AND" operations (i.e Read TRACK DETECT and INTERNAL SENSORS = 0x01 || 0x02)

SET_ALL_SENSORS		 0x3F
SET_TRACK_DETECT     0x01
SET_INTERNAL_SENSORS 0x02
SET_SELECTED_TRACK	 0x04
SET_TRACKS			 0x08
SET_GYRO			 0x10
SET_MARKERS			 0x20
