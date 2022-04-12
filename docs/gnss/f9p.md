#### single point positioning by F9P

1. RTKLIB settings.

Change `inpstr1-path` of `$HOME/RTKLIB/app/rtkrcv/conf/rtklib_ros_bridge_single.conf` according to the serial device you use.

ie)
>inpstr1-path =/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00:230400:8:n:1:off  

2. nmea_comms settings.

Change `arg name="port"` of `$HOME/catkin_ws/src/nmea_comms/launch/f9p_nmea_sentence.launch` according to the serial device you use.

ie)
>\<arg name="port" default="/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AG0JNPDS-if00-port0" />

3. GNSS receiver settings.
Configure the receiver settings using [u-center](https://www.u-blox.com/product/u-center).

* UART1(Connect to RTKLIB) Enable UBX message (output rate 5Hz, baudrate 230400) ※ Set to output only RAWX and SFRBX
* UART2(Connect to nmea_comms) Enable NMEA message (output rate 1Hz, baudrate 115200) ※ Set to output only GGA and RMC

[This file (eagleye_f9p_conf.txt)](https://www.dropbox.com/s/5mq9hbygnviojoh/eagleye_f9p_conf.txt?dl=0) is a sample configuration file for F9P.  
Open u-center.  
Tools/Receiver Configuration.../Load configuration "Transfer file -> GNSS"

To load the configuration, change the ublox FW version to 1.10.

#### Real Time Kinematic by F9P

1. RTKLIB settings.

Change `inpstr1-path`, `inpstr2-path`, `inpstr2-format`, and `ant2-postype` of `$HOME/RTKLIB/app/rtkrcv/conf/rtklib_ros_bridge_meijo_rtk.conf` according to the serial device you use.

ie)
>inpstr1-path =/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00:230400:8:n:1:off  
>inpstr2-path =:@rtk2go.com:2101/Meijo-Ublox  
>inpstr2-format =ubx  
>ant2-postype       =llh        # (0:llh,1:xyz,2:single,3:posfile,4:rinexhead,5:rtcm,6:raw)  
ant2-pos1          =35.1348599331534          # (deg|m) If ant2-postype is llh or xyz, the position of the reference station must be specified by ant2-pos1, ant2-pos2, and ant2-pos3.  
ant2-pos2          =136.973613158051          # (deg|m)  
ant2-pos3          =102.502548295454          # (m|m)  

2. nmea_comms settings.  

Change `arg name="port"` of `$HOME/catkin_ws/src/nmea_comms/launch/f9p_nmea_sentence.launch` according to the serial device you use.

ie)
>\<arg name="port" default="/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AG0JNPDS-if00-port0" />

3. GNSS receiver settings.
Configure the receiver settings using [u-center](https://www.u-blox.com/product/u-center).

The following is a sample configuration file for F9P.  

(1) Settings for receivers that output aircraft that output NMEA (positioning results RTK'd by the F9P internal engine)  
eagleye_f9p_nmea_conf.txt  
https://www.dropbox.com/s/3viqyqutipn5dpj/eagleye_f9p_nmea_conf.txt?dl=0  
(2) Settings for receivers that measure RAW data through RTKLIB and output Doppler velocity.  
eagleye_f9p_raw_conf.txt  
https://www.dropbox.com/s/acz98v30rtgbmsx/eagleye_f9p_raw_conf.txt?dl=0  
Open u-center.  
Tools/Receiver Configuration.../Load configuration "Transfer file -> GNSS"

To load the configuration, change the ublox FW version to 1.10.
