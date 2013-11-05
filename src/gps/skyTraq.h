
/* Auto generated structure include file for SkyTraq Venus 6
   Copywrite 2013 Ashima Research
 */

#define PACKED __attribute__((__packed__))

      
  
enum message_ids_t { 
    system_restart = 0x1, /* Force system to restart */
    query_software_version = 0x2, /* Query revision information of software */
    query_software_crc = 0x3, /* Query the CRC of the software */
    set_factory_defaults = 0x4, /* Set system to factory default values */
    configure_serial_port = 0x5, /* Set up serial port COM, baud rate, data bits, stop bits and parity */
    
    
    configure_nmea = 0x8, /* Configure NMEA output message */
    configure_output_message_format = 0x9, /* Configure the output message format from GPS receiver */
    configure_power_mode = 0xC, /* Set system power mode */
    configure_position_update_rate = 0xE, /* Configure the position update rate of GPS system */
    query_position_update_rate = 0x10, /* Query the position update rate of GPS system */
    configure_navigation_message_interval = 0x11, /* Configure the navigation output message interval */
    configure_datum = 0x29, /* Configure Datum of the GPS receiver */
    query_datum = 0x2D, /* Query datum used by the GPS receiver */
    get_ephemeris = 0x30, /* Retrieve ephemeris data of the GPS receiver */
    set_ephemeris = 0x31, /* Set ephemeris data to the GPS receiver */
    configure_waas = 0x37, /* Configure the enable or disable of WAAS */
    query_waas_status = 0x38, /* Query WAAS status of GPS receiver */
    configure_position_pinning = 0x39, /* Enable or disable position pinning of GPS receiver */
    query_position_pinning = 0x3A, /* Query position pinning status of the GPS receiver */
    configure_position_pinning_parameters = 0x3B, /* Set position pinning parameters of GPS receiver */
    configuration_navigation_mode = 0x3C, /* Configure the navigation mode of GPS system */
    query_navigation_mode = 0x3D, /* Query the navigation mode of GPS receiver */
    configure_1pps_mode = 0x3E, /* Set 1PPS mode to the GPS receiver */
    query_1pps_mode = 0x3F, /* Query 1PPS mode of the GPS receiver */
    software_version = 0x80, /* Software revision of the receiver */
    software_crc = 0x81, /* Software CRC of the receiver */
    
    ack = 0x83, /* ACK to a successful input message */
    nack = 0x84, /* Response to an unsuccessful input message */
    position_update_rate = 0x86, /* Position update rate of GPS system */
    navigation_data = 0xA8, /* Output user navigation data in binary format */
    gps_datum = 0xAE, /* Datum used by the GPS receiver */
    gps_waas_status = 0xB3, /* WAAS status of the GPS receiver */
    gps_position_pinning_status = 0xB4, /* Position pinning status of the GPS receiver */
    gps_navigation_mode = 0xB5, /* Navigation mode of the GPS receiver */
    gps_1pps_mode = 0xB6, /* 1PPS mode of GPS receiver */
   };
  
  
typedef struct PACKED message_0x01_t { 
    uint8_t  message_id ; /*  */
    uint8_t  start_mode ; /* 00 = Reserved 01 = System Reset, Hot start 02 = System Reset, Warm start 03 = System Reset, Cold start 04 = Reserved */
    uint16_t  utc_year ; /* >= 1980 */
    uint8_t  utc_month ; /* 1 ~ 12 */
    uint8_t  utc_day ; /* 1 ~ 31 */
    uint8_t  utc_hour ; /* 0 ~ 23 */
    uint8_t  utc_minute ; /* 0 ~ 59 */
    uint8_t  utc_second ; /* 0 ~ 59 */
    int16_t  latitude ; /* Between – 9000 and 9000 > 0: North Hemisphere < 0: South Hemisphere */
    int16_t  longitude ; /* Between – 18000 and 18000 > 0: East Hemisphere < 0: West Hemisphere */
    int16_t  altitude ; /* Between –1000 and 18300 */
   } message_0x01_t;
  
  
typedef struct PACKED message_0x02_t { 
    uint8_t  message_id ; /*  */
    uint8_t  software_type ; /* 00 = Reserved 01 = System code */
   } message_0x02_t;
  
  
typedef struct PACKED message_0x03_t { 
    uint8_t  message_id ; /*  */
    uint8_t  software_type ; /* 00 = Reserved 01 = System code */
   } message_0x03_t;
  
  
typedef struct PACKED message_0x04_t { 
    uint8_t  message_id ; /*  */
    uint8_t  type ; /* 00 = Reserved 01 = reboot after setting to factory defaults */
   } message_0x04_t;
  
  
typedef struct PACKED message_0x05_t { 
    uint8_t  message_id ; /*  */
    uint8_t  com_port ; /* 00 = COM 1 */
    uint8_t  baud_rate ; /* 0: 4800 1: 9600 2: 19200 3: 38400 4: 57600 5: 115200 */
    uint8_t  attributes ; /* 0: update to SRAM 1: update to both SRAM & FLASH */
   } message_0x05_t;
  
  
typedef struct PACKED message_0x08_t { 
    uint8_t  message_id ; /*  */
    uint8_t  gga_interval ; /* 0 ~255, 0: disable */
    uint8_t  gsa_interval ; /* 0 ~255, 0: disable */
    uint8_t  gsv_interval ; /* 0 ~255, 0: disable */
    uint8_t  gll_interval ; /* 0 ~255, 0: disable */
    uint8_t  rmc_interval ; /* 0 ~255, 0: disable */
    uint8_t  vtg_interval ; /* 0 ~255, 0: disable */
    uint8_t  zda_interval ; /* 0 ~255, 0: disable */
    uint8_t  attributes ; /* 0: update to SRAM 1: update to both SRAM & FLASH */
   } message_0x08_t;
  
  
typedef struct PACKED message_0x09_t { 
    uint8_t  message_id ; /*  */
    uint8_t  type ; /* 00 : No output 01 : NMEA message 02 : Binary Message */
    uint8_t  attributes ; /* 0: update to SRAM 1: update to both SRAM & FLASH */
   } message_0x09_t;
  
  
typedef struct PACKED message_0x0C_t { 
    uint8_t  message_id ; /*  */
    uint8_t  mode ; /* 00 = Normal (disable) 01 = Power Save (enable) */
    uint8_t  attributes ; /* 0: update to SRAM 1: update to both SRAM & FLASH 2: temporarily enabled */
   } message_0x0C_t;
  
  
typedef struct PACKED message_0x0E_t { 
    uint8_t  message_id ; /*  */
    uint8_t  rate ; /* Value with 1, 2, 4, 5, 8 or 10 01: 1Hz update rate Note: value with 4 or higher should work with baud rate 38400 or higher */
    uint8_t  attributes ; /* 0: update to SRAM 1: update to both SRAM & FLASH */
   } message_0x0E_t;
  
  
typedef struct PACKED message_0x10_t { 
    uint8_t  message_id ; /*  */
   } message_0x10_t;
  
  
typedef struct PACKED message_0x11_t { 
    uint8_t  message_id ; /*  */
    uint8_t  navigation_message_interval ; /* 0 ~255, 0: disable */
    uint8_t  attributes ; /* 0: update to SRAM 1: update to both SRAM & FLASH */
   } message_0x11_t;
  
  
typedef struct PACKED message_0x29_t { 
    uint8_t  message_id ; /*  */
    uint16_t  index ; /* Refer to Appendix B for available Datum */
    uint8_t  ellip_idx ; /* Refer to Appendix A for available Value */
    int16_t  delta_x ; /* Refer to Appendix A and B for available Delta X */
    int16_t  delta_y ; /* Refer to Appendix A and B for available Delta Y */
    int16_t  delta_z ; /* Refer to Appendix A and B for available Delta Z */
    uint32_t  semi_majoraxis ; /* Refer to Appendix A */
    uint32_t  inversed_flattening ; /* Refer to Appendix A */
    uint8_t  attributes ; /* 0: update to SRAM 1: update to both SRAM & FLASH */
   } message_0x29_t;
  
  
typedef struct PACKED message_0x2D_t { 
    uint8_t  message_id ; /*  */
   } message_0x2D_t;
  
  
typedef struct PACKED message_0x30_t { 
    uint8_t  message_id ; /*  */
    uint8_t  sv_ ; /* 0: means all SVs 1~32 : mean for the particular SV */
   } message_0x30_t;
  
  
typedef struct PACKED message_0x31_t { 
    uint8_t  message_id ; /*  */
    uint16_t  sv_id ; /* Satellite id */
    uint8_t  subframedata_0_0 ; /* Eph data subframe 1 */
    uint8_t  subframedata_0_1 ; /* Eph data subframe 1 */
    uint8_t  subframedata_0_2 ; /* Eph data subframe 1 */
    uint8_t  subframedata_0_3 ; /* Eph data subframe 1 */
    uint8_t  subframedata_0_4 ; /* Eph data subframe 1 */
    uint8_t  subframedata_0_5 ; /* Eph data subframe 1 */
    uint8_t  subframedata_0_6 ; /* Eph data subframe 1 */
    uint8_t  subframedata_0_7 ; /* Eph data subframe 1 */
    uint8_t  subframedata_0_8 ; /* Eph data subframe 1 */
    uint8_t  subframedata_0_9 ; /* Eph data subframe 1 */
    uint8_t  subframedata_0_10 ; /* Eph data subframe 1 */
    uint8_t  subframedata_0_11 ; /* Eph data subframe 1 */
    uint8_t  subframedata_0_12 ; /* Eph data subframe 1 */
    uint8_t  subframedata_0_13 ; /* Eph data subframe 1 */
    uint8_t  subframedata_0_14 ; /* Eph data subframe 1 */
    uint8_t  subframedata_0_15 ; /* Eph data subframe 1 */
    uint8_t  subframedata_0_16 ; /* Eph data subframe 1 */
    uint8_t  subframedata_0_17 ; /* Eph data subframe 1 */
    uint8_t  subframedata_0_18 ; /* Eph data subframe 1 */
    uint8_t  subframedata_0_19 ; /* Eph data subframe 1 */
    uint8_t  subframedata_0_20 ; /* Eph data subframe 1 */
    uint8_t  subframedata_0_21 ; /* Eph data subframe 1 */
    uint8_t  subframedata_0_22 ; /* Eph data subframe 1 */
    uint8_t  subframedata_0_23 ; /* Eph data subframe 1 */
    uint8_t  subframedata_0_24 ; /* Eph data subframe 1 */
    uint8_t  subframedata_0_25 ; /* Eph data subframe 1 */
    uint8_t  subframedata_0_26 ; /* Eph data subframe 1 */
    uint8_t  subframedata_0_27 ; /* Eph data subframe 1 */
    uint8_t  subframedata_1_027 ; /* Eph data subframe 2, same as field 4-31 */
    uint8_t  subframedata_2_027 ; /* Eph data subframe 3, same as field 4-31 */
   } message_0x31_t;
  
  
typedef struct PACKED message_0x37_t { 
    uint8_t  message_id ; /*  */
    uint8_t  enable ; /* 0: disable 1: enable */
    uint8_t  attributes ; /* 0: update to SRAM 1: update to both SRAM & FLASH */
   } message_0x37_t;
  
  
typedef struct PACKED message_0x38_t { 
    uint8_t  message_id ; /*  */
   } message_0x38_t;
  
  
typedef struct PACKED message_0x39_t { 
    uint8_t  message_id ; /*  */
    uint8_t  position_pinning ; /* 0: disable (default) 1: enable */
   } message_0x39_t;
  
  
typedef struct PACKED message_0x3A_t { 
    uint8_t  message_id ; /*  */
   } message_0x3A_t;
  
  
typedef struct PACKED message_0x3B_t { 
    uint8_t  message_id ; /*  */
    uint16_t  pinning_speed ; /*  */
    uint16_t  pinning_cnt ; /*  */
    uint16_t  unpinning_speed ; /*  */
    uint16_t  unpinning_cnt ; /*  */
    uint16_t  unpinning_distance ; /*  */
   } message_0x3B_t;
  
  
typedef struct PACKED message_0x3C_t { 
    uint8_t  message_id ; /*  */
    uint8_t  navigation_mode ; /* 0: car 1: pedestrian */
    uint8_t  attributes ; /* 0: update to SRAM 1: update to both SRAM & FLASH */
   } message_0x3C_t;
  
  
typedef struct PACKED message_0x3D_t { 
    uint8_t  message_id ; /*  */
   } message_0x3D_t;
  
  
typedef struct PACKED message_0x3E_t { 
    uint8_t  message_id ; /*  */
    uint8_t  n1pps_mode ; /* 0: off 1: on when 3D fix 2: on when 1 SV */
    uint8_t  attributes ; /* 0: update to SRAM 1: update to both SRAM & FLASH */
   } message_0x3E_t;
  
  
typedef struct PACKED message_0x3F_t { 
    uint8_t  message_id ; /*  */
   } message_0x3F_t;
  
  
typedef struct PACKED message_0x80_t { 
    uint8_t  message_id ; /*  */
    uint8_t  software_type ; /* 0: Reserved 1: System code */
    uint32_t  kernel_version ; /* X1.Y1.Z1 = SkyTraq Kernel Version Ex. X1=01, Y1=00, Z1=01 (1.0.1) */
    uint32_t  odm_version ; /* X1.Y1.Z1 = SkyTraq Version Ex. X1=01, Y1=03, Z1=01 (1.3.1) */
    uint32_t  revision ; /* YYMMDD = SkyTraq Revision Ex. YY=06, MM=01, DD=10 (060110) */
   } message_0x80_t;
  
  
typedef struct PACKED message_0x81_t { 
    uint8_t  message_id ; /*  */
    uint8_t  software_type ; /* 0: Reserved 1: System code */
    uint16_t  crc ; /* CRC value */
   } message_0x81_t;
  
  
typedef struct PACKED message_0x83_t { 
    uint8_t  message_id ; /*  */
    uint8_t  ack_id ; /* Message ID of the request message */
   } message_0x83_t;
  
  
typedef struct PACKED message_0x84_t { 
    uint8_t  message_id ; /*  */
    uint8_t  ack_id ; /* Message ID of the request message */
   } message_0x84_t;
  
  
typedef struct PACKED message_0x86_t { 
    uint8_t  message_id ; /*  */
    uint8_t  update_rate ; /* 01: 1Hz */
   } message_0x86_t;
  
  
typedef struct PACKED message_0xA8_t { 
    uint8_t  message_id ; /*  */
    uint8_t  fix_mode ; /* Quality of fix 0: no fix 1: 2D 2: 3D 3: 3D+DGPS */
    uint8_t  number_of_sv_in_fix ; /* Number of SV in fix 0-12 */
    uint16_t  gps_week ; /* GPS week number */
    uint32_t  tow ; /* GPS time of week Scaling 0.01 */
    int32_t  latitude ; /* > 0: North Hemisphere < 0: South Hemisphere Scaling 1e-7 */
    int32_t  longitude ; /* > 0: East Hemisphere < 0: West Hemisphere */
    uint32_t  ellipsoid_altitude ; /* height above ellipsoid Scaling 0.01 */
    uint32_t  mean_sea_level_altitude ; /* height above mean sea level Scaling 0.01 */
    uint16_t  gdop ; /* Geometric dilution of precision Scaling 0.01 */
    uint16_t  pdop ; /* Position dilution of precision Scaling 0.01 */
    uint16_t  hdop ; /* Horizontal dilution of precision Scaling 0.01 */
    uint16_t  vdop ; /* Vertical dilution of precision Scaling 0.01 */
    uint16_t  tdop ; /* Time dilution of precision Scaling 0.01 */
    int32_t  ecef_x ; /* ECEF X coordinate Scaling 0.01 */
    int32_t  ecef_y ; /* ECEF Y coordinate Scaling 0.01 */
    int32_t  ecef_z ; /* ECEF Z coordinate Scaling 0.01 */
    int32_t  ecef_vx ; /* ECEF X Veolcity Scaling 0.01 */
    int32_t  ecef_vy ; /* ECEF Y Veolcity Scaling 0.01 */
    int32_t  ecef_vz ; /* ECEF Z Veolcity Scaling 0.01 */
   } message_0xA8_t;
  
  
typedef struct PACKED message_0xAE_t { 
    uint8_t  message_id ; /*  */
    uint16_t  datum_index ; /* Datum index Refer to Appendix A & B */
   } message_0xAE_t;
  
  
typedef struct PACKED message_0xB1_t { 
    uint8_t  message_id ; /*  */
    uint16_t  sv_id ; /* Satellite id */
    uint8_t  subframedata_0_0 ; /* Eph data subframe 1 */
    uint8_t  subframedata_0_1 ; /* Eph data subframe 1 */
    uint8_t  subframedata_0_2 ; /* Eph data subframe 1 */
    uint8_t  subframedata_0_3 ; /* Eph data subframe 1 */
    uint8_t  subframedata_0_4 ; /* Eph data subframe 1 */
    uint8_t  subframedata_0_5 ; /* Eph data subframe 1 */
    uint8_t  subframedata_0_6 ; /* Eph data subframe 1 */
    uint8_t  subframedata_0_7 ; /* Eph data subframe 1 */
    uint8_t  subframedata_0_8 ; /* Eph data subframe 1 */
    uint8_t  subframedata_0_9 ; /* Eph data subframe 1 */
    uint8_t  subframedata_0_10 ; /* Eph data subframe 1 */
    uint8_t  subframedata_0_11 ; /* Eph data subframe 1 */
    uint8_t  subframedata_0_12 ; /* Eph data subframe 1 */
    uint8_t  subframedata_0_13 ; /* Eph data subframe 1 */
    uint8_t  subframedata_0_14 ; /* Eph data subframe 1 */
    uint8_t  subframedata_0_15 ; /* Eph data subframe 1 */
    uint8_t  subframedata_0_16 ; /* Eph data subframe 1 */
    uint8_t  subframedata_0_17 ; /* Eph data subframe 1 */
    uint8_t  subframedata_0_18 ; /* Eph data subframe 1 */
    uint8_t  subframedata_0_19 ; /* Eph data subframe 1 */
    uint8_t  subframedata_0_20 ; /* Eph data subframe 1 */
    uint8_t  subframedata_0_21 ; /* Eph data subframe 1 */
    uint8_t  subframedata_0_22 ; /* Eph data subframe 1 */
    uint8_t  subframedata_0_23 ; /* Eph data subframe 1 */
    uint8_t  subframedata_0_24 ; /* Eph data subframe 1 */
    uint8_t  subframedata_0_25 ; /* Eph data subframe 1 */
    uint8_t  subframedata_0_26 ; /* Eph data subframe 1 */
    uint8_t  subframedata_0_27 ; /* Eph data subframe 1 */
    uint8_t  subframedata_1_027 ; /* Eph data subframe 2, same as field 4-31 */
    uint8_t  subframedata_2_027 ; /* Eph data subframe 3, same as field 4-31 */
   } message_0xB1_t;
  
  
typedef struct PACKED message_0xB3_t { 
    uint8_t  message_id ; /*  */
    uint8_t  waas_status ; /* 0: disable 1: enable */
   } message_0xB3_t;
  
  
typedef struct PACKED message_0xB4_t { 
    uint8_t  message_id ; /*  */
    uint8_t  status ; /* 0: disable 1: enable */
   } message_0xB4_t;
  
  
typedef struct PACKED message_0xB5_t { 
    uint8_t  message_id ; /*  */
    uint8_t  navigation_mode ; /* 0: car 1: pedestrian */
   } message_0xB5_t;
  
  
typedef struct PACKED message_0xB6_t { 
    uint8_t  message_id ; /*  */
    uint8_t  n1pps_mode ; /* 0: off 1: on */
   } message_0xB6_t;
  
