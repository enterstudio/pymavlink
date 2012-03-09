using System;
using System.Collections.Generic;
using System.Text;
using System.Runtime.InteropServices;

namespace ArdupilotMega
{
    #if !MAVLINK10
    partial class MAVLink
    {
        public const string MAVLINK_BUILD_DATE = "Fri Mar  9 22:46:25 2012";
        public const string MAVLINK_WIRE_PROTOCOL_VERSION = "0.9";
        public const int MAVLINK_MAX_DIALECT_PAYLOAD_SIZE = 30;

        public const int MAVLINK_LITTLE_ENDIAN = 1;
        public const int MAVLINK_BIG_ENDIAN = 0;

        public const byte MAVLINK_STX = 85;

        public const byte MAVLINK_ENDIAN = MAVLINK_BIG_ENDIAN;

        public const bool MAVLINK_ALIGNED_FIELDS = (0 == 1);

        public const byte MAVLINK_CRC_EXTRA = 0;
        
        public const bool MAVLINK_NEED_BYTE_SWAP = (MAVLINK_ENDIAN == MAVLINK_LITTLE_ENDIAN);
        
        public byte packetcount = 0;
        
        public byte[] MAVLINK_MESSAGE_LENGTHS = new byte[] {3, 4, 8, 14, 8, 28, 3, 32, 0, 2, 3, 2, 2, 0, 0, 0, 0, 0, 0, 0, 19, 2, 23, 21, 0, 37, 26, 101, 26, 16, 32, 32, 37, 32, 11, 17, 17, 16, 18, 36, 4, 4, 2, 2, 4, 2, 2, 3, 14, 12, 18, 16, 8, 27, 25, 18, 18, 24, 24, 0, 0, 0, 26, 16, 36, 5, 6, 56, 26, 21, 18, 0, 0, 18, 20, 20, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 18, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 36, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4, 10, 24, 18, 0, 0, 30, 24, 0, 7, 13, 3, 0, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 30, 14, 14, 51, 5};

        public byte[] MAVLINK_MESSAGE_CRCS = new byte[] {72, 39, 190, 92, 191, 217, 104, 119, 0, 219, 60, 186, 10, 0, 0, 0, 0, 0, 0, 0, 89, 159, 162, 121, 0, 149, 222, 110, 179, 136, 66, 126, 185, 147, 112, 252, 162, 215, 229, 128, 9, 106, 101, 213, 4, 229, 21, 214, 215, 14, 206, 50, 157, 126, 108, 213, 95, 5, 127, 0, 0, 0, 57, 126, 130, 119, 193, 191, 236, 158, 143, 0, 0, 104, 123, 131, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 174, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 155, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 150, 232, 168, 2, 0, 0, 120, 167, 0, 16, 2, 52, 0, 202, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 178, 224, 60, 106, 7};

        public Type[] MAVLINK_MESSAGE_INFO = new Type[] {typeof( mavlink_heartbeat_t ), typeof( mavlink_boot_t ), typeof( mavlink_system_time_t ), typeof( mavlink_ping_t ), typeof( mavlink_system_time_utc_t ), typeof( mavlink_change_operator_control_t ), typeof( mavlink_change_operator_control_ack_t ), typeof( mavlink_auth_key_t ), null, typeof( mavlink_action_ack_t ), typeof( mavlink_action_t ), typeof( mavlink_set_mode_t ), typeof( mavlink_set_nav_mode_t ), null, null, null, null, null, null, null, typeof( mavlink_param_request_read_t ), typeof( mavlink_param_request_list_t ), typeof( mavlink_param_value_t ), typeof( mavlink_param_set_t ), null, typeof( mavlink_gps_raw_int_t ), typeof( mavlink_scaled_imu_t ), typeof( mavlink_gps_status_t ), typeof( mavlink_raw_imu_t ), typeof( mavlink_raw_pressure_t ), typeof( mavlink_attitude_t ), typeof( mavlink_local_position_t ), typeof( mavlink_gps_raw_t ), typeof( mavlink_global_position_t ), typeof( mavlink_sys_status_t ), typeof( mavlink_rc_channels_raw_t ), typeof( mavlink_rc_channels_scaled_t ), typeof( mavlink_servo_output_raw_t ), typeof( mavlink_scaled_pressure_t ), typeof( mavlink_waypoint_t ), typeof( mavlink_waypoint_request_t ), typeof( mavlink_waypoint_set_current_t ), typeof( mavlink_waypoint_current_t ), typeof( mavlink_waypoint_request_list_t ), typeof( mavlink_waypoint_count_t ), typeof( mavlink_waypoint_clear_all_t ), typeof( mavlink_waypoint_reached_t ), typeof( mavlink_waypoint_ack_t ), typeof( mavlink_gps_set_global_origin_t ), typeof( mavlink_gps_local_origin_set_t ), typeof( mavlink_local_position_setpoint_set_t ), typeof( mavlink_local_position_setpoint_t ), typeof( mavlink_control_status_t ), typeof( mavlink_safety_set_allowed_area_t ), typeof( mavlink_safety_allowed_area_t ), typeof( mavlink_set_roll_pitch_yaw_thrust_t ), typeof( mavlink_set_roll_pitch_yaw_speed_thrust_t ), typeof( mavlink_roll_pitch_yaw_thrust_setpoint_t ), typeof( mavlink_roll_pitch_yaw_speed_thrust_setpoint_t ), null, null, null, typeof( mavlink_nav_controller_output_t ), typeof( mavlink_position_target_t ), typeof( mavlink_state_correction_t ), typeof( mavlink_set_altitude_t ), typeof( mavlink_request_data_stream_t ), typeof( mavlink_hil_state_t ), typeof( mavlink_hil_controls_t ), typeof( mavlink_manual_control_t ), typeof( mavlink_rc_channels_override_t ), null, null, typeof( mavlink_global_position_int_t ), typeof( mavlink_vfr_hud_t ), typeof( mavlink_command_t ), typeof( mavlink_command_ack_t ), null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, typeof( mavlink_optical_flow_t ), null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, typeof( mavlink_object_detection_event_t ), null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, typeof( mavlink_cpu_load_t ), typeof( mavlink_air_data_t ), typeof( mavlink_sensor_bias_t ), typeof( mavlink_diagnostic_t ), null, null, typeof( mavlink_slugs_navigation_t ), typeof( mavlink_data_log_t ), null, typeof( mavlink_gps_date_time_t ), typeof( mavlink_mid_lvl_cmds_t ), typeof( mavlink_ctrl_srfc_pt_t ), null, typeof( mavlink_slugs_action_t ), null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, typeof( mavlink_debug_vect_t ), typeof( mavlink_named_value_float_t ), typeof( mavlink_named_value_int_t ), typeof( mavlink_statustext_t ), typeof( mavlink_debug_t )};

        public const byte MAVLINK_VERSION = 2;
    
        
    

    public const byte MAVLINK_MSG_ID_CPU_LOAD = 170;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=4)]
    public struct mavlink_cpu_load_t
    {
        /// <summary> Sensor DSC Load </summary>
        public  byte sensLoad;
            /// <summary> Control DSC Load </summary>
        public  byte ctrlLoad;
            /// <summary> Battery Voltage in millivolts </summary>
        public  UInt16 batVolt;
    
    };


    public const byte MAVLINK_MSG_ID_AIR_DATA = 171;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=10)]
    public struct mavlink_air_data_t
    {
        /// <summary> Dynamic pressure (Pa) </summary>
        public  Single dynamicPressure;
            /// <summary> Static pressure (Pa) </summary>
        public  Single staticPressure;
            /// <summary> Board temperature </summary>
        public  UInt16 temperature;
    
    };


    public const byte MAVLINK_MSG_ID_SENSOR_BIAS = 172;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=24)]
    public struct mavlink_sensor_bias_t
    {
        /// <summary> Accelerometer X bias (m/s) </summary>
        public  Single axBias;
            /// <summary> Accelerometer Y bias (m/s) </summary>
        public  Single ayBias;
            /// <summary> Accelerometer Z bias (m/s) </summary>
        public  Single azBias;
            /// <summary> Gyro X bias (rad/s) </summary>
        public  Single gxBias;
            /// <summary> Gyro Y bias (rad/s) </summary>
        public  Single gyBias;
            /// <summary> Gyro Z bias (rad/s) </summary>
        public  Single gzBias;
    
    };


    public const byte MAVLINK_MSG_ID_DIAGNOSTIC = 173;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=18)]
    public struct mavlink_diagnostic_t
    {
        /// <summary> Diagnostic float 1 </summary>
        public  Single diagFl1;
            /// <summary> Diagnostic float 2 </summary>
        public  Single diagFl2;
            /// <summary> Diagnostic float 3 </summary>
        public  Single diagFl3;
            /// <summary> Diagnostic short 1 </summary>
        public  Int16 diagSh1;
            /// <summary> Diagnostic short 2 </summary>
        public  Int16 diagSh2;
            /// <summary> Diagnostic short 3 </summary>
        public  Int16 diagSh3;
    
    };


    public const byte MAVLINK_MSG_ID_SLUGS_NAVIGATION = 176;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=30)]
    public struct mavlink_slugs_navigation_t
    {
        /// <summary> Measured Airspeed prior to the Nav Filter </summary>
        public  Single u_m;
            /// <summary> Commanded Roll </summary>
        public  Single phi_c;
            /// <summary> Commanded Pitch </summary>
        public  Single theta_c;
            /// <summary> Commanded Turn rate </summary>
        public  Single psiDot_c;
            /// <summary> Y component of the body acceleration </summary>
        public  Single ay_body;
            /// <summary> Total Distance to Run on this leg of Navigation </summary>
        public  Single totalDist;
            /// <summary> Remaining distance to Run on this leg of Navigation </summary>
        public  Single dist2Go;
            /// <summary> Origin WP </summary>
        public  byte fromWP;
            /// <summary> Destination WP </summary>
        public  byte toWP;
    
    };


    public const byte MAVLINK_MSG_ID_DATA_LOG = 177;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=24)]
    public struct mavlink_data_log_t
    {
        /// <summary> Log value 1  </summary>
        public  Single fl_1;
            /// <summary> Log value 2  </summary>
        public  Single fl_2;
            /// <summary> Log value 3  </summary>
        public  Single fl_3;
            /// <summary> Log value 4  </summary>
        public  Single fl_4;
            /// <summary> Log value 5  </summary>
        public  Single fl_5;
            /// <summary> Log value 6  </summary>
        public  Single fl_6;
    
    };


    public const byte MAVLINK_MSG_ID_GPS_DATE_TIME = 179;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=7)]
    public struct mavlink_gps_date_time_t
    {
        /// <summary> Year reported by Gps  </summary>
        public  byte year;
            /// <summary> Month reported by Gps  </summary>
        public  byte month;
            /// <summary> Day reported by Gps  </summary>
        public  byte day;
            /// <summary> Hour reported by Gps  </summary>
        public  byte hour;
            /// <summary> Min reported by Gps  </summary>
        public  byte min;
            /// <summary> Sec reported by Gps   </summary>
        public  byte sec;
            /// <summary> Visible sattelites reported by Gps   </summary>
        public  byte visSat;
    
    };


    public const byte MAVLINK_MSG_ID_MID_LVL_CMDS = 180;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=13)]
    public struct mavlink_mid_lvl_cmds_t
    {
        /// <summary> The system setting the commands </summary>
        public  byte target;
            /// <summary> Commanded Airspeed </summary>
        public  Single hCommand;
            /// <summary> Log value 2  </summary>
        public  Single uCommand;
            /// <summary> Log value 3  </summary>
        public  Single rCommand;
    
    };


    public const byte MAVLINK_MSG_ID_CTRL_SRFC_PT = 181;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=3)]
    public struct mavlink_ctrl_srfc_pt_t
    {
        /// <summary> The system setting the commands </summary>
        public  byte target;
            /// <summary> Bitfield containing the PT configuration </summary>
        public  UInt16 bitfieldPt;
    
    };


    public const byte MAVLINK_MSG_ID_SLUGS_ACTION = 183;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=4)]
    public struct mavlink_slugs_action_t
    {
        /// <summary> The system reporting the action </summary>
        public  byte target;
            /// <summary> Action ID. See apDefinitions.h in the SLUGS /clib directory for the ID names </summary>
        public  byte actionId;
            /// <summary> Value associated with the action </summary>
        public  UInt16 actionVal;
    
    };


    public const byte MAVLINK_MSG_ID_HEARTBEAT = 0;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=3)]
    public struct mavlink_heartbeat_t
    {
        /// <summary> Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM) </summary>
        public  byte type;
            /// <summary> Type of the Autopilot: 0: Generic, 1: PIXHAWK, 2: SLUGS, 3: Ardupilot (up to 15 types), defined in MAV_AUTOPILOT_TYPE ENUM </summary>
        public  byte autopilot;
            /// <summary> MAVLink version </summary>
        public  byte mavlink_version;
    
    };


    public const byte MAVLINK_MSG_ID_BOOT = 1;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=4)]
    public struct mavlink_boot_t
    {
        /// <summary> The onboard software version </summary>
        public  UInt32 version;
    
    };


    public const byte MAVLINK_MSG_ID_SYSTEM_TIME = 2;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=8)]
    public struct mavlink_system_time_t
    {
        /// <summary> Timestamp of the master clock in microseconds since UNIX epoch. </summary>
        public  UInt64 time_usec;
    
    };


    public const byte MAVLINK_MSG_ID_PING = 3;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=14)]
    public struct mavlink_ping_t
    {
        /// <summary> PING sequence </summary>
        public  UInt32 seq;
            /// <summary> 0: request ping from all receiving systems, if greater than 0: message is a ping response and number is the system id of the requesting system </summary>
        public  byte target_system;
            /// <summary> 0: request ping from all receiving components, if greater than 0: message is a ping response and number is the system id of the requesting system </summary>
        public  byte target_component;
            /// <summary> Unix timestamp in microseconds </summary>
        public  UInt64 time;
    
    };


    public const byte MAVLINK_MSG_ID_SYSTEM_TIME_UTC = 4;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=8)]
    public struct mavlink_system_time_utc_t
    {
        /// <summary> GPS UTC date ddmmyy </summary>
        public  UInt32 utc_date;
            /// <summary> GPS UTC time hhmmss </summary>
        public  UInt32 utc_time;
    
    };


    public const byte MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL = 5;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=28)]
    public struct mavlink_change_operator_control_t
    {
        /// <summary> System the GCS requests control for </summary>
        public  byte target_system;
            /// <summary> 0: request control of this MAV, 1: Release control of this MAV </summary>
        public  byte control_request;
            /// <summary> 0: key as plaintext, 1-255: future, different hashing/encryption variants. The GCS should in general use the safest mode possible initially and then gradually move down the encryption level if it gets a NACK message indicating an encryption mismatch. </summary>
        public  byte version;
            /// <summary> Password / Key, depending on version plaintext or encrypted. 25 or less characters, NULL terminated. The characters may involve A-Z, a-z, 0-9, and "!?,.-" </summary>
        [MarshalAs(UnmanagedType.ByValArray,SizeConst=25)]
		public string passkey;
    
    };


    public const byte MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL_ACK = 6;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=3)]
    public struct mavlink_change_operator_control_ack_t
    {
        /// <summary> ID of the GCS this message  </summary>
        public  byte gcs_system_id;
            /// <summary> 0: request control of this MAV, 1: Release control of this MAV </summary>
        public  byte control_request;
            /// <summary> 0: ACK, 1: NACK: Wrong passkey, 2: NACK: Unsupported passkey encryption method, 3: NACK: Already under control </summary>
        public  byte ack;
    
    };


    public const byte MAVLINK_MSG_ID_AUTH_KEY = 7;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=32)]
    public struct mavlink_auth_key_t
    {
        /// <summary> key </summary>
        [MarshalAs(UnmanagedType.ByValArray,SizeConst=32)]
		public string key;
    
    };


    public const byte MAVLINK_MSG_ID_ACTION_ACK = 9;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=2)]
    public struct mavlink_action_ack_t
    {
        /// <summary> The action id </summary>
        public  byte action;
            /// <summary> 0: Action DENIED, 1: Action executed </summary>
        public  byte result;
    
    };


    public const byte MAVLINK_MSG_ID_ACTION = 10;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=3)]
    public struct mavlink_action_t
    {
        /// <summary> The system executing the action </summary>
        public  byte target;
            /// <summary> The component executing the action </summary>
        public  byte target_component;
            /// <summary> The action id </summary>
        public  byte action;
    
    };


    public const byte MAVLINK_MSG_ID_SET_MODE = 11;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=2)]
    public struct mavlink_set_mode_t
    {
        /// <summary> The system setting the mode </summary>
        public  byte target;
            /// <summary> The new mode </summary>
        public  byte mode;
    
    };


    public const byte MAVLINK_MSG_ID_SET_NAV_MODE = 12;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=2)]
    public struct mavlink_set_nav_mode_t
    {
        /// <summary> The system setting the mode </summary>
        public  byte target;
            /// <summary> The new navigation mode </summary>
        public  byte nav_mode;
    
    };


    public const byte MAVLINK_MSG_ID_PARAM_REQUEST_READ = 20;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=19)]
    public struct mavlink_param_request_read_t
    {
        /// <summary> System ID </summary>
        public  byte target_system;
            /// <summary> Component ID </summary>
        public  byte target_component;
            /// <summary> Onboard parameter id </summary>
        [MarshalAs(UnmanagedType.ByValArray,SizeConst=15)]
		public byte[] param_id;
            /// <summary> Parameter index. Send -1 to use the param ID field as identifier </summary>
        public  Int16 param_index;
    
    };


    public const byte MAVLINK_MSG_ID_PARAM_REQUEST_LIST = 21;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=2)]
    public struct mavlink_param_request_list_t
    {
        /// <summary> System ID </summary>
        public  byte target_system;
            /// <summary> Component ID </summary>
        public  byte target_component;
    
    };


    public const byte MAVLINK_MSG_ID_PARAM_VALUE = 22;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=23)]
    public struct mavlink_param_value_t
    {
        /// <summary> Onboard parameter id </summary>
        [MarshalAs(UnmanagedType.ByValArray,SizeConst=15)]
		public byte[] param_id;
            /// <summary> Onboard parameter value </summary>
        public  Single param_value;
            /// <summary> Total number of onboard parameters </summary>
        public  UInt16 param_count;
            /// <summary> Index of this onboard parameter </summary>
        public  UInt16 param_index;
    
    };


    public const byte MAVLINK_MSG_ID_PARAM_SET = 23;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=21)]
    public struct mavlink_param_set_t
    {
        /// <summary> System ID </summary>
        public  byte target_system;
            /// <summary> Component ID </summary>
        public  byte target_component;
            /// <summary> Onboard parameter id </summary>
        [MarshalAs(UnmanagedType.ByValArray,SizeConst=15)]
		public byte[] param_id;
            /// <summary> Onboard parameter value </summary>
        public  Single param_value;
    
    };


    public const byte MAVLINK_MSG_ID_GPS_RAW_INT = 25;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=37)]
    public struct mavlink_gps_raw_int_t
    {
        /// <summary> Timestamp (microseconds since UNIX epoch or microseconds since system boot) </summary>
        public  UInt64 usec;
            /// <summary> 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix. </summary>
        public  byte fix_type;
            /// <summary> Latitude in 1E7 degrees </summary>
        public  Int32 lat;
            /// <summary> Longitude in 1E7 degrees </summary>
        public  Int32 lon;
            /// <summary> Altitude in 1E3 meters (millimeters) </summary>
        public  Int32 alt;
            /// <summary> GPS HDOP </summary>
        public  Single eph;
            /// <summary> GPS VDOP </summary>
        public  Single epv;
            /// <summary> GPS ground speed (m/s) </summary>
        public  Single v;
            /// <summary> Compass heading in degrees, 0..360 degrees </summary>
        public  Single hdg;
    
    };


    public const byte MAVLINK_MSG_ID_SCALED_IMU = 26;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=26)]
    public struct mavlink_scaled_imu_t
    {
        /// <summary> Timestamp (microseconds since UNIX epoch or microseconds since system boot) </summary>
        public  UInt64 usec;
            /// <summary> X acceleration (mg) </summary>
        public  Int16 xacc;
            /// <summary> Y acceleration (mg) </summary>
        public  Int16 yacc;
            /// <summary> Z acceleration (mg) </summary>
        public  Int16 zacc;
            /// <summary> Angular speed around X axis (millirad /sec) </summary>
        public  Int16 xgyro;
            /// <summary> Angular speed around Y axis (millirad /sec) </summary>
        public  Int16 ygyro;
            /// <summary> Angular speed around Z axis (millirad /sec) </summary>
        public  Int16 zgyro;
            /// <summary> X Magnetic field (milli tesla) </summary>
        public  Int16 xmag;
            /// <summary> Y Magnetic field (milli tesla) </summary>
        public  Int16 ymag;
            /// <summary> Z Magnetic field (milli tesla) </summary>
        public  Int16 zmag;
    
    };


    public const byte MAVLINK_MSG_ID_GPS_STATUS = 27;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=101)]
    public struct mavlink_gps_status_t
    {
        /// <summary> Number of satellites visible </summary>
        public  byte satellites_visible;
            /// <summary> Global satellite ID </summary>
        [MarshalAs(UnmanagedType.ByValArray,SizeConst=20)]
		public byte[] satellite_prn;
            /// <summary> 0: Satellite not used, 1: used for localization </summary>
        [MarshalAs(UnmanagedType.ByValArray,SizeConst=20)]
		public byte[] satellite_used;
            /// <summary> Elevation (0: right on top of receiver, 90: on the horizon) of satellite </summary>
        [MarshalAs(UnmanagedType.ByValArray,SizeConst=20)]
		public byte[] satellite_elevation;
            /// <summary> Direction of satellite, 0: 0 deg, 255: 360 deg. </summary>
        [MarshalAs(UnmanagedType.ByValArray,SizeConst=20)]
		public byte[] satellite_azimuth;
            /// <summary> Signal to noise ratio of satellite </summary>
        [MarshalAs(UnmanagedType.ByValArray,SizeConst=20)]
		public byte[] satellite_snr;
    
    };


    public const byte MAVLINK_MSG_ID_RAW_IMU = 28;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=26)]
    public struct mavlink_raw_imu_t
    {
        /// <summary> Timestamp (microseconds since UNIX epoch or microseconds since system boot) </summary>
        public  UInt64 usec;
            /// <summary> X acceleration (raw) </summary>
        public  Int16 xacc;
            /// <summary> Y acceleration (raw) </summary>
        public  Int16 yacc;
            /// <summary> Z acceleration (raw) </summary>
        public  Int16 zacc;
            /// <summary> Angular speed around X axis (raw) </summary>
        public  Int16 xgyro;
            /// <summary> Angular speed around Y axis (raw) </summary>
        public  Int16 ygyro;
            /// <summary> Angular speed around Z axis (raw) </summary>
        public  Int16 zgyro;
            /// <summary> X Magnetic field (raw) </summary>
        public  Int16 xmag;
            /// <summary> Y Magnetic field (raw) </summary>
        public  Int16 ymag;
            /// <summary> Z Magnetic field (raw) </summary>
        public  Int16 zmag;
    
    };


    public const byte MAVLINK_MSG_ID_RAW_PRESSURE = 29;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=16)]
    public struct mavlink_raw_pressure_t
    {
        /// <summary> Timestamp (microseconds since UNIX epoch or microseconds since system boot) </summary>
        public  UInt64 usec;
            /// <summary> Absolute pressure (raw) </summary>
        public  Int16 press_abs;
            /// <summary> Differential pressure 1 (raw) </summary>
        public  Int16 press_diff1;
            /// <summary> Differential pressure 2 (raw) </summary>
        public  Int16 press_diff2;
            /// <summary> Raw Temperature measurement (raw) </summary>
        public  Int16 temperature;
    
    };


    public const byte MAVLINK_MSG_ID_SCALED_PRESSURE = 38;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=18)]
    public struct mavlink_scaled_pressure_t
    {
        /// <summary> Timestamp (microseconds since UNIX epoch or microseconds since system boot) </summary>
        public  UInt64 usec;
            /// <summary> Absolute pressure (hectopascal) </summary>
        public  Single press_abs;
            /// <summary> Differential pressure 1 (hectopascal) </summary>
        public  Single press_diff;
            /// <summary> Temperature measurement (0.01 degrees celsius) </summary>
        public  Int16 temperature;
    
    };


    public const byte MAVLINK_MSG_ID_ATTITUDE = 30;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=32)]
    public struct mavlink_attitude_t
    {
        /// <summary> Timestamp (microseconds since UNIX epoch or microseconds since system boot) </summary>
        public  UInt64 usec;
            /// <summary> Roll angle (rad) </summary>
        public  Single roll;
            /// <summary> Pitch angle (rad) </summary>
        public  Single pitch;
            /// <summary> Yaw angle (rad) </summary>
        public  Single yaw;
            /// <summary> Roll angular speed (rad/s) </summary>
        public  Single rollspeed;
            /// <summary> Pitch angular speed (rad/s) </summary>
        public  Single pitchspeed;
            /// <summary> Yaw angular speed (rad/s) </summary>
        public  Single yawspeed;
    
    };


    public const byte MAVLINK_MSG_ID_LOCAL_POSITION = 31;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=32)]
    public struct mavlink_local_position_t
    {
        /// <summary> Timestamp (microseconds since UNIX epoch or microseconds since system boot) </summary>
        public  UInt64 usec;
            /// <summary> X Position </summary>
        public  Single x;
            /// <summary> Y Position </summary>
        public  Single y;
            /// <summary> Z Position </summary>
        public  Single z;
            /// <summary> X Speed </summary>
        public  Single vx;
            /// <summary> Y Speed </summary>
        public  Single vy;
            /// <summary> Z Speed </summary>
        public  Single vz;
    
    };


    public const byte MAVLINK_MSG_ID_GLOBAL_POSITION = 33;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=32)]
    public struct mavlink_global_position_t
    {
        /// <summary> Timestamp (microseconds since unix epoch) </summary>
        public  UInt64 usec;
            /// <summary> Latitude, in degrees </summary>
        public  Single lat;
            /// <summary> Longitude, in degrees </summary>
        public  Single lon;
            /// <summary> Absolute altitude, in meters </summary>
        public  Single alt;
            /// <summary> X Speed (in Latitude direction, positive: going north) </summary>
        public  Single vx;
            /// <summary> Y Speed (in Longitude direction, positive: going east) </summary>
        public  Single vy;
            /// <summary> Z Speed (in Altitude direction, positive: going up) </summary>
        public  Single vz;
    
    };


    public const byte MAVLINK_MSG_ID_GPS_RAW = 32;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=37)]
    public struct mavlink_gps_raw_t
    {
        /// <summary> Timestamp (microseconds since UNIX epoch or microseconds since system boot) </summary>
        public  UInt64 usec;
            /// <summary> 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix. </summary>
        public  byte fix_type;
            /// <summary> Latitude in degrees </summary>
        public  Single lat;
            /// <summary> Longitude in degrees </summary>
        public  Single lon;
            /// <summary> Altitude in meters </summary>
        public  Single alt;
            /// <summary> GPS HDOP </summary>
        public  Single eph;
            /// <summary> GPS VDOP </summary>
        public  Single epv;
            /// <summary> GPS ground speed </summary>
        public  Single v;
            /// <summary> Compass heading in degrees, 0..360 degrees </summary>
        public  Single hdg;
    
    };


    public const byte MAVLINK_MSG_ID_SYS_STATUS = 34;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=11)]
    public struct mavlink_sys_status_t
    {
        /// <summary> System mode, see MAV_MODE ENUM in mavlink/include/mavlink_types.h </summary>
        public  byte mode;
            /// <summary> Navigation mode, see MAV_NAV_MODE ENUM </summary>
        public  byte nav_mode;
            /// <summary> System status flag, see MAV_STATUS ENUM </summary>
        public  byte status;
            /// <summary> Maximum usage in percent of the mainloop time, (0%: 0, 100%: 1000) should be always below 1000 </summary>
        public  UInt16 load;
            /// <summary> Battery voltage, in millivolts (1 = 1 millivolt) </summary>
        public  UInt16 vbat;
            /// <summary> Remaining battery energy: (0%: 0, 100%: 1000) </summary>
        public  UInt16 battery_remaining;
            /// <summary> Dropped packets (packets that were corrupted on reception on the MAV) </summary>
        public  UInt16 packet_drop;
    
    };


    public const byte MAVLINK_MSG_ID_RC_CHANNELS_RAW = 35;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=17)]
    public struct mavlink_rc_channels_raw_t
    {
        /// <summary> RC channel 1 value, in microseconds </summary>
        public  UInt16 chan1_raw;
            /// <summary> RC channel 2 value, in microseconds </summary>
        public  UInt16 chan2_raw;
            /// <summary> RC channel 3 value, in microseconds </summary>
        public  UInt16 chan3_raw;
            /// <summary> RC channel 4 value, in microseconds </summary>
        public  UInt16 chan4_raw;
            /// <summary> RC channel 5 value, in microseconds </summary>
        public  UInt16 chan5_raw;
            /// <summary> RC channel 6 value, in microseconds </summary>
        public  UInt16 chan6_raw;
            /// <summary> RC channel 7 value, in microseconds </summary>
        public  UInt16 chan7_raw;
            /// <summary> RC channel 8 value, in microseconds </summary>
        public  UInt16 chan8_raw;
            /// <summary> Receive signal strength indicator, 0: 0%, 255: 100% </summary>
        public  byte rssi;
    
    };


    public const byte MAVLINK_MSG_ID_RC_CHANNELS_SCALED = 36;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=17)]
    public struct mavlink_rc_channels_scaled_t
    {
        /// <summary> RC channel 1 value scaled, (-100%) -10000, (0%) 0, (100%) 10000 </summary>
        public  Int16 chan1_scaled;
            /// <summary> RC channel 2 value scaled, (-100%) -10000, (0%) 0, (100%) 10000 </summary>
        public  Int16 chan2_scaled;
            /// <summary> RC channel 3 value scaled, (-100%) -10000, (0%) 0, (100%) 10000 </summary>
        public  Int16 chan3_scaled;
            /// <summary> RC channel 4 value scaled, (-100%) -10000, (0%) 0, (100%) 10000 </summary>
        public  Int16 chan4_scaled;
            /// <summary> RC channel 5 value scaled, (-100%) -10000, (0%) 0, (100%) 10000 </summary>
        public  Int16 chan5_scaled;
            /// <summary> RC channel 6 value scaled, (-100%) -10000, (0%) 0, (100%) 10000 </summary>
        public  Int16 chan6_scaled;
            /// <summary> RC channel 7 value scaled, (-100%) -10000, (0%) 0, (100%) 10000 </summary>
        public  Int16 chan7_scaled;
            /// <summary> RC channel 8 value scaled, (-100%) -10000, (0%) 0, (100%) 10000 </summary>
        public  Int16 chan8_scaled;
            /// <summary> Receive signal strength indicator, 0: 0%, 255: 100% </summary>
        public  byte rssi;
    
    };


    public const byte MAVLINK_MSG_ID_SERVO_OUTPUT_RAW = 37;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=16)]
    public struct mavlink_servo_output_raw_t
    {
        /// <summary> Servo output 1 value, in microseconds </summary>
        public  UInt16 servo1_raw;
            /// <summary> Servo output 2 value, in microseconds </summary>
        public  UInt16 servo2_raw;
            /// <summary> Servo output 3 value, in microseconds </summary>
        public  UInt16 servo3_raw;
            /// <summary> Servo output 4 value, in microseconds </summary>
        public  UInt16 servo4_raw;
            /// <summary> Servo output 5 value, in microseconds </summary>
        public  UInt16 servo5_raw;
            /// <summary> Servo output 6 value, in microseconds </summary>
        public  UInt16 servo6_raw;
            /// <summary> Servo output 7 value, in microseconds </summary>
        public  UInt16 servo7_raw;
            /// <summary> Servo output 8 value, in microseconds </summary>
        public  UInt16 servo8_raw;
    
    };


    public const byte MAVLINK_MSG_ID_WAYPOINT = 39;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=36)]
    public struct mavlink_waypoint_t
    {
        /// <summary> System ID </summary>
        public  byte target_system;
            /// <summary> Component ID </summary>
        public  byte target_component;
            /// <summary> Sequence </summary>
        public  UInt16 seq;
            /// <summary> The coordinate system of the waypoint. see MAV_FRAME in mavlink_types.h </summary>
        public  byte frame;
            /// <summary> The scheduled action for the waypoint. see MAV_COMMAND in common.xml MAVLink specs </summary>
        public  byte command;
            /// <summary> false:0, true:1 </summary>
        public  byte current;
            /// <summary> autocontinue to next wp </summary>
        public  byte autocontinue;
            /// <summary> PARAM1 / For NAV command waypoints: Radius in which the waypoint is accepted as reached, in meters </summary>
        public  Single param1;
            /// <summary> PARAM2 / For NAV command waypoints: Time that the MAV should stay inside the PARAM1 radius before advancing, in milliseconds </summary>
        public  Single param2;
            /// <summary> PARAM3 / For LOITER command waypoints: Orbit to circle around the waypoint, in meters. If positive the orbit direction should be clockwise, if negative the orbit direction should be counter-clockwise. </summary>
        public  Single param3;
            /// <summary> PARAM4 / For NAV and LOITER command waypoints: Yaw orientation in degrees, [0..360] 0 = NORTH </summary>
        public  Single param4;
            /// <summary> PARAM5 / local: x position, global: latitude </summary>
        public  Single x;
            /// <summary> PARAM6 / y position: global: longitude </summary>
        public  Single y;
            /// <summary> PARAM7 / z position: global: altitude </summary>
        public  Single z;
    
    };


    public const byte MAVLINK_MSG_ID_WAYPOINT_REQUEST = 40;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=4)]
    public struct mavlink_waypoint_request_t
    {
        /// <summary> System ID </summary>
        public  byte target_system;
            /// <summary> Component ID </summary>
        public  byte target_component;
            /// <summary> Sequence </summary>
        public  UInt16 seq;
    
    };


    public const byte MAVLINK_MSG_ID_WAYPOINT_SET_CURRENT = 41;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=4)]
    public struct mavlink_waypoint_set_current_t
    {
        /// <summary> System ID </summary>
        public  byte target_system;
            /// <summary> Component ID </summary>
        public  byte target_component;
            /// <summary> Sequence </summary>
        public  UInt16 seq;
    
    };


    public const byte MAVLINK_MSG_ID_WAYPOINT_CURRENT = 42;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=2)]
    public struct mavlink_waypoint_current_t
    {
        /// <summary> Sequence </summary>
        public  UInt16 seq;
    
    };


    public const byte MAVLINK_MSG_ID_WAYPOINT_REQUEST_LIST = 43;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=2)]
    public struct mavlink_waypoint_request_list_t
    {
        /// <summary> System ID </summary>
        public  byte target_system;
            /// <summary> Component ID </summary>
        public  byte target_component;
    
    };


    public const byte MAVLINK_MSG_ID_WAYPOINT_COUNT = 44;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=4)]
    public struct mavlink_waypoint_count_t
    {
        /// <summary> System ID </summary>
        public  byte target_system;
            /// <summary> Component ID </summary>
        public  byte target_component;
            /// <summary> Number of Waypoints in the Sequence </summary>
        public  UInt16 count;
    
    };


    public const byte MAVLINK_MSG_ID_WAYPOINT_CLEAR_ALL = 45;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=2)]
    public struct mavlink_waypoint_clear_all_t
    {
        /// <summary> System ID </summary>
        public  byte target_system;
            /// <summary> Component ID </summary>
        public  byte target_component;
    
    };


    public const byte MAVLINK_MSG_ID_WAYPOINT_REACHED = 46;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=2)]
    public struct mavlink_waypoint_reached_t
    {
        /// <summary> Sequence </summary>
        public  UInt16 seq;
    
    };


    public const byte MAVLINK_MSG_ID_WAYPOINT_ACK = 47;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=3)]
    public struct mavlink_waypoint_ack_t
    {
        /// <summary> System ID </summary>
        public  byte target_system;
            /// <summary> Component ID </summary>
        public  byte target_component;
            /// <summary> 0: OK, 1: Error </summary>
        public  byte type;
    
    };


    public const byte MAVLINK_MSG_ID_GPS_SET_GLOBAL_ORIGIN = 48;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=14)]
    public struct mavlink_gps_set_global_origin_t
    {
        /// <summary> System ID </summary>
        public  byte target_system;
            /// <summary> Component ID </summary>
        public  byte target_component;
            /// <summary> global position * 1E7 </summary>
        public  Int32 latitude;
            /// <summary> global position * 1E7 </summary>
        public  Int32 longitude;
            /// <summary> global position * 1000 </summary>
        public  Int32 altitude;
    
    };


    public const byte MAVLINK_MSG_ID_GPS_LOCAL_ORIGIN_SET = 49;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=12)]
    public struct mavlink_gps_local_origin_set_t
    {
        /// <summary> Latitude (WGS84), expressed as * 1E7 </summary>
        public  Int32 latitude;
            /// <summary> Longitude (WGS84), expressed as * 1E7 </summary>
        public  Int32 longitude;
            /// <summary> Altitude(WGS84), expressed as * 1000 </summary>
        public  Int32 altitude;
    
    };


    public const byte MAVLINK_MSG_ID_LOCAL_POSITION_SETPOINT_SET = 50;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=18)]
    public struct mavlink_local_position_setpoint_set_t
    {
        /// <summary> System ID </summary>
        public  byte target_system;
            /// <summary> Component ID </summary>
        public  byte target_component;
            /// <summary> x position </summary>
        public  Single x;
            /// <summary> y position </summary>
        public  Single y;
            /// <summary> z position </summary>
        public  Single z;
            /// <summary> Desired yaw angle </summary>
        public  Single yaw;
    
    };


    public const byte MAVLINK_MSG_ID_LOCAL_POSITION_SETPOINT = 51;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=16)]
    public struct mavlink_local_position_setpoint_t
    {
        /// <summary> x position </summary>
        public  Single x;
            /// <summary> y position </summary>
        public  Single y;
            /// <summary> z position </summary>
        public  Single z;
            /// <summary> Desired yaw angle </summary>
        public  Single yaw;
    
    };


    public const byte MAVLINK_MSG_ID_CONTROL_STATUS = 52;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=8)]
    public struct mavlink_control_status_t
    {
        /// <summary> Position fix: 0: lost, 2: 2D position fix, 3: 3D position fix  </summary>
        public  byte position_fix;
            /// <summary> Vision position fix: 0: lost, 1: 2D local position hold, 2: 2D global position fix, 3: 3D global position fix  </summary>
        public  byte vision_fix;
            /// <summary> GPS position fix: 0: no reception, 1: Minimum 1 satellite, but no position fix, 2: 2D position fix, 3: 3D position fix  </summary>
        public  byte gps_fix;
            /// <summary> Attitude estimation health: 0: poor, 255: excellent </summary>
        public  byte ahrs_health;
            /// <summary> 0: Attitude control disabled, 1: enabled </summary>
        public  byte control_att;
            /// <summary> 0: X, Y position control disabled, 1: enabled </summary>
        public  byte control_pos_xy;
            /// <summary> 0: Z position control disabled, 1: enabled </summary>
        public  byte control_pos_z;
            /// <summary> 0: Yaw angle control disabled, 1: enabled </summary>
        public  byte control_pos_yaw;
    
    };


    public const byte MAVLINK_MSG_ID_SAFETY_SET_ALLOWED_AREA = 53;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=27)]
    public struct mavlink_safety_set_allowed_area_t
    {
        /// <summary> System ID </summary>
        public  byte target_system;
            /// <summary> Component ID </summary>
        public  byte target_component;
            /// <summary> Coordinate frame, as defined by MAV_FRAME enum in mavlink_types.h. Can be either global, GPS, right-handed with Z axis up or local, right handed, Z axis down. </summary>
        public  byte frame;
            /// <summary> x position 1 / Latitude 1 </summary>
        public  Single p1x;
            /// <summary> y position 1 / Longitude 1 </summary>
        public  Single p1y;
            /// <summary> z position 1 / Altitude 1 </summary>
        public  Single p1z;
            /// <summary> x position 2 / Latitude 2 </summary>
        public  Single p2x;
            /// <summary> y position 2 / Longitude 2 </summary>
        public  Single p2y;
            /// <summary> z position 2 / Altitude 2 </summary>
        public  Single p2z;
    
    };


    public const byte MAVLINK_MSG_ID_SAFETY_ALLOWED_AREA = 54;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=25)]
    public struct mavlink_safety_allowed_area_t
    {
        /// <summary> Coordinate frame, as defined by MAV_FRAME enum in mavlink_types.h. Can be either global, GPS, right-handed with Z axis up or local, right handed, Z axis down. </summary>
        public  byte frame;
            /// <summary> x position 1 / Latitude 1 </summary>
        public  Single p1x;
            /// <summary> y position 1 / Longitude 1 </summary>
        public  Single p1y;
            /// <summary> z position 1 / Altitude 1 </summary>
        public  Single p1z;
            /// <summary> x position 2 / Latitude 2 </summary>
        public  Single p2x;
            /// <summary> y position 2 / Longitude 2 </summary>
        public  Single p2y;
            /// <summary> z position 2 / Altitude 2 </summary>
        public  Single p2z;
    
    };


    public const byte MAVLINK_MSG_ID_SET_ROLL_PITCH_YAW_THRUST = 55;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=18)]
    public struct mavlink_set_roll_pitch_yaw_thrust_t
    {
        /// <summary> System ID </summary>
        public  byte target_system;
            /// <summary> Component ID </summary>
        public  byte target_component;
            /// <summary> Desired roll angle in radians </summary>
        public  Single roll;
            /// <summary> Desired pitch angle in radians </summary>
        public  Single pitch;
            /// <summary> Desired yaw angle in radians </summary>
        public  Single yaw;
            /// <summary> Collective thrust, normalized to 0 .. 1 </summary>
        public  Single thrust;
    
    };


    public const byte MAVLINK_MSG_ID_SET_ROLL_PITCH_YAW_SPEED_THRUST = 56;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=18)]
    public struct mavlink_set_roll_pitch_yaw_speed_thrust_t
    {
        /// <summary> System ID </summary>
        public  byte target_system;
            /// <summary> Component ID </summary>
        public  byte target_component;
            /// <summary> Desired roll angular speed in rad/s </summary>
        public  Single roll_speed;
            /// <summary> Desired pitch angular speed in rad/s </summary>
        public  Single pitch_speed;
            /// <summary> Desired yaw angular speed in rad/s </summary>
        public  Single yaw_speed;
            /// <summary> Collective thrust, normalized to 0 .. 1 </summary>
        public  Single thrust;
    
    };


    public const byte MAVLINK_MSG_ID_ROLL_PITCH_YAW_THRUST_SETPOINT = 57;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=24)]
    public struct mavlink_roll_pitch_yaw_thrust_setpoint_t
    {
        /// <summary> Timestamp in micro seconds since unix epoch </summary>
        public  UInt64 time_us;
            /// <summary> Desired roll angle in radians </summary>
        public  Single roll;
            /// <summary> Desired pitch angle in radians </summary>
        public  Single pitch;
            /// <summary> Desired yaw angle in radians </summary>
        public  Single yaw;
            /// <summary> Collective thrust, normalized to 0 .. 1 </summary>
        public  Single thrust;
    
    };


    public const byte MAVLINK_MSG_ID_ROLL_PITCH_YAW_SPEED_THRUST_SETPOINT = 58;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=24)]
    public struct mavlink_roll_pitch_yaw_speed_thrust_setpoint_t
    {
        /// <summary> Timestamp in micro seconds since unix epoch </summary>
        public  UInt64 time_us;
            /// <summary> Desired roll angular speed in rad/s </summary>
        public  Single roll_speed;
            /// <summary> Desired pitch angular speed in rad/s </summary>
        public  Single pitch_speed;
            /// <summary> Desired yaw angular speed in rad/s </summary>
        public  Single yaw_speed;
            /// <summary> Collective thrust, normalized to 0 .. 1 </summary>
        public  Single thrust;
    
    };


    public const byte MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT = 62;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=26)]
    public struct mavlink_nav_controller_output_t
    {
        /// <summary> Current desired roll in degrees </summary>
        public  Single nav_roll;
            /// <summary> Current desired pitch in degrees </summary>
        public  Single nav_pitch;
            /// <summary> Current desired heading in degrees </summary>
        public  Int16 nav_bearing;
            /// <summary> Bearing to current waypoint/target in degrees </summary>
        public  Int16 target_bearing;
            /// <summary> Distance to active waypoint in meters </summary>
        public  UInt16 wp_dist;
            /// <summary> Current altitude error in meters </summary>
        public  Single alt_error;
            /// <summary> Current airspeed error in meters/second </summary>
        public  Single aspd_error;
            /// <summary> Current crosstrack error on x-y plane in meters </summary>
        public  Single xtrack_error;
    
    };


    public const byte MAVLINK_MSG_ID_POSITION_TARGET = 63;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=16)]
    public struct mavlink_position_target_t
    {
        /// <summary> x position </summary>
        public  Single x;
            /// <summary> y position </summary>
        public  Single y;
            /// <summary> z position </summary>
        public  Single z;
            /// <summary> yaw orientation in radians, 0 = NORTH </summary>
        public  Single yaw;
    
    };


    public const byte MAVLINK_MSG_ID_STATE_CORRECTION = 64;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=36)]
    public struct mavlink_state_correction_t
    {
        /// <summary> x position error </summary>
        public  Single xErr;
            /// <summary> y position error </summary>
        public  Single yErr;
            /// <summary> z position error </summary>
        public  Single zErr;
            /// <summary> roll error (radians) </summary>
        public  Single rollErr;
            /// <summary> pitch error (radians) </summary>
        public  Single pitchErr;
            /// <summary> yaw error (radians) </summary>
        public  Single yawErr;
            /// <summary> x velocity </summary>
        public  Single vxErr;
            /// <summary> y velocity </summary>
        public  Single vyErr;
            /// <summary> z velocity </summary>
        public  Single vzErr;
    
    };


    public const byte MAVLINK_MSG_ID_SET_ALTITUDE = 65;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=5)]
    public struct mavlink_set_altitude_t
    {
        /// <summary> The system setting the altitude </summary>
        public  byte target;
            /// <summary> The new altitude in meters </summary>
        public  UInt32 mode;
    
    };


    public const byte MAVLINK_MSG_ID_REQUEST_DATA_STREAM = 66;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=6)]
    public struct mavlink_request_data_stream_t
    {
        /// <summary> The target requested to send the message stream. </summary>
        public  byte target_system;
            /// <summary> The target requested to send the message stream. </summary>
        public  byte target_component;
            /// <summary> The ID of the requested message type </summary>
        public  byte req_stream_id;
            /// <summary> Update rate in Hertz </summary>
        public  UInt16 req_message_rate;
            /// <summary> 1 to start sending, 0 to stop sending. </summary>
        public  byte start_stop;
    
    };


    public const byte MAVLINK_MSG_ID_HIL_STATE = 67;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=56)]
    public struct mavlink_hil_state_t
    {
        /// <summary> Timestamp (microseconds since UNIX epoch or microseconds since system boot) </summary>
        public  UInt64 usec;
            /// <summary> Roll angle (rad) </summary>
        public  Single roll;
            /// <summary> Pitch angle (rad) </summary>
        public  Single pitch;
            /// <summary> Yaw angle (rad) </summary>
        public  Single yaw;
            /// <summary> Roll angular speed (rad/s) </summary>
        public  Single rollspeed;
            /// <summary> Pitch angular speed (rad/s) </summary>
        public  Single pitchspeed;
            /// <summary> Yaw angular speed (rad/s) </summary>
        public  Single yawspeed;
            /// <summary> Latitude, expressed as * 1E7 </summary>
        public  Int32 lat;
            /// <summary> Longitude, expressed as * 1E7 </summary>
        public  Int32 lon;
            /// <summary> Altitude in meters, expressed as * 1000 (millimeters) </summary>
        public  Int32 alt;
            /// <summary> Ground X Speed (Latitude), expressed as m/s * 100 </summary>
        public  Int16 vx;
            /// <summary> Ground Y Speed (Longitude), expressed as m/s * 100 </summary>
        public  Int16 vy;
            /// <summary> Ground Z Speed (Altitude), expressed as m/s * 100 </summary>
        public  Int16 vz;
            /// <summary> X acceleration (mg) </summary>
        public  Int16 xacc;
            /// <summary> Y acceleration (mg) </summary>
        public  Int16 yacc;
            /// <summary> Z acceleration (mg) </summary>
        public  Int16 zacc;
    
    };


    public const byte MAVLINK_MSG_ID_HIL_CONTROLS = 68;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=26)]
    public struct mavlink_hil_controls_t
    {
        /// <summary> Timestamp (microseconds since UNIX epoch or microseconds since system boot) </summary>
        public  UInt64 time_us;
            /// <summary> Control output -3 .. 1 </summary>
        public  Single roll_ailerons;
            /// <summary> Control output -1 .. 1 </summary>
        public  Single pitch_elevator;
            /// <summary> Control output -1 .. 1 </summary>
        public  Single yaw_rudder;
            /// <summary> Throttle 0 .. 1 </summary>
        public  Single throttle;
            /// <summary> System mode (MAV_MODE) </summary>
        public  byte mode;
            /// <summary> Navigation mode (MAV_NAV_MODE) </summary>
        public  byte nav_mode;
    
    };


    public const byte MAVLINK_MSG_ID_MANUAL_CONTROL = 69;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=21)]
    public struct mavlink_manual_control_t
    {
        /// <summary> The system to be controlled </summary>
        public  byte target;
            /// <summary> roll </summary>
        public  Single roll;
            /// <summary> pitch </summary>
        public  Single pitch;
            /// <summary> yaw </summary>
        public  Single yaw;
            /// <summary> thrust </summary>
        public  Single thrust;
            /// <summary> roll control enabled auto:0, manual:1 </summary>
        public  byte roll_manual;
            /// <summary> pitch auto:0, manual:1 </summary>
        public  byte pitch_manual;
            /// <summary> yaw auto:0, manual:1 </summary>
        public  byte yaw_manual;
            /// <summary> thrust auto:0, manual:1 </summary>
        public  byte thrust_manual;
    
    };


    public const byte MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE = 70;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=18)]
    public struct mavlink_rc_channels_override_t
    {
        /// <summary> System ID </summary>
        public  byte target_system;
            /// <summary> Component ID </summary>
        public  byte target_component;
            /// <summary> RC channel 1 value, in microseconds </summary>
        public  UInt16 chan1_raw;
            /// <summary> RC channel 2 value, in microseconds </summary>
        public  UInt16 chan2_raw;
            /// <summary> RC channel 3 value, in microseconds </summary>
        public  UInt16 chan3_raw;
            /// <summary> RC channel 4 value, in microseconds </summary>
        public  UInt16 chan4_raw;
            /// <summary> RC channel 5 value, in microseconds </summary>
        public  UInt16 chan5_raw;
            /// <summary> RC channel 6 value, in microseconds </summary>
        public  UInt16 chan6_raw;
            /// <summary> RC channel 7 value, in microseconds </summary>
        public  UInt16 chan7_raw;
            /// <summary> RC channel 8 value, in microseconds </summary>
        public  UInt16 chan8_raw;
    
    };


    public const byte MAVLINK_MSG_ID_GLOBAL_POSITION_INT = 73;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=18)]
    public struct mavlink_global_position_int_t
    {
        /// <summary> Latitude, expressed as * 1E7 </summary>
        public  Int32 lat;
            /// <summary> Longitude, expressed as * 1E7 </summary>
        public  Int32 lon;
            /// <summary> Altitude in meters, expressed as * 1000 (millimeters) </summary>
        public  Int32 alt;
            /// <summary> Ground X Speed (Latitude), expressed as m/s * 100 </summary>
        public  Int16 vx;
            /// <summary> Ground Y Speed (Longitude), expressed as m/s * 100 </summary>
        public  Int16 vy;
            /// <summary> Ground Z Speed (Altitude), expressed as m/s * 100 </summary>
        public  Int16 vz;
    
    };


    public const byte MAVLINK_MSG_ID_VFR_HUD = 74;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=20)]
    public struct mavlink_vfr_hud_t
    {
        /// <summary> Current airspeed in m/s </summary>
        public  Single airspeed;
            /// <summary> Current ground speed in m/s </summary>
        public  Single groundspeed;
            /// <summary> Current heading in degrees, in compass units (0..360, 0=north) </summary>
        public  Int16 heading;
            /// <summary> Current throttle setting in integer percent, 0 to 100 </summary>
        public  UInt16 throttle;
            /// <summary> Current altitude (MSL), in meters </summary>
        public  Single alt;
            /// <summary> Current climb rate in meters/second </summary>
        public  Single climb;
    
    };


    public const byte MAVLINK_MSG_ID_COMMAND = 75;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=20)]
    public struct mavlink_command_t
    {
        /// <summary> System which should execute the command </summary>
        public  byte target_system;
            /// <summary> Component which should execute the command, 0 for all components </summary>
        public  byte target_component;
            /// <summary> Command ID, as defined by MAV_CMD enum. </summary>
        public  byte command;
            /// <summary> 0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command) </summary>
        public  byte confirmation;
            /// <summary> Parameter 1, as defined by MAV_CMD enum. </summary>
        public  Single param1;
            /// <summary> Parameter 2, as defined by MAV_CMD enum. </summary>
        public  Single param2;
            /// <summary> Parameter 3, as defined by MAV_CMD enum. </summary>
        public  Single param3;
            /// <summary> Parameter 4, as defined by MAV_CMD enum. </summary>
        public  Single param4;
    
    };


    public const byte MAVLINK_MSG_ID_COMMAND_ACK = 76;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=8)]
    public struct mavlink_command_ack_t
    {
        /// <summary> Current airspeed in m/s </summary>
        public  Single command;
            /// <summary> 1: Action ACCEPTED and EXECUTED, 1: Action TEMPORARY REJECTED/DENIED, 2: Action PERMANENTLY DENIED, 3: Action UNKNOWN/UNSUPPORTED, 4: Requesting CONFIRMATION </summary>
        public  Single result;
    
    };


    public const byte MAVLINK_MSG_ID_OPTICAL_FLOW = 100;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=18)]
    public struct mavlink_optical_flow_t
    {
        /// <summary> Timestamp (UNIX) </summary>
        public  UInt64 time;
            /// <summary> Sensor ID </summary>
        public  byte sensor_id;
            /// <summary> Flow in pixels in x-sensor direction </summary>
        public  Int16 flow_x;
            /// <summary> Flow in pixels in y-sensor direction </summary>
        public  Int16 flow_y;
            /// <summary> Optical flow quality / confidence. 0: bad, 255: maximum quality </summary>
        public  byte quality;
            /// <summary> Ground distance in meters </summary>
        public  Single ground_distance;
    
    };


    public const byte MAVLINK_MSG_ID_OBJECT_DETECTION_EVENT = 140;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=36)]
    public struct mavlink_object_detection_event_t
    {
        /// <summary> Timestamp in milliseconds since system boot </summary>
        public  UInt32 time;
            /// <summary> Object ID </summary>
        public  UInt16 object_id;
            /// <summary> Object type: 0: image, 1: letter, 2: ground vehicle, 3: air vehicle, 4: surface vehicle, 5: sub-surface vehicle, 6: human, 7: animal </summary>
        public  byte type;
            /// <summary> Name of the object as defined by the detector </summary>
        [MarshalAs(UnmanagedType.ByValArray,SizeConst=20)]
		public string name;
            /// <summary> Detection quality / confidence. 0: bad, 255: maximum confidence </summary>
        public  byte quality;
            /// <summary> Angle of the object with respect to the body frame in NED coordinates in radians. 0: front </summary>
        public  Single bearing;
            /// <summary> Ground distance in meters </summary>
        public  Single distance;
    
    };


    public const byte MAVLINK_MSG_ID_DEBUG_VECT = 251;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=30)]
    public struct mavlink_debug_vect_t
    {
        /// <summary> Name </summary>
        [MarshalAs(UnmanagedType.ByValArray,SizeConst=10)]
		public string name;
            /// <summary> Timestamp </summary>
        public  UInt64 usec;
            /// <summary> x </summary>
        public  Single x;
            /// <summary> y </summary>
        public  Single y;
            /// <summary> z </summary>
        public  Single z;
    
    };


    public const byte MAVLINK_MSG_ID_NAMED_VALUE_FLOAT = 252;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=14)]
    public struct mavlink_named_value_float_t
    {
        /// <summary> Name of the debug variable </summary>
        [MarshalAs(UnmanagedType.ByValArray,SizeConst=10)]
		public string name;
            /// <summary> Floating point value </summary>
        public  Single value;
    
    };


    public const byte MAVLINK_MSG_ID_NAMED_VALUE_INT = 253;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=14)]
    public struct mavlink_named_value_int_t
    {
        /// <summary> Name of the debug variable </summary>
        [MarshalAs(UnmanagedType.ByValArray,SizeConst=10)]
		public string name;
            /// <summary> Signed integer value </summary>
        public  Int32 value;
    
    };


    public const byte MAVLINK_MSG_ID_STATUSTEXT = 254;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=51)]
    public struct mavlink_statustext_t
    {
        /// <summary> Severity of status, 0 = info message, 255 = critical fault </summary>
        public  byte severity;
            /// <summary> Status text message, without null termination character </summary>
        [MarshalAs(UnmanagedType.ByValArray,SizeConst=50)]
		public byte[] text;
    
    };


    public const byte MAVLINK_MSG_ID_DEBUG = 255;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=5)]
    public struct mavlink_debug_t
    {
        /// <summary> index of debug variable </summary>
        public  byte ind;
            /// <summary> DEBUG value </summary>
        public  Single value;
    
    };

     }
     #endif
}
