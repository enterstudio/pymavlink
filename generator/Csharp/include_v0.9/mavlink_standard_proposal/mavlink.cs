using System;
using System.Collections.Generic;
using System.Text;
using System.Runtime.InteropServices;

namespace ArdupilotMega
{
    #if MAVLINK10
    partial class MAVLink
    {
        public const string MAVLINK_BUILD_DATE = "Fri Apr  6 21:10:56 2012";
        public const string MAVLINK_WIRE_PROTOCOL_VERSION = "0.9";
        public const int MAVLINK_MAX_DIALECT_PAYLOAD_SIZE = 101;

        public const int MAVLINK_LITTLE_ENDIAN = 1;
        public const int MAVLINK_BIG_ENDIAN = 0;

        public const byte MAVLINK_STX = 85;

        public const byte MAVLINK_ENDIAN = MAVLINK_BIG_ENDIAN;

        public const bool MAVLINK_ALIGNED_FIELDS = (0 == 1);

        public const byte MAVLINK_CRC_EXTRA = 0;
        
        public const bool MAVLINK_NEED_BYTE_SWAP = (MAVLINK_ENDIAN == MAVLINK_LITTLE_ENDIAN);
        
        public byte[] MAVLINK_MESSAGE_LENGTHS = new byte[] {2, 18, 7, 0, 0, 0, 0, 0, 0, 2, 2, 21, 17, 15, 9, 4, 29, 4, 2, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 17, 19, 3, 13, 7, 2, 27, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 18, 20, 29, 101, 32, 25, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 51, 5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

        public byte[] MAVLINK_MESSAGE_CRCS = new byte[] {115, 238, 123, 0, 0, 0, 0, 0, 0, 186, 10, 121, 166, 187, 69, 79, 49, 13, 229, 90, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 15, 243, 203, 12, 78, 205, 52, 55, 249, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 65, 225, 101, 147, 66, 3, 87, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 106, 7, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

        public Type[] MAVLINK_MESSAGE_INFO = new Type[] {typeof( mavlink_heartbeat_t ), typeof( mavlink_ping_t ), typeof( mavlink_sys_status_t ), null, null, null, null, null, null, typeof( mavlink_set_mode_t ), typeof( mavlink_set_nav_mode_t ), typeof( mavlink_param_set_t ), typeof( mavlink_param_write_storage_t ), typeof( mavlink_pid_set_t ), typeof( mavlink_rc_channels_trim_set_t ), typeof( mavlink_rc_channels_mapping_set_t ), typeof( mavlink_waypoint_set_t ), typeof( mavlink_waypoint_set_current_t ), typeof( mavlink_waypoint_clear_all_t ), typeof( mavlink_action_t ), null, null, null, null, null, null, null, null, null, null, typeof( mavlink_acknowledge_t ), typeof( mavlink_param_request_read_t ), typeof( mavlink_param_value_t ), typeof( mavlink_param_request_list_t ), typeof( mavlink_pid_t ), typeof( mavlink_rc_channels_trim_t ), typeof( mavlink_rc_channels_mapping_t ), typeof( mavlink_waypoint_t ), typeof( mavlink_waypoint_status_t ), null, null, null, null, null, null, null, null, null, null, null, typeof( mavlink_raw_imu_t ), typeof( mavlink_raw_pressure_t ), typeof( mavlink_gps_raw_t ), typeof( mavlink_gps_sat_status_t ), typeof( mavlink_attitude_t ), typeof( mavlink_position_t ), typeof( mavlink_rc_channels_raw_t ), null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, typeof( mavlink_statustext_t ), typeof( mavlink_debug_t ), null, null, null, null, null, null, null, null, null, null, null, null, null, null};

        public const byte MAVLINK_VERSION = 2;
    
        
    

    public const byte MAVLINK_MSG_ID_HEARTBEAT = 0;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=2)]
    public struct mavlink_heartbeat_t
    {
        /// <summary> Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM) </summary>
        public  byte type;
            /// <summary> Type of the Autopilot: 0: Generic, 1: PIXHAWK, 2: SLUGS, 3: Ardupilot (up to 15 types), defined in MAV_AUTOPILOT_TYPE ENUM </summary>
        public  byte autopilot;
    
    };


    public const byte MAVLINK_MSG_ID_PING = 1;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=18)]
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
            /// <summary> The onboard software version </summary>
        public  UInt32 version;
    
    };


    public const byte MAVLINK_MSG_ID_SYS_STATUS = 2;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=7)]
    public struct mavlink_sys_status_t
    {
        /// <summary> System mode, see MAV_MODE ENUM in mavlink/include/mavlink_types.h </summary>
        public  byte mode;
            /// <summary> Navigation mode, see MAV_NAV_MODE ENUM </summary>
        public  byte nav_mode;
            /// <summary> System status flag, see MAV_STATUS ENUM </summary>
        public  byte status;
            /// <summary> Failure code description, see MAV_FAILURE ENUM </summary>
        public  byte failure;
            /// <summary> Motor block status flag </summary>
        public  byte motor_block;
            /// <summary> Dropped packets </summary>
        public  UInt16 packet_drop;
    
    };


    public const byte MAVLINK_MSG_ID_SET_MODE = 9;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=2)]
    public struct mavlink_set_mode_t
    {
        /// <summary> The system setting the mode </summary>
        public  byte target;
            /// <summary> The new mode </summary>
        public  byte mode;
    
    };


    public const byte MAVLINK_MSG_ID_SET_NAV_MODE = 10;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=2)]
    public struct mavlink_set_nav_mode_t
    {
        /// <summary> The system setting the mode </summary>
        public  byte target;
            /// <summary> The new navigation mode </summary>
        public  byte nav_mode;
    
    };


    public const byte MAVLINK_MSG_ID_PARAM_SET = 11;
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


    public const byte MAVLINK_MSG_ID_PARAM_WRITE_STORAGE = 12;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=17)]
    public struct mavlink_param_write_storage_t
    {
        /// <summary> System ID </summary>
        public  byte target_system;
            /// <summary> Component ID </summary>
        public  byte target_component;
            /// <summary> Onboard parameter id </summary>
        [MarshalAs(UnmanagedType.ByValArray,SizeConst=15)]
		public byte[] param_id;
    
    };


    public const byte MAVLINK_MSG_ID_PID_SET = 13;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=15)]
    public struct mavlink_pid_set_t
    {
        /// <summary> System ID </summary>
        public  byte target_system;
            /// <summary> Component ID </summary>
        public  byte target_component;
            /// <summary> PID ID </summary>
        public  byte pid_id;
            /// <summary> P </summary>
        public  Single k_p;
            /// <summary> I </summary>
        public  Single k_i;
            /// <summary> D </summary>
        public  Single k_d;
    
    };


    public const byte MAVLINK_MSG_ID_RC_CHANNELS_TRIM_SET = 14;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=9)]
    public struct mavlink_rc_channels_trim_set_t
    {
        /// <summary> System ID </summary>
        public  byte target_system;
            /// <summary> Component ID </summary>
        public  byte target_component;
            /// <summary> RC channel id </summary>
        public  byte chan_id;
            /// <summary> RC channel 1 min value, in microseconds </summary>
        public  UInt16 chan_min;
            /// <summary> RC channel 1 zero value, in microseconds </summary>
        public  UInt16 chan_zero;
            /// <summary> RC channel 1 max value, in microseconds </summary>
        public  UInt16 chan_max;
    
    };


    public const byte MAVLINK_MSG_ID_RC_CHANNELS_MAPPING_SET = 15;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=4)]
    public struct mavlink_rc_channels_mapping_set_t
    {
        /// <summary> System ID </summary>
        public  byte target_system;
            /// <summary> Component ID </summary>
        public  byte target_component;
            /// <summary> RC channel id </summary>
        public  byte chan_id;
            /// <summary> RC channel function, as defined in ENUM MAVLINK_RC_CHAN_MAPPING in mavlink/include/mavlink_types.h </summary>
        public  byte chan_function;
    
    };


    public const byte MAVLINK_MSG_ID_WAYPOINT_SET = 16;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=29)]
    public struct mavlink_waypoint_set_t
    {
        /// <summary> System ID </summary>
        public  byte target_system;
            /// <summary> Component ID </summary>
        public  byte target_component;
            /// <summary> Waypoint ID </summary>
        public  UInt16 wp_id;
            /// <summary> 0: global (GPS), 1: local, 2: global orbit, 3: local orbit </summary>
        public  byte type;
            /// <summary> Orbit to circle around the waypoint, in meters </summary>
        public  Single param1;
            /// <summary> Time that the MAV should stay inside the orbit before advancing, in milliseconds </summary>
        public  Single param2;
            /// <summary> 0: No orbit, 1: Right Orbit, 2: Left Orbit </summary>
        public  byte orbit;
            /// <summary> false:0, true:1 </summary>
        public  byte current;
            /// <summary> local: x position, global: longitude </summary>
        public  Single x;
            /// <summary> y position: global: latitude </summary>
        public  Single y;
            /// <summary> z position: global: altitude </summary>
        public  Single z;
            /// <summary> autocontinue to next wp </summary>
        public  UInt16 autocontinue;
    
    };


    public const byte MAVLINK_MSG_ID_WAYPOINT_SET_CURRENT = 17;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=4)]
    public struct mavlink_waypoint_set_current_t
    {
        /// <summary> System ID </summary>
        public  byte target_system;
            /// <summary> Component ID </summary>
        public  byte target_component;
            /// <summary> Waypoint ID </summary>
        public  UInt16 wp_id;
    
    };


    public const byte MAVLINK_MSG_ID_WAYPOINT_CLEAR_ALL = 18;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=2)]
    public struct mavlink_waypoint_clear_all_t
    {
        /// <summary> System ID </summary>
        public  byte target_system;
            /// <summary> Component ID </summary>
        public  byte target_component;
    
    };


    public const byte MAVLINK_MSG_ID_ACTION = 19;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=3)]
    public struct mavlink_action_t
    {
        /// <summary> The system executing the action </summary>
        public  byte target_system;
            /// <summary> Component ID </summary>
        public  byte target_component;
            /// <summary> The action id </summary>
        public  byte action;
    
    };


    public const byte MAVLINK_MSG_ID_ACKNOWLEDGE = 30;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=3)]
    public struct mavlink_acknowledge_t
    {
        /// <summary> The system executing the action </summary>
        public  byte target_system;
            /// <summary> Component ID </summary>
        public  byte target_component;
            /// <summary> The id of the action being successfully executed and acknowledged </summary>
        public  byte akn_id;
    
    };


    public const byte MAVLINK_MSG_ID_PARAM_REQUEST_READ = 31;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=17)]
    public struct mavlink_param_request_read_t
    {
        /// <summary> System ID </summary>
        public  byte target_system;
            /// <summary> Component ID </summary>
        public  byte target_component;
            /// <summary> Onboard parameter id </summary>
        [MarshalAs(UnmanagedType.ByValArray,SizeConst=15)]
		public byte[] param_id;
    
    };


    public const byte MAVLINK_MSG_ID_PARAM_VALUE = 32;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=19)]
    public struct mavlink_param_value_t
    {
        /// <summary> Onboard parameter id </summary>
        [MarshalAs(UnmanagedType.ByValArray,SizeConst=15)]
		public byte[] param_id;
            /// <summary> Onboard parameter value </summary>
        public  Single param_value;
    
    };


    public const byte MAVLINK_MSG_ID_PARAM_REQUEST_LIST = 33;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=3)]
    public struct mavlink_param_request_list_t
    {
        /// <summary> System ID </summary>
        public  byte target_system;
            /// <summary> Component ID </summary>
        public  byte target_component;
            /// <summary> 0: All parameters, else report a subset of parameters as defined in MAVLIN_SUBSET_PARAM enum </summary>
        public  byte param_subset_id;
    
    };


    public const byte MAVLINK_MSG_ID_PID = 34;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=13)]
    public struct mavlink_pid_t
    {
        /// <summary> PID ID </summary>
        public  byte pid_id;
            /// <summary> P </summary>
        public  Single k_p;
            /// <summary> I </summary>
        public  Single k_i;
            /// <summary> D </summary>
        public  Single k_d;
    
    };


    public const byte MAVLINK_MSG_ID_RC_CHANNELS_TRIM = 35;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=7)]
    public struct mavlink_rc_channels_trim_t
    {
        /// <summary> RC channel id </summary>
        public  byte chan_id;
            /// <summary> RC channel 1 min value, in microseconds </summary>
        public  UInt16 chan_min;
            /// <summary> RC channel 1 zero value, in microseconds </summary>
        public  UInt16 chan_zero;
            /// <summary> RC channel 1 max value, in microseconds </summary>
        public  UInt16 chan_max;
    
    };


    public const byte MAVLINK_MSG_ID_RC_CHANNELS_MAPPING = 36;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=2)]
    public struct mavlink_rc_channels_mapping_t
    {
        /// <summary> RC channel id </summary>
        public  byte chan_id;
            /// <summary> RC channel function, as defined in ENUM MAVLINK_RC_CHAN_MAPPING in mavlink/include/mavlink_types.h </summary>
        public  byte chan_function;
    
    };


    public const byte MAVLINK_MSG_ID_WAYPOINT = 37;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=27)]
    public struct mavlink_waypoint_t
    {
        /// <summary> Waypoint ID </summary>
        public  UInt16 wp_id;
            /// <summary> 0: global (GPS), 1: local, 2: global orbit, 3: local orbit </summary>
        public  byte type;
            /// <summary> Orbit to circle around the waypoint, in meters </summary>
        public  Single param1;
            /// <summary> Time that the MAV should stay inside the orbit before advancing, in milliseconds </summary>
        public  Single param2;
            /// <summary> 0: No orbit, 1: Right Orbit, 2: Left Orbit </summary>
        public  byte orbit;
            /// <summary> false:0, true:1 </summary>
        public  byte current;
            /// <summary> local: x position, global: longitude </summary>
        public  Single x;
            /// <summary> y position: global: latitude </summary>
        public  Single y;
            /// <summary> z position: global: altitude </summary>
        public  Single z;
            /// <summary> autocontinue to next wp </summary>
        public  UInt16 autocontinue;
    
    };


    public const byte MAVLINK_MSG_ID_WAYPOINT_STATUS = 38;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=3)]
    public struct mavlink_waypoint_status_t
    {
        /// <summary> Waypoint ID </summary>
        public  UInt16 wp_id;
            /// <summary> Waypoint status: 0: Ok, 1: Reached, 2: Orbit time expired, 254: Error </summary>
        public  byte wp_status;
    
    };


    public const byte MAVLINK_MSG_ID_RAW_IMU = 50;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=18)]
    public struct mavlink_raw_imu_t
    {
        /// <summary> X acceleration (mg raw) </summary>
        public  Int16 xacc;
            /// <summary> Y acceleration (mg raw) </summary>
        public  Int16 yacc;
            /// <summary> Z acceleration (mg raw) </summary>
        public  Int16 zacc;
            /// <summary> Angular speed around X axis (adc units) </summary>
        public  Int16 xgyro;
            /// <summary> Angular speed around Y axis (adc units) </summary>
        public  Int16 ygyro;
            /// <summary> Angular speed around Z axis (adc units) </summary>
        public  Int16 zgyro;
            /// <summary> X Magnetic field (milli tesla) </summary>
        public  Int16 xmag;
            /// <summary> Y Magnetic field (milli tesla) </summary>
        public  Int16 ymag;
            /// <summary> Z Magnetic field (milli tesla) </summary>
        public  Int16 zmag;
    
    };


    public const byte MAVLINK_MSG_ID_RAW_PRESSURE = 51;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=20)]
    public struct mavlink_raw_pressure_t
    {
        /// <summary> Timestamp (microseconds since UNIX epoch) </summary>
        public  UInt64 usec;
            /// <summary> Absolute pressure (hectopascal) </summary>
        public  Int32 press_abs;
            /// <summary> Differential pressure 1 (hectopascal) </summary>
        public  Int32 press_diff1;
            /// <summary> Differential pressure 2 (hectopascal) </summary>
        public  Int32 press_diff2;
    
    };


    public const byte MAVLINK_MSG_ID_GPS_RAW = 52;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=29)]
    public struct mavlink_gps_raw_t
    {
        /// <summary> 0-1: no fix, 2: 2D fix, 3: 3D fix </summary>
        public  byte fix_type;
            /// <summary> X Position </summary>
        public  Single lat;
            /// <summary> Y Position </summary>
        public  Single lon;
            /// <summary> Z Position in meters </summary>
        public  Single alt;
            /// <summary> Uncertainty in meters of latitude </summary>
        public  Single eph;
            /// <summary> Uncertainty in meters of longitude </summary>
        public  Single epv;
            /// <summary> Overall speed </summary>
        public  Single v;
            /// <summary> Heading, in FIXME </summary>
        public  Single hdg;
    
    };


    public const byte MAVLINK_MSG_ID_GPS_SAT_STATUS = 53;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=101)]
    public struct mavlink_gps_sat_status_t
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


    public const byte MAVLINK_MSG_ID_ATTITUDE = 54;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=32)]
    public struct mavlink_attitude_t
    {
        /// <summary> Timestamp (microseconds) </summary>
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


    public const byte MAVLINK_MSG_ID_POSITION = 55;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=25)]
    public struct mavlink_position_t
    {
        /// <summary> Position type: 0: Local, 1: Global </summary>
        public  byte type;
            /// <summary> X (long) Position </summary>
        public  Single x;
            /// <summary> Y (lat) Position </summary>
        public  Single y;
            /// <summary> Z (alt) Position </summary>
        public  Single z;
            /// <summary> Vx </summary>
        public  Single vx;
            /// <summary> Vy </summary>
        public  Single vy;
            /// <summary> Vz </summary>
        public  Single vz;
    
    };


    public const byte MAVLINK_MSG_ID_RC_CHANNELS_RAW = 56;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=10)]
    public struct mavlink_rc_channels_raw_t
    {
        /// <summary> RC channel 1 value, in microseconds </summary>
        public  UInt16 chan1;
            /// <summary> RC channel 2 value, in microseconds </summary>
        public  UInt16 chan2;
            /// <summary> RC channel 3 value, in microseconds </summary>
        public  UInt16 chan3;
            /// <summary> RC channel 3 value, in microseconds </summary>
        public  UInt16 chan4;
            /// <summary> RC channel 3 value, in microseconds </summary>
        public  UInt16 chan5;
    
    };


    public const byte MAVLINK_MSG_ID_STATUSTEXT = 240;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=51)]
    public struct mavlink_statustext_t
    {
        /// <summary> Severity of status, 0 = info message, 255 = critical fault </summary>
        public  byte severity;
            /// <summary> Status text message, without null termination character </summary>
        [MarshalAs(UnmanagedType.ByValArray,SizeConst=50)]
		public byte[] text;
    
    };


    public const byte MAVLINK_MSG_ID_DEBUG = 241;
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
