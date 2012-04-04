using System;
using System.Collections.Generic;
using System.Text;
using System.Runtime.InteropServices;

namespace ArdupilotMega
{
    #if MAVLINK10
    partial class MAVLink
    {
        public const string MAVLINK_BUILD_DATE = "Wed Apr  4 18:13:07 2012";
        public const string MAVLINK_WIRE_PROTOCOL_VERSION = "0.9";
        public const int MAVLINK_MAX_DIALECT_PAYLOAD_SIZE = 255;

        public const int MAVLINK_LITTLE_ENDIAN = 1;
        public const int MAVLINK_BIG_ENDIAN = 0;

        public const byte MAVLINK_STX = 85;

        public const byte MAVLINK_ENDIAN = MAVLINK_BIG_ENDIAN;

        public const bool MAVLINK_ALIGNED_FIELDS = (0 == 1);

        public const byte MAVLINK_CRC_EXTRA = 0;
        
        public const bool MAVLINK_NEED_BYTE_SWAP = (MAVLINK_ENDIAN == MAVLINK_LITTLE_ENDIAN);
        
        public byte[] MAVLINK_MESSAGE_LENGTHS = new byte[] {3, 4, 8, 14, 8, 28, 3, 32, 0, 2, 3, 2, 2, 0, 0, 0, 0, 0, 0, 0, 19, 2, 23, 21, 0, 37, 26, 101, 26, 16, 32, 32, 37, 32, 11, 17, 17, 16, 18, 36, 4, 4, 2, 2, 4, 2, 2, 3, 14, 12, 18, 16, 8, 27, 25, 18, 18, 24, 24, 0, 0, 0, 26, 16, 36, 5, 6, 56, 26, 21, 18, 0, 0, 18, 20, 20, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 18, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 36, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 11, 52, 1, 92, 0, 32, 32, 20, 20, 18, 0, 0, 0, 0, 0, 0, 0, 0, 0, 18, 26, 16, 0, 0, 0, 0, 0, 0, 0, 4, 255, 12, 6, 0, 0, 0, 0, 0, 0, 106, 43, 55, 8, 255, 53, 0, 0, 0, 0, 21, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 30, 14, 14, 51, 5};

        public byte[] MAVLINK_MESSAGE_CRCS = new byte[] {72, 39, 190, 92, 191, 217, 104, 119, 0, 219, 60, 186, 10, 0, 0, 0, 0, 0, 0, 0, 89, 159, 162, 121, 0, 149, 222, 110, 179, 136, 66, 126, 185, 147, 112, 252, 162, 215, 229, 128, 9, 106, 101, 213, 4, 229, 21, 214, 215, 14, 206, 50, 157, 126, 108, 213, 95, 5, 127, 0, 0, 0, 57, 126, 130, 119, 193, 191, 236, 158, 143, 0, 0, 104, 123, 131, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 174, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 155, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 204, 86, 95, 49, 0, 158, 56, 208, 218, 115, 0, 0, 0, 0, 0, 0, 0, 0, 0, 220, 136, 140, 0, 0, 0, 0, 0, 0, 0, 153, 110, 92, 188, 0, 0, 0, 0, 0, 0, 106, 154, 83, 98, 223, 254, 0, 0, 0, 0, 84, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 178, 224, 60, 106, 7};

        public Type[] MAVLINK_MESSAGE_INFO = new Type[] {typeof( mavlink_heartbeat_t ), typeof( mavlink_boot_t ), typeof( mavlink_system_time_t ), typeof( mavlink_ping_t ), typeof( mavlink_system_time_utc_t ), typeof( mavlink_change_operator_control_t ), typeof( mavlink_change_operator_control_ack_t ), typeof( mavlink_auth_key_t ), null, typeof( mavlink_action_ack_t ), typeof( mavlink_action_t ), typeof( mavlink_set_mode_t ), typeof( mavlink_set_nav_mode_t ), null, null, null, null, null, null, null, typeof( mavlink_param_request_read_t ), typeof( mavlink_param_request_list_t ), typeof( mavlink_param_value_t ), typeof( mavlink_param_set_t ), null, typeof( mavlink_gps_raw_int_t ), typeof( mavlink_scaled_imu_t ), typeof( mavlink_gps_status_t ), typeof( mavlink_raw_imu_t ), typeof( mavlink_raw_pressure_t ), typeof( mavlink_attitude_t ), typeof( mavlink_local_position_t ), typeof( mavlink_gps_raw_t ), typeof( mavlink_global_position_t ), typeof( mavlink_sys_status_t ), typeof( mavlink_rc_channels_raw_t ), typeof( mavlink_rc_channels_scaled_t ), typeof( mavlink_servo_output_raw_t ), typeof( mavlink_scaled_pressure_t ), typeof( mavlink_waypoint_t ), typeof( mavlink_waypoint_request_t ), typeof( mavlink_waypoint_set_current_t ), typeof( mavlink_waypoint_current_t ), typeof( mavlink_waypoint_request_list_t ), typeof( mavlink_waypoint_count_t ), typeof( mavlink_waypoint_clear_all_t ), typeof( mavlink_waypoint_reached_t ), typeof( mavlink_waypoint_ack_t ), typeof( mavlink_gps_set_global_origin_t ), typeof( mavlink_gps_local_origin_set_t ), typeof( mavlink_local_position_setpoint_set_t ), typeof( mavlink_local_position_setpoint_t ), typeof( mavlink_control_status_t ), typeof( mavlink_safety_set_allowed_area_t ), typeof( mavlink_safety_allowed_area_t ), typeof( mavlink_set_roll_pitch_yaw_thrust_t ), typeof( mavlink_set_roll_pitch_yaw_speed_thrust_t ), typeof( mavlink_roll_pitch_yaw_thrust_setpoint_t ), typeof( mavlink_roll_pitch_yaw_speed_thrust_setpoint_t ), null, null, null, typeof( mavlink_nav_controller_output_t ), typeof( mavlink_position_target_t ), typeof( mavlink_state_correction_t ), typeof( mavlink_set_altitude_t ), typeof( mavlink_request_data_stream_t ), typeof( mavlink_hil_state_t ), typeof( mavlink_hil_controls_t ), typeof( mavlink_manual_control_t ), typeof( mavlink_rc_channels_override_t ), null, null, typeof( mavlink_global_position_int_t ), typeof( mavlink_vfr_hud_t ), typeof( mavlink_command_t ), typeof( mavlink_command_ack_t ), null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, typeof( mavlink_optical_flow_t ), null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, typeof( mavlink_object_detection_event_t ), null, null, null, null, null, null, null, null, null, null, typeof( mavlink_set_cam_shutter_t ), typeof( mavlink_image_triggered_t ), typeof( mavlink_image_trigger_control_t ), typeof( mavlink_image_available_t ), null, typeof( mavlink_vision_position_estimate_t ), typeof( mavlink_vicon_position_estimate_t ), typeof( mavlink_vision_speed_estimate_t ), typeof( mavlink_position_control_setpoint_set_t ), typeof( mavlink_position_control_offset_set_t ), null, null, null, null, null, null, null, null, null, typeof( mavlink_position_control_setpoint_t ), typeof( mavlink_marker_t ), typeof( mavlink_raw_aux_t ), null, null, null, null, null, null, null, typeof( mavlink_watchdog_heartbeat_t ), typeof( mavlink_watchdog_process_info_t ), typeof( mavlink_watchdog_process_status_t ), typeof( mavlink_watchdog_command_t ), null, null, null, null, null, null, typeof( mavlink_pattern_detected_t ), typeof( mavlink_point_of_interest_t ), typeof( mavlink_point_of_interest_connection_t ), typeof( mavlink_data_transmission_handshake_t ), typeof( mavlink_encapsulated_data_t ), typeof( mavlink_brief_feature_t ), null, null, null, null, typeof( mavlink_attitude_control_t ), null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, typeof( mavlink_debug_vect_t ), typeof( mavlink_named_value_float_t ), typeof( mavlink_named_value_int_t ), typeof( mavlink_statustext_t ), typeof( mavlink_debug_t )};

        public const byte MAVLINK_VERSION = 2;
    
        
        /** @brief Content Types for data transmission handshake */
        public enum DATA_TYPES
        {
    	///<summary>  | </summary>
            DATA_TYPE_JPEG_IMAGE=1, 
        	///<summary>  | </summary>
            DATA_TYPE_RAW_IMAGE=2, 
        	///<summary>  | </summary>
            DATA_TYPE_KINECT=3, 
        	///<summary>  | </summary>
            ENUM_END=4, 
        
        };
        
    

    public const byte MAVLINK_MSG_ID_SET_CAM_SHUTTER = 151;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=11)]
    public struct mavlink_set_cam_shutter_t
    {
        /// <summary> Camera id </summary>
        public  byte cam_no;
            /// <summary> Camera mode: 0 = auto, 1 = manual </summary>
        public  byte cam_mode;
            /// <summary> Trigger pin, 0-3 for PtGrey FireFly </summary>
        public  byte trigger_pin;
            /// <summary> Shutter interval, in microseconds </summary>
        public  UInt16 interval;
            /// <summary> Exposure time, in microseconds </summary>
        public  UInt16 exposure;
            /// <summary> Camera gain </summary>
        public  Single gain;
    
    };


    public const byte MAVLINK_MSG_ID_IMAGE_TRIGGERED = 152;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=52)]
    public struct mavlink_image_triggered_t
    {
        /// <summary> Timestamp </summary>
        public  UInt64 timestamp;
            /// <summary> IMU seq </summary>
        public  UInt32 seq;
            /// <summary> Roll angle in rad </summary>
        public  Single roll;
            /// <summary> Pitch angle in rad </summary>
        public  Single pitch;
            /// <summary> Yaw angle in rad </summary>
        public  Single yaw;
            /// <summary> Local frame Z coordinate (height over ground) </summary>
        public  Single local_z;
            /// <summary> GPS X coordinate </summary>
        public  Single lat;
            /// <summary> GPS Y coordinate </summary>
        public  Single lon;
            /// <summary> Global frame altitude </summary>
        public  Single alt;
            /// <summary> Ground truth X </summary>
        public  Single ground_x;
            /// <summary> Ground truth Y </summary>
        public  Single ground_y;
            /// <summary> Ground truth Z </summary>
        public  Single ground_z;
    
    };


    public const byte MAVLINK_MSG_ID_IMAGE_TRIGGER_CONTROL = 153;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=1)]
    public struct mavlink_image_trigger_control_t
    {
        /// <summary> 0 to disable, 1 to enable </summary>
        public  byte enable;
    
    };


    public const byte MAVLINK_MSG_ID_IMAGE_AVAILABLE = 154;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=92)]
    public struct mavlink_image_available_t
    {
        /// <summary> Camera id </summary>
        public  UInt64 cam_id;
            /// <summary> Camera # (starts with 0) </summary>
        public  byte cam_no;
            /// <summary> Timestamp </summary>
        public  UInt64 timestamp;
            /// <summary> Until which timestamp this buffer will stay valid </summary>
        public  UInt64 valid_until;
            /// <summary> The image sequence number </summary>
        public  UInt32 img_seq;
            /// <summary> Position of the image in the buffer, starts with 0 </summary>
        public  UInt32 img_buf_index;
            /// <summary> Image width </summary>
        public  UInt16 width;
            /// <summary> Image height </summary>
        public  UInt16 height;
            /// <summary> Image depth </summary>
        public  UInt16 depth;
            /// <summary> Image channels </summary>
        public  byte channels;
            /// <summary> Shared memory area key </summary>
        public  UInt32 key;
            /// <summary> Exposure time, in microseconds </summary>
        public  UInt32 exposure;
            /// <summary> Camera gain </summary>
        public  Single gain;
            /// <summary> Roll angle in rad </summary>
        public  Single roll;
            /// <summary> Pitch angle in rad </summary>
        public  Single pitch;
            /// <summary> Yaw angle in rad </summary>
        public  Single yaw;
            /// <summary> Local frame Z coordinate (height over ground) </summary>
        public  Single local_z;
            /// <summary> GPS X coordinate </summary>
        public  Single lat;
            /// <summary> GPS Y coordinate </summary>
        public  Single lon;
            /// <summary> Global frame altitude </summary>
        public  Single alt;
            /// <summary> Ground truth X </summary>
        public  Single ground_x;
            /// <summary> Ground truth Y </summary>
        public  Single ground_y;
            /// <summary> Ground truth Z </summary>
        public  Single ground_z;
    
    };


    public const byte MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE = 156;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=32)]
    public struct mavlink_vision_position_estimate_t
    {
        /// <summary> Timestamp (milliseconds) </summary>
        public  UInt64 usec;
            /// <summary> Global X position </summary>
        public  Single x;
            /// <summary> Global Y position </summary>
        public  Single y;
            /// <summary> Global Z position </summary>
        public  Single z;
            /// <summary> Roll angle in rad </summary>
        public  Single roll;
            /// <summary> Pitch angle in rad </summary>
        public  Single pitch;
            /// <summary> Yaw angle in rad </summary>
        public  Single yaw;
    
    };


    public const byte MAVLINK_MSG_ID_VICON_POSITION_ESTIMATE = 157;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=32)]
    public struct mavlink_vicon_position_estimate_t
    {
        /// <summary> Timestamp (milliseconds) </summary>
        public  UInt64 usec;
            /// <summary> Global X position </summary>
        public  Single x;
            /// <summary> Global Y position </summary>
        public  Single y;
            /// <summary> Global Z position </summary>
        public  Single z;
            /// <summary> Roll angle in rad </summary>
        public  Single roll;
            /// <summary> Pitch angle in rad </summary>
        public  Single pitch;
            /// <summary> Yaw angle in rad </summary>
        public  Single yaw;
    
    };


    public const byte MAVLINK_MSG_ID_VISION_SPEED_ESTIMATE = 158;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=20)]
    public struct mavlink_vision_speed_estimate_t
    {
        /// <summary> Timestamp (milliseconds) </summary>
        public  UInt64 usec;
            /// <summary> Global X speed </summary>
        public  Single x;
            /// <summary> Global Y speed </summary>
        public  Single y;
            /// <summary> Global Z speed </summary>
        public  Single z;
    
    };


    public const byte MAVLINK_MSG_ID_POSITION_CONTROL_SETPOINT_SET = 159;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=20)]
    public struct mavlink_position_control_setpoint_set_t
    {
        /// <summary> System ID </summary>
        public  byte target_system;
            /// <summary> Component ID </summary>
        public  byte target_component;
            /// <summary> ID of waypoint, 0 for plain position </summary>
        public  UInt16 id;
            /// <summary> x position </summary>
        public  Single x;
            /// <summary> y position </summary>
        public  Single y;
            /// <summary> z position </summary>
        public  Single z;
            /// <summary> yaw orientation in radians, 0 = NORTH </summary>
        public  Single yaw;
    
    };


    public const byte MAVLINK_MSG_ID_POSITION_CONTROL_OFFSET_SET = 160;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=18)]
    public struct mavlink_position_control_offset_set_t
    {
        /// <summary> System ID </summary>
        public  byte target_system;
            /// <summary> Component ID </summary>
        public  byte target_component;
            /// <summary> x position offset </summary>
        public  Single x;
            /// <summary> y position offset </summary>
        public  Single y;
            /// <summary> z position offset </summary>
        public  Single z;
            /// <summary> yaw orientation offset in radians, 0 = NORTH </summary>
        public  Single yaw;
    
    };


    public const byte MAVLINK_MSG_ID_POSITION_CONTROL_SETPOINT = 170;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=18)]
    public struct mavlink_position_control_setpoint_t
    {
        /// <summary> ID of waypoint, 0 for plain position </summary>
        public  UInt16 id;
            /// <summary> x position </summary>
        public  Single x;
            /// <summary> y position </summary>
        public  Single y;
            /// <summary> z position </summary>
        public  Single z;
            /// <summary> yaw orientation in radians, 0 = NORTH </summary>
        public  Single yaw;
    
    };


    public const byte MAVLINK_MSG_ID_MARKER = 171;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=26)]
    public struct mavlink_marker_t
    {
        /// <summary> ID </summary>
        public  UInt16 id;
            /// <summary> x position </summary>
        public  Single x;
            /// <summary> y position </summary>
        public  Single y;
            /// <summary> z position </summary>
        public  Single z;
            /// <summary> roll orientation </summary>
        public  Single roll;
            /// <summary> pitch orientation </summary>
        public  Single pitch;
            /// <summary> yaw orientation </summary>
        public  Single yaw;
    
    };


    public const byte MAVLINK_MSG_ID_RAW_AUX = 172;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=16)]
    public struct mavlink_raw_aux_t
    {
        /// <summary> ADC1 (J405 ADC3, LPC2148 AD0.6) </summary>
        public  UInt16 adc1;
            /// <summary> ADC2 (J405 ADC5, LPC2148 AD0.2) </summary>
        public  UInt16 adc2;
            /// <summary> ADC3 (J405 ADC6, LPC2148 AD0.1) </summary>
        public  UInt16 adc3;
            /// <summary> ADC4 (J405 ADC7, LPC2148 AD1.3) </summary>
        public  UInt16 adc4;
            /// <summary> Battery voltage </summary>
        public  UInt16 vbat;
            /// <summary> Temperature (degrees celcius) </summary>
        public  Int16 temp;
            /// <summary> Barometric pressure (hecto Pascal) </summary>
        public  Int32 baro;
    
    };


    public const byte MAVLINK_MSG_ID_WATCHDOG_HEARTBEAT = 180;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=4)]
    public struct mavlink_watchdog_heartbeat_t
    {
        /// <summary> Watchdog ID </summary>
        public  UInt16 watchdog_id;
            /// <summary> Number of processes </summary>
        public  UInt16 process_count;
    
    };


    public const byte MAVLINK_MSG_ID_WATCHDOG_PROCESS_INFO = 181;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=255)]
    public struct mavlink_watchdog_process_info_t
    {
        /// <summary> Watchdog ID </summary>
        public  UInt16 watchdog_id;
            /// <summary> Process ID </summary>
        public  UInt16 process_id;
            /// <summary> Process name </summary>
        [MarshalAs(UnmanagedType.ByValArray,SizeConst=100)]
		public string name;
            /// <summary> Process arguments </summary>
        [MarshalAs(UnmanagedType.ByValArray,SizeConst=147)]
		public string arguments;
            /// <summary> Timeout (seconds) </summary>
        public  Int32 timeout;
    
    };


    public const byte MAVLINK_MSG_ID_WATCHDOG_PROCESS_STATUS = 182;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=12)]
    public struct mavlink_watchdog_process_status_t
    {
        /// <summary> Watchdog ID </summary>
        public  UInt16 watchdog_id;
            /// <summary> Process ID </summary>
        public  UInt16 process_id;
            /// <summary> Is running / finished / suspended / crashed </summary>
        public  byte state;
            /// <summary> Is muted </summary>
        public  byte muted;
            /// <summary> PID </summary>
        public  Int32 pid;
            /// <summary> Number of crashes </summary>
        public  UInt16 crashes;
    
    };


    public const byte MAVLINK_MSG_ID_WATCHDOG_COMMAND = 183;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=6)]
    public struct mavlink_watchdog_command_t
    {
        /// <summary> Target system ID </summary>
        public  byte target_system_id;
            /// <summary> Watchdog ID </summary>
        public  UInt16 watchdog_id;
            /// <summary> Process ID </summary>
        public  UInt16 process_id;
            /// <summary> Command ID </summary>
        public  byte command_id;
    
    };


    public const byte MAVLINK_MSG_ID_PATTERN_DETECTED = 190;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=106)]
    public struct mavlink_pattern_detected_t
    {
        /// <summary> 0: Pattern, 1: Letter </summary>
        public  byte type;
            /// <summary> Confidence of detection </summary>
        public  Single confidence;
            /// <summary> Pattern file name </summary>
        [MarshalAs(UnmanagedType.ByValArray,SizeConst=100)]
		public string file;
            /// <summary> Accepted as true detection, 0 no, 1 yes </summary>
        public  byte detected;
    
    };


    public const byte MAVLINK_MSG_ID_POINT_OF_INTEREST = 191;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=43)]
    public struct mavlink_point_of_interest_t
    {
        /// <summary> 0: Notice, 1: Warning, 2: Critical, 3: Emergency, 4: Debug </summary>
        public  byte type;
            /// <summary> 0: blue, 1: yellow, 2: red, 3: orange, 4: green, 5: magenta </summary>
        public  byte color;
            /// <summary> 0: global, 1:local </summary>
        public  byte coordinate_system;
            /// <summary> 0: no timeout, >1: timeout in seconds </summary>
        public  UInt16 timeout;
            /// <summary> X Position </summary>
        public  Single x;
            /// <summary> Y Position </summary>
        public  Single y;
            /// <summary> Z Position </summary>
        public  Single z;
            /// <summary> POI name </summary>
        [MarshalAs(UnmanagedType.ByValArray,SizeConst=26)]
		public string name;
    
    };


    public const byte MAVLINK_MSG_ID_POINT_OF_INTEREST_CONNECTION = 192;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=55)]
    public struct mavlink_point_of_interest_connection_t
    {
        /// <summary> 0: Notice, 1: Warning, 2: Critical, 3: Emergency, 4: Debug </summary>
        public  byte type;
            /// <summary> 0: blue, 1: yellow, 2: red, 3: orange, 4: green, 5: magenta </summary>
        public  byte color;
            /// <summary> 0: global, 1:local </summary>
        public  byte coordinate_system;
            /// <summary> 0: no timeout, >1: timeout in seconds </summary>
        public  UInt16 timeout;
            /// <summary> X1 Position </summary>
        public  Single xp1;
            /// <summary> Y1 Position </summary>
        public  Single yp1;
            /// <summary> Z1 Position </summary>
        public  Single zp1;
            /// <summary> X2 Position </summary>
        public  Single xp2;
            /// <summary> Y2 Position </summary>
        public  Single yp2;
            /// <summary> Z2 Position </summary>
        public  Single zp2;
            /// <summary> POI connection name </summary>
        [MarshalAs(UnmanagedType.ByValArray,SizeConst=26)]
		public string name;
    
    };


    public const byte MAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE = 193;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=8)]
    public struct mavlink_data_transmission_handshake_t
    {
        /// <summary> type of requested/acknowledged data (as defined in ENUM DATA_TYPES in mavlink/include/mavlink_types.h) </summary>
        public  byte type;
            /// <summary> total data size in bytes (set on ACK only) </summary>
        public  UInt32 size;
            /// <summary> number of packets beeing sent (set on ACK only) </summary>
        public  byte packets;
            /// <summary> payload size per packet (normally 253 byte, see DATA field size in message ENCAPSULATED_DATA) (set on ACK only) </summary>
        public  byte payload;
            /// <summary> JPEG quality out of [1,100] </summary>
        public  byte jpg_quality;
    
    };


    public const byte MAVLINK_MSG_ID_ENCAPSULATED_DATA = 194;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=255)]
    public struct mavlink_encapsulated_data_t
    {
        /// <summary> sequence number (starting with 0 on every transmission) </summary>
        public  UInt16 seqnr;
            /// <summary> image data bytes </summary>
        [MarshalAs(UnmanagedType.ByValArray,SizeConst=253)]
		public byte[] data;
    
    };


    public const byte MAVLINK_MSG_ID_BRIEF_FEATURE = 195;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=53)]
    public struct mavlink_brief_feature_t
    {
        /// <summary> x position in m </summary>
        public  Single x;
            /// <summary> y position in m </summary>
        public  Single y;
            /// <summary> z position in m </summary>
        public  Single z;
            /// <summary> Orientation assignment 0: false, 1:true </summary>
        public  byte orientation_assignment;
            /// <summary> Size in pixels </summary>
        public  UInt16 size;
            /// <summary> Orientation </summary>
        public  UInt16 orientation;
            /// <summary> Descriptor </summary>
        [MarshalAs(UnmanagedType.ByValArray,SizeConst=32)]
		public byte[] descriptor;
            /// <summary> Harris operator response at this location </summary>
        public  Single response;
    
    };


    public const byte MAVLINK_MSG_ID_ATTITUDE_CONTROL = 200;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=21)]
    public struct mavlink_attitude_control_t
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
