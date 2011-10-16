/** @file
 *	@brief MAVLink comm protocol generated from ualberta.xml
 *	@see http://qgroundcontrol.org/mavlink/
 */
 using System;

// MESSAGE LENGTHS AND CRCS

public partial class Mavlink
{

    public byte[] MAVLINK_MESSAGE_LENGTHS = new byte[] {9, 31, 12, 0, 14, 28, 3, 32, 0, 0, 0, 4, 0, 0, 0, 0, 0, 0, 0, 0, 20, 2, 25, 23, 30, 101, 22, 26, 16, 14, 28, 32, 28, 0, 28, 22, 22, 21, 0, 36, 4, 4, 2, 2, 4, 2, 2, 3, 13, 12, 19, 17, 15, 15, 27, 25, 18, 18, 20, 20, 0, 0, 26, 0, 36, 0, 6, 4, 0, 21, 18, 0, 0, 0, 20, 20, 32, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 56, 42, 33, 0, 0, 0, 0, 0, 0, 0, 18, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 32, 42, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 36, 30, 18, 18, 51, 9, 3};

    public byte[]  MAVLINK_MESSAGE_CRCS = new byte[] {50, 124, 137, 0, 237, 217, 104, 119, 0, 0, 0, 197, 0, 0, 0, 0, 0, 0, 0, 0, 214, 159, 220, 168, 24, 23, 170, 144, 67, 115, 39, 246, 185, 0, 104, 244, 237, 222, 0, 158, 230, 28, 28, 132, 221, 232, 11, 153, 41, 39, 214, 223, 141, 33, 15, 3, 100, 24, 239, 238, 0, 0, 183, 0, 130, 0, 148, 21, 0, 52, 124, 0, 0, 0, 20, 160, 168, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 183, 63, 54, 0, 0, 0, 0, 0, 0, 0, 19, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 34, 71, 15, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 204, 49, 170, 44, 83, 46, 247};

    public Type[] MAVLINK_MESSAGE_INFO = new Type[] {typeof( mavlink_heartbeat_t ), typeof( mavlink_sys_status_t ), typeof( mavlink_system_time_t ), null, typeof( mavlink_ping_t ), typeof( mavlink_change_operator_control_t ), typeof( mavlink_change_operator_control_ack_t ), typeof( mavlink_auth_key_t ), null, null, null, typeof( mavlink_set_mode_t ), null, null, null, null, null, null, null, null, typeof( mavlink_param_request_read_t ), typeof( mavlink_param_request_list_t ), typeof( mavlink_param_value_t ), typeof( mavlink_param_set_t ), typeof( mavlink_gps_raw_int_t ), typeof( mavlink_gps_status_t ), typeof( mavlink_scaled_imu_t ), typeof( mavlink_raw_imu_t ), typeof( mavlink_raw_pressure_t ), typeof( mavlink_scaled_pressure_t ), typeof( mavlink_attitude_t ), typeof( mavlink_attitude_quaternion_t ), typeof( mavlink_local_position_ned_t ), null, typeof( mavlink_global_position_int_t ), typeof( mavlink_rc_channels_raw_t ), typeof( mavlink_rc_channels_scaled_t ), typeof( mavlink_servo_output_raw_t ), null, typeof( mavlink_mission_item_t ), typeof( mavlink_mission_request_t ), typeof( mavlink_mission_set_current_t ), typeof( mavlink_mission_current_t ), typeof( mavlink_mission_request_list_t ), typeof( mavlink_mission_count_t ), typeof( mavlink_mission_clear_all_t ), typeof( mavlink_mission_item_reached_t ), typeof( mavlink_mission_ack_t ), typeof( mavlink_set_gps_global_origin_t ), typeof( mavlink_gps_global_origin_t ), typeof( mavlink_set_local_position_setpoint_t ), typeof( mavlink_local_position_setpoint_t ), typeof( mavlink_global_position_setpoint_int_t ), typeof( mavlink_set_global_position_setpoint_int_t ), typeof( mavlink_safety_set_allowed_area_t ), typeof( mavlink_safety_allowed_area_t ), typeof( mavlink_set_roll_pitch_yaw_thrust_t ), typeof( mavlink_set_roll_pitch_yaw_speed_thrust_t ), typeof( mavlink_roll_pitch_yaw_thrust_setpoint_t ), typeof( mavlink_roll_pitch_yaw_speed_thrust_setpoint_t ), null, null, typeof( mavlink_nav_controller_output_t ), null, typeof( mavlink_state_correction_t ), null, typeof( mavlink_request_data_stream_t ), typeof( mavlink_data_stream_t ), null, typeof( mavlink_manual_control_t ), typeof( mavlink_rc_channels_override_t ), null, null, null, typeof( mavlink_vfr_hud_t ), typeof( mavlink_command_short_t ), typeof( mavlink_command_long_t ), typeof( mavlink_command_ack_t ), null, null, null, null, null, null, null, null, null, null, null, null, typeof( mavlink_hil_state_t ), typeof( mavlink_hil_controls_t ), typeof( mavlink_hil_rc_inputs_raw_t ), null, null, null, null, null, null, null, typeof( mavlink_optical_flow_t ), null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, typeof( mavlink_nav_filter_bias_t ), typeof( mavlink_radio_calibration_t ), typeof( mavlink_ualberta_sys_status_t ), null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, typeof( mavlink_memory_vect_t ), typeof( mavlink_debug_vect_t ), typeof( mavlink_named_value_float_t ), typeof( mavlink_named_value_int_t ), typeof( mavlink_statustext_t ), typeof( mavlink_debug_t ), typeof( mavlink_extended_message_t )};

    public const byte MAVLINK_VERSION = 2;
    
    public static byte getByte(byte[] msg, int offset) 
    {
        return msg[offset];
    }
    
    public static byte[] getBytes(byte[] msg, int length, int offset)
    {
        byte[] temp = new byte[length];
        Array.Copy(msg, offset, temp, 0, length);
        return temp;
    }

// ENUM DEFINITIONS

public struct mavlink_message_t {
	public byte magic;   ///< protocol magic marker
	public byte len;     ///< Length of payload
	public byte seq;     ///< Sequence of packet
	public byte sysid;   ///< ID of message sender system/aircraft
	public byte compid;  ///< ID of the message sender component
	public byte msgid;   ///< ID of message in payload
	public byte[] payload;
    public UInt16 checksum; /// sent at end of packet
};


/** @brief Available autopilot modes for ualberta uav */
    public enum UALBERTA_AUTOPILOT_MODE
    {
        	MODE_MANUAL_DIRECT=0, /*  | */
    	MODE_MANUAL_SCALED=1, /*  | */
    	MODE_AUTO_PID_ATT=2, /*  | */
    	MODE_AUTO_PID_VEL=3, /*  | */
    	MODE_AUTO_PID_POS=4, /*  | */
    	UALBERTA_AUTOPILOT_MODE_ENUM_END=5, /*  | */
    
    };

/** @brief Navigation filter mode */
    public enum UALBERTA_NAV_MODE
    {
        	NAV_AHRS_INIT=0, /*  | */
    	NAV_AHRS=1, /*  | */
    	NAV_INS_GPS_INIT=2, /*  | */
    	NAV_INS_GPS=3, /*  | */
    	UALBERTA_NAV_MODE_ENUM_END=4, /*  | */
    
    };

/** @brief Mode currently commanded by pilot */
    public enum UALBERTA_PILOT_MODE
    {
        	PILOT_MANUAL=0, /*  | */
    	PILOT_AUTO=1, /*  | */
    	PILOT_ROTO=2, /*  | */
    	UALBERTA_PILOT_MODE_ENUM_END=3, /*  | */
    
    };

}
