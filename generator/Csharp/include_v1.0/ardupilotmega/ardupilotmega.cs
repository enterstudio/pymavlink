/** @file
 *	@brief MAVLink comm protocol generated from ardupilotmega.xml
 *	@see http://qgroundcontrol.org/mavlink/
 */
 using System;

// MESSAGE LENGTHS AND CRCS

public partial class Mavlink
{

    public byte[] MAVLINK_MESSAGE_LENGTHS = new byte[] {8, 23, 12, 8, 14, 28, 3, 32, 0, 0, 0, 2, 2, 2, 0, 0, 0, 0, 0, 0, 20, 2, 25, 23, 0, 30, 26, 101, 26, 16, 32, 32, 38, 32, 0, 17, 17, 16, 18, 36, 4, 4, 2, 2, 4, 2, 2, 3, 14, 12, 18, 16, 14, 27, 25, 18, 18, 20, 20, 0, 0, 0, 26, 16, 36, 0, 6, 4, 0, 21, 18, 0, 0, 20, 20, 20, 32, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 56, 42, 33, 0, 0, 0, 0, 0, 0, 0, 18, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 42, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 36, 30, 14, 14, 51, 5};

    public byte[]  MAVLINK_MESSAGE_CRCS = new byte[] {153, 114, 143, 191, 105, 217, 104, 119, 0, 0, 0, 186, 194, 8, 0, 0, 0, 0, 0, 0, 214, 159, 220, 168, 0, 185, 222, 23, 179, 136, 66, 126, 198, 147, 0, 252, 162, 215, 229, 205, 51, 80, 101, 213, 8, 229, 21, 214, 170, 14, 73, 50, 142, 15, 3, 100, 24, 141, 148, 0, 0, 0, 183, 126, 130, 0, 148, 21, 0, 52, 124, 0, 0, 241, 20, 160, 168, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 12, 250, 156, 0, 0, 0, 0, 0, 0, 0, 146, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 134, 219, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 204, 15, 248, 63, 83, 127};

    public Type[] MAVLINK_MESSAGE_INFO = new Type[] {typeof( mavlink_heartbeat_t ), typeof( mavlink_sys_status_t ), typeof( mavlink_system_time_t ), typeof( mavlink_system_time_utc_t ), typeof( mavlink_ping_t ), typeof( mavlink_change_operator_control_t ), typeof( mavlink_change_operator_control_ack_t ), typeof( mavlink_auth_key_t ), null, null, null, typeof( mavlink_set_mode_t ), typeof( mavlink_set_flight_mode_t ), typeof( mavlink_set_safety_mode_t ), null, null, null, null, null, null, typeof( mavlink_param_request_read_t ), typeof( mavlink_param_request_list_t ), typeof( mavlink_param_value_t ), typeof( mavlink_param_set_t ), null, typeof( mavlink_gps_raw_int_t ), typeof( mavlink_scaled_imu_t ), typeof( mavlink_gps_status_t ), typeof( mavlink_raw_imu_t ), typeof( mavlink_raw_pressure_t ), typeof( mavlink_attitude_t ), typeof( mavlink_local_position_t ), typeof( mavlink_gps_raw_t ), typeof( mavlink_global_position_t ), null, typeof( mavlink_rc_channels_raw_t ), typeof( mavlink_rc_channels_scaled_t ), typeof( mavlink_servo_output_raw_t ), typeof( mavlink_scaled_pressure_t ), typeof( mavlink_waypoint_t ), typeof( mavlink_waypoint_request_t ), typeof( mavlink_waypoint_set_current_t ), typeof( mavlink_waypoint_current_t ), typeof( mavlink_waypoint_request_list_t ), typeof( mavlink_waypoint_count_t ), typeof( mavlink_waypoint_clear_all_t ), typeof( mavlink_waypoint_reached_t ), typeof( mavlink_waypoint_ack_t ), typeof( mavlink_gps_set_global_origin_t ), typeof( mavlink_gps_local_origin_set_t ), typeof( mavlink_local_position_setpoint_set_t ), typeof( mavlink_local_position_setpoint_t ), typeof( mavlink_global_position_setpoint_int_t ), typeof( mavlink_safety_set_allowed_area_t ), typeof( mavlink_safety_allowed_area_t ), typeof( mavlink_set_roll_pitch_yaw_thrust_t ), typeof( mavlink_set_roll_pitch_yaw_speed_thrust_t ), typeof( mavlink_roll_pitch_yaw_thrust_setpoint_t ), typeof( mavlink_roll_pitch_yaw_speed_thrust_setpoint_t ), null, null, null, typeof( mavlink_nav_controller_output_t ), typeof( mavlink_position_target_t ), typeof( mavlink_state_correction_t ), null, typeof( mavlink_request_data_stream_t ), typeof( mavlink_data_stream_t ), null, typeof( mavlink_manual_control_t ), typeof( mavlink_rc_channels_override_t ), null, null, typeof( mavlink_global_position_int_t ), typeof( mavlink_vfr_hud_t ), typeof( mavlink_command_short_t ), typeof( mavlink_command_long_t ), typeof( mavlink_command_ack_t ), null, null, null, null, null, null, null, null, null, null, null, null, typeof( mavlink_hil_state_t ), typeof( mavlink_hil_controls_t ), typeof( mavlink_hil_rc_inputs_raw_t ), null, null, null, null, null, null, null, typeof( mavlink_optical_flow_t ), null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, typeof( mavlink_sensor_offsets_t ), typeof( mavlink_set_mag_offsets_t ), null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, typeof( mavlink_memory_vect_t ), typeof( mavlink_debug_vect_t ), typeof( mavlink_named_value_float_t ), typeof( mavlink_named_value_int_t ), typeof( mavlink_statustext_t ), typeof( mavlink_debug_t )};

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



}
