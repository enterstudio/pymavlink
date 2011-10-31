/** @file
 *	@brief MAVLink comm protocol generated from pixhawk.xml
 *	@see http://qgroundcontrol.org/mavlink/
 */
 using System;
 using System.Runtime.InteropServices;

// MESSAGE LENGTHS AND CRCS

public partial class Mavlink
{

    public byte[] MAVLINK_MESSAGE_LENGTHS = new byte[] {3, 4, 8, 14, 8, 28, 3, 32, 0, 2, 3, 2, 2, 0, 0, 0, 0, 0, 0, 0, 19, 2, 23, 21, 0, 37, 26, 101, 26, 16, 32, 32, 37, 32, 11, 17, 17, 16, 18, 36, 4, 4, 2, 2, 4, 2, 2, 3, 14, 12, 18, 16, 8, 27, 25, 18, 18, 24, 24, 0, 0, 0, 26, 16, 36, 5, 6, 56, 26, 21, 18, 0, 0, 18, 20, 20, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 18, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 36, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 11, 52, 1, 92, 0, 32, 32, 20, 20, 18, 0, 0, 0, 0, 0, 0, 0, 0, 0, 18, 26, 16, 0, 0, 0, 0, 0, 0, 0, 4, 255, 12, 6, 0, 0, 0, 0, 0, 0, 106, 43, 55, 8, 255, 53, 0, 0, 0, 0, 21, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 30, 14, 14, 51, 5};

    public byte[] MAVLINK_MESSAGE_CRCS = new byte[] {72, 39, 190, 92, 191, 217, 104, 119, 0, 219, 60, 186, 10, 0, 0, 0, 0, 0, 0, 0, 89, 159, 162, 121, 0, 149, 222, 110, 179, 136, 66, 126, 185, 147, 112, 252, 162, 215, 229, 128, 9, 106, 101, 213, 4, 229, 21, 214, 215, 14, 206, 50, 157, 126, 108, 213, 95, 5, 127, 0, 0, 0, 57, 126, 130, 119, 193, 191, 236, 158, 143, 0, 0, 104, 123, 131, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 174, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 155, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 204, 86, 95, 49, 0, 158, 56, 208, 218, 115, 0, 0, 0, 0, 0, 0, 0, 0, 0, 220, 136, 140, 0, 0, 0, 0, 0, 0, 0, 153, 110, 92, 188, 0, 0, 0, 0, 0, 0, 106, 154, 83, 98, 223, 254, 0, 0, 0, 0, 84, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 178, 224, 60, 106, 7};

    public Type[] MAVLINK_MESSAGE_INFO = new Type[] {typeof( mavlink_heartbeat_t ), typeof( mavlink_boot_t ), typeof( mavlink_system_time_t ), typeof( mavlink_ping_t ), typeof( mavlink_system_time_utc_t ), typeof( mavlink_change_operator_control_t ), typeof( mavlink_change_operator_control_ack_t ), typeof( mavlink_auth_key_t ), null, typeof( mavlink_action_ack_t ), typeof( mavlink_action_t ), typeof( mavlink_set_mode_t ), typeof( mavlink_set_nav_mode_t ), null, null, null, null, null, null, null, typeof( mavlink_param_request_read_t ), typeof( mavlink_param_request_list_t ), typeof( mavlink_param_value_t ), typeof( mavlink_param_set_t ), null, typeof( mavlink_gps_raw_int_t ), typeof( mavlink_scaled_imu_t ), typeof( mavlink_gps_status_t ), typeof( mavlink_raw_imu_t ), typeof( mavlink_raw_pressure_t ), typeof( mavlink_attitude_t ), typeof( mavlink_local_position_t ), typeof( mavlink_gps_raw_t ), typeof( mavlink_global_position_t ), typeof( mavlink_sys_status_t ), typeof( mavlink_rc_channels_raw_t ), typeof( mavlink_rc_channels_scaled_t ), typeof( mavlink_servo_output_raw_t ), typeof( mavlink_scaled_pressure_t ), typeof( mavlink_waypoint_t ), typeof( mavlink_waypoint_request_t ), typeof( mavlink_waypoint_set_current_t ), typeof( mavlink_waypoint_current_t ), typeof( mavlink_waypoint_request_list_t ), typeof( mavlink_waypoint_count_t ), typeof( mavlink_waypoint_clear_all_t ), typeof( mavlink_waypoint_reached_t ), typeof( mavlink_waypoint_ack_t ), typeof( mavlink_gps_set_global_origin_t ), typeof( mavlink_gps_local_origin_set_t ), typeof( mavlink_local_position_setpoint_set_t ), typeof( mavlink_local_position_setpoint_t ), typeof( mavlink_control_status_t ), typeof( mavlink_safety_set_allowed_area_t ), typeof( mavlink_safety_allowed_area_t ), typeof( mavlink_set_roll_pitch_yaw_thrust_t ), typeof( mavlink_set_roll_pitch_yaw_speed_thrust_t ), typeof( mavlink_roll_pitch_yaw_thrust_setpoint_t ), typeof( mavlink_roll_pitch_yaw_speed_thrust_setpoint_t ), null, null, null, typeof( mavlink_nav_controller_output_t ), typeof( mavlink_position_target_t ), typeof( mavlink_state_correction_t ), typeof( mavlink_set_altitude_t ), typeof( mavlink_request_data_stream_t ), typeof( mavlink_hil_state_t ), typeof( mavlink_hil_controls_t ), typeof( mavlink_manual_control_t ), typeof( mavlink_rc_channels_override_t ), null, null, typeof( mavlink_global_position_int_t ), typeof( mavlink_vfr_hud_t ), typeof( mavlink_command_t ), typeof( mavlink_command_ack_t ), null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, typeof( mavlink_optical_flow_t ), null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, typeof( mavlink_object_detection_event_t ), null, null, null, null, null, null, null, null, null, null, typeof( mavlink_set_cam_shutter_t ), typeof( mavlink_image_triggered_t ), typeof( mavlink_image_trigger_control_t ), typeof( mavlink_image_available_t ), null, typeof( mavlink_vision_position_estimate_t ), typeof( mavlink_vicon_position_estimate_t ), typeof( mavlink_vision_speed_estimate_t ), typeof( mavlink_position_control_setpoint_set_t ), typeof( mavlink_position_control_offset_set_t ), null, null, null, null, null, null, null, null, null, typeof( mavlink_position_control_setpoint_t ), typeof( mavlink_marker_t ), typeof( mavlink_raw_aux_t ), null, null, null, null, null, null, null, typeof( mavlink_watchdog_heartbeat_t ), typeof( mavlink_watchdog_process_info_t ), typeof( mavlink_watchdog_process_status_t ), typeof( mavlink_watchdog_command_t ), null, null, null, null, null, null, typeof( mavlink_pattern_detected_t ), typeof( mavlink_point_of_interest_t ), typeof( mavlink_point_of_interest_connection_t ), typeof( mavlink_data_transmission_handshake_t ), typeof( mavlink_encapsulated_data_t ), typeof( mavlink_brief_feature_t ), null, null, null, null, typeof( mavlink_attitude_control_t ), null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, typeof( mavlink_debug_vect_t ), typeof( mavlink_named_value_float_t ), typeof( mavlink_named_value_int_t ), typeof( mavlink_statustext_t ), typeof( mavlink_debug_t )};

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
    
    public static char[] toArray(string text)
    {
        return text.ToCharArray();
    }
    
    public static byte[] toArray(byte[] data)
    {
        return data;
    }

    const int X25_INIT_CRC = 0xffff;
    const int X25_VALIDATE_CRC = 0xf0b8;

    ushort crc_accumulate(byte b, ushort crc)
    {
        unchecked
        {
            byte ch = (byte)(b ^ (byte)(crc & 0x00ff));
            ch = (byte)(ch ^ (ch << 4));
            return (ushort)((crc >> 8) ^ (ch << 8) ^ (ch << 3) ^ (ch >> 4));
        }
    }

    ushort crc_calculate(byte[] pBuffer, int length)
    {
        if (length < 1)
        {
            return 0xffff;
        }
        // For a "message" of length bytes contained in the unsigned char array
        // pointed to by pBuffer, calculate the CRC
        // crcCalculate(unsigned char* pBuffer, int length, unsigned short* checkConst) < not needed

        ushort crcTmp;
        int i;

        crcTmp = X25_INIT_CRC;

        for (i = 1; i < length; i++) // skips header U
        {
            crcTmp = crc_accumulate(pBuffer[i], crcTmp);
            //Console.WriteLine(crcTmp + " " + pBuffer[i] + " " + length);
        }

        return (crcTmp);
    }
    
    public byte[] generatePacket(object indata, byte sysid, byte compid)
    {
        byte messageType = 0;
        foreach (Type ty in MAVLINK_MESSAGE_INFO)
        {
            if (ty == indata.GetType()) {
                break;
            }
            messageType++;
        }
        
        byte[] data;
        
        if (MAVLINK_ENDIAN == MAVLINK_LITTLE_ENDIAN)
        {
            data = StructureToByteArray(indata);
        }
        else
        {
            data = StructureToByteArrayEndian(indata);
        }
        
        byte[] packet = new byte[data.Length + 6 + 2];
        
        packet[0] = (byte)MAVLINK_STX;
        packet[1] = (byte)data.Length;
        packet[2] = packetcount;
        packet[3] = sysid;
        packet[4] = compid;
        packet[5] = messageType;
        
        int i = 6;
        foreach (byte b in data)
        {
            packet[i] = b;
            i++;
        }
        
        
        ushort checksum = crc_calculate(packet, packet[1] + 6);

        if (MAVLINK_CRC_EXTRA == 1)
        {
            checksum = crc_accumulate(MAVLINK_MESSAGE_CRCS[messageType], checksum);
        }

        byte ck_a = (byte)(checksum & 0xFF); ///< High byte
        byte ck_b = (byte)(checksum >> 8); ///< Low byte

        packet[i] = ck_a;
        i += 1;
        packet[i] = ck_b;
        i += 1;
        
        return packet;
    }
    
    byte[] StructureToByteArrayEndian(params object[] list)
    {
        // The copy is made becuase SetValue won't work on a struct.
        // Boxing was used because SetValue works on classes/objects.
        // Unfortunately, it results in 2 copy operations.
        object thisBoxed = list[0]; // Why make a copy?
        Type test = thisBoxed.GetType();

        int offset = 0;
        byte[] data = new byte[Marshal.SizeOf(thisBoxed)];

        // System.Net.IPAddress.NetworkToHostOrder is used to perform byte swapping.
        // To convert unsigned to signed, 'unchecked()' was used.
        // See http://stackoverflow.com/questions/1131843/how-do-i-convert-uint-to-int-in-c

        object fieldValue;
        TypeCode typeCode;

        byte[] temp;

        // Enumerate each structure field using reflection.
        foreach (var field in test.GetFields())
        {
            // field.Name has the field's name.

            fieldValue = field.GetValue(thisBoxed); // Get value

            // Get the TypeCode enumeration. Multiple types get mapped to a common typecode.
            typeCode = Type.GetTypeCode(fieldValue.GetType());

            switch (typeCode)
            {
                case TypeCode.Single: // float
                    {
                        temp = BitConverter.GetBytes((Single)fieldValue);
                        Array.Reverse(temp);
                        Array.Copy(temp, 0, data, offset, sizeof(Single));
                        break;
                    }
                case TypeCode.Int32:
                    {
                        temp = BitConverter.GetBytes((Int32)fieldValue);
                        Array.Reverse(temp);
                        Array.Copy(temp, 0, data, offset, sizeof(Int32));
                        break;
                    }
                case TypeCode.UInt32:
                    {
                        temp = BitConverter.GetBytes((UInt32)fieldValue);
                        Array.Reverse(temp);
                        Array.Copy(temp, 0, data, offset, sizeof(UInt32));
                        break;
                    }
                case TypeCode.Int16:
                    {
                        temp = BitConverter.GetBytes((Int16)fieldValue);
                        Array.Reverse(temp);
                        Array.Copy(temp, 0, data, offset, sizeof(Int16));
                        break;
                    }
                case TypeCode.UInt16:
                    {
                        temp = BitConverter.GetBytes((UInt16)fieldValue);
                        Array.Reverse(temp);
                        Array.Copy(temp, 0, data, offset, sizeof(UInt16));
                        break;
                    }
                case TypeCode.Int64:
                    {
                        temp = BitConverter.GetBytes((Int64)fieldValue);
                        Array.Reverse(temp);
                        Array.Copy(temp, 0, data, offset, sizeof(Int64));
                        break;
                    }
                case TypeCode.UInt64:
                    {
                        temp = BitConverter.GetBytes((UInt64)fieldValue);
                        Array.Reverse(temp);
                        Array.Copy(temp, 0, data, offset, sizeof(UInt64));
                        break;
                    }
                case TypeCode.Double:
                    {
                        temp = BitConverter.GetBytes((Double)fieldValue);
                        Array.Reverse(temp);
                        Array.Copy(temp, 0, data, offset, sizeof(Double));
                        break;
                    }
                case TypeCode.Byte:
                    {
                        data[offset] = (Byte)fieldValue;
                        break;
                    }
                default:
                    {
                        //System.Diagnostics.Debug.Fail("No conversion provided for this type : " + typeCode.ToString());
                        break;
                    }
            }; // switch
            if (typeCode == TypeCode.Object)
            {
                int length = ((byte[])fieldValue).Length;
                Array.Copy(((byte[])fieldValue), 0, data, offset, length);
                offset += length;
            }
            else
            {
                offset += Marshal.SizeOf(fieldValue);
            }
        } // foreach

        return data;
    } // Swap
    
    byte[] StructureToByteArray(object obj)
    {

        int len = Marshal.SizeOf(obj);

        byte[] arr = new byte[len];

        IntPtr ptr = Marshal.AllocHGlobal(len);

        Marshal.StructureToPtr(obj, ptr, true);

        Marshal.Copy(ptr, arr, 0, len);

        Marshal.FreeHGlobal(ptr);

        return arr;

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


/** @brief Content Types for data transmission handshake */
    public enum DATA_TYPES
    {
	///<summary> /*  | */ </summary>
        DATA_TYPE_JPEG_IMAGE=1, 
    	///<summary> /*  | */ </summary>
        DATA_TYPE_RAW_IMAGE=2, 
    	///<summary> /*  | */ </summary>
        DATA_TYPE_KINECT=3, 
    	///<summary> /*  | */ </summary>
        DATA_TYPES_ENUM_END=4, 
    
    };

}
