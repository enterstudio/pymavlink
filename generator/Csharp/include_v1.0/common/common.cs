/** @file
 *	@brief MAVLink comm protocol generated from common.xml
 *	@see http://qgroundcontrol.org/mavlink/
 */
 using System;
 using System.Runtime.InteropServices;

// MESSAGE LENGTHS AND CRCS

public partial class Mavlink
{

    public byte[] MAVLINK_MESSAGE_LENGTHS = new byte[] {9, 31, 12, 0, 14, 28, 3, 32, 0, 0, 0, 6, 0, 0, 0, 0, 0, 0, 0, 0, 20, 2, 25, 23, 30, 101, 22, 26, 16, 14, 28, 32, 28, 0, 28, 22, 22, 21, 0, 36, 4, 4, 2, 2, 4, 2, 2, 3, 13, 12, 19, 17, 15, 15, 27, 25, 18, 18, 20, 20, 0, 0, 26, 0, 36, 0, 6, 4, 0, 21, 18, 0, 0, 0, 20, 20, 32, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 56, 42, 33, 0, 0, 0, 0, 0, 0, 0, 18, 32, 32, 20, 32, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 36, 30, 18, 18, 51, 9, 3};

    public byte[] MAVLINK_MESSAGE_CRCS = new byte[] {50, 124, 137, 0, 237, 217, 104, 119, 0, 0, 0, 89, 0, 0, 0, 0, 0, 0, 0, 0, 214, 159, 220, 168, 24, 23, 170, 144, 67, 115, 39, 246, 185, 0, 104, 244, 237, 222, 0, 158, 230, 28, 28, 132, 221, 232, 11, 153, 41, 39, 214, 223, 141, 33, 15, 3, 100, 24, 239, 238, 0, 0, 183, 0, 130, 0, 148, 21, 0, 52, 124, 0, 0, 0, 20, 160, 168, 143, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 183, 63, 54, 0, 0, 0, 0, 0, 0, 0, 19, 102, 158, 208, 56, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 204, 49, 170, 44, 83, 46, 247};

    public Type[] MAVLINK_MESSAGE_INFO = new Type[] {typeof( mavlink_heartbeat_t ), typeof( mavlink_sys_status_t ), typeof( mavlink_system_time_t ), null, typeof( mavlink_ping_t ), typeof( mavlink_change_operator_control_t ), typeof( mavlink_change_operator_control_ack_t ), typeof( mavlink_auth_key_t ), null, null, null, typeof( mavlink_set_mode_t ), null, null, null, null, null, null, null, null, typeof( mavlink_param_request_read_t ), typeof( mavlink_param_request_list_t ), typeof( mavlink_param_value_t ), typeof( mavlink_param_set_t ), typeof( mavlink_gps_raw_int_t ), typeof( mavlink_gps_status_t ), typeof( mavlink_scaled_imu_t ), typeof( mavlink_raw_imu_t ), typeof( mavlink_raw_pressure_t ), typeof( mavlink_scaled_pressure_t ), typeof( mavlink_attitude_t ), typeof( mavlink_attitude_quaternion_t ), typeof( mavlink_local_position_ned_t ), null, typeof( mavlink_global_position_int_t ), typeof( mavlink_rc_channels_raw_t ), typeof( mavlink_rc_channels_scaled_t ), typeof( mavlink_servo_output_raw_t ), null, typeof( mavlink_mission_item_t ), typeof( mavlink_mission_request_t ), typeof( mavlink_mission_set_current_t ), typeof( mavlink_mission_current_t ), typeof( mavlink_mission_request_list_t ), typeof( mavlink_mission_count_t ), typeof( mavlink_mission_clear_all_t ), typeof( mavlink_mission_item_reached_t ), typeof( mavlink_mission_ack_t ), typeof( mavlink_set_gps_global_origin_t ), typeof( mavlink_gps_global_origin_t ), typeof( mavlink_set_local_position_setpoint_t ), typeof( mavlink_local_position_setpoint_t ), typeof( mavlink_global_position_setpoint_int_t ), typeof( mavlink_set_global_position_setpoint_int_t ), typeof( mavlink_safety_set_allowed_area_t ), typeof( mavlink_safety_allowed_area_t ), typeof( mavlink_set_roll_pitch_yaw_thrust_t ), typeof( mavlink_set_roll_pitch_yaw_speed_thrust_t ), typeof( mavlink_roll_pitch_yaw_thrust_setpoint_t ), typeof( mavlink_roll_pitch_yaw_speed_thrust_setpoint_t ), null, null, typeof( mavlink_nav_controller_output_t ), null, typeof( mavlink_state_correction_t ), null, typeof( mavlink_request_data_stream_t ), typeof( mavlink_data_stream_t ), null, typeof( mavlink_manual_control_t ), typeof( mavlink_rc_channels_override_t ), null, null, null, typeof( mavlink_vfr_hud_t ), typeof( mavlink_command_short_t ), typeof( mavlink_command_long_t ), typeof( mavlink_command_ack_t ), null, null, null, null, null, null, null, null, null, null, null, null, typeof( mavlink_hil_state_t ), typeof( mavlink_hil_controls_t ), typeof( mavlink_hil_rc_inputs_raw_t ), null, null, null, null, null, null, null, typeof( mavlink_optical_flow_t ), typeof( mavlink_global_vision_position_estimate_t ), typeof( mavlink_vision_position_estimate_t ), typeof( mavlink_vision_speed_estimate_t ), typeof( mavlink_vicon_position_estimate_t ), null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, typeof( mavlink_memory_vect_t ), typeof( mavlink_debug_vect_t ), typeof( mavlink_named_value_float_t ), typeof( mavlink_named_value_int_t ), typeof( mavlink_statustext_t ), typeof( mavlink_debug_t ), typeof( mavlink_extended_message_t )};

    public const byte MAVLINK_VERSION = 3;
    
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


/** @brief Micro air vehicle / autopilot classes. This identifies the individual model. */
    public enum MAV_AUTOPILOT
    {
	///<summary> /* Generic autopilot, full support for everything | */ </summary>
        MAV_AUTOPILOT_GENERIC=0, 
    	///<summary> /* PIXHAWK autopilot, http://pixhawk.ethz.ch | */ </summary>
        MAV_AUTOPILOT_PIXHAWK=1, 
    	///<summary> /* SLUGS autopilot, http://slugsuav.soe.ucsc.edu | */ </summary>
        MAV_AUTOPILOT_SLUGS=2, 
    	///<summary> /* ArduPilotMega / ArduCopter, http://diydrones.com | */ </summary>
        MAV_AUTOPILOT_ARDUPILOTMEGA=3, 
    	///<summary> /* OpenPilot, http://openpilot.org | */ </summary>
        MAV_AUTOPILOT_OPENPILOT=4, 
    	///<summary> /* Generic autopilot only supporting simple waypoints | */ </summary>
        MAV_AUTOPILOT_GENERIC_WAYPOINTS_ONLY=5, 
    	///<summary> /* Generic autopilot supporting waypoints and other simple navigation commands | */ </summary>
        MAV_AUTOPILOT_GENERIC_WAYPOINTS_AND_SIMPLE_NAVIGATION_ONLY=6, 
    	///<summary> /* Generic autopilot supporting the full mission command set | */ </summary>
        MAV_AUTOPILOT_GENERIC_MISSION_FULL=7, 
    	///<summary> /* No valid autopilot, e.g. a GCS or other MAVLink component | */ </summary>
        MAV_AUTOPILOT_INVALID=8, 
    	///<summary> /* PPZ UAV - http://nongnu.org/paparazzi | */ </summary>
        MAV_AUTOPILOT_PPZ=9, 
    	///<summary> /* UAV Dev Board | */ </summary>
        MAV_AUTOPILOT_UDB=10, 
    	///<summary> /* FlexiPilot | */ </summary>
        MAV_AUTOPILOT_FP=11, 
    	///<summary> /*  | */ </summary>
        MAV_AUTOPILOT_ENUM_END=12, 
    
    };

/** @brief These flags encode the MAV mode. */
    public enum MAV_MODE_FLAG
    {
	///<summary> /* 0b10000000 MAV safety set to armed. Motors are enabled / running / can start. Ready to fly. | */ </summary>
        MAV_MODE_FLAG_SAFETY_ARMED=128, 
    	///<summary> /* 0b01000000 remote control input is enabled. | */ </summary>
        MAV_MODE_FLAG_MANUAL_INPUT_ENABLED=64, 
    	///<summary> /* 0b00100000 hardware in the loop simulation. All motors / actuators are blocked, but internal software is full operational. | */ </summary>
        MAV_MODE_FLAG_HIL_ENABLED=32, 
    	///<summary> /* 0b00010000 system stabilizes electronically its attitude (and optionally position). It needs however further control inputs to move around. | */ </summary>
        MAV_MODE_FLAG_STABILIZE_ENABLED=16, 
    	///<summary> /* 0b00001000 guided mode enabled, system flies MISSIONs / mission items. | */ </summary>
        MAV_MODE_FLAG_GUIDED_ENABLED=8, 
    	///<summary> /* 0b00000100 autonomous mode enabled, system finds its own goal positions. Guided flag can be set or not, depends on the actual implementation. | */ </summary>
        MAV_MODE_FLAG_AUTO_ENABLED=4, 
    	///<summary> /* 0b00000010 system has a test mode enabled. This flag is intended for temporary system tests and should not be used for stable implementations. | */ </summary>
        MAV_MODE_FLAG_TEST_ENABLED=2, 
    	///<summary> /* 0b00000001 Reserved for future use. | */ </summary>
        MAV_MODE_FLAG_RESERVED_ENABLED=1, 
    	///<summary> /*  | */ </summary>
        MAV_MODE_FLAG_ENUM_END=129, 
    
    };

/** @brief These values encode the bit positions of the decode position. These values can be used to read the value of a flag bit by combining the base_mode variable with AND with the flag position value. The result will be either 0 or 1, depending on if the flag is set or not. */
    public enum MAV_MODE_FLAG_DECODE_POSITION
    {
	///<summary> /* First bit:  10000000 | */ </summary>
        MAV_MODE_FLAG_DECODE_POSITION_SAFETY=128, 
    	///<summary> /* Second bit: 01000000 | */ </summary>
        MAV_MODE_FLAG_DECODE_POSITION_MANUAL=64, 
    	///<summary> /* Third bit:  00100000 | */ </summary>
        MAV_MODE_FLAG_DECODE_POSITION_HIL=32, 
    	///<summary> /* Fourth bit: 00010000 | */ </summary>
        MAV_MODE_FLAG_DECODE_POSITION_STABILIZE=16, 
    	///<summary> /* Fifth bit:  00001000 | */ </summary>
        MAV_MODE_FLAG_DECODE_POSITION_GUIDED=8, 
    	///<summary> /* Sixt bit:   00000100 | */ </summary>
        MAV_MODE_FLAG_DECODE_POSITION_AUTO=4, 
    	///<summary> /* Seventh bit: 00000010 | */ </summary>
        MAV_MODE_FLAG_DECODE_POSITION_TEST=2, 
    	///<summary> /* Eighth bit: 00000001 | */ </summary>
        MAV_MODE_FLAG_DECODE_POSITION_RESERVED=1, 
    	///<summary> /*  | */ </summary>
        MAV_MODE_FLAG_DECODE_POSITION_ENUM_END=129, 
    
    };

/** @brief Override command, pauses current mission execution and moves immediately to a position */
    public enum MAV_GOTO
    {
	///<summary> /* Hold at the current position. | */ </summary>
        MAV_GOTO_DO_HOLD=0, 
    	///<summary> /* Continue with the mission execution. | */ </summary>
        MAV_GOTO_DO_CONTINUE=1, 
    	///<summary> /* Hold at the current position of the system | */ </summary>
        MAV_GOTO_HOLD_AT_CURRENT_POSITION=2, 
    	///<summary> /* Hold at the position specified in the parameters of the DO_HOLD action | */ </summary>
        MAV_GOTO_HOLD_AT_SPECIFIED_POSITION=3, 
    	///<summary> /*  | */ </summary>
        MAV_GOTO_ENUM_END=4, 
    
    };

/** @brief These defines are predefined OR-combined mode flags. There is no need to use values from this enum, but it
               simplifies the use of the mode flags. Note that manual input is enabled in all modes as a safety override. */
    public enum MAV_MODE
    {
	///<summary> /* System is not ready to fly, booting, calibrating, etc. No flag is set. | */ </summary>
        MAV_MODE_PREFLIGHT=0, 
    	///<summary> /* System is allowed to be active, under assisted RC control. | */ </summary>
        MAV_MODE_STABILIZE_DISARMED=80, 
    	///<summary> /* System is allowed to be active, under assisted RC control. | */ </summary>
        MAV_MODE_STABILIZE_ARMED=208, 
    	///<summary> /* System is allowed to be active, under manual (RC) control, no stabilization | */ </summary>
        MAV_MODE_MANUAL_DISARMED=64, 
    	///<summary> /* System is allowed to be active, under manual (RC) control, no stabilization | */ </summary>
        MAV_MODE_MANUAL_ARMED=192, 
    	///<summary> /* System is allowed to be active, under autonomous control, manual setpoint | */ </summary>
        MAV_MODE_GUIDED_DISARMED=88, 
    	///<summary> /* System is allowed to be active, under autonomous control, manual setpoint | */ </summary>
        MAV_MODE_GUIDED_ARMED=216, 
    	///<summary> /* System is allowed to be active, under autonomous control and navigation (the trajectory is decided onboard and not pre-programmed by MISSIONs) | */ </summary>
        MAV_MODE_AUTO_DISARMED=92, 
    	///<summary> /* System is allowed to be active, under autonomous control and navigation (the trajectory is decided onboard and not pre-programmed by MISSIONs) | */ </summary>
        MAV_MODE_AUTO_ARMED=220, 
    	///<summary> /* UNDEFINED mode. This solely depends on the autopilot - use with caution, intended for developers only. | */ </summary>
        MAV_MODE_TEST_DISARMED=66, 
    	///<summary> /* UNDEFINED mode. This solely depends on the autopilot - use with caution, intended for developers only. | */ </summary>
        MAV_MODE_TEST_ARMED=194, 
    	///<summary> /*  | */ </summary>
        MAV_MODE_ENUM_END=221, 
    
    };

/** @brief  */
    public enum MAV_STATE
    {
	///<summary> /* Uninitialized system, state is unknown. | */ </summary>
        MAV_STATE_UNINIT=0, 
    	///<summary> /* System is booting up. | */ </summary>
        MAV_STATE_BOOT=1, 
    	///<summary> /* System is calibrating and not flight-ready. | */ </summary>
        MAV_STATE_CALIBRATING=2, 
    	///<summary> /* System is grounded and on standby. It can be launched any time. | */ </summary>
        MAV_STATE_STANDBY=3, 
    	///<summary> /* System is active and might be already airborne. Motors are engaged. | */ </summary>
        MAV_STATE_ACTIVE=4, 
    	///<summary> /* System is in a non-normal flight mode. It can however still navigate. | */ </summary>
        MAV_STATE_CRITICAL=5, 
    	///<summary> /* System is in a non-normal flight mode. It lost control over parts or over the whole airframe. It is in mayday and going down. | */ </summary>
        MAV_STATE_EMERGENCY=6, 
    	///<summary> /* System just initialized its power-down sequence, will shut down now. | */ </summary>
        MAV_STATE_POWEROFF=7, 
    	///<summary> /*  | */ </summary>
        MAV_STATE_ENUM_END=8, 
    
    };

/** @brief  */
    public enum MAV_TYPE
    {
	///<summary> /* Generic micro air vehicle. | */ </summary>
        MAV_TYPE_GENERIC=0, 
    	///<summary> /* Fixed wing aircraft. | */ </summary>
        MAV_TYPE_FIXED_WING=1, 
    	///<summary> /* Quadrotor | */ </summary>
        MAV_TYPE_QUADROTOR=2, 
    	///<summary> /* Coaxial helicopter | */ </summary>
        MAV_TYPE_COAXIAL=3, 
    	///<summary> /* Normal helicopter with tail rotor. | */ </summary>
        MAV_TYPE_HELICOPTER=4, 
    	///<summary> /* Ground installation | */ </summary>
        MAV_TYPE_ANTENNA_TRACKER=5, 
    	///<summary> /* Operator control unit / ground control station | */ </summary>
        MAV_TYPE_GCS=6, 
    	///<summary> /* Airship, controlled | */ </summary>
        MAV_TYPE_AIRSHIP=7, 
    	///<summary> /* Free balloon, uncontrolled | */ </summary>
        MAV_TYPE_FREE_BALLOON=8, 
    	///<summary> /* Rocket | */ </summary>
        MAV_TYPE_ROCKET=9, 
    	///<summary> /* Ground rover | */ </summary>
        MAV_TYPE_GROUND_ROVER=10, 
    	///<summary> /* Surface vessel, boat, ship | */ </summary>
        MAV_TYPE_SURFACE_BOAT=11, 
    	///<summary> /* Submarine | */ </summary>
        MAV_TYPE_SUBMARINE=12, 
    	///<summary> /* Hexarotor | */ </summary>
        MAV_TYPE_HEXAROTOR=13, 
    	///<summary> /* Octorotor | */ </summary>
        MAV_TYPE_OCTOROTOR=14, 
    	///<summary> /* Octorotor | */ </summary>
        MAV_TYPE_TRICOPTER=15, 
    	///<summary> /* Flapping wing | */ </summary>
        MAV_TYPE_FLAPPING_WING=16, 
    	///<summary> /*  | */ </summary>
        MAV_TYPE_ENUM_END=17, 
    
    };

/** @brief  */
    public enum MAV_COMPONENT
    {
	///<summary> /*  | */ </summary>
        MAV_COMP_ID_ALL=0, 
    	///<summary> /*  | */ </summary>
        MAV_COMP_ID_GPS=220, 
    	///<summary> /*  | */ </summary>
        MAV_COMP_ID_MISSIONPLANNER=190, 
    	///<summary> /*  | */ </summary>
        MAV_COMP_ID_PATHPLANNER=195, 
    	///<summary> /*  | */ </summary>
        MAV_COMP_ID_MAPPER=180, 
    	///<summary> /*  | */ </summary>
        MAV_COMP_ID_CAMERA=100, 
    	///<summary> /*  | */ </summary>
        MAV_COMP_ID_IMU=200, 
    	///<summary> /*  | */ </summary>
        MAV_COMP_ID_IMU_2=201, 
    	///<summary> /*  | */ </summary>
        MAV_COMP_ID_IMU_3=202, 
    	///<summary> /*  | */ </summary>
        MAV_COMP_ID_UDP_BRIDGE=240, 
    	///<summary> /*  | */ </summary>
        MAV_COMP_ID_UART_BRIDGE=241, 
    	///<summary> /*  | */ </summary>
        MAV_COMP_ID_SYSTEM_CONTROL=250, 
    	///<summary> /*  | */ </summary>
        MAV_COMP_ID_SERVO1=140, 
    	///<summary> /*  | */ </summary>
        MAV_COMP_ID_SERVO2=141, 
    	///<summary> /*  | */ </summary>
        MAV_COMP_ID_SERVO3=142, 
    	///<summary> /*  | */ </summary>
        MAV_COMP_ID_SERVO4=143, 
    	///<summary> /*  | */ </summary>
        MAV_COMP_ID_SERVO5=144, 
    	///<summary> /*  | */ </summary>
        MAV_COMP_ID_SERVO6=145, 
    	///<summary> /*  | */ </summary>
        MAV_COMP_ID_SERVO7=146, 
    	///<summary> /*  | */ </summary>
        MAV_COMP_ID_SERVO8=147, 
    	///<summary> /*  | */ </summary>
        MAV_COMP_ID_SERVO9=148, 
    	///<summary> /*  | */ </summary>
        MAV_COMP_ID_SERVO10=149, 
    	///<summary> /*  | */ </summary>
        MAV_COMP_ID_SERVO11=150, 
    	///<summary> /*  | */ </summary>
        MAV_COMP_ID_SERVO12=151, 
    	///<summary> /*  | */ </summary>
        MAV_COMP_ID_SERVO13=152, 
    	///<summary> /*  | */ </summary>
        MAV_COMP_ID_SERVO14=153, 
    	///<summary> /*  | */ </summary>
        MAV_COMPONENT_ENUM_END=251, 
    
    };

/** @brief  */
    public enum MAV_FRAME
    {
	///<summary> /* Global coordinate frame, WGS84 coordinate system. First value / x: latitude, second value / y: longitude, third value / z: positive altitude over mean sea level (MSL) | */ </summary>
        MAV_FRAME_GLOBAL=0, 
    	///<summary> /* Local coordinate frame, Z-up (x: north, y: east, z: down). | */ </summary>
        MAV_FRAME_LOCAL_NED=1, 
    	///<summary> /* NOT a coordinate frame, indicates a mission command. | */ </summary>
        MAV_FRAME_MISSION=2, 
    	///<summary> /* Global coordinate frame, WGS84 coordinate system, relative altitude over ground with respect to the home position. First value / x: latitude, second value / y: longitude, third value / z: positive altitude with 0 being at the altitude of the home location. | */ </summary>
        MAV_FRAME_GLOBAL_RELATIVE_ALT=3, 
    	///<summary> /* Local coordinate frame, Z-down (x: east, y: north, z: up) | */ </summary>
        MAV_FRAME_LOCAL_ENU=4, 
    	///<summary> /*  | */ </summary>
        MAV_FRAME_ENUM_END=5, 
    
    };

/** @brief  */
    public enum MAVLINK_DATA_STREAM_TYPE
    {
	///<summary> /*  | */ </summary>
        MAVLINK_DATA_STREAM_IMG_JPEG=1, 
    	///<summary> /*  | */ </summary>
        MAVLINK_DATA_STREAM_IMG_BMP=2, 
    	///<summary> /*  | */ </summary>
        MAVLINK_DATA_STREAM_IMG_RAW8U=3, 
    	///<summary> /*  | */ </summary>
        MAVLINK_DATA_STREAM_IMG_RAW32U=4, 
    	///<summary> /*  | */ </summary>
        MAVLINK_DATA_STREAM_IMG_PGM=5, 
    	///<summary> /*  | */ </summary>
        MAVLINK_DATA_STREAM_IMG_PNG=6, 
    	///<summary> /*  | */ </summary>
        MAVLINK_DATA_STREAM_TYPE_ENUM_END=7, 
    
    };

/** @brief Commands to be executed by the MAV. They can be executed on user request,
      or as part of a mission script. If the action is used in a mission, the parameter mapping
      to the waypoint/mission message is as follows:
      Param 1, Param 2, Param 3, Param 4, X: Param 5, Y:Param 6, Z:Param 7. This command list is similar what
      ARINC 424 is for commercial aircraft: A data format how to interpret waypoint/mission data. */
    public enum MAV_CMD
    {
	///<summary> /* Navigate to MISSION. |Hold time in decimal seconds. (ignored by fixed wing, time to stay at MISSION for rotary wing)| Acceptance radius in meters (if the sphere with this radius is hit, the MISSION counts as reached)| 0 to pass through the WP, if > 0 radius in meters to pass by WP. Positive value for clockwise orbit, negative value for counter-clockwise orbit. Allows trajectory control.| Desired yaw angle at MISSION (rotary wing)| Latitude| Longitude| Altitude|  */ </summary>
        MAV_CMD_NAV_WAYPOINT=16, 
    	///<summary> /* Loiter around this MISSION an unlimited amount of time |Empty| Empty| Radius around MISSION, in meters. If positive loiter clockwise, else counter-clockwise| Desired yaw angle.| Latitude| Longitude| Altitude|  */ </summary>
        MAV_CMD_NAV_LOITER_UNLIM=17, 
    	///<summary> /* Loiter around this MISSION for X turns |Turns| Empty| Radius around MISSION, in meters. If positive loiter clockwise, else counter-clockwise| Desired yaw angle.| Latitude| Longitude| Altitude|  */ </summary>
        MAV_CMD_NAV_LOITER_TURNS=18, 
    	///<summary> /* Loiter around this MISSION for X seconds |Seconds (decimal)| Empty| Radius around MISSION, in meters. If positive loiter clockwise, else counter-clockwise| Desired yaw angle.| Latitude| Longitude| Altitude|  */ </summary>
        MAV_CMD_NAV_LOITER_TIME=19, 
    	///<summary> /* Return to launch location |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  */ </summary>
        MAV_CMD_NAV_RETURN_TO_LAUNCH=20, 
    	///<summary> /* Land at location |Empty| Empty| Empty| Desired yaw angle.| Latitude| Longitude| Altitude|  */ </summary>
        MAV_CMD_NAV_LAND=21, 
    	///<summary> /* Takeoff from ground / hand |Minimum pitch (if airspeed sensor present), desired pitch without sensor| Empty| Empty| Yaw angle (if magnetometer present), ignored without magnetometer| Latitude| Longitude| Altitude|  */ </summary>
        MAV_CMD_NAV_TAKEOFF=22, 
    	///<summary> /* Sets the region of interest (ROI) for a sensor set or the             vehicle itself. This can then be used by the vehicles control             system to control the vehicle attitude and the attitude of various             sensors such as cameras. |Region of intereset mode. (see MAV_ROI enum)| MISSION index/ target ID. (see MAV_ROI enum)| ROI index (allows a vehicle to manage multiple ROI's)| Empty| x the location of the fixed ROI (see MAV_FRAME)| y| z|  */ </summary> 
        MAV_CMD_NAV_ROI=80, 
    	///<summary> /* Control autonomous path planning on the MAV. |0: Disable local obstacle avoidance / local path planning (without resetting map), 1: Enable local path planning, 2: Enable and reset local path planning| 0: Disable full path planning (without resetting map), 1: Enable, 2: Enable and reset map/occupancy grid, 3: Enable and reset planned route, but not occupancy grid| Empty| Yaw angle at goal, in compass degrees, [0..360]| Latitude/X of goal| Longitude/Y of goal| Altitude/Z of goal|  */ </summary>
        MAV_CMD_NAV_PATHPLANNING=81, 
    	///<summary> /* NOP - This command is only used to mark the upper limit of the NAV/ACTION commands in the enumeration |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  */ </summary>
        MAV_CMD_NAV_LAST=95, 
    	///<summary> /* Delay mission state machine. |Delay in seconds (decimal)| Empty| Empty| Empty| Empty| Empty| Empty|  */ </summary>
        MAV_CMD_CONDITION_DELAY=112, 
    	///<summary> /* Ascend/descend at rate.  Delay mission state machine until desired altitude reached. |Descent / Ascend rate (m/s)| Empty| Empty| Empty| Empty| Empty| Finish Altitude|  */ </summary>
        MAV_CMD_CONDITION_CHANGE_ALT=113, 
    	///<summary> /* Delay mission state machine until within desired distance of next NAV point. |Distance (meters)| Empty| Empty| Empty| Empty| Empty| Empty|  */ </summary>
        MAV_CMD_CONDITION_DISTANCE=114, 
    	///<summary> /* Reach a certain target angle. |target angle: [0-360], 0 is north| speed during yaw change:[deg per second]| direction: negative: counter clockwise, positive: clockwise [-1,1]| relative offset or absolute angle: [ 1,0]| Empty| Empty| Empty|  */ </summary>
        MAV_CMD_CONDITION_YAW=115, 
    	///<summary> /* NOP - This command is only used to mark the upper limit of the CONDITION commands in the enumeration |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  */ </summary>
        MAV_CMD_CONDITION_LAST=159, 
    	///<summary> /* Set system mode. |Mode, as defined by ENUM MAV_MODE| Empty| Empty| Empty| Empty| Empty| Empty|  */ </summary>
        MAV_CMD_DO_SET_MODE=176, 
    	///<summary> /* Jump to the desired command in the mission list.  Repeat this action only the specified number of times |Sequence number| Repeat count| Empty| Empty| Empty| Empty| Empty|  */ </summary>
        MAV_CMD_DO_JUMP=177, 
    	///<summary> /* Change speed and/or throttle set points. |Speed type (0=Airspeed, 1=Ground Speed)| Speed  (m/s, -1 indicates no change)| Throttle  ( Percent, -1 indicates no change)| Empty| Empty| Empty| Empty|  */ </summary>
        MAV_CMD_DO_CHANGE_SPEED=178, 
    	///<summary> /* Changes the home location either to the current location or a specified location. |Use current (1=use current location, 0=use specified location)| Empty| Empty| Empty| Latitude| Longitude| Altitude|  */ </summary>
        MAV_CMD_DO_SET_HOME=179, 
    	///<summary> /* Set a system parameter.  Caution!  Use of this command requires knowledge of the numeric enumeration value of the parameter. |Parameter number| Parameter value| Empty| Empty| Empty| Empty| Empty|  */ </summary>
        MAV_CMD_DO_SET_PARAMETER=180, 
    	///<summary> /* Set a relay to a condition. |Relay number| Setting (1=on, 0=off, others possible depending on system hardware)| Empty| Empty| Empty| Empty| Empty|  */ </summary>
        MAV_CMD_DO_SET_RELAY=181, 
    	///<summary> /* Cycle a relay on and off for a desired number of cyles with a desired period. |Relay number| Cycle count| Cycle time (seconds, decimal)| Empty| Empty| Empty| Empty|  */ </summary>
        MAV_CMD_DO_REPEAT_RELAY=182, 
    	///<summary> /* Set a servo to a desired PWM value. |Servo number| PWM (microseconds, 1000 to 2000 typical)| Empty| Empty| Empty| Empty| Empty|  */ </summary>
        MAV_CMD_DO_SET_SERVO=183, 
    	///<summary> /* Cycle a between its nominal setting and a desired PWM for a desired number of cycles with a desired period. |Servo number| PWM (microseconds, 1000 to 2000 typical)| Cycle count| Cycle time (seconds)| Empty| Empty| Empty|  */ </summary>
        MAV_CMD_DO_REPEAT_SERVO=184, 
    	///<summary> /* Control onboard camera system. |Camera ID (-1 for all)| Transmission: 0: disabled, 1: enabled compressed, 2: enabled raw| Transmission mode: 0: video stream, >0: single images every n seconds (decimal)| Recording: 0: disabled, 1: enabled compressed, 2: enabled raw| Empty| Empty| Empty|  */ </summary>
        MAV_CMD_DO_CONTROL_VIDEO=200, 
    	///<summary> /* NOP - This command is only used to mark the upper limit of the DO commands in the enumeration |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  */ </summary>
        MAV_CMD_DO_LAST=240, 
    	///<summary> /* Trigger calibration. This command will be only accepted if in pre-flight mode. |Gyro calibration: 0: no, 1: yes| Magnetometer calibration: 0: no, 1: yes| Ground pressure: 0: no, 1: yes| Radio calibration: 0: no, 1: yes| Empty| Empty| Empty|  */ </summary>
        MAV_CMD_PREFLIGHT_CALIBRATION=241, 
    	///<summary> /* Set sensor offsets. This command will be only accepted if in pre-flight mode. |Sensor to adjust the offsets for: 0: gyros, 1: accelerometer, 2: magnetometer, 3: barometer, 4: optical flow| X axis offset (or generic dimension 1), in the sensor's raw units| Y axis offset (or generic dimension 2), in the sensor's raw units| Z axis offset (or generic dimension 3), in the sensor's raw units| Generic dimension 4, in the sensor's raw units| Generic dimension 5, in the sensor's raw units| Generic dimension 6, in the sensor's raw units|  */ </summary>
        MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS=242, 
    	///<summary> /* Request storage of different parameter values and logs. This command will be only accepted if in pre-flight mode. |Parameter storage: 0: READ FROM FLASH/EEPROM, 1: WRITE CURRENT TO FLASH/EEPROM| Mission storage: 0: READ FROM FLASH/EEPROM, 1: WRITE CURRENT TO FLASH/EEPROM| Reserved| Reserved| Empty| Empty| Empty|  */ </summary>
        MAV_CMD_PREFLIGHT_STORAGE=245, 
    	///<summary> /* Request the reboot or shutdown of system components. |0: Do nothing for autopilot, 1: Reboot autopilot, 2: Shutdown autopilot.| 0: Do nothing for onboard computer, 1: Reboot onboard computer, 2: Shutdown onboard computer.| Reserved| Reserved| Empty| Empty| Empty|  */ </summary>
        MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN=246, 
    	///<summary> /* Hold / continue the current action |MAV_GOTO_DO_HOLD: hold MAV_GOTO_DO_CONTINUE: continue with mission plan| MAV_GOTO_HOLD_AT_CURRENT_POSITION: Hold at current position MAV_GOTO_HOLD_AT_SPECIFIED_POSITION: hold at specified position| MAV_FRAME coordinate frame of hold point| Desired yaw angle in degrees| Latitude / X position| Longitude / Y position| Altitude / Z position|  */ </summary>
        MAV_CMD_OVERRIDE_GOTO=252, 
    	///<summary> /*  | */ </summary>
        MAV_CMD_ENUM_END=253, 
    
    };

/** @brief Data stream IDs. A data stream is not a fixed set of messages, but rather a
     recommendation to the autopilot software. Individual autopilots may or may not obey
     the recommended messages.
      */
    public enum MAV_DATA_STREAM
    {
	///<summary> /* Enable all data streams | */ </summary>
        MAV_DATA_STREAM_ALL=0, 
    	///<summary> /* Enable IMU_RAW, GPS_RAW, GPS_STATUS packets. | */ </summary>
        MAV_DATA_STREAM_RAW_SENSORS=1, 
    	///<summary> /* Enable GPS_STATUS, CONTROL_STATUS, AUX_STATUS | */ </summary>
        MAV_DATA_STREAM_EXTENDED_STATUS=2, 
    	///<summary> /* Enable RC_CHANNELS_SCALED, RC_CHANNELS_RAW, SERVO_OUTPUT_RAW | */ </summary>
        MAV_DATA_STREAM_RC_CHANNELS=3, 
    	///<summary> /* Enable ATTITUDE_CONTROLLER_OUTPUT, POSITION_CONTROLLER_OUTPUT, NAV_CONTROLLER_OUTPUT. | */ </summary>
        MAV_DATA_STREAM_RAW_CONTROLLER=4, 
    	///<summary> /* Enable LOCAL_POSITION, GLOBAL_POSITION/GLOBAL_POSITION_INT messages. | */ </summary>
        MAV_DATA_STREAM_POSITION=6, 
    	///<summary> /* Dependent on the autopilot | */ </summary>
        MAV_DATA_STREAM_EXTRA1=10, 
    	///<summary> /* Dependent on the autopilot | */ </summary>
        MAV_DATA_STREAM_EXTRA2=11, 
    	///<summary> /* Dependent on the autopilot | */ </summary>
        MAV_DATA_STREAM_EXTRA3=12, 
    	///<summary> /*  | */ </summary>
        MAV_DATA_STREAM_ENUM_END=13, 
    
    };

/** @brief  The ROI (region of interest) for the vehicle. This can be
                be used by the vehicle for camera/vehicle attitude alignment (see
                MAV_CMD_NAV_ROI).
             */
    public enum MAV_ROI
    {
	///<summary> /* No region of interest. | */ </summary>
        MAV_ROI_NONE=0, 
    	///<summary> /* Point toward next MISSION. | */ </summary>
        MAV_ROI_WPNEXT=1, 
    	///<summary> /* Point toward given MISSION. | */ </summary>
        MAV_ROI_WPINDEX=2, 
    	///<summary> /* Point toward fixed location. | */ </summary>
        MAV_ROI_LOCATION=3, 
    	///<summary> /* Point toward of given id. | */ </summary>
        MAV_ROI_TARGET=4, 
    	///<summary> /*  | */ </summary>
        MAV_ROI_ENUM_END=5, 
    
    };

/** @brief ACK / NACK / ERROR values as a result of MAV_CMDs and for mission item transmission. */
    public enum MAV_CMD_ACK
    {
	///<summary> /* Command / mission item is ok. | */ </summary>
        MAV_CMD_ACK_OK=1, 
    	///<summary> /* Generic error message if none of the other reasons fails or if no detailed error reporting is implemented. | */ </summary>
        MAV_CMD_ACK_ERR_FAIL=2, 
    	///<summary> /* The system is refusing to accept this command from this source / communication partner. | */ </summary>
        MAV_CMD_ACK_ERR_ACCESS_DENIED=3, 
    	///<summary> /* Command or mission item is not supported, other commands would be accepted. | */ </summary>
        MAV_CMD_ACK_ERR_NOT_SUPPORTED=4, 
    	///<summary> /* The coordinate frame of this command / mission item is not supported. | */ </summary>
        MAV_CMD_ACK_ERR_COORDINATE_FRAME_NOT_SUPPORTED=5, 
    	///<summary> /* The coordinate frame of this command is ok, but he coordinate values exceed the safety limits of this system. This is a generic error, please use the more specific error messages below if possible. | */ </summary>
        MAV_CMD_ACK_ERR_COORDINATES_OUT_OF_RANGE=6, 
    	///<summary> /* The X or latitude value is out of range. | */ </summary>
        MAV_CMD_ACK_ERR_X_LAT_OUT_OF_RANGE=7, 
    	///<summary> /* The Y or longitude value is out of range. | */ </summary>
        MAV_CMD_ACK_ERR_Y_LON_OUT_OF_RANGE=8, 
    	///<summary> /* The Z or altitude value is out of range. | */ </summary>
        MAV_CMD_ACK_ERR_Z_ALT_OUT_OF_RANGE=9, 
    	///<summary> /*  | */ </summary>
        MAV_CMD_ACK_ENUM_END=10, 
    
    };

/** @brief type of a mavlink parameter */
    public enum MAV_VAR
    {
	///<summary> /* 32 bit float | */ </summary>
        MAV_VAR_FLOAT=0, 
    	///<summary> /* 8 bit unsigned integer | */ </summary>
        MAV_VAR_UINT8=1, 
    	///<summary> /* 8 bit signed integer | */ </summary>
        MAV_VAR_INT8=2, 
    	///<summary> /* 16 bit unsigned integer | */ </summary>
        MAV_VAR_UINT16=3, 
    	///<summary> /* 16 bit signed integer | */ </summary>
        MAV_VAR_INT16=4, 
    	///<summary> /* 32 bit unsigned integer | */ </summary>
        MAV_VAR_UINT32=5, 
    	///<summary> /* 32 bit signed integer | */ </summary>
        MAV_VAR_INT32=6, 
    	///<summary> /*  | */ </summary>
        MAV_VAR_ENUM_END=7, 
    
    };

/** @brief result from a mavlink command */
    public enum MAV_RESULT
    {
	///<summary> /* Command ACCEPTED and EXECUTED | */ </summary>
        MAV_RESULT_ACCEPTED=0, 
    	///<summary> /* Command TEMPORARY REJECTED/DENIED | */ </summary>
        MAV_RESULT_TEMPORARILY_REJECTED=1, 
    	///<summary> /* Command PERMANENTLY DENIED | */ </summary>
        MAV_RESULT_DENIED=2, 
    	///<summary> /* Command UNKNOWN/UNSUPPORTED | */ </summary>
        MAV_RESULT_UNSUPPORTED=3, 
    	///<summary> /* Command executed, but failed | */ </summary>
        MAV_RESULT_FAILED=4, 
    	///<summary> /*  | */ </summary>
        MAV_RESULT_ENUM_END=5, 
    
    };

/** @brief result in a mavlink mission ack */
    public enum MAV_MISSION_RESULT
    {
	///<summary> /* mission accepted OK | */ </summary>
        MAV_MISSION_ACCEPTED=0, 
    	///<summary> /* generic error / not accepting mission commands at all right now | */ </summary>
        MAV_MISSION_ERROR=1, 
    	///<summary> /* coordinate frame is not supported | */ </summary>
        MAV_MISSION_UNSUPPORTED_FRAME=2, 
    	///<summary> /* command is not supported | */ </summary>
        MAV_MISSION_UNSUPPORTED=3, 
    	///<summary> /* mission item exceeds storage space | */ </summary>
        MAV_MISSION_NO_SPACE=4, 
    	///<summary> /* one of the parameters has an invalid value | */ </summary>
        MAV_MISSION_INVALID=5, 
    	///<summary> /* param1 has an invalid value | */ </summary>
        MAV_MISSION_INVALID_PARAM1=6, 
    	///<summary> /* param2 has an invalid value | */ </summary>
        MAV_MISSION_INVALID_PARAM2=7, 
    	///<summary> /* param3 has an invalid value | */ </summary>
        MAV_MISSION_INVALID_PARAM3=8, 
    	///<summary> /* param4 has an invalid value | */ </summary>
        MAV_MISSION_INVALID_PARAM4=9, 
    	///<summary> /* x/param5 has an invalid value | */ </summary>
        MAV_MISSION_INVALID_PARAM5_X=10, 
    	///<summary> /* y/param6 has an invalid value | */ </summary>
        MAV_MISSION_INVALID_PARAM6_Y=11, 
    	///<summary> /* param7 has an invalid value | */ </summary>
        MAV_MISSION_INVALID_PARAM7=12, 
    	///<summary> /* received waypoint out of sequence | */ </summary>
        MAV_MISSION_INVALID_SEQUENCE=13, 
    	///<summary> /* not accepting any mission commands from this communication partner | */ </summary>
        MAV_MISSION_DENIED=14, 
    	///<summary> /*  | */ </summary>
        MAV_MISSION_RESULT_ENUM_END=15, 
    
    };

}
