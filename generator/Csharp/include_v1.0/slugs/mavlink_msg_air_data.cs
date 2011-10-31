// MESSAGE AIR_DATA PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_AIR_DATA = 171;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_air_data_t
    {
        /// <summary>
        /// Dynamic pressure (Pa)
        /// </summary>
        public  Single dynamicPressure;
            /// <summary>
        /// Static pressure (Pa)
        /// </summary>
        public  Single staticPressure;
            /// <summary>
        /// Board temperature
        /// </summary>
        public  UInt16 temperature;
    
    };

/// <summary>
/// * @brief Pack a air_data message
/// * @param system_id ID of this system
/// * @param component_id ID of this component (e.g. 200 for IMU)
/// * @param msg The MAVLink message to compress the data into
/// *
/// * @param dynamicPressure Dynamic pressure (Pa)
/// * @param staticPressure Static pressure (Pa)
/// * @param temperature Board temperature
/// * @return length of the message in bytes (excluding serial stream start sign)
/// </summary>
 
public static UInt16 mavlink_msg_air_data_pack(byte system_id, byte component_id, byte[] msg,
                               Single dynamicPressure, Single staticPressure, UInt16 temperature)
{
if (MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS) {
	Array.Copy(BitConverter.GetBytes(dynamicPressure),0,msg,0,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(staticPressure),0,msg,4,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(temperature),0,msg,8,sizeof(UInt16));

} else {
    mavlink_air_data_t packet = new mavlink_air_data_t();
	packet.dynamicPressure = dynamicPressure;
	packet.staticPressure = staticPressure;
	packet.temperature = temperature;

        
        int len = 10;
        msg = new byte[len];
        IntPtr ptr = Marshal.AllocHGlobal(len);
        Marshal.StructureToPtr(packet, ptr, true);
        Marshal.Copy(ptr, msg, 0, len);
        Marshal.FreeHGlobal(ptr);
}

    //msg.msgid = MAVLINK_MSG_ID_AIR_DATA;
    //return mavlink_finalize_message(msg, system_id, component_id, 10, 232);
    return 0;
}

/**
 * @brief Pack a air_data message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param dynamicPressure Dynamic pressure (Pa)
 * @param staticPressure Static pressure (Pa)
 * @param temperature Board temperature
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_air_data_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   Single public dynamicPressure,Single public staticPressure,UInt16 public temperature)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[10];
	_mav_put_Single(buf, 0, dynamicPressure);
	_mav_put_Single(buf, 4, staticPressure);
	_mav_put_UInt16(buf, 8, temperature);

        memcpy(_MAV_PAYLOAD(msg), buf, 10);
#else
    mavlink_air_data_t packet;
	packet.dynamicPressure = dynamicPressure;
	packet.staticPressure = staticPressure;
	packet.temperature = temperature;

        memcpy(_MAV_PAYLOAD(msg), &packet, 10);
#endif

    msg->msgid = MAVLINK_MSG_ID_AIR_DATA;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 10, 232);
}
*/
/**
 * @brief Encode a air_data struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param air_data C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_air_data_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_air_data_t* air_data)
{
    return mavlink_msg_air_data_pack(system_id, component_id, msg, air_data->dynamicPressure, air_data->staticPressure, air_data->temperature);
}
*/
/**
 * @brief Send a air_data message
 * @param chan MAVLink channel to send the message
 *
 * @param dynamicPressure Dynamic pressure (Pa)
 * @param staticPressure Static pressure (Pa)
 * @param temperature Board temperature
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_air_data_send(mavlink_channel_t chan, Single public dynamicPressure, Single public staticPressure, UInt16 public temperature)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[10];
	_mav_put_Single(buf, 0, dynamicPressure);
	_mav_put_Single(buf, 4, staticPressure);
	_mav_put_UInt16(buf, 8, temperature);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AIR_DATA, buf, 10, 232);
#else
    mavlink_air_data_t packet;
	packet.dynamicPressure = dynamicPressure;
	packet.staticPressure = staticPressure;
	packet.temperature = temperature;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AIR_DATA, (const char *)&packet, 10, 232);
#endif
}

#endif
*/
// MESSAGE AIR_DATA UNPACKING


/**
 * @brief Get field dynamicPressure from air_data message
 *
 * @return Dynamic pressure (Pa)
 */
public static Single mavlink_msg_air_data_get_dynamicPressure(byte[] msg)
{
    return BitConverter.ToSingle(msg,  0);
}

/**
 * @brief Get field staticPressure from air_data message
 *
 * @return Static pressure (Pa)
 */
public static Single mavlink_msg_air_data_get_staticPressure(byte[] msg)
{
    return BitConverter.ToSingle(msg,  4);
}

/**
 * @brief Get field temperature from air_data message
 *
 * @return Board temperature
 */
public static UInt16 mavlink_msg_air_data_get_temperature(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  8);
}

/**
 * @brief Decode a air_data message into a struct
 *
 * @param msg The message to decode
 * @param air_data C-struct to decode the message contents into
 */
public static void mavlink_msg_air_data_decode(byte[] msg, ref mavlink_air_data_t air_data)
{
    if (MAVLINK_NEED_BYTE_SWAP) {
    	air_data.dynamicPressure = mavlink_msg_air_data_get_dynamicPressure(msg);
    	air_data.staticPressure = mavlink_msg_air_data_get_staticPressure(msg);
    	air_data.temperature = mavlink_msg_air_data_get_temperature(msg);
    
    } else {
        int len = 10; //Marshal.SizeOf(air_data);
        IntPtr i = Marshal.AllocHGlobal(len);
        Marshal.Copy(msg, 0, i, len);
        air_data = (mavlink_air_data_t)Marshal.PtrToStructure(i, ((object)air_data).GetType());
        Marshal.FreeHGlobal(i);
    }
}

}
