// MESSAGE IMAGE_TRIGGERED PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_IMAGE_TRIGGERED = 152;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_image_triggered_t
    {
        /// <summary>
        /// Timestamp
        /// </summary>
        public  UInt64 timestamp;
            /// <summary>
        /// IMU seq
        /// </summary>
        public  UInt32 seq;
            /// <summary>
        /// Roll angle in rad
        /// </summary>
        public  Single roll;
            /// <summary>
        /// Pitch angle in rad
        /// </summary>
        public  Single pitch;
            /// <summary>
        /// Yaw angle in rad
        /// </summary>
        public  Single yaw;
            /// <summary>
        /// Local frame Z coordinate (height over ground)
        /// </summary>
        public  Single local_z;
            /// <summary>
        /// GPS X coordinate
        /// </summary>
        public  Single lat;
            /// <summary>
        /// GPS Y coordinate
        /// </summary>
        public  Single lon;
            /// <summary>
        /// Global frame altitude
        /// </summary>
        public  Single alt;
            /// <summary>
        /// Ground truth X
        /// </summary>
        public  Single ground_x;
            /// <summary>
        /// Ground truth Y
        /// </summary>
        public  Single ground_y;
            /// <summary>
        /// Ground truth Z
        /// </summary>
        public  Single ground_z;
    
    };

/// <summary>
/// * @brief Pack a image_triggered message
/// * @param system_id ID of this system
/// * @param component_id ID of this component (e.g. 200 for IMU)
/// * @param msg The MAVLink message to compress the data into
/// *
/// * @param timestamp Timestamp
/// * @param seq IMU seq
/// * @param roll Roll angle in rad
/// * @param pitch Pitch angle in rad
/// * @param yaw Yaw angle in rad
/// * @param local_z Local frame Z coordinate (height over ground)
/// * @param lat GPS X coordinate
/// * @param lon GPS Y coordinate
/// * @param alt Global frame altitude
/// * @param ground_x Ground truth X
/// * @param ground_y Ground truth Y
/// * @param ground_z Ground truth Z
/// * @return length of the message in bytes (excluding serial stream start sign)
/// </summary>
 
public static UInt16 mavlink_msg_image_triggered_pack(byte system_id, byte component_id, byte[] msg,
                               UInt64 timestamp, UInt32 seq, Single roll, Single pitch, Single yaw, Single local_z, Single lat, Single lon, Single alt, Single ground_x, Single ground_y, Single ground_z)
{
if (MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS) {
	Array.Copy(BitConverter.GetBytes(timestamp),0,msg,0,sizeof(UInt64));
	Array.Copy(BitConverter.GetBytes(seq),0,msg,8,sizeof(UInt32));
	Array.Copy(BitConverter.GetBytes(roll),0,msg,12,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(pitch),0,msg,16,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(yaw),0,msg,20,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(local_z),0,msg,24,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(lat),0,msg,28,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(lon),0,msg,32,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(alt),0,msg,36,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(ground_x),0,msg,40,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(ground_y),0,msg,44,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(ground_z),0,msg,48,sizeof(Single));

} else {
    mavlink_image_triggered_t packet = new mavlink_image_triggered_t();
	packet.timestamp = timestamp;
	packet.seq = seq;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;
	packet.local_z = local_z;
	packet.lat = lat;
	packet.lon = lon;
	packet.alt = alt;
	packet.ground_x = ground_x;
	packet.ground_y = ground_y;
	packet.ground_z = ground_z;

        
        int len = 52;
        msg = new byte[len];
        IntPtr ptr = Marshal.AllocHGlobal(len);
        Marshal.StructureToPtr(packet, ptr, true);
        Marshal.Copy(ptr, msg, 0, len);
        Marshal.FreeHGlobal(ptr);
}

    //msg.msgid = MAVLINK_MSG_ID_IMAGE_TRIGGERED;
    //return mavlink_finalize_message(msg, system_id, component_id, 52);
    return 0;
}

/**
 * @brief Pack a image_triggered message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp Timestamp
 * @param seq IMU seq
 * @param roll Roll angle in rad
 * @param pitch Pitch angle in rad
 * @param yaw Yaw angle in rad
 * @param local_z Local frame Z coordinate (height over ground)
 * @param lat GPS X coordinate
 * @param lon GPS Y coordinate
 * @param alt Global frame altitude
 * @param ground_x Ground truth X
 * @param ground_y Ground truth Y
 * @param ground_z Ground truth Z
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_image_triggered_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   UInt64 public timestamp,UInt32 public seq,Single public roll,Single public pitch,Single public yaw,Single public local_z,Single public lat,Single public lon,Single public alt,Single public ground_x,Single public ground_y,Single public ground_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[52];
	_mav_put_UInt64(buf, 0, timestamp);
	_mav_put_UInt32(buf, 8, seq);
	_mav_put_Single(buf, 12, roll);
	_mav_put_Single(buf, 16, pitch);
	_mav_put_Single(buf, 20, yaw);
	_mav_put_Single(buf, 24, local_z);
	_mav_put_Single(buf, 28, lat);
	_mav_put_Single(buf, 32, lon);
	_mav_put_Single(buf, 36, alt);
	_mav_put_Single(buf, 40, ground_x);
	_mav_put_Single(buf, 44, ground_y);
	_mav_put_Single(buf, 48, ground_z);

        memcpy(_MAV_PAYLOAD(msg), buf, 52);
#else
    mavlink_image_triggered_t packet;
	packet.timestamp = timestamp;
	packet.seq = seq;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;
	packet.local_z = local_z;
	packet.lat = lat;
	packet.lon = lon;
	packet.alt = alt;
	packet.ground_x = ground_x;
	packet.ground_y = ground_y;
	packet.ground_z = ground_z;

        memcpy(_MAV_PAYLOAD(msg), &packet, 52);
#endif

    msg->msgid = MAVLINK_MSG_ID_IMAGE_TRIGGERED;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 52);
}
*/
/**
 * @brief Encode a image_triggered struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param image_triggered C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_image_triggered_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_image_triggered_t* image_triggered)
{
    return mavlink_msg_image_triggered_pack(system_id, component_id, msg, image_triggered->timestamp, image_triggered->seq, image_triggered->roll, image_triggered->pitch, image_triggered->yaw, image_triggered->local_z, image_triggered->lat, image_triggered->lon, image_triggered->alt, image_triggered->ground_x, image_triggered->ground_y, image_triggered->ground_z);
}
*/
/**
 * @brief Send a image_triggered message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp Timestamp
 * @param seq IMU seq
 * @param roll Roll angle in rad
 * @param pitch Pitch angle in rad
 * @param yaw Yaw angle in rad
 * @param local_z Local frame Z coordinate (height over ground)
 * @param lat GPS X coordinate
 * @param lon GPS Y coordinate
 * @param alt Global frame altitude
 * @param ground_x Ground truth X
 * @param ground_y Ground truth Y
 * @param ground_z Ground truth Z
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_image_triggered_send(mavlink_channel_t chan, UInt64 public timestamp, UInt32 public seq, Single public roll, Single public pitch, Single public yaw, Single public local_z, Single public lat, Single public lon, Single public alt, Single public ground_x, Single public ground_y, Single public ground_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[52];
	_mav_put_UInt64(buf, 0, timestamp);
	_mav_put_UInt32(buf, 8, seq);
	_mav_put_Single(buf, 12, roll);
	_mav_put_Single(buf, 16, pitch);
	_mav_put_Single(buf, 20, yaw);
	_mav_put_Single(buf, 24, local_z);
	_mav_put_Single(buf, 28, lat);
	_mav_put_Single(buf, 32, lon);
	_mav_put_Single(buf, 36, alt);
	_mav_put_Single(buf, 40, ground_x);
	_mav_put_Single(buf, 44, ground_y);
	_mav_put_Single(buf, 48, ground_z);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IMAGE_TRIGGERED, buf, 52);
#else
    mavlink_image_triggered_t packet;
	packet.timestamp = timestamp;
	packet.seq = seq;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;
	packet.local_z = local_z;
	packet.lat = lat;
	packet.lon = lon;
	packet.alt = alt;
	packet.ground_x = ground_x;
	packet.ground_y = ground_y;
	packet.ground_z = ground_z;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IMAGE_TRIGGERED, (const char *)&packet, 52);
#endif
}

#endif
*/
// MESSAGE IMAGE_TRIGGERED UNPACKING


/**
 * @brief Get field timestamp from image_triggered message
 *
 * @return Timestamp
 */
public static UInt64 mavlink_msg_image_triggered_get_timestamp(byte[] msg)
{
    return BitConverter.ToUInt64(msg,  0);
}

/**
 * @brief Get field seq from image_triggered message
 *
 * @return IMU seq
 */
public static UInt32 mavlink_msg_image_triggered_get_seq(byte[] msg)
{
    return BitConverter.ToUInt32(msg,  8);
}

/**
 * @brief Get field roll from image_triggered message
 *
 * @return Roll angle in rad
 */
public static Single mavlink_msg_image_triggered_get_roll(byte[] msg)
{
    return BitConverter.ToSingle(msg,  12);
}

/**
 * @brief Get field pitch from image_triggered message
 *
 * @return Pitch angle in rad
 */
public static Single mavlink_msg_image_triggered_get_pitch(byte[] msg)
{
    return BitConverter.ToSingle(msg,  16);
}

/**
 * @brief Get field yaw from image_triggered message
 *
 * @return Yaw angle in rad
 */
public static Single mavlink_msg_image_triggered_get_yaw(byte[] msg)
{
    return BitConverter.ToSingle(msg,  20);
}

/**
 * @brief Get field local_z from image_triggered message
 *
 * @return Local frame Z coordinate (height over ground)
 */
public static Single mavlink_msg_image_triggered_get_local_z(byte[] msg)
{
    return BitConverter.ToSingle(msg,  24);
}

/**
 * @brief Get field lat from image_triggered message
 *
 * @return GPS X coordinate
 */
public static Single mavlink_msg_image_triggered_get_lat(byte[] msg)
{
    return BitConverter.ToSingle(msg,  28);
}

/**
 * @brief Get field lon from image_triggered message
 *
 * @return GPS Y coordinate
 */
public static Single mavlink_msg_image_triggered_get_lon(byte[] msg)
{
    return BitConverter.ToSingle(msg,  32);
}

/**
 * @brief Get field alt from image_triggered message
 *
 * @return Global frame altitude
 */
public static Single mavlink_msg_image_triggered_get_alt(byte[] msg)
{
    return BitConverter.ToSingle(msg,  36);
}

/**
 * @brief Get field ground_x from image_triggered message
 *
 * @return Ground truth X
 */
public static Single mavlink_msg_image_triggered_get_ground_x(byte[] msg)
{
    return BitConverter.ToSingle(msg,  40);
}

/**
 * @brief Get field ground_y from image_triggered message
 *
 * @return Ground truth Y
 */
public static Single mavlink_msg_image_triggered_get_ground_y(byte[] msg)
{
    return BitConverter.ToSingle(msg,  44);
}

/**
 * @brief Get field ground_z from image_triggered message
 *
 * @return Ground truth Z
 */
public static Single mavlink_msg_image_triggered_get_ground_z(byte[] msg)
{
    return BitConverter.ToSingle(msg,  48);
}

/**
 * @brief Decode a image_triggered message into a struct
 *
 * @param msg The message to decode
 * @param image_triggered C-struct to decode the message contents into
 */
public static void mavlink_msg_image_triggered_decode(byte[] msg, ref mavlink_image_triggered_t image_triggered)
{
    if (MAVLINK_NEED_BYTE_SWAP) {
    	image_triggered.timestamp = mavlink_msg_image_triggered_get_timestamp(msg);
    	image_triggered.seq = mavlink_msg_image_triggered_get_seq(msg);
    	image_triggered.roll = mavlink_msg_image_triggered_get_roll(msg);
    	image_triggered.pitch = mavlink_msg_image_triggered_get_pitch(msg);
    	image_triggered.yaw = mavlink_msg_image_triggered_get_yaw(msg);
    	image_triggered.local_z = mavlink_msg_image_triggered_get_local_z(msg);
    	image_triggered.lat = mavlink_msg_image_triggered_get_lat(msg);
    	image_triggered.lon = mavlink_msg_image_triggered_get_lon(msg);
    	image_triggered.alt = mavlink_msg_image_triggered_get_alt(msg);
    	image_triggered.ground_x = mavlink_msg_image_triggered_get_ground_x(msg);
    	image_triggered.ground_y = mavlink_msg_image_triggered_get_ground_y(msg);
    	image_triggered.ground_z = mavlink_msg_image_triggered_get_ground_z(msg);
    
    } else {
        int len = 52; //Marshal.SizeOf(image_triggered);
        IntPtr i = Marshal.AllocHGlobal(len);
        Marshal.Copy(msg, 0, i, len);
        image_triggered = (mavlink_image_triggered_t)Marshal.PtrToStructure(i, ((object)image_triggered).GetType());
        Marshal.FreeHGlobal(i);
    }
}

}
