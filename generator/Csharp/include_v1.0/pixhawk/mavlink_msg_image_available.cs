// MESSAGE IMAGE_AVAILABLE PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_IMAGE_AVAILABLE = 154;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_image_available_t
    {
         public  UInt64 cam_id; /// Camera id
     public  UInt64 timestamp; /// Timestamp
     public  UInt64 valid_until; /// Until which timestamp this buffer will stay valid
     public  UInt32 img_seq; /// The image sequence number
     public  UInt32 img_buf_index; /// Position of the image in the buffer, starts with 0
     public  UInt32 key; /// Shared memory area key
     public  UInt32 exposure; /// Exposure time, in microseconds
     public  Single gain; /// Camera gain
     public  Single roll; /// Roll angle in rad
     public  Single pitch; /// Pitch angle in rad
     public  Single yaw; /// Yaw angle in rad
     public  Single local_z; /// Local frame Z coordinate (height over ground)
     public  Single lat; /// GPS X coordinate
     public  Single lon; /// GPS Y coordinate
     public  Single alt; /// Global frame altitude
     public  Single ground_x; /// Ground truth X
     public  Single ground_y; /// Ground truth Y
     public  Single ground_z; /// Ground truth Z
     public  UInt16 width; /// Image width
     public  UInt16 height; /// Image height
     public  UInt16 depth; /// Image depth
     public  byte cam_no; /// Camera # (starts with 0)
     public  byte channels; /// Image channels
    
    };

/**
 * @brief Pack a image_available message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param cam_id Camera id
 * @param cam_no Camera # (starts with 0)
 * @param timestamp Timestamp
 * @param valid_until Until which timestamp this buffer will stay valid
 * @param img_seq The image sequence number
 * @param img_buf_index Position of the image in the buffer, starts with 0
 * @param width Image width
 * @param height Image height
 * @param depth Image depth
 * @param channels Image channels
 * @param key Shared memory area key
 * @param exposure Exposure time, in microseconds
 * @param gain Camera gain
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
 
public static UInt16 mavlink_msg_image_available_pack(byte system_id, byte component_id, byte[] msg,
                               UInt64 cam_id, byte cam_no, UInt64 timestamp, UInt64 valid_until, UInt32 img_seq, UInt32 img_buf_index, UInt16 width, UInt16 height, UInt16 depth, byte channels, UInt32 key, UInt32 exposure, Single gain, Single roll, Single pitch, Single yaw, Single local_z, Single lat, Single lon, Single alt, Single ground_x, Single ground_y, Single ground_z)
{
if (MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS) {
	Array.Copy(BitConverter.GetBytes(cam_id),0,msg,0,sizeof(UInt64));
	Array.Copy(BitConverter.GetBytes(timestamp),0,msg,8,sizeof(UInt64));
	Array.Copy(BitConverter.GetBytes(valid_until),0,msg,16,sizeof(UInt64));
	Array.Copy(BitConverter.GetBytes(img_seq),0,msg,24,sizeof(UInt32));
	Array.Copy(BitConverter.GetBytes(img_buf_index),0,msg,28,sizeof(UInt32));
	Array.Copy(BitConverter.GetBytes(key),0,msg,32,sizeof(UInt32));
	Array.Copy(BitConverter.GetBytes(exposure),0,msg,36,sizeof(UInt32));
	Array.Copy(BitConverter.GetBytes(gain),0,msg,40,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(roll),0,msg,44,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(pitch),0,msg,48,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(yaw),0,msg,52,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(local_z),0,msg,56,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(lat),0,msg,60,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(lon),0,msg,64,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(alt),0,msg,68,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(ground_x),0,msg,72,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(ground_y),0,msg,76,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(ground_z),0,msg,80,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(width),0,msg,84,sizeof(UInt16));
	Array.Copy(BitConverter.GetBytes(height),0,msg,86,sizeof(UInt16));
	Array.Copy(BitConverter.GetBytes(depth),0,msg,88,sizeof(UInt16));
	Array.Copy(BitConverter.GetBytes(cam_no),0,msg,90,sizeof(byte));
	Array.Copy(BitConverter.GetBytes(channels),0,msg,91,sizeof(byte));

} else {
    mavlink_image_available_t packet = new mavlink_image_available_t();
	packet.cam_id = cam_id;
	packet.timestamp = timestamp;
	packet.valid_until = valid_until;
	packet.img_seq = img_seq;
	packet.img_buf_index = img_buf_index;
	packet.key = key;
	packet.exposure = exposure;
	packet.gain = gain;
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
	packet.width = width;
	packet.height = height;
	packet.depth = depth;
	packet.cam_no = cam_no;
	packet.channels = channels;

        
        int len = 92;
        msg = new byte[len];
        IntPtr ptr = Marshal.AllocHGlobal(len);
        Marshal.StructureToPtr(packet, ptr, true);
        Marshal.Copy(ptr, msg, 0, len);
        Marshal.FreeHGlobal(ptr);
}

    //msg.msgid = MAVLINK_MSG_ID_IMAGE_AVAILABLE;
    //return mavlink_finalize_message(msg, system_id, component_id, 92, 224);
    return 0;
}

/**
 * @brief Pack a image_available message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param cam_id Camera id
 * @param cam_no Camera # (starts with 0)
 * @param timestamp Timestamp
 * @param valid_until Until which timestamp this buffer will stay valid
 * @param img_seq The image sequence number
 * @param img_buf_index Position of the image in the buffer, starts with 0
 * @param width Image width
 * @param height Image height
 * @param depth Image depth
 * @param channels Image channels
 * @param key Shared memory area key
 * @param exposure Exposure time, in microseconds
 * @param gain Camera gain
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
static inline uint16_t mavlink_msg_image_available_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   UInt64 public cam_id,byte public cam_no,UInt64 public timestamp,UInt64 public valid_until,UInt32 public img_seq,UInt32 public img_buf_index,UInt16 public width,UInt16 public height,UInt16 public depth,byte public channels,UInt32 public key,UInt32 public exposure,Single public gain,Single public roll,Single public pitch,Single public yaw,Single public local_z,Single public lat,Single public lon,Single public alt,Single public ground_x,Single public ground_y,Single public ground_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[92];
	_mav_put_UInt64(buf, 0, cam_id);
	_mav_put_UInt64(buf, 8, timestamp);
	_mav_put_UInt64(buf, 16, valid_until);
	_mav_put_UInt32(buf, 24, img_seq);
	_mav_put_UInt32(buf, 28, img_buf_index);
	_mav_put_UInt32(buf, 32, key);
	_mav_put_UInt32(buf, 36, exposure);
	_mav_put_Single(buf, 40, gain);
	_mav_put_Single(buf, 44, roll);
	_mav_put_Single(buf, 48, pitch);
	_mav_put_Single(buf, 52, yaw);
	_mav_put_Single(buf, 56, local_z);
	_mav_put_Single(buf, 60, lat);
	_mav_put_Single(buf, 64, lon);
	_mav_put_Single(buf, 68, alt);
	_mav_put_Single(buf, 72, ground_x);
	_mav_put_Single(buf, 76, ground_y);
	_mav_put_Single(buf, 80, ground_z);
	_mav_put_UInt16(buf, 84, width);
	_mav_put_UInt16(buf, 86, height);
	_mav_put_UInt16(buf, 88, depth);
	_mav_put_byte(buf, 90, cam_no);
	_mav_put_byte(buf, 91, channels);

        memcpy(_MAV_PAYLOAD(msg), buf, 92);
#else
    mavlink_image_available_t packet;
	packet.cam_id = cam_id;
	packet.timestamp = timestamp;
	packet.valid_until = valid_until;
	packet.img_seq = img_seq;
	packet.img_buf_index = img_buf_index;
	packet.key = key;
	packet.exposure = exposure;
	packet.gain = gain;
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
	packet.width = width;
	packet.height = height;
	packet.depth = depth;
	packet.cam_no = cam_no;
	packet.channels = channels;

        memcpy(_MAV_PAYLOAD(msg), &packet, 92);
#endif

    msg->msgid = MAVLINK_MSG_ID_IMAGE_AVAILABLE;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 92, 224);
}
*/
/**
 * @brief Encode a image_available struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param image_available C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_image_available_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_image_available_t* image_available)
{
    return mavlink_msg_image_available_pack(system_id, component_id, msg, image_available->cam_id, image_available->cam_no, image_available->timestamp, image_available->valid_until, image_available->img_seq, image_available->img_buf_index, image_available->width, image_available->height, image_available->depth, image_available->channels, image_available->key, image_available->exposure, image_available->gain, image_available->roll, image_available->pitch, image_available->yaw, image_available->local_z, image_available->lat, image_available->lon, image_available->alt, image_available->ground_x, image_available->ground_y, image_available->ground_z);
}
*/
/**
 * @brief Send a image_available message
 * @param chan MAVLink channel to send the message
 *
 * @param cam_id Camera id
 * @param cam_no Camera # (starts with 0)
 * @param timestamp Timestamp
 * @param valid_until Until which timestamp this buffer will stay valid
 * @param img_seq The image sequence number
 * @param img_buf_index Position of the image in the buffer, starts with 0
 * @param width Image width
 * @param height Image height
 * @param depth Image depth
 * @param channels Image channels
 * @param key Shared memory area key
 * @param exposure Exposure time, in microseconds
 * @param gain Camera gain
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

static inline void mavlink_msg_image_available_send(mavlink_channel_t chan, UInt64 public cam_id, byte public cam_no, UInt64 public timestamp, UInt64 public valid_until, UInt32 public img_seq, UInt32 public img_buf_index, UInt16 public width, UInt16 public height, UInt16 public depth, byte public channels, UInt32 public key, UInt32 public exposure, Single public gain, Single public roll, Single public pitch, Single public yaw, Single public local_z, Single public lat, Single public lon, Single public alt, Single public ground_x, Single public ground_y, Single public ground_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[92];
	_mav_put_UInt64(buf, 0, cam_id);
	_mav_put_UInt64(buf, 8, timestamp);
	_mav_put_UInt64(buf, 16, valid_until);
	_mav_put_UInt32(buf, 24, img_seq);
	_mav_put_UInt32(buf, 28, img_buf_index);
	_mav_put_UInt32(buf, 32, key);
	_mav_put_UInt32(buf, 36, exposure);
	_mav_put_Single(buf, 40, gain);
	_mav_put_Single(buf, 44, roll);
	_mav_put_Single(buf, 48, pitch);
	_mav_put_Single(buf, 52, yaw);
	_mav_put_Single(buf, 56, local_z);
	_mav_put_Single(buf, 60, lat);
	_mav_put_Single(buf, 64, lon);
	_mav_put_Single(buf, 68, alt);
	_mav_put_Single(buf, 72, ground_x);
	_mav_put_Single(buf, 76, ground_y);
	_mav_put_Single(buf, 80, ground_z);
	_mav_put_UInt16(buf, 84, width);
	_mav_put_UInt16(buf, 86, height);
	_mav_put_UInt16(buf, 88, depth);
	_mav_put_byte(buf, 90, cam_no);
	_mav_put_byte(buf, 91, channels);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IMAGE_AVAILABLE, buf, 92, 224);
#else
    mavlink_image_available_t packet;
	packet.cam_id = cam_id;
	packet.timestamp = timestamp;
	packet.valid_until = valid_until;
	packet.img_seq = img_seq;
	packet.img_buf_index = img_buf_index;
	packet.key = key;
	packet.exposure = exposure;
	packet.gain = gain;
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
	packet.width = width;
	packet.height = height;
	packet.depth = depth;
	packet.cam_no = cam_no;
	packet.channels = channels;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IMAGE_AVAILABLE, (const char *)&packet, 92, 224);
#endif
}

#endif
*/
// MESSAGE IMAGE_AVAILABLE UNPACKING


/**
 * @brief Get field cam_id from image_available message
 *
 * @return Camera id
 */
public static UInt64 mavlink_msg_image_available_get_cam_id(byte[] msg)
{
    return BitConverter.ToUInt64(msg,  0);
}

/**
 * @brief Get field cam_no from image_available message
 *
 * @return Camera # (starts with 0)
 */
public static byte mavlink_msg_image_available_get_cam_no(byte[] msg)
{
    return getByte(msg,  90);
}

/**
 * @brief Get field timestamp from image_available message
 *
 * @return Timestamp
 */
public static UInt64 mavlink_msg_image_available_get_timestamp(byte[] msg)
{
    return BitConverter.ToUInt64(msg,  8);
}

/**
 * @brief Get field valid_until from image_available message
 *
 * @return Until which timestamp this buffer will stay valid
 */
public static UInt64 mavlink_msg_image_available_get_valid_until(byte[] msg)
{
    return BitConverter.ToUInt64(msg,  16);
}

/**
 * @brief Get field img_seq from image_available message
 *
 * @return The image sequence number
 */
public static UInt32 mavlink_msg_image_available_get_img_seq(byte[] msg)
{
    return BitConverter.ToUInt32(msg,  24);
}

/**
 * @brief Get field img_buf_index from image_available message
 *
 * @return Position of the image in the buffer, starts with 0
 */
public static UInt32 mavlink_msg_image_available_get_img_buf_index(byte[] msg)
{
    return BitConverter.ToUInt32(msg,  28);
}

/**
 * @brief Get field width from image_available message
 *
 * @return Image width
 */
public static UInt16 mavlink_msg_image_available_get_width(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  84);
}

/**
 * @brief Get field height from image_available message
 *
 * @return Image height
 */
public static UInt16 mavlink_msg_image_available_get_height(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  86);
}

/**
 * @brief Get field depth from image_available message
 *
 * @return Image depth
 */
public static UInt16 mavlink_msg_image_available_get_depth(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  88);
}

/**
 * @brief Get field channels from image_available message
 *
 * @return Image channels
 */
public static byte mavlink_msg_image_available_get_channels(byte[] msg)
{
    return getByte(msg,  91);
}

/**
 * @brief Get field key from image_available message
 *
 * @return Shared memory area key
 */
public static UInt32 mavlink_msg_image_available_get_key(byte[] msg)
{
    return BitConverter.ToUInt32(msg,  32);
}

/**
 * @brief Get field exposure from image_available message
 *
 * @return Exposure time, in microseconds
 */
public static UInt32 mavlink_msg_image_available_get_exposure(byte[] msg)
{
    return BitConverter.ToUInt32(msg,  36);
}

/**
 * @brief Get field gain from image_available message
 *
 * @return Camera gain
 */
public static Single mavlink_msg_image_available_get_gain(byte[] msg)
{
    return BitConverter.ToSingle(msg,  40);
}

/**
 * @brief Get field roll from image_available message
 *
 * @return Roll angle in rad
 */
public static Single mavlink_msg_image_available_get_roll(byte[] msg)
{
    return BitConverter.ToSingle(msg,  44);
}

/**
 * @brief Get field pitch from image_available message
 *
 * @return Pitch angle in rad
 */
public static Single mavlink_msg_image_available_get_pitch(byte[] msg)
{
    return BitConverter.ToSingle(msg,  48);
}

/**
 * @brief Get field yaw from image_available message
 *
 * @return Yaw angle in rad
 */
public static Single mavlink_msg_image_available_get_yaw(byte[] msg)
{
    return BitConverter.ToSingle(msg,  52);
}

/**
 * @brief Get field local_z from image_available message
 *
 * @return Local frame Z coordinate (height over ground)
 */
public static Single mavlink_msg_image_available_get_local_z(byte[] msg)
{
    return BitConverter.ToSingle(msg,  56);
}

/**
 * @brief Get field lat from image_available message
 *
 * @return GPS X coordinate
 */
public static Single mavlink_msg_image_available_get_lat(byte[] msg)
{
    return BitConverter.ToSingle(msg,  60);
}

/**
 * @brief Get field lon from image_available message
 *
 * @return GPS Y coordinate
 */
public static Single mavlink_msg_image_available_get_lon(byte[] msg)
{
    return BitConverter.ToSingle(msg,  64);
}

/**
 * @brief Get field alt from image_available message
 *
 * @return Global frame altitude
 */
public static Single mavlink_msg_image_available_get_alt(byte[] msg)
{
    return BitConverter.ToSingle(msg,  68);
}

/**
 * @brief Get field ground_x from image_available message
 *
 * @return Ground truth X
 */
public static Single mavlink_msg_image_available_get_ground_x(byte[] msg)
{
    return BitConverter.ToSingle(msg,  72);
}

/**
 * @brief Get field ground_y from image_available message
 *
 * @return Ground truth Y
 */
public static Single mavlink_msg_image_available_get_ground_y(byte[] msg)
{
    return BitConverter.ToSingle(msg,  76);
}

/**
 * @brief Get field ground_z from image_available message
 *
 * @return Ground truth Z
 */
public static Single mavlink_msg_image_available_get_ground_z(byte[] msg)
{
    return BitConverter.ToSingle(msg,  80);
}

/**
 * @brief Decode a image_available message into a struct
 *
 * @param msg The message to decode
 * @param image_available C-struct to decode the message contents into
 */
public static void mavlink_msg_image_available_decode(byte[] msg, ref mavlink_image_available_t image_available)
{
    if (MAVLINK_NEED_BYTE_SWAP) {
    	image_available.cam_id = mavlink_msg_image_available_get_cam_id(msg);
    	image_available.timestamp = mavlink_msg_image_available_get_timestamp(msg);
    	image_available.valid_until = mavlink_msg_image_available_get_valid_until(msg);
    	image_available.img_seq = mavlink_msg_image_available_get_img_seq(msg);
    	image_available.img_buf_index = mavlink_msg_image_available_get_img_buf_index(msg);
    	image_available.key = mavlink_msg_image_available_get_key(msg);
    	image_available.exposure = mavlink_msg_image_available_get_exposure(msg);
    	image_available.gain = mavlink_msg_image_available_get_gain(msg);
    	image_available.roll = mavlink_msg_image_available_get_roll(msg);
    	image_available.pitch = mavlink_msg_image_available_get_pitch(msg);
    	image_available.yaw = mavlink_msg_image_available_get_yaw(msg);
    	image_available.local_z = mavlink_msg_image_available_get_local_z(msg);
    	image_available.lat = mavlink_msg_image_available_get_lat(msg);
    	image_available.lon = mavlink_msg_image_available_get_lon(msg);
    	image_available.alt = mavlink_msg_image_available_get_alt(msg);
    	image_available.ground_x = mavlink_msg_image_available_get_ground_x(msg);
    	image_available.ground_y = mavlink_msg_image_available_get_ground_y(msg);
    	image_available.ground_z = mavlink_msg_image_available_get_ground_z(msg);
    	image_available.width = mavlink_msg_image_available_get_width(msg);
    	image_available.height = mavlink_msg_image_available_get_height(msg);
    	image_available.depth = mavlink_msg_image_available_get_depth(msg);
    	image_available.cam_no = mavlink_msg_image_available_get_cam_no(msg);
    	image_available.channels = mavlink_msg_image_available_get_channels(msg);
    
    } else {
        int len = 92; //Marshal.SizeOf(image_available);
        IntPtr i = Marshal.AllocHGlobal(len);
        Marshal.Copy(msg, 0, i, len);
        image_available = (mavlink_image_available_t)Marshal.PtrToStructure(i, ((object)image_available).GetType());
        Marshal.FreeHGlobal(i);
    }
}

}
