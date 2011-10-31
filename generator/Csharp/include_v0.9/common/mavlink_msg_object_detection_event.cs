// MESSAGE OBJECT_DETECTION_EVENT PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_OBJECT_DETECTION_EVENT = 140;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_object_detection_event_t
    {
        /// <summary>
        /// Timestamp in milliseconds since system boot
        /// </summary>
        public  UInt32 time;
            /// <summary>
        /// Object ID
        /// </summary>
        public  UInt16 object_id;
            /// <summary>
        /// Object type: 0: image, 1: letter, 2: ground vehicle, 3: air vehicle, 4: surface vehicle, 5: sub-surface vehicle, 6: human, 7: animal
        /// </summary>
        public  byte type;
            /// <summary>
        /// Name of the object as defined by the detector
        /// </summary>
        [MarshalAs(UnmanagedType.ByValArray,SizeConst=20)]
 public string name;
            /// <summary>
        /// Detection quality / confidence. 0: bad, 255: maximum confidence
        /// </summary>
        public  byte quality;
            /// <summary>
        /// Angle of the object with respect to the body frame in NED coordinates in radians. 0: front
        /// </summary>
        public  Single bearing;
            /// <summary>
        /// Ground distance in meters
        /// </summary>
        public  Single distance;
    
    };

/// <summary>
/// * @brief Pack a object_detection_event message
/// * @param system_id ID of this system
/// * @param component_id ID of this component (e.g. 200 for IMU)
/// * @param msg The MAVLink message to compress the data into
/// *
/// * @param time Timestamp in milliseconds since system boot
/// * @param object_id Object ID
/// * @param type Object type: 0: image, 1: letter, 2: ground vehicle, 3: air vehicle, 4: surface vehicle, 5: sub-surface vehicle, 6: human, 7: animal
/// * @param name Name of the object as defined by the detector
/// * @param quality Detection quality / confidence. 0: bad, 255: maximum confidence
/// * @param bearing Angle of the object with respect to the body frame in NED coordinates in radians. 0: front
/// * @param distance Ground distance in meters
/// * @return length of the message in bytes (excluding serial stream start sign)
/// </summary>
 
public static UInt16 mavlink_msg_object_detection_event_pack(byte system_id, byte component_id, byte[] msg,
                               UInt32 time, UInt16 object_id, byte type, string name, byte quality, Single bearing, Single distance)
{
if (MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS) {
	Array.Copy(BitConverter.GetBytes(time),0,msg,0,sizeof(UInt32));
	Array.Copy(BitConverter.GetBytes(object_id),0,msg,4,sizeof(UInt16));
	Array.Copy(BitConverter.GetBytes(type),0,msg,6,sizeof(byte));
	Array.Copy(BitConverter.GetBytes(quality),0,msg,27,sizeof(byte));
	Array.Copy(BitConverter.GetBytes(bearing),0,msg,28,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(distance),0,msg,32,sizeof(Single));
	Array.Copy(toArray(name),0,msg,7,20);
} else {
    mavlink_object_detection_event_t packet = new mavlink_object_detection_event_t();
	packet.time = time;
	packet.object_id = object_id;
	packet.type = type;
	packet.quality = quality;
	packet.bearing = bearing;
	packet.distance = distance;
	packet.name = name;
        
        int len = 36;
        msg = new byte[len];
        IntPtr ptr = Marshal.AllocHGlobal(len);
        Marshal.StructureToPtr(packet, ptr, true);
        Marshal.Copy(ptr, msg, 0, len);
        Marshal.FreeHGlobal(ptr);
}

    //msg.msgid = MAVLINK_MSG_ID_OBJECT_DETECTION_EVENT;
    //return mavlink_finalize_message(msg, system_id, component_id, 36);
    return 0;
}

/**
 * @brief Pack a object_detection_event message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param time Timestamp in milliseconds since system boot
 * @param object_id Object ID
 * @param type Object type: 0: image, 1: letter, 2: ground vehicle, 3: air vehicle, 4: surface vehicle, 5: sub-surface vehicle, 6: human, 7: animal
 * @param name Name of the object as defined by the detector
 * @param quality Detection quality / confidence. 0: bad, 255: maximum confidence
 * @param bearing Angle of the object with respect to the body frame in NED coordinates in radians. 0: front
 * @param distance Ground distance in meters
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_object_detection_event_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   UInt32 public time,UInt16 public object_id,byte public type,const string [MarshalAs(UnmanagedType.ByValArray,SizeConst=20)]
 publicname,byte public quality,Single public bearing,Single public distance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[36];
	_mav_put_UInt32(buf, 0, time);
	_mav_put_UInt16(buf, 4, object_id);
	_mav_put_byte(buf, 6, type);
	_mav_put_byte(buf, 27, quality);
	_mav_put_Single(buf, 28, bearing);
	_mav_put_Single(buf, 32, distance);
	_mav_put_string_array(buf, 7, name, 20);
        memcpy(_MAV_PAYLOAD(msg), buf, 36);
#else
    mavlink_object_detection_event_t packet;
	packet.time = time;
	packet.object_id = object_id;
	packet.type = type;
	packet.quality = quality;
	packet.bearing = bearing;
	packet.distance = distance;
	memcpy(packet.name, name, sizeof(string)*20);
        memcpy(_MAV_PAYLOAD(msg), &packet, 36);
#endif

    msg->msgid = MAVLINK_MSG_ID_OBJECT_DETECTION_EVENT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 36);
}
*/
/**
 * @brief Encode a object_detection_event struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param object_detection_event C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_object_detection_event_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_object_detection_event_t* object_detection_event)
{
    return mavlink_msg_object_detection_event_pack(system_id, component_id, msg, object_detection_event->time, object_detection_event->object_id, object_detection_event->type, object_detection_event->name, object_detection_event->quality, object_detection_event->bearing, object_detection_event->distance);
}
*/
/**
 * @brief Send a object_detection_event message
 * @param chan MAVLink channel to send the message
 *
 * @param time Timestamp in milliseconds since system boot
 * @param object_id Object ID
 * @param type Object type: 0: image, 1: letter, 2: ground vehicle, 3: air vehicle, 4: surface vehicle, 5: sub-surface vehicle, 6: human, 7: animal
 * @param name Name of the object as defined by the detector
 * @param quality Detection quality / confidence. 0: bad, 255: maximum confidence
 * @param bearing Angle of the object with respect to the body frame in NED coordinates in radians. 0: front
 * @param distance Ground distance in meters
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_object_detection_event_send(mavlink_channel_t chan, UInt32 public time, UInt16 public object_id, byte public type, const string [MarshalAs(UnmanagedType.ByValArray,SizeConst=20)]
 publicname, byte public quality, Single public bearing, Single public distance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[36];
	_mav_put_UInt32(buf, 0, time);
	_mav_put_UInt16(buf, 4, object_id);
	_mav_put_byte(buf, 6, type);
	_mav_put_byte(buf, 27, quality);
	_mav_put_Single(buf, 28, bearing);
	_mav_put_Single(buf, 32, distance);
	_mav_put_string_array(buf, 7, name, 20);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OBJECT_DETECTION_EVENT, buf, 36);
#else
    mavlink_object_detection_event_t packet;
	packet.time = time;
	packet.object_id = object_id;
	packet.type = type;
	packet.quality = quality;
	packet.bearing = bearing;
	packet.distance = distance;
	memcpy(packet.name, name, sizeof(string)*20);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OBJECT_DETECTION_EVENT, (const char *)&packet, 36);
#endif
}

#endif
*/
// MESSAGE OBJECT_DETECTION_EVENT UNPACKING


/**
 * @brief Get field time from object_detection_event message
 *
 * @return Timestamp in milliseconds since system boot
 */
public static UInt32 mavlink_msg_object_detection_event_get_time(byte[] msg)
{
    return BitConverter.ToUInt32(msg,  0);
}

/**
 * @brief Get field object_id from object_detection_event message
 *
 * @return Object ID
 */
public static UInt16 mavlink_msg_object_detection_event_get_object_id(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  4);
}

/**
 * @brief Get field type from object_detection_event message
 *
 * @return Object type: 0: image, 1: letter, 2: ground vehicle, 3: air vehicle, 4: surface vehicle, 5: sub-surface vehicle, 6: human, 7: animal
 */
public static byte mavlink_msg_object_detection_event_get_type(byte[] msg)
{
    return getByte(msg,  6);
}

/**
 * @brief Get field name from object_detection_event message
 *
 * @return Name of the object as defined by the detector
 */
public static string mavlink_msg_object_detection_event_get_name(byte[] msg)
{
    return System.Text.ASCIIEncoding.ASCII.GetString(msg,7,20); //(msg, 20,  7);
}

/**
 * @brief Get field quality from object_detection_event message
 *
 * @return Detection quality / confidence. 0: bad, 255: maximum confidence
 */
public static byte mavlink_msg_object_detection_event_get_quality(byte[] msg)
{
    return getByte(msg,  27);
}

/**
 * @brief Get field bearing from object_detection_event message
 *
 * @return Angle of the object with respect to the body frame in NED coordinates in radians. 0: front
 */
public static Single mavlink_msg_object_detection_event_get_bearing(byte[] msg)
{
    return BitConverter.ToSingle(msg,  28);
}

/**
 * @brief Get field distance from object_detection_event message
 *
 * @return Ground distance in meters
 */
public static Single mavlink_msg_object_detection_event_get_distance(byte[] msg)
{
    return BitConverter.ToSingle(msg,  32);
}

/**
 * @brief Decode a object_detection_event message into a struct
 *
 * @param msg The message to decode
 * @param object_detection_event C-struct to decode the message contents into
 */
public static void mavlink_msg_object_detection_event_decode(byte[] msg, ref mavlink_object_detection_event_t object_detection_event)
{
    if (MAVLINK_NEED_BYTE_SWAP) {
    	object_detection_event.time = mavlink_msg_object_detection_event_get_time(msg);
    	object_detection_event.object_id = mavlink_msg_object_detection_event_get_object_id(msg);
    	object_detection_event.type = mavlink_msg_object_detection_event_get_type(msg);
    	object_detection_event.name = mavlink_msg_object_detection_event_get_name(msg);
    	object_detection_event.quality = mavlink_msg_object_detection_event_get_quality(msg);
    	object_detection_event.bearing = mavlink_msg_object_detection_event_get_bearing(msg);
    	object_detection_event.distance = mavlink_msg_object_detection_event_get_distance(msg);
    
    } else {
        int len = 36; //Marshal.SizeOf(object_detection_event);
        IntPtr i = Marshal.AllocHGlobal(len);
        Marshal.Copy(msg, 0, i, len);
        object_detection_event = (mavlink_object_detection_event_t)Marshal.PtrToStructure(i, ((object)object_detection_event).GetType());
        Marshal.FreeHGlobal(i);
    }
}

}
