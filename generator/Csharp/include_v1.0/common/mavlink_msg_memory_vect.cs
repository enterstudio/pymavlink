// MESSAGE MEMORY_VECT PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_MEMORY_VECT = 249;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_memory_vect_t
    {
        /// <summary>
        /// Starting address of the debug variables
        /// </summary>
        public  UInt16 address;
            /// <summary>
        /// Version code of the type variable. 0=unknown, type ignored and assumed int16_t. 1=as below
        /// </summary>
        public  byte ver;
            /// <summary>
        /// Type code of the memory variables. for ver = 1: 0=16 x int16_t, 1=16 x uint16_t, 2=16 x Q15, 3=16 x 1Q14
        /// </summary>
        public  byte type;
            /// <summary>
        /// Memory contents at specified address
        /// </summary>
        [MarshalAs(UnmanagedType.ByValArray,SizeConst=32)]
 public byte[] value;
    
    };

/// <summary>
/// * @brief Pack a memory_vect message
/// * @param system_id ID of this system
/// * @param component_id ID of this component (e.g. 200 for IMU)
/// * @param msg The MAVLink message to compress the data into
/// *
/// * @param address Starting address of the debug variables
/// * @param ver Version code of the type variable. 0=unknown, type ignored and assumed int16_t. 1=as below
/// * @param type Type code of the memory variables. for ver = 1: 0=16 x int16_t, 1=16 x uint16_t, 2=16 x Q15, 3=16 x 1Q14
/// * @param value Memory contents at specified address
/// * @return length of the message in bytes (excluding serial stream start sign)
/// </summary>
 
public static UInt16 mavlink_msg_memory_vect_pack(byte system_id, byte component_id, byte[] msg,
                               UInt16 address, byte ver, byte type, byte[] value)
{
if (MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS) {
	Array.Copy(BitConverter.GetBytes(address),0,msg,0,sizeof(UInt16));
	Array.Copy(BitConverter.GetBytes(ver),0,msg,2,sizeof(byte));
	Array.Copy(BitConverter.GetBytes(type),0,msg,3,sizeof(byte));
	Array.Copy(toArray(value),0,msg,4,32);
} else {
    mavlink_memory_vect_t packet = new mavlink_memory_vect_t();
	packet.address = address;
	packet.ver = ver;
	packet.type = type;
	packet.value = value;
        
        int len = 36;
        msg = new byte[len];
        IntPtr ptr = Marshal.AllocHGlobal(len);
        Marshal.StructureToPtr(packet, ptr, true);
        Marshal.Copy(ptr, msg, 0, len);
        Marshal.FreeHGlobal(ptr);
}

    //msg.msgid = MAVLINK_MSG_ID_MEMORY_VECT;
    //return mavlink_finalize_message(msg, system_id, component_id, 36, 204);
    return 0;
}

/**
 * @brief Pack a memory_vect message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param address Starting address of the debug variables
 * @param ver Version code of the type variable. 0=unknown, type ignored and assumed int16_t. 1=as below
 * @param type Type code of the memory variables. for ver = 1: 0=16 x int16_t, 1=16 x uint16_t, 2=16 x Q15, 3=16 x 1Q14
 * @param value Memory contents at specified address
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_memory_vect_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   UInt16 public address,byte public ver,byte public type,const byte[] [MarshalAs(UnmanagedType.ByValArray,SizeConst=32)]
 publicvalue)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[36];
	_mav_put_UInt16(buf, 0, address);
	_mav_put_byte(buf, 2, ver);
	_mav_put_byte(buf, 3, type);
	_mav_put_byte[]_array(buf, 4, value, 32);
        memcpy(_MAV_PAYLOAD(msg), buf, 36);
#else
    mavlink_memory_vect_t packet;
	packet.address = address;
	packet.ver = ver;
	packet.type = type;
	memcpy(packet.value, value, sizeof(byte[])*32);
        memcpy(_MAV_PAYLOAD(msg), &packet, 36);
#endif

    msg->msgid = MAVLINK_MSG_ID_MEMORY_VECT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 36, 204);
}
*/
/**
 * @brief Encode a memory_vect struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param memory_vect C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_memory_vect_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_memory_vect_t* memory_vect)
{
    return mavlink_msg_memory_vect_pack(system_id, component_id, msg, memory_vect->address, memory_vect->ver, memory_vect->type, memory_vect->value);
}
*/
/**
 * @brief Send a memory_vect message
 * @param chan MAVLink channel to send the message
 *
 * @param address Starting address of the debug variables
 * @param ver Version code of the type variable. 0=unknown, type ignored and assumed int16_t. 1=as below
 * @param type Type code of the memory variables. for ver = 1: 0=16 x int16_t, 1=16 x uint16_t, 2=16 x Q15, 3=16 x 1Q14
 * @param value Memory contents at specified address
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_memory_vect_send(mavlink_channel_t chan, UInt16 public address, byte public ver, byte public type, const byte[] [MarshalAs(UnmanagedType.ByValArray,SizeConst=32)]
 publicvalue)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[36];
	_mav_put_UInt16(buf, 0, address);
	_mav_put_byte(buf, 2, ver);
	_mav_put_byte(buf, 3, type);
	_mav_put_byte[]_array(buf, 4, value, 32);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MEMORY_VECT, buf, 36, 204);
#else
    mavlink_memory_vect_t packet;
	packet.address = address;
	packet.ver = ver;
	packet.type = type;
	memcpy(packet.value, value, sizeof(byte[])*32);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MEMORY_VECT, (const char *)&packet, 36, 204);
#endif
}

#endif
*/
// MESSAGE MEMORY_VECT UNPACKING


/**
 * @brief Get field address from memory_vect message
 *
 * @return Starting address of the debug variables
 */
public static UInt16 mavlink_msg_memory_vect_get_address(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  0);
}

/**
 * @brief Get field ver from memory_vect message
 *
 * @return Version code of the type variable. 0=unknown, type ignored and assumed int16_t. 1=as below
 */
public static byte mavlink_msg_memory_vect_get_ver(byte[] msg)
{
    return getByte(msg,  2);
}

/**
 * @brief Get field type from memory_vect message
 *
 * @return Type code of the memory variables. for ver = 1: 0=16 x int16_t, 1=16 x uint16_t, 2=16 x Q15, 3=16 x 1Q14
 */
public static byte mavlink_msg_memory_vect_get_type(byte[] msg)
{
    return getByte(msg,  3);
}

/**
 * @brief Get field value from memory_vect message
 *
 * @return Memory contents at specified address
 */
public static byte[] mavlink_msg_memory_vect_get_value(byte[] msg)
{
    return getBytes(msg, 32,  4);
}

/**
 * @brief Decode a memory_vect message into a struct
 *
 * @param msg The message to decode
 * @param memory_vect C-struct to decode the message contents into
 */
public static void mavlink_msg_memory_vect_decode(byte[] msg, ref mavlink_memory_vect_t memory_vect)
{
    if (MAVLINK_NEED_BYTE_SWAP) {
    	memory_vect.address = mavlink_msg_memory_vect_get_address(msg);
    	memory_vect.ver = mavlink_msg_memory_vect_get_ver(msg);
    	memory_vect.type = mavlink_msg_memory_vect_get_type(msg);
    	memory_vect.value = mavlink_msg_memory_vect_get_value(msg);
    
    } else {
        int len = 36; //Marshal.SizeOf(memory_vect);
        IntPtr i = Marshal.AllocHGlobal(len);
        Marshal.Copy(msg, 0, i, len);
        memory_vect = (mavlink_memory_vect_t)Marshal.PtrToStructure(i, ((object)memory_vect).GetType());
        Marshal.FreeHGlobal(i);
    }
}

}
