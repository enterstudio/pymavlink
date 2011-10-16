// MESSAGE CPU_LOAD PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_CPU_LOAD = 170;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_cpu_load_t
    {
         public  byte sensLoad; /// Sensor DSC Load
     public  byte ctrlLoad; /// Control DSC Load
     public  UInt16 batVolt; /// Battery Voltage in millivolts
    
    };

/**
 * @brief Pack a cpu_load message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param sensLoad Sensor DSC Load
 * @param ctrlLoad Control DSC Load
 * @param batVolt Battery Voltage in millivolts
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 
public static UInt16 mavlink_msg_cpu_load_pack(byte system_id, byte component_id, byte[] msg,
                               byte sensLoad, byte ctrlLoad, UInt16 batVolt)
{
if (MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS) {
	Array.Copy(BitConverter.GetBytes(sensLoad),0,msg,0,sizeof(byte));
	Array.Copy(BitConverter.GetBytes(ctrlLoad),0,msg,1,sizeof(byte));
	Array.Copy(BitConverter.GetBytes(batVolt),0,msg,2,sizeof(UInt16));

} else {
    mavlink_cpu_load_t packet = new mavlink_cpu_load_t();
	packet.sensLoad = sensLoad;
	packet.ctrlLoad = ctrlLoad;
	packet.batVolt = batVolt;

        
        int len = 4;
        msg = new byte[len];
        IntPtr ptr = Marshal.AllocHGlobal(len);
        Marshal.StructureToPtr(packet, ptr, true);
        Marshal.Copy(ptr, msg, 0, len);
        Marshal.FreeHGlobal(ptr);
}

    //msg.msgid = MAVLINK_MSG_ID_CPU_LOAD;
    //return mavlink_finalize_message(msg, system_id, component_id, 4);
    return 0;
}

/**
 * @brief Pack a cpu_load message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param sensLoad Sensor DSC Load
 * @param ctrlLoad Control DSC Load
 * @param batVolt Battery Voltage in millivolts
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_cpu_load_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   byte public sensLoad,byte public ctrlLoad,UInt16 public batVolt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[4];
	_mav_put_byte(buf, 0, sensLoad);
	_mav_put_byte(buf, 1, ctrlLoad);
	_mav_put_UInt16(buf, 2, batVolt);

        memcpy(_MAV_PAYLOAD(msg), buf, 4);
#else
    mavlink_cpu_load_t packet;
	packet.sensLoad = sensLoad;
	packet.ctrlLoad = ctrlLoad;
	packet.batVolt = batVolt;

        memcpy(_MAV_PAYLOAD(msg), &packet, 4);
#endif

    msg->msgid = MAVLINK_MSG_ID_CPU_LOAD;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 4);
}
*/
/**
 * @brief Encode a cpu_load struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param cpu_load C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_cpu_load_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_cpu_load_t* cpu_load)
{
    return mavlink_msg_cpu_load_pack(system_id, component_id, msg, cpu_load->sensLoad, cpu_load->ctrlLoad, cpu_load->batVolt);
}
*/
/**
 * @brief Send a cpu_load message
 * @param chan MAVLink channel to send the message
 *
 * @param sensLoad Sensor DSC Load
 * @param ctrlLoad Control DSC Load
 * @param batVolt Battery Voltage in millivolts
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_cpu_load_send(mavlink_channel_t chan, byte public sensLoad, byte public ctrlLoad, UInt16 public batVolt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[4];
	_mav_put_byte(buf, 0, sensLoad);
	_mav_put_byte(buf, 1, ctrlLoad);
	_mav_put_UInt16(buf, 2, batVolt);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CPU_LOAD, buf, 4);
#else
    mavlink_cpu_load_t packet;
	packet.sensLoad = sensLoad;
	packet.ctrlLoad = ctrlLoad;
	packet.batVolt = batVolt;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CPU_LOAD, (const char *)&packet, 4);
#endif
}

#endif
*/
// MESSAGE CPU_LOAD UNPACKING


/**
 * @brief Get field sensLoad from cpu_load message
 *
 * @return Sensor DSC Load
 */
public static byte mavlink_msg_cpu_load_get_sensLoad(byte[] msg)
{
    return getByte(msg,  0);
}

/**
 * @brief Get field ctrlLoad from cpu_load message
 *
 * @return Control DSC Load
 */
public static byte mavlink_msg_cpu_load_get_ctrlLoad(byte[] msg)
{
    return getByte(msg,  1);
}

/**
 * @brief Get field batVolt from cpu_load message
 *
 * @return Battery Voltage in millivolts
 */
public static UInt16 mavlink_msg_cpu_load_get_batVolt(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  2);
}

/**
 * @brief Decode a cpu_load message into a struct
 *
 * @param msg The message to decode
 * @param cpu_load C-struct to decode the message contents into
 */
public static void mavlink_msg_cpu_load_decode(byte[] msg, ref mavlink_cpu_load_t cpu_load)
{
    if (MAVLINK_NEED_BYTE_SWAP) {
    	cpu_load.sensLoad = mavlink_msg_cpu_load_get_sensLoad(msg);
    	cpu_load.ctrlLoad = mavlink_msg_cpu_load_get_ctrlLoad(msg);
    	cpu_load.batVolt = mavlink_msg_cpu_load_get_batVolt(msg);
    
    } else {
        int len = 4; //Marshal.SizeOf(cpu_load);
        IntPtr i = Marshal.AllocHGlobal(len);
        Marshal.Copy(msg, 0, i, len);
        cpu_load = (mavlink_cpu_load_t)Marshal.PtrToStructure(i, ((object)cpu_load).GetType());
        Marshal.FreeHGlobal(i);
    }
}

}
