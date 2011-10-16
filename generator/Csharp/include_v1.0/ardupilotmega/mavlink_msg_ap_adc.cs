// MESSAGE AP_ADC PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_AP_ADC = 153;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_ap_adc_t
    {
         public  UInt16 adc1; /// ADC output 1
     public  UInt16 adc2; /// ADC output 2
     public  UInt16 adc3; /// ADC output 3
     public  UInt16 adc4; /// ADC output 4
     public  UInt16 adc5; /// ADC output 5
     public  UInt16 adc6; /// ADC output 6
    
    };

/**
 * @brief Pack a ap_adc message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param adc1 ADC output 1
 * @param adc2 ADC output 2
 * @param adc3 ADC output 3
 * @param adc4 ADC output 4
 * @param adc5 ADC output 5
 * @param adc6 ADC output 6
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 
public static UInt16 mavlink_msg_ap_adc_pack(byte system_id, byte component_id, byte[] msg,
                               UInt16 adc1, UInt16 adc2, UInt16 adc3, UInt16 adc4, UInt16 adc5, UInt16 adc6)
{
if (MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS) {
	Array.Copy(BitConverter.GetBytes(adc1),0,msg,0,sizeof(UInt16));
	Array.Copy(BitConverter.GetBytes(adc2),0,msg,2,sizeof(UInt16));
	Array.Copy(BitConverter.GetBytes(adc3),0,msg,4,sizeof(UInt16));
	Array.Copy(BitConverter.GetBytes(adc4),0,msg,6,sizeof(UInt16));
	Array.Copy(BitConverter.GetBytes(adc5),0,msg,8,sizeof(UInt16));
	Array.Copy(BitConverter.GetBytes(adc6),0,msg,10,sizeof(UInt16));

} else {
    mavlink_ap_adc_t packet = new mavlink_ap_adc_t();
	packet.adc1 = adc1;
	packet.adc2 = adc2;
	packet.adc3 = adc3;
	packet.adc4 = adc4;
	packet.adc5 = adc5;
	packet.adc6 = adc6;

        
        int len = 12;
        msg = new byte[len];
        IntPtr ptr = Marshal.AllocHGlobal(len);
        Marshal.StructureToPtr(packet, ptr, true);
        Marshal.Copy(ptr, msg, 0, len);
        Marshal.FreeHGlobal(ptr);
}

    //msg.msgid = MAVLINK_MSG_ID_AP_ADC;
    //return mavlink_finalize_message(msg, system_id, component_id, 12, 188);
    return 0;
}

/**
 * @brief Pack a ap_adc message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param adc1 ADC output 1
 * @param adc2 ADC output 2
 * @param adc3 ADC output 3
 * @param adc4 ADC output 4
 * @param adc5 ADC output 5
 * @param adc6 ADC output 6
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_ap_adc_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   UInt16 public adc1,UInt16 public adc2,UInt16 public adc3,UInt16 public adc4,UInt16 public adc5,UInt16 public adc6)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[12];
	_mav_put_UInt16(buf, 0, adc1);
	_mav_put_UInt16(buf, 2, adc2);
	_mav_put_UInt16(buf, 4, adc3);
	_mav_put_UInt16(buf, 6, adc4);
	_mav_put_UInt16(buf, 8, adc5);
	_mav_put_UInt16(buf, 10, adc6);

        memcpy(_MAV_PAYLOAD(msg), buf, 12);
#else
    mavlink_ap_adc_t packet;
	packet.adc1 = adc1;
	packet.adc2 = adc2;
	packet.adc3 = adc3;
	packet.adc4 = adc4;
	packet.adc5 = adc5;
	packet.adc6 = adc6;

        memcpy(_MAV_PAYLOAD(msg), &packet, 12);
#endif

    msg->msgid = MAVLINK_MSG_ID_AP_ADC;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 12, 188);
}
*/
/**
 * @brief Encode a ap_adc struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param ap_adc C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_ap_adc_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_ap_adc_t* ap_adc)
{
    return mavlink_msg_ap_adc_pack(system_id, component_id, msg, ap_adc->adc1, ap_adc->adc2, ap_adc->adc3, ap_adc->adc4, ap_adc->adc5, ap_adc->adc6);
}
*/
/**
 * @brief Send a ap_adc message
 * @param chan MAVLink channel to send the message
 *
 * @param adc1 ADC output 1
 * @param adc2 ADC output 2
 * @param adc3 ADC output 3
 * @param adc4 ADC output 4
 * @param adc5 ADC output 5
 * @param adc6 ADC output 6
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_ap_adc_send(mavlink_channel_t chan, UInt16 public adc1, UInt16 public adc2, UInt16 public adc3, UInt16 public adc4, UInt16 public adc5, UInt16 public adc6)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[12];
	_mav_put_UInt16(buf, 0, adc1);
	_mav_put_UInt16(buf, 2, adc2);
	_mav_put_UInt16(buf, 4, adc3);
	_mav_put_UInt16(buf, 6, adc4);
	_mav_put_UInt16(buf, 8, adc5);
	_mav_put_UInt16(buf, 10, adc6);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AP_ADC, buf, 12, 188);
#else
    mavlink_ap_adc_t packet;
	packet.adc1 = adc1;
	packet.adc2 = adc2;
	packet.adc3 = adc3;
	packet.adc4 = adc4;
	packet.adc5 = adc5;
	packet.adc6 = adc6;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AP_ADC, (const char *)&packet, 12, 188);
#endif
}

#endif
*/
// MESSAGE AP_ADC UNPACKING


/**
 * @brief Get field adc1 from ap_adc message
 *
 * @return ADC output 1
 */
public static UInt16 mavlink_msg_ap_adc_get_adc1(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  0);
}

/**
 * @brief Get field adc2 from ap_adc message
 *
 * @return ADC output 2
 */
public static UInt16 mavlink_msg_ap_adc_get_adc2(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  2);
}

/**
 * @brief Get field adc3 from ap_adc message
 *
 * @return ADC output 3
 */
public static UInt16 mavlink_msg_ap_adc_get_adc3(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  4);
}

/**
 * @brief Get field adc4 from ap_adc message
 *
 * @return ADC output 4
 */
public static UInt16 mavlink_msg_ap_adc_get_adc4(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  6);
}

/**
 * @brief Get field adc5 from ap_adc message
 *
 * @return ADC output 5
 */
public static UInt16 mavlink_msg_ap_adc_get_adc5(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  8);
}

/**
 * @brief Get field adc6 from ap_adc message
 *
 * @return ADC output 6
 */
public static UInt16 mavlink_msg_ap_adc_get_adc6(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  10);
}

/**
 * @brief Decode a ap_adc message into a struct
 *
 * @param msg The message to decode
 * @param ap_adc C-struct to decode the message contents into
 */
public static void mavlink_msg_ap_adc_decode(byte[] msg, ref mavlink_ap_adc_t ap_adc)
{
    if (MAVLINK_NEED_BYTE_SWAP) {
    	ap_adc.adc1 = mavlink_msg_ap_adc_get_adc1(msg);
    	ap_adc.adc2 = mavlink_msg_ap_adc_get_adc2(msg);
    	ap_adc.adc3 = mavlink_msg_ap_adc_get_adc3(msg);
    	ap_adc.adc4 = mavlink_msg_ap_adc_get_adc4(msg);
    	ap_adc.adc5 = mavlink_msg_ap_adc_get_adc5(msg);
    	ap_adc.adc6 = mavlink_msg_ap_adc_get_adc6(msg);
    
    } else {
        int len = 12; //Marshal.SizeOf(ap_adc);
        IntPtr i = Marshal.AllocHGlobal(len);
        Marshal.Copy(msg, 0, i, len);
        ap_adc = (mavlink_ap_adc_t)Marshal.PtrToStructure(i, ((object)ap_adc).GetType());
        Marshal.FreeHGlobal(i);
    }
}

}
