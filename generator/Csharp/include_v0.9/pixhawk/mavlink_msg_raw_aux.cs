// MESSAGE RAW_AUX PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_RAW_AUX = 172;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_raw_aux_t
    {
         public  UInt16 adc1; /// ADC1 (J405 ADC3, LPC2148 AD0.6)
     public  UInt16 adc2; /// ADC2 (J405 ADC5, LPC2148 AD0.2)
     public  UInt16 adc3; /// ADC3 (J405 ADC6, LPC2148 AD0.1)
     public  UInt16 adc4; /// ADC4 (J405 ADC7, LPC2148 AD1.3)
     public  UInt16 vbat; /// Battery voltage
     public  Int16 temp; /// Temperature (degrees celcius)
     public  Int32 baro; /// Barometric pressure (hecto Pascal)
    
    };

/**
 * @brief Pack a raw_aux message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param adc1 ADC1 (J405 ADC3, LPC2148 AD0.6)
 * @param adc2 ADC2 (J405 ADC5, LPC2148 AD0.2)
 * @param adc3 ADC3 (J405 ADC6, LPC2148 AD0.1)
 * @param adc4 ADC4 (J405 ADC7, LPC2148 AD1.3)
 * @param vbat Battery voltage
 * @param temp Temperature (degrees celcius)
 * @param baro Barometric pressure (hecto Pascal)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static uint16 mavlink_msg_raw_aux_pack(byte system_id, byte component_id, ref byte[] msg,
                               UInt16 public adc1, UInt16 public adc2, UInt16 public adc3, UInt16 public adc4, UInt16 public vbat, Int16 public temp, Int32 public baro)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    byte buf[16];
	_mav_put_UInt16(buf, 0, adc1);
	_mav_put_UInt16(buf, 2, adc2);
	_mav_put_UInt16(buf, 4, adc3);
	_mav_put_UInt16(buf, 6, adc4);
	_mav_put_UInt16(buf, 8, vbat);
	_mav_put_Int16(buf, 10, temp);
	_mav_put_Int32(buf, 12, baro);

        memcpy(_MAV_PAYLOAD(msg), buf, 16);
#else
    mavlink_raw_aux_t packet;
	packet.adc1 = adc1;
	packet.adc2 = adc2;
	packet.adc3 = adc3;
	packet.adc4 = adc4;
	packet.vbat = vbat;
	packet.temp = temp;
	packet.baro = baro;

        memcpy(_MAV_PAYLOAD(msg), &packet, 16);
#endif

    msg->msgid = MAVLINK_MSG_ID_RAW_AUX;
    return mavlink_finalize_message(msg, system_id, component_id, 16);
}
*/
/**
 * @brief Pack a raw_aux message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param adc1 ADC1 (J405 ADC3, LPC2148 AD0.6)
 * @param adc2 ADC2 (J405 ADC5, LPC2148 AD0.2)
 * @param adc3 ADC3 (J405 ADC6, LPC2148 AD0.1)
 * @param adc4 ADC4 (J405 ADC7, LPC2148 AD1.3)
 * @param vbat Battery voltage
 * @param temp Temperature (degrees celcius)
 * @param baro Barometric pressure (hecto Pascal)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_raw_aux_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   UInt16 public adc1,UInt16 public adc2,UInt16 public adc3,UInt16 public adc4,UInt16 public vbat,Int16 public temp,Int32 public baro)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[16];
	_mav_put_UInt16(buf, 0, adc1);
	_mav_put_UInt16(buf, 2, adc2);
	_mav_put_UInt16(buf, 4, adc3);
	_mav_put_UInt16(buf, 6, adc4);
	_mav_put_UInt16(buf, 8, vbat);
	_mav_put_Int16(buf, 10, temp);
	_mav_put_Int32(buf, 12, baro);

        memcpy(_MAV_PAYLOAD(msg), buf, 16);
#else
    mavlink_raw_aux_t packet;
	packet.adc1 = adc1;
	packet.adc2 = adc2;
	packet.adc3 = adc3;
	packet.adc4 = adc4;
	packet.vbat = vbat;
	packet.temp = temp;
	packet.baro = baro;

        memcpy(_MAV_PAYLOAD(msg), &packet, 16);
#endif

    msg->msgid = MAVLINK_MSG_ID_RAW_AUX;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 16);
}
*/
/**
 * @brief Encode a raw_aux struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param raw_aux C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_raw_aux_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_raw_aux_t* raw_aux)
{
    return mavlink_msg_raw_aux_pack(system_id, component_id, msg, raw_aux->adc1, raw_aux->adc2, raw_aux->adc3, raw_aux->adc4, raw_aux->vbat, raw_aux->temp, raw_aux->baro);
}
*/
/**
 * @brief Send a raw_aux message
 * @param chan MAVLink channel to send the message
 *
 * @param adc1 ADC1 (J405 ADC3, LPC2148 AD0.6)
 * @param adc2 ADC2 (J405 ADC5, LPC2148 AD0.2)
 * @param adc3 ADC3 (J405 ADC6, LPC2148 AD0.1)
 * @param adc4 ADC4 (J405 ADC7, LPC2148 AD1.3)
 * @param vbat Battery voltage
 * @param temp Temperature (degrees celcius)
 * @param baro Barometric pressure (hecto Pascal)
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_raw_aux_send(mavlink_channel_t chan, UInt16 public adc1, UInt16 public adc2, UInt16 public adc3, UInt16 public adc4, UInt16 public vbat, Int16 public temp, Int32 public baro)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[16];
	_mav_put_UInt16(buf, 0, adc1);
	_mav_put_UInt16(buf, 2, adc2);
	_mav_put_UInt16(buf, 4, adc3);
	_mav_put_UInt16(buf, 6, adc4);
	_mav_put_UInt16(buf, 8, vbat);
	_mav_put_Int16(buf, 10, temp);
	_mav_put_Int32(buf, 12, baro);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RAW_AUX, buf, 16);
#else
    mavlink_raw_aux_t packet;
	packet.adc1 = adc1;
	packet.adc2 = adc2;
	packet.adc3 = adc3;
	packet.adc4 = adc4;
	packet.vbat = vbat;
	packet.temp = temp;
	packet.baro = baro;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RAW_AUX, (const char *)&packet, 16);
#endif
}

#endif
*/
// MESSAGE RAW_AUX UNPACKING


/**
 * @brief Get field adc1 from raw_aux message
 *
 * @return ADC1 (J405 ADC3, LPC2148 AD0.6)
 */
public static UInt16 mavlink_msg_raw_aux_get_adc1(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  0);
}

/**
 * @brief Get field adc2 from raw_aux message
 *
 * @return ADC2 (J405 ADC5, LPC2148 AD0.2)
 */
public static UInt16 mavlink_msg_raw_aux_get_adc2(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  2);
}

/**
 * @brief Get field adc3 from raw_aux message
 *
 * @return ADC3 (J405 ADC6, LPC2148 AD0.1)
 */
public static UInt16 mavlink_msg_raw_aux_get_adc3(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  4);
}

/**
 * @brief Get field adc4 from raw_aux message
 *
 * @return ADC4 (J405 ADC7, LPC2148 AD1.3)
 */
public static UInt16 mavlink_msg_raw_aux_get_adc4(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  6);
}

/**
 * @brief Get field vbat from raw_aux message
 *
 * @return Battery voltage
 */
public static UInt16 mavlink_msg_raw_aux_get_vbat(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  8);
}

/**
 * @brief Get field temp from raw_aux message
 *
 * @return Temperature (degrees celcius)
 */
public static Int16 mavlink_msg_raw_aux_get_temp(byte[] msg)
{
    return BitConverter.ToInt16(msg,  10);
}

/**
 * @brief Get field baro from raw_aux message
 *
 * @return Barometric pressure (hecto Pascal)
 */
public static Int32 mavlink_msg_raw_aux_get_baro(byte[] msg)
{
    return BitConverter.ToInt32(msg,  12);
}

/**
 * @brief Decode a raw_aux message into a struct
 *
 * @param msg The message to decode
 * @param raw_aux C-struct to decode the message contents into
 */
public static void mavlink_msg_raw_aux_decode(byte[] msg, ref mavlink_raw_aux_t raw_aux)
{
if (MAVLINK_NEED_BYTE_SWAP) {
	raw_aux.adc1 = mavlink_msg_raw_aux_get_adc1(msg);
	raw_aux.adc2 = mavlink_msg_raw_aux_get_adc2(msg);
	raw_aux.adc3 = mavlink_msg_raw_aux_get_adc3(msg);
	raw_aux.adc4 = mavlink_msg_raw_aux_get_adc4(msg);
	raw_aux.vbat = mavlink_msg_raw_aux_get_vbat(msg);
	raw_aux.temp = mavlink_msg_raw_aux_get_temp(msg);
	raw_aux.baro = mavlink_msg_raw_aux_get_baro(msg);
} else {
    int len = 16; //Marshal.SizeOf(raw_aux);
    IntPtr i = Marshal.AllocHGlobal(len);
    Marshal.Copy(msg, 0, i, len);
    raw_aux = (mavlink_raw_aux_t)Marshal.PtrToStructure(i, ((object)raw_aux).GetType());
    Marshal.FreeHGlobal(i);
}
}

}
