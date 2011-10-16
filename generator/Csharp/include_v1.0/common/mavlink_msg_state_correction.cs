// MESSAGE STATE_CORRECTION PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_STATE_CORRECTION = 64;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_state_correction_t
    {
         public  Single xErr; /// x position error
     public  Single yErr; /// y position error
     public  Single zErr; /// z position error
     public  Single rollErr; /// roll error (radians)
     public  Single pitchErr; /// pitch error (radians)
     public  Single yawErr; /// yaw error (radians)
     public  Single vxErr; /// x velocity
     public  Single vyErr; /// y velocity
     public  Single vzErr; /// z velocity
    
    };

/**
 * @brief Pack a state_correction message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param xErr x position error
 * @param yErr y position error
 * @param zErr z position error
 * @param rollErr roll error (radians)
 * @param pitchErr pitch error (radians)
 * @param yawErr yaw error (radians)
 * @param vxErr x velocity
 * @param vyErr y velocity
 * @param vzErr z velocity
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 
public static UInt16 mavlink_msg_state_correction_pack(byte system_id, byte component_id, byte[] msg,
                               Single xErr, Single yErr, Single zErr, Single rollErr, Single pitchErr, Single yawErr, Single vxErr, Single vyErr, Single vzErr)
{
if (MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS) {
	Array.Copy(BitConverter.GetBytes(xErr),0,msg,0,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(yErr),0,msg,4,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(zErr),0,msg,8,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(rollErr),0,msg,12,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(pitchErr),0,msg,16,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(yawErr),0,msg,20,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(vxErr),0,msg,24,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(vyErr),0,msg,28,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(vzErr),0,msg,32,sizeof(Single));

} else {
    mavlink_state_correction_t packet = new mavlink_state_correction_t();
	packet.xErr = xErr;
	packet.yErr = yErr;
	packet.zErr = zErr;
	packet.rollErr = rollErr;
	packet.pitchErr = pitchErr;
	packet.yawErr = yawErr;
	packet.vxErr = vxErr;
	packet.vyErr = vyErr;
	packet.vzErr = vzErr;

        
        int len = 36;
        msg = new byte[len];
        IntPtr ptr = Marshal.AllocHGlobal(len);
        Marshal.StructureToPtr(packet, ptr, true);
        Marshal.Copy(ptr, msg, 0, len);
        Marshal.FreeHGlobal(ptr);
}

    //msg.msgid = MAVLINK_MSG_ID_STATE_CORRECTION;
    //return mavlink_finalize_message(msg, system_id, component_id, 36, 130);
    return 0;
}

/**
 * @brief Pack a state_correction message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param xErr x position error
 * @param yErr y position error
 * @param zErr z position error
 * @param rollErr roll error (radians)
 * @param pitchErr pitch error (radians)
 * @param yawErr yaw error (radians)
 * @param vxErr x velocity
 * @param vyErr y velocity
 * @param vzErr z velocity
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_state_correction_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   Single public xErr,Single public yErr,Single public zErr,Single public rollErr,Single public pitchErr,Single public yawErr,Single public vxErr,Single public vyErr,Single public vzErr)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[36];
	_mav_put_Single(buf, 0, xErr);
	_mav_put_Single(buf, 4, yErr);
	_mav_put_Single(buf, 8, zErr);
	_mav_put_Single(buf, 12, rollErr);
	_mav_put_Single(buf, 16, pitchErr);
	_mav_put_Single(buf, 20, yawErr);
	_mav_put_Single(buf, 24, vxErr);
	_mav_put_Single(buf, 28, vyErr);
	_mav_put_Single(buf, 32, vzErr);

        memcpy(_MAV_PAYLOAD(msg), buf, 36);
#else
    mavlink_state_correction_t packet;
	packet.xErr = xErr;
	packet.yErr = yErr;
	packet.zErr = zErr;
	packet.rollErr = rollErr;
	packet.pitchErr = pitchErr;
	packet.yawErr = yawErr;
	packet.vxErr = vxErr;
	packet.vyErr = vyErr;
	packet.vzErr = vzErr;

        memcpy(_MAV_PAYLOAD(msg), &packet, 36);
#endif

    msg->msgid = MAVLINK_MSG_ID_STATE_CORRECTION;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 36, 130);
}
*/
/**
 * @brief Encode a state_correction struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param state_correction C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_state_correction_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_state_correction_t* state_correction)
{
    return mavlink_msg_state_correction_pack(system_id, component_id, msg, state_correction->xErr, state_correction->yErr, state_correction->zErr, state_correction->rollErr, state_correction->pitchErr, state_correction->yawErr, state_correction->vxErr, state_correction->vyErr, state_correction->vzErr);
}
*/
/**
 * @brief Send a state_correction message
 * @param chan MAVLink channel to send the message
 *
 * @param xErr x position error
 * @param yErr y position error
 * @param zErr z position error
 * @param rollErr roll error (radians)
 * @param pitchErr pitch error (radians)
 * @param yawErr yaw error (radians)
 * @param vxErr x velocity
 * @param vyErr y velocity
 * @param vzErr z velocity
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_state_correction_send(mavlink_channel_t chan, Single public xErr, Single public yErr, Single public zErr, Single public rollErr, Single public pitchErr, Single public yawErr, Single public vxErr, Single public vyErr, Single public vzErr)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[36];
	_mav_put_Single(buf, 0, xErr);
	_mav_put_Single(buf, 4, yErr);
	_mav_put_Single(buf, 8, zErr);
	_mav_put_Single(buf, 12, rollErr);
	_mav_put_Single(buf, 16, pitchErr);
	_mav_put_Single(buf, 20, yawErr);
	_mav_put_Single(buf, 24, vxErr);
	_mav_put_Single(buf, 28, vyErr);
	_mav_put_Single(buf, 32, vzErr);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STATE_CORRECTION, buf, 36, 130);
#else
    mavlink_state_correction_t packet;
	packet.xErr = xErr;
	packet.yErr = yErr;
	packet.zErr = zErr;
	packet.rollErr = rollErr;
	packet.pitchErr = pitchErr;
	packet.yawErr = yawErr;
	packet.vxErr = vxErr;
	packet.vyErr = vyErr;
	packet.vzErr = vzErr;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STATE_CORRECTION, (const char *)&packet, 36, 130);
#endif
}

#endif
*/
// MESSAGE STATE_CORRECTION UNPACKING


/**
 * @brief Get field xErr from state_correction message
 *
 * @return x position error
 */
public static Single mavlink_msg_state_correction_get_xErr(byte[] msg)
{
    return BitConverter.ToSingle(msg,  0);
}

/**
 * @brief Get field yErr from state_correction message
 *
 * @return y position error
 */
public static Single mavlink_msg_state_correction_get_yErr(byte[] msg)
{
    return BitConverter.ToSingle(msg,  4);
}

/**
 * @brief Get field zErr from state_correction message
 *
 * @return z position error
 */
public static Single mavlink_msg_state_correction_get_zErr(byte[] msg)
{
    return BitConverter.ToSingle(msg,  8);
}

/**
 * @brief Get field rollErr from state_correction message
 *
 * @return roll error (radians)
 */
public static Single mavlink_msg_state_correction_get_rollErr(byte[] msg)
{
    return BitConverter.ToSingle(msg,  12);
}

/**
 * @brief Get field pitchErr from state_correction message
 *
 * @return pitch error (radians)
 */
public static Single mavlink_msg_state_correction_get_pitchErr(byte[] msg)
{
    return BitConverter.ToSingle(msg,  16);
}

/**
 * @brief Get field yawErr from state_correction message
 *
 * @return yaw error (radians)
 */
public static Single mavlink_msg_state_correction_get_yawErr(byte[] msg)
{
    return BitConverter.ToSingle(msg,  20);
}

/**
 * @brief Get field vxErr from state_correction message
 *
 * @return x velocity
 */
public static Single mavlink_msg_state_correction_get_vxErr(byte[] msg)
{
    return BitConverter.ToSingle(msg,  24);
}

/**
 * @brief Get field vyErr from state_correction message
 *
 * @return y velocity
 */
public static Single mavlink_msg_state_correction_get_vyErr(byte[] msg)
{
    return BitConverter.ToSingle(msg,  28);
}

/**
 * @brief Get field vzErr from state_correction message
 *
 * @return z velocity
 */
public static Single mavlink_msg_state_correction_get_vzErr(byte[] msg)
{
    return BitConverter.ToSingle(msg,  32);
}

/**
 * @brief Decode a state_correction message into a struct
 *
 * @param msg The message to decode
 * @param state_correction C-struct to decode the message contents into
 */
public static void mavlink_msg_state_correction_decode(byte[] msg, ref mavlink_state_correction_t state_correction)
{
    if (MAVLINK_NEED_BYTE_SWAP) {
    	state_correction.xErr = mavlink_msg_state_correction_get_xErr(msg);
    	state_correction.yErr = mavlink_msg_state_correction_get_yErr(msg);
    	state_correction.zErr = mavlink_msg_state_correction_get_zErr(msg);
    	state_correction.rollErr = mavlink_msg_state_correction_get_rollErr(msg);
    	state_correction.pitchErr = mavlink_msg_state_correction_get_pitchErr(msg);
    	state_correction.yawErr = mavlink_msg_state_correction_get_yawErr(msg);
    	state_correction.vxErr = mavlink_msg_state_correction_get_vxErr(msg);
    	state_correction.vyErr = mavlink_msg_state_correction_get_vyErr(msg);
    	state_correction.vzErr = mavlink_msg_state_correction_get_vzErr(msg);
    
    } else {
        int len = 36; //Marshal.SizeOf(state_correction);
        IntPtr i = Marshal.AllocHGlobal(len);
        Marshal.Copy(msg, 0, i, len);
        state_correction = (mavlink_state_correction_t)Marshal.PtrToStructure(i, ((object)state_correction).GetType());
        Marshal.FreeHGlobal(i);
    }
}

}
