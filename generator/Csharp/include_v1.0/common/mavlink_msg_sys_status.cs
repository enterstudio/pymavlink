// MESSAGE SYS_STATUS PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_SYS_STATUS = 1;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_sys_status_t
    {
         public  UInt32 onboard_control_sensors_present; /// Bitmask showing which onboard controllers and sensors are present. Value of 0: not present. Value of 1: present. Indices: 0: 3D gyro, 1: 3D acc, 2: 3D mag, 3: absolute pressure, 4: differential pressure, 5: GPS, 6: optical flow, 7: computer vision position, 8: laser based position, 9: external ground-truth (Vicon or Leica). Controllers: 10: 3D angular rate control 11: attitude stabilization, 12: yaw position, 13: z/altitude control, 14: x/y position control, 15: motor outputs / control
     public  UInt32 onboard_control_sensors_enabled; /// Bitmask showing which onboard controllers and sensors are enabled:  Value of 0: not enabled. Value of 1: enabled. Indices: 0: 3D gyro, 1: 3D acc, 2: 3D mag, 3: absolute pressure, 4: differential pressure, 5: GPS, 6: optical flow, 7: computer vision position, 8: laser based position, 9: external ground-truth (Vicon or Leica). Controllers: 10: 3D angular rate control 11: attitude stabilization, 12: yaw position, 13: z/altitude control, 14: x/y position control, 15: motor outputs / control
     public  UInt32 onboard_control_sensors_health; /// Bitmask showing which onboard controllers and sensors are operational or have an error:  Value of 0: not enabled. Value of 1: enabled. Indices: 0: 3D gyro, 1: 3D acc, 2: 3D mag, 3: absolute pressure, 4: differential pressure, 5: GPS, 6: optical flow, 7: computer vision position, 8: laser based position, 9: external ground-truth (Vicon or Leica). Controllers: 10: 3D angular rate control 11: attitude stabilization, 12: yaw position, 13: z/altitude control, 14: x/y position control, 15: motor outputs / control
     public  UInt16 load; /// Maximum usage in percent of the mainloop time, (0%: 0, 100%: 1000) should be always below 1000
     public  UInt16 voltage_battery; /// Battery voltage, in millivolts (1 = 1 millivolt)
     public  Int16 current_battery; /// Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the current
     public  UInt16 drop_rate_comm; /// Communication drops in percent, (0%: 0, 100%: 10'000), (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)
     public  UInt16 errors_comm; /// Communication errors (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)
     public  UInt16 errors_count1; /// Autopilot-specific errors
     public  UInt16 errors_count2; /// Autopilot-specific errors
     public  UInt16 errors_count3; /// Autopilot-specific errors
     public  UInt16 errors_count4; /// Autopilot-specific errors
     public  byte battery_remaining; /// Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot estimate the remaining battery
    
    };

/**
 * @brief Pack a sys_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param onboard_control_sensors_present Bitmask showing which onboard controllers and sensors are present. Value of 0: not present. Value of 1: present. Indices: 0: 3D gyro, 1: 3D acc, 2: 3D mag, 3: absolute pressure, 4: differential pressure, 5: GPS, 6: optical flow, 7: computer vision position, 8: laser based position, 9: external ground-truth (Vicon or Leica). Controllers: 10: 3D angular rate control 11: attitude stabilization, 12: yaw position, 13: z/altitude control, 14: x/y position control, 15: motor outputs / control
 * @param onboard_control_sensors_enabled Bitmask showing which onboard controllers and sensors are enabled:  Value of 0: not enabled. Value of 1: enabled. Indices: 0: 3D gyro, 1: 3D acc, 2: 3D mag, 3: absolute pressure, 4: differential pressure, 5: GPS, 6: optical flow, 7: computer vision position, 8: laser based position, 9: external ground-truth (Vicon or Leica). Controllers: 10: 3D angular rate control 11: attitude stabilization, 12: yaw position, 13: z/altitude control, 14: x/y position control, 15: motor outputs / control
 * @param onboard_control_sensors_health Bitmask showing which onboard controllers and sensors are operational or have an error:  Value of 0: not enabled. Value of 1: enabled. Indices: 0: 3D gyro, 1: 3D acc, 2: 3D mag, 3: absolute pressure, 4: differential pressure, 5: GPS, 6: optical flow, 7: computer vision position, 8: laser based position, 9: external ground-truth (Vicon or Leica). Controllers: 10: 3D angular rate control 11: attitude stabilization, 12: yaw position, 13: z/altitude control, 14: x/y position control, 15: motor outputs / control
 * @param load Maximum usage in percent of the mainloop time, (0%: 0, 100%: 1000) should be always below 1000
 * @param voltage_battery Battery voltage, in millivolts (1 = 1 millivolt)
 * @param current_battery Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the current
 * @param battery_remaining Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot estimate the remaining battery
 * @param drop_rate_comm Communication drops in percent, (0%: 0, 100%: 10'000), (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)
 * @param errors_comm Communication errors (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)
 * @param errors_count1 Autopilot-specific errors
 * @param errors_count2 Autopilot-specific errors
 * @param errors_count3 Autopilot-specific errors
 * @param errors_count4 Autopilot-specific errors
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 
public static UInt16 mavlink_msg_sys_status_pack(byte system_id, byte component_id, byte[] msg,
                               UInt32 onboard_control_sensors_present, UInt32 onboard_control_sensors_enabled, UInt32 onboard_control_sensors_health, UInt16 load, UInt16 voltage_battery, Int16 current_battery, byte battery_remaining, UInt16 drop_rate_comm, UInt16 errors_comm, UInt16 errors_count1, UInt16 errors_count2, UInt16 errors_count3, UInt16 errors_count4)
{
if (MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS) {
	Array.Copy(BitConverter.GetBytes(onboard_control_sensors_present),0,msg,0,sizeof(UInt32));
	Array.Copy(BitConverter.GetBytes(onboard_control_sensors_enabled),0,msg,4,sizeof(UInt32));
	Array.Copy(BitConverter.GetBytes(onboard_control_sensors_health),0,msg,8,sizeof(UInt32));
	Array.Copy(BitConverter.GetBytes(load),0,msg,12,sizeof(UInt16));
	Array.Copy(BitConverter.GetBytes(voltage_battery),0,msg,14,sizeof(UInt16));
	Array.Copy(BitConverter.GetBytes(current_battery),0,msg,16,sizeof(Int16));
	Array.Copy(BitConverter.GetBytes(drop_rate_comm),0,msg,18,sizeof(UInt16));
	Array.Copy(BitConverter.GetBytes(errors_comm),0,msg,20,sizeof(UInt16));
	Array.Copy(BitConverter.GetBytes(errors_count1),0,msg,22,sizeof(UInt16));
	Array.Copy(BitConverter.GetBytes(errors_count2),0,msg,24,sizeof(UInt16));
	Array.Copy(BitConverter.GetBytes(errors_count3),0,msg,26,sizeof(UInt16));
	Array.Copy(BitConverter.GetBytes(errors_count4),0,msg,28,sizeof(UInt16));
	Array.Copy(BitConverter.GetBytes(battery_remaining),0,msg,30,sizeof(byte));

} else {
    mavlink_sys_status_t packet = new mavlink_sys_status_t();
	packet.onboard_control_sensors_present = onboard_control_sensors_present;
	packet.onboard_control_sensors_enabled = onboard_control_sensors_enabled;
	packet.onboard_control_sensors_health = onboard_control_sensors_health;
	packet.load = load;
	packet.voltage_battery = voltage_battery;
	packet.current_battery = current_battery;
	packet.drop_rate_comm = drop_rate_comm;
	packet.errors_comm = errors_comm;
	packet.errors_count1 = errors_count1;
	packet.errors_count2 = errors_count2;
	packet.errors_count3 = errors_count3;
	packet.errors_count4 = errors_count4;
	packet.battery_remaining = battery_remaining;

        
        int len = 31;
        msg = new byte[len];
        IntPtr ptr = Marshal.AllocHGlobal(len);
        Marshal.StructureToPtr(packet, ptr, true);
        Marshal.Copy(ptr, msg, 0, len);
        Marshal.FreeHGlobal(ptr);
}

    //msg.msgid = MAVLINK_MSG_ID_SYS_STATUS;
    //return mavlink_finalize_message(msg, system_id, component_id, 31, 124);
    return 0;
}

/**
 * @brief Pack a sys_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param onboard_control_sensors_present Bitmask showing which onboard controllers and sensors are present. Value of 0: not present. Value of 1: present. Indices: 0: 3D gyro, 1: 3D acc, 2: 3D mag, 3: absolute pressure, 4: differential pressure, 5: GPS, 6: optical flow, 7: computer vision position, 8: laser based position, 9: external ground-truth (Vicon or Leica). Controllers: 10: 3D angular rate control 11: attitude stabilization, 12: yaw position, 13: z/altitude control, 14: x/y position control, 15: motor outputs / control
 * @param onboard_control_sensors_enabled Bitmask showing which onboard controllers and sensors are enabled:  Value of 0: not enabled. Value of 1: enabled. Indices: 0: 3D gyro, 1: 3D acc, 2: 3D mag, 3: absolute pressure, 4: differential pressure, 5: GPS, 6: optical flow, 7: computer vision position, 8: laser based position, 9: external ground-truth (Vicon or Leica). Controllers: 10: 3D angular rate control 11: attitude stabilization, 12: yaw position, 13: z/altitude control, 14: x/y position control, 15: motor outputs / control
 * @param onboard_control_sensors_health Bitmask showing which onboard controllers and sensors are operational or have an error:  Value of 0: not enabled. Value of 1: enabled. Indices: 0: 3D gyro, 1: 3D acc, 2: 3D mag, 3: absolute pressure, 4: differential pressure, 5: GPS, 6: optical flow, 7: computer vision position, 8: laser based position, 9: external ground-truth (Vicon or Leica). Controllers: 10: 3D angular rate control 11: attitude stabilization, 12: yaw position, 13: z/altitude control, 14: x/y position control, 15: motor outputs / control
 * @param load Maximum usage in percent of the mainloop time, (0%: 0, 100%: 1000) should be always below 1000
 * @param voltage_battery Battery voltage, in millivolts (1 = 1 millivolt)
 * @param current_battery Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the current
 * @param battery_remaining Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot estimate the remaining battery
 * @param drop_rate_comm Communication drops in percent, (0%: 0, 100%: 10'000), (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)
 * @param errors_comm Communication errors (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)
 * @param errors_count1 Autopilot-specific errors
 * @param errors_count2 Autopilot-specific errors
 * @param errors_count3 Autopilot-specific errors
 * @param errors_count4 Autopilot-specific errors
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_sys_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   UInt32 public onboard_control_sensors_present,UInt32 public onboard_control_sensors_enabled,UInt32 public onboard_control_sensors_health,UInt16 public load,UInt16 public voltage_battery,Int16 public current_battery,byte public battery_remaining,UInt16 public drop_rate_comm,UInt16 public errors_comm,UInt16 public errors_count1,UInt16 public errors_count2,UInt16 public errors_count3,UInt16 public errors_count4)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[31];
	_mav_put_UInt32(buf, 0, onboard_control_sensors_present);
	_mav_put_UInt32(buf, 4, onboard_control_sensors_enabled);
	_mav_put_UInt32(buf, 8, onboard_control_sensors_health);
	_mav_put_UInt16(buf, 12, load);
	_mav_put_UInt16(buf, 14, voltage_battery);
	_mav_put_Int16(buf, 16, current_battery);
	_mav_put_UInt16(buf, 18, drop_rate_comm);
	_mav_put_UInt16(buf, 20, errors_comm);
	_mav_put_UInt16(buf, 22, errors_count1);
	_mav_put_UInt16(buf, 24, errors_count2);
	_mav_put_UInt16(buf, 26, errors_count3);
	_mav_put_UInt16(buf, 28, errors_count4);
	_mav_put_byte(buf, 30, battery_remaining);

        memcpy(_MAV_PAYLOAD(msg), buf, 31);
#else
    mavlink_sys_status_t packet;
	packet.onboard_control_sensors_present = onboard_control_sensors_present;
	packet.onboard_control_sensors_enabled = onboard_control_sensors_enabled;
	packet.onboard_control_sensors_health = onboard_control_sensors_health;
	packet.load = load;
	packet.voltage_battery = voltage_battery;
	packet.current_battery = current_battery;
	packet.drop_rate_comm = drop_rate_comm;
	packet.errors_comm = errors_comm;
	packet.errors_count1 = errors_count1;
	packet.errors_count2 = errors_count2;
	packet.errors_count3 = errors_count3;
	packet.errors_count4 = errors_count4;
	packet.battery_remaining = battery_remaining;

        memcpy(_MAV_PAYLOAD(msg), &packet, 31);
#endif

    msg->msgid = MAVLINK_MSG_ID_SYS_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 31, 124);
}
*/
/**
 * @brief Encode a sys_status struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param sys_status C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_sys_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_sys_status_t* sys_status)
{
    return mavlink_msg_sys_status_pack(system_id, component_id, msg, sys_status->onboard_control_sensors_present, sys_status->onboard_control_sensors_enabled, sys_status->onboard_control_sensors_health, sys_status->load, sys_status->voltage_battery, sys_status->current_battery, sys_status->battery_remaining, sys_status->drop_rate_comm, sys_status->errors_comm, sys_status->errors_count1, sys_status->errors_count2, sys_status->errors_count3, sys_status->errors_count4);
}
*/
/**
 * @brief Send a sys_status message
 * @param chan MAVLink channel to send the message
 *
 * @param onboard_control_sensors_present Bitmask showing which onboard controllers and sensors are present. Value of 0: not present. Value of 1: present. Indices: 0: 3D gyro, 1: 3D acc, 2: 3D mag, 3: absolute pressure, 4: differential pressure, 5: GPS, 6: optical flow, 7: computer vision position, 8: laser based position, 9: external ground-truth (Vicon or Leica). Controllers: 10: 3D angular rate control 11: attitude stabilization, 12: yaw position, 13: z/altitude control, 14: x/y position control, 15: motor outputs / control
 * @param onboard_control_sensors_enabled Bitmask showing which onboard controllers and sensors are enabled:  Value of 0: not enabled. Value of 1: enabled. Indices: 0: 3D gyro, 1: 3D acc, 2: 3D mag, 3: absolute pressure, 4: differential pressure, 5: GPS, 6: optical flow, 7: computer vision position, 8: laser based position, 9: external ground-truth (Vicon or Leica). Controllers: 10: 3D angular rate control 11: attitude stabilization, 12: yaw position, 13: z/altitude control, 14: x/y position control, 15: motor outputs / control
 * @param onboard_control_sensors_health Bitmask showing which onboard controllers and sensors are operational or have an error:  Value of 0: not enabled. Value of 1: enabled. Indices: 0: 3D gyro, 1: 3D acc, 2: 3D mag, 3: absolute pressure, 4: differential pressure, 5: GPS, 6: optical flow, 7: computer vision position, 8: laser based position, 9: external ground-truth (Vicon or Leica). Controllers: 10: 3D angular rate control 11: attitude stabilization, 12: yaw position, 13: z/altitude control, 14: x/y position control, 15: motor outputs / control
 * @param load Maximum usage in percent of the mainloop time, (0%: 0, 100%: 1000) should be always below 1000
 * @param voltage_battery Battery voltage, in millivolts (1 = 1 millivolt)
 * @param current_battery Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the current
 * @param battery_remaining Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot estimate the remaining battery
 * @param drop_rate_comm Communication drops in percent, (0%: 0, 100%: 10'000), (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)
 * @param errors_comm Communication errors (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)
 * @param errors_count1 Autopilot-specific errors
 * @param errors_count2 Autopilot-specific errors
 * @param errors_count3 Autopilot-specific errors
 * @param errors_count4 Autopilot-specific errors
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_sys_status_send(mavlink_channel_t chan, UInt32 public onboard_control_sensors_present, UInt32 public onboard_control_sensors_enabled, UInt32 public onboard_control_sensors_health, UInt16 public load, UInt16 public voltage_battery, Int16 public current_battery, byte public battery_remaining, UInt16 public drop_rate_comm, UInt16 public errors_comm, UInt16 public errors_count1, UInt16 public errors_count2, UInt16 public errors_count3, UInt16 public errors_count4)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[31];
	_mav_put_UInt32(buf, 0, onboard_control_sensors_present);
	_mav_put_UInt32(buf, 4, onboard_control_sensors_enabled);
	_mav_put_UInt32(buf, 8, onboard_control_sensors_health);
	_mav_put_UInt16(buf, 12, load);
	_mav_put_UInt16(buf, 14, voltage_battery);
	_mav_put_Int16(buf, 16, current_battery);
	_mav_put_UInt16(buf, 18, drop_rate_comm);
	_mav_put_UInt16(buf, 20, errors_comm);
	_mav_put_UInt16(buf, 22, errors_count1);
	_mav_put_UInt16(buf, 24, errors_count2);
	_mav_put_UInt16(buf, 26, errors_count3);
	_mav_put_UInt16(buf, 28, errors_count4);
	_mav_put_byte(buf, 30, battery_remaining);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SYS_STATUS, buf, 31, 124);
#else
    mavlink_sys_status_t packet;
	packet.onboard_control_sensors_present = onboard_control_sensors_present;
	packet.onboard_control_sensors_enabled = onboard_control_sensors_enabled;
	packet.onboard_control_sensors_health = onboard_control_sensors_health;
	packet.load = load;
	packet.voltage_battery = voltage_battery;
	packet.current_battery = current_battery;
	packet.drop_rate_comm = drop_rate_comm;
	packet.errors_comm = errors_comm;
	packet.errors_count1 = errors_count1;
	packet.errors_count2 = errors_count2;
	packet.errors_count3 = errors_count3;
	packet.errors_count4 = errors_count4;
	packet.battery_remaining = battery_remaining;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SYS_STATUS, (const char *)&packet, 31, 124);
#endif
}

#endif
*/
// MESSAGE SYS_STATUS UNPACKING


/**
 * @brief Get field onboard_control_sensors_present from sys_status message
 *
 * @return Bitmask showing which onboard controllers and sensors are present. Value of 0: not present. Value of 1: present. Indices: 0: 3D gyro, 1: 3D acc, 2: 3D mag, 3: absolute pressure, 4: differential pressure, 5: GPS, 6: optical flow, 7: computer vision position, 8: laser based position, 9: external ground-truth (Vicon or Leica). Controllers: 10: 3D angular rate control 11: attitude stabilization, 12: yaw position, 13: z/altitude control, 14: x/y position control, 15: motor outputs / control
 */
public static UInt32 mavlink_msg_sys_status_get_onboard_control_sensors_present(byte[] msg)
{
    return BitConverter.ToUInt32(msg,  0);
}

/**
 * @brief Get field onboard_control_sensors_enabled from sys_status message
 *
 * @return Bitmask showing which onboard controllers and sensors are enabled:  Value of 0: not enabled. Value of 1: enabled. Indices: 0: 3D gyro, 1: 3D acc, 2: 3D mag, 3: absolute pressure, 4: differential pressure, 5: GPS, 6: optical flow, 7: computer vision position, 8: laser based position, 9: external ground-truth (Vicon or Leica). Controllers: 10: 3D angular rate control 11: attitude stabilization, 12: yaw position, 13: z/altitude control, 14: x/y position control, 15: motor outputs / control
 */
public static UInt32 mavlink_msg_sys_status_get_onboard_control_sensors_enabled(byte[] msg)
{
    return BitConverter.ToUInt32(msg,  4);
}

/**
 * @brief Get field onboard_control_sensors_health from sys_status message
 *
 * @return Bitmask showing which onboard controllers and sensors are operational or have an error:  Value of 0: not enabled. Value of 1: enabled. Indices: 0: 3D gyro, 1: 3D acc, 2: 3D mag, 3: absolute pressure, 4: differential pressure, 5: GPS, 6: optical flow, 7: computer vision position, 8: laser based position, 9: external ground-truth (Vicon or Leica). Controllers: 10: 3D angular rate control 11: attitude stabilization, 12: yaw position, 13: z/altitude control, 14: x/y position control, 15: motor outputs / control
 */
public static UInt32 mavlink_msg_sys_status_get_onboard_control_sensors_health(byte[] msg)
{
    return BitConverter.ToUInt32(msg,  8);
}

/**
 * @brief Get field load from sys_status message
 *
 * @return Maximum usage in percent of the mainloop time, (0%: 0, 100%: 1000) should be always below 1000
 */
public static UInt16 mavlink_msg_sys_status_get_load(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  12);
}

/**
 * @brief Get field voltage_battery from sys_status message
 *
 * @return Battery voltage, in millivolts (1 = 1 millivolt)
 */
public static UInt16 mavlink_msg_sys_status_get_voltage_battery(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  14);
}

/**
 * @brief Get field current_battery from sys_status message
 *
 * @return Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the current
 */
public static Int16 mavlink_msg_sys_status_get_current_battery(byte[] msg)
{
    return BitConverter.ToInt16(msg,  16);
}

/**
 * @brief Get field battery_remaining from sys_status message
 *
 * @return Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot estimate the remaining battery
 */
public static byte mavlink_msg_sys_status_get_battery_remaining(byte[] msg)
{
    return getByte(msg,  30);
}

/**
 * @brief Get field drop_rate_comm from sys_status message
 *
 * @return Communication drops in percent, (0%: 0, 100%: 10'000), (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)
 */
public static UInt16 mavlink_msg_sys_status_get_drop_rate_comm(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  18);
}

/**
 * @brief Get field errors_comm from sys_status message
 *
 * @return Communication errors (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)
 */
public static UInt16 mavlink_msg_sys_status_get_errors_comm(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  20);
}

/**
 * @brief Get field errors_count1 from sys_status message
 *
 * @return Autopilot-specific errors
 */
public static UInt16 mavlink_msg_sys_status_get_errors_count1(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  22);
}

/**
 * @brief Get field errors_count2 from sys_status message
 *
 * @return Autopilot-specific errors
 */
public static UInt16 mavlink_msg_sys_status_get_errors_count2(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  24);
}

/**
 * @brief Get field errors_count3 from sys_status message
 *
 * @return Autopilot-specific errors
 */
public static UInt16 mavlink_msg_sys_status_get_errors_count3(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  26);
}

/**
 * @brief Get field errors_count4 from sys_status message
 *
 * @return Autopilot-specific errors
 */
public static UInt16 mavlink_msg_sys_status_get_errors_count4(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  28);
}

/**
 * @brief Decode a sys_status message into a struct
 *
 * @param msg The message to decode
 * @param sys_status C-struct to decode the message contents into
 */
public static void mavlink_msg_sys_status_decode(byte[] msg, ref mavlink_sys_status_t sys_status)
{
    if (MAVLINK_NEED_BYTE_SWAP) {
    	sys_status.onboard_control_sensors_present = mavlink_msg_sys_status_get_onboard_control_sensors_present(msg);
    	sys_status.onboard_control_sensors_enabled = mavlink_msg_sys_status_get_onboard_control_sensors_enabled(msg);
    	sys_status.onboard_control_sensors_health = mavlink_msg_sys_status_get_onboard_control_sensors_health(msg);
    	sys_status.load = mavlink_msg_sys_status_get_load(msg);
    	sys_status.voltage_battery = mavlink_msg_sys_status_get_voltage_battery(msg);
    	sys_status.current_battery = mavlink_msg_sys_status_get_current_battery(msg);
    	sys_status.drop_rate_comm = mavlink_msg_sys_status_get_drop_rate_comm(msg);
    	sys_status.errors_comm = mavlink_msg_sys_status_get_errors_comm(msg);
    	sys_status.errors_count1 = mavlink_msg_sys_status_get_errors_count1(msg);
    	sys_status.errors_count2 = mavlink_msg_sys_status_get_errors_count2(msg);
    	sys_status.errors_count3 = mavlink_msg_sys_status_get_errors_count3(msg);
    	sys_status.errors_count4 = mavlink_msg_sys_status_get_errors_count4(msg);
    	sys_status.battery_remaining = mavlink_msg_sys_status_get_battery_remaining(msg);
    
    } else {
        int len = 31; //Marshal.SizeOf(sys_status);
        IntPtr i = Marshal.AllocHGlobal(len);
        Marshal.Copy(msg, 0, i, len);
        sys_status = (mavlink_sys_status_t)Marshal.PtrToStructure(i, ((object)sys_status).GetType());
        Marshal.FreeHGlobal(i);
    }
}

}
