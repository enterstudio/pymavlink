// MESSAGE SYS_STATUS PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_SYS_STATUS = 1;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_sys_status_t
    {
         public  UInt16 onboard_control_sensors_present; /// Bitmask showing which onboard controllers and sensors are present. Value of 0: not present. Value of 1: present. Indices: 0: 3D gyro, 1: 3D acc, 2: 3D mag, 3: absolute pressure, 4: differential pressure, 5: GPS, 6: optical flow, 7: computer vision position, 8: laser based position, 9: external ground-truth (Vicon or Leica). Controllers: 10: 3D angular rate control 11: attitude stabilization, 12: yaw position, 13: z/altitude control, 14: x/y position control, 15: motor outputs / control
     public  UInt16 onboard_control_sensors_enabled; /// Bitmask showing which onboard controllers and sensors are enabled:  Value of 0: not enabled. Value of 1: enabled. Indices: 0: 3D gyro, 1: 3D acc, 2: 3D mag, 3: absolute pressure, 4: differential pressure, 5: GPS, 6: optical flow, 7: computer vision position, 8: laser based position, 9: external ground-truth (Vicon or Leica). Controllers: 10: 3D angular rate control 11: attitude stabilization, 12: yaw position, 13: z/altitude control, 14: x/y position control, 15: motor outputs / control
     public  UInt16 onboard_control_sensors_health; /// Bitmask showing which onboard controllers and sensors are operational or have an error:  Value of 0: not enabled. Value of 1: enabled. Indices: 0: 3D gyro, 1: 3D acc, 2: 3D mag, 3: absolute pressure, 4: differential pressure, 5: GPS, 6: optical flow, 7: computer vision position, 8: laser based position, 9: external ground-truth (Vicon or Leica). Controllers: 10: 3D angular rate control 11: attitude stabilization, 12: yaw position, 13: z/altitude control, 14: x/y position control, 15: motor outputs / control
     public  UInt16 load; /// Maximum usage in percent of the mainloop time, (0%: 0, 100%: 1000) should be always below 1000
     public  UInt16 voltage_battery; /// Battery voltage, in millivolts (1 = 1 millivolt)
     public  UInt16 current_battery; /// Battery current, in milliamperes (1 = 1 milliampere)
     public  UInt16 watt; /// Watts consumed from this battery since startup
     public  UInt16 errors_uart; /// Dropped packets on all links (packets that were corrupted on reception on the MAV)
     public  UInt16 errors_i2c; /// Dropped packets on all links (packets that were corrupted on reception)
     public  UInt16 errors_spi; /// Dropped packets on all links (packets that were corrupted on reception)
     public  UInt16 errors_can; /// Dropped packets on all links (packets that were corrupted on reception)
     public  byte battery_percent; /// Remaining battery energy: (0%: 0, 100%: 255)
    
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
 * @param current_battery Battery current, in milliamperes (1 = 1 milliampere)
 * @param watt Watts consumed from this battery since startup
 * @param battery_percent Remaining battery energy: (0%: 0, 100%: 255)
 * @param errors_uart Dropped packets on all links (packets that were corrupted on reception on the MAV)
 * @param errors_i2c Dropped packets on all links (packets that were corrupted on reception)
 * @param errors_spi Dropped packets on all links (packets that were corrupted on reception)
 * @param errors_can Dropped packets on all links (packets that were corrupted on reception)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static uint16 mavlink_msg_sys_status_pack(byte system_id, byte component_id, ref byte[] msg,
                               UInt16 public onboard_control_sensors_present, UInt16 public onboard_control_sensors_enabled, UInt16 public onboard_control_sensors_health, UInt16 public load, UInt16 public voltage_battery, UInt16 public current_battery, UInt16 public watt, byte public battery_percent, UInt16 public errors_uart, UInt16 public errors_i2c, UInt16 public errors_spi, UInt16 public errors_can)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    byte buf[23];
	_mav_put_UInt16(buf, 0, onboard_control_sensors_present);
	_mav_put_UInt16(buf, 2, onboard_control_sensors_enabled);
	_mav_put_UInt16(buf, 4, onboard_control_sensors_health);
	_mav_put_UInt16(buf, 6, load);
	_mav_put_UInt16(buf, 8, voltage_battery);
	_mav_put_UInt16(buf, 10, current_battery);
	_mav_put_UInt16(buf, 12, watt);
	_mav_put_UInt16(buf, 14, errors_uart);
	_mav_put_UInt16(buf, 16, errors_i2c);
	_mav_put_UInt16(buf, 18, errors_spi);
	_mav_put_UInt16(buf, 20, errors_can);
	_mav_put_byte(buf, 22, battery_percent);

        memcpy(_MAV_PAYLOAD(msg), buf, 23);
#else
    mavlink_sys_status_t packet;
	packet.onboard_control_sensors_present = onboard_control_sensors_present;
	packet.onboard_control_sensors_enabled = onboard_control_sensors_enabled;
	packet.onboard_control_sensors_health = onboard_control_sensors_health;
	packet.load = load;
	packet.voltage_battery = voltage_battery;
	packet.current_battery = current_battery;
	packet.watt = watt;
	packet.errors_uart = errors_uart;
	packet.errors_i2c = errors_i2c;
	packet.errors_spi = errors_spi;
	packet.errors_can = errors_can;
	packet.battery_percent = battery_percent;

        memcpy(_MAV_PAYLOAD(msg), &packet, 23);
#endif

    msg->msgid = MAVLINK_MSG_ID_SYS_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, 23, 114);
}
*/
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
 * @param current_battery Battery current, in milliamperes (1 = 1 milliampere)
 * @param watt Watts consumed from this battery since startup
 * @param battery_percent Remaining battery energy: (0%: 0, 100%: 255)
 * @param errors_uart Dropped packets on all links (packets that were corrupted on reception on the MAV)
 * @param errors_i2c Dropped packets on all links (packets that were corrupted on reception)
 * @param errors_spi Dropped packets on all links (packets that were corrupted on reception)
 * @param errors_can Dropped packets on all links (packets that were corrupted on reception)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_sys_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   UInt16 public onboard_control_sensors_present,UInt16 public onboard_control_sensors_enabled,UInt16 public onboard_control_sensors_health,UInt16 public load,UInt16 public voltage_battery,UInt16 public current_battery,UInt16 public watt,byte public battery_percent,UInt16 public errors_uart,UInt16 public errors_i2c,UInt16 public errors_spi,UInt16 public errors_can)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[23];
	_mav_put_UInt16(buf, 0, onboard_control_sensors_present);
	_mav_put_UInt16(buf, 2, onboard_control_sensors_enabled);
	_mav_put_UInt16(buf, 4, onboard_control_sensors_health);
	_mav_put_UInt16(buf, 6, load);
	_mav_put_UInt16(buf, 8, voltage_battery);
	_mav_put_UInt16(buf, 10, current_battery);
	_mav_put_UInt16(buf, 12, watt);
	_mav_put_UInt16(buf, 14, errors_uart);
	_mav_put_UInt16(buf, 16, errors_i2c);
	_mav_put_UInt16(buf, 18, errors_spi);
	_mav_put_UInt16(buf, 20, errors_can);
	_mav_put_byte(buf, 22, battery_percent);

        memcpy(_MAV_PAYLOAD(msg), buf, 23);
#else
    mavlink_sys_status_t packet;
	packet.onboard_control_sensors_present = onboard_control_sensors_present;
	packet.onboard_control_sensors_enabled = onboard_control_sensors_enabled;
	packet.onboard_control_sensors_health = onboard_control_sensors_health;
	packet.load = load;
	packet.voltage_battery = voltage_battery;
	packet.current_battery = current_battery;
	packet.watt = watt;
	packet.errors_uart = errors_uart;
	packet.errors_i2c = errors_i2c;
	packet.errors_spi = errors_spi;
	packet.errors_can = errors_can;
	packet.battery_percent = battery_percent;

        memcpy(_MAV_PAYLOAD(msg), &packet, 23);
#endif

    msg->msgid = MAVLINK_MSG_ID_SYS_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 23, 114);
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
    return mavlink_msg_sys_status_pack(system_id, component_id, msg, sys_status->onboard_control_sensors_present, sys_status->onboard_control_sensors_enabled, sys_status->onboard_control_sensors_health, sys_status->load, sys_status->voltage_battery, sys_status->current_battery, sys_status->watt, sys_status->battery_percent, sys_status->errors_uart, sys_status->errors_i2c, sys_status->errors_spi, sys_status->errors_can);
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
 * @param current_battery Battery current, in milliamperes (1 = 1 milliampere)
 * @param watt Watts consumed from this battery since startup
 * @param battery_percent Remaining battery energy: (0%: 0, 100%: 255)
 * @param errors_uart Dropped packets on all links (packets that were corrupted on reception on the MAV)
 * @param errors_i2c Dropped packets on all links (packets that were corrupted on reception)
 * @param errors_spi Dropped packets on all links (packets that were corrupted on reception)
 * @param errors_can Dropped packets on all links (packets that were corrupted on reception)
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_sys_status_send(mavlink_channel_t chan, UInt16 public onboard_control_sensors_present, UInt16 public onboard_control_sensors_enabled, UInt16 public onboard_control_sensors_health, UInt16 public load, UInt16 public voltage_battery, UInt16 public current_battery, UInt16 public watt, byte public battery_percent, UInt16 public errors_uart, UInt16 public errors_i2c, UInt16 public errors_spi, UInt16 public errors_can)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[23];
	_mav_put_UInt16(buf, 0, onboard_control_sensors_present);
	_mav_put_UInt16(buf, 2, onboard_control_sensors_enabled);
	_mav_put_UInt16(buf, 4, onboard_control_sensors_health);
	_mav_put_UInt16(buf, 6, load);
	_mav_put_UInt16(buf, 8, voltage_battery);
	_mav_put_UInt16(buf, 10, current_battery);
	_mav_put_UInt16(buf, 12, watt);
	_mav_put_UInt16(buf, 14, errors_uart);
	_mav_put_UInt16(buf, 16, errors_i2c);
	_mav_put_UInt16(buf, 18, errors_spi);
	_mav_put_UInt16(buf, 20, errors_can);
	_mav_put_byte(buf, 22, battery_percent);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SYS_STATUS, buf, 23, 114);
#else
    mavlink_sys_status_t packet;
	packet.onboard_control_sensors_present = onboard_control_sensors_present;
	packet.onboard_control_sensors_enabled = onboard_control_sensors_enabled;
	packet.onboard_control_sensors_health = onboard_control_sensors_health;
	packet.load = load;
	packet.voltage_battery = voltage_battery;
	packet.current_battery = current_battery;
	packet.watt = watt;
	packet.errors_uart = errors_uart;
	packet.errors_i2c = errors_i2c;
	packet.errors_spi = errors_spi;
	packet.errors_can = errors_can;
	packet.battery_percent = battery_percent;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SYS_STATUS, (const char *)&packet, 23, 114);
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
public static UInt16 mavlink_msg_sys_status_get_onboard_control_sensors_present(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  0);
}

/**
 * @brief Get field onboard_control_sensors_enabled from sys_status message
 *
 * @return Bitmask showing which onboard controllers and sensors are enabled:  Value of 0: not enabled. Value of 1: enabled. Indices: 0: 3D gyro, 1: 3D acc, 2: 3D mag, 3: absolute pressure, 4: differential pressure, 5: GPS, 6: optical flow, 7: computer vision position, 8: laser based position, 9: external ground-truth (Vicon or Leica). Controllers: 10: 3D angular rate control 11: attitude stabilization, 12: yaw position, 13: z/altitude control, 14: x/y position control, 15: motor outputs / control
 */
public static UInt16 mavlink_msg_sys_status_get_onboard_control_sensors_enabled(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  2);
}

/**
 * @brief Get field onboard_control_sensors_health from sys_status message
 *
 * @return Bitmask showing which onboard controllers and sensors are operational or have an error:  Value of 0: not enabled. Value of 1: enabled. Indices: 0: 3D gyro, 1: 3D acc, 2: 3D mag, 3: absolute pressure, 4: differential pressure, 5: GPS, 6: optical flow, 7: computer vision position, 8: laser based position, 9: external ground-truth (Vicon or Leica). Controllers: 10: 3D angular rate control 11: attitude stabilization, 12: yaw position, 13: z/altitude control, 14: x/y position control, 15: motor outputs / control
 */
public static UInt16 mavlink_msg_sys_status_get_onboard_control_sensors_health(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  4);
}

/**
 * @brief Get field load from sys_status message
 *
 * @return Maximum usage in percent of the mainloop time, (0%: 0, 100%: 1000) should be always below 1000
 */
public static UInt16 mavlink_msg_sys_status_get_load(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  6);
}

/**
 * @brief Get field voltage_battery from sys_status message
 *
 * @return Battery voltage, in millivolts (1 = 1 millivolt)
 */
public static UInt16 mavlink_msg_sys_status_get_voltage_battery(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  8);
}

/**
 * @brief Get field current_battery from sys_status message
 *
 * @return Battery current, in milliamperes (1 = 1 milliampere)
 */
public static UInt16 mavlink_msg_sys_status_get_current_battery(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  10);
}

/**
 * @brief Get field watt from sys_status message
 *
 * @return Watts consumed from this battery since startup
 */
public static UInt16 mavlink_msg_sys_status_get_watt(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  12);
}

/**
 * @brief Get field battery_percent from sys_status message
 *
 * @return Remaining battery energy: (0%: 0, 100%: 255)
 */
public static byte mavlink_msg_sys_status_get_battery_percent(byte[] msg)
{
    return getByte(msg,  22);
}

/**
 * @brief Get field errors_uart from sys_status message
 *
 * @return Dropped packets on all links (packets that were corrupted on reception on the MAV)
 */
public static UInt16 mavlink_msg_sys_status_get_errors_uart(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  14);
}

/**
 * @brief Get field errors_i2c from sys_status message
 *
 * @return Dropped packets on all links (packets that were corrupted on reception)
 */
public static UInt16 mavlink_msg_sys_status_get_errors_i2c(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  16);
}

/**
 * @brief Get field errors_spi from sys_status message
 *
 * @return Dropped packets on all links (packets that were corrupted on reception)
 */
public static UInt16 mavlink_msg_sys_status_get_errors_spi(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  18);
}

/**
 * @brief Get field errors_can from sys_status message
 *
 * @return Dropped packets on all links (packets that were corrupted on reception)
 */
public static UInt16 mavlink_msg_sys_status_get_errors_can(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  20);
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
	sys_status.watt = mavlink_msg_sys_status_get_watt(msg);
	sys_status.errors_uart = mavlink_msg_sys_status_get_errors_uart(msg);
	sys_status.errors_i2c = mavlink_msg_sys_status_get_errors_i2c(msg);
	sys_status.errors_spi = mavlink_msg_sys_status_get_errors_spi(msg);
	sys_status.errors_can = mavlink_msg_sys_status_get_errors_can(msg);
	sys_status.battery_percent = mavlink_msg_sys_status_get_battery_percent(msg);
} else {
    int len = 23; //Marshal.SizeOf(sys_status);
    IntPtr i = Marshal.AllocHGlobal(len);
    Marshal.Copy(msg, 0, i, len);
    sys_status = (mavlink_sys_status_t)Marshal.PtrToStructure(i, ((object)sys_status).GetType());
    Marshal.FreeHGlobal(i);
}
}

}
