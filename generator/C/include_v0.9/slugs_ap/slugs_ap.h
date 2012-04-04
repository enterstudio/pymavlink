/** @file
 *	@brief MAVLink comm protocol generated from slugs_ap.xml
 *	@see http://qgroundcontrol.org/mavlink/
 */
#ifndef SLUGS_AP_H
#define SLUGS_AP_H

#ifdef __cplusplus
extern "C" {
#endif

// MESSAGE LENGTHS AND CRCS

#ifndef MAVLINK_MESSAGE_LENGTHS
#define MAVLINK_MESSAGE_LENGTHS {2, 4, 8, 14, 0, 0, 0, 0, 0, 0, 3, 2, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 101, 26, 16, 32, 32, 37, 0, 8, 0, 0, 0, 0, 36, 4, 4, 2, 0, 4, 2, 2, 3, 34, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4, 10, 24, 18, 10, 20, 30, 24, 36, 7, 13, 3, 14, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 51, 5}
#endif

#ifndef MAVLINK_MESSAGE_CRCS
#define MAVLINK_MESSAGE_CRCS {115, 39, 190, 92, 0, 0, 0, 0, 0, 0, 60, 186, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 110, 179, 136, 66, 126, 185, 0, 146, 0, 0, 0, 0, 171, 9, 106, 101, 0, 4, 229, 21, 214, 147, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 219, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 150, 232, 168, 2, 51, 24, 120, 167, 135, 16, 2, 52, 233, 202, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 106, 7}
#endif

#ifndef MAVLINK_MESSAGE_INFO
#define MAVLINK_MESSAGE_INFO {MAVLINK_MESSAGE_INFO_HEARTBEAT, MAVLINK_MESSAGE_INFO_BOOT, MAVLINK_MESSAGE_INFO_SYSTEM_TIME, MAVLINK_MESSAGE_INFO_PING, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, MAVLINK_MESSAGE_INFO_ACTION, MAVLINK_MESSAGE_INFO_SET_MODE, MAVLINK_MESSAGE_INFO_SET_NAV_MODE, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, MAVLINK_MESSAGE_INFO_GPS_STATUS, MAVLINK_MESSAGE_INFO_RAW_IMU, MAVLINK_MESSAGE_INFO_RAW_PRESSURE, MAVLINK_MESSAGE_INFO_ATTITUDE, MAVLINK_MESSAGE_INFO_LOCAL_POSITION, MAVLINK_MESSAGE_INFO_GPS_RAW, {NULL}, MAVLINK_MESSAGE_INFO_SYS_STATUS, {NULL}, {NULL}, {NULL}, {NULL}, MAVLINK_MESSAGE_INFO_WAYPOINT, MAVLINK_MESSAGE_INFO_WAYPOINT_REQUEST, MAVLINK_MESSAGE_INFO_WAYPOINT_SET_CURRENT, MAVLINK_MESSAGE_INFO_WAYPOINT_CURRENT, {NULL}, MAVLINK_MESSAGE_INFO_WAYPOINT_COUNT, MAVLINK_MESSAGE_INFO_WAYPOINT_CLEAR_ALL, MAVLINK_MESSAGE_INFO_WAYPOINT_REACHED, MAVLINK_MESSAGE_INFO_WAYPOINT_ACK, MAVLINK_MESSAGE_INFO_WAYPOINT_SET_GLOBAL_REFERENCE, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, MAVLINK_MESSAGE_INFO_ACTION_ACK, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, MAVLINK_MESSAGE_INFO_CPU_LOAD, MAVLINK_MESSAGE_INFO_AIR_DATA, MAVLINK_MESSAGE_INFO_SENSOR_BIAS, MAVLINK_MESSAGE_INFO_DIAGNOSTIC, MAVLINK_MESSAGE_INFO_PILOT_CONSOLE, MAVLINK_MESSAGE_INFO_PWM_COMMANDS, MAVLINK_MESSAGE_INFO_SLUGS_NAVIGATION, MAVLINK_MESSAGE_INFO_DATA_LOG, MAVLINK_MESSAGE_INFO_FILTERED_DATA, MAVLINK_MESSAGE_INFO_GPS_DATE_TIME, MAVLINK_MESSAGE_INFO_MID_LVL_CMDS, MAVLINK_MESSAGE_INFO_CTRL_SRFC_PT, MAVLINK_MESSAGE_INFO_PID, MAVLINK_MESSAGE_INFO_SLUGS_ACTION, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, MAVLINK_MESSAGE_INFO_STATUSTEXT, MAVLINK_MESSAGE_INFO_DEBUG}
#endif

#include "../protocol.h"

#define MAVLINK_ENABLED_SLUGS_AP

#include "../common_slugs/common_slugs.h"

// MAVLINK VERSION

#ifndef MAVLINK_VERSION
#define MAVLINK_VERSION 2
#endif

#if (MAVLINK_VERSION == 0)
#undef MAVLINK_VERSION
#define MAVLINK_VERSION 2
#endif

// ENUM DEFINITIONS



// MESSAGE DEFINITIONS
#include "./mavlink_msg_cpu_load.h"
#include "./mavlink_msg_air_data.h"
#include "./mavlink_msg_sensor_bias.h"
#include "./mavlink_msg_diagnostic.h"
#include "./mavlink_msg_pilot_console.h"
#include "./mavlink_msg_pwm_commands.h"
#include "./mavlink_msg_slugs_navigation.h"
#include "./mavlink_msg_data_log.h"
#include "./mavlink_msg_filtered_data.h"
#include "./mavlink_msg_gps_date_time.h"
#include "./mavlink_msg_mid_lvl_cmds.h"
#include "./mavlink_msg_ctrl_srfc_pt.h"
#include "./mavlink_msg_pid.h"
#include "./mavlink_msg_slugs_action.h"

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // SLUGS_AP_H
