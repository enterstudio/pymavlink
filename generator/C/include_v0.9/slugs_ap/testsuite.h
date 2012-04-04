/** @file
 *	@brief MAVLink comm protocol testsuite generated from slugs_ap.xml
 *	@see http://qgroundcontrol.org/mavlink/
 */
#ifndef SLUGS_AP_TESTSUITE_H
#define SLUGS_AP_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL
static void mavlink_test_common_slugs(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_slugs_ap(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_test_common_slugs(system_id, component_id, last_msg);
	mavlink_test_slugs_ap(system_id, component_id, last_msg);
}
#endif

#include "../common_slugs/testsuite.h"


static void mavlink_test_cpu_load(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_cpu_load_t packet_in = {
		5,
	72,
	17339,
	};
	mavlink_cpu_load_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.sensLoad = packet_in.sensLoad;
        	packet1.ctrlLoad = packet_in.ctrlLoad;
        	packet1.batVolt = packet_in.batVolt;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_cpu_load_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_cpu_load_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_cpu_load_pack(system_id, component_id, &msg , packet1.sensLoad , packet1.ctrlLoad , packet1.batVolt );
	mavlink_msg_cpu_load_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_cpu_load_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.sensLoad , packet1.ctrlLoad , packet1.batVolt );
	mavlink_msg_cpu_load_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_cpu_load_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_cpu_load_send(MAVLINK_COMM_1 , packet1.sensLoad , packet1.ctrlLoad , packet1.batVolt );
	mavlink_msg_cpu_load_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_air_data(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_air_data_t packet_in = {
		17.0,
	45.0,
	17651,
	};
	mavlink_air_data_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.dynamicPressure = packet_in.dynamicPressure;
        	packet1.staticPressure = packet_in.staticPressure;
        	packet1.temperature = packet_in.temperature;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_air_data_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_air_data_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_air_data_pack(system_id, component_id, &msg , packet1.dynamicPressure , packet1.staticPressure , packet1.temperature );
	mavlink_msg_air_data_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_air_data_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.dynamicPressure , packet1.staticPressure , packet1.temperature );
	mavlink_msg_air_data_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_air_data_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_air_data_send(MAVLINK_COMM_1 , packet1.dynamicPressure , packet1.staticPressure , packet1.temperature );
	mavlink_msg_air_data_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_sensor_bias(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_sensor_bias_t packet_in = {
		17.0,
	45.0,
	73.0,
	101.0,
	129.0,
	157.0,
	};
	mavlink_sensor_bias_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.axBias = packet_in.axBias;
        	packet1.ayBias = packet_in.ayBias;
        	packet1.azBias = packet_in.azBias;
        	packet1.gxBias = packet_in.gxBias;
        	packet1.gyBias = packet_in.gyBias;
        	packet1.gzBias = packet_in.gzBias;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_sensor_bias_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_sensor_bias_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_sensor_bias_pack(system_id, component_id, &msg , packet1.axBias , packet1.ayBias , packet1.azBias , packet1.gxBias , packet1.gyBias , packet1.gzBias );
	mavlink_msg_sensor_bias_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_sensor_bias_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.axBias , packet1.ayBias , packet1.azBias , packet1.gxBias , packet1.gyBias , packet1.gzBias );
	mavlink_msg_sensor_bias_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_sensor_bias_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_sensor_bias_send(MAVLINK_COMM_1 , packet1.axBias , packet1.ayBias , packet1.azBias , packet1.gxBias , packet1.gyBias , packet1.gzBias );
	mavlink_msg_sensor_bias_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_diagnostic(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_diagnostic_t packet_in = {
		17.0,
	45.0,
	73.0,
	17859,
	17963,
	18067,
	};
	mavlink_diagnostic_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.diagFl1 = packet_in.diagFl1;
        	packet1.diagFl2 = packet_in.diagFl2;
        	packet1.diagFl3 = packet_in.diagFl3;
        	packet1.diagSh1 = packet_in.diagSh1;
        	packet1.diagSh2 = packet_in.diagSh2;
        	packet1.diagSh3 = packet_in.diagSh3;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_diagnostic_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_diagnostic_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_diagnostic_pack(system_id, component_id, &msg , packet1.diagFl1 , packet1.diagFl2 , packet1.diagFl3 , packet1.diagSh1 , packet1.diagSh2 , packet1.diagSh3 );
	mavlink_msg_diagnostic_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_diagnostic_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.diagFl1 , packet1.diagFl2 , packet1.diagFl3 , packet1.diagSh1 , packet1.diagSh2 , packet1.diagSh3 );
	mavlink_msg_diagnostic_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_diagnostic_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_diagnostic_send(MAVLINK_COMM_1 , packet1.diagFl1 , packet1.diagFl2 , packet1.diagFl3 , packet1.diagSh1 , packet1.diagSh2 , packet1.diagSh3 );
	mavlink_msg_diagnostic_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_pilot_console(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_pilot_console_t packet_in = {
		17235,
	17339,
	17443,
	17547,
	17651,
	};
	mavlink_pilot_console_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.dt = packet_in.dt;
        	packet1.dla = packet_in.dla;
        	packet1.dra = packet_in.dra;
        	packet1.dr = packet_in.dr;
        	packet1.de = packet_in.de;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_pilot_console_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_pilot_console_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_pilot_console_pack(system_id, component_id, &msg , packet1.dt , packet1.dla , packet1.dra , packet1.dr , packet1.de );
	mavlink_msg_pilot_console_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_pilot_console_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.dt , packet1.dla , packet1.dra , packet1.dr , packet1.de );
	mavlink_msg_pilot_console_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_pilot_console_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_pilot_console_send(MAVLINK_COMM_1 , packet1.dt , packet1.dla , packet1.dra , packet1.dr , packet1.de );
	mavlink_msg_pilot_console_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_pwm_commands(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_pwm_commands_t packet_in = {
		17235,
	17339,
	17443,
	17547,
	17651,
	17755,
	17859,
	17963,
	18067,
	18171,
	};
	mavlink_pwm_commands_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.dt_c = packet_in.dt_c;
        	packet1.dla_c = packet_in.dla_c;
        	packet1.dra_c = packet_in.dra_c;
        	packet1.dr_c = packet_in.dr_c;
        	packet1.dle_c = packet_in.dle_c;
        	packet1.dre_c = packet_in.dre_c;
        	packet1.dlf_c = packet_in.dlf_c;
        	packet1.drf_c = packet_in.drf_c;
        	packet1.aux1 = packet_in.aux1;
        	packet1.aux2 = packet_in.aux2;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_pwm_commands_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_pwm_commands_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_pwm_commands_pack(system_id, component_id, &msg , packet1.dt_c , packet1.dla_c , packet1.dra_c , packet1.dr_c , packet1.dle_c , packet1.dre_c , packet1.dlf_c , packet1.drf_c , packet1.aux1 , packet1.aux2 );
	mavlink_msg_pwm_commands_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_pwm_commands_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.dt_c , packet1.dla_c , packet1.dra_c , packet1.dr_c , packet1.dle_c , packet1.dre_c , packet1.dlf_c , packet1.drf_c , packet1.aux1 , packet1.aux2 );
	mavlink_msg_pwm_commands_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_pwm_commands_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_pwm_commands_send(MAVLINK_COMM_1 , packet1.dt_c , packet1.dla_c , packet1.dra_c , packet1.dr_c , packet1.dle_c , packet1.dre_c , packet1.dlf_c , packet1.drf_c , packet1.aux1 , packet1.aux2 );
	mavlink_msg_pwm_commands_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_slugs_navigation(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_slugs_navigation_t packet_in = {
		17.0,
	45.0,
	73.0,
	101.0,
	129.0,
	157.0,
	185.0,
	89,
	156,
	};
	mavlink_slugs_navigation_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.u_m = packet_in.u_m;
        	packet1.phi_c = packet_in.phi_c;
        	packet1.theta_c = packet_in.theta_c;
        	packet1.psiDot_c = packet_in.psiDot_c;
        	packet1.ay_body = packet_in.ay_body;
        	packet1.totalDist = packet_in.totalDist;
        	packet1.dist2Go = packet_in.dist2Go;
        	packet1.fromWP = packet_in.fromWP;
        	packet1.toWP = packet_in.toWP;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_slugs_navigation_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_slugs_navigation_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_slugs_navigation_pack(system_id, component_id, &msg , packet1.u_m , packet1.phi_c , packet1.theta_c , packet1.psiDot_c , packet1.ay_body , packet1.totalDist , packet1.dist2Go , packet1.fromWP , packet1.toWP );
	mavlink_msg_slugs_navigation_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_slugs_navigation_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.u_m , packet1.phi_c , packet1.theta_c , packet1.psiDot_c , packet1.ay_body , packet1.totalDist , packet1.dist2Go , packet1.fromWP , packet1.toWP );
	mavlink_msg_slugs_navigation_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_slugs_navigation_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_slugs_navigation_send(MAVLINK_COMM_1 , packet1.u_m , packet1.phi_c , packet1.theta_c , packet1.psiDot_c , packet1.ay_body , packet1.totalDist , packet1.dist2Go , packet1.fromWP , packet1.toWP );
	mavlink_msg_slugs_navigation_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_data_log(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_data_log_t packet_in = {
		17.0,
	45.0,
	73.0,
	101.0,
	129.0,
	157.0,
	};
	mavlink_data_log_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.fl_1 = packet_in.fl_1;
        	packet1.fl_2 = packet_in.fl_2;
        	packet1.fl_3 = packet_in.fl_3;
        	packet1.fl_4 = packet_in.fl_4;
        	packet1.fl_5 = packet_in.fl_5;
        	packet1.fl_6 = packet_in.fl_6;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_data_log_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_data_log_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_data_log_pack(system_id, component_id, &msg , packet1.fl_1 , packet1.fl_2 , packet1.fl_3 , packet1.fl_4 , packet1.fl_5 , packet1.fl_6 );
	mavlink_msg_data_log_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_data_log_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.fl_1 , packet1.fl_2 , packet1.fl_3 , packet1.fl_4 , packet1.fl_5 , packet1.fl_6 );
	mavlink_msg_data_log_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_data_log_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_data_log_send(MAVLINK_COMM_1 , packet1.fl_1 , packet1.fl_2 , packet1.fl_3 , packet1.fl_4 , packet1.fl_5 , packet1.fl_6 );
	mavlink_msg_data_log_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_filtered_data(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_filtered_data_t packet_in = {
		17.0,
	45.0,
	73.0,
	101.0,
	129.0,
	157.0,
	185.0,
	213.0,
	241.0,
	};
	mavlink_filtered_data_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.aX = packet_in.aX;
        	packet1.aY = packet_in.aY;
        	packet1.aZ = packet_in.aZ;
        	packet1.gX = packet_in.gX;
        	packet1.gY = packet_in.gY;
        	packet1.gZ = packet_in.gZ;
        	packet1.mX = packet_in.mX;
        	packet1.mY = packet_in.mY;
        	packet1.mZ = packet_in.mZ;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_filtered_data_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_filtered_data_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_filtered_data_pack(system_id, component_id, &msg , packet1.aX , packet1.aY , packet1.aZ , packet1.gX , packet1.gY , packet1.gZ , packet1.mX , packet1.mY , packet1.mZ );
	mavlink_msg_filtered_data_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_filtered_data_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.aX , packet1.aY , packet1.aZ , packet1.gX , packet1.gY , packet1.gZ , packet1.mX , packet1.mY , packet1.mZ );
	mavlink_msg_filtered_data_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_filtered_data_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_filtered_data_send(MAVLINK_COMM_1 , packet1.aX , packet1.aY , packet1.aZ , packet1.gX , packet1.gY , packet1.gZ , packet1.mX , packet1.mY , packet1.mZ );
	mavlink_msg_filtered_data_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_gps_date_time(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_gps_date_time_t packet_in = {
		5,
	72,
	139,
	206,
	17,
	84,
	151,
	};
	mavlink_gps_date_time_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.year = packet_in.year;
        	packet1.month = packet_in.month;
        	packet1.day = packet_in.day;
        	packet1.hour = packet_in.hour;
        	packet1.min = packet_in.min;
        	packet1.sec = packet_in.sec;
        	packet1.visSat = packet_in.visSat;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_gps_date_time_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_gps_date_time_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_gps_date_time_pack(system_id, component_id, &msg , packet1.year , packet1.month , packet1.day , packet1.hour , packet1.min , packet1.sec , packet1.visSat );
	mavlink_msg_gps_date_time_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_gps_date_time_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.year , packet1.month , packet1.day , packet1.hour , packet1.min , packet1.sec , packet1.visSat );
	mavlink_msg_gps_date_time_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_gps_date_time_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_gps_date_time_send(MAVLINK_COMM_1 , packet1.year , packet1.month , packet1.day , packet1.hour , packet1.min , packet1.sec , packet1.visSat );
	mavlink_msg_gps_date_time_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_mid_lvl_cmds(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_mid_lvl_cmds_t packet_in = {
		5,
	24.0,
	52.0,
	80.0,
	};
	mavlink_mid_lvl_cmds_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.target = packet_in.target;
        	packet1.hCommand = packet_in.hCommand;
        	packet1.uCommand = packet_in.uCommand;
        	packet1.rCommand = packet_in.rCommand;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_mid_lvl_cmds_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_mid_lvl_cmds_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_mid_lvl_cmds_pack(system_id, component_id, &msg , packet1.target , packet1.hCommand , packet1.uCommand , packet1.rCommand );
	mavlink_msg_mid_lvl_cmds_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_mid_lvl_cmds_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target , packet1.hCommand , packet1.uCommand , packet1.rCommand );
	mavlink_msg_mid_lvl_cmds_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_mid_lvl_cmds_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_mid_lvl_cmds_send(MAVLINK_COMM_1 , packet1.target , packet1.hCommand , packet1.uCommand , packet1.rCommand );
	mavlink_msg_mid_lvl_cmds_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_ctrl_srfc_pt(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_ctrl_srfc_pt_t packet_in = {
		5,
	17287,
	};
	mavlink_ctrl_srfc_pt_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.target = packet_in.target;
        	packet1.bitfieldPt = packet_in.bitfieldPt;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_ctrl_srfc_pt_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_ctrl_srfc_pt_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_ctrl_srfc_pt_pack(system_id, component_id, &msg , packet1.target , packet1.bitfieldPt );
	mavlink_msg_ctrl_srfc_pt_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_ctrl_srfc_pt_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target , packet1.bitfieldPt );
	mavlink_msg_ctrl_srfc_pt_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_ctrl_srfc_pt_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_ctrl_srfc_pt_send(MAVLINK_COMM_1 , packet1.target , packet1.bitfieldPt );
	mavlink_msg_ctrl_srfc_pt_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_pid(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_pid_t packet_in = {
		5,
	24.0,
	52.0,
	80.0,
	108,
	};
	mavlink_pid_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.target = packet_in.target;
        	packet1.pVal = packet_in.pVal;
        	packet1.iVal = packet_in.iVal;
        	packet1.dVal = packet_in.dVal;
        	packet1.idx = packet_in.idx;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_pid_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_pid_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_pid_pack(system_id, component_id, &msg , packet1.target , packet1.pVal , packet1.iVal , packet1.dVal , packet1.idx );
	mavlink_msg_pid_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_pid_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target , packet1.pVal , packet1.iVal , packet1.dVal , packet1.idx );
	mavlink_msg_pid_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_pid_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_pid_send(MAVLINK_COMM_1 , packet1.target , packet1.pVal , packet1.iVal , packet1.dVal , packet1.idx );
	mavlink_msg_pid_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_slugs_action(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_slugs_action_t packet_in = {
		5,
	72,
	17339,
	};
	mavlink_slugs_action_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.target = packet_in.target;
        	packet1.actionId = packet_in.actionId;
        	packet1.actionVal = packet_in.actionVal;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_slugs_action_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_slugs_action_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_slugs_action_pack(system_id, component_id, &msg , packet1.target , packet1.actionId , packet1.actionVal );
	mavlink_msg_slugs_action_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_slugs_action_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target , packet1.actionId , packet1.actionVal );
	mavlink_msg_slugs_action_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_slugs_action_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_slugs_action_send(MAVLINK_COMM_1 , packet1.target , packet1.actionId , packet1.actionVal );
	mavlink_msg_slugs_action_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_slugs_ap(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_test_cpu_load(system_id, component_id, last_msg);
	mavlink_test_air_data(system_id, component_id, last_msg);
	mavlink_test_sensor_bias(system_id, component_id, last_msg);
	mavlink_test_diagnostic(system_id, component_id, last_msg);
	mavlink_test_pilot_console(system_id, component_id, last_msg);
	mavlink_test_pwm_commands(system_id, component_id, last_msg);
	mavlink_test_slugs_navigation(system_id, component_id, last_msg);
	mavlink_test_data_log(system_id, component_id, last_msg);
	mavlink_test_filtered_data(system_id, component_id, last_msg);
	mavlink_test_gps_date_time(system_id, component_id, last_msg);
	mavlink_test_mid_lvl_cmds(system_id, component_id, last_msg);
	mavlink_test_ctrl_srfc_pt(system_id, component_id, last_msg);
	mavlink_test_pid(system_id, component_id, last_msg);
	mavlink_test_slugs_action(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // SLUGS_AP_TESTSUITE_H
