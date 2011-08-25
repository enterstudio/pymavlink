#!/usr/bin/env python
'''
parse a MAVLink protocol XML file and generate a C implementation

Copyright Andrew Tridgell 2011
Released under GNU GPL version 3 or later
'''

import sys, textwrap, os, time
import mavparse, mavtemplate

t = mavtemplate.MAVTemplate()

def generate_mavlink_h(directory, xml):
    '''generate mavlink.h'''
    f = open(os.path.join(directory, "mavlink.h"), mode='w')
    t.write(f,'''
/** @file
 *	@brief MAVLink comm protocol built from ${basename}.xml
 *	@see http://pixhawk.ethz.ch/software/mavlink
 *	Generated on ${parse_time}
 */
#ifndef MAVLINK_H
#define MAVLINK_H

#ifndef MAVLINK_STX
#define MAVLINK_STX ${protocol_marker}
#endif

#ifndef MAVLINK_ENDIAN
#define MAVLINK_ENDIAN ${mavlink_endian}
#endif

#ifndef MAVLINK_CRC_EXTRA
#define MAVLINK_CRC_EXTRA ${crc_extra_define}
#endif

#include "${basename}.h"

#endif // MAVLINK_H
''', xml)
    f.close()

def generate_main_h(directory, xml):
    '''generate main header per XML file'''
    f = open(os.path.join(directory, xml.basename + ".h"), mode='w')
    t.write(f, '''
/** @file
 *	@brief MAVLink comm protocol generated from ${basename}.xml
 *	@see http://qgroundcontrol.org/mavlink/
 *	Generated on ${parse_time}
 */
#ifndef ${basename_upper}_H
#define ${basename_upper}_H

#ifdef __cplusplus
extern "C" {
#endif

// MESSAGE LENGTHS AND CRCS

#undef MAVLINK_MESSAGE_LENGTHS
#define MAVLINK_MESSAGE_LENGTHS {${message_lengths_array}}

#undef MAVLINK_MESSAGE_CRCS
#define MAVLINK_MESSAGE_CRCS {${message_crcs_array}}
    

#include "../protocol.h"

#define MAVLINK_ENABLED_${basename_upper}

${include_headers}

// MAVLINK VERSION

#ifndef MAVLINK_VERSION
#define MAVLINK_VERSION ${version}
#endif

#if (MAVLINK_VERSION == 0)
#undef MAVLINK_VERSION
#define MAVLINK_VERSION ${version}
#endif

// ENUM DEFINITIONS

${{enum:
/** @brief ${description} */
enum ${name}
{
${{entry:	${name}=${value}, /* ${description} |${{param:${description}| }} */
}}
};
}}

// MESSAGE DEFINITIONS
${{message:#include "./mavlink_msg_${name_lower}.h"
}}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // ${basename_upper}_H
''', xml)

    f.close()
             

def generate_message_h(directory, m):
    '''generate per-message header for a XML file'''
    f = open(os.path.join(directory, 'mavlink_msg_%s.h' % m.name_lower), mode='w')
    t.write(f, '''
// MESSAGE ${name} PACKING

#define MAVLINK_MSG_ID_${name} ${id}

typedef struct __mavlink_${name_lower}_t
{
${{ordered_fields: ${type} ${name}${array_suffix}; ///< ${description}
}}
} mavlink_${name_lower}_t;

/**
 * @brief Pack a ${name_lower} message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
${{arg_fields: * @param ${name} ${description}
}}
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_${name_lower}_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						      ${{arg_fields: ${array_const}${type} ${array_prefix}${name},}})
{
	msg->msgid = MAVLINK_MSG_ID_${name};

${{ordered_fields:	put_${type}${array_tag}_by_index(${putname}, ${wire_offset}, ${array_arg} MAVLINK_PAYLOAD(msg)); // ${description}
}}

	return mavlink_finalize_message(msg, system_id, component_id, ${wire_length}, ${crc_extra});
}

/**
 * @brief Pack a ${name_lower} message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
${{arg_fields: * @param ${name} ${description}
}}
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_${name_lower}_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           ${{arg_fields:${array_const}${type} ${array_prefix}${name},}})
{
	msg->msgid = MAVLINK_MSG_ID_${name};

${{ordered_fields:	put_${type}${array_tag}_by_index(${putname}, ${wire_offset}, ${array_arg} MAVLINK_PAYLOAD(msg)); // ${description}
}}

	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, ${wire_length}, ${crc_extra});
}

#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

/**
 * @brief Pack a ${name_lower} message on a channel and send
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
${{arg_fields: * @param ${name} ${description}
}}
 */
static inline void mavlink_msg_${name_lower}_pack_chan_send(mavlink_channel_t chan,
							   mavlink_message_t* msg,
						           ${{arg_fields:${array_const}${type} ${array_prefix}${name},}})
{
	msg->msgid = MAVLINK_MSG_ID_${name};

${{ordered_fields:	put_${type}${array_tag}_by_index(${putname}, ${wire_offset}, ${array_arg} MAVLINK_PAYLOAD(msg)); // ${description}
}}

	mavlink_finalize_message_chan_send(msg, chan, ${wire_length}, ${crc_extra});
}
#endif // MAVLINK_USE_CONVENIENCE_FUNCTIONS


/**
 * @brief Encode a ${name_lower} struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param ${name_lower} C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_${name_lower}_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_${name_lower}_t* ${name_lower})
{
	return mavlink_msg_${name_lower}_pack(system_id, component_id, msg,${{arg_fields: ${name_lower}->${name},}});
}

/**
 * @brief Send a ${name_lower} message
 * @param chan MAVLink channel to send the message
 *
${{arg_fields: * @param ${name} ${description}
}}
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_${name_lower}_send(mavlink_channel_t chan,${{arg_fields: ${array_const}${type} ${array_prefix}${name},}})
{
	MAVLINK_ALIGNED_MESSAGE(msg, ${wire_length});
	mavlink_msg_${name_lower}_pack_chan_send(chan, msg,${{arg_fields: ${name},}});
}

#endif

// MESSAGE ${name} UNPACKING

${{fields:
/**
 * @brief Get field ${name} from ${name_lower} message
 *
 * @return ${description}
 */
static inline ${return_type} mavlink_msg_${name_lower}_get_${name}(const mavlink_message_t* msg${get_arg})
{
	return MAVLINK_MSG_RETURN_${type}${array_tag}(msg, ${array_return_arg} ${wire_offset});
}
}}

/**
 * @brief Decode a ${name_lower} message into a struct
 *
 * @param msg The message to decode
 * @param ${name_lower} C-struct to decode the message contents into
 */
static inline void mavlink_msg_${name_lower}_decode(const mavlink_message_t* msg, mavlink_${name_lower}_t* ${name_lower})
{
#if MAVLINK_NEED_BYTE_SWAP
${{ordered_fields:	${decode_left}mavlink_msg_${name_lower}_get_${name}(msg${decode_right});
}}
#else
	memcpy(${name_lower}, MAVLINK_PAYLOAD(msg), ${wire_length});
#endif
}
''', m)
    f.close()


def generate_testsuite_h(directory, xml):
    '''generate testsuite.h per XML file'''
    f = open(os.path.join(directory, xml.basename + "_testsuite.h"), mode='w')
    t.write(f, '''
/** @file
 *	@brief MAVLink comm protocol testsuite generated from ${basename}.xml
 *	@see http://qgroundcontrol.org/mavlink/
 *	Generated on ${parse_time}
 */
#ifndef ${basename_upper}_TESTSUITE_H
#define ${basename_upper}_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

static void mavtest_generate_outputs(mavlink_channel_t chan, uint8_t system_id, uint8_t component_id)
{
	mavlink_message_t msg;
${{message:	mavlink_msg_${name_lower}_send(chan ${{arg_fields:, ${c_test_value} }});
}}
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // ${basename_upper}_TESTSUITE_H
''', xml)

    f.close()


def generate_one(basename, xml):
    '''generate headers for one XML file'''

    directory = os.path.join(basename, xml.basename)

    print("Generating C implementation in directory %s" % directory)
    mavparse.mkdir_p(directory)

    if xml.little_endian:
        xml.mavlink_endian = "MAVLINK_LITTLE_ENDIAN"
    else:
        xml.mavlink_endian = "MAVLINK_BIG_ENDIAN"

    if xml.crc_extra:
        xml.crc_extra_define = "1"
    else:
        xml.crc_extra_define = "0"

    # work out the included headers
    xml.include_headers = ''
    for i in xml.include:
        base = i[:-4]
        xml.include_headers += '#include "../%s/%s.h"\n' % (base, base)

    # form message lengths array
    xml.message_lengths_array = ''
    for mlen in xml.message_lengths:
        xml.message_lengths_array += '%u, ' % mlen
    xml.message_lengths_array = xml.message_lengths_array[:-2]

    # and message CRCs array
    xml.message_crcs_array = ''
    for crc in xml.message_crcs:
        xml.message_crcs_array += '%u, ' % crc
    xml.message_crcs_array = xml.message_crcs_array[:-2]

    # add some extra field attributes for convenience with arrays
    for m in xml.message:
        for f in m.fields:
            if f.array_length is not None:
                f.array_suffix = '[%u]' % f.array_length
                f.array_prefix = '*'
                f.array_tag = '_array'
                f.array_arg = '%u, ' % f.array_length
                f.array_return_arg = '%s, %u, ' % (f.name, f.array_length)
                f.array_const = 'const '
                f.decode_left = ''
                f.decode_right = ', %s->%s' % (m.name_lower, f.name)
                f.return_type = 'uint16_t'
                f.get_arg = ', %s *%s' % (f.type, f.name)
                if f.type == 'int8_t':
                    f.c_test_value = '(int8_t *)"%s"' % f.test_value
                else:
                    f.c_test_value = '"%s"' % f.test_value
            else:
                f.array_suffix = ''
                f.array_prefix = ''
                f.array_tag = ''
                f.array_arg = ''
                f.array_return_arg = ''
                f.array_const = ''
                f.decode_left = "%s->%s = " % (m.name_lower, f.name)
                f.decode_right = ''
                f.get_arg = ''
                f.return_type = f.type
                if f.type == 'char':
                    f.c_test_value = "'%s'" % f.test_value
                elif f.type == 'uint64_t':
                    f.c_test_value = "%sLL" % f.test_value                    
                else:
                    f.c_test_value = f.test_value

    # cope with uint8_t_mavlink_version
    for m in xml.message:
        m.arg_fields = []
        for f in m.fields:
            if not f.omit_arg:
                m.arg_fields.append(f)
                f.putname = f.name
            else:
                f.putname = f.const_value

    generate_mavlink_h(directory, xml)
    generate_main_h(directory, xml)
    for m in xml.message:
        generate_message_h(directory, m)
    generate_testsuite_h(directory, xml)


def generate(basename, xml_list):
    '''generate complete MAVLink C implemenation'''

    for xml in xml_list:
        generate_one(basename, xml)
