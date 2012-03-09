#!/usr/bin/env python
'''
parse a MAVLink protocol XML file and generate a C implementation

Copyright Michael Oborne 2011
Released under GNU GPL version 3 or later
'''

import sys, textwrap, os, time
import mavparse, mavtemplate

t = mavtemplate.MAVTemplate()



def generate_version_h(directory, xml):
    '''generate version.h'''
    f = open(os.path.join(directory, "version.cs"), mode='w')
    t.write(f,'''
/** @file
 *	@brief MAVLink comm protocol built from ${basename}.xml
 *	@see http://pixhawk.ethz.ch/software/mavlink
 */
 
 using System;

public partial class Mavlink
{
    public const string MAVLINK_BUILD_DATE = "${parse_time}";
    public const string MAVLINK_WIRE_PROTOCOL_VERSION = "${wire_protocol_version}";
    public const int MAVLINK_MAX_DIALECT_PAYLOAD_SIZE = ${largest_payload};
}
''', xml)
    f.close()

def generate_mavlink_h(directory, xml):
    '''generate mavlink.h'''
    f = open(os.path.join(directory, "mavlink.cs"), mode='w')
    t.write(f,'''
/** @file
 *	@brief MAVLink comm protocol built from ${basename}.xml
 *	@see http://pixhawk.ethz.ch/software/mavlink
 */
 using System;
 
public partial class Mavlink
{
    public const int MAVLINK_LITTLE_ENDIAN = 1;

    public const byte MAVLINK_STX = ${protocol_marker};

    public const byte MAVLINK_ENDIAN = ${mavlink_endian};

    public const bool MAVLINK_ALIGNED_FIELDS = (${aligned_fields_define} == 1);

    public const byte MAVLINK_CRC_EXTRA = ${crc_extra_define};
    
    public const bool MAVLINK_NEED_BYTE_SWAP = (MAVLINK_ENDIAN == MAVLINK_LITTLE_ENDIAN);
    
    public byte packetcount = 0;
}
''', xml)
    f.close()

def generate_main_h(directory, xml):
    '''generate main header per XML file'''
    f = open(os.path.join(directory, xml.basename + ".cs"), mode='w')
    t.write(f, '''
/** @file
 *	@brief MAVLink comm protocol generated from ${basename}.xml
 *	@see http://qgroundcontrol.org/mavlink/
 */
 using System;
 using System.Runtime.InteropServices;

// MESSAGE LENGTHS AND CRCS

public partial class Mavlink
{

    public byte[] MAVLINK_MESSAGE_LENGTHS = new byte[] {${message_lengths_array}};

    public byte[] MAVLINK_MESSAGE_CRCS = new byte[] {${message_crcs_array}};

    public Type[] MAVLINK_MESSAGE_INFO = new Type[] {${message_info_array}};

    public const byte MAVLINK_VERSION = ${version};
    
    public static byte getByte(byte[] msg, int offset) 
    {
        return msg[offset];
    }
    
    public static byte[] getBytes(byte[] msg, int length, int offset)
    {
        byte[] temp = new byte[length];
        Array.Copy(msg, offset, temp, 0, length);
        return temp;
    }
    
    public static char[] toArray(string text)
    {
        return text.ToCharArray();
    }
    
    public static byte[] toArray(byte[] data)
    {
        return data;
    }

    const int X25_INIT_CRC = 0xffff;
    const int X25_VALIDATE_CRC = 0xf0b8;

    ushort crc_accumulate(byte b, ushort crc)
    {
        unchecked
        {
            byte ch = (byte)(b ^ (byte)(crc & 0x00ff));
            ch = (byte)(ch ^ (ch << 4));
            return (ushort)((crc >> 8) ^ (ch << 8) ^ (ch << 3) ^ (ch >> 4));
        }
    }

    ushort crc_calculate(byte[] pBuffer, int length)
    {
        if (length < 1)
        {
            return 0xffff;
        }
        // For a "message" of length bytes contained in the unsigned char array
        // pointed to by pBuffer, calculate the CRC
        // crcCalculate(unsigned char* pBuffer, int length, unsigned short* checkConst) < not needed

        ushort crcTmp;
        int i;

        crcTmp = X25_INIT_CRC;

        for (i = 1; i < length; i++) // skips header U
        {
            crcTmp = crc_accumulate(pBuffer[i], crcTmp);
            //Console.WriteLine(crcTmp + " " + pBuffer[i] + " " + length);
        }

        return (crcTmp);
    }
    
    public byte[] generatePacket(object indata, byte sysid, byte compid)
    {
        byte messageType = 0;
        foreach (Type ty in MAVLINK_MESSAGE_INFO)
        {
            if (ty == indata.GetType()) {
                break;
            }
            messageType++;
        }
        
        byte[] data;
        
        if (MAVLINK_ENDIAN == MAVLINK_LITTLE_ENDIAN)
        {
            data = StructureToByteArray(indata);
        }
        else
        {
            data = StructureToByteArrayEndian(indata);
        }
        
        byte[] packet = new byte[data.Length + 6 + 2];
        
        packet[0] = (byte)MAVLINK_STX;
        packet[1] = (byte)data.Length;
        packet[2] = packetcount;
        packet[3] = sysid;
        packet[4] = compid;
        packet[5] = messageType;
        
        int i = 6;
        foreach (byte b in data)
        {
            packet[i] = b;
            i++;
        }
        
        
        ushort checksum = crc_calculate(packet, packet[1] + 6);

        if (MAVLINK_CRC_EXTRA == 1)
        {
            checksum = crc_accumulate(MAVLINK_MESSAGE_CRCS[messageType], checksum);
        }

        byte ck_a = (byte)(checksum & 0xFF); ///< High byte
        byte ck_b = (byte)(checksum >> 8); ///< Low byte

        packet[i] = ck_a;
        i += 1;
        packet[i] = ck_b;
        i += 1;
        
        return packet;
    }
    
    byte[] StructureToByteArrayEndian(params object[] list)
    {
        // The copy is made becuase SetValue won't work on a struct.
        // Boxing was used because SetValue works on classes/objects.
        // Unfortunately, it results in 2 copy operations.
        object thisBoxed = list[0]; // Why make a copy?
        Type test = thisBoxed.GetType();

        int offset = 0;
        byte[] data = new byte[Marshal.SizeOf(thisBoxed)];

        // System.Net.IPAddress.NetworkToHostOrder is used to perform byte swapping.
        // To convert unsigned to signed, 'unchecked()' was used.
        // See http://stackoverflow.com/questions/1131843/how-do-i-convert-uint-to-int-in-c

        object fieldValue;
        TypeCode typeCode;

        byte[] temp;

        // Enumerate each structure field using reflection.
        foreach (var field in test.GetFields())
        {
            // field.Name has the field's name.

            fieldValue = field.GetValue(thisBoxed); // Get value

            // Get the TypeCode enumeration. Multiple types get mapped to a common typecode.
            typeCode = Type.GetTypeCode(fieldValue.GetType());

            switch (typeCode)
            {
                case TypeCode.Single: // float
                    {
                        temp = BitConverter.GetBytes((Single)fieldValue);
                        Array.Reverse(temp);
                        Array.Copy(temp, 0, data, offset, sizeof(Single));
                        break;
                    }
                case TypeCode.Int32:
                    {
                        temp = BitConverter.GetBytes((Int32)fieldValue);
                        Array.Reverse(temp);
                        Array.Copy(temp, 0, data, offset, sizeof(Int32));
                        break;
                    }
                case TypeCode.UInt32:
                    {
                        temp = BitConverter.GetBytes((UInt32)fieldValue);
                        Array.Reverse(temp);
                        Array.Copy(temp, 0, data, offset, sizeof(UInt32));
                        break;
                    }
                case TypeCode.Int16:
                    {
                        temp = BitConverter.GetBytes((Int16)fieldValue);
                        Array.Reverse(temp);
                        Array.Copy(temp, 0, data, offset, sizeof(Int16));
                        break;
                    }
                case TypeCode.UInt16:
                    {
                        temp = BitConverter.GetBytes((UInt16)fieldValue);
                        Array.Reverse(temp);
                        Array.Copy(temp, 0, data, offset, sizeof(UInt16));
                        break;
                    }
                case TypeCode.Int64:
                    {
                        temp = BitConverter.GetBytes((Int64)fieldValue);
                        Array.Reverse(temp);
                        Array.Copy(temp, 0, data, offset, sizeof(Int64));
                        break;
                    }
                case TypeCode.UInt64:
                    {
                        temp = BitConverter.GetBytes((UInt64)fieldValue);
                        Array.Reverse(temp);
                        Array.Copy(temp, 0, data, offset, sizeof(UInt64));
                        break;
                    }
                case TypeCode.Double:
                    {
                        temp = BitConverter.GetBytes((Double)fieldValue);
                        Array.Reverse(temp);
                        Array.Copy(temp, 0, data, offset, sizeof(Double));
                        break;
                    }
                case TypeCode.Byte:
                    {
                        data[offset] = (Byte)fieldValue;
                        break;
                    }
                default:
                    {
                        //System.Diagnostics.Debug.Fail("No conversion provided for this type : " + typeCode.ToString());
                        break;
                    }
            }; // switch
            if (typeCode == TypeCode.Object)
            {
                int length = ((byte[])fieldValue).Length;
                Array.Copy(((byte[])fieldValue), 0, data, offset, length);
                offset += length;
            }
            else
            {
                offset += Marshal.SizeOf(fieldValue);
            }
        } // foreach

        return data;
    } // Swap
    
    byte[] StructureToByteArray(object obj)
    {

        int len = Marshal.SizeOf(obj);

        byte[] arr = new byte[len];

        IntPtr ptr = Marshal.AllocHGlobal(len);

        Marshal.StructureToPtr(obj, ptr, true);

        Marshal.Copy(ptr, arr, 0, len);

        Marshal.FreeHGlobal(ptr);

        return arr;

    }
    
// ENUM DEFINITIONS

public struct mavlink_message_t {
	public byte magic;   ///< protocol magic marker
	public byte len;     ///< Length of payload
	public byte seq;     ///< Sequence of packet
	public byte sysid;   ///< ID of message sender system/aircraft
	public byte compid;  ///< ID of the message sender component
	public byte msgid;   ///< ID of message in payload
	public byte[] payload;
    public UInt16 checksum; /// sent at end of packet
};

${{enum:
/** @brief ${description} */
    public enum ${name}
    {
${{entry:	///<summary> ${description} |${{param:${description}| }} </summary>
        ${name}=${value}, 
    }}
    };
}}

}
''', xml)

    f.close()
             

def generate_message_h(directory, m):
    '''generate per-message header for a XML file'''
    f = open(os.path.join(directory, 'mavlink_msg_%s.cs' % m.name_lower), mode='w')
    t.write(f, '''
// MESSAGE ${name} PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_${name} = ${id};

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_${name_lower}_t
    {
${{ordered_fields:        /// <summary>
        /// ${description}
        /// </summary>
        ${array_prefix} ${type} ${name}${array_suffix};
    }}
    };

/// <summary>
/// * @brief Pack a ${name_lower} message
/// * @param system_id ID of this system
/// * @param component_id ID of this component (e.g. 200 for IMU)
/// * @param msg The MAVLink message to compress the data into
/// *
${{arg_fields:/// * @param ${name} ${description}
}}
/// * @return length of the message in bytes (excluding serial stream start sign)
/// </summary>
 
public static UInt16 mavlink_msg_${name_lower}_pack(byte system_id, byte component_id, byte[] msg,
                              ${{arg_fields: ${type} ${name},}})
{
if (MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS) {
${{scalar_fields:	Array.Copy(BitConverter.GetBytes(${putname}),0,msg,${wire_offset},sizeof(${type}));
}}
${{array_fields:	Array.Copy(toArray(${name}),0,msg,${wire_offset},${array_length});
}}
} else {
    mavlink_${name_lower}_t packet = new mavlink_${name_lower}_t();
${{scalar_fields:	packet.${name} = ${putname};
}}
${{array_fields:	packet.${name} = ${putname};
}}
        
        int len = ${wire_length};
        msg = new byte[len];
        IntPtr ptr = Marshal.AllocHGlobal(len);
        Marshal.StructureToPtr(packet, ptr, true);
        Marshal.Copy(ptr, msg, 0, len);
        Marshal.FreeHGlobal(ptr);
}

    //msg.msgid = MAVLINK_MSG_ID_${name};
    //return mavlink_finalize_message(msg, system_id, component_id, ${wire_length}${crc_extra_arg});
    return 0;
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
 /*
static inline uint16_t mavlink_msg_${name_lower}_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   ${{arg_fields:${array_const}${type} ${array_prefix}${name},}})
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[${wire_length}];
${{scalar_fields:	_mav_put_${type}(buf, ${wire_offset}, ${putname});
}}
${{array_fields:	_mav_put_${type}_array(buf, ${wire_offset}, ${name}, ${array_length});
}}
        memcpy(_MAV_PAYLOAD(msg), buf, ${wire_length});
#else
    mavlink_${name_lower}_t packet;
${{scalar_fields:	packet.${name} = ${putname};
}}
${{array_fields:	memcpy(packet.${name}, ${name}, sizeof(${type})*${array_length});
}}
        memcpy(_MAV_PAYLOAD(msg), &packet, ${wire_length});
#endif

    msg->msgid = MAVLINK_MSG_ID_${name};
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, ${wire_length}${crc_extra_arg});
}
*/
/**
 * @brief Encode a ${name_lower} struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param ${name_lower} C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_${name_lower}_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_${name_lower}_t* ${name_lower})
{
    return mavlink_msg_${name_lower}_pack(system_id, component_id, msg,${{arg_fields: ${name_lower}->${name},}});
}
*/
/**
 * @brief Send a ${name_lower} message
 * @param chan MAVLink channel to send the message
 *
${{arg_fields: * @param ${name} ${description}
}}
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_${name_lower}_send(mavlink_channel_t chan,${{arg_fields: ${array_const}${type} ${array_prefix}${name},}})
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[${wire_length}];
${{scalar_fields:	_mav_put_${type}(buf, ${wire_offset}, ${putname});
}}
${{array_fields:	_mav_put_${type}_array(buf, ${wire_offset}, ${name}, ${array_length});
}}
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_${name}, buf, ${wire_length}${crc_extra_arg});
#else
    mavlink_${name_lower}_t packet;
${{scalar_fields:	packet.${name} = ${putname};
}}
${{array_fields:	memcpy(packet.${name}, ${name}, sizeof(${type})*${array_length});
}}
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_${name}, (const char *)&packet, ${wire_length}${crc_extra_arg});
#endif
}

#endif
*/
// MESSAGE ${name} UNPACKING

${{fields:
/**
 * @brief Get field ${name} from ${name_lower} message
 *
 * @return ${description}
 */
public static ${return_type} mavlink_msg_${name_lower}_get_${name}(byte[] msg)
{
    return ${array_tag}(msg, ${array_return_arg} ${wire_offset});
}
}}

/**
 * @brief Decode a ${name_lower} message into a struct
 *
 * @param msg The message to decode
 * @param ${name_lower} C-struct to decode the message contents into
 */
public static void mavlink_msg_${name_lower}_decode(byte[] msg, ref mavlink_${name_lower}_t ${name_lower})
{
    if (MAVLINK_NEED_BYTE_SWAP) {
    ${{ordered_fields:	${decode_left}mavlink_msg_${name_lower}_get_${name}(msg${decode_right});
    }}
    } else {
        int len = ${wire_length}; //Marshal.SizeOf(${name_lower});
        IntPtr i = Marshal.AllocHGlobal(len);
        Marshal.Copy(msg, 0, i, len);
        ${name_lower} = (mavlink_${name_lower}_t)Marshal.PtrToStructure(i, ((object)${name_lower}).GetType());
        Marshal.FreeHGlobal(i);
    }
}

}
''', m)
    f.close()

def copy_fixed_headers(directory, xml):
    '''copy the fixed protocol headers to the target directory'''
    import shutil
    hlist = [ 'protocol.h', 'mavlink_helpers.h', 'mavlink_types.h', 'checksum.h' ]
    basepath = os.path.dirname(os.path.realpath(__file__))
    srcpath = os.path.join(basepath, 'C/include_v%s' % xml.wire_protocol_version)
    print("Copying fixed headers")
    for h in hlist:
        src = os.path.realpath(os.path.join(srcpath, h))
        dest = os.path.realpath(os.path.join(directory, h))
        if src == dest:
            continue
        shutil.copy(src, dest)

class mav_include(object):
    def __init__(self, base):
        self.base = base

def generate_one(basename, xml):
    '''generate headers for one XML file'''

    directory = os.path.join(basename, xml.basename)

    print("Generating CSharp implementation in directory %s" % directory)
    mavparse.mkdir_p(directory)

    if xml.little_endian:
        xml.mavlink_endian = "MAVLINK_LITTLE_ENDIAN"
    else:
        xml.mavlink_endian = "MAVLINK_BIG_ENDIAN"

    if xml.crc_extra:
        xml.crc_extra_define = "1"
    else:
        xml.crc_extra_define = "0"

    if xml.sort_fields:
        xml.aligned_fields_define = "1"
    else:
        xml.aligned_fields_define = "0"

    # work out the included headers
    xml.include_list = []
    for i in xml.include:
        base = i[:-4]
        xml.include_list.append(mav_include(base))

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

    # form message info array
    xml.message_info_array = ''
    for name in xml.message_names:
        if name is not None:
            xml.message_info_array += 'typeof( mavlink_%s_t ), ' % name.lower()
        else:
            #xml.message_info_array += '{"EMPTY",0,{}}, '
            xml.message_info_array += 'null, '
    xml.message_info_array = xml.message_info_array[:-2]

    # add some extra field attributes for convenience with arrays
    for m in xml.message:
        m.msg_name = m.name
        if xml.crc_extra:
            m.crc_extra_arg = ", %s" % m.crc_extra
        else:
            m.crc_extra_arg = ""
        for f in m.fields:
#            if f.print_format is None:
#                f.c_print_format = 'null'
#            else:
#                f.c_print_format = '"%s"' % f.print_format
            if f.array_length != 0:
                f.array_suffix = ''
                f.array_prefix = '[MarshalAs(UnmanagedType.ByValArray,SizeConst=%u)]\n public' % f.array_length
                f.array_arg = ', %u' % f.array_length
                f.array_return_arg = '%u, ' % (f.array_length)
                f.array_tag = ''
                f.array_const = 'const '
                f.decode_left = "%s.%s = " % (m.name_lower, f.name)
                f.decode_right = ''
                f.return_type = 'void'
                f.return_value = 'void'
                if f.type == 'char': 
                    f.type = "string"
                    f.array_tag = 'System.Text.ASCIIEncoding.ASCII.GetString(msg,%u,%u); //' % (f.wire_offset, f.array_length)
                    f.return_type = 'string'
                    f.c_test_value = ".ToCharArray()";
                elif f.type == 'uint8_t':
                    f.type = "byte[]";
                    f.array_tag = 'getBytes'
                    f.return_type = 'byte[]'
                elif f.type == 'int8_t':
                    f.type = "byte[]";
                    f.array_tag = 'getBytes'
                    f.return_type = 'byte[]'
                else:
                    test_strings = []
                    for v in f.test_value:
                        test_strings.append(str(v))
                    f.c_test_value = '{ %s }' % ', '.join(test_strings)
                    f.array_tag = '!!!%s' % f.type
                f.get_arg = ', %s %s' % (f.type, f.name)
            else:
                if f.type == 'char':
                    f.type = "byte";
                elif f.type == 'uint8_t':
                    f.type = "byte";
                elif f.type == 'int8_t':
                    f.type = "byte";
                elif f.type == 'uint16_t': 
                    f.type = "UInt16";
                elif f.type == 'uint32_t':
                    f.type = "UInt32";
                elif f.type == 'int16_t': 
                    f.type = "Int16";
                elif f.type == 'int32_t':
                    f.type = "Int32";
                elif f.type == 'uint64_t':
                    f.type = "UInt64";                  
                elif f.type == 'int64_t':     
                    f.type = "Int64";   
                elif f.type == 'float':     
                    f.type = "Single"; 					
                else:
                    f.c_test_value = f.test_value
                f.array_suffix = ''
                f.array_prefix = 'public '
                f.array_tag = 'BitConverter.To%s' % f.type
                if f.type == 'byte':
                    f.array_tag = 'getByte'
                f.array_arg = ''
                f.array_return_arg = ''
                f.array_const = ''
                f.decode_left = "%s.%s = " % (m.name_lower, f.name)
                f.decode_right = ''
                f.get_arg = ''
                f.c_test_value = f.test_value
                f.return_type = f.type

    # cope with uint8_t_mavlink_version
    for m in xml.message:
        m.arg_fields = []
        m.array_fields = []
        m.scalar_fields = []
        for f in m.ordered_fields:
            if f.array_length != 0:
                m.array_fields.append(f)
            else:
                m.scalar_fields.append(f)
        for f in m.fields:
            if not f.omit_arg:
                m.arg_fields.append(f)
                f.putname = f.name
            else:
                f.putname = f.const_value

#    generate_mavlink_h(directory, xml)
#    generate_version_h(directory, xml)
#    generate_main_h(directory, xml)
    for m in xml.message:
        generate_message_h(directory, m)


def generate(basename, xml_list):
    '''generate complete MAVLink C implemenation'''

    for xml in xml_list:
        generate_one(basename, xml)
    copy_fixed_headers(basename, xml_list[0])
