#!/usr/bin/env python
'''
parse a MAVLink protocol XML file and generate a C# implementation

Copyright Michael Oborne 2011
Released under GNU GPL version 3 or later
'''

import sys, textwrap, os, time
import mavparse, mavtemplate

t = mavtemplate.MAVTemplate()

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
    
def generate_message_header(f, xml):

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
    for m in xml.enum:
        m.description = m.description.replace("\n"," ")
        m.description = m.description.replace("\r"," ")
        for fe in m.entry:
            fe.description = fe.description.replace("\n"," ")
            fe.description = fe.description.replace("\r"," ")
            fe.name = fe.name.replace(m.name + "_","")
            fe.name = fe.name.replace("NAV_","")
           
    
    if xml.version == "0.9":
        text = "#if !MAVLINK10";
    else:
        text = "#if MAVLINK10";
    t.write(f, '''
using System;
using System.Collections.Generic;
using System.Text;
using System.Runtime.InteropServices;

namespace ArdupilotMega
{
    partial class MAVLink
    {
        public const string MAVLINK_BUILD_DATE = "${parse_time}";
        public const string MAVLINK_WIRE_PROTOCOL_VERSION = "${wire_protocol_version}";
        public const int MAVLINK_MAX_DIALECT_PAYLOAD_SIZE = ${largest_payload};

        public const int MAVLINK_LITTLE_ENDIAN = 1;
        public const int MAVLINK_BIG_ENDIAN = 0;

        public const byte MAVLINK_STX = ${protocol_marker};

        public const byte MAVLINK_ENDIAN = ${mavlink_endian};

        public const bool MAVLINK_ALIGNED_FIELDS = (${aligned_fields_define} == 1);

        public const byte MAVLINK_CRC_EXTRA = ${crc_extra_define};
        
        public const bool MAVLINK_NEED_BYTE_SWAP = (MAVLINK_ENDIAN == MAVLINK_LITTLE_ENDIAN);
        
        public byte[] MAVLINK_MESSAGE_LENGTHS = new byte[] {${message_lengths_array}};

        public byte[] MAVLINK_MESSAGE_CRCS = new byte[] {${message_crcs_array}};

        public Type[] MAVLINK_MESSAGE_INFO = new Type[] {${message_info_array}};

        public const byte MAVLINK_VERSION = ${version};
    
        ${{enum:
        /** @brief ${description} */
        public enum ${name}
        {
    ${{entry:	///<summary> ${description} |${{param:${description}| }} </summary>
            ${name}=${value}, 
        }}
        };
        }}
    
''', xml)

def generate_message_enums(f, xml):
    # add some extra field attributes for convenience with arrays
    for m in xml.enum:
        m.description = m.description.replace("\n"," ")
        m.description = m.description.replace("\r"," ")
        for fe in m.entry:
            fe.description = fe.description.replace("\n"," ")
            fe.description = fe.description.replace("\r"," ")
            fe.name = fe.name.replace(m.name + "_","")
            
    t.write(f, '''
        ${{enum:
        /** @brief ${description} */
        public enum ${name}
        {
    ${{entry:	///<summary> ${description} |${{param:${description}| }} </summary>
            ${name}=${value}, 
        }}
        };
        }}
''', xml)


def generate_message_footer(f, xml):
    t.write(f, '''
     }
}
''', xml)
    f.close()
             

def generate_message_h(f, directory, m):
    '''generate per-message header for a XML file'''
    #f = open(os.path.join(directory, 'mavlink_msg_%s.cs' % m.name_lower), mode='w')
    t.write(f, '''

    public const byte MAVLINK_MSG_ID_${name} = ${id};
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=${wire_length})]
    public struct mavlink_${name_lower}_t
    {
${{ordered_fields:        /// <summary> ${description} </summary>
        ${array_prefix} ${type} ${name}${array_suffix};
    }}
    };

''', m)
#    f.close()


class mav_include(object):
    def __init__(self, base):
        self.base = base

def generate_one(fh, basename, xml):
    '''generate headers for one XML file'''

    directory = os.path.join(basename, xml.basename)

    print("Generating CSharp implementation in directory %s" % directory)
    mavparse.mkdir_p(directory)

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
            f.description = f.description.replace("\n"," ")
            f.description = f.description.replace("\r","")
            if f.array_length != 0:
                f.array_suffix = ''
                f.array_prefix = '[MarshalAs(UnmanagedType.ByValArray,SizeConst=%u)]\n\t\tpublic' % f.array_length
                f.array_arg = ', %u' % f.array_length
                f.array_return_arg = '%u, ' % (f.array_length)
                f.array_tag = ''
                f.array_const = 'const '
                f.decode_left = "%s.%s = " % (m.name_lower, f.name)
                f.decode_right = ''
                f.return_type = 'void'
                f.return_value = 'void'
                if f.type == 'char': 
                    f.type = "byte[]"
                    f.array_tag = 'System.Text.ASCIIEncoding.ASCII.GetString(msg,%u,%u); //' % (f.wire_offset, f.array_length)
                    f.return_type = 'byte[]'
                    f.c_test_value = ".ToCharArray()";
                elif f.type == 'uint8_t':
                    f.type = "byte[]";
                    f.array_tag = 'getBytes'
                    f.return_type = 'byte[]'
                elif f.type == 'int8_t':
                    f.type = "byte[]";
                    f.array_tag = 'getBytes'
                    f.return_type = 'byte[]'
                elif f.type == 'int16_t':
                    f.type = "Int16[]";
                    f.array_tag = 'getBytes'
                    f.return_type = 'Int16[]'
                elif f.type == 'uint16_t':
                    f.type = "UInt16[]";
                    f.array_tag = 'getBytes'
                    f.return_type = 'UInt16[]'
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
                elif f.type == 'int16_t': 
                    f.type = "Int16";
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
                if f.name == 'fixed':   # this is a keyword
                    f.name = 'fixedp'
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
        generate_message_h(fh, directory, m)
        



def generate(basename, xml_list):
    '''generate complete MAVLink C implemenation'''
    
    print "HERE ",basename, xml_list[0]
    
    directory = os.path.join(basename, xml_list[0].basename)
    
    if not os.path.exists(directory): 
        os.makedirs(directory) 


    f = open(os.path.join(directory, "mavlink.cs"), mode='w')
    
    generate_message_header(f, xml_list[0])
    
    if len(xml_list) > 1:
        generate_message_enums(f, xml_list[1]);
    
    for xml in xml_list:
        generate_one(f, basename, xml)
#    copy_fixed_headers(basename, xml_list[0])
        
    generate_message_footer(f,xml)
    
