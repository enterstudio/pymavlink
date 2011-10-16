// MESSAGE TEST_TYPES PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_TEST_TYPES = 0;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_test_types_t
    {
         public  byte c; /// char
     [MarshalAs(UnmanagedType.ByValArray,SizeConst=10)]
 public string s; /// string
     public  byte u8; /// uint8_t
     public  UInt16 u16; /// uint16_t
     public  UInt32 u32; /// uint32_t
     public  UInt64 u64; /// uint64_t
     public  byte s8; /// int8_t
     public  Int16 s16; /// int16_t
     public  Int32 s32; /// int32_t
     public  Int64 s64; /// int64_t
     public  Single f; /// float
     public  double d; /// double
     [MarshalAs(UnmanagedType.ByValArray,SizeConst=3)]
 public byte[] u8_array; /// uint8_t_array
     [MarshalAs(UnmanagedType.ByValArray,SizeConst=3)]
 public uint16_t u16_array; /// uint16_t_array
     [MarshalAs(UnmanagedType.ByValArray,SizeConst=3)]
 public uint32_t u32_array; /// uint32_t_array
     [MarshalAs(UnmanagedType.ByValArray,SizeConst=3)]
 public uint64_t u64_array; /// uint64_t_array
     [MarshalAs(UnmanagedType.ByValArray,SizeConst=3)]
 public byte[] s8_array; /// int8_t_array
     [MarshalAs(UnmanagedType.ByValArray,SizeConst=3)]
 public int16_t s16_array; /// int16_t_array
     [MarshalAs(UnmanagedType.ByValArray,SizeConst=3)]
 public int32_t s32_array; /// int32_t_array
     [MarshalAs(UnmanagedType.ByValArray,SizeConst=3)]
 public int64_t s64_array; /// int64_t_array
     [MarshalAs(UnmanagedType.ByValArray,SizeConst=3)]
 public float f_array; /// float_array
     [MarshalAs(UnmanagedType.ByValArray,SizeConst=3)]
 public double d_array; /// double_array
    
    };

/**
 * @brief Pack a test_types message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param c char
 * @param s string
 * @param u8 uint8_t
 * @param u16 uint16_t
 * @param u32 uint32_t
 * @param u64 uint64_t
 * @param s8 int8_t
 * @param s16 int16_t
 * @param s32 int32_t
 * @param s64 int64_t
 * @param f float
 * @param d double
 * @param u8_array uint8_t_array
 * @param u16_array uint16_t_array
 * @param u32_array uint32_t_array
 * @param u64_array uint64_t_array
 * @param s8_array int8_t_array
 * @param s16_array int16_t_array
 * @param s32_array int32_t_array
 * @param s64_array int64_t_array
 * @param f_array float_array
 * @param d_array double_array
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 
public static UInt16 mavlink_msg_test_types_pack(byte system_id, byte component_id, byte[] msg,
                               byte c, string s, byte u8, UInt16 u16, UInt32 u32, UInt64 u64, byte s8, Int16 s16, Int32 s32, Int64 s64, Single f, double d, byte[] u8_array, uint16_t u16_array, uint32_t u32_array, uint64_t u64_array, byte[] s8_array, int16_t s16_array, int32_t s32_array, int64_t s64_array, float f_array, double d_array)
{
if (MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS) {
	Array.Copy(BitConverter.GetBytes(c),0,msg,0,sizeof(byte));
	Array.Copy(BitConverter.GetBytes(u8),0,msg,11,sizeof(byte));
	Array.Copy(BitConverter.GetBytes(u16),0,msg,12,sizeof(UInt16));
	Array.Copy(BitConverter.GetBytes(u32),0,msg,14,sizeof(UInt32));
	Array.Copy(BitConverter.GetBytes(u64),0,msg,18,sizeof(UInt64));
	Array.Copy(BitConverter.GetBytes(s8),0,msg,26,sizeof(byte));
	Array.Copy(BitConverter.GetBytes(s16),0,msg,27,sizeof(Int16));
	Array.Copy(BitConverter.GetBytes(s32),0,msg,29,sizeof(Int32));
	Array.Copy(BitConverter.GetBytes(s64),0,msg,33,sizeof(Int64));
	Array.Copy(BitConverter.GetBytes(f),0,msg,41,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(d),0,msg,45,sizeof(double));
	//Array.Copy(s,0,msg,1,10);
	//Array.Copy(u8_array,0,msg,53,3);
	//Array.Copy(u16_array,0,msg,56,3);
	//Array.Copy(u32_array,0,msg,62,3);
	//Array.Copy(u64_array,0,msg,74,3);
	//Array.Copy(s8_array,0,msg,98,3);
	//Array.Copy(s16_array,0,msg,101,3);
	//Array.Copy(s32_array,0,msg,107,3);
	//Array.Copy(s64_array,0,msg,119,3);
	//Array.Copy(f_array,0,msg,143,3);
	//Array.Copy(d_array,0,msg,155,3);
} else {
    mavlink_test_types_t packet = new mavlink_test_types_t();
	packet.c = c;
	packet.u8 = u8;
	packet.u16 = u16;
	packet.u32 = u32;
	packet.u64 = u64;
	packet.s8 = s8;
	packet.s16 = s16;
	packet.s32 = s32;
	packet.s64 = s64;
	packet.f = f;
	packet.d = d;
	packet.s = s;
	packet.u8_array = u8_array;
	packet.u16_array = u16_array;
	packet.u32_array = u32_array;
	packet.u64_array = u64_array;
	packet.s8_array = s8_array;
	packet.s16_array = s16_array;
	packet.s32_array = s32_array;
	packet.s64_array = s64_array;
	packet.f_array = f_array;
	packet.d_array = d_array;
        
        int len = 179;
        msg = new byte[len];
        IntPtr ptr = Marshal.AllocHGlobal(len);
        Marshal.StructureToPtr(packet, ptr, true);
        Marshal.Copy(ptr, msg, 0, len);
        Marshal.FreeHGlobal(ptr);
}

    //msg.msgid = MAVLINK_MSG_ID_TEST_TYPES;
    //return mavlink_finalize_message(msg, system_id, component_id, 179);
    return 0;
}

/**
 * @brief Pack a test_types message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param c char
 * @param s string
 * @param u8 uint8_t
 * @param u16 uint16_t
 * @param u32 uint32_t
 * @param u64 uint64_t
 * @param s8 int8_t
 * @param s16 int16_t
 * @param s32 int32_t
 * @param s64 int64_t
 * @param f float
 * @param d double
 * @param u8_array uint8_t_array
 * @param u16_array uint16_t_array
 * @param u32_array uint32_t_array
 * @param u64_array uint64_t_array
 * @param s8_array int8_t_array
 * @param s16_array int16_t_array
 * @param s32_array int32_t_array
 * @param s64_array int64_t_array
 * @param f_array float_array
 * @param d_array double_array
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_test_types_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   byte public c,const string [MarshalAs(UnmanagedType.ByValArray,SizeConst=10)]
 publics,byte public u8,UInt16 public u16,UInt32 public u32,UInt64 public u64,byte public s8,Int16 public s16,Int32 public s32,Int64 public s64,Single public f,double public d,const byte[] [MarshalAs(UnmanagedType.ByValArray,SizeConst=3)]
 publicu8_array,const uint16_t [MarshalAs(UnmanagedType.ByValArray,SizeConst=3)]
 publicu16_array,const uint32_t [MarshalAs(UnmanagedType.ByValArray,SizeConst=3)]
 publicu32_array,const uint64_t [MarshalAs(UnmanagedType.ByValArray,SizeConst=3)]
 publicu64_array,const byte[] [MarshalAs(UnmanagedType.ByValArray,SizeConst=3)]
 publics8_array,const int16_t [MarshalAs(UnmanagedType.ByValArray,SizeConst=3)]
 publics16_array,const int32_t [MarshalAs(UnmanagedType.ByValArray,SizeConst=3)]
 publics32_array,const int64_t [MarshalAs(UnmanagedType.ByValArray,SizeConst=3)]
 publics64_array,const float [MarshalAs(UnmanagedType.ByValArray,SizeConst=3)]
 publicf_array,const double [MarshalAs(UnmanagedType.ByValArray,SizeConst=3)]
 publicd_array)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[179];
	_mav_put_byte(buf, 0, c);
	_mav_put_byte(buf, 11, u8);
	_mav_put_UInt16(buf, 12, u16);
	_mav_put_UInt32(buf, 14, u32);
	_mav_put_UInt64(buf, 18, u64);
	_mav_put_byte(buf, 26, s8);
	_mav_put_Int16(buf, 27, s16);
	_mav_put_Int32(buf, 29, s32);
	_mav_put_Int64(buf, 33, s64);
	_mav_put_Single(buf, 41, f);
	_mav_put_double(buf, 45, d);
	_mav_put_string_array(buf, 1, s, 10);
	_mav_put_byte[]_array(buf, 53, u8_array, 3);
	_mav_put_uint16_t_array(buf, 56, u16_array, 3);
	_mav_put_uint32_t_array(buf, 62, u32_array, 3);
	_mav_put_uint64_t_array(buf, 74, u64_array, 3);
	_mav_put_byte[]_array(buf, 98, s8_array, 3);
	_mav_put_int16_t_array(buf, 101, s16_array, 3);
	_mav_put_int32_t_array(buf, 107, s32_array, 3);
	_mav_put_int64_t_array(buf, 119, s64_array, 3);
	_mav_put_float_array(buf, 143, f_array, 3);
	_mav_put_double_array(buf, 155, d_array, 3);
        memcpy(_MAV_PAYLOAD(msg), buf, 179);
#else
    mavlink_test_types_t packet;
	packet.c = c;
	packet.u8 = u8;
	packet.u16 = u16;
	packet.u32 = u32;
	packet.u64 = u64;
	packet.s8 = s8;
	packet.s16 = s16;
	packet.s32 = s32;
	packet.s64 = s64;
	packet.f = f;
	packet.d = d;
	memcpy(packet.s, s, sizeof(string)*10);
	memcpy(packet.u8_array, u8_array, sizeof(byte[])*3);
	memcpy(packet.u16_array, u16_array, sizeof(uint16_t)*3);
	memcpy(packet.u32_array, u32_array, sizeof(uint32_t)*3);
	memcpy(packet.u64_array, u64_array, sizeof(uint64_t)*3);
	memcpy(packet.s8_array, s8_array, sizeof(byte[])*3);
	memcpy(packet.s16_array, s16_array, sizeof(int16_t)*3);
	memcpy(packet.s32_array, s32_array, sizeof(int32_t)*3);
	memcpy(packet.s64_array, s64_array, sizeof(int64_t)*3);
	memcpy(packet.f_array, f_array, sizeof(float)*3);
	memcpy(packet.d_array, d_array, sizeof(double)*3);
        memcpy(_MAV_PAYLOAD(msg), &packet, 179);
#endif

    msg->msgid = MAVLINK_MSG_ID_TEST_TYPES;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 179);
}
*/
/**
 * @brief Encode a test_types struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param test_types C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_test_types_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_test_types_t* test_types)
{
    return mavlink_msg_test_types_pack(system_id, component_id, msg, test_types->c, test_types->s, test_types->u8, test_types->u16, test_types->u32, test_types->u64, test_types->s8, test_types->s16, test_types->s32, test_types->s64, test_types->f, test_types->d, test_types->u8_array, test_types->u16_array, test_types->u32_array, test_types->u64_array, test_types->s8_array, test_types->s16_array, test_types->s32_array, test_types->s64_array, test_types->f_array, test_types->d_array);
}
*/
/**
 * @brief Send a test_types message
 * @param chan MAVLink channel to send the message
 *
 * @param c char
 * @param s string
 * @param u8 uint8_t
 * @param u16 uint16_t
 * @param u32 uint32_t
 * @param u64 uint64_t
 * @param s8 int8_t
 * @param s16 int16_t
 * @param s32 int32_t
 * @param s64 int64_t
 * @param f float
 * @param d double
 * @param u8_array uint8_t_array
 * @param u16_array uint16_t_array
 * @param u32_array uint32_t_array
 * @param u64_array uint64_t_array
 * @param s8_array int8_t_array
 * @param s16_array int16_t_array
 * @param s32_array int32_t_array
 * @param s64_array int64_t_array
 * @param f_array float_array
 * @param d_array double_array
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_test_types_send(mavlink_channel_t chan, byte public c, const string [MarshalAs(UnmanagedType.ByValArray,SizeConst=10)]
 publics, byte public u8, UInt16 public u16, UInt32 public u32, UInt64 public u64, byte public s8, Int16 public s16, Int32 public s32, Int64 public s64, Single public f, double public d, const byte[] [MarshalAs(UnmanagedType.ByValArray,SizeConst=3)]
 publicu8_array, const uint16_t [MarshalAs(UnmanagedType.ByValArray,SizeConst=3)]
 publicu16_array, const uint32_t [MarshalAs(UnmanagedType.ByValArray,SizeConst=3)]
 publicu32_array, const uint64_t [MarshalAs(UnmanagedType.ByValArray,SizeConst=3)]
 publicu64_array, const byte[] [MarshalAs(UnmanagedType.ByValArray,SizeConst=3)]
 publics8_array, const int16_t [MarshalAs(UnmanagedType.ByValArray,SizeConst=3)]
 publics16_array, const int32_t [MarshalAs(UnmanagedType.ByValArray,SizeConst=3)]
 publics32_array, const int64_t [MarshalAs(UnmanagedType.ByValArray,SizeConst=3)]
 publics64_array, const float [MarshalAs(UnmanagedType.ByValArray,SizeConst=3)]
 publicf_array, const double [MarshalAs(UnmanagedType.ByValArray,SizeConst=3)]
 publicd_array)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[179];
	_mav_put_byte(buf, 0, c);
	_mav_put_byte(buf, 11, u8);
	_mav_put_UInt16(buf, 12, u16);
	_mav_put_UInt32(buf, 14, u32);
	_mav_put_UInt64(buf, 18, u64);
	_mav_put_byte(buf, 26, s8);
	_mav_put_Int16(buf, 27, s16);
	_mav_put_Int32(buf, 29, s32);
	_mav_put_Int64(buf, 33, s64);
	_mav_put_Single(buf, 41, f);
	_mav_put_double(buf, 45, d);
	_mav_put_string_array(buf, 1, s, 10);
	_mav_put_byte[]_array(buf, 53, u8_array, 3);
	_mav_put_uint16_t_array(buf, 56, u16_array, 3);
	_mav_put_uint32_t_array(buf, 62, u32_array, 3);
	_mav_put_uint64_t_array(buf, 74, u64_array, 3);
	_mav_put_byte[]_array(buf, 98, s8_array, 3);
	_mav_put_int16_t_array(buf, 101, s16_array, 3);
	_mav_put_int32_t_array(buf, 107, s32_array, 3);
	_mav_put_int64_t_array(buf, 119, s64_array, 3);
	_mav_put_float_array(buf, 143, f_array, 3);
	_mav_put_double_array(buf, 155, d_array, 3);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TEST_TYPES, buf, 179);
#else
    mavlink_test_types_t packet;
	packet.c = c;
	packet.u8 = u8;
	packet.u16 = u16;
	packet.u32 = u32;
	packet.u64 = u64;
	packet.s8 = s8;
	packet.s16 = s16;
	packet.s32 = s32;
	packet.s64 = s64;
	packet.f = f;
	packet.d = d;
	memcpy(packet.s, s, sizeof(string)*10);
	memcpy(packet.u8_array, u8_array, sizeof(byte[])*3);
	memcpy(packet.u16_array, u16_array, sizeof(uint16_t)*3);
	memcpy(packet.u32_array, u32_array, sizeof(uint32_t)*3);
	memcpy(packet.u64_array, u64_array, sizeof(uint64_t)*3);
	memcpy(packet.s8_array, s8_array, sizeof(byte[])*3);
	memcpy(packet.s16_array, s16_array, sizeof(int16_t)*3);
	memcpy(packet.s32_array, s32_array, sizeof(int32_t)*3);
	memcpy(packet.s64_array, s64_array, sizeof(int64_t)*3);
	memcpy(packet.f_array, f_array, sizeof(float)*3);
	memcpy(packet.d_array, d_array, sizeof(double)*3);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TEST_TYPES, (const char *)&packet, 179);
#endif
}

#endif
*/
// MESSAGE TEST_TYPES UNPACKING


/**
 * @brief Get field c from test_types message
 *
 * @return char
 */
public static byte mavlink_msg_test_types_get_c(byte[] msg)
{
    return getByte(msg,  0);
}

/**
 * @brief Get field s from test_types message
 *
 * @return string
 */
public static string mavlink_msg_test_types_get_s(byte[] msg)
{
    return System.Text.ASCIIEncoding.ASCII.GetString(msg,1,10); //(msg, 10,  1);
}

/**
 * @brief Get field u8 from test_types message
 *
 * @return uint8_t
 */
public static byte mavlink_msg_test_types_get_u8(byte[] msg)
{
    return getByte(msg,  11);
}

/**
 * @brief Get field u16 from test_types message
 *
 * @return uint16_t
 */
public static UInt16 mavlink_msg_test_types_get_u16(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  12);
}

/**
 * @brief Get field u32 from test_types message
 *
 * @return uint32_t
 */
public static UInt32 mavlink_msg_test_types_get_u32(byte[] msg)
{
    return BitConverter.ToUInt32(msg,  14);
}

/**
 * @brief Get field u64 from test_types message
 *
 * @return uint64_t
 */
public static UInt64 mavlink_msg_test_types_get_u64(byte[] msg)
{
    return BitConverter.ToUInt64(msg,  18);
}

/**
 * @brief Get field s8 from test_types message
 *
 * @return int8_t
 */
public static byte mavlink_msg_test_types_get_s8(byte[] msg)
{
    return getByte(msg,  26);
}

/**
 * @brief Get field s16 from test_types message
 *
 * @return int16_t
 */
public static Int16 mavlink_msg_test_types_get_s16(byte[] msg)
{
    return BitConverter.ToInt16(msg,  27);
}

/**
 * @brief Get field s32 from test_types message
 *
 * @return int32_t
 */
public static Int32 mavlink_msg_test_types_get_s32(byte[] msg)
{
    return BitConverter.ToInt32(msg,  29);
}

/**
 * @brief Get field s64 from test_types message
 *
 * @return int64_t
 */
public static Int64 mavlink_msg_test_types_get_s64(byte[] msg)
{
    return BitConverter.ToInt64(msg,  33);
}

/**
 * @brief Get field f from test_types message
 *
 * @return float
 */
public static Single mavlink_msg_test_types_get_f(byte[] msg)
{
    return BitConverter.ToSingle(msg,  41);
}

/**
 * @brief Get field d from test_types message
 *
 * @return double
 */
public static double mavlink_msg_test_types_get_d(byte[] msg)
{
    return BitConverter.Todouble(msg,  45);
}

/**
 * @brief Get field u8_array from test_types message
 *
 * @return uint8_t_array
 */
public static byte[] mavlink_msg_test_types_get_u8_array(byte[] msg)
{
    return getBytes(msg, 3,  53);
}

/**
 * @brief Get field u16_array from test_types message
 *
 * @return uint16_t_array
 */
public static void mavlink_msg_test_types_get_u16_array(byte[] msg)
{
    return !!!uint16_t(msg, 3,  56);
}

/**
 * @brief Get field u32_array from test_types message
 *
 * @return uint32_t_array
 */
public static void mavlink_msg_test_types_get_u32_array(byte[] msg)
{
    return !!!uint32_t(msg, 3,  62);
}

/**
 * @brief Get field u64_array from test_types message
 *
 * @return uint64_t_array
 */
public static void mavlink_msg_test_types_get_u64_array(byte[] msg)
{
    return !!!uint64_t(msg, 3,  74);
}

/**
 * @brief Get field s8_array from test_types message
 *
 * @return int8_t_array
 */
public static byte[] mavlink_msg_test_types_get_s8_array(byte[] msg)
{
    return getBytes(msg, 3,  98);
}

/**
 * @brief Get field s16_array from test_types message
 *
 * @return int16_t_array
 */
public static void mavlink_msg_test_types_get_s16_array(byte[] msg)
{
    return !!!int16_t(msg, 3,  101);
}

/**
 * @brief Get field s32_array from test_types message
 *
 * @return int32_t_array
 */
public static void mavlink_msg_test_types_get_s32_array(byte[] msg)
{
    return !!!int32_t(msg, 3,  107);
}

/**
 * @brief Get field s64_array from test_types message
 *
 * @return int64_t_array
 */
public static void mavlink_msg_test_types_get_s64_array(byte[] msg)
{
    return !!!int64_t(msg, 3,  119);
}

/**
 * @brief Get field f_array from test_types message
 *
 * @return float_array
 */
public static void mavlink_msg_test_types_get_f_array(byte[] msg)
{
    return !!!float(msg, 3,  143);
}

/**
 * @brief Get field d_array from test_types message
 *
 * @return double_array
 */
public static void mavlink_msg_test_types_get_d_array(byte[] msg)
{
    return !!!double(msg, 3,  155);
}

/**
 * @brief Decode a test_types message into a struct
 *
 * @param msg The message to decode
 * @param test_types C-struct to decode the message contents into
 */
public static void mavlink_msg_test_types_decode(byte[] msg, ref mavlink_test_types_t test_types)
{
    if (MAVLINK_NEED_BYTE_SWAP) {
    	test_types.c = mavlink_msg_test_types_get_c(msg);
    	test_types.s = mavlink_msg_test_types_get_s(msg);
    	test_types.u8 = mavlink_msg_test_types_get_u8(msg);
    	test_types.u16 = mavlink_msg_test_types_get_u16(msg);
    	test_types.u32 = mavlink_msg_test_types_get_u32(msg);
    	test_types.u64 = mavlink_msg_test_types_get_u64(msg);
    	test_types.s8 = mavlink_msg_test_types_get_s8(msg);
    	test_types.s16 = mavlink_msg_test_types_get_s16(msg);
    	test_types.s32 = mavlink_msg_test_types_get_s32(msg);
    	test_types.s64 = mavlink_msg_test_types_get_s64(msg);
    	test_types.f = mavlink_msg_test_types_get_f(msg);
    	test_types.d = mavlink_msg_test_types_get_d(msg);
    	test_types.u8_array = mavlink_msg_test_types_get_u8_array(msg);
    	test_types.u16_array = mavlink_msg_test_types_get_u16_array(msg);
    	test_types.u32_array = mavlink_msg_test_types_get_u32_array(msg);
    	test_types.u64_array = mavlink_msg_test_types_get_u64_array(msg);
    	test_types.s8_array = mavlink_msg_test_types_get_s8_array(msg);
    	test_types.s16_array = mavlink_msg_test_types_get_s16_array(msg);
    	test_types.s32_array = mavlink_msg_test_types_get_s32_array(msg);
    	test_types.s64_array = mavlink_msg_test_types_get_s64_array(msg);
    	test_types.f_array = mavlink_msg_test_types_get_f_array(msg);
    	test_types.d_array = mavlink_msg_test_types_get_d_array(msg);
    
    } else {
        int len = 179; //Marshal.SizeOf(test_types);
        IntPtr i = Marshal.AllocHGlobal(len);
        Marshal.Copy(msg, 0, i, len);
        test_types = (mavlink_test_types_t)Marshal.PtrToStructure(i, ((object)test_types).GetType());
        Marshal.FreeHGlobal(i);
    }
}

}
