/** @file
 *	@brief MAVLink comm protocol built from slugs.xml
 *	@see http://pixhawk.ethz.ch/software/mavlink
 */
 using System;
 
public partial class Mavlink
{
    public const int MAVLINK_LITTLE_ENDIAN = 1;

    public const byte MAVLINK_STX = 85;

    public const byte MAVLINK_ENDIAN = MAVLINK_BIG_ENDIAN;

    public const bool MAVLINK_ALIGNED_FIELDS = (0 == 1);

    public const byte MAVLINK_CRC_EXTRA = 0;
    
    public const bool MAVLINK_NEED_BYTE_SWAP = (MAVLINK_ENDIAN == MAVLINK_LITTLE_ENDIAN);
    
    public byte packetcount = 0;
}