/** @file
 *	@brief MAVLink comm protocol built from slugs.xml
 *	@see http://pixhawk.ethz.ch/software/mavlink
 */
 using System;
 
public partial class Mavlink
{
    const int MAVLINK_LITTLE_ENDIAN = 1;

    const byte MAVLINK_STX = 85;

    const byte MAVLINK_ENDIAN = MAVLINK_BIG_ENDIAN;

    const byte MAVLINK_ALIGNED_FIELDS = 0;

    const byte MAVLINK_CRC_EXTRA = 0;
    
    const bool MAVLINK_NEED_BYTE_SWAP = (MAVLINK_ENDIAN == MAVLINK_LITTLE_ENDIAN);
}
