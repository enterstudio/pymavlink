using System;
using System.Collections.Generic;
using System.Text;
using System.Runtime.InteropServices;

namespace ArdupilotMega
{
    #if MAVLINK10
    partial class MAVLink
    {
        public const string MAVLINK_BUILD_DATE = "Thu Jul 26 18:28:26 2012";
        public const string MAVLINK_WIRE_PROTOCOL_VERSION = "1.0";
        public const int MAVLINK_MAX_DIALECT_PAYLOAD_SIZE = 9;

        public const int MAVLINK_LITTLE_ENDIAN = 1;
        public const int MAVLINK_BIG_ENDIAN = 0;

        public const byte MAVLINK_STX = 254;

        public const byte MAVLINK_ENDIAN = MAVLINK_LITTLE_ENDIAN;

        public const bool MAVLINK_ALIGNED_FIELDS = (1 == 1);

        public const byte MAVLINK_CRC_EXTRA = 1;
        
        public const bool MAVLINK_NEED_BYTE_SWAP = (MAVLINK_ENDIAN == MAVLINK_LITTLE_ENDIAN);
        
        public byte[] MAVLINK_MESSAGE_LENGTHS = new byte[] {9, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

        public byte[] MAVLINK_MESSAGE_CRCS = new byte[] {50, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

        public Type[] MAVLINK_MESSAGE_INFO = new Type[] {typeof( mavlink_heartbeat_t ), null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null};

        public const byte MAVLINK_VERSION = 2;
    
        
        /** @brief Micro air vehicle / autopilot classes. This identifies the individual model. */
        public enum MAV_AUTOPILOT
        {
    	///<summary> Generic autopilot, full support for everything | </summary>
            GENERIC=0, 
        	///<summary> PIXHAWK autopilot, http://pixhawk.ethz.ch | </summary>
            PIXHAWK=1, 
        	///<summary> SLUGS autopilot, http://slugsuav.soe.ucsc.edu | </summary>
            SLUGS=2, 
        	///<summary> ArduPilotMega / ArduCopter, http://diydrones.com | </summary>
            ARDUPILOTMEGA=3, 
        	///<summary> OpenPilot, http://openpilot.org | </summary>
            OPENPILOT=4, 
        	///<summary> Generic autopilot only supporting simple waypoints | </summary>
            GENERIC_WAYPOINTS_ONLY=5, 
        	///<summary> Generic autopilot supporting waypoints and other simple navigation commands | </summary>
            GENERIC_WAYPOINTS_AND_SIMPLE_NAVIGATION_ONLY=6, 
        	///<summary> Generic autopilot supporting the full mission command set | </summary>
            GENERIC_MISSION_FULL=7, 
        	///<summary> No valid autopilot, e.g. a GCS or other MAVLink component | </summary>
            INVALID=8, 
        	///<summary> PPZ UAV - http://nongnu.org/paparazzi | </summary>
            PPZ=9, 
        	///<summary> UAV Dev Board | </summary>
            UDB=10, 
        	///<summary> FlexiPilot | </summary>
            FP=11, 
        	///<summary>  | </summary>
            ENUM_END=12, 
        
        };
        
        /** @brief  */
        public enum MAV_TYPE
        {
    	///<summary> Generic micro air vehicle. | </summary>
            GENERIC=0, 
        	///<summary> Fixed wing aircraft. | </summary>
            FIXED_WING=1, 
        	///<summary> Quadrotor | </summary>
            QUADROTOR=2, 
        	///<summary> Coaxial helicopter | </summary>
            COAXIAL=3, 
        	///<summary> Normal helicopter with tail rotor. | </summary>
            HELICOPTER=4, 
        	///<summary> Ground installation | </summary>
            ANTENNA_TRACKER=5, 
        	///<summary> Operator control unit / ground control station | </summary>
            GCS=6, 
        	///<summary> Airship, controlled | </summary>
            AIRSHIP=7, 
        	///<summary> Free balloon, uncontrolled | </summary>
            FREE_BALLOON=8, 
        	///<summary> Rocket | </summary>
            ROCKET=9, 
        	///<summary> Ground rover | </summary>
            GROUND_ROVER=10, 
        	///<summary> Surface vessel, boat, ship | </summary>
            SURFACE_BOAT=11, 
        	///<summary> Submarine | </summary>
            SUBMARINE=12, 
        	///<summary> Hexarotor | </summary>
            HEXAROTOR=13, 
        	///<summary> Octorotor | </summary>
            OCTOROTOR=14, 
        	///<summary> Octorotor | </summary>
            TRICOPTER=15, 
        	///<summary> Flapping wing | </summary>
            FLAPPING_WING=16, 
        	///<summary>  | </summary>
            ENUM_END=17, 
        
        };
        
        /** @brief These flags encode the MAV mode. */
        public enum MAV_MODE_FLAG
        {
    	///<summary> 0b00000001 Reserved for future use. | </summary>
            CUSTOM_MODE_ENABLED=1, 
        	///<summary> 0b00000010 system has a test mode enabled. This flag is intended for temporary system tests and should not be used for stable implementations. | </summary>
            TEST_ENABLED=2, 
        	///<summary> 0b00000100 autonomous mode enabled, system finds its own goal positions. Guided flag can be set or not, depends on the actual implementation. | </summary>
            AUTO_ENABLED=4, 
        	///<summary> 0b00001000 guided mode enabled, system flies MISSIONs / mission items. | </summary>
            GUIDED_ENABLED=8, 
        	///<summary> 0b00010000 system stabilizes electronically its attitude (and optionally position). It needs however further control inputs to move around. | </summary>
            STABILIZE_ENABLED=16, 
        	///<summary> 0b00100000 hardware in the loop simulation. All motors / actuators are blocked, but internal software is full operational. | </summary>
            HIL_ENABLED=32, 
        	///<summary> 0b01000000 remote control input is enabled. | </summary>
            MANUAL_INPUT_ENABLED=64, 
        	///<summary> 0b10000000 MAV safety set to armed. Motors are enabled / running / can start. Ready to fly. | </summary>
            SAFETY_ARMED=128, 
        	///<summary>  | </summary>
            ENUM_END=129, 
        
        };
        
        /** @brief These values encode the bit positions of the decode position. These values can be used to read the value of a flag bit by combining the base_mode variable with AND with the flag position value. The result will be either 0 or 1, depending on if the flag is set or not. */
        public enum MAV_MODE_FLAG_DECODE_POSITION
        {
    	///<summary> Eighth bit: 00000001 | </summary>
            CUSTOM_MODE=1, 
        	///<summary> Seventh bit: 00000010 | </summary>
            TEST=2, 
        	///<summary> Sixt bit:   00000100 | </summary>
            AUTO=4, 
        	///<summary> Fifth bit:  00001000 | </summary>
            GUIDED=8, 
        	///<summary> Fourth bit: 00010000 | </summary>
            STABILIZE=16, 
        	///<summary> Third bit:  00100000 | </summary>
            HIL=32, 
        	///<summary> Second bit: 01000000 | </summary>
            MANUAL=64, 
        	///<summary> First bit:  10000000 | </summary>
            SAFETY=128, 
        	///<summary>  | </summary>
            ENUM_END=129, 
        
        };
        
        /** @brief  */
        public enum MAV_STATE
        {
    	///<summary> Uninitialized system, state is unknown. | </summary>
            UNINIT=0, 
        	///<summary> System is booting up. | </summary>
            BOOT=1, 
        	///<summary> System is calibrating and not flight-ready. | </summary>
            CALIBRATING=2, 
        	///<summary> System is grounded and on standby. It can be launched any time. | </summary>
            STANDBY=3, 
        	///<summary> System is active and might be already airborne. Motors are engaged. | </summary>
            ACTIVE=4, 
        	///<summary> System is in a non-normal flight mode. It can however still navigate. | </summary>
            CRITICAL=5, 
        	///<summary> System is in a non-normal flight mode. It lost control over parts or over the whole airframe. It is in mayday and going down. | </summary>
            EMERGENCY=6, 
        	///<summary> System just initialized its power-down sequence, will shut down now. | </summary>
            POWEROFF=7, 
        	///<summary>  | </summary>
            ENUM_END=8, 
        
        };
        
    

    public const byte MAVLINK_MSG_ID_HEARTBEAT = 0;
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=9)]
    public struct mavlink_heartbeat_t
    {
        /// <summary> A bitfield for use for autopilot-specific flags. </summary>
        public  UInt32 custom_mode;
            /// <summary> Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM) </summary>
        public  byte type;
            /// <summary> Autopilot type / class. defined in MAV_AUTOPILOT ENUM </summary>
        public  byte autopilot;
            /// <summary> System mode bitfield, see MAV_MODE_FLAGS ENUM in mavlink/include/mavlink_types.h </summary>
        public  byte base_mode;
            /// <summary> System status flag, see MAV_STATE ENUM </summary>
        public  byte system_status;
            /// <summary> MAVLink version </summary>
        public  byte mavlink_version;
    
    };

     }
     #endif
}
