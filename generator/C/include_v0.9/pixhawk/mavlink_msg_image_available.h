// MESSAGE IMAGE_AVAILABLE PACKING

#define MAVLINK_MSG_ID_IMAGE_AVAILABLE 154

typedef struct __mavlink_image_available_t
{
 UInt64 cam_id; ///< Camera id
 byte cam_no; ///< Camera # (starts with 0)
 UInt64 timestamp; ///< Timestamp
 UInt64 valid_until; ///< Until which timestamp this buffer will stay valid
 UInt32 img_seq; ///< The image sequence number
 UInt32 img_buf_index; ///< Position of the image in the buffer, starts with 0
 UInt16 width; ///< Image width
 UInt16 height; ///< Image height
 UInt16 depth; ///< Image depth
 byte channels; ///< Image channels
 UInt32 key; ///< Shared memory area key
 UInt32 exposure; ///< Exposure time, in microseconds
 Single gain; ///< Camera gain
 Single roll; ///< Roll angle in rad
 Single pitch; ///< Pitch angle in rad
 Single yaw; ///< Yaw angle in rad
 Single local_z; ///< Local frame Z coordinate (height over ground)
 Single lat; ///< GPS X coordinate
 Single lon; ///< GPS Y coordinate
 Single alt; ///< Global frame altitude
 Single ground_x; ///< Ground truth X
 Single ground_y; ///< Ground truth Y
 Single ground_z; ///< Ground truth Z
} mavlink_image_available_t;

#define MAVLINK_MSG_ID_IMAGE_AVAILABLE_LEN 92
#define MAVLINK_MSG_ID_154_LEN 92



#define MAVLINK_MESSAGE_INFO_IMAGE_AVAILABLE { \
	"IMAGE_AVAILABLE", \
	23, \
	{  { "cam_id", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_image_available_t, cam_id) }, \
         { "cam_no", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_image_available_t, cam_no) }, \
         { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 9, offsetof(mavlink_image_available_t, timestamp) }, \
         { "valid_until", NULL, MAVLINK_TYPE_UINT64_T, 0, 17, offsetof(mavlink_image_available_t, valid_until) }, \
         { "img_seq", NULL, MAVLINK_TYPE_UINT32_T, 0, 25, offsetof(mavlink_image_available_t, img_seq) }, \
         { "img_buf_index", NULL, MAVLINK_TYPE_UINT32_T, 0, 29, offsetof(mavlink_image_available_t, img_buf_index) }, \
         { "width", NULL, MAVLINK_TYPE_UINT16_T, 0, 33, offsetof(mavlink_image_available_t, width) }, \
         { "height", NULL, MAVLINK_TYPE_UINT16_T, 0, 35, offsetof(mavlink_image_available_t, height) }, \
         { "depth", NULL, MAVLINK_TYPE_UINT16_T, 0, 37, offsetof(mavlink_image_available_t, depth) }, \
         { "channels", NULL, MAVLINK_TYPE_UINT8_T, 0, 39, offsetof(mavlink_image_available_t, channels) }, \
         { "key", NULL, MAVLINK_TYPE_UINT32_T, 0, 40, offsetof(mavlink_image_available_t, key) }, \
         { "exposure", NULL, MAVLINK_TYPE_UINT32_T, 0, 44, offsetof(mavlink_image_available_t, exposure) }, \
         { "gain", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_image_available_t, gain) }, \
         { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_image_available_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 56, offsetof(mavlink_image_available_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 60, offsetof(mavlink_image_available_t, yaw) }, \
         { "local_z", NULL, MAVLINK_TYPE_FLOAT, 0, 64, offsetof(mavlink_image_available_t, local_z) }, \
         { "lat", NULL, MAVLINK_TYPE_FLOAT, 0, 68, offsetof(mavlink_image_available_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_FLOAT, 0, 72, offsetof(mavlink_image_available_t, lon) }, \
         { "alt", NULL, MAVLINK_TYPE_FLOAT, 0, 76, offsetof(mavlink_image_available_t, alt) }, \
         { "ground_x", NULL, MAVLINK_TYPE_FLOAT, 0, 80, offsetof(mavlink_image_available_t, ground_x) }, \
         { "ground_y", NULL, MAVLINK_TYPE_FLOAT, 0, 84, offsetof(mavlink_image_available_t, ground_y) }, \
         { "ground_z", NULL, MAVLINK_TYPE_FLOAT, 0, 88, offsetof(mavlink_image_available_t, ground_z) }, \
         } \
}


/**
 * @brief Pack a image_available message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param cam_id Camera id
 * @param cam_no Camera # (starts with 0)
 * @param timestamp Timestamp
 * @param valid_until Until which timestamp this buffer will stay valid
 * @param img_seq The image sequence number
 * @param img_buf_index Position of the image in the buffer, starts with 0
 * @param width Image width
 * @param height Image height
 * @param depth Image depth
 * @param channels Image channels
 * @param key Shared memory area key
 * @param exposure Exposure time, in microseconds
 * @param gain Camera gain
 * @param roll Roll angle in rad
 * @param pitch Pitch angle in rad
 * @param yaw Yaw angle in rad
 * @param local_z Local frame Z coordinate (height over ground)
 * @param lat GPS X coordinate
 * @param lon GPS Y coordinate
 * @param alt Global frame altitude
 * @param ground_x Ground truth X
 * @param ground_y Ground truth Y
 * @param ground_z Ground truth Z
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_image_available_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       UInt64 cam_id, byte cam_no, UInt64 timestamp, UInt64 valid_until, UInt32 img_seq, UInt32 img_buf_index, UInt16 width, UInt16 height, UInt16 depth, byte channels, UInt32 key, UInt32 exposure, Single gain, Single roll, Single pitch, Single yaw, Single local_z, Single lat, Single lon, Single alt, Single ground_x, Single ground_y, Single ground_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[92];
	_mav_put_UInt64(buf, 0, cam_id);
	_mav_put_byte(buf, 8, cam_no);
	_mav_put_UInt64(buf, 9, timestamp);
	_mav_put_UInt64(buf, 17, valid_until);
	_mav_put_UInt32(buf, 25, img_seq);
	_mav_put_UInt32(buf, 29, img_buf_index);
	_mav_put_UInt16(buf, 33, width);
	_mav_put_UInt16(buf, 35, height);
	_mav_put_UInt16(buf, 37, depth);
	_mav_put_byte(buf, 39, channels);
	_mav_put_UInt32(buf, 40, key);
	_mav_put_UInt32(buf, 44, exposure);
	_mav_put_Single(buf, 48, gain);
	_mav_put_Single(buf, 52, roll);
	_mav_put_Single(buf, 56, pitch);
	_mav_put_Single(buf, 60, yaw);
	_mav_put_Single(buf, 64, local_z);
	_mav_put_Single(buf, 68, lat);
	_mav_put_Single(buf, 72, lon);
	_mav_put_Single(buf, 76, alt);
	_mav_put_Single(buf, 80, ground_x);
	_mav_put_Single(buf, 84, ground_y);
	_mav_put_Single(buf, 88, ground_z);

        memcpy(_MAV_PAYLOAD(msg), buf, 92);
#else
	mavlink_image_available_t packet;
	packet.cam_id = cam_id;
	packet.cam_no = cam_no;
	packet.timestamp = timestamp;
	packet.valid_until = valid_until;
	packet.img_seq = img_seq;
	packet.img_buf_index = img_buf_index;
	packet.width = width;
	packet.height = height;
	packet.depth = depth;
	packet.channels = channels;
	packet.key = key;
	packet.exposure = exposure;
	packet.gain = gain;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;
	packet.local_z = local_z;
	packet.lat = lat;
	packet.lon = lon;
	packet.alt = alt;
	packet.ground_x = ground_x;
	packet.ground_y = ground_y;
	packet.ground_z = ground_z;

        memcpy(_MAV_PAYLOAD(msg), &packet, 92);
#endif

	msg->msgid = MAVLINK_MSG_ID_IMAGE_AVAILABLE;
	return mavlink_finalize_message(msg, system_id, component_id, 92);
}

/**
 * @brief Pack a image_available message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param cam_id Camera id
 * @param cam_no Camera # (starts with 0)
 * @param timestamp Timestamp
 * @param valid_until Until which timestamp this buffer will stay valid
 * @param img_seq The image sequence number
 * @param img_buf_index Position of the image in the buffer, starts with 0
 * @param width Image width
 * @param height Image height
 * @param depth Image depth
 * @param channels Image channels
 * @param key Shared memory area key
 * @param exposure Exposure time, in microseconds
 * @param gain Camera gain
 * @param roll Roll angle in rad
 * @param pitch Pitch angle in rad
 * @param yaw Yaw angle in rad
 * @param local_z Local frame Z coordinate (height over ground)
 * @param lat GPS X coordinate
 * @param lon GPS Y coordinate
 * @param alt Global frame altitude
 * @param ground_x Ground truth X
 * @param ground_y Ground truth Y
 * @param ground_z Ground truth Z
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_image_available_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           UInt64 cam_id,byte cam_no,UInt64 timestamp,UInt64 valid_until,UInt32 img_seq,UInt32 img_buf_index,UInt16 width,UInt16 height,UInt16 depth,byte channels,UInt32 key,UInt32 exposure,Single gain,Single roll,Single pitch,Single yaw,Single local_z,Single lat,Single lon,Single alt,Single ground_x,Single ground_y,Single ground_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[92];
	_mav_put_UInt64(buf, 0, cam_id);
	_mav_put_byte(buf, 8, cam_no);
	_mav_put_UInt64(buf, 9, timestamp);
	_mav_put_UInt64(buf, 17, valid_until);
	_mav_put_UInt32(buf, 25, img_seq);
	_mav_put_UInt32(buf, 29, img_buf_index);
	_mav_put_UInt16(buf, 33, width);
	_mav_put_UInt16(buf, 35, height);
	_mav_put_UInt16(buf, 37, depth);
	_mav_put_byte(buf, 39, channels);
	_mav_put_UInt32(buf, 40, key);
	_mav_put_UInt32(buf, 44, exposure);
	_mav_put_Single(buf, 48, gain);
	_mav_put_Single(buf, 52, roll);
	_mav_put_Single(buf, 56, pitch);
	_mav_put_Single(buf, 60, yaw);
	_mav_put_Single(buf, 64, local_z);
	_mav_put_Single(buf, 68, lat);
	_mav_put_Single(buf, 72, lon);
	_mav_put_Single(buf, 76, alt);
	_mav_put_Single(buf, 80, ground_x);
	_mav_put_Single(buf, 84, ground_y);
	_mav_put_Single(buf, 88, ground_z);

        memcpy(_MAV_PAYLOAD(msg), buf, 92);
#else
	mavlink_image_available_t packet;
	packet.cam_id = cam_id;
	packet.cam_no = cam_no;
	packet.timestamp = timestamp;
	packet.valid_until = valid_until;
	packet.img_seq = img_seq;
	packet.img_buf_index = img_buf_index;
	packet.width = width;
	packet.height = height;
	packet.depth = depth;
	packet.channels = channels;
	packet.key = key;
	packet.exposure = exposure;
	packet.gain = gain;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;
	packet.local_z = local_z;
	packet.lat = lat;
	packet.lon = lon;
	packet.alt = alt;
	packet.ground_x = ground_x;
	packet.ground_y = ground_y;
	packet.ground_z = ground_z;

        memcpy(_MAV_PAYLOAD(msg), &packet, 92);
#endif

	msg->msgid = MAVLINK_MSG_ID_IMAGE_AVAILABLE;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 92);
}

/**
 * @brief Encode a image_available struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param image_available C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_image_available_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_image_available_t* image_available)
{
	return mavlink_msg_image_available_pack(system_id, component_id, msg, image_available->cam_id, image_available->cam_no, image_available->timestamp, image_available->valid_until, image_available->img_seq, image_available->img_buf_index, image_available->width, image_available->height, image_available->depth, image_available->channels, image_available->key, image_available->exposure, image_available->gain, image_available->roll, image_available->pitch, image_available->yaw, image_available->local_z, image_available->lat, image_available->lon, image_available->alt, image_available->ground_x, image_available->ground_y, image_available->ground_z);
}

/**
 * @brief Send a image_available message
 * @param chan MAVLink channel to send the message
 *
 * @param cam_id Camera id
 * @param cam_no Camera # (starts with 0)
 * @param timestamp Timestamp
 * @param valid_until Until which timestamp this buffer will stay valid
 * @param img_seq The image sequence number
 * @param img_buf_index Position of the image in the buffer, starts with 0
 * @param width Image width
 * @param height Image height
 * @param depth Image depth
 * @param channels Image channels
 * @param key Shared memory area key
 * @param exposure Exposure time, in microseconds
 * @param gain Camera gain
 * @param roll Roll angle in rad
 * @param pitch Pitch angle in rad
 * @param yaw Yaw angle in rad
 * @param local_z Local frame Z coordinate (height over ground)
 * @param lat GPS X coordinate
 * @param lon GPS Y coordinate
 * @param alt Global frame altitude
 * @param ground_x Ground truth X
 * @param ground_y Ground truth Y
 * @param ground_z Ground truth Z
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_image_available_send(mavlink_channel_t chan, UInt64 cam_id, byte cam_no, UInt64 timestamp, UInt64 valid_until, UInt32 img_seq, UInt32 img_buf_index, UInt16 width, UInt16 height, UInt16 depth, byte channels, UInt32 key, UInt32 exposure, Single gain, Single roll, Single pitch, Single yaw, Single local_z, Single lat, Single lon, Single alt, Single ground_x, Single ground_y, Single ground_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[92];
	_mav_put_UInt64(buf, 0, cam_id);
	_mav_put_byte(buf, 8, cam_no);
	_mav_put_UInt64(buf, 9, timestamp);
	_mav_put_UInt64(buf, 17, valid_until);
	_mav_put_UInt32(buf, 25, img_seq);
	_mav_put_UInt32(buf, 29, img_buf_index);
	_mav_put_UInt16(buf, 33, width);
	_mav_put_UInt16(buf, 35, height);
	_mav_put_UInt16(buf, 37, depth);
	_mav_put_byte(buf, 39, channels);
	_mav_put_UInt32(buf, 40, key);
	_mav_put_UInt32(buf, 44, exposure);
	_mav_put_Single(buf, 48, gain);
	_mav_put_Single(buf, 52, roll);
	_mav_put_Single(buf, 56, pitch);
	_mav_put_Single(buf, 60, yaw);
	_mav_put_Single(buf, 64, local_z);
	_mav_put_Single(buf, 68, lat);
	_mav_put_Single(buf, 72, lon);
	_mav_put_Single(buf, 76, alt);
	_mav_put_Single(buf, 80, ground_x);
	_mav_put_Single(buf, 84, ground_y);
	_mav_put_Single(buf, 88, ground_z);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IMAGE_AVAILABLE, buf, 92);
#else
	mavlink_image_available_t packet;
	packet.cam_id = cam_id;
	packet.cam_no = cam_no;
	packet.timestamp = timestamp;
	packet.valid_until = valid_until;
	packet.img_seq = img_seq;
	packet.img_buf_index = img_buf_index;
	packet.width = width;
	packet.height = height;
	packet.depth = depth;
	packet.channels = channels;
	packet.key = key;
	packet.exposure = exposure;
	packet.gain = gain;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;
	packet.local_z = local_z;
	packet.lat = lat;
	packet.lon = lon;
	packet.alt = alt;
	packet.ground_x = ground_x;
	packet.ground_y = ground_y;
	packet.ground_z = ground_z;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IMAGE_AVAILABLE, (const char *)&packet, 92);
#endif
}

#endif

// MESSAGE IMAGE_AVAILABLE UNPACKING


/**
 * @brief Get field cam_id from image_available message
 *
 * @return Camera id
 */
static inline UInt64 mavlink_msg_image_available_get_cam_id(const mavlink_message_t* msg)
{
	return _MAV_RETURN_UInt64(msg,  0);
}

/**
 * @brief Get field cam_no from image_available message
 *
 * @return Camera # (starts with 0)
 */
static inline byte mavlink_msg_image_available_get_cam_no(const mavlink_message_t* msg)
{
	return _MAV_RETURN_byte(msg,  8);
}

/**
 * @brief Get field timestamp from image_available message
 *
 * @return Timestamp
 */
static inline UInt64 mavlink_msg_image_available_get_timestamp(const mavlink_message_t* msg)
{
	return _MAV_RETURN_UInt64(msg,  9);
}

/**
 * @brief Get field valid_until from image_available message
 *
 * @return Until which timestamp this buffer will stay valid
 */
static inline UInt64 mavlink_msg_image_available_get_valid_until(const mavlink_message_t* msg)
{
	return _MAV_RETURN_UInt64(msg,  17);
}

/**
 * @brief Get field img_seq from image_available message
 *
 * @return The image sequence number
 */
static inline UInt32 mavlink_msg_image_available_get_img_seq(const mavlink_message_t* msg)
{
	return _MAV_RETURN_UInt32(msg,  25);
}

/**
 * @brief Get field img_buf_index from image_available message
 *
 * @return Position of the image in the buffer, starts with 0
 */
static inline UInt32 mavlink_msg_image_available_get_img_buf_index(const mavlink_message_t* msg)
{
	return _MAV_RETURN_UInt32(msg,  29);
}

/**
 * @brief Get field width from image_available message
 *
 * @return Image width
 */
static inline UInt16 mavlink_msg_image_available_get_width(const mavlink_message_t* msg)
{
	return _MAV_RETURN_UInt16(msg,  33);
}

/**
 * @brief Get field height from image_available message
 *
 * @return Image height
 */
static inline UInt16 mavlink_msg_image_available_get_height(const mavlink_message_t* msg)
{
	return _MAV_RETURN_UInt16(msg,  35);
}

/**
 * @brief Get field depth from image_available message
 *
 * @return Image depth
 */
static inline UInt16 mavlink_msg_image_available_get_depth(const mavlink_message_t* msg)
{
	return _MAV_RETURN_UInt16(msg,  37);
}

/**
 * @brief Get field channels from image_available message
 *
 * @return Image channels
 */
static inline byte mavlink_msg_image_available_get_channels(const mavlink_message_t* msg)
{
	return _MAV_RETURN_byte(msg,  39);
}

/**
 * @brief Get field key from image_available message
 *
 * @return Shared memory area key
 */
static inline UInt32 mavlink_msg_image_available_get_key(const mavlink_message_t* msg)
{
	return _MAV_RETURN_UInt32(msg,  40);
}

/**
 * @brief Get field exposure from image_available message
 *
 * @return Exposure time, in microseconds
 */
static inline UInt32 mavlink_msg_image_available_get_exposure(const mavlink_message_t* msg)
{
	return _MAV_RETURN_UInt32(msg,  44);
}

/**
 * @brief Get field gain from image_available message
 *
 * @return Camera gain
 */
static inline Single mavlink_msg_image_available_get_gain(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Single(msg,  48);
}

/**
 * @brief Get field roll from image_available message
 *
 * @return Roll angle in rad
 */
static inline Single mavlink_msg_image_available_get_roll(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Single(msg,  52);
}

/**
 * @brief Get field pitch from image_available message
 *
 * @return Pitch angle in rad
 */
static inline Single mavlink_msg_image_available_get_pitch(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Single(msg,  56);
}

/**
 * @brief Get field yaw from image_available message
 *
 * @return Yaw angle in rad
 */
static inline Single mavlink_msg_image_available_get_yaw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Single(msg,  60);
}

/**
 * @brief Get field local_z from image_available message
 *
 * @return Local frame Z coordinate (height over ground)
 */
static inline Single mavlink_msg_image_available_get_local_z(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Single(msg,  64);
}

/**
 * @brief Get field lat from image_available message
 *
 * @return GPS X coordinate
 */
static inline Single mavlink_msg_image_available_get_lat(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Single(msg,  68);
}

/**
 * @brief Get field lon from image_available message
 *
 * @return GPS Y coordinate
 */
static inline Single mavlink_msg_image_available_get_lon(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Single(msg,  72);
}

/**
 * @brief Get field alt from image_available message
 *
 * @return Global frame altitude
 */
static inline Single mavlink_msg_image_available_get_alt(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Single(msg,  76);
}

/**
 * @brief Get field ground_x from image_available message
 *
 * @return Ground truth X
 */
static inline Single mavlink_msg_image_available_get_ground_x(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Single(msg,  80);
}

/**
 * @brief Get field ground_y from image_available message
 *
 * @return Ground truth Y
 */
static inline Single mavlink_msg_image_available_get_ground_y(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Single(msg,  84);
}

/**
 * @brief Get field ground_z from image_available message
 *
 * @return Ground truth Z
 */
static inline Single mavlink_msg_image_available_get_ground_z(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Single(msg,  88);
}

/**
 * @brief Decode a image_available message into a struct
 *
 * @param msg The message to decode
 * @param image_available C-struct to decode the message contents into
 */
static inline void mavlink_msg_image_available_decode(const mavlink_message_t* msg, mavlink_image_available_t* image_available)
{
#if MAVLINK_NEED_BYTE_SWAP
	image_available->cam_id = mavlink_msg_image_available_get_cam_id(msg);
	image_available->cam_no = mavlink_msg_image_available_get_cam_no(msg);
	image_available->timestamp = mavlink_msg_image_available_get_timestamp(msg);
	image_available->valid_until = mavlink_msg_image_available_get_valid_until(msg);
	image_available->img_seq = mavlink_msg_image_available_get_img_seq(msg);
	image_available->img_buf_index = mavlink_msg_image_available_get_img_buf_index(msg);
	image_available->width = mavlink_msg_image_available_get_width(msg);
	image_available->height = mavlink_msg_image_available_get_height(msg);
	image_available->depth = mavlink_msg_image_available_get_depth(msg);
	image_available->channels = mavlink_msg_image_available_get_channels(msg);
	image_available->key = mavlink_msg_image_available_get_key(msg);
	image_available->exposure = mavlink_msg_image_available_get_exposure(msg);
	image_available->gain = mavlink_msg_image_available_get_gain(msg);
	image_available->roll = mavlink_msg_image_available_get_roll(msg);
	image_available->pitch = mavlink_msg_image_available_get_pitch(msg);
	image_available->yaw = mavlink_msg_image_available_get_yaw(msg);
	image_available->local_z = mavlink_msg_image_available_get_local_z(msg);
	image_available->lat = mavlink_msg_image_available_get_lat(msg);
	image_available->lon = mavlink_msg_image_available_get_lon(msg);
	image_available->alt = mavlink_msg_image_available_get_alt(msg);
	image_available->ground_x = mavlink_msg_image_available_get_ground_x(msg);
	image_available->ground_y = mavlink_msg_image_available_get_ground_y(msg);
	image_available->ground_z = mavlink_msg_image_available_get_ground_z(msg);
#else
	memcpy(image_available, _MAV_PAYLOAD(msg), 92);
#endif
}
