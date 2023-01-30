#pragma once
// MESSAGE ATMOSPHERIC_SENSOR PACKING

#define MAVLINK_MSG_ID_ATMOSPHERIC_SENSOR 13000


typedef struct __mavlink_atmospheric_sensor_t {
 float temperature; /*< [cdegC] Temperature*/
 float humidity; /*< [c%] Humidity*/
 float press_abs; /*< [hPa] Absolute pressure*/
 float direction; /*< [deg] Wind direction (that wind is coming from).*/
 float speed; /*< [m/s] Wind speed in ground plane.*/
 float speed_z; /*< [m/s] Vertical wind speed.*/
 uint16_t sequence; /*<  Sequence number.*/
} mavlink_atmospheric_sensor_t;

#define MAVLINK_MSG_ID_ATMOSPHERIC_SENSOR_LEN 26
#define MAVLINK_MSG_ID_ATMOSPHERIC_SENSOR_MIN_LEN 26
#define MAVLINK_MSG_ID_13000_LEN 26
#define MAVLINK_MSG_ID_13000_MIN_LEN 26

#define MAVLINK_MSG_ID_ATMOSPHERIC_SENSOR_CRC 231
#define MAVLINK_MSG_ID_13000_CRC 231



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ATMOSPHERIC_SENSOR { \
    13000, \
    "ATMOSPHERIC_SENSOR", \
    7, \
    {  { "sequence", NULL, MAVLINK_TYPE_UINT16_T, 0, 24, offsetof(mavlink_atmospheric_sensor_t, sequence) }, \
         { "temperature", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_atmospheric_sensor_t, temperature) }, \
         { "humidity", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_atmospheric_sensor_t, humidity) }, \
         { "press_abs", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_atmospheric_sensor_t, press_abs) }, \
         { "direction", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_atmospheric_sensor_t, direction) }, \
         { "speed", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_atmospheric_sensor_t, speed) }, \
         { "speed_z", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_atmospheric_sensor_t, speed_z) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ATMOSPHERIC_SENSOR { \
    "ATMOSPHERIC_SENSOR", \
    7, \
    {  { "sequence", NULL, MAVLINK_TYPE_UINT16_T, 0, 24, offsetof(mavlink_atmospheric_sensor_t, sequence) }, \
         { "temperature", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_atmospheric_sensor_t, temperature) }, \
         { "humidity", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_atmospheric_sensor_t, humidity) }, \
         { "press_abs", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_atmospheric_sensor_t, press_abs) }, \
         { "direction", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_atmospheric_sensor_t, direction) }, \
         { "speed", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_atmospheric_sensor_t, speed) }, \
         { "speed_z", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_atmospheric_sensor_t, speed_z) }, \
         } \
}
#endif

/**
 * @brief Pack a atmospheric_sensor message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param sequence  Sequence number.
 * @param temperature [cdegC] Temperature
 * @param humidity [c%] Humidity
 * @param press_abs [hPa] Absolute pressure
 * @param direction [deg] Wind direction (that wind is coming from).
 * @param speed [m/s] Wind speed in ground plane.
 * @param speed_z [m/s] Vertical wind speed.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_atmospheric_sensor_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint16_t sequence, float temperature, float humidity, float press_abs, float direction, float speed, float speed_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ATMOSPHERIC_SENSOR_LEN];
    _mav_put_float(buf, 0, temperature);
    _mav_put_float(buf, 4, humidity);
    _mav_put_float(buf, 8, press_abs);
    _mav_put_float(buf, 12, direction);
    _mav_put_float(buf, 16, speed);
    _mav_put_float(buf, 20, speed_z);
    _mav_put_uint16_t(buf, 24, sequence);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ATMOSPHERIC_SENSOR_LEN);
#else
    mavlink_atmospheric_sensor_t packet;
    packet.temperature = temperature;
    packet.humidity = humidity;
    packet.press_abs = press_abs;
    packet.direction = direction;
    packet.speed = speed;
    packet.speed_z = speed_z;
    packet.sequence = sequence;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ATMOSPHERIC_SENSOR_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ATMOSPHERIC_SENSOR;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ATMOSPHERIC_SENSOR_MIN_LEN, MAVLINK_MSG_ID_ATMOSPHERIC_SENSOR_LEN, MAVLINK_MSG_ID_ATMOSPHERIC_SENSOR_CRC);
}

/**
 * @brief Pack a atmospheric_sensor message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param sequence  Sequence number.
 * @param temperature [cdegC] Temperature
 * @param humidity [c%] Humidity
 * @param press_abs [hPa] Absolute pressure
 * @param direction [deg] Wind direction (that wind is coming from).
 * @param speed [m/s] Wind speed in ground plane.
 * @param speed_z [m/s] Vertical wind speed.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_atmospheric_sensor_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint16_t sequence,float temperature,float humidity,float press_abs,float direction,float speed,float speed_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ATMOSPHERIC_SENSOR_LEN];
    _mav_put_float(buf, 0, temperature);
    _mav_put_float(buf, 4, humidity);
    _mav_put_float(buf, 8, press_abs);
    _mav_put_float(buf, 12, direction);
    _mav_put_float(buf, 16, speed);
    _mav_put_float(buf, 20, speed_z);
    _mav_put_uint16_t(buf, 24, sequence);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ATMOSPHERIC_SENSOR_LEN);
#else
    mavlink_atmospheric_sensor_t packet;
    packet.temperature = temperature;
    packet.humidity = humidity;
    packet.press_abs = press_abs;
    packet.direction = direction;
    packet.speed = speed;
    packet.speed_z = speed_z;
    packet.sequence = sequence;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ATMOSPHERIC_SENSOR_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ATMOSPHERIC_SENSOR;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ATMOSPHERIC_SENSOR_MIN_LEN, MAVLINK_MSG_ID_ATMOSPHERIC_SENSOR_LEN, MAVLINK_MSG_ID_ATMOSPHERIC_SENSOR_CRC);
}

/**
 * @brief Encode a atmospheric_sensor struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param atmospheric_sensor C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_atmospheric_sensor_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_atmospheric_sensor_t* atmospheric_sensor)
{
    return mavlink_msg_atmospheric_sensor_pack(system_id, component_id, msg, atmospheric_sensor->sequence, atmospheric_sensor->temperature, atmospheric_sensor->humidity, atmospheric_sensor->press_abs, atmospheric_sensor->direction, atmospheric_sensor->speed, atmospheric_sensor->speed_z);
}

/**
 * @brief Encode a atmospheric_sensor struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param atmospheric_sensor C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_atmospheric_sensor_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_atmospheric_sensor_t* atmospheric_sensor)
{
    return mavlink_msg_atmospheric_sensor_pack_chan(system_id, component_id, chan, msg, atmospheric_sensor->sequence, atmospheric_sensor->temperature, atmospheric_sensor->humidity, atmospheric_sensor->press_abs, atmospheric_sensor->direction, atmospheric_sensor->speed, atmospheric_sensor->speed_z);
}

/**
 * @brief Send a atmospheric_sensor message
 * @param chan MAVLink channel to send the message
 *
 * @param sequence  Sequence number.
 * @param temperature [cdegC] Temperature
 * @param humidity [c%] Humidity
 * @param press_abs [hPa] Absolute pressure
 * @param direction [deg] Wind direction (that wind is coming from).
 * @param speed [m/s] Wind speed in ground plane.
 * @param speed_z [m/s] Vertical wind speed.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_atmospheric_sensor_send(mavlink_channel_t chan, uint16_t sequence, float temperature, float humidity, float press_abs, float direction, float speed, float speed_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ATMOSPHERIC_SENSOR_LEN];
    _mav_put_float(buf, 0, temperature);
    _mav_put_float(buf, 4, humidity);
    _mav_put_float(buf, 8, press_abs);
    _mav_put_float(buf, 12, direction);
    _mav_put_float(buf, 16, speed);
    _mav_put_float(buf, 20, speed_z);
    _mav_put_uint16_t(buf, 24, sequence);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ATMOSPHERIC_SENSOR, buf, MAVLINK_MSG_ID_ATMOSPHERIC_SENSOR_MIN_LEN, MAVLINK_MSG_ID_ATMOSPHERIC_SENSOR_LEN, MAVLINK_MSG_ID_ATMOSPHERIC_SENSOR_CRC);
#else
    mavlink_atmospheric_sensor_t packet;
    packet.temperature = temperature;
    packet.humidity = humidity;
    packet.press_abs = press_abs;
    packet.direction = direction;
    packet.speed = speed;
    packet.speed_z = speed_z;
    packet.sequence = sequence;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ATMOSPHERIC_SENSOR, (const char *)&packet, MAVLINK_MSG_ID_ATMOSPHERIC_SENSOR_MIN_LEN, MAVLINK_MSG_ID_ATMOSPHERIC_SENSOR_LEN, MAVLINK_MSG_ID_ATMOSPHERIC_SENSOR_CRC);
#endif
}

/**
 * @brief Send a atmospheric_sensor message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_atmospheric_sensor_send_struct(mavlink_channel_t chan, const mavlink_atmospheric_sensor_t* atmospheric_sensor)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_atmospheric_sensor_send(chan, atmospheric_sensor->sequence, atmospheric_sensor->temperature, atmospheric_sensor->humidity, atmospheric_sensor->press_abs, atmospheric_sensor->direction, atmospheric_sensor->speed, atmospheric_sensor->speed_z);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ATMOSPHERIC_SENSOR, (const char *)atmospheric_sensor, MAVLINK_MSG_ID_ATMOSPHERIC_SENSOR_MIN_LEN, MAVLINK_MSG_ID_ATMOSPHERIC_SENSOR_LEN, MAVLINK_MSG_ID_ATMOSPHERIC_SENSOR_CRC);
#endif
}

#if MAVLINK_MSG_ID_ATMOSPHERIC_SENSOR_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_atmospheric_sensor_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint16_t sequence, float temperature, float humidity, float press_abs, float direction, float speed, float speed_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, temperature);
    _mav_put_float(buf, 4, humidity);
    _mav_put_float(buf, 8, press_abs);
    _mav_put_float(buf, 12, direction);
    _mav_put_float(buf, 16, speed);
    _mav_put_float(buf, 20, speed_z);
    _mav_put_uint16_t(buf, 24, sequence);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ATMOSPHERIC_SENSOR, buf, MAVLINK_MSG_ID_ATMOSPHERIC_SENSOR_MIN_LEN, MAVLINK_MSG_ID_ATMOSPHERIC_SENSOR_LEN, MAVLINK_MSG_ID_ATMOSPHERIC_SENSOR_CRC);
#else
    mavlink_atmospheric_sensor_t *packet = (mavlink_atmospheric_sensor_t *)msgbuf;
    packet->temperature = temperature;
    packet->humidity = humidity;
    packet->press_abs = press_abs;
    packet->direction = direction;
    packet->speed = speed;
    packet->speed_z = speed_z;
    packet->sequence = sequence;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ATMOSPHERIC_SENSOR, (const char *)packet, MAVLINK_MSG_ID_ATMOSPHERIC_SENSOR_MIN_LEN, MAVLINK_MSG_ID_ATMOSPHERIC_SENSOR_LEN, MAVLINK_MSG_ID_ATMOSPHERIC_SENSOR_CRC);
#endif
}
#endif

#endif

// MESSAGE ATMOSPHERIC_SENSOR UNPACKING


/**
 * @brief Get field sequence from atmospheric_sensor message
 *
 * @return  Sequence number.
 */
static inline uint16_t mavlink_msg_atmospheric_sensor_get_sequence(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  24);
}

/**
 * @brief Get field temperature from atmospheric_sensor message
 *
 * @return [cdegC] Temperature
 */
static inline float mavlink_msg_atmospheric_sensor_get_temperature(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field humidity from atmospheric_sensor message
 *
 * @return [c%] Humidity
 */
static inline float mavlink_msg_atmospheric_sensor_get_humidity(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field press_abs from atmospheric_sensor message
 *
 * @return [hPa] Absolute pressure
 */
static inline float mavlink_msg_atmospheric_sensor_get_press_abs(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field direction from atmospheric_sensor message
 *
 * @return [deg] Wind direction (that wind is coming from).
 */
static inline float mavlink_msg_atmospheric_sensor_get_direction(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field speed from atmospheric_sensor message
 *
 * @return [m/s] Wind speed in ground plane.
 */
static inline float mavlink_msg_atmospheric_sensor_get_speed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field speed_z from atmospheric_sensor message
 *
 * @return [m/s] Vertical wind speed.
 */
static inline float mavlink_msg_atmospheric_sensor_get_speed_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Decode a atmospheric_sensor message into a struct
 *
 * @param msg The message to decode
 * @param atmospheric_sensor C-struct to decode the message contents into
 */
static inline void mavlink_msg_atmospheric_sensor_decode(const mavlink_message_t* msg, mavlink_atmospheric_sensor_t* atmospheric_sensor)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    atmospheric_sensor->temperature = mavlink_msg_atmospheric_sensor_get_temperature(msg);
    atmospheric_sensor->humidity = mavlink_msg_atmospheric_sensor_get_humidity(msg);
    atmospheric_sensor->press_abs = mavlink_msg_atmospheric_sensor_get_press_abs(msg);
    atmospheric_sensor->direction = mavlink_msg_atmospheric_sensor_get_direction(msg);
    atmospheric_sensor->speed = mavlink_msg_atmospheric_sensor_get_speed(msg);
    atmospheric_sensor->speed_z = mavlink_msg_atmospheric_sensor_get_speed_z(msg);
    atmospheric_sensor->sequence = mavlink_msg_atmospheric_sensor_get_sequence(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ATMOSPHERIC_SENSOR_LEN? msg->len : MAVLINK_MSG_ID_ATMOSPHERIC_SENSOR_LEN;
        memset(atmospheric_sensor, 0, MAVLINK_MSG_ID_ATMOSPHERIC_SENSOR_LEN);
    memcpy(atmospheric_sensor, _MAV_PAYLOAD(msg), len);
#endif
}
