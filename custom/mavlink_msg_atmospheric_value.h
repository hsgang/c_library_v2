#pragma once
// MESSAGE ATMOSPHERIC_VALUE PACKING

#define MAVLINK_MSG_ID_ATMOSPHERIC_VALUE 55020


typedef struct __mavlink_atmospheric_value_t {
 float count; /*<  Count*/
 float temperature; /*<  Temperature*/
 float humidity; /*<  Humidity*/
 float pressure; /*<  Pressure*/
 float wind_direction; /*<  Wind Direction*/
 float wind_speed; /*<  Wind Speed*/
} mavlink_atmospheric_value_t;

#define MAVLINK_MSG_ID_ATMOSPHERIC_VALUE_LEN 24
#define MAVLINK_MSG_ID_ATMOSPHERIC_VALUE_MIN_LEN 24
#define MAVLINK_MSG_ID_55020_LEN 24
#define MAVLINK_MSG_ID_55020_MIN_LEN 24

#define MAVLINK_MSG_ID_ATMOSPHERIC_VALUE_CRC 155
#define MAVLINK_MSG_ID_55020_CRC 155



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ATMOSPHERIC_VALUE { \
    55020, \
    "ATMOSPHERIC_VALUE", \
    6, \
    {  { "count", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_atmospheric_value_t, count) }, \
         { "temperature", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_atmospheric_value_t, temperature) }, \
         { "humidity", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_atmospheric_value_t, humidity) }, \
         { "pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_atmospheric_value_t, pressure) }, \
         { "wind_direction", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_atmospheric_value_t, wind_direction) }, \
         { "wind_speed", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_atmospheric_value_t, wind_speed) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ATMOSPHERIC_VALUE { \
    "ATMOSPHERIC_VALUE", \
    6, \
    {  { "count", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_atmospheric_value_t, count) }, \
         { "temperature", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_atmospheric_value_t, temperature) }, \
         { "humidity", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_atmospheric_value_t, humidity) }, \
         { "pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_atmospheric_value_t, pressure) }, \
         { "wind_direction", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_atmospheric_value_t, wind_direction) }, \
         { "wind_speed", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_atmospheric_value_t, wind_speed) }, \
         } \
}
#endif

/**
 * @brief Pack a atmospheric_value message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param count  Count
 * @param temperature  Temperature
 * @param humidity  Humidity
 * @param pressure  Pressure
 * @param wind_direction  Wind Direction
 * @param wind_speed  Wind Speed
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_atmospheric_value_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float count, float temperature, float humidity, float pressure, float wind_direction, float wind_speed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ATMOSPHERIC_VALUE_LEN];
    _mav_put_float(buf, 0, count);
    _mav_put_float(buf, 4, temperature);
    _mav_put_float(buf, 8, humidity);
    _mav_put_float(buf, 12, pressure);
    _mav_put_float(buf, 16, wind_direction);
    _mav_put_float(buf, 20, wind_speed);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ATMOSPHERIC_VALUE_LEN);
#else
    mavlink_atmospheric_value_t packet;
    packet.count = count;
    packet.temperature = temperature;
    packet.humidity = humidity;
    packet.pressure = pressure;
    packet.wind_direction = wind_direction;
    packet.wind_speed = wind_speed;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ATMOSPHERIC_VALUE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ATMOSPHERIC_VALUE;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ATMOSPHERIC_VALUE_MIN_LEN, MAVLINK_MSG_ID_ATMOSPHERIC_VALUE_LEN, MAVLINK_MSG_ID_ATMOSPHERIC_VALUE_CRC);
}

/**
 * @brief Pack a atmospheric_value message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param count  Count
 * @param temperature  Temperature
 * @param humidity  Humidity
 * @param pressure  Pressure
 * @param wind_direction  Wind Direction
 * @param wind_speed  Wind Speed
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_atmospheric_value_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float count,float temperature,float humidity,float pressure,float wind_direction,float wind_speed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ATMOSPHERIC_VALUE_LEN];
    _mav_put_float(buf, 0, count);
    _mav_put_float(buf, 4, temperature);
    _mav_put_float(buf, 8, humidity);
    _mav_put_float(buf, 12, pressure);
    _mav_put_float(buf, 16, wind_direction);
    _mav_put_float(buf, 20, wind_speed);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ATMOSPHERIC_VALUE_LEN);
#else
    mavlink_atmospheric_value_t packet;
    packet.count = count;
    packet.temperature = temperature;
    packet.humidity = humidity;
    packet.pressure = pressure;
    packet.wind_direction = wind_direction;
    packet.wind_speed = wind_speed;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ATMOSPHERIC_VALUE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ATMOSPHERIC_VALUE;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ATMOSPHERIC_VALUE_MIN_LEN, MAVLINK_MSG_ID_ATMOSPHERIC_VALUE_LEN, MAVLINK_MSG_ID_ATMOSPHERIC_VALUE_CRC);
}

/**
 * @brief Encode a atmospheric_value struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param atmospheric_value C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_atmospheric_value_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_atmospheric_value_t* atmospheric_value)
{
    return mavlink_msg_atmospheric_value_pack(system_id, component_id, msg, atmospheric_value->count, atmospheric_value->temperature, atmospheric_value->humidity, atmospheric_value->pressure, atmospheric_value->wind_direction, atmospheric_value->wind_speed);
}

/**
 * @brief Encode a atmospheric_value struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param atmospheric_value C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_atmospheric_value_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_atmospheric_value_t* atmospheric_value)
{
    return mavlink_msg_atmospheric_value_pack_chan(system_id, component_id, chan, msg, atmospheric_value->count, atmospheric_value->temperature, atmospheric_value->humidity, atmospheric_value->pressure, atmospheric_value->wind_direction, atmospheric_value->wind_speed);
}

/**
 * @brief Send a atmospheric_value message
 * @param chan MAVLink channel to send the message
 *
 * @param count  Count
 * @param temperature  Temperature
 * @param humidity  Humidity
 * @param pressure  Pressure
 * @param wind_direction  Wind Direction
 * @param wind_speed  Wind Speed
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_atmospheric_value_send(mavlink_channel_t chan, float count, float temperature, float humidity, float pressure, float wind_direction, float wind_speed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ATMOSPHERIC_VALUE_LEN];
    _mav_put_float(buf, 0, count);
    _mav_put_float(buf, 4, temperature);
    _mav_put_float(buf, 8, humidity);
    _mav_put_float(buf, 12, pressure);
    _mav_put_float(buf, 16, wind_direction);
    _mav_put_float(buf, 20, wind_speed);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ATMOSPHERIC_VALUE, buf, MAVLINK_MSG_ID_ATMOSPHERIC_VALUE_MIN_LEN, MAVLINK_MSG_ID_ATMOSPHERIC_VALUE_LEN, MAVLINK_MSG_ID_ATMOSPHERIC_VALUE_CRC);
#else
    mavlink_atmospheric_value_t packet;
    packet.count = count;
    packet.temperature = temperature;
    packet.humidity = humidity;
    packet.pressure = pressure;
    packet.wind_direction = wind_direction;
    packet.wind_speed = wind_speed;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ATMOSPHERIC_VALUE, (const char *)&packet, MAVLINK_MSG_ID_ATMOSPHERIC_VALUE_MIN_LEN, MAVLINK_MSG_ID_ATMOSPHERIC_VALUE_LEN, MAVLINK_MSG_ID_ATMOSPHERIC_VALUE_CRC);
#endif
}

/**
 * @brief Send a atmospheric_value message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_atmospheric_value_send_struct(mavlink_channel_t chan, const mavlink_atmospheric_value_t* atmospheric_value)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_atmospheric_value_send(chan, atmospheric_value->count, atmospheric_value->temperature, atmospheric_value->humidity, atmospheric_value->pressure, atmospheric_value->wind_direction, atmospheric_value->wind_speed);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ATMOSPHERIC_VALUE, (const char *)atmospheric_value, MAVLINK_MSG_ID_ATMOSPHERIC_VALUE_MIN_LEN, MAVLINK_MSG_ID_ATMOSPHERIC_VALUE_LEN, MAVLINK_MSG_ID_ATMOSPHERIC_VALUE_CRC);
#endif
}

#if MAVLINK_MSG_ID_ATMOSPHERIC_VALUE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_atmospheric_value_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float count, float temperature, float humidity, float pressure, float wind_direction, float wind_speed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, count);
    _mav_put_float(buf, 4, temperature);
    _mav_put_float(buf, 8, humidity);
    _mav_put_float(buf, 12, pressure);
    _mav_put_float(buf, 16, wind_direction);
    _mav_put_float(buf, 20, wind_speed);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ATMOSPHERIC_VALUE, buf, MAVLINK_MSG_ID_ATMOSPHERIC_VALUE_MIN_LEN, MAVLINK_MSG_ID_ATMOSPHERIC_VALUE_LEN, MAVLINK_MSG_ID_ATMOSPHERIC_VALUE_CRC);
#else
    mavlink_atmospheric_value_t *packet = (mavlink_atmospheric_value_t *)msgbuf;
    packet->count = count;
    packet->temperature = temperature;
    packet->humidity = humidity;
    packet->pressure = pressure;
    packet->wind_direction = wind_direction;
    packet->wind_speed = wind_speed;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ATMOSPHERIC_VALUE, (const char *)packet, MAVLINK_MSG_ID_ATMOSPHERIC_VALUE_MIN_LEN, MAVLINK_MSG_ID_ATMOSPHERIC_VALUE_LEN, MAVLINK_MSG_ID_ATMOSPHERIC_VALUE_CRC);
#endif
}
#endif

#endif

// MESSAGE ATMOSPHERIC_VALUE UNPACKING


/**
 * @brief Get field count from atmospheric_value message
 *
 * @return  Count
 */
static inline float mavlink_msg_atmospheric_value_get_count(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field temperature from atmospheric_value message
 *
 * @return  Temperature
 */
static inline float mavlink_msg_atmospheric_value_get_temperature(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field humidity from atmospheric_value message
 *
 * @return  Humidity
 */
static inline float mavlink_msg_atmospheric_value_get_humidity(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field pressure from atmospheric_value message
 *
 * @return  Pressure
 */
static inline float mavlink_msg_atmospheric_value_get_pressure(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field wind_direction from atmospheric_value message
 *
 * @return  Wind Direction
 */
static inline float mavlink_msg_atmospheric_value_get_wind_direction(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field wind_speed from atmospheric_value message
 *
 * @return  Wind Speed
 */
static inline float mavlink_msg_atmospheric_value_get_wind_speed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Decode a atmospheric_value message into a struct
 *
 * @param msg The message to decode
 * @param atmospheric_value C-struct to decode the message contents into
 */
static inline void mavlink_msg_atmospheric_value_decode(const mavlink_message_t* msg, mavlink_atmospheric_value_t* atmospheric_value)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    atmospheric_value->count = mavlink_msg_atmospheric_value_get_count(msg);
    atmospheric_value->temperature = mavlink_msg_atmospheric_value_get_temperature(msg);
    atmospheric_value->humidity = mavlink_msg_atmospheric_value_get_humidity(msg);
    atmospheric_value->pressure = mavlink_msg_atmospheric_value_get_pressure(msg);
    atmospheric_value->wind_direction = mavlink_msg_atmospheric_value_get_wind_direction(msg);
    atmospheric_value->wind_speed = mavlink_msg_atmospheric_value_get_wind_speed(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ATMOSPHERIC_VALUE_LEN? msg->len : MAVLINK_MSG_ID_ATMOSPHERIC_VALUE_LEN;
        memset(atmospheric_value, 0, MAVLINK_MSG_ID_ATMOSPHERIC_VALUE_LEN);
    memcpy(atmospheric_value, _MAV_PAYLOAD(msg), len);
#endif
}
