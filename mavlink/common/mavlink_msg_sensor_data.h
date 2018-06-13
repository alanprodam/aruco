#pragma once
// MESSAGE SENSOR_DATA PACKING

#define MAVLINK_MSG_ID_SENSOR_DATA 188

MAVPACKED(
typedef struct __mavlink_sensor_data_t {
 float temp_container; /*< Temperature inside the container*/
 float hum_container; /*< RH inside the container*/
 float temp_control; /*< Temperature outside the container*/
 float hum_control; /*< RH outside the container*/
 float spare2; /*< Spare param for future development*/
 float spare3; /*< Spare param for future development*/
 float spare4; /*< Spare param for future development*/
 uint8_t state_flag; /*< Box status: 1 - Unlock; 2 - Lock; 3 - Deploy*/
}) mavlink_sensor_data_t;

#define MAVLINK_MSG_ID_SENSOR_DATA_LEN 29
#define MAVLINK_MSG_ID_SENSOR_DATA_MIN_LEN 29
#define MAVLINK_MSG_ID_188_LEN 29
#define MAVLINK_MSG_ID_188_MIN_LEN 29

#define MAVLINK_MSG_ID_SENSOR_DATA_CRC 92
#define MAVLINK_MSG_ID_188_CRC 92



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SENSOR_DATA { \
    188, \
    "SENSOR_DATA", \
    8, \
    {  { "temp_container", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_sensor_data_t, temp_container) }, \
         { "hum_container", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_sensor_data_t, hum_container) }, \
         { "temp_control", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_sensor_data_t, temp_control) }, \
         { "hum_control", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_sensor_data_t, hum_control) }, \
         { "state_flag", NULL, MAVLINK_TYPE_UINT8_T, 0, 28, offsetof(mavlink_sensor_data_t, state_flag) }, \
         { "spare2", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_sensor_data_t, spare2) }, \
         { "spare3", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_sensor_data_t, spare3) }, \
         { "spare4", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_sensor_data_t, spare4) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SENSOR_DATA { \
    "SENSOR_DATA", \
    8, \
    {  { "temp_container", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_sensor_data_t, temp_container) }, \
         { "hum_container", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_sensor_data_t, hum_container) }, \
         { "temp_control", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_sensor_data_t, temp_control) }, \
         { "hum_control", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_sensor_data_t, hum_control) }, \
         { "state_flag", NULL, MAVLINK_TYPE_UINT8_T, 0, 28, offsetof(mavlink_sensor_data_t, state_flag) }, \
         { "spare2", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_sensor_data_t, spare2) }, \
         { "spare3", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_sensor_data_t, spare3) }, \
         { "spare4", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_sensor_data_t, spare4) }, \
         } \
}
#endif

/**
 * @brief Pack a sensor_data message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param temp_container Temperature inside the container
 * @param hum_container RH inside the container
 * @param temp_control Temperature outside the container
 * @param hum_control RH outside the container
 * @param state_flag Box status: 1 - Unlock; 2 - Lock; 3 - Deploy
 * @param spare2 Spare param for future development
 * @param spare3 Spare param for future development
 * @param spare4 Spare param for future development
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_sensor_data_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float temp_container, float hum_container, float temp_control, float hum_control, uint8_t state_flag, float spare2, float spare3, float spare4)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SENSOR_DATA_LEN];
    _mav_put_float(buf, 0, temp_container);
    _mav_put_float(buf, 4, hum_container);
    _mav_put_float(buf, 8, temp_control);
    _mav_put_float(buf, 12, hum_control);
    _mav_put_float(buf, 16, spare2);
    _mav_put_float(buf, 20, spare3);
    _mav_put_float(buf, 24, spare4);
    _mav_put_uint8_t(buf, 28, state_flag);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SENSOR_DATA_LEN);
#else
    mavlink_sensor_data_t packet;
    packet.temp_container = temp_container;
    packet.hum_container = hum_container;
    packet.temp_control = temp_control;
    packet.hum_control = hum_control;
    packet.spare2 = spare2;
    packet.spare3 = spare3;
    packet.spare4 = spare4;
    packet.state_flag = state_flag;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SENSOR_DATA_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SENSOR_DATA;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SENSOR_DATA_MIN_LEN, MAVLINK_MSG_ID_SENSOR_DATA_LEN, MAVLINK_MSG_ID_SENSOR_DATA_CRC);
}

/**
 * @brief Pack a sensor_data message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param temp_container Temperature inside the container
 * @param hum_container RH inside the container
 * @param temp_control Temperature outside the container
 * @param hum_control RH outside the container
 * @param state_flag Box status: 1 - Unlock; 2 - Lock; 3 - Deploy
 * @param spare2 Spare param for future development
 * @param spare3 Spare param for future development
 * @param spare4 Spare param for future development
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_sensor_data_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float temp_container,float hum_container,float temp_control,float hum_control,uint8_t state_flag,float spare2,float spare3,float spare4)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SENSOR_DATA_LEN];
    _mav_put_float(buf, 0, temp_container);
    _mav_put_float(buf, 4, hum_container);
    _mav_put_float(buf, 8, temp_control);
    _mav_put_float(buf, 12, hum_control);
    _mav_put_float(buf, 16, spare2);
    _mav_put_float(buf, 20, spare3);
    _mav_put_float(buf, 24, spare4);
    _mav_put_uint8_t(buf, 28, state_flag);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SENSOR_DATA_LEN);
#else
    mavlink_sensor_data_t packet;
    packet.temp_container = temp_container;
    packet.hum_container = hum_container;
    packet.temp_control = temp_control;
    packet.hum_control = hum_control;
    packet.spare2 = spare2;
    packet.spare3 = spare3;
    packet.spare4 = spare4;
    packet.state_flag = state_flag;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SENSOR_DATA_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SENSOR_DATA;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SENSOR_DATA_MIN_LEN, MAVLINK_MSG_ID_SENSOR_DATA_LEN, MAVLINK_MSG_ID_SENSOR_DATA_CRC);
}

/**
 * @brief Encode a sensor_data struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param sensor_data C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_sensor_data_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_sensor_data_t* sensor_data)
{
    return mavlink_msg_sensor_data_pack(system_id, component_id, msg, sensor_data->temp_container, sensor_data->hum_container, sensor_data->temp_control, sensor_data->hum_control, sensor_data->state_flag, sensor_data->spare2, sensor_data->spare3, sensor_data->spare4);
}

/**
 * @brief Encode a sensor_data struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param sensor_data C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_sensor_data_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_sensor_data_t* sensor_data)
{
    return mavlink_msg_sensor_data_pack_chan(system_id, component_id, chan, msg, sensor_data->temp_container, sensor_data->hum_container, sensor_data->temp_control, sensor_data->hum_control, sensor_data->state_flag, sensor_data->spare2, sensor_data->spare3, sensor_data->spare4);
}

/**
 * @brief Send a sensor_data message
 * @param chan MAVLink channel to send the message
 *
 * @param temp_container Temperature inside the container
 * @param hum_container RH inside the container
 * @param temp_control Temperature outside the container
 * @param hum_control RH outside the container
 * @param state_flag Box status: 1 - Unlock; 2 - Lock; 3 - Deploy
 * @param spare2 Spare param for future development
 * @param spare3 Spare param for future development
 * @param spare4 Spare param for future development
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_sensor_data_send(mavlink_channel_t chan, float temp_container, float hum_container, float temp_control, float hum_control, uint8_t state_flag, float spare2, float spare3, float spare4)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SENSOR_DATA_LEN];
    _mav_put_float(buf, 0, temp_container);
    _mav_put_float(buf, 4, hum_container);
    _mav_put_float(buf, 8, temp_control);
    _mav_put_float(buf, 12, hum_control);
    _mav_put_float(buf, 16, spare2);
    _mav_put_float(buf, 20, spare3);
    _mav_put_float(buf, 24, spare4);
    _mav_put_uint8_t(buf, 28, state_flag);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SENSOR_DATA, buf, MAVLINK_MSG_ID_SENSOR_DATA_MIN_LEN, MAVLINK_MSG_ID_SENSOR_DATA_LEN, MAVLINK_MSG_ID_SENSOR_DATA_CRC);
#else
    mavlink_sensor_data_t packet;
    packet.temp_container = temp_container;
    packet.hum_container = hum_container;
    packet.temp_control = temp_control;
    packet.hum_control = hum_control;
    packet.spare2 = spare2;
    packet.spare3 = spare3;
    packet.spare4 = spare4;
    packet.state_flag = state_flag;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SENSOR_DATA, (const char *)&packet, MAVLINK_MSG_ID_SENSOR_DATA_MIN_LEN, MAVLINK_MSG_ID_SENSOR_DATA_LEN, MAVLINK_MSG_ID_SENSOR_DATA_CRC);
#endif
}

/**
 * @brief Send a sensor_data message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_sensor_data_send_struct(mavlink_channel_t chan, const mavlink_sensor_data_t* sensor_data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_sensor_data_send(chan, sensor_data->temp_container, sensor_data->hum_container, sensor_data->temp_control, sensor_data->hum_control, sensor_data->state_flag, sensor_data->spare2, sensor_data->spare3, sensor_data->spare4);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SENSOR_DATA, (const char *)sensor_data, MAVLINK_MSG_ID_SENSOR_DATA_MIN_LEN, MAVLINK_MSG_ID_SENSOR_DATA_LEN, MAVLINK_MSG_ID_SENSOR_DATA_CRC);
#endif
}

#if MAVLINK_MSG_ID_SENSOR_DATA_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_sensor_data_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float temp_container, float hum_container, float temp_control, float hum_control, uint8_t state_flag, float spare2, float spare3, float spare4)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, temp_container);
    _mav_put_float(buf, 4, hum_container);
    _mav_put_float(buf, 8, temp_control);
    _mav_put_float(buf, 12, hum_control);
    _mav_put_float(buf, 16, spare2);
    _mav_put_float(buf, 20, spare3);
    _mav_put_float(buf, 24, spare4);
    _mav_put_uint8_t(buf, 28, state_flag);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SENSOR_DATA, buf, MAVLINK_MSG_ID_SENSOR_DATA_MIN_LEN, MAVLINK_MSG_ID_SENSOR_DATA_LEN, MAVLINK_MSG_ID_SENSOR_DATA_CRC);
#else
    mavlink_sensor_data_t *packet = (mavlink_sensor_data_t *)msgbuf;
    packet->temp_container = temp_container;
    packet->hum_container = hum_container;
    packet->temp_control = temp_control;
    packet->hum_control = hum_control;
    packet->spare2 = spare2;
    packet->spare3 = spare3;
    packet->spare4 = spare4;
    packet->state_flag = state_flag;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SENSOR_DATA, (const char *)packet, MAVLINK_MSG_ID_SENSOR_DATA_MIN_LEN, MAVLINK_MSG_ID_SENSOR_DATA_LEN, MAVLINK_MSG_ID_SENSOR_DATA_CRC);
#endif
}
#endif

#endif

// MESSAGE SENSOR_DATA UNPACKING


/**
 * @brief Get field temp_container from sensor_data message
 *
 * @return Temperature inside the container
 */
static inline float mavlink_msg_sensor_data_get_temp_container(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field hum_container from sensor_data message
 *
 * @return RH inside the container
 */
static inline float mavlink_msg_sensor_data_get_hum_container(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field temp_control from sensor_data message
 *
 * @return Temperature outside the container
 */
static inline float mavlink_msg_sensor_data_get_temp_control(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field hum_control from sensor_data message
 *
 * @return RH outside the container
 */
static inline float mavlink_msg_sensor_data_get_hum_control(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field state_flag from sensor_data message
 *
 * @return Box status: 1 - Unlock; 2 - Lock; 3 - Deploy
 */
static inline uint8_t mavlink_msg_sensor_data_get_state_flag(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  28);
}

/**
 * @brief Get field spare2 from sensor_data message
 *
 * @return Spare param for future development
 */
static inline float mavlink_msg_sensor_data_get_spare2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field spare3 from sensor_data message
 *
 * @return Spare param for future development
 */
static inline float mavlink_msg_sensor_data_get_spare3(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field spare4 from sensor_data message
 *
 * @return Spare param for future development
 */
static inline float mavlink_msg_sensor_data_get_spare4(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Decode a sensor_data message into a struct
 *
 * @param msg The message to decode
 * @param sensor_data C-struct to decode the message contents into
 */
static inline void mavlink_msg_sensor_data_decode(const mavlink_message_t* msg, mavlink_sensor_data_t* sensor_data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    sensor_data->temp_container = mavlink_msg_sensor_data_get_temp_container(msg);
    sensor_data->hum_container = mavlink_msg_sensor_data_get_hum_container(msg);
    sensor_data->temp_control = mavlink_msg_sensor_data_get_temp_control(msg);
    sensor_data->hum_control = mavlink_msg_sensor_data_get_hum_control(msg);
    sensor_data->spare2 = mavlink_msg_sensor_data_get_spare2(msg);
    sensor_data->spare3 = mavlink_msg_sensor_data_get_spare3(msg);
    sensor_data->spare4 = mavlink_msg_sensor_data_get_spare4(msg);
    sensor_data->state_flag = mavlink_msg_sensor_data_get_state_flag(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SENSOR_DATA_LEN? msg->len : MAVLINK_MSG_ID_SENSOR_DATA_LEN;
        memset(sensor_data, 0, MAVLINK_MSG_ID_SENSOR_DATA_LEN);
    memcpy(sensor_data, _MAV_PAYLOAD(msg), len);
#endif
}
