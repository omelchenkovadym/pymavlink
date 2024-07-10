#pragma once
// MESSAGE RADIO_SIGNAL PACKING

#define MAVLINK_MSG_ID_RADIO_SIGNAL 13000


typedef struct __mavlink_radio_signal_t {
 float rate; /*< [Hz] Rate of baseline signal*/
 int16_t heading; /*< [deg] Direction of the signal relative to the direction of the
                UAV (0-180).
            */
 int16_t level; /*< [dB] Signal level, from -140 to 0.*/
} mavlink_radio_signal_t;

#define MAVLINK_MSG_ID_RADIO_SIGNAL_LEN 8
#define MAVLINK_MSG_ID_RADIO_SIGNAL_MIN_LEN 8
#define MAVLINK_MSG_ID_13000_LEN 8
#define MAVLINK_MSG_ID_13000_MIN_LEN 8

#define MAVLINK_MSG_ID_RADIO_SIGNAL_CRC 231
#define MAVLINK_MSG_ID_13000_CRC 231



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_RADIO_SIGNAL { \
    13000, \
    "RADIO_SIGNAL", \
    3, \
    {  { "rate", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_radio_signal_t, rate) }, \
         { "heading", NULL, MAVLINK_TYPE_INT16_T, 0, 4, offsetof(mavlink_radio_signal_t, heading) }, \
         { "level", NULL, MAVLINK_TYPE_INT16_T, 0, 6, offsetof(mavlink_radio_signal_t, level) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_RADIO_SIGNAL { \
    "RADIO_SIGNAL", \
    3, \
    {  { "rate", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_radio_signal_t, rate) }, \
         { "heading", NULL, MAVLINK_TYPE_INT16_T, 0, 4, offsetof(mavlink_radio_signal_t, heading) }, \
         { "level", NULL, MAVLINK_TYPE_INT16_T, 0, 6, offsetof(mavlink_radio_signal_t, level) }, \
         } \
}
#endif

/**
 * @brief Pack a radio_signal message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param rate [Hz] Rate of baseline signal
 * @param heading [deg] Direction of the signal relative to the direction of the
                UAV (0-180).
            
 * @param level [dB] Signal level, from -140 to 0.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_radio_signal_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float rate, int16_t heading, int16_t level)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RADIO_SIGNAL_LEN];
    _mav_put_float(buf, 0, rate);
    _mav_put_int16_t(buf, 4, heading);
    _mav_put_int16_t(buf, 6, level);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RADIO_SIGNAL_LEN);
#else
    mavlink_radio_signal_t packet;
    packet.rate = rate;
    packet.heading = heading;
    packet.level = level;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RADIO_SIGNAL_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_RADIO_SIGNAL;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_RADIO_SIGNAL_MIN_LEN, MAVLINK_MSG_ID_RADIO_SIGNAL_LEN, MAVLINK_MSG_ID_RADIO_SIGNAL_CRC);
}

/**
 * @brief Pack a radio_signal message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param rate [Hz] Rate of baseline signal
 * @param heading [deg] Direction of the signal relative to the direction of the
                UAV (0-180).
            
 * @param level [dB] Signal level, from -140 to 0.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_radio_signal_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               float rate, int16_t heading, int16_t level)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RADIO_SIGNAL_LEN];
    _mav_put_float(buf, 0, rate);
    _mav_put_int16_t(buf, 4, heading);
    _mav_put_int16_t(buf, 6, level);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RADIO_SIGNAL_LEN);
#else
    mavlink_radio_signal_t packet;
    packet.rate = rate;
    packet.heading = heading;
    packet.level = level;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RADIO_SIGNAL_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_RADIO_SIGNAL;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_RADIO_SIGNAL_MIN_LEN, MAVLINK_MSG_ID_RADIO_SIGNAL_LEN, MAVLINK_MSG_ID_RADIO_SIGNAL_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_RADIO_SIGNAL_MIN_LEN, MAVLINK_MSG_ID_RADIO_SIGNAL_LEN);
#endif
}

/**
 * @brief Pack a radio_signal message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param rate [Hz] Rate of baseline signal
 * @param heading [deg] Direction of the signal relative to the direction of the
                UAV (0-180).
            
 * @param level [dB] Signal level, from -140 to 0.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_radio_signal_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float rate,int16_t heading,int16_t level)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RADIO_SIGNAL_LEN];
    _mav_put_float(buf, 0, rate);
    _mav_put_int16_t(buf, 4, heading);
    _mav_put_int16_t(buf, 6, level);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RADIO_SIGNAL_LEN);
#else
    mavlink_radio_signal_t packet;
    packet.rate = rate;
    packet.heading = heading;
    packet.level = level;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RADIO_SIGNAL_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_RADIO_SIGNAL;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_RADIO_SIGNAL_MIN_LEN, MAVLINK_MSG_ID_RADIO_SIGNAL_LEN, MAVLINK_MSG_ID_RADIO_SIGNAL_CRC);
}

/**
 * @brief Encode a radio_signal struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param radio_signal C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_radio_signal_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_radio_signal_t* radio_signal)
{
    return mavlink_msg_radio_signal_pack(system_id, component_id, msg, radio_signal->rate, radio_signal->heading, radio_signal->level);
}

/**
 * @brief Encode a radio_signal struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param radio_signal C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_radio_signal_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_radio_signal_t* radio_signal)
{
    return mavlink_msg_radio_signal_pack_chan(system_id, component_id, chan, msg, radio_signal->rate, radio_signal->heading, radio_signal->level);
}

/**
 * @brief Encode a radio_signal struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param radio_signal C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_radio_signal_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_radio_signal_t* radio_signal)
{
    return mavlink_msg_radio_signal_pack_status(system_id, component_id, _status, msg,  radio_signal->rate, radio_signal->heading, radio_signal->level);
}

/**
 * @brief Send a radio_signal message
 * @param chan MAVLink channel to send the message
 *
 * @param rate [Hz] Rate of baseline signal
 * @param heading [deg] Direction of the signal relative to the direction of the
                UAV (0-180).
            
 * @param level [dB] Signal level, from -140 to 0.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_radio_signal_send(mavlink_channel_t chan, float rate, int16_t heading, int16_t level)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RADIO_SIGNAL_LEN];
    _mav_put_float(buf, 0, rate);
    _mav_put_int16_t(buf, 4, heading);
    _mav_put_int16_t(buf, 6, level);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RADIO_SIGNAL, buf, MAVLINK_MSG_ID_RADIO_SIGNAL_MIN_LEN, MAVLINK_MSG_ID_RADIO_SIGNAL_LEN, MAVLINK_MSG_ID_RADIO_SIGNAL_CRC);
#else
    mavlink_radio_signal_t packet;
    packet.rate = rate;
    packet.heading = heading;
    packet.level = level;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RADIO_SIGNAL, (const char *)&packet, MAVLINK_MSG_ID_RADIO_SIGNAL_MIN_LEN, MAVLINK_MSG_ID_RADIO_SIGNAL_LEN, MAVLINK_MSG_ID_RADIO_SIGNAL_CRC);
#endif
}

/**
 * @brief Send a radio_signal message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_radio_signal_send_struct(mavlink_channel_t chan, const mavlink_radio_signal_t* radio_signal)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_radio_signal_send(chan, radio_signal->rate, radio_signal->heading, radio_signal->level);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RADIO_SIGNAL, (const char *)radio_signal, MAVLINK_MSG_ID_RADIO_SIGNAL_MIN_LEN, MAVLINK_MSG_ID_RADIO_SIGNAL_LEN, MAVLINK_MSG_ID_RADIO_SIGNAL_CRC);
#endif
}

#if MAVLINK_MSG_ID_RADIO_SIGNAL_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_radio_signal_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float rate, int16_t heading, int16_t level)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, rate);
    _mav_put_int16_t(buf, 4, heading);
    _mav_put_int16_t(buf, 6, level);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RADIO_SIGNAL, buf, MAVLINK_MSG_ID_RADIO_SIGNAL_MIN_LEN, MAVLINK_MSG_ID_RADIO_SIGNAL_LEN, MAVLINK_MSG_ID_RADIO_SIGNAL_CRC);
#else
    mavlink_radio_signal_t *packet = (mavlink_radio_signal_t *)msgbuf;
    packet->rate = rate;
    packet->heading = heading;
    packet->level = level;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RADIO_SIGNAL, (const char *)packet, MAVLINK_MSG_ID_RADIO_SIGNAL_MIN_LEN, MAVLINK_MSG_ID_RADIO_SIGNAL_LEN, MAVLINK_MSG_ID_RADIO_SIGNAL_CRC);
#endif
}
#endif

#endif

// MESSAGE RADIO_SIGNAL UNPACKING


/**
 * @brief Get field rate from radio_signal message
 *
 * @return [Hz] Rate of baseline signal
 */
static inline float mavlink_msg_radio_signal_get_rate(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field heading from radio_signal message
 *
 * @return [deg] Direction of the signal relative to the direction of the
                UAV (0-180).
            
 */
static inline int16_t mavlink_msg_radio_signal_get_heading(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  4);
}

/**
 * @brief Get field level from radio_signal message
 *
 * @return [dB] Signal level, from -140 to 0.
 */
static inline int16_t mavlink_msg_radio_signal_get_level(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  6);
}

/**
 * @brief Decode a radio_signal message into a struct
 *
 * @param msg The message to decode
 * @param radio_signal C-struct to decode the message contents into
 */
static inline void mavlink_msg_radio_signal_decode(const mavlink_message_t* msg, mavlink_radio_signal_t* radio_signal)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    radio_signal->rate = mavlink_msg_radio_signal_get_rate(msg);
    radio_signal->heading = mavlink_msg_radio_signal_get_heading(msg);
    radio_signal->level = mavlink_msg_radio_signal_get_level(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_RADIO_SIGNAL_LEN? msg->len : MAVLINK_MSG_ID_RADIO_SIGNAL_LEN;
        memset(radio_signal, 0, MAVLINK_MSG_ID_RADIO_SIGNAL_LEN);
    memcpy(radio_signal, _MAV_PAYLOAD(msg), len);
#endif
}
