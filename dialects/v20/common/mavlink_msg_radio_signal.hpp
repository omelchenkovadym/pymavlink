// MESSAGE RADIO_SIGNAL support class

#pragma once

namespace mavlink {
namespace common {
namespace msg {

/**
 * @brief RADIO_SIGNAL message
 *
 * Indicators of radio signal search results.
 */
struct RADIO_SIGNAL : mavlink::Message {
    static constexpr msgid_t MSG_ID = 13000;
    static constexpr size_t LENGTH = 8;
    static constexpr size_t MIN_LENGTH = 8;
    static constexpr uint8_t CRC_EXTRA = 231;
    static constexpr auto NAME = "RADIO_SIGNAL";


    float rate; /*< [Hz] Rate of baseline signal */
    int16_t heading; /*< [deg] Direction of the signal relative to the direction of the
                UAV (0-180).
             */
    int16_t level; /*< [dB] Signal level, from -140 to 0. */


    inline std::string get_name(void) const override
    {
            return NAME;
    }

    inline Info get_message_info(void) const override
    {
            return { MSG_ID, LENGTH, MIN_LENGTH, CRC_EXTRA };
    }

    inline std::string to_yaml(void) const override
    {
        std::stringstream ss;

        ss << NAME << ":" << std::endl;
        ss << "  rate: " << rate << std::endl;
        ss << "  heading: " << heading << std::endl;
        ss << "  level: " << level << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << rate;                          // offset: 0
        map << heading;                       // offset: 4
        map << level;                         // offset: 6
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> rate;                          // offset: 0
        map >> heading;                       // offset: 4
        map >> level;                         // offset: 6
    }
};

} // namespace msg
} // namespace common
} // namespace mavlink
