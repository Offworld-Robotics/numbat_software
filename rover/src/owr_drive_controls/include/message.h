#ifndef MESSAGE_H
#define MESSAGE_H

#define NUM_MSG 8

namespace serial_msg {
    struct message {
        uint32_t startMagic;
        double data[NUM_MSG];
        uint32_t endMagic;
    };

    const uint32_t startMagic = 0xFEEDBEEF;
    const uint32_t endMagic = 0xDEADBEEF;
}

#endif
