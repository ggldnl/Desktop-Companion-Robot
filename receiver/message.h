#include <cstdint>
#include <cstring> // for memcpy

class Message {
private:

    uint8_t total_num_packets;
    uint8_t current_num_packets;
    uint8_t* byte_array;
    uint8_t byte_array_length;

public:

    uint8_t array_id;

    Message(uint8_t id, uint8_t total_packets, uint8_t data_length) :
        array_id(id),
        total_num_packets(total_packets),
        current_num_packets(0),
        byte_array_length(data_length) {
        byte_array = new uint8_t[data_length];
    }

    ~Message() {
        delete[] byte_array;
    }

    uint8_t addData(uint8_t seqNum, const uint8_t* payload, uint8_t payloadSize) {
        if (seqNum >= total_num_packets) {
            // Invalid sequence number
            return 1;
        }

        std::memcpy(byte_array + (seqNum * payloadSize), payload, payloadSize);
        current_num_packets++;
        return 0;
    }

    bool isComplete() {
        return current_num_packets == total_num_packets;
    }
};
