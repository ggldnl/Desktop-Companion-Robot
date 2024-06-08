#include <cstdint>
#include <cstring> // for memcpy


class Message {
private:

  uint8_t totalPacketsNumber;
  uint8_t currentPacketsNumber;

public:

  // Define exit codes (exceptions are disabled)
  const uint8_t INVALID_SEQUENCE_NUMBER_EXCEPTION = 1;

  uint8_t width;
  uint8_t height;
  uint8_t messageId;
  uint8_t* byteArray; // TODO private
  int byteArrayLength; // TODO private

  Message(uint8_t messageId, uint8_t totalPacketsNumber, uint8_t payloadSize, uint8_t width, uint8_t height) :
    width(width),
    height(height),
    messageId(messageId),
    totalPacketsNumber(totalPacketsNumber),
    currentPacketsNumber(0),
    byteArrayLength(payloadSize * totalPacketsNumber) {
    byteArray = new uint8_t[byteArrayLength];
  }

  ~Message() {
    delete[] byteArray;
  }

  uint8_t addData(uint8_t seqNum, const uint8_t* payload, uint8_t payloadSize) {
    if (seqNum >= totalPacketsNumber) {
      // Invalid sequence number
      return INVALID_SEQUENCE_NUMBER_EXCEPTION;
    }

    /*
    // Check if adding the data would exceed allocated memory
    if ((seqNum + 1) * payloadSize > byteArrayLength) {
      // Not enough memory allocated
      return INSUFFICIENT_MEMORY_EXCEPTION;
    }
    */

    std::memcpy(&byteArray[seqNum * payloadSize], payload, payloadSize);
    // std::memcpy(byteArray + (seqNum * payloadSize), payload, payloadSize);
    currentPacketsNumber++;
    return 0;
  }

  bool isComplete() {
    return currentPacketsNumber == totalPacketsNumber;
  }
};
