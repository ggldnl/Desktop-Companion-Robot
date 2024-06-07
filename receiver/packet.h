#ifndef ARRAY_PACKET_H
#define ARRAY_PACKET_H

#include <vector>
#include <algorithm>

struct Packet {
  int arrayId;
  int totalPackets;
  int width;
  int height;
  std::vector<std::vector<byte>> packets;
  int packetsReceived;

  Packet(int id, int total, int w, int h) 
    : arrayId(id), totalPackets(total), width(w), height(h), packets(total), packetsReceived(0) {}

  void addPacket(int seqNum, const std::vector<byte>& data) {
    packets[seqNum] = data;
    packetsReceived++;
  }

  bool isComplete() const {
    return packetsReceived == totalPackets;
  }

  std::vector<byte> reconstructArray() const {
    std::vector<byte> fullArray;
    for (const auto& packet : packets) {
      fullArray.insert(fullArray.end(), packet.begin(), packet.end());
    }
    return fullArray;
  }
};

bool comparePacket(const Packet& a, const Packet& b) {
  return a.arrayId < b.arrayId;
}

#endif // ARRAY_PACKET_H
