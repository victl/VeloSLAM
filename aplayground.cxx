#include "HDLFrame.h"
#include "vtkPacketFileReader.h"
#include "vtkPacketFileWriter.h"
#include <iostream>
#include <iomanip>
#include <boost/shared_ptr.hpp>

using namespace std;

struct hehe {
    fpos_t fpos;
};

int main(int argc, char* argv[]) {
    std::string packet(1206, 'a');
    char c = 'a';
    for (int i = 0; i < 1206; ++i) {
        packet[i] = i % 26 + c;
    }
    ptime t = microsec_clock::local_time();
    vtkPacketFileReader reader;
    vtkPacketFileWriter writer;
    writer.open("/tmp/test.pcap");
    writer.writePacket(reinterpret_cast<const unsigned char*>(packet.c_str()), 1206, t);
    writer.writePacket(reinterpret_cast<const unsigned char*>(packet.c_str()), 1206, t);
    writer.writePacket(reinterpret_cast<const unsigned char*>(packet.c_str()), 1206, t);
    writer.close();
    std::vector<fpos_t> filePositions;
    long long offset = PCAP_GLOBAL_HEADER_LEN;
    for (int i = 0; i < 3; ++i) {
        hehe* hep = new hehe;
        NUM_TO_FPOS_T(hep->fpos, offset);
        offset += PCAP_PACKET_LEN;
        filePositions.push_back(hep->fpos);
        delete hep;
    }
    reader.open("/tmp/test.pcap");
    const unsigned char* data = 0;
    unsigned int dataLength = 0;
    ptime packetTime;
    for (auto & val : filePositions) {
        reader.setFilePosition(&val);
        reader.nextPacket(data, dataLength, packetTime);
        std::string s(reinterpret_cast<const char*>(data), dataLength);
        assert(s == packet);
        assert(packetTime == t);
    }
    reader.close();
    cout << "Well done!" << endl;
}
