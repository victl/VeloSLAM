#include <fstream>
#include <iostream>
#include <vector>
#include "type_defs.h"
#include "HDLFrame.h"
#include "HDLManager.h"

int main(int argc, char* argv[]) {
    HDLManager hdlMgr(5);
//    hdlMgr.loadOffline("/Users/victor/Repo/HDL_Data/718/meta/20160718_170312.carposes"
//                       , "/Users/victor/Repo/HDL_Data/718/meta/20160718T170344.105134.pcap");
//    hdlMgr.saveHDLMeta();
//    hdlMgr.saveINSMeta();
    hdlMgr.setBufferDir("/Volumes/Camp/victor/Repo/HDL_Data/718/meta", false);
    assert(hdlMgr.loadHDLMeta());
    assert(hdlMgr.loadINSMeta());
    auto frame = hdlMgr.getAllFrameMeta().at(5);
    auto f = hdlMgr.getFrameAt(frame->timestamp);
    for (int i = 0; i < f->points->size(); ++i) {
        f->dumpToPCD("/tmp", i);
    }
    frame = hdlMgr.getAllFrameMeta().at(50);
    f = hdlMgr.getFrameAt(frame->timestamp);
    for (int i = 0; i < f->points->size(); ++i) {
        f->dumpToPCD("/tmp", i);
    }
    std::cout << "Done.\n";
}
