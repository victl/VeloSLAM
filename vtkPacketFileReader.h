// Copyright 2013 Velodyne Acoustics, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkPacketFileReader.h

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
// .NAME vtkPacketFileReader -
// .SECTION Description
//

#ifndef __vtkPacketFileReader_h
#define __vtkPacketFileReader_h

#include <pcap.h>
#include <string>
#include <type_defs.h>
#include <boost/date_time.hpp>

/* Macro function that turns a number into a valid fpos_t.
 * Notes: The actual length of fpos_t on my system is 16 bytes, while as
 * 'long long' is 8 bytes. How wondering that this macro 'just works'...
 * Maybe the reason is that my system (X86 platform) is little-endian,
 * and the fpos_t type "is truly a character count..." (ref:
 * http://www.gnu.org/software/libc/manual/html_node/Portable-Positioning.html ).
 * So I guess it may fail on big-endian system. In that case, pls try this one:
 *
 * #define NUM_TO_FPOS_T(fpos, val) (*(reinterpret_cast<long long*>(&fpos + 8)) = (val))
 *
 * However, as stated in that ref, it "is an opaque data structure that contains
 * internal data to represent file offset and conversion state information". So
 * don't count on it. BE SURE to run the test case inside "fpos_test.cxx".
 * This macro is used by HDLManager.writePacketsLoop()
*/
#define NUM_TO_FPOS_T(fpos, val) (*(reinterpret_cast<long long*>(&fpos)) = (val))

#define PCAP_GLOBAL_HEADER_LEN 24

/* 1206 (actual data len)
   + 42 (udp header len)
   + 16 (pcap packet header len)
   = 1264
 *
 * ref: https://wiki.wireshark.org/Development/LibpcapFileFormat
 */
#define PCAP_PACKET_LEN 1264

// Some versions of libpcap do not have PCAP_NETMASK_UNKNOWN
#if !defined(PCAP_NETMASK_UNKNOWN)
  #define PCAP_NETMASK_UNKNOWN 0xffffffff
#endif

class vtkPacketFileReader
{
public:

  vtkPacketFileReader()
  {
    this->pcapFile = 0;
  }

  ~vtkPacketFileReader()
  {
    this->close();
  }

  bool open(const std::string& filename)
  {
      if (filename == this->fileName) {
          return true;
      }
      if (isOpen()) close();
    char errbuff[PCAP_ERRBUF_SIZE];
    pcap_t *pcapFile = pcap_open_offline(filename.c_str (), errbuff);
    if (!pcapFile)
      {
      this->lastError = errbuff;
      return false;
      }

    struct bpf_program filter;

    if (pcap_compile(pcapFile, &filter, "udp", 0, PCAP_NETMASK_UNKNOWN) == -1)
      {
      this->lastError = pcap_geterr(pcapFile);
      return false;
      }

    if (pcap_setfilter(pcapFile, &filter) == -1)
      {
      this->lastError = pcap_geterr(pcapFile);
      return false;
      }

    this->fileName = filename;
    this->pcapFile = pcapFile;
    this->startTime.tv_sec = this->startTime.tv_usec = 0;
    return true;
  }

  bool isOpen()
  {
    return (this->pcapFile != 0);
  }

  void close()
  {
    if (this->pcapFile)
      {
      pcap_close(this->pcapFile);
      this->pcapFile = 0;
      this->fileName.clear();
      }
  }

  const std::string& getLastError()
  {
    return this->lastError;
  }

  const std::string& getFileName()
  {
    return this->fileName;
  }

  void getFilePosition(fpos_t* position)
  {
#ifdef _MSC_VER
    pcap_fgetpos(this->PCAPFile, position);
#else
    FILE* f = pcap_file(this->pcapFile);
    fgetpos(f, position);
#endif
  }

  void setFilePosition(fpos_t* position)
  {
#ifdef _MSC_VER
    pcap_fsetpos(this->PCAPFile, position);
#else
    FILE* f = pcap_file(this->pcapFile);
    fsetpos(f, position);
#endif
  }

  bool nextPacket(const unsigned char*& data, unsigned int& dataLength, ptime& t, pcap_pkthdr** headerReference=NULL)
  {
    if (!this->pcapFile)
      {
      return false;
      }

    struct pcap_pkthdr *header;
    int returnValue = pcap_next_ex(this->pcapFile, &header, &data);
    if (returnValue < 0)
      {
      this->close();
      return false;
      }

    if (headerReference != NULL)
      {
      *headerReference = header;
      dataLength = header->len;
      timevalToPtime(header->ts, t);
//      timeSinceStart = getElapsedTime(header->ts, this->startTime);
      return true;
      }

    // The ethernet header is 42 bytes long; unnecessary
    const unsigned int bytesToSkip = 42;
    dataLength = header->len - bytesToSkip;
    data = data + bytesToSkip; // Will this action cause a tiny memory leak of 42 bytes?
    timevalToPtime(header->ts, t);
//    timeSinceStart = getElapsedTime(header->ts, this->startTime);
    return true;
  }

protected:

//  double getElapsedTime(const struct timeval& end, const struct timeval& start)
//  {
//    return (end.tv_sec - start.tv_sec) + (end.tv_usec - start.tv_usec) / 1000000.00;
//  }

  pcap_t* pcapFile;
  std::string fileName;
  std::string lastError;
  struct timeval startTime;
};

#endif
