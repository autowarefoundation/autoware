/* This file is part of the Pangolin Project.
 * http://github.com/stevenlovegrove/Pangolin
 *
 * Copyright (c) 2014 Steven Lovegrove
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

#ifndef PANGOLIN_PACKETSTREAM_H
#define PANGOLIN_PACKETSTREAM_H

#include <pangolin/platform.h>
#include <pangolin/utils/threadedfilebuf.h>
#include <pangolin/compat/function.h>
#include <pangolin/compat/mutex.h>
#include <pangolin/compat/condition_variable.h>
#include <pangolin/utils/picojson.h>
#include <stdint.h>


namespace pangolin
{

const static std::string PANGO_MAGIC = "PANGO";

const unsigned int TAG_LENGTH = 3;

#define PANGO_TAG(a,b,c) ( (c<<16) | (b<<8) | a)
const uint32_t TAG_PANGO_HDR   = PANGO_TAG('L', 'I', 'N');
const uint32_t TAG_PANGO_SYNC  = PANGO_TAG('S', 'Y', 'N');
const uint32_t TAG_PANGO_STATS = PANGO_TAG('S', 'T', 'A');
const uint32_t TAG_ADD_SOURCE  = PANGO_TAG('S', 'R', 'C');
const uint32_t TAG_SRC_JSON    = PANGO_TAG('J', 'S', 'N');
const uint32_t TAG_SRC_PACKET  = PANGO_TAG('P', 'K', 'T');
const uint32_t TAG_END         = PANGO_TAG('E', 'N', 'D');
#undef PANGO_TAG

struct PANGOLIN_EXPORT PacketStreamSource
{
    std::string     driver;
    int             id;
    std::string     uri;
    json::value     info;
    json::value     meta;
    int64_t         version;
    int64_t         data_alignment_bytes;
    std::string     data_definitions;
    int64_t         data_size_bytes;
};

typedef unsigned int PacketStreamSourceId;

PANGOLIN_EXPORT
int64_t PlaybackTime_us();

PANGOLIN_EXPORT
void SetCurrentPlaybackTime_us(int64_t time_us = 0);

class PANGOLIN_EXPORT PacketStreamWriter
{
public:
    PacketStreamWriter();
    PacketStreamWriter(const std::string& filename, unsigned int buffer_size_bytes = 10000000);
    void Open(const std::string& filename, unsigned int buffer_size_bytes = 10000000);


    ~PacketStreamWriter();

    PacketStreamSourceId AddSource(
        const std::string& source_driver,
        const std::string& source_uri,
        const json::value& json_header = json::value(),
        const size_t       packet_size_bytes = 0,
        const std::string& packet_definitions = ""
    );

    void WriteSourcePacketMeta(PacketStreamSourceId src, const json::value& json);

    void WriteSourcePacket(PacketStreamSourceId src, const char* data, size_t n);

    void WritePangoHeader();

    void WriteStats();

    void WriteSync();

protected:
    inline void WriteCompressedUnsignedInt(size_t n)
    {
        while(n >= 0x80) {
            writer.put( 0x80 | (n & 0x7F) );
            n >>= 7;
        }
        writer.put( (unsigned char)n );
    }

    inline void WriteTimestamp()
    {
        const int64_t time_us = PlaybackTime_us();
        writer.write((char*)&time_us, sizeof(int64_t));
    }

    inline void WriteTag(const uint32_t tag)
    {
        writer.write((char*)&tag, TAG_LENGTH);
    }

    std::vector<PacketStreamSource> sources;
    threadedfilebuf buffer;
    std::ostream writer;

    unsigned int bytes_written;
};

class PANGOLIN_EXPORT PacketStreamReader
{
public:
    ~PacketStreamReader();
    PacketStreamReader();
    PacketStreamReader(const std::string& filename, bool realtime = true);

    void Open(const std::string& filename, bool realtime = true);
    void Close();

    inline const std::vector<PacketStreamSource>& Sources() const
    {
        return sources;
    }

    bool ReadToSourcePacketAndLock(PacketStreamSourceId src_id);

    void ReleaseSourcePacketLock(PacketStreamSourceId src_id);

    // Should only read once lock is aquired
    inline std::basic_istream<char>& Read(char* s, size_t n)
    {
        return reader.read(s,n);
    }

protected:
    inline int64_t ReadTimestamp()
    {
        int64_t time_us;
        reader.read((char*)&time_us, sizeof(int64_t));
        return time_us;
    }

    inline size_t ReadCompressedUnsignedInt()
    {
        size_t n = 0;
        size_t v = reader.get();
        while( v & 0x80 ) {
            n |= v & 0x7F;
            n <<= 7;
            v = reader.get();
        }
        return n|v;
    }

    void ProcessMessage();
    void ProcessMessagesUntilSourcePacket(int& nxt_src_id, int64_t &time_us);

    bool ReadTag();
    void ReadHeaderPacket();
    void ReadSourcePacketMeta(json::value &json);
    void ReadNewSourcePacket();
    void ReadStatsPacket();
    void ReadOverSourcePacket(PacketStreamSourceId src_id);
    uint32_t next_tag;

    std::vector<PacketStreamSource> sources;

    std::ifstream reader;
    boostd::mutex read_mutex;

    int packets;
    bool realtime;
};

}

#endif // PANGOLIN_PACKETSTREAM_H
