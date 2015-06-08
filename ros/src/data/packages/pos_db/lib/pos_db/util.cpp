#include <stdio.h>
#include <cstring>
#include <arpa/inet.h>

#include <pos_db.h>

// magic that I am C++
static constexpr const char *MAGIC = "MPWC";
static constexpr int MAJOR_VERSION = 1;
static constexpr int MINOR_VERSION = 0;

std::string make_header(int32_t sql_inst, int32_t sql_num)
{
  int16_t major = htons(MAJOR_VERSION);
  int16_t minor = htons(MINOR_VERSION);
  sql_inst = htonl(sql_inst);
  sql_num = htonl(sql_num);

  char header[32];
  int len = strlen(MAGIC);

  std::memcpy(header, MAGIC, len);
  std::memcpy(header + len, &major, sizeof(int16_t));
  len += sizeof(int16_t);
  std::memcpy(header + len, &minor, sizeof(int16_t));
  len += sizeof(int16_t);
  std::memcpy(header + len, &sql_inst, sizeof(int32_t));
  len += sizeof(int32_t);
  std::memcpy(header + len, &sql_num, sizeof(int32_t));
  len += sizeof(int32_t);
  header[len] = '\0';

  for (int i=0; i<len; i++)
    header[i] |= 0x80; // TODO: see SendData::Sender()

#ifdef DEBUG_POS_DB
  std::string str = std::string(static_cast<const char*>(header));
  const char *cstr = str.c_str();
  for (int i=0; i<16; i++)
    printf("%02x%c", cstr[i]&0xff, (i==15) ? '\n':' ');
#endif /* DEBUG_POS_DB */

  return std::string(static_cast<const char*>(header));
}
