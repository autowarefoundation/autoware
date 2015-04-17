#include <cstring>
#include <arpa/inet.h>

#include <obj_db.h>

// magic that I am C++
static constexpr const char *MAGIC = "MPWC";
static constexpr int MAJOR_VERSION = 1;
static constexpr int MINOR_VERSION = 0;

std::string make_header(int32_t sql_inst, int32_t sql_num)
{
  int major = htons(MAJOR_VERSION);
  int minor = htons(MINOR_VERSION);
  sql_inst = htonl(sql_inst);
  sql_num = htonl(sql_num);

  char header[16];
  std::memcpy(header, MAGIC, 4);
  std::memcpy(header + 4, &major, sizeof(int16_t));
  std::memcpy(header + 6, &minor, sizeof(int16_t));
  std::memcpy(header + 8, &sql_inst, sizeof(int32_t));
  std::memcpy(header + 12, &sql_num, sizeof(int32_t));

  return std::string(static_cast<const char*>(header));
}
