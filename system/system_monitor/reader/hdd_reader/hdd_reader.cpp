// Copyright 2020 Autoware Foundation
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

/**
 * @file hdd_reader.cpp
 * @brief HDD information read class
 */

#include <hdd_reader/hdd_reader.hpp>

#include <boost/algorithm/string.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/process.hpp>
#include <boost/serialization/vector.hpp>

#include <fcntl.h>
#include <fmt/format.h>
#include <getopt.h>
#include <linux/nvme_ioctl.h>
#include <netinet/in.h>
#include <scsi/sg.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <syslog.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <regex>
#include <string>
#include <utility>
#include <vector>

// 7634-7647 Unassigned
constexpr int PORT = 7635;

/**
 * @brief ATA PASS-THROUGH (12) command
 * @note For details please see the document below.
 * - ATA Command Pass-Through
 *   https://www.t10.org/ftp/t10/document.04/04-262r8.pdf
 */
struct AtaPassThrough12
{
  uint8_t operation_code_;      //!< @brief OPERATION CODE (A1h)
  uint8_t reserved0_ : 1;       //!< @brief Reserved
  uint8_t protocol_ : 4;        //!< @brief PROTOCOL
  uint8_t multiple_count_ : 3;  //!< @brief MULTIPLE_COUNT
  uint8_t t_length_ : 2;        //!< @brief T_LENGTH
  uint8_t byt_blok_ : 1;        //!< @brief BYT_BLOK
  uint8_t t_dir_ : 1;           //!< @brief T_DIR
  uint8_t reserved1_ : 1;       //!< @brief Reserved
  uint8_t ck_cond_ : 1;         //!< @brief CK_COND
  uint8_t off_line_ : 2;        //!< @brief OFF_LINE
  uint8_t features_;            //!< @brief FEATURES (0:7)
  uint8_t sector_count_;        //!< @brief SECTOR_COUNT (0:7)
  uint8_t lba_low_;             //!< @brief LBA_LOW (0:7)
  uint8_t lba_mid_;             //!< @brief LBA_MID (0:7)
  uint8_t lbe_high_;            //!< @brief LBE_HIGH (0:7)
  uint8_t device_;              //!< @brief DEVICE
  uint8_t command_;             //!< @brief COMMAND
  uint8_t reserved2_;           //!< @brief Reserved
  uint8_t control_;             //!< @brief CONTROL
};

/**
 * @brief Attribute Table Format
 * @note For details please see the documents below.
 * - SMART Attribute Overview
 *   http://www.t13.org/Documents/UploadedDocuments/docs2005/e05171r0-ACS-SMARTAttributes_Overview.pdf
 */
struct AttributeEntry
{
  uint8_t attribute_id_;  //!< @brief Attribute ID
  //  Flags
  uint16_t warranty_ : 1;           //!< @brief Bit 0 - Warranty
  uint16_t offline_ : 1;            //!< @brief Bit 1 - Offline
  uint16_t performance_ : 1;        //!< @brief Bit 2 - Performance
  uint16_t error_rate_ : 1;         //!< @brief Bit 3 - Error rate
  uint16_t event_count_ : 1;        //!< @brief Bit 4 - Event count
  uint16_t self_preservation_ : 1;  //!< @brief Bit 5 - Self-preservation
  uint16_t reserved_ : 10;          //!< @brief Bits 6-15 - Reserved

  uint8_t current_value_;        //!< @brief Current value
  uint8_t worst_value_;          //!< @brief Worst value
  uint32_t data_;                //!< @brief Data
  uint16_t attribute_specific_;  //!< @brief Attribute-specific
  uint8_t threshold_;            //!< @brief Threshold
} __attribute__((packed));       // Minimize total struct memory 16 to 12

/**
 * @brief Device SMART data structure
 * @note For details please see the documents below.
 * - ATA/ATAPI Command Set - 3 (ACS-3)
 *   http://www.t13.org/Documents/UploadedDocuments/docs2013/d2161r5-ATAATAPI_Command_Set_-_3.pdf
 * - SMART Attribute Overview
 *   http://www.t13.org/Documents/UploadedDocuments/docs2005/e05171r0-ACS-SMARTAttributes_Overview.pdf
 */
struct SmartData
{
  // Offset 0..361 X Vendor specific
  uint16_t smart_structure_version_;    //!< @brief SMART structure version
  AttributeEntry attribute_entry_[30];  //!< @brief Attribute entry 1 - 30
  // Offset 362 to 511
  uint8_t off_line_data_collection_status_;      //!< @brief Off-line data collection status
  uint8_t self_test_execution_status_byte_;      //!< @brief Self-test execution status byte
  uint16_t vendor_specific0_;                    //!< @brief Vendor specific
  uint8_t vendor_specific1_;                     //!< @brief Vendor specific
  uint8_t off_line_data_collection_capability_;  //!< @brief Off-line data collection capability
  uint16_t smart_capability_;                    //!< @brief SMART capability
  uint8_t error_logging_capability_;             //!< @brief Error logging capability
  uint8_t vendor_specific2_;                     //!< @brief Vendor specific
  uint8_t short_self_test_polling_time_;     //!< @brief Short self-test polling time (in minutes)
  uint8_t extended_self_test_polling_time_;  //!< @brief Extended self-test polling time in minutes
  uint8_t
    conveyance_self_test_polling_time_;     //!< @brief Conveyance self-test polling time in minutes
  uint16_t                                  //!< @brief Extended self-test polling time
    extended_self_test_polling_time_word_;  //!<   in minutes (word)
  uint8_t reserved_[9];                     //!< @brief Reserved
  uint8_t vendor_specific3_[125];           //!< @brief Vendor specific
  uint8_t data_structure_checksum_;         //!< @brief Data structure checksum
} __attribute__((packed));                  // Minimize total struct memory 514 to 512

/**
 * @brief print usage
 */
void usage()
{
  printf("Usage: hdd_reader [options]\n");
  printf("  -h --help   : Display help\n");
  printf("  -p --port # : Port number to listen to.\n");
  printf("\n");
}

/**
 * @brief exchanges the values of 2 bytes
 * @param [inout] str a string reference to ATA string
 * @param [in] size size of ATA string
 * @note Each pair of bytes in an ATA string is swapped.
 * FIRMWARE REVISION field example
 * Word Value
 * 23   6162h ("ba")
 * 24   6364h ("dc")
 * 25   6566h ("fe")
 * 26   6720h (" g")
 * -> "abcdefg "
 */
void swap_char(std::string & str, size_t size)
{
  for (auto i = 0U; i < size; i += 2U) {
    std::swap(str[i], str[i + 1]);
  }
}

/**
 * @brief get IDENTIFY DEVICE for ATA drive
 * @param [in] fd file descriptor to device
 * @param [out] info a pointer to HDD information
 * @return 0 on success, otherwise error
 * @note For details please see the documents below.
 * - ATA Command Pass-Through
 *   https://www.t10.org/ftp/t10/document.04/04-262r8.pdf
 * - ATA Command Set - 4  (ACS-4)
 *   http://www.t13.org/Documents/UploadedDocuments/docs2016/di529r14-ATAATAPI_Command_Set_-_4.pdf
 */
int get_ata_identify(int fd, HddInfo * info)
{
  sg_io_hdr_t hdr{};
  AtaPassThrough12 ata{};
  unsigned char data[512]{};  // 256 words

  // Create a command descriptor block(CDB)
  ata.operation_code_ = 0xA1;  // ATA PASS-THROUGH (12) command
  ata.protocol_ = 0x4;         // PIO Data-In
  ata.t_dir_ = 0x1;            // from the ATA device to the application client
  ata.byt_blok_ = 0x1;         // the number of blocks specified in the T_LENGTH field
  ata.t_length_ = 0x2;         // length is specified in the SECTOR_COUNT field
  ata.sector_count_ = 0x01;    // 1 sector
  ata.command_ = 0xEC;         // IDENTIFY DEVICE

  // Create a control structure
  hdr.interface_id = 'S';                   // This must be set to 'S'
  hdr.dxfer_direction = SG_DXFER_FROM_DEV;  // a SCSI READ command
  hdr.cmd_len = sizeof(ata);         // length in bytes of the SCSI command that 'cmdp' points to
  hdr.cmdp = (unsigned char *)&ata;  // SCSI command to be executed
  hdr.dxfer_len = sizeof(data);      // number of bytes to be moved in the data transfer
  hdr.dxferp = data;                 // a pointer to user memory
  hdr.timeout = 1000;                // 1 second

  // send SCSI command to device
  if (ioctl(fd, SG_IO, &hdr) < 0) {
    return errno;
  }

  // IDENTIFY DEVICE
  // Word 10..19 Serial number
  info->serial_ = std::string(reinterpret_cast<char *>(data) + 20, 20);
  swap_char(info->serial_, 20);
  boost::trim(info->serial_);

  // Word 27..46 Model number
  info->model_ = std::string(reinterpret_cast<char *>(data) + 54, 40);
  swap_char(info->model_, 40);
  boost::trim(info->model_);

  return EXIT_SUCCESS;
}

/**
 * @brief get SMART DATA for ATA drive
 * @param [in] fd file descriptor to device
 * @param [out] info a pointer to HDD information
 * @param [in] device a reference to HDD device
 * @return 0 on success, otherwise error
 * @note For details please see the documents below.
 * - ATA Command Pass-Through
 *   https://www.t10.org/ftp/t10/document.04/04-262r8.pdf
 * - ATA/ATAPI Command Set - 3 (ACS-3)
 *   http://www.t13.org/Documents/UploadedDocuments/docs2013/d2161r5-ATAATAPI_Command_Set_-_3.pdf
 * - SMART Attribute Overview
 *   http://www.t13.org/Documents/UploadedDocuments/docs2005/e05171r0-ACS-SMARTAttributes_Overview.pdf
 * - SMART Attribute Annex
 *   http://www.t13.org/documents/uploadeddocuments/docs2005/e05148r0-acs-smartattributesannex.pdf
 */
int get_ata_smart_data(int fd, HddInfo * info, const HddDevice & device)
{
  sg_io_hdr_t hdr{};
  AtaPassThrough12 ata{};
  SmartData data{};

  // Create a command descriptor block(CDB)
  ata.operation_code_ = 0xA1;  // ATA PASS-THROUGH (12) command
  ata.protocol_ = 0x4;         // PIO Data-In
  ata.t_dir_ = 0x1;            // from the ATA device to the application client
  ata.byt_blok_ = 0x1;         // the number of blocks specified in the T_LENGTH field
  ata.t_length_ = 0x2;         // length is specified in the SECTOR_COUNT field
  ata.features_ = 0xD0;        // SMART READ DATA
  ata.sector_count_ = 0x01;    // 1 sector
  ata.lba_mid_ = 0x4F;         // Fixed
  ata.lbe_high_ = 0xC2;        // Fixed
  ata.command_ = 0xB0;         // SMART READ DATA

  // Create a control structure
  hdr.interface_id = 'S';                   // This must be set to 'S'
  hdr.dxfer_direction = SG_DXFER_FROM_DEV;  // a SCSI READ command
  hdr.cmd_len = sizeof(ata);         // length in bytes of the SCSI command that 'cmdp' points to
  hdr.cmdp = (unsigned char *)&ata;  // SCSI command to be executed
  hdr.dxfer_len = sizeof(data);      // number of bytes to be moved in the data transfer
  hdr.dxferp = &data;                // a pointer to user memory
  hdr.timeout = 1000;                // 1 second

  // send SCSI command to device
  if (ioctl(fd, SG_IO, &hdr) < 0) {
    return errno;
  }

  info->is_valid_temp_ = false;
  info->is_valid_power_on_hours_ = false;
  info->is_valid_total_data_written_ = false;
  info->is_valid_recovered_error_ = false;
  // Retrieve S.M.A.R.T. Informations
  for (int i = 0; i < 30; ++i) {
    if (data.attribute_entry_[i].attribute_id_ == device.temp_attribute_id_) {  // Temperature -
                                                                                // Device Internal
      info->temp_ = static_cast<uint8_t>(data.attribute_entry_[i].data_);
      info->is_valid_temp_ = true;
    } else if (
      data.attribute_entry_[i].attribute_id_ ==
      device.power_on_hours_attribute_id_) {  // Power-on Hours Count
      info->power_on_hours_ = data.attribute_entry_[i].data_;
      info->is_valid_power_on_hours_ = true;
    } else if (
      data.attribute_entry_[i].attribute_id_ ==
      device.total_data_written_attribute_id_) {  // Total LBAs Written
      info->total_data_written_ =
        (data.attribute_entry_[i].data_ |
         (static_cast<uint64_t>(data.attribute_entry_[i].attribute_specific_) << 32));
      info->is_valid_total_data_written_ = true;
    } else if (
      data.attribute_entry_[i].attribute_id_ ==
      device.recovered_error_attribute_id_) {  // Hardware ECC Recovered
      info->recovered_error_ = data.attribute_entry_[i].data_;
      info->is_valid_recovered_error_ = true;
    }
  }

  return EXIT_SUCCESS;
}

/**
 * @brief get Identify for NVMe drive
 * @param [in] fd file descriptor to device
 * @param [out] info a pointer to HDD information
 * @return 0 on success, otherwise error
 * @note For details please see the document below.
 * - NVM Express 1.2b
 *   https://www.nvmexpress.org/wp-content/uploads/NVM_Express_1_2b_Gold_20160603.pdf
 */
int get_nvme_identify(int fd, HddInfo * info)
{
  nvme_admin_cmd cmd{};
  char data[4096]{};  // Fixed size for Identify command

  // The Identify command returns a data buffer that describes information about the NVM subsystem
  cmd.opcode = 0x06;            // Identify
  cmd.addr = (uint64_t)data;    // memory address of data
  cmd.data_len = sizeof(data);  // length of data
  cmd.cdw10 = 0x01;             // Identify Controller data structure

  // send Admin Command to device
  int ret = ioctl(fd, NVME_IOCTL_ADMIN_CMD, &cmd);
  if (ret < 0) {
    return errno;
  }

  // Identify Controller Data Structure
  // Bytes 23:04 Serial Number (SN)
  info->serial_ = std::string(data + 4, 20);
  boost::trim(info->serial_);

  // Bytes 63:24 Model Number (MN)
  info->model_ = std::string(data + 24, 40);
  boost::trim(info->model_);

  return EXIT_SUCCESS;
}

/**
 * @brief get SMART / Health Information for NVMe drive
 * @param [in] fd file descriptor to device
 * @param [inout] info a pointer to HDD information
 * @return 0 on success, otherwise error
 * @note For details please see the document below.
 * - NVM Express 1.2b
 *   https://www.nvmexpress.org/wp-content/uploads/NVM_Express_1_2b_Gold_20160603.pdf
 */
int get_nvme_smart_data(int fd, HddInfo * info)
{
  nvme_admin_cmd cmd{};
  unsigned char data[144]{};  // 36 Dword (get byte 0 to 143)

  // The Get Log Page command returns a data buffer containing the log page requested
  cmd.opcode = 0x02;            // Get Log Page
  cmd.nsid = 0xFFFFFFFF;        // Global log page
  cmd.addr = (uint64_t)data;    // memory address of data
  cmd.data_len = sizeof(data);  // length of data
  cmd.cdw10 = 0x00240002;       // Bit 27:16 Number of Dwords (NUMD) = 024h (36 Dword)
                                // - Minimum necessary size to obtain S.M.A.R.T. informations
                                // Bit 07:00 = 02h (SMART / Health Information)

  // send Admin Command to device
  int ret = ioctl(fd, NVME_IOCTL_ADMIN_CMD, &cmd);
  if (ret < 0) {
    return errno;
  }

  // Bytes 2:1 Composite Temperature
  // Convert kelvin to celsius
  unsigned int temperature = ((data[2] << 8) | data[1]) - 273;
  info->is_valid_temp_ = true;
  info->temp_ = static_cast<uint8_t>(temperature);

  // Bytes 63:48 Data Units Written
  // This value is reported in thousands
  // (i.e., a value of 1 corresponds to 1,000 units of 512 bytes written)
  // and is rounded up
  // (e.g., one indicates that the number of 512 byte data units written
  // is from 1 to 1,000, three indicates that the number of 512 byte data
  // units written is from 2,001 to 3,000)
  info->is_valid_total_data_written_ = true;
  info->total_data_written_ = *(reinterpret_cast<uint64_t *>(&data[48]));

  // Bytes 143:128 Power On Hours
  info->is_valid_power_on_hours_ = true;
  info->power_on_hours_ = *(reinterpret_cast<uint64_t *>(&data[128]));

  // NVMe S.M.A.R.T has no information of recovered error count
  info->is_valid_recovered_error_ = false;

  return EXIT_SUCCESS;
}

/**
 * @brief get HDD information
 * @param [in] boost::archive::text_iarchive object
 * @param [out] boost::archive::text_oarchive object
 * @return 0 on success, otherwise error
 */
int get_hdd_info(boost::archive::text_iarchive & ia, boost::archive::text_oarchive & oa)
{
  std::vector<HddDevice> hdd_devices;
  HddInfoList list;

  try {
    ia & hdd_devices;
  } catch (const std::exception & e) {
    syslog(LOG_ERR, "exception. %s\n", e.what());
    return -1;
  }

  for (auto & hdd_device : hdd_devices) {
    HddInfo info{};

    // Open a file
    int fd = open(hdd_device.name_.c_str(), O_RDONLY);
    if (fd < 0) {
      info.error_code_ = errno;
      syslog(LOG_ERR, "Failed to open a file. %s\n", strerror(info.error_code_));
      continue;
    }

    // AHCI device
    if (boost::starts_with(hdd_device.name_.c_str(), "/dev/sd")) {
      // Get IDENTIFY DEVICE for ATA drive
      info.error_code_ = get_ata_identify(fd, &info);
      if (info.error_code_ != 0) {
        syslog(
          LOG_ERR, "Failed to get IDENTIFY DEVICE for ATA drive. %s\n", strerror(info.error_code_));
        close(fd);
        continue;
      }
      // Get SMART DATA for ATA drive
      info.error_code_ = get_ata_smart_data(fd, &info, hdd_device);
      if (info.error_code_ != 0) {
        syslog(LOG_ERR, "Failed to get SMART LOG for ATA drive. %s\n", strerror(info.error_code_));
        close(fd);
        continue;
      }
    } else if (boost::starts_with(hdd_device.name_.c_str(), "/dev/nvme")) {  // NVMe device
      // Get Identify for NVMe drive
      info.error_code_ = get_nvme_identify(fd, &info);
      if (info.error_code_ != 0) {
        syslog(LOG_ERR, "Failed to get Identify for NVMe drive. %s\n", strerror(info.error_code_));
        close(fd);
        continue;
      }
      // Get SMART / Health Information for NVMe drive
      info.error_code_ = get_nvme_smart_data(fd, &info);
      if (info.error_code_ != 0) {
        syslog(
          LOG_ERR, "Failed to get SMART / Health Information for NVMe drive. %s\n",
          strerror(info.error_code_));
        close(fd);
        continue;
      }
    }

    // Close the file descriptor FD
    info.error_code_ = close(fd);
    if (info.error_code_ < 0) {
      info.error_code_ = errno;
      syslog(LOG_ERR, "Failed to close the file descriptor FD. %s\n", strerror(info.error_code_));
    }

    list[hdd_device.name_] = info;
  }

  oa << list;
  return 0;
}

/**
 * @brief unmount device with lazy option
 * @param [in] boost::archive::text_iarchive object
 * @param [out] boost::archive::text_oarchive object
 * @return 0 on success, otherwise error
 */
int unmount_device_with_lazy(boost::archive::text_iarchive & ia, boost::archive::text_oarchive & oa)
{
  std::vector<UnmountDeviceInfo> unmount_devices;
  std::vector<int> responses;

  try {
    ia & unmount_devices;
  } catch (const std::exception & e) {
    syslog(LOG_ERR, "exception. %s\n", e.what());
    return -1;
  }

  for (auto & unmount_device : unmount_devices) {
    int ret = 0;
    boost::process::ipstream is_out;
    boost::process::ipstream is_err;

    boost::process::child c(
      "/bin/sh", "-c", fmt::format("umount -l {}", unmount_device.part_device_.c_str()),
      boost::process::std_out > is_out, boost::process::std_err > is_err);
    c.wait();

    if (c.exit_code() != 0) {
      syslog(
        LOG_ERR, "Failed to execute umount command. %s\n", unmount_device.part_device_.c_str());
      ret = -1;
    }
    responses.push_back(ret);
  }

  oa << responses;
  return 0;
}

/**
 * @brief hdd_reader main procedure
 * @param [in] port port to listen
 */
void run(int port)
{
  // Create a new socket
  int sock = socket(AF_INET, SOCK_STREAM, 0);
  if (sock < 0) {
    syslog(LOG_ERR, "Failed to create a new socket. %s\n", strerror(errno));
    return;
  }

  // Allow address reuse
  int ret = 0;
  int opt = 1;
  ret = setsockopt(
    sock, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<char *>(&opt), (socklen_t)sizeof(opt));
  if (ret < 0) {
    syslog(LOG_ERR, "Failed to set socket FD's option. %s\n", strerror(errno));
    close(sock);
    return;
  }

  // Give the socket FD the local address ADDR
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(port);
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  ret = bind(sock, (struct sockaddr *)&addr, sizeof(addr));
  if (ret < 0) {
    syslog(LOG_ERR, "Failed to give the socket FD the local address ADDR. %s\n", strerror(errno));
    close(sock);
    return;
  }

  // Prepare to accept connections on socket FD
  ret = listen(sock, 5);
  if (ret < 0) {
    syslog(LOG_ERR, "Failed to prepare to accept connections on socket FD. %s\n", strerror(errno));
    close(sock);
    return;
  }

  sockaddr_in client{};
  socklen_t len = sizeof(client);

  while (true) {
    // Await a connection on socket FD
    int new_sock = accept(sock, reinterpret_cast<sockaddr *>(&client), &len);
    if (new_sock < 0) {
      syslog(
        LOG_ERR, "Failed to prepare to accept connections on socket FD. %s\n", strerror(errno));
      close(sock);
      return;
    }

    // Receive list of device from a socket
    char buf[1024]{};
    ret = recv(new_sock, buf, sizeof(buf) - 1, 0);
    if (ret < 0) {
      syslog(LOG_ERR, "Failed to receive. %s\n", strerror(errno));
      close(new_sock);
      close(sock);
      return;
    }
    // No data received
    if (ret == 0) {
      syslog(LOG_ERR, "No data received. %s\n", strerror(errno));
      close(new_sock);
      close(sock);
      return;
    }

    uint8_t request_id;

    buf[sizeof(buf) - 1] = '\0';
    std::istringstream iss(buf);
    boost::archive::text_iarchive ia(iss);

    try {
      ia & request_id;
    } catch (const std::exception & e) {
      syslog(LOG_ERR, "exception. %s\n", e.what());
      close(new_sock);
      close(sock);
      return;
    }

    std::ostringstream oss;
    boost::archive::text_oarchive oa(oss);

    switch (request_id) {
      case HddReaderRequestId::GetHddInfo:
        ret = get_hdd_info(ia, oa);
        break;
      case HddReaderRequestId::UnmountDevice:
        ret = unmount_device_with_lazy(ia, oa);
        break;
      default:
        syslog(LOG_ERR, "Request ID is invalid. %d\n", request_id);
        continue;
    }
    if (ret != 0) {
      close(new_sock);
      close(sock);
      return;
    }

    // Write N bytes of BUF to FD
    ret = write(new_sock, oss.str().c_str(), oss.str().length());
    if (ret < 0) {
      syslog(LOG_ERR, "Failed to write N bytes of BUF to FD. %s\n", strerror(errno));
    }

    // Close the file descriptor FD
    ret = close(new_sock);
    if (ret < 0) {
      syslog(LOG_ERR, "Failed to close the file descriptor FD. %s\n", strerror(errno));
    }
  }

  close(sock);
}

int main(int argc, char ** argv)
{
  static struct option long_options[] = {
    {"help", no_argument, nullptr, 'h'},
    {"port", required_argument, nullptr, 'p'},
    {nullptr, 0, nullptr, 0}};

  // Parse command-line options
  int c = 0;
  int option_index = 0;
  int port = PORT;
  while ((c = getopt_long(argc, argv, "hp:", long_options, &option_index)) != -1) {
    switch (c) {
      case 'h':
        usage();
        return EXIT_SUCCESS;

      case 'p':
        try {
          port = boost::lexical_cast<int>(optarg);
        } catch (const boost::bad_lexical_cast & e) {
          printf("Error: %s\n", e.what());
          return EXIT_FAILURE;
        }
        break;

      default:
        break;
    }
  }

  // Put the program in the background
  if (daemon(0, 0) < 0) {
    printf("Failed to put the program in the background. %s\n", strerror(errno));
    return errno;
  }

  // Open connection to system logger
  openlog(nullptr, LOG_PID, LOG_DAEMON);

  run(port);

  // Close descriptor used to write to system logger
  closelog();

  return EXIT_SUCCESS;
}
