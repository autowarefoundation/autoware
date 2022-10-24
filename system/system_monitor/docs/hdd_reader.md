# hdd_reader

## Name

hdd_reader - Read S.M.A.R.T. information for monitoring HDD temperature and life of HDD

## Synopsis

hdd_reader [OPTION]

## Description

Read S.M.A.R.T. information for monitoring HDD temperature and life of HDD.<br>
This runs as a daemon process and listens to a TCP/IP port (7635 by default).

**Options:**<br>
_-h, --help_<br>
&nbsp;&nbsp;&nbsp;&nbsp;Display help<br>
_-p, --port #_<br>
&nbsp;&nbsp;&nbsp;&nbsp;Port number to listen to

**Exit status:**<br>
Returns 0 if OK; non-zero otherwise.

## Notes

The 'hdd_reader' accesses minimal data enough to get Model number, Serial number, HDD temperature, and life of HDD.<br>
This is an approach to limit its functionality, however, the functionality can be expanded for further improvements and considerations in the future.<br><br>

### [ATA]

| Purpose                      | Name                 | Length               |
| ---------------------------- | -------------------- | -------------------- |
| Model number, Serial number  | IDENTIFY DEVICE data | 256 words(512 bytes) |
| HDD temperature, life of HDD | SMART READ DATA      | 256 words(512 bytes) |

For details please see the documents below.<br>

- [ATA Command Set - 4 (ACS-4)](https://www.t13.org/system/files/Project%20Drafts/2017/di529r20-ATA/ATAPI%20Command%20Set%20-%204_2.pdf)
- [ATA/ATAPI Command Set - 3 (ACS-3)](https://www.t13.org/system/files/Standards/2013/d2161r5-ATA/ATAPI%20Command%20Set%20-%203.pdf)
- [SMART Attribute Overview](https://www.t13.org/system/files/Documents/2005/e05171r0-SMART%20Attributes%20Overview_1.pdf)
- [SMART Attribute Annex](https://www.t13.org/system/files/Documents/2005/e05148r0-ACS-SMART%20Attributes%20Annex_1.pdf)

### [NVMe]

| Purpose                      | Name                               | Length              |
| ---------------------------- | ---------------------------------- | ------------------- |
| Model number, Serial number  | Identify Controller data structure | 4096 bytes          |
| HDD temperature, life of HDD | SMART / Health Information         | 36 Dword(144 bytes) |

For details please see the documents below.<br>

- [NVM Express 1.2b](https://www.nvmexpress.org/wp-content/uploads/NVM_Express_1_2b_Gold_20160603.pdf)

## Operation confirmed drives

<!-- cspell: ignore MZVLB1T0HALR -->

- SAMSUNG MZVLB1T0HALR (SSD)
- Western Digital My Passport (Portable HDD)
