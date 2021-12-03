# msr_reader

## Name

msr_reader - Read MSR register for monitoring thermal throttling event

## Synopsis

msr_reader [OPTION]

## Description

Read MSR register for monitoring thermal throttling event.<br>
This runs as a daemon process and listens to a TCP/IP port (7634 by default).

**Options:**<br>
_-h, --help_<br>
&nbsp;&nbsp;&nbsp;&nbsp;Display help<br>
_-p, --port #_<br>
&nbsp;&nbsp;&nbsp;&nbsp;Port number to listen to

**Exit status:**<br>
Returns 0 if OK; non-zero otherwise.

## Notes

The 'msr_reader' accesses minimal data enough to get thermal throttling event.<br>
This is an approach to limit its functionality, however, the functionality can be expanded for further improvements and considerations in the future.

| Register Address | Name                      | Length |
| ---------------- | ------------------------- | ------ |
| 1B1H             | IA32_PACKAGE_THERM_STATUS | 64bit  |

For details please see the documents below.<br>

- [Intel® 64 and IA-32 ArchitecturesSoftware Developer’s Manual](https://software.intel.com/sites/default/files/managed/39/c5/325462-sdm-vol-1-2abcd-3abcd.pdf)

## Operation confirmed platform

- PC system intel core i7
