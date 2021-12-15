autoware_auto_cmake {#autoware-auto-cmake-design}
===========

This is the design document for the `autoware_auto_cmake` package.


# Purpose

Provide common CMake variables and functions to Autoware packages.

Those include:

- Setting the language standard
- Getting user-provided variables
- Providing functions to:
  + set compiler flags
  + turn off optimizations

# Design

## Usage

Add `autoware_auto_cmake` as a "build_depend" in the dependent packages.

### CMake variables {#cmake-config-variables}

|Name|Type|Descritpion|Default|
|----|----|-----------|-------|
|`DOWNLOAD_ARTIFACTS`|*BOOL*|Allow downloading artifacts at build time.|`OFF`|
