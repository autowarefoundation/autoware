# autoware_universe

This role installs development/runtime dependencies for Autoware Universe.

## Inputs

None.

## Manual Installation

```bash
sudo apt install geographiclib-tools

# Add EGM2008 geoid grid to geographiclib
sudo geographiclib-get-geoids egm2008-1
```
