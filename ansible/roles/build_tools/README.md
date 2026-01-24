# Build Tools

This role installs build tools for building Autoware.

## Tools

- ccache

## Inputs

## Manual Installation

```bash
sudo apt-get update
sudo apt-get install -y ccache
```

## Configuration (🆕 Recommended)

> ℹ️ These steps differ slightly from the existing Ansible setup and represent the preferred configuration.  
> The Ansible version will be updated to match the steps below.

```bash
# Make sure the ccache directory exists
mkdir -p "$HOME/.cache/ccache"

# Add the following lines to ~/.bashrc file
export CMAKE_C_COMPILER_LAUNCHER=ccache
export CMAKE_CXX_COMPILER_LAUNCHER=ccache
export CCACHE_DIR="$HOME/.cache/ccache/"
export CCACHE_LOGFILE=/tmp/ccache.log
```

Configure ccache maximum size:
`gedit $HOME/.cache/ccache/ccache.conf`

Add the following lines and save the file:

```bash
# Set maximum cache size
max_size = 15G
```

**Also see:** [🔗 Autoware Documentation / Using ccache to speed up compilation](https://autowarefoundation.github.io/autoware-documentation/main/tutorials/others/advanced-usage-of-colcon/#using-ccache-to-speed-up-recompilation).
