# Build Tools

This role installs build tools for building Autoware.

## Tools

- ccache

## Inputs

## Manual Installation

```bash
# Update package lists
sudo apt-get update

# Install ccache
sudo apt-get install -y ccache

# Add ccache to PATH
echo 'export PATH="/usr/lib/ccache/:$PATH"' >> ~/.bashrc
```
