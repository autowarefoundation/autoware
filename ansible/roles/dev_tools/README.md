# devel

This role installs optional development tools for Autoware.

## Tools

- Git LFS
- pre-commit
- clang-format
- Go

## Inputs

| Name          | Required | Description                             |
| ------------- | -------- | --------------------------------------- |
| clang-version | true     | The version of clang-format to install. |

## Manual Installation

```bash
#!/bin/bash

# Update package lists
sudo apt-get update

# Install Git LFS
sudo apt-get install -y git-lfs

# Setup Git LFS
git lfs install

# Install pre-commit using pip3
pip3 install pre-commit

# Install a specific version of clang-format using pip3
# Replace X.Y with the actual version you want to install
pip3 install clang-format==${pre_commit_clang_format_version}

# Install Go
sudo apt-get install -y golang
```
