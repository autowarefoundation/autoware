# pre_commit

This role installs dependent tools for [pre-commit](https://pre-commit.com/).

## Inputs

| Name                 | Required | Description                 |
| -------------------- | -------- | --------------------------- |
| clang_format_version | false    | The version of ClangFormat. |

## Manual Installation

The `clang_format_version` variable can also be found in:
[./defaults/main.yaml](./defaults/main.yaml)

```bash
clang_format_version=14.0.6
pip3 install pre-commit clang-format==${clang_format_version}

# Install Golang (Add Go PPA for shfmt)
sudo add-apt-repository ppa:longsleep/golang-backports
sudo apt install golang
```
