# pre_commit

This role installs dependent tools for [pre-commit](https://pre-commit.com/).

## Inputs

| Name                 | Required | Description                 |
| -------------------- | -------- | --------------------------- |
| pre_commit_clang_format_version | false    | The version of ClangFormat. |

## Manual Installation

The `pre_commit_clang_format_version` variable can also be found in:
[./defaults/main.yaml](./defaults/main.yaml)

```bash
pre_commit_clang_format_version=17.0.4
pip3 install pre-commit clang-format==${pre_commit_clang_format_version}

sudo apt install golang
```
