# rust

This role installs the Rust toolchain (rustc and cargo) via [rustup](https://rustup.rs/) for building Autoware components that depend on Rust (e.g. acados and tera_renderer).

## Tools

- rustc
- cargo

## Inputs

| Name   | Required | Description                              |
| ------ | -------- | ---------------------------------------- |
| (none) | -        | This role has no configurable variables. |

## Manual Installation

Reference: [Rust installation documentation](https://www.rust-lang.org/tools/install).

```bash
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs -o /tmp/rustup-init.sh
chmod +x /tmp/rustup-init.sh

/tmp/rustup-init.sh -y
```

Add the following line to your `~/.bashrc`.

```
. "$HOME/.cargo/env"
```

And source the file. (`source ~/.bashrc` or just call `bash` in the terminal).

```bash
# Verify
cargo --version
rustc --version
```

## Ansible Installation

Install ansible following the instructions in the [ansible installation guide](../../README.md#ansible-installation).

### Install ansible collections

```bash
cd ~/autoware # The root directory of the cloned repository
ansible-galaxy collection install -f -r "ansible-galaxy-requirements.yaml"
```

This step should be repeated when the ansible directory is updated.

### Run the playbook

```bash
ansible-playbook autoware.dev_env.install_rust --ask-become-pass
```
