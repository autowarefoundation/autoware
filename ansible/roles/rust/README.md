# rust

This role installs the Rust toolchain (rustc and cargo) via apt for building Autoware components that depend on Rust (e.g. acados and tera_renderer). Rustup is not installed; the system compiler and cargo from the distribution are used.

## Tools

- rustc (apt)
- cargo (apt)

## Inputs

| Name   | Required | Description                              |
| ------ | -------- | ---------------------------------------- |
| (none) | -        | This role has no configurable variables. |

## Manual Installation

```bash
sudo apt-get update
sudo apt-get install -y rustc cargo build-essential

# Verify
cargo --version
rustc --version
```

For a user-managed toolchain (multiple Rust versions, rustup), see [Rust installation documentation](https://www.rust-lang.org/tools/install).

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
