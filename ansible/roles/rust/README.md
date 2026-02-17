# rust

This role installs the Rust toolchain (rustup, cargo, rustc) for building Autoware components that depend on Rust (e.g. acados tera_renderer).

## Tools

- rustup
- cargo
- rustc

## Inputs

| Name   | Required | Description                              |
| ------ | -------- | ---------------------------------------- |
| (none) | -        | This role has no configurable variables. |

## Manual Installation

```bash
# Install dependencies
sudo apt-get update
sudo apt-get install -y curl build-essential

# Download and run the Rust installer
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y

# Add Cargo to PATH (or restart your shell)
. "$HOME/.cargo/env"

# Verify
cargo --version
```

To make the environment persistent, add the following to your `~/.bashrc`:

```bash
. "$HOME/.cargo/env"
```

See [Rust installation documentation](https://www.rust-lang.org/tools/install) for more options.
