# rust

<<<<<<< Updated upstream
This role installs the Rust toolchain (rustup, cargo, rustc) for building Autoware components that depend on Rust (e.g. acados and tera_renderer).
=======
This role installs the Rust toolchain (rustc and cargo) via apt for building Autoware components that depend on Rust (e.g. acados and tera_renderer). rustup is not installed; the system compiler and cargo from the distribution are used.
>>>>>>> Stashed changes

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
