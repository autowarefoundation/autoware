# Rust

This role sets up the Rust toolchain (rustup, cargo, rustc) for building Autoware components that depend on Rust (e.g. acados tera_renderer).

## Role in system design

Used to build packages that require Rust.

## Dependencies

- curl
- build-essential

## Usage

### Variables

None

### Preparation

Example of adding the role to a playbook:

```yaml
- { role: rust, tags: [rust] }
```

Or with the collection:

```yaml
- role: autoware.dev_env.rust
```

## Related links

<https://www.rust-lang.org/tools/install>

### Remarks
