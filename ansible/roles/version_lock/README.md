# version_lock

## Purpose

This role pins APT package versions for reproducible dependency installation. When enabled, it writes an APT preferences file (`/etc/apt/preferences.d/autoware-lock`) with `Pin-Priority: 1001` entries generated from a lockfile, forcing APT to install (or downgrade to) the exact locked versions. When disabled (the default), the role removes any existing pin file, so it is always safe to keep registered in the playbook. The lockfile covers the packages installed by the `install_dev_env` playbook; Docker Engine packages are out of scope because they are installed by the separate `install_docker` playbook.

## Usage

Version locking is opt-in. Enable it by setting `use_locked_versions=true`:

```bash
ansible-playbook autoware.dev_env.install_dev_env --extra-vars use_locked_versions=true --ask-become-pass
```

### Lockfile resolution

The lockfile path is resolved automatically from the ROS distro and CPU architecture as `ansible/vars/locked-versions-<rosdistro>-<arch>.yaml` (e.g. `locked-versions-humble-amd64.yaml`), where `<arch>` is the Debian architecture name mapped from `ansible_architecture` (`x86_64` -> `amd64`, `aarch64` -> `arm64`).

To use a custom lockfile, override `lockfile_path`:

```bash
ansible-playbook autoware.dev_env.install_dev_env --extra-vars "use_locked_versions=true lockfile_path=/path/to/my-lockfile.yaml" --ask-become-pass
```

## Generating lockfiles

On a machine already provisioned by `install_dev_env`, run:

```bash
ROS_DISTRO=<distro> ./ansible/scripts/generate_ansible_lockfile.sh
```

The script reads the package names from the existing lockfile for the current distro/architecture and fills in the versions currently installed on the machine.

## Validating lockfiles

```bash
./ansible/scripts/validate_lockfiles.sh
```

This checks that every `ansible/vars/locked-versions-*.yaml` file is valid YAML and a flat dictionary.

## Lockfile format

A flat YAML dictionary mapping `package: version`, sorted alphabetically by package name:

```yaml
ccache: 4.5.1-1
python3-pip: 22.0.2+dfsg-1ubuntu0.4
```
