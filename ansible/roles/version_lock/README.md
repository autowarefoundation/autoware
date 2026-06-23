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
ROS_DISTRO=<distro> ROS_SNAPSHOT_DATE=<YYYY-MM-DD> ./ansible/scripts/generate_ansible_lockfile.sh
```

`ROS_SNAPSHOT_DATE` must match the snapshot the machine was provisioned from.

The script reads the package names from the existing lockfile for the current distro/architecture and fills in the versions currently installed on the machine.

## Validating lockfiles

```bash
./ansible/scripts/validate_lockfiles.sh
```

This checks that every `ansible/vars/locked-versions-*.yaml` file is valid YAML with a `ros_snapshot_date` and `apt_pins`/`pip_pins`/`ros_overrides` mappings.

## Lockfile format

A YAML mapping with four keys, one file per distro/arch
(`ansible/vars/locked-versions-<rosdistro>-<arch>.yaml`):

```yaml
ros_snapshot_date: "2026-04-13" # a real published date under snapshots.ros.org/<distro>/
apt_pins: # Ubuntu-archive origin only, sorted by name; rendered as APT pins
  ccache: 4.9.1-1
  git-lfs: 3.4.1-1ubuntu0.4
pip_pins: # pip/pipx origin; consumed by roles (e.g. gdown), NOT rendered as APT pins
  gdown: 6.1.0
ros_overrides: {} # exception pins for individual ROS packages; normally empty
```

- `ros_snapshot_date` drives the dated `snapshots.ros.org` source configured by the `ros2` role
  in locked mode, which freezes the **entire** ROS dependency closure to that date.
- `apt_pins` covers Ubuntu-archive packages. It is rendered into `/etc/apt/preferences.d/autoware-lock`
  with `Pin-Priority: 1001`.
- `pip_pins` covers pip/pipx-managed packages (e.g. `gdown`). It is consumed directly by the
  relevant roles and is **not** rendered as APT pins.
- `ros_overrides` is rendered into `/etc/apt/preferences.d/autoware-lock` with `Pin-Priority: 1001`.
- ROS-repo packages (`ros-*`, `python3-colcon-*`, `python3-rosdep`, `python3-vcs2l`,
  `python3-bloom`) are **not** listed in `apt_pins`: the snapshot date freezes them. To move a
  single ROS package ahead of the snapshot, add a one-line entry under `ros_overrides`.

### Overriding a single ROS package

`ros_overrides` is normally `{}` — the `ros_snapshot_date` freezes the whole ROS closure, so no
per-package ROS pins are needed. Use it only to move **one** ROS package ahead of the snapshot
(for example, to pick up a security or bug fix) without advancing `ros_snapshot_date` for everything
else. Each entry is `package: version`, where `version` is the exact APT version string:

```yaml
ros_snapshot_date: "2026-04-13"
apt_pins:
  ccache: 4.9.1-1
pip_pins:
  gdown: 6.1.0
ros_overrides:
  ros-jazzy-rmw-cyclonedds-cpp: 2.2.4-1noble.20260520.083000
```

The override above is rendered into `/etc/apt/preferences.d/autoware-lock` exactly like an `apt_pins`
entry:

```text
Package: ros-jazzy-rmw-cyclonedds-cpp
Pin: version 2.2.4-1noble.20260520.083000
Pin-Priority: 1001
```

The pinned version must be reachable from a configured APT source. The frozen snapshot only serves
its own date's builds, so an override to a **newer** build usually means pointing `ros_snapshot_date`
at a later snapshot that contains it (or adding a secondary source for that one package). Keep the
override list as small as possible and drop entries once the snapshot date catches up.
