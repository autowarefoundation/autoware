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

#### Overriding only the NVIDIA closure

`(rosdistro, arch)` does not identify the CUDA/TensorRT closure. That closure is a function of `(cuda_repo_distro, cuda_version, tensorrt_version, arch)`, and the `cuda` and `tensorrt` roles let a consumer override all four — so `locked-versions-<rosdistro>-<arch>.yaml` can only carry the generation those roles default to for that distro (CUDA 13.0 on 24.04, 12.8 on 22.04). A consumer that overrides `cuda_version` or `tensorrt_version` must supply the matching closure through `nvidia_lockfile_path`:

```bash
ansible-playbook autoware.dev_env.install_nvidia \
  --extra-vars "rosdistro=jazzy use_locked_versions=true cuda_version=12.8 tensorrt_version=10.8.0.43-1+cuda12.8" \
  --extra-vars "nvidia_lockfile_path=/path/to/locked-nvidia-ubuntu2404-cuda12.8-amd64.yaml"
```

`rosdistro` is needed even though the closure does not depend on it: unlike `install_dev_env`, the `install_nvidia` playbook does not define it, and `lockfile_path` — loaded before `nvidia_lockfile_path` is consulted — interpolates it. (`base-cuda.Dockerfile` passes it, which is why the Docker path needs no such note.)

The file needs only a top-level `nvidia_pins` mapping, in the same format as a lockfile's, and is produced the same way (`emit_nvidia_pins.py`, see below). It **replaces** `nvidia_pins` rather than merging into it, and `verify.yaml` then checks the substituted closure, so the pins stay exact and the freeze is unchanged — only the source file moves.

Leaving `nvidia_lockfile_path` empty while overriding a version is not silently tolerated: the `cuda` role fails when no `cuda-*-<version>` entry covers this run, and the `tensorrt` role fails when the pinned `libnvinfer10` disagrees with `tensorrt_version`. Without those guards the CUDA half would install unfrozen (its version-suffixed names simply are not in the closure, and `verify.yaml` reports drift only for keys that are installed) and the TensorRT half would fail with an unattributable `no available installation candidate`.

### Snapshot reconcile

In locked mode the role does not only pin future installs: right after the
dated snapshot source is configured, a reconcile step walks every **already
installed** package that the snapshot serves at a different version back to
the snapshot's version (`tasks/reconcile.yaml`, driven by
`files/list_ros_snapshot_drift.py`). This matters most for Docker builds: the
digest-pinned `ros:<distro>-ros-base` base image is built from a floating tag
and is usually newer than `ros_snapshot_date`, so without the reconcile the
final image would carry a hybrid ROS closure. The step is scoped to packages
whose apt candidate originates from `snapshots.ros.org`; Ubuntu-archive
packages are left at the digest-frozen base-image state plus explicit
`apt_pins`. After all roles run, verification asserts the whole installed
closure matches the snapshot (`tasks/verify.yaml`).

Packages under an `apt-mark hold` are **skipped** by both the reconcile and
the closure assert, and listed in the play output. A hold is an explicit
operator decision to keep one package where it is; apt refuses to act on a
held package unless told to override the hold, so including one would abort
the entire reconcile. The trade-off is that a held package is outside the
freeze — release the hold if you want locked mode to govern it.

## Generating lockfiles

On a machine already provisioned by `install_dev_env`, run:

```bash
ROS_DISTRO=<distro> ROS_SNAPSHOT_DATE=<YYYY-MM-DD> ./ansible/scripts/generate_ansible_lockfile.sh
```

`ROS_SNAPSHOT_DATE` must match the snapshot the machine was provisioned from.

The script reads the package names from the existing lockfile for the current distro/architecture and fills in the versions currently installed on the machine. It rewrites `apt_pins` and `pip_pins`, and preserves `nvidia_pins` and `ros_overrides` verbatim.

### Recording the NVIDIA closure

`nvidia_pins` is filled by a separate script, because the generator above has no way to
tell an NVIDIA-repo package from an Ubuntu-archive one. On a machine that has just
completed an **unlocked** `install_nvidia`, run:

```bash
sudo apt-get update && sudo apt-get install -y python3-yaml   # the script imports yaml and shells out to apt-get/apt-helper/dpkg-query
./ansible/scripts/emit_nvidia_pins.py ansible/vars/locked-versions-<rosdistro>-<arch>.yaml
```

<!-- cspell:ignore indextargets -->

The script enumerates every package offered by an NVIDIA apt index (via `apt-get indextargets`,
so it works with apt's compressed index formats), keeps the ones that are actually installed at a
version that index offers, and rewrites **only** the `nvidia_pins` section, carrying over the header
comment block and the contents of every other section. It fails rather than writing a partial file
if the lockfile has an unknown top-level key or if no NVIDIA-origin packages are installed.

The two generators are order-independent on an already-filled lockfile — `generate_ansible_lockfile.sh`
preserves `nvidia_pins` and this script preserves `apt_pins`/`pip_pins` — but both must run on a
machine provisioned from the same snapshot date. On a lockfile that has no `ros_snapshot_date` yet,
run `generate_ansible_lockfile.sh` first; this script refuses an unfilled lockfile.

The same script records a **closure-only** file for `nvidia_lockfile_path`. Given a file whose only
top-level key is `nvidia_pins` (an empty one, or one carrying just a header comment), it fills that
key and leaves the file with no ROS sections — so a consumer's closure never carries a
`ros_snapshot_date` it does not own. Run it on a machine provisioned with the `cuda_version` /
`tensorrt_version` that file is meant to describe:

```bash
printf '# NVIDIA closure for ubuntu2404 / CUDA 12.8 / amd64\nnvidia_pins: {}\n' >closure.yaml
./ansible/scripts/emit_nvidia_pins.py closure.yaml
```

## When you add a new dependency

Key insertion is the responsibility of the PR that changes a role; the
`regenerate-lockfiles` workflow refreshes **values** of existing keys but
never discovers new ones.

| New dependency                                      | Lockfile action                                                                                                                       | Enforced by                                                               |
| --------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------- |
| ROS-repo package (`ros-*`, `python3-colcon-*`, ...) | none — `ros_snapshot_date` freezes it                                                                                                 | snapshot reconcile + closure verify                                       |
| Ubuntu-archive package                              | add the key to `apt_pins` in **all four** lockfiles in the same PR (any current version; the next regeneration canonicalizes it)      | locked-mode verify fails listing unpinned newly-installed Ubuntu packages |
| pip package                                         | add the key to `pip_pins` in the same PR                                                                                              | documented convention only                                                |
| NVIDIA-repo package                                 | run `emit_nvidia_pins.py` on a machine after an unlocked `install_nvidia`; the regenerate workflow does **not** refresh `nvidia_pins` | documented convention only                                                |
| Third-party apt repo package (PPA, vendor repo)     | no lockfile section covers it — see "What the freeze does not cover" below                                                            | nothing                                                                   |
| Version refresh of existing keys                    | dispatch `regenerate-lockfiles.yaml` (updates lockfiles, snapshot dates, and base-image digests atomically)                           | `validate-lockfiles`                                                      |

The Ubuntu-archive check's "Enforced by" only holds on a fresh host: it diffs
`apt-mark showmanual` taken before any role runs against the same list taken
after, so a package that is already manually installed going in — a rerun on
a developer machine, or a Docker stage inheriting from one that installed it
already — never shows up in the diff and slips past unchecked. Treat a green
local run as covering only what was newly installed manual on that host, not
as proof the lockfiles have no gaps; the check is reliable on the fresh
containers CI builds from.

A key in `apt_pins` also obliges the task that installs the package to
**converge** it — `state: latest` with
`allow_downgrade: "{{ use_locked_versions | default(false) | bool }}"`, the
idiom `ros2_dev_tools` uses. `state: present` is a no-op on an
already-installed package, so on a pre-provisioned machine the pin would never
be applied while the verification still fails on the mismatch, with advice the
user cannot act on. Pin only what a task actually moves.

Kernel-coupled packages (`linux-headers-*`, `linux-image-*`, `linux-modules-*`)
are exempt from the check. `agnocast` installs `linux-headers-{{ ansible_kernel }}`
on bare metal; its name varies per machine and changes on every kernel upgrade,
so no static lockfile key could cover it. It is a host property, not a
dependency of Autoware, and freezing it is neither possible nor desirable.

### What the freeze does not cover

The table above is not a complete inventory of what an install pulls in. Five
categories sit outside the freeze today, and the completeness check reports
none of them:

- **Third-party apt repositories.** The check only reports packages whose
  installed version carries `Origin: Ubuntu`, so a package from any other
  origin passes silently. There are three live cases, all installed top-level
  with `state: present` and covered by none of `ros_snapshot_date`,
  `apt_pins`, `nvidia_pins`, or `pip_pins`. `agnocast` installs two packages
  from a Launchpad PPA (`Origin: LP-PPA-...`): its heaphook package, and
  `agnocast_kmod_package` on any non-container host. `cuda` installs the
  `nvidia-open` driver metapackage from NVIDIA's apt repository
  (`developer.download.nvidia.com`) when `cuda_install_drivers=true`;
  `nvidia_pins` freezes the CUDA toolkit and TensorRT packages that role names
  explicitly, not the driver. A locked install that runs either role is
  therefore not fully frozen.
- **pip/pipx-managed packages.** `pip_pins` is currently empty and no role
  reads it. The pipx installs in `dev_tools` and `huggingface_cli`
  (`pre-commit`, `clang-format`, `huggingface_hub`) and the Python virtual
  environment installs in `acados` all resolve to whatever the Python package
  index serves at install time.
- **ROS packages the snapshot does not serve at all.** `ros_snapshot_date`
  freezes the packages the dated snapshot carries. A `ros-*` package absent
  from the snapshot is not frozen, and what happens to it depends on whether a
  rolling `packages.ros.org` source is configured on the host, because
  `ros2/tasks/main.yaml:1-2` gates the whole `ros-apt-source` block on
  `not use_locked_versions` and `snapshot_source.yaml` writes only the snapshot
  source:
  - In Docker, where `ros:<distro>-ros-base` brings its own source, and on a
    machine that previously ran unlocked, such a package **floats**: it
    installs from rolling at the default priority 500, since the
    `Pin: origin snapshots.ros.org` wildcard cannot apply to a package that
    origin does not offer.
  - On a fresh bare-metal locked run, no rolling source is ever configured,
    so such a package is **unreachable** rather than floating.

  This is not hypothetical: the noble rolling index carries 71
  `ros-jazzy-autoware-*` names the 2026-04-13 snapshot lacks, and
  `core.Dockerfile` deletes the `autoware_core` and `autoware_rviz_plugins`
  `package.xml` so that the rest of `src/core` resolves them from apt instead
  of from source. The closure verify does not report any of this either: it
  compares against packages whose apt _candidate_ comes from the snapshot, and
  theirs does not.

- **Packages installed outside the roles.** The completeness check diffs
  `apt-mark showmanual` across the play, so anything installed before
  `version_lock` runs is invisible to it — including the five packages
  `docker/base.Dockerfile` installs with a raw `apt-get` before the pin file
  exists. Four of those five have no lockfile key at all; `pipx` has a key in
  all four lockfiles, but the pin is not in effect for that install, so its
  version is equally unfrozen there. Transitive Ubuntu dependencies are not
  checked either (only top-level installs are), nor are the packages the ten
  `rosdep install` call sites resolve.
- **Packages outside the freeze's remit.** `unzip` is installed only by
  `demo_artifacts`, which is `tags: [never]` and appears in no other playbook,
  so no normal locked path converges it — while `verify.yaml` asserts every pin
  whose package is installed regardless of which roles ran. Since `unzip` is a
  hard `Depends` of `ubuntu-desktop`, pinning it failed a locked
  `install_dev_env` on any desktop whose `unzip` differed from the pin, with no
  task in the play able to fix it. It is therefore exempt via `OUT_OF_SCOPE` in
  `check_pin_completeness.py` rather than pinned, and floats on a
  `--tags demo_artifacts` run.

These are known gaps rather than oversights: closing them means changing how
those roles install, which is out of scope for the lockfile mechanism itself.

## Validating lockfiles

```bash
./ansible/scripts/validate_lockfiles.sh
```

This checks that every `ansible/vars/locked-versions-*.yaml` file is valid YAML with a `ros_snapshot_date` and `apt_pins`/`pip_pins`/`nvidia_pins`/`ros_overrides` mappings.

Run with no arguments, it additionally cross-checks that the set of ROS distros
in `docker/docker-bake.hcl`'s `BASE_IMAGE_DIGESTS` map exactly matches the distros
that have lockfiles here, and fails if they drift. A locked Docker build needs
both halves — the lockfile freezes the apt/ROS closure, the digest pins the base
image — so a distro present in only one silently defeats reproducibility. When
you add a lockfile for a new distro, add its base-image digest to the map (and
vice-versa); regenerate both together so the digest matches `ros_snapshot_date`.

## Lockfile format

A YAML mapping with five keys, one file per distro/arch
(`ansible/vars/locked-versions-<rosdistro>-<arch>.yaml`):

```yaml
ros_snapshot_date: "2026-04-13" # a real published date under snapshots.ros.org/<distro>/
apt_pins: # Ubuntu-archive origin only, sorted by name; rendered as APT pins
  ccache: 4.9.1-1
  git-lfs: 3.4.1-1ubuntu0.4
pip_pins: {} # pip/pipx origin; consumed by roles, NOT rendered as APT pins
nvidia_pins: # NVIDIA apt origin (CUDA/TensorRT closure); rendered as APT pins
  cuda-nvcc-12-8: 12.8.93-1
  libnvinfer10: 10.8.0.43-1+cuda12.8
ros_overrides: {} # exception pins for individual ROS packages; normally empty
```

- `ros_snapshot_date` drives the dated `snapshots.ros.org` source configured by the `ros2` role
  in locked mode, which freezes the **entire** ROS dependency closure to that date. The pin file
  also carries a `Package: * / Pin: origin snapshots.ros.org / Pin-Priority: 1001` stanza so the
  snapshot outranks the rolling `packages.ros.org` repo even on a machine that already had it —
  otherwise both sit at priority 500 and apt would install the newer rolling build.
- `apt_pins` covers Ubuntu-archive packages. It is rendered into `/etc/apt/preferences.d/autoware-lock`
  with `Pin-Priority: 1001`.
- `pip_pins` is the declared home for pip/pipx-managed packages. It is meant to be consumed
  directly by the relevant roles and is **not** rendered as APT pins. It is currently `{}` in
  every lockfile and no role reads it: `gdown`, its last consumer, was removed. See "What the
  freeze does not cover" for the pip/pipx installs that remain unpinned.
- `nvidia_pins` covers the CUDA/TensorRT closure from NVIDIA's apt repo, rendered into
  `/etc/apt/preferences.d/autoware-lock` with `Pin-Priority: 1001` like `apt_pins`. It is a
  separate section because NVIDIA publishes no dated snapshot to freeze against — its repo is
  accretive and only ever serves the newest build by default — so every package in the installed
  closure has to be pinned by exact version. It is populated by `emit_nvidia_pins.py`, not by
  `generate_ansible_lockfile.sh` (see "Recording the NVIDIA closure" above), and consumed by the
  `install_nvidia` playbook, which also enables apt downgrades in locked mode so a machine holding
  a newer CUDA can be walked back to the pinned versions.
- `ros_overrides` is rendered into `/etc/apt/preferences.d/autoware-lock` with `Pin-Priority: 1002`,
  one above the snapshot-origin pin so an override wins for its one package.
- ROS-repo packages (`ros-*`, `python3-colcon-*`, `python3-rosdep`, `python3-vcs2l`,
  `python3-bloom`) are **not** listed in `apt_pins`: the snapshot date freezes them. To move a
  single ROS package ahead of the snapshot, add a one-line entry under `ros_overrides`.
- After all roles run, the playbook verifies (via `tasks/verify.yaml`) that every `apt_pins`,
  `nvidia_pins`, and `ros_overrides` entry actually installed at its locked version, and **fails**
  listing any drift.
  A pin whose version is reachable in no configured source matches nothing, so apt would otherwise
  fall back to the newest candidate silently.

### Overriding a single ROS package

`ros_overrides` is normally `{}` — the `ros_snapshot_date` freezes every ROS package the snapshot
serves, so no per-package ROS pins are needed. Use it only to move **one** ROS package to a
different version than the snapshot (for example, to pick up a security or bug fix) without
advancing `ros_snapshot_date` for everything else. Each entry is `package: version`, where `version`
is the exact APT version string, and **that version must be reachable from a configured APT
source**.

This is the important constraint: in locked mode the dated snapshot is the only ROS source
`version_lock` configures, and it serves exactly one build per package, so an override to any other
version resolves to nothing unless you also make it reachable. (A `ros-*` package the snapshot does
not carry at all is a separate case — see "What the freeze does not cover" above.) In practice that
means pointing the whole snapshot at a later date that contains the build you want, and pinning that
exact build so nothing else moves:

```yaml
ros_snapshot_date: "2026-05-20" # a snapshot that actually serves the build below
apt_pins:
  ccache: 4.9.1-1
pip_pins: {}
ros_overrides:
  ros-jazzy-rmw-cyclonedds-cpp: 2.2.4-1noble.20260520.083000 # a build present in the 2026-05-20 snapshot
```

The override above is rendered into `/etc/apt/preferences.d/autoware-lock`, one priority above the
snapshot-origin pin so it wins for its single package:

```text
Package: ros-jazzy-rmw-cyclonedds-cpp
Pin: version 2.2.4-1noble.20260520.083000
Pin-Priority: 1002
```

If the pinned version is **not** reachable from any configured source, apt silently installs the
newest candidate instead — but the post-install verification (see `tasks/verify.yaml`) now catches
that and fails the run. Keep the override list as small as possible and drop entries once the
snapshot date catches up.
