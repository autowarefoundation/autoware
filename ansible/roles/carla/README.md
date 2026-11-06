# carla

Downloads a [CARLA](https://carla.org/) simulator package for Linux and installs the packages the server needs at run time.

This role is for the work on CARLA on Unreal Engine 5.8 ([#7385](https://github.com/orgs/autowarefoundation/discussions/7385)). Autoware does not drive the UE 5.8 server yet: `autoware_carla_interface` targets CARLA 0.9.15, and the [CARLA demo](../../../docker/examples/demos/carla/README.md) runs the `carlasim/carla:0.9.16` image. To drive Autoware with CARLA today, use those.

| `carla__version` | Package                                                                                                           | Engine            | Pinned               |
| ---------------- | ----------------------------------------------------------------------------------------------------------------- | ----------------- | -------------------- |
| `1.0` (default)  | [CARLA 1.0 release](https://github.com/carla-simulator/carla/releases/tag/1.0)                                    | Unreal Engine 5.8 | yes, by sha256       |
| `nightly`        | [Nightly build](https://github.com/carla-simulator/carla/blob/ue58-dev/Docs/download.md) of the `ue58-dev` branch | Unreal Engine 5.8 | no, updated on rerun |

Notes on the packages:

- The file published as 1.0 reports itself as `0.10.0`: its top folder is `Carla-0.10.0-Linux-Shipping`.
- The 1.0 package ships the Python client as a `cp310` wheel only, which fits Ubuntu 22.04 (Humble). There is no client for the Python 3.12 of Ubuntu 24.04 (Jazzy).
- The nightly build changes under one URL, so an install of it is not reproducible. Use it to try the latest `ue58-dev`, not as a dependency.

CARLA publishes its Linux server for x86_64 only; the role stops on any other architecture. The package is about 13 GiB to download and about 14 GiB unpacked, and CARLA recommends an NVIDIA RTX GPU with driver 550 or later (see the [CARLA quick start](https://github.com/carla-simulator/carla/blob/ue58-dev/Docs/start_quickstart.md)). The new package is unpacked next to the old one, so updating an install needs room for the archive and both installs at once: about 42 GiB free, against about 27 GiB for a first install.

## Inputs

| Name                  | Default                                  | Description                                                                                                          |
| --------------------- | ---------------------------------------- | -------------------------------------------------------------------------------------------------------------------- |
| `carla__version`      | `1.0`                                    | Which package to install: a key of `carla__packages` (`1.0` or `nightly`).                                           |
| `carla__packages`     | `1.0` and `nightly`                      | The packages the role can install, each with its `url`, `sha256` (empty: unpinned), `launcher` (a list: candidates). |
| `carla__install_root` | `~/autoware_simulation/carla`            | Directory that holds one install per version.                                                                        |
| `carla__install_dir`  | `<carla__install_root>/<carla__version>` | Where the package is unpacked.                                                                                       |
| `carla__apt_packages` | `libomp5`, `libsdl2-2.0-0`, ...          | Runtime dependencies of the server, from CARLA's `Util/Docker/Release.Dockerfile`.                                   |

`xserver-xorg`, which the CARLA image also installs, is left out: the server runs headless with `-RenderOffScreen`, and `libsdl2-2.0-0` already brings the X client libraries. Install it yourself to show the server's window on a host without a desktop.

## Updates

A pinned package is downloaded only when `<carla__install_dir>/.carla-<sha256>` is missing, and the download must match the checksum. A new upload under the same URL therefore fails at the checksum instead of replacing the install.

For the nightly build, the role records the `ETag`, `Last-Modified` and size of the download in `<carla__install_dir>/.carla_download.json`. On the next run it asks the server about the package again and downloads it only if it changed, and does nothing when it is current or the server does not answer.

Either way, the new package is unpacked into `<carla__install_dir>.partial` and replaces the old install only once it is complete. The archive and the staging folder are removed at the end of the run, and at the start of the next one if a run stopped halfway. To download a package again, remove its install folder.

## Run

```bash
ansible-galaxy collection install -f -r "ansible-galaxy-requirements.yaml"

# CARLA 1.0
ansible-playbook autoware.dev_env.install_dev_env --tags carla --ask-become-pass

# The nightly build
ansible-playbook autoware.dev_env.install_dev_env --tags carla -e carla__version=nightly --ask-become-pass
```

Then start the server:

```bash
~/autoware_simulation/carla/1.0/Linux/CarlaUnreal.sh -RenderOffScreen -nosound
```

The role prints the launcher at the end of the run. A nightly built after [carla-simulator/carla#10158](https://github.com/carla-simulator/carla/pull/10158) (2026-11-03) has it at the top of the package, `~/autoware_simulation/carla/nightly/CarlaUnreal.sh`; an older one under `Linux/`. The role accepts either.

The role installs the server only. The Python client ships in the package, as wheels under `<carla__install_dir>/PythonAPI/carla/dist/`; install it into the environment that talks to CARLA yourself.
