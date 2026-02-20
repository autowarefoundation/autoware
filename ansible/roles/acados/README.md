# Setup acados

Installs [acados](https://github.com/acados/acados), a fast and embedded solver for nonlinear optimal control, from source.

## What it does

1. Clones the acados repository (shallow) to `/opt/acados`
2. Initializes submodules (shallow)
3. Builds and installs acados in-tree with `QPOASES` and position-independent code enabled
4. Downloads the [tera renderer](https://github.com/acados/tera_renderer) binary to `/opt/acados/bin/t_renderer` (supports `x86_64` and `aarch64`)
5. Adds `CMAKE_PREFIX_PATH` and `ACADOS_SOURCE_DIR` to the user's `.bashrc`

## Installation

Install ansible following the instructions in the [ansible installation guide](../../README.md#ansible-installation).

```bash
cd ~/autoware # The root directory of the cloned repository
ansible-galaxy collection install -f -r "ansible-galaxy-requirements.yaml"
```

This step should be repeated when the ansible directory is updated.

```bash
ansible-playbook autoware.dev_env.setup_acados --ask-become-pass
```

## Configuration

The acados version is pinned to `v0.5.3`. To update, change the `version` field in the git clone task and the role will rebuild automatically on the next run.

## Directory layout

After running, `/opt/acados` contains both the source tree and installed artifacts:

```text
/opt/acados/
├── bin/t_renderer    # tera renderer binary
├── build/            # cmake build directory
├── cmake/            # acadosConfig.cmake (used by find_package)
├── lib/              # built shared libraries
├── include/          # headers
├── external/         # submodules (qpoases, etc.)
└── ...               # remaining source tree
```

## Usage in CMake

After provisioning, any CMake project can use acados with:

```cmake
find_package(acados REQUIRED)
target_link_libraries(your_target PRIVATE acados)
```

No additional path configuration is needed — `CMAKE_PREFIX_PATH` is set via `.bashrc`.

## Requirements

- CMake
- A C compiler (gcc/clang)
- `make`
- `git`

## Supported architectures

- `x86_64` (amd64)
- `aarch64` (arm64)

The role will fail with a clear error on unsupported architectures.

## Idempotency

The role is safe to run multiple times. The git clone resets any local modifications, and the build steps re-run on each invocation to ensure correctness when the version is changed. The `.bashrc` entries use `lineinfile` with regex matching to avoid duplication.
