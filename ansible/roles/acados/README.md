# Setup acados

Installs [acados](https://github.com/acados/acados), a fast and embedded solver for nonlinear optimal control, from source.

## What it does

1. Clones the acados repository (shallow) to `/opt/acados`
2. Initializes submodules (shallow)
3. Builds and installs acados in-tree with `QPOASES` and position-independent code enabled
4. Downloads the [tera renderer](https://github.com/acados/tera_renderer) binary to `/opt/acados/bin/t_renderer` (supports `x86_64` and `aarch64`)
5. Creates a Python virtual environment at `/opt/acados/.venv`
6. Installs `casadi`, `sympy`, and `acados_template` (editable) in the venv
7. Adds `CMAKE_PREFIX_PATH`, `ACADOS_SOURCE_DIR`, and `LD_LIBRARY_PATH` to the user's `.bashrc`

## Installation ⭐

Install ansible following the instructions in the [ansible installation guide](../../README.md#ansible-installation).

```bash
cd ~/autoware # The root directory of the cloned repository
ansible-galaxy collection install -f -r "ansible-galaxy-requirements.yaml"
```

This step should be repeated when the ansible directory is updated.

```bash
ansible-playbook autoware.dev_env.setup_acados --ask-become-pass
```

## Directory layout

After running, `/opt/acados` contains both the source tree and installed artifacts:

```text
/opt/acados/
├── .venv/            # Python virtual environment
├── bin/t_renderer    # tera renderer binary
├── build/            # cmake build directory
├── cmake/            # acadosConfig.cmake (used by find_package)
├── lib/              # built shared libraries
├── include/          # headers
├── interfaces/       # acados_template Python package (installed to .venv)
├── external/         # submodules (qpoases, etc.)
└── ...               # remaining source tree
```

## Usage in CMake

After provisioning, any CMake project can use acados with:

```cmake
find_package(acados REQUIRED)
target_link_libraries(your_target PRIVATE acados)
```

No additional path configuration is needed, `CMAKE_PREFIX_PATH` is set via `.bashrc`.

## Using the Python interface during `colcon build`

If your ROS 2 package generates acados C code at build time (e.g. via a Python script that uses `acados_template` and `casadi`), you need to source the venv so the code-generation script can find the installed packages.

The Ansible role already exports `ACADOS_SOURCE_DIR` and `LD_LIBRARY_PATH` in `.bashrc`, so those are available to `colcon build` as long as you've sourced your shell. The only extra step is pointing CMake at the venv Python so `casadi`, `sympy`, and `acados_template` are importable:

```cmake
find_program(ACADOS_PYTHON NAMES python3 PATHS $ENV{ACADOS_SOURCE_DIR}/.venv/bin NO_DEFAULT_PATH)

add_custom_command(
  OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/generated/acados_solver.c
  COMMAND ${ACADOS_PYTHON} ${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_solver.py
    --output-dir ${CMAKE_CURRENT_BINARY_DIR}/generated
  DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_solver.py
  COMMENT "Generating acados solver C code"
)
```

The generated C code is then compiled as part of your normal CMake target, no Python dependency at runtime.

## Requirements

- CMake
- A C compiler (gcc/clang)
- `make`
- `git`
- `python3` with `venv` module (`python3-venv` on Ubuntu)

## Idempotency

The role is safe to run multiple times. The git clone resets any local modifications, and the build steps re-run on each invocation to ensure correctness when the version is changed. The `.bashrc` entries use `lineinfile` with regex matching to avoid duplication. The venv creation is skipped if it already exists (`creates` guard).
