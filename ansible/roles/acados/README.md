# acados role

Pre-builds **acados** and **t_renderer** so that `autoware_path_optimizer` can use them at colcon build time without building them in CMake.

## When to use

- After `vcs import src < repositories/autoware.repos --recursive` (so `src/universe/external/acados` and `tera_renderer` exist).
- After the **rust** role if you want t_renderer built (Rust/cargo required for that step).

## What it does

1. Configures and builds acados from `src/universe/external/acados`, installs to `src/universe/external/acados_install`.
2. Builds t_renderer with `cargo build --release` in `src/universe/external/tera_renderer`, copies the binary to `src/universe/external/acados/bin/t_renderer`.

After this, `colcon build --packages-select autoware_path_optimizer` will use these pre-built artifacts (when CMake is set up to prefer them).

## Usage

From the autoware repo root:

```bash
ANSIBLE_ROLES_PATH=ansible/roles ansible-playbook -i localhost, ansible/playbooks/build_acados.yaml
```

To pass the workspace path explicitly (e.g. in CI):

```bash
ansible-playbook -i localhost, ansible/playbooks/build_acados.yaml -e acados_workspace_root=$PWD
```

## Variables

- `acados_workspace_root`: Autoware repo root (default: inferred from playbook path).
- See `defaults/main.yaml` for paths derived from it (`acados_install_dir`, `tera_renderer_source_dir`, etc.).
