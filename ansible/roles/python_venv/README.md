# python_venv

This role creates a Python virtual environment inside the Autoware workspace (`<workspace>/.venv`) so that rosdep keys
using the `pip` installer (`*-pip`) can be resolved on Ubuntu 24.04 / ROS 2 Jazzy.

Since Python 3.11, the system Python of Ubuntu is marked as externally managed ([PEP 668](https://peps.python.org/pep-0668/)),
so `rosdep install` fails for pip keys because `pip install` into the system Python is refused.
rosdep's pip installer works when rosdep itself runs inside a virtual environment, so this role:

- installs `python3-venv`,
- creates `<workspace>/.venv` with `--system-site-packages` so that apt-installed ROS 2 Python packages (`rclpy`, colcon extensions, etc.) stay visible,
- writes small `rosdep` and `colcon` launchers into the venv. They run the apt-installed tools on the venv interpreter, so nothing comes from PyPI.
  - rosdep must run on the venv interpreter, otherwise its PEP 668 check fails.
  - colcon must run on the venv interpreter, because `ament_python` executables get the shebang of the interpreter that built them.
- adds `COLCON_IGNORE` to the venv so that colcon does not search packages in it.

The venv is created per workspace, not per package. If you have several Autoware workspaces (e.g. different versions),
each workspace has its own venv and you should activate the one of the workspace you are working on.

This role is optional and is not run by default. It works on Humble (Ubuntu 22.04) as well.

## Inputs

| Name                              | Required | Description                                                                                                                                    |
| --------------------------------- | -------- | ---------------------------------------------------------------------------------------------------------------------------------------------- |
| `python_venv__workspace_dir`      | false    | Root of the Autoware workspace. Defaults to the directory `ansible-playbook` is run from.                                                      |
| `python_venv__venv_dir`           | false    | Path of the venv. Defaults to `<python_venv__workspace_dir>/.venv`.                                                                            |
| `python_venv__tools`              | false    | apt-installed Python tools that get a launcher in the venv. Defaults to `rosdep` and `colcon`.                                                 |
| `python_venv__require_virtualenv` | false    | If `true`, sets `require-virtualenv = true` in `/etc/pip.conf`, so pip refuses every install outside a venv on this host. Defaults to `false`. |

## Run

```bash
cd ~/autoware # The root directory of the cloned repository
ansible-galaxy collection install -f -r "ansible-galaxy-requirements.yaml"
ansible-playbook autoware.dev_env.install_dev_env --tags python_venv --ask-become-pass
```

To create the venv for another workspace:

```bash
ansible-playbook autoware.dev_env.install_dev_env --tags python_venv \
  -e "python_venv__workspace_dir=$HOME/autoware_jazzy" --ask-become-pass
```

## Usage

Activate the venv before running rosdep, colcon and Autoware.

```bash
cd ~/autoware
source .venv/bin/activate
source /opt/ros/"$ROS_DISTRO"/setup.bash

# `--as-root pip:false` is required: otherwise rosdep runs `sudo -H pip3`, which resets PATH and uses the system pip.
# apt keys are still installed with sudo.
rosdep install -y --from-paths src --ignore-src --rosdistro "$ROS_DISTRO" --as-root pip:false

colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

source install/setup.bash
```

Python packages installed by rosdep pip keys go to `.venv` and do not affect the system Python.
To reset the environment, remove `.venv` and run the role again.

## Manual Installation

```bash
sudo apt-get update
sudo apt-get install -y python3-pip python3-venv

cd ~/autoware
python3 -m venv --system-site-packages .venv
touch .venv/COLCON_IGNORE

# rosdep and colcon come from apt. These launchers run them on the venv interpreter.
printf '#!%s/bin/python3\nimport sys\nfrom rosdep2.main import rosdep_main\nsys.exit(rosdep_main())\n' "$PWD/.venv" >.venv/bin/rosdep
printf '#!%s/bin/python3\nimport sys\nfrom colcon_core.command import main\nsys.exit(main())\n' "$PWD/.venv" >.venv/bin/colcon
chmod 755 .venv/bin/rosdep .venv/bin/colcon
```

## Docker images

Docker images create the same kind of venv at `/opt/autoware-venv` in `docker/base.Dockerfile` and put it first on `PATH` (see [docker/README.md](../../../docker/README.md)).
