# 2. Installation

Setup for both x86-based and Jetson-based ECUs.

> **Note**
>
> Internet connection is required in this step.

## 2-1. x86-based ECU

> **Warning**
>
> Network settings are automatically updated.
>
> During this procedure, IP addresses are assigned to some network interfaces (refer to the connection diagram on [1.Hardware setup](01_hardware_setup.md) for more detail) using `netplan`.
> This behavior may cause unexpected disconnection, if you are accessing the ECU remotely via those interfaces.
>
> If you would like to change network interfaces or IP addresses to be assigned, edit `edge-auto/ansible/playbooks/vars/edge_auto.yaml` before running `setup-dev-env.sh`

### Download the repository and setup your environment

As a first step, clone `tier4/edge-auto` and move to the directory.

```sh
git clone https://github.com/tier4/edge-auto.git
cd edge-auto
```

You can install the dependencies using the provided ansible script.

```sh
./setup-dev-env.sh
```

Finally, please reboot the system to make the installed dependencies and permission settings effective.

```sh
sudo reboot
```

### Build edge-auto

Create your ROS workspace and clone repositories using vcstool.

```sh
cd edge-auto
mkdir src
vcs import src < autoware.repos
```

Install ros package dependencies and build your ROS workspace.

```sh
rosdep update
./build.sh

...
Finished <<< image_projection_based_fusion [0.25s]
Starting >>> edge_auto_launch
Finished <<< edge_auto_launch [0.22s]

Summary: 66 packages finished [6.85s]
  2 packages had stderr output: extrinsic_interactive_calibrator intrinsic_camera_calibrator
```

## 2-2. Jetson-based ECU

> **Note**
>
> This following steps can be performed from your x86-based ECU via ssh.

### Download the repository and setup the environment

As a first step, clone `tier4/edge-auto-jetson` and move to the directory.

```sh
git clone https://github.com/tier4/edge-auto-jetson.git
cd edge-auto-jetson
```

You can install the dependencies using the provided ansible script.
During the installation process, you will be asked if you want to install the TIER IV camera driver.
If you already have the driver installed and want to skip this step, please type `N` to continue.

> **Note**
>
> `setup-dev-env.sh` script may take several hours.

```sh
./setup-dev-env.sh

[Warning] Do you want to install/update the TIER IV camera driver? [y/N]:
[Warning] Do you want to configure the network? This configuration may overwrite the IP address of the specific network interface [y/N]:
```

Finally, please reboot the system to make the installed dependencies and permission settings effective.

```sh
sudo reboot
```

If you chose `y` for the prompt of the network configuration,
the IP address for the specified network interface is fixed (default: `192.168.2.2` for `eth1`).
In that case, you can access the Jetson-based ECU and perform the following steps via remote access, such as:
```sh
ssh <IP_address> -l <username>
```

### Build edge-auto-jetson workspace

Create your ROS workspace and clone repositories using vcstool.

```sh
cd edge-auto-jetson
mkdir src
vcs import src < autoware.repos
```

Build your ROS workspace.

```sh
./build.sh
```

## (Optional) Update your workspace

If you want to update cloned repositories, use the following command.

```sh
vcs import src < autoware.repos
vcs pull src
```

## (Optional) Modify camera exposure timing

> **Note**
>
> On the sample system introduced in [1.Hardware setup](01_hardware_setup.md) step, this doesn't need to be changed.

If you want to change the exposure time of cameras for sensor synchronization, please modify the following files.

```sh
edge-auto-jetson/src/individual_params/individual_params/config/
└── default
    ├── camera0
    │   ├── trigger.param.yaml
    ├── camera1
    │   ├── trigger.param.yaml
```

For more details, please refer to the [tier4/sensor_trigger](https://github.com/tier4/sensor_trigger) repository.
