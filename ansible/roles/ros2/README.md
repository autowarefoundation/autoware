# ros2

This role installs [ROS 2](http://www.ros2.org/) following [this page](https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html).

Additional steps may be needed depending on the `rosdistro` you choose.

<!-- TODO: Add these steps to the role if Humble requires. -->

```bash
$ sudo apt install software-properties-common
$ sudo add-apt-repository universe
$ apt-cache policy | grep universe
 500 http://us.archive.ubuntu.com/ubuntu focal/universe amd64 Packages
     release v=20.04,o=Ubuntu,a=focal,n=focal,l=Ubuntu,c=universe,b=amd64
```

## Inputs

| Name              | Required | Description                                      |
| ----------------- | -------- | ------------------------------------------------ |
| rosdistro         | true     | ROS distro.                                      |
| installation_type | false    | The installation type (`desktop` or `ros-base`). |
