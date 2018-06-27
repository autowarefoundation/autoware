# Autoware Docker on 96boards

See [Wiki](https://github.com/CPFL/Autoware/wiki/Docker) for more information.

## How to Build
```
$ cd Autoware/docker/96boards/

# Ubuntu 16.04 (Kinetic)
$ sh build.sh kinetic
```

## How to Run
```
# Default shared directory path is /home/$USER/shared_dir

# Ubuntu 16.04 (Kinetic)
$ sh run.sh kinetic

# If you select your shared directory path
$ sh run.sh kinetic {SHARED_DIR_PATH}
```
