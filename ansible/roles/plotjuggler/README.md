# plotjuggler

This role installs PlotJuggler. You can access detailed information about PlotJuggler with this [link](https://www.plotjuggler.io/).

## Inputs

None.

## Manual Installation

```bash
wget -O /tmp/amd64.env https://raw.githubusercontent.com/autowarefoundation/autoware/main/amd64.env && source /tmp/amd64.env

sudo apt update && sudo apt install -y \
  ros-$rosdistro-plotjuggler-ros
```
