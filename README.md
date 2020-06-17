[![Autoware](https://www.autoware.ai/static/img/autoware_web_img.png)](https://www.autoware.ai)

![Native CI workflow](https://github.com/Autoware-AI/autoware.ai/workflows/Native%20CI%20workflow/badge.svg) ![CUDA CI workflow](https://github.com/Autoware-AI/autoware.ai/workflows/CUDA%20CI%20workflow/badge.svg) ![Cross CI workflow](https://github.com/Autoware-AI/autoware.ai/workflows/Cross%20CI%20workflow/badge.svg)

[Autoware](https://www.autoware.ai) is the world's first "all-in-one" open-source software for self-driving vehicles. The capabilities of Autoware are primarily well-suited for urban cities, but highways, freeways, mesomountaineous regions, and geofenced areas can be also covered. The code base of Autoware is protected by the Apache 2 License. Please use it at your own discretion. For safe use, we provide a ROSBAG-based simulation environment for those who do not own real autonomous vehicles. If you plan to use Autoware with real autonomous vehicles, **please formulate safety measures and assessment of risk before field testing.**

You may refer to [Autoware Wiki](https://github.com/Autoware-AI/autoware.ai/wiki/home) for **Users Guide** and **Developers Guide**.

## What Is Autoware

[![Autoware
Overview](docs/images/autoware_overview.png)](https://github.com/Autoware-AI/autoware.ai/wiki/Overview)

Autoware provides a rich set of self-driving modules composed of sensing, computing, and actuation capabilities. An overview of those capabilities is described [here](https://github.com/Autoware-AI/autoware.ai/wiki/Overview). Keywords include *Localization, Mapping, Object Detection & Tracking, Traffic Light Recognition, Mission & Motion Planning, Trajectory Generation, Lane Detection & Selection, Vehicle Control, Sensor Fusion, Cameras, LiDARs, RADARs, Deep Learning, Rule-based System, Connected Navigation, Logging, Virtual Reality, and so on*.

Free manuals can be also found at [Autoware-Manuals](https://github.com/CPFL/Autoware-Manuals). You are encouraged to contribute to the maintenance of these manuals. Thank you for your cooperation!

## Getting Started

[![Autoware Demo](docs/images/autoware_demo.png)](https://github.com/Autoware-AI/autoware.ai/wiki/Demo)

### Recommended System Specifications

- Number of CPU cores: 8
- RAM size: 32GB
- Storage size: 64GB+

### Users Guide

1. [Installation](https://github.com/Autoware-AI/autoware.ai/wiki/Installation)
    1. [Docker](https://github.com/Autoware-AI/autoware.ai/wiki/Docker)
    1. [Source](https://github.com/Autoware-AI/autoware.ai/wiki/Source-Build)
1. [Demo](https://github.com/Autoware-AI/autoware.ai/wiki/ROSBAG-Demo)
1. [Field Test](https://github.com/Autoware-AI/autoware.ai/wiki/Field-Test)
1. [Simulation Test](https://github.com/Autoware-AI/autoware.ai/wiki/Simulation-Demo)
1. [Videos](https://github.com/Autoware-AI/autoware.ai/wiki/Videos)

### Developers Guide

1. [Contribution Rules](https://github.com/Autoware-AI/autoware.ai/wiki/Contributing-to-Autoware) (**Must Read**)
1. [Overview](https://github.com/Autoware-AI/autoware.ai/wiki/Overvieww)
1. [Specification](https://github.com/Autoware-AI/autoware.ai/wiki/Specification)


## Research Papers for Citation

1. S. Kato, S. Tokunaga, Y. Maruyama, S. Maeda, M. Hirabayashi, Y. Kitsukawa, A. Monrroy, T. Ando, Y. Fujii, and T. Azumi,``Autoware on Board: Enabling Autonomous Vehicles with Embedded Systems,'' In Proceedings of the 9th ACM/IEEE International Conference on Cyber-Physical Systems (ICCPS2018),  pp. 287-296, 2018. [Link](https://dl.acm.org/citation.cfm?id=3207930)

2. S. Kato, E. Takeuchi, Y. Ishiguro, Y. Ninomiya, K. Takeda, and T. Hamada. ``An Open Approach to Autonomous Vehicles,'' IEEE Micro, Vol. 35, No. 6, pp. 60-69, 2015. [Link](https://ieeexplore.ieee.org/document/7368032/)

## Cloud Services

### Autoware Online

You may test Autoware at [Autoware Online](http://autoware.online/). No need to install the Autoware repository to your local environment.

### Automan

You may annotate and train your ROSBAG data using your web browser through [Automan](https://www.automan.ai). The trained models can be used for deep neural network algorithms in Autoware, such as SSD and Yolo.

### ROSBAG STORE

You may download a number of test and simulation data sets from Tier IV's [ROSBAG STORE](https://rosbag.tier4.jp). Note that free accounts would not allow you to access image data due to privacy matters.

### Map Tools

You may create 3D map data through Tier IV's [Map Tools](https://maptools.tier4.jp/). The 3D map data used in Autoware are composed of point cloud structure data and vector feature data.

## License

Autoware is provided under the [Apache 2 License](https://github.com/Autoware-AI/autoware.ai/blob/master/LICENSE).

## Contact

[Autoware Discourse](https://discourse.ros.org/c/autoware)

[Autoware Developers Slack Team](https://autoware.herokuapp.com/)

Please see the [Support Guidelines](https://github.com/Autoware-AI/autoware.ai/wiki/Support-guidelines) for more details about getting help.

***
<div align="center"><img src="docs/images/autoware_logo_1.png" width="400"/></div>
