# Autoware demo artifacts

Downloads sample maps and rosbag recordings used by the Autoware demos:

- [Planning simulation](https://autowarefoundation.github.io/autoware-documentation/main/demos/planning-sim/) — uses `sample-map-planning`
- [Rosbag replay simulation](https://autowarefoundation.github.io/autoware-documentation/main/demos/rosbag-replay-simulation/) — uses `sample-map-rosbag` and `sample-rosbag`
- [CARLA simulation](../../../docker/examples/demos/carla/README.md) — uses `carla-kashiwanoha`

The maps and the recordings are hosted on the `autoware-files` S3 bucket. The CARLA map is hosted on [Hugging Face](https://huggingface.co/datasets/AutowareFoundation/map-carla-kashiwanoha), pinned to tag `0.2.0`. The download drops the demo video in that dataset, which no node reads.

## Layout

After running the role, the following layout is created under `demo_artifacts__autoware_data_dir` (default `~/autoware_data`):

```console
~/autoware_data
├── maps
│   ├── carla-kashiwanoha/
│   ├── sample-map-planning/
│   ├── sample-map-planning.zip
│   ├── sample-map-rosbag/
│   └── sample-map-rosbag.zip
└── recordings
    └── bags
        ├── sample-rosbag/
        └── sample-rosbag.zip
```

`carla-kashiwanoha/` is a complete map directory:

```console
~/autoware_data/maps/carla-kashiwanoha
├── kashiwanoha.xodr          # read by CARLA, to build the world
├── lanelet2_map.osm          # read by Autoware
└── map_projector_info.yaml   # read by Autoware
```

CARLA has no map of this area, so it builds the world from `kashiwanoha.xodr`. Both files describe the same roads, against the one origin in `map_projector_info.yaml`. The [dataset card](https://huggingface.co/datasets/AutowareFoundation/map-carla-kashiwanoha) carries the world build step and the provenance of each file.

## Run

```bash
ansible-galaxy collection install -f -r "ansible-galaxy-requirements.yaml"
ansible-playbook autoware.dev_env.install_dev_env --tags demo_artifacts --ask-become-pass
```

To change the install location:

```bash
ansible-playbook autoware.dev_env.install_dev_env --tags demo_artifacts \
  -e "demo_artifacts__autoware_data_dir=$HOME/autoware_data" --ask-become-pass
```
