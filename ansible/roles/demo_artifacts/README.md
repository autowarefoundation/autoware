# Autoware demo artifacts

Downloads sample maps and rosbag recordings used by the Autoware demos:

- [Planning simulation](https://autowarefoundation.github.io/autoware-documentation/main/demos/planning-sim/) — uses `sample-map-planning`
- [Rosbag replay simulation](https://autowarefoundation.github.io/autoware-documentation/main/demos/rosbag-replay-simulation/) — uses `sample-map-rosbag` and `sample-rosbag`
- [CARLA simulation](../../../docker/examples/demos/carla/README.md) — uses `carla-kashiwanoha`

The role also downloads Autoware maps for the CARLA towns as CARLA 0.10 re-authored them. Nothing in this repository runs CARLA 0.10 yet — see [Maps for CARLA 0.10](../../../docker/examples/demos/carla/README.md#maps-for-carla-010) for what is still missing, and do not point the demo above at them.

The maps and the recordings are hosted on the `autoware-files` S3 bucket. The CARLA maps are hosted on Hugging Face, each pinned to a revision:

| Dataset                                                                                             | Revision         | Dropped                                                 |
| --------------------------------------------------------------------------------------------------- | ---------------- | ------------------------------------------------------- |
| [`map-carla-kashiwanoha`](https://huggingface.co/datasets/AutowareFoundation/map-carla-kashiwanoha) | tag `0.2.0`      | the demo video, which no node reads                     |
| [`carla-ue5-maps`](https://huggingface.co/datasets/AutowareFoundation/carla-ue5-maps)               | commit `682ddd4` | every world but `Town10HD_Opt`, and the preview renders |

`carla-ue5-maps` carries no tag yet, so it is pinned to the commit that published `Town10HD_Opt`. Add a world to the `include` patterns in `tasks/main.yaml` to pull more of them.

## Layout

After running the role, the following layout is created under `demo_artifacts__autoware_data_dir` (default `~/autoware_data`):

```console
~/autoware_data
├── maps
│   ├── autoware_maps/
│   │   └── Town10HD_Opt/
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

`autoware_maps/` holds one directory per CARLA world, named after the world CARLA loads:

```console
~/autoware_data/maps/autoware_maps/Town10HD_Opt
├── lanelet2_map.osm          # read by Autoware
├── map_projector_info.yaml   # read by Autoware
└── pointcloud_map.pcd        # read by Autoware, for localization
```

Here CARLA already ships the town, so nothing in the directory is read by the simulator — pass the directory to Autoware as `map_path`, and its basename to CARLA as the world to load. The point clouds published alongside the CARLA towns were recorded on CARLA 0.9, whose towns are different geometry from the ones CARLA 0.10 re-authored in Unreal Engine 5; the ones here were recorded from 0.10 itself, so NDT has a map that matches the world.

That cuts both ways: **these maps need a CARLA 0.10 server**. Against the 0.9.16 server the [CARLA demo](../../../docker/examples/demos/carla/README.md) pins, the 0.10 point cloud describes geometry the world does not have, which is the same mismatch in the other direction. The role downloads the maps either way — they are ~24 MB and cost nothing to have on disk — but [Maps for CARLA 0.10](../../../docker/examples/demos/carla/README.md#maps-for-carla-010) lists what a 0.10 runtime still needs.

The dataset keeps the `autoware_maps/` prefix on its own files, and `hf download` preserves it, which is why the worlds land one level below `maps/` rather than beside `carla-kashiwanoha/`.

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
