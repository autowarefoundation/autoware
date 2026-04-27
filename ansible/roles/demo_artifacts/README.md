# Autoware demo artifacts

Downloads sample maps and rosbag recordings used by the Autoware demos:

- [Planning simulation](https://autowarefoundation.github.io/autoware-documentation/main/demos/planning-sim/) — uses `sample-map-planning`
- [Rosbag replay simulation](https://autowarefoundation.github.io/autoware-documentation/main/demos/rosbag-replay-simulation/) — uses `sample-map-rosbag` and `sample-rosbag`

The artifacts are hosted on the `autoware-files` S3 bucket.

## Layout

After running the role, the following layout is created under `demo_artifacts__autoware_data_dir` (default `~/autoware_data`):

```console
~/autoware_data
├── maps
│   └── demos
│       ├── sample-map-planning/
│       ├── sample-map-planning.zip
│       ├── sample-map-rosbag/
│       └── sample-map-rosbag.zip
└── recordings
    └── bags
        └── demos
            ├── sample-rosbag/
            └── sample-rosbag.zip
```

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
