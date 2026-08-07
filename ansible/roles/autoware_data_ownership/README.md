# Role: autoware_data_ownership

This role makes the user that runs the playbook the owner of `~/autoware_data` and of every path under it.

`~/autoware_data` holds the maps, the ML models, the recordings and the scenarios. It sits in the home directory of the user because every consumer reads it as that user:

- The ansible roles that download into it
- The ROS nodes that write TensorRT engine files next to the models
- The containers under `docker/`, which mount it for the unprivileged `aw` user

Older installs downloaded into this directory as root. A root-owned directory stops all three consumers. This role corrects such an install one time.

When `~/autoware_data` does not exist, the role does nothing. When the user already owns every path in it, the role does nothing. Only the repair needs sudo, so a clean install needs no password.

Remove this role when the installs in use have all migrated. Nothing in this repository creates a root-owned path under `~/autoware_data` any more, so a corrected install cannot return to that state:

- The roles that write into it no longer use `become`
- The compose files under `docker/` set `create_host_path: false`, so Compose stops on a missing mount source instead of creating it as root

## Inputs

| Name                           | Default                                      | Description                                                   |
| ------------------------------ | -------------------------------------------- | ------------------------------------------------------------- |
| `autoware_data_ownership__dir` | `{{ ansible_facts.env.HOME }}/autoware_data` | The directory to take ownership of, with everything under it. |

## Manual correction

```bash
sudo chown -Rh "$USER:$USER" "$(readlink -f ~/autoware_data)"
```

`readlink -f` resolves `~/autoware_data` when it is a symbolic link, so a tree that sits on another disk is corrected too. `-h` acts on a symbolic link inside the tree instead of the file it points to, so a link that points out of `~/autoware_data` leaves the owner of its target alone. The role runs this same command.
