# Scripts for Autoware

## freeze-repos.sh

### Prerequisites

- [yq](https://github.com/mikefarah/yq)

  ```bash
  sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys CC86BB64
  sudo add-apt-repository ppa:rmescandon/yq
  sudo apt update
  sudo apt install yq -y
  ```

### Usage

```bash
./scripts/freeze-repos.sh
```
