# spconv

This role install the `cumm` and `spconv` libraries needed to perform sparse convolutions.
The [original implementation](https://github.com/traveller59/spconv) did not provide a shared library, which is pre-generated c++ code and pre-compiled libraries were prepared [separately](https://github.com/autowarefoundation/spconv_cpp).

## Architecture Support

This role supports different architectures with platform-specific package variants:

- **amd64**: No suffix (e.g., `cumm_0.5.3_amd64.deb`)
- **arm64**:
  - Default: `-sbsa` suffix for server platforms (e.g., `cumm_0.5.3_arm64-sbsa.deb`)
  - Jetson: `-jetson` suffix when `spconv_is_jetson=true` (e.g., `cumm_0.5.3_arm64-jetson.deb`)

## Manual Installation

For manual installation, please follow the instructions in [this](https://github.com/autowarefoundation/spconv_cpp) repository.

## Run the playbook

The following command will install a particular version of the packages using ansible.

### Standard installation (amd64 or arm64 server)

```bash
export CUMM_VERSION=0.5.3
export SPCONV_VERSION=2.3.8
ansible-playbook autoware.dev_env.install_spconv.yaml -e cumm_version=${CUMM_VERSION} -e spconv_version=${SPCONV_VERSION} --ask-become-pass
```

### Installation for NVIDIA Jetson platforms

```bash
export CUMM_VERSION=0.5.3
export SPCONV_VERSION=2.3.8
ansible-playbook autoware.dev_env.install_spconv.yaml -e cumm_version=${CUMM_VERSION} -e spconv_version=${SPCONV_VERSION} -e spconv_is_jetson=true --ask-become-pass
```
