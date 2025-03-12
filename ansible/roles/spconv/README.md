# spconv

This role install the `cumm` and `spconv` libraries needed to perform sparse convolutions.
The [original implementation](https://github.com/traveller59/spconv) did not provide a shared library, which is pre-generated c++ code and pre-compiled libraries were prepared [separatedly](https://github.com/autowarefoundation/spconv_cpp).

## Manual Installation

For manual installation, please follow the instructions in [this](https://github.com/autowarefoundation/spconv_cpp) repository.

## Run the playbook

The following command will install a particular version of the packages using ansible.

```bash
export CUMM_VERSION=0.5.3
export SPCONV_VERSION=2.3.8
ansible-playbook autoware.dev_env.install_spconv.yaml -e cumm_version=${CUMM_VERSION} -e spconv_version=${SPCONV_VERSION} --ask-become-pass
```
