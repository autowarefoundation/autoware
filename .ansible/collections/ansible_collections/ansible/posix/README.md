# ansible.posix
<!-- Add CI and code coverage badges here. Samples included below. -->
[![Build Status](
https://dev.azure.com/ansible/ansible.posix/_apis/build/status/CI?branchName=main)](https://dev.azure.com/ansible/ansible.posix/_build?definitionId=26)
[![Codecov](https://img.shields.io/codecov/c/github/ansible-collections/ansible.posix)](https://codecov.io/gh/ansible-collections/ansible.posix)

## Communication

* Join the Ansible forum:
  * [Get Help](https://forum.ansible.com/c/help/6): get help or help others.
  * [Social Spaces](https://forum.ansible.com/c/chat/4): gather and interact with fellow enthusiasts.
  * [News & Announcements](https://forum.ansible.com/c/news/5): track project-wide announcements including social events.

## Description

<!-- Describe the collection and why a user would want to use it. What does the collection do? -->
An Ansible Collection of modules and plugins that target POSIX UNIX/Linux and derivative Operating Systems.

## Requirements

* Python:
  * The Python interpreter version must meet Ansible Core's requirements.
* Ansible Core:
  - ansible-core 2.16 or later

## Installation

Before using this collection, you need to install it with the Ansible Galaxy command-line tool:

```shell
ansible-galaxy collection install ansible.posix
```

You can also include it in a requirements.yml file and install it with ansible-galaxy collection install -r requirements.yml, using the format:


```yaml
collections:
  - name: ansible.posix
```

Note that if you install any collections from Ansible Galaxy, they will not be upgraded automatically when you upgrade the Ansible package.
To upgrade the collection to the latest available version, run the following command:

```shell
ansible-galaxy collection install ansible.posix --upgrade
```

You can also install a specific version of the collection, for example, if you need to downgrade when something is broken in the latest version (please report an issue in this repository). Use the following syntax to install version 2.0.0:

```shell
ansible-galaxy collection install ansible.posix:==2.0.0
```

See [using Ansible collections](https://docs.ansible.com/ansible/devel/user_guide/collections_using.html) for more details.

* The Ansible [Bullhorn newsletter](https://docs.ansible.com/ansible/devel/community/communication.html#the-bullhorn): used to announce releases and important changes.

For more information about communication, see the [Ansible communication guide](https://docs.ansible.com/ansible/devel/community/communication.html).

## Use Cases

You can see the general use-cases as an example by `ansible-doc` command like below.

For example, ansible.posix.firewalld module:
```shell
ansible-doc ansible.posix.firewalld
```

Also, if you want to confirm the plugins descriptions, you can follow the following option with `ansible-doc` command:

For example, ansible.posix.profile_tasks callback plugin:
```shell
ansible-doc -t callback ansible.posix.profile_tasks
```

## Testing

The following ansible-core versions have been tested with this collection:

- ansible-core 2.22 (devel)
- ansible-core 2.21 (stable)
- ansible-core 2.20 (stable)
- ansible-core 2.19 (stable)
- ansible-core 2.18 (LTS)
- ansible-core 2.17 (EoL)
- ansible-core 2.16 (EoL)

For most up to date support info, see the [ansible-core support matrix](https://docs.ansible.com/projects/ansible/latest/reference_appendices/release_and_maintenance.html#ansible-core-support-matrix)

## Contributing

We welcome community contributions to this collection. For more details, see [Contributing to Ansible-maintained collections](https://docs.ansible.com/ansible/devel/community/contributing_maintained_collections.html#contributing-maintained-collections) for complete details.

* [Issues](https://github.com/ansible-collections/ansible.posix/issues)
* [Pull Requests](https://github.com/ansible-collections/ansible.posix/pulls)
* [Ansible Community Guide](https://docs.ansible.com/ansible/latest/community/index.html)


## Support

* Community users can find help through the [Communication](#Communication) section.
* Red Hat Ansible Automation Platform subscribers can request support through [Automation Hub](https://console.redhat.com/ansible/automation-hub/collections/published/ansible/posix/) by using the "Create issue" button on the collection page.

## Release Notes and Roadmap

See [changelog](https://github.com/ansible-collections/ansible.posix/blob/main/CHANGELOG.rst) for more details.

## Related Information

This document was written using the following [template](https://access.redhat.com/articles/7068606).

The README has been carefully prepared to cover the [community template](https://github.com/ansible-collections/collection_template/blob/main/README.md), but if you find any problems, please file a [documentation issue](https://github.com/ansible-collections/ansible.posix/issues/new?assignees=&labels=&projects=&template=documentation_report.md).

## License Information

GNU General Public License v3.0 or later.

See [COPYING](https://www.gnu.org/licenses/gpl-3.0.txt) to see the full text.
