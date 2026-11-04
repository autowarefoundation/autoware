#!/usr/bin/python
# -*- coding: utf-8 -*-

# Copyright: Red Hat Inc.
# GNU General Public License v3.0+ (see COPYING or https://www.gnu.org/licenses/gpl-3.0.txt)

from __future__ import absolute_import, division, print_function
__metaclass__ = type

DOCUMENTATION = '''
---
module: rhel_facts
version_added: 1.5.0
short_description: Facts module to set or override RHEL specific facts.
description:
  - Compatibility layer for using the M(ansible.builtin.package) module for rpm-ostree based systems via setting the C(pkg_mgr) fact correctly.
author:
  - Adam Miller (@maxamillion)
requirements:
  - rpm-ostree
seealso:
  - module: ansible.builtin.package
options: {}
'''

EXAMPLES = '''
- name: Playbook to use the package module on all RHEL footprints
  vars:
    ansible_facts_modules:
      - setup # REQUIRED to be run before all custom fact modules
      - ansible.posix.rhel_facts
  tasks:
    - name: Ensure packages are installed
      ansible.builtin.package:
        name:
          - htop
          - ansible
        state: present
'''

RETURN = """
ansible_facts:
    description: Relevant Ansible Facts
    returned: when needed
    type: complex
    contains:
        pkg_mgr:
            description: System-level package manager override
            returned: when needed
            type: str
            sample: {'pkg_mgr': 'ansible.posix.rhel_facts'}
"""

import os

from ansible.module_utils.basic import AnsibleModule


def main():

    module = AnsibleModule(
        argument_spec=dict(),
        supports_check_mode=True,
    )

    ansible_facts = {}

    # Verify that the platform is an rpm-ostree based system
    if os.path.exists("/run/ostree-booted"):
        ansible_facts['pkg_mgr'] = 'ansible.posix.rhel_rpm_ostree'

    module.exit_json(ansible_facts=ansible_facts, changed=False)


if __name__ == '__main__':
    main()
