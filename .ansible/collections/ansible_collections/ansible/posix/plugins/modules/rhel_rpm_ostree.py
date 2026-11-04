#!/usr/bin/python
# -*- coding: utf-8 -*-

# Copyright: Red Hat Inc.
# GNU General Public License v3.0+ (see COPYING or https://www.gnu.org/licenses/gpl-3.0.txt)

from __future__ import absolute_import, division, print_function
__metaclass__ = type


DOCUMENTATION = '''
---
module: rhel_rpm_ostree
version_added: 1.5.0
short_description: Ensure packages exist in a RHEL for Edge rpm-ostree based system
description:
  - Compatibility layer for using the "package" module for RHEL for Edge systems utilizing the RHEL System Roles.
author:
  - Adam Miller (@maxamillion)
requirements:
  - rpm-ostree
options:
  name:
    description:
      - A package name or package specifier with version, like V(name-1.0).
      - Comparison operators for package version are valid here C(>), C(<), C(>=), C(<=). Example - C(name>=1.0).
      - If a previous version is specified, the task also needs to turn C(allow_downgrade) on.
        See the C(allow_downgrade) documentation for caveats with downgrading packages.
      - When using O(state=latest), this can be V('*') which means run C(yum -y update).
      - You can also pass a url or a local path to a rpm file (using O(state=present)).
        To operate on several packages this can accept a comma separated string of packages or (as of 2.0) a list of packages.
    aliases: [ pkg ]
    type: list
    elements: str
    default: []
  state:
    description:
      - Whether to install (V(present) or V(installed), V(latest)), or remove (V(absent) or V(removed)) a package.
      - V(present) and V(installed) will simply ensure that a desired package is installed.
      - V(latest) will update the specified package if it's not of the latest available version.
      - V(absent) and V(removed) will remove the specified package.
      - Default is C(null), however in effect the default action is V(present) unless the C(autoremove) option is
        enabled for this module, then V(absent) is inferred.
    type: str
    choices: [ absent, installed, latest, present, removed ]
notes:
  - This module does not support installing or removing packages to/from an overlay as this is not supported
    by RHEL for Edge, packages needed should be defined in the osbuild Blueprint and provided to Image Builder
    at build time. This module exists only for C(package) module compatibility.
'''

EXAMPLES = '''
- name: Ensure htop and ansible are installed on rpm-ostree based RHEL
  ansible.posix.rhel_rpm_ostree:
    name:
      - htop
      - ansible
    state: present
'''

RETURN = """
msg:
    description: status of rpm transaction
    returned: always
    type: str
    sample: "No changes made."
"""

import os
import traceback

from ansible.module_utils.basic import AnsibleModule
from ansible.module_utils.common.text.converters import to_text


def locally_installed(module, pkgname):
    (rc, out, err) = module.run_command('{0} -q {1}'.format(module.get_bin_path("rpm"), pkgname).split())
    return (rc == 0)


def rpm_ostree_transaction(module):
    pkgs = []

    if module.params['state'] in ['present', 'installed', 'latest']:
        for pkg in module.params['name']:
            if not locally_installed(module, pkg):
                pkgs.append(pkg)
    elif module.params['state'] in ['absent', 'removed']:
        for pkg in module.params['name']:
            if locally_installed(module, pkg):
                pkgs.append(pkg)

    if not pkgs:
        module.exit_json(msg="No changes made.")
    else:
        if module.params['state'] in ['present', 'installed', 'latest']:
            module.fail_json(msg="The following packages are absent in the currently booted rpm-ostree commit: %s" ' '.join(pkgs))
        else:
            module.fail_json(msg="The following packages are present in the currently booted rpm-ostree commit: %s" ' '.join(pkgs))


def main():
    module = AnsibleModule(
        argument_spec=dict(
            name=dict(type='list', elements='str', aliases=['pkg'], default=[]),
            state=dict(type='str', default=None, choices=['absent', 'installed', 'latest', 'present', 'removed']),
        ),
    )

    # Verify that the platform is an rpm-ostree based system
    if not os.path.exists("/run/ostree-booted"):
        module.fail_json(msg="Module rpm_ostree is only applicable for rpm-ostree based systems.")

    try:
        rpm_ostree_transaction(module)
    except Exception as e:
        module.fail_json(msg=to_text(e), exception=traceback.format_exc())


if __name__ == '__main__':
    main()
