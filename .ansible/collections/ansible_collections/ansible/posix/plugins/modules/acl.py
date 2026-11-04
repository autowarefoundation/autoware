#!/usr/bin/python
# -*- coding: utf-8 -*-

# Copyright: (c) 2017, Ansible Project
# GNU General Public License v3.0+ (see COPYING or https://www.gnu.org/licenses/gpl-3.0.txt)

from __future__ import absolute_import, division, print_function
__metaclass__ = type


DOCUMENTATION = r'''
---
module: acl
short_description: Set and retrieve file ACL information.
description:
- Set and retrieve file ACL information.
version_added: "1.0.0"
options:
  path:
    description:
    - The full path of the file or object.
    type: path
    required: true
    aliases: [ name ]
  state:
    description:
    - Define whether the ACL should be present or not.
    - The V(query) state gets the current ACL without changing it, for use in C(register) operations.
    choices: [ absent, present, query ]
    default: query
    type: str
  follow:
    description:
    - Whether to follow symlinks on the path if a symlink is encountered.
    type: bool
    default: true
  default:
    description:
    - If O(path) is a directory, setting this to V(true) will make it the default ACL for entities created inside the directory.
    - Setting O(default=true) causes an error if O(path) is a file.
    type: bool
    default: false
  entity:
    description:
    - The actual user or group that the ACL applies to when matching entity types user or group are selected.
    type: str
    default: ""
  etype:
    description:
    - The entity type of the ACL to apply, see C(setfacl) documentation for more info.
    choices: [ group, mask, other, user ]
    type: str
  permissions:
    description:
    - The permissions to apply/remove can be any combination of C(r), C(w), C(x)
      (read, write and execute respectively), and C(X) (execute permission if the file is a directory or already has execute permission for some user)
    type: str
  entry:
    description:
    - DEPRECATED.
    - The ACL to set or remove.
    - This must always be quoted in the form of C(<etype>:<qualifier>:<perms>).
    - The qualifier may be empty for some types, but the type and perms are always required.
    - C(-) can be used as placeholder when you do not care about permissions.
    - This is now superseded by entity, type and permissions fields.
    type: str
  recursive:
    description:
    - Recursively sets the specified ACL.
    - Incompatible with O(state=query).
    - Alias O(recurse) added in version 1.3.0.
    type: bool
    default: false
    aliases: [ recurse ]
  use_nfsv4_acls:
    description:
    - Use NFSv4 ACLs instead of POSIX ACLs.
    - This feature uses C(nfs4_setfacl) and C(nfs4_getfacl). The behavior depends on those implementation.
      And currently it only supports C(A) in ACE, so C(D) must be replaced with the appropriate C(A).
    - Permission is set as optimised ACLs by the system. You can check the actual ACLs that has been set using the return value.
    - More info C(man nfs4_setfacl)
    type: bool
    default: false
  recalculate_mask:
    description:
    - Select if and when to recalculate the effective right masks of the files.
    - See C(setfacl) documentation for more info.
    - Incompatible with O(state=query).
    choices: [ default, mask, no_mask ]
    default: default
    type: str
author:
- Brian Coca (@bcoca)
- Jérémie Astori (@astorije)
notes:
- The M(ansible.posix.acl) module requires that ACLs are enabled on the target filesystem and that the C(setfacl) and C(getfacl) binaries are installed.
- As of Ansible 2.0, this module only supports Linux distributions.
- As of Ansible 2.3, the O(name) option has been changed to O(path) as default, but O(name) still works as well.
'''

EXAMPLES = r'''
- name: Grant user Joe read access to a file
  ansible.posix.acl:
    path: /etc/foo.conf
    entity: joe
    etype: user
    permissions: r
    state: present

- name: Removes the ACL for Joe on a specific file
  ansible.posix.acl:
    path: /etc/foo.conf
    entity: joe
    etype: user
    state: absent

- name: Sets default ACL for joe on /etc/foo.d/
  ansible.posix.acl:
    path: /etc/foo.d/
    entity: joe
    etype: user
    permissions: rw
    default: true
    state: present

- name: Same as previous but using entry shorthand
  ansible.posix.acl:
    path: /etc/foo.d/
    entry: default:user:joe:rw-
    state: present

- name: Obtain the ACL for a specific file
  ansible.posix.acl:
    path: /etc/foo.conf
  register: acl_info
'''

RETURN = r'''
acl:
    description: Current ACL on provided path (after changes, if any)
    returned: success
    type: list
    sample: [ "user::rwx", "group::rwx", "other::rwx" ]
'''

import os
import platform

from ansible.module_utils.basic import AnsibleModule
from ansible.module_utils.common.text.converters import to_native


def split_entry(entry):
    ''' splits entry and ensures normalized return'''

    a = entry.split(':')

    d = None
    if entry.lower().startswith("d"):
        d = True
        a.pop(0)

    if len(a) == 2:
        a.append(None)

    t, e, p = a
    t = t.lower()

    if t.startswith("u"):
        t = "user"
    elif t.startswith("g"):
        t = "group"
    elif t.startswith("m"):
        t = "mask"
    elif t.startswith("o"):
        t = "other"
    else:
        t = None

    return [d, t, e, p]


def build_entry(etype, entity, permissions=None, use_nfsv4_acls=False):
    '''Builds and returns an entry string. Does not include the permissions bit if they are not provided.'''
    if use_nfsv4_acls:
        return ':'.join(['A', 'g' if etype == 'group' else '', entity, permissions + 'tcy'])

    if permissions:
        return etype + ':' + entity + ':' + permissions

    return etype + ':' + entity


def build_command(module, mode, path, follow, default, recursive, recalculate_mask, use_nfsv4_acls, entry=''):
    '''Builds and returns a getfacl/setfacl command.'''
    if mode == 'set':
        cmd = [module.get_bin_path('nfs4_setfacl' if use_nfsv4_acls else 'setfacl', True)]
        cmd.extend(['-a' if use_nfsv4_acls else '-m', entry])
    elif mode == 'rm':
        cmd = [module.get_bin_path('nfs4_setfacl' if use_nfsv4_acls else 'setfacl', True)]
        cmd.extend(['-x', entry])
    else:  # mode == 'get'
        cmd = [module.get_bin_path('getfacl', True)]
        # prevents absolute path warnings and removes headers
        if platform.system().lower() == 'linux':
            if use_nfsv4_acls:
                # use nfs4_getfacl instead of getfacl if use_nfsv4_acls is True
                cmd = [module.get_bin_path('nfs4_getfacl', True)]
            else:
                cmd = [module.get_bin_path('getfacl', True)]
                cmd.append('--absolute-names')
            cmd.append('--omit-header')

    if recursive and not use_nfsv4_acls:
        cmd.append('--recursive')

    if recalculate_mask == 'mask' and mode in ['set', 'rm']:
        cmd.append('--mask')
    elif recalculate_mask == 'no_mask' and mode in ['set', 'rm']:
        cmd.append('--no-mask')

    if not follow and not use_nfsv4_acls:
        if platform.system().lower() == 'linux':
            cmd.append('--physical')
        elif platform.system().lower() == 'freebsd':
            cmd.append('-h')

    if default:
        cmd.insert(1, '-d')

    cmd.append(path)
    return cmd


def acl_changed(module, cmd, entry, use_nfsv4_acls=False):
    '''Returns true if the provided command affects the existing ACLs, false otherwise.'''
    # To check the ACL changes, use the output of setfacl or nfs4_setfacl with '--test'.
    # FreeBSD do not have a --test flag, so by default, it is safer to always say "true".
    if platform.system().lower() == 'freebsd':
        return True

    cmd = cmd[:]  # lists are mutables so cmd would be overwritten without this
    cmd.insert(1, '--test')
    lines = run_acl(module, cmd)
    counter = 0
    for line in lines:
        if not use_nfsv4_acls and not line.endswith('*,*'):
            return True
        # if use_nfsv4_acls and entry is listed
        if use_nfsv4_acls and entry == line:
            counter += 1

    # The current 'nfs4_setfacl --test' lists a new entry,
    # which will be added at the top of the list, followed by the existing entries.
    # So if the entry has already been registered, the entry should be found twice.
    if not use_nfsv4_acls or counter == 2:
        return False
    return True


def run_acl(module, cmd, check_rc=True):
    '''Runs the provided command and returns the output as a list of lines.'''
    try:
        (rc, out, err) = module.run_command(cmd, check_rc=check_rc)
    except Exception as e:
        module.fail_json(msg=to_native(e))

    lines = []
    for l in out.splitlines():
        if not l.startswith('#'):
            lines.append(l.strip())

    if lines and not lines[-1].split():
        # trim last line only when it is empty
        return lines[:-1]

    return lines


def main():
    module = AnsibleModule(
        argument_spec=dict(
            path=dict(type='path', required=True, aliases=['name']),
            entry=dict(type='str'),
            entity=dict(type='str', default=''),
            etype=dict(
                type='str',
                choices=['group', 'mask', 'other', 'user'],
            ),
            permissions=dict(type='str'),
            state=dict(
                type='str',
                default='query',
                choices=['absent', 'present', 'query'],
            ),
            follow=dict(type='bool', default=True),
            default=dict(type='bool', default=False),
            recursive=dict(type='bool', default=False, aliases=['recurse']),
            recalculate_mask=dict(
                type='str',
                default='default',
                choices=['default', 'mask', 'no_mask'],
            ),
            use_nfsv4_acls=dict(type='bool', default=False)
        ),
        supports_check_mode=True,
    )

    if platform.system().lower() not in ['linux', 'freebsd']:
        module.fail_json(msg="The acl module is not available on this system.")

    path = module.params.get('path')
    entry = module.params.get('entry')
    entity = module.params.get('entity')
    etype = module.params.get('etype')
    permissions = module.params.get('permissions')
    state = module.params.get('state')
    follow = module.params.get('follow')
    default = module.params.get('default')
    recursive = module.params.get('recursive')
    recalculate_mask = module.params.get('recalculate_mask')
    use_nfsv4_acls = module.params.get('use_nfsv4_acls')

    if not os.path.exists(path):
        module.fail_json(msg="Path not found or not accessible.")

    if state == 'query':
        if recursive:
            module.fail_json(msg="'recursive' MUST NOT be set when 'state=query'.")

        if recalculate_mask in ['mask', 'no_mask']:
            module.fail_json(msg="'recalculate_mask' MUST NOT be set to 'mask' or 'no_mask' when 'state=query'.")

    if not entry:
        if state == 'absent' and permissions and not use_nfsv4_acls:
            module.fail_json(msg="'permissions' MUST NOT be set when 'state=absent'.")

        if state == 'absent' and not entity:
            module.fail_json(msg="'entity' MUST be set when 'state=absent'.")

        if state in ['present', 'absent'] and not etype:
            module.fail_json(msg="'etype' MUST be set when 'state=%s'." % state)

    if entry:
        if etype or entity or permissions:
            module.fail_json(msg="'entry' MUST NOT be set when 'entity', 'etype' or 'permissions' are set.")

        if state == 'present' and not entry.count(":") in [2, 3]:
            module.fail_json(msg="'entry' MUST have 3 or 4 sections divided by ':' when 'state=present'.")

        if state == 'absent' and not entry.count(":") in [1, 2]:
            module.fail_json(msg="'entry' MUST have 2 or 3 sections divided by ':' when 'state=absent'.")

        if state == 'query':
            module.fail_json(msg="'entry' MUST NOT be set when 'state=query'.")

        default_flag, etype, entity, permissions = split_entry(entry)
        if default_flag is not None:
            default = default_flag

    if platform.system().lower() == 'freebsd':
        if recursive:
            module.fail_json(msg="recursive is not supported on that platform.")

    changed = False
    msg = ""

    if state == 'present':
        entry = build_entry(etype, entity, permissions, use_nfsv4_acls)
        command = build_command(
            module, 'set', path, follow,
            default, recursive, recalculate_mask, use_nfsv4_acls, entry
        )
        changed = acl_changed(module, command, entry, use_nfsv4_acls)

        if changed and not module.check_mode:
            run_acl(module, command)
        msg = "%s is present" % entry

    elif state == 'absent':
        if use_nfsv4_acls:
            entry = build_entry(etype, entity, permissions, use_nfsv4_acls)
        else:
            entry = build_entry(etype, entity, use_nfsv4_acls)
        command = build_command(
            module, 'rm', path, follow,
            default, recursive, recalculate_mask, use_nfsv4_acls, entry
        )
        changed = acl_changed(module, command, entry, use_nfsv4_acls)

        if changed and not module.check_mode:
            run_acl(module, command, False)
        msg = "%s is absent" % entry

    elif state == 'query':
        msg = "current acl"

    acl = run_acl(
        module,
        build_command(
            module, 'get', path, follow, default, recursive,
            recalculate_mask, use_nfsv4_acls
        )
    )

    module.exit_json(changed=changed, msg=msg, acl=acl)


if __name__ == '__main__':
    main()
