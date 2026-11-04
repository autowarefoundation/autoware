#!/usr/bin/python
# -*- coding: utf-8 -*-

# Copyright: (c) 2012, Brad Olson <brado@movedbylight.com>
# GNU General Public License v3.0+ (see COPYING or https://www.gnu.org/licenses/gpl-3.0.txt)

from __future__ import absolute_import, division, print_function
__metaclass__ = type


DOCUMENTATION = r'''
---
module: authorized_key
short_description: Adds or removes an SSH authorized key
description:
    - Adds or removes SSH authorized keys for particular user accounts.
version_added: "1.0.0"
options:
  user:
    description:
      - The username on the remote host whose authorized_keys file will be modified.
    type: str
    required: true
  key:
    description:
      - The SSH public key(s), as a string or (since Ansible 1.9) url (https://github.com/username.keys).
      - You can also use V(file://) prefix to search remote for a file with SSH key(s).
    type: str
    required: true
  path:
    description:
      - Alternative path to the authorized_keys file.
      - The default value is the V(.ssh/authorized_keys) of the home of the user specified in the O(user) parameter.
      - Most of the time, it is not necessary to set this key.
      - Use the path to your target authorized_keys if you need to explicitly point on it.
    type: path
  manage_dir:
    description:
      - Whether this module should manage the directory of the authorized key file.
      - If set to V(true), the module will create the directory, as well as set the owner and permissions
        of an existing directory.
      - Be sure to set O(manage_dir=false) if you are using an alternate directory for authorized_keys,
        as set with O(path), since you could lock yourself out of SSH access.
      - See the example below.
    type: bool
    default: true
  state:
    description:
      - Whether the given key (with the given key_options) should or should not be in the file.
    type: str
    choices: [ absent, present ]
    default: present
  key_options:
    description:
      - A string of ssh key options to be prepended to the key in the authorized_keys file.
    type: str
  exclusive:
    description:
      - Whether to remove all other non-specified keys from the authorized_keys file.
      - Multiple keys can be specified in a single O(key) string value by separating them by newlines.
      - This option is not loop aware, so if you use C(with_) , it will be exclusive per iteration of the loop.
      - If you want multiple keys in the file you need to pass them all to O(key) in a single batch as mentioned above.
    type: bool
    default: false
  validate_certs:
    description:
      - This only applies if using a https url as the source of the keys.
      - If set to V(false), the SSL certificates will not be validated.
      - This should only set to V(false) used on personally controlled sites using self-signed certificates as it avoids verifying the source site.
      - Prior to 2.1 the code worked as if this was set to V(true).
    type: bool
    default: true
  comment:
    description:
      - Change the comment on the public key.
      - Rewriting the comment is useful in cases such as fetching it from GitHub or GitLab.
      - If no comment is specified, the existing comment will be kept.
    type: str
  follow:
    description:
      - Follow path symlink instead of replacing it.
    type: bool
    default: false
author: Ansible Core Team
'''

EXAMPLES = r'''
- name: Set authorized key taken from file
  ansible.posix.authorized_key:
    user: charlie
    state: present
    key: "{{ lookup('file', '/home/charlie/.ssh/id_rsa.pub') }}"

- name: Set authorized keys taken from url
  ansible.posix.authorized_key:
    user: charlie
    state: present
    key: https://github.com/charlie.keys

- name: Set authorized keys taken from path on controller node
  ansible.posix.authorized_key:
    user: charlie
    state: present
    key: file:///home/charlie/.ssh/id_rsa.pub

- name: Set authorized keys taken from url using lookup
  ansible.posix.authorized_key:
    user: charlie
    state: present
    key: "{{ lookup('url', 'https://github.com/charlie.keys', split_lines=False) }}"

- name: Set authorized key in alternate location
  ansible.posix.authorized_key:
    user: charlie
    state: present
    key: "{{ lookup('file', '/home/charlie/.ssh/id_rsa.pub') }}"
    path: /etc/ssh/authorized_keys/charlie
    manage_dir: false

- name: Set up multiple authorized keys
  ansible.posix.authorized_key:
    user: deploy
    state: present
    key: "{{ lookup('file', item) }}"
    loop:
      - public_keys/doe-jane
      - public_keys/doe-john

- name: Set authorized key defining key options
  ansible.posix.authorized_key:
    user: charlie
    state: present
    key: "{{ lookup('file', '/home/charlie/.ssh/id_rsa.pub') }}"
    key_options: 'no-port-forwarding,from="10.0.1.1"'

- name: Set authorized key without validating the TLS/SSL certificates
  ansible.posix.authorized_key:
    user: charlie
    state: present
    key: https://github.com/user.keys
    validate_certs: false

- name: Set authorized key, removing all the authorized keys already set
  ansible.posix.authorized_key:
    user: root
    key: "{{ lookup('file', 'public_keys/doe-jane') }}"
    state: present
    exclusive: true

- name: Set authorized key for user ubuntu copying it from current user
  ansible.posix.authorized_key:
    user: ubuntu
    state: present
    key: "{{ lookup('file', lookup('env','HOME') + '/.ssh/id_rsa.pub') }}"
'''

RETURN = r'''
exclusive:
  description: If the key has been forced to be exclusive or not.
  returned: success
  type: bool
  sample: false
key:
  description: The key that the module was running against.
  returned: success
  type: str
  sample: https://github.com/user.keys
key_option:
  description: Key options related to the key.
  returned: success
  type: str
  sample: null
keyfile:
  description: Path for authorized key file.
  returned: success
  type: str
  sample: /home/user/.ssh/authorized_keys
manage_dir:
  description: Whether this module managed the directory of the authorized key file.
  returned: success
  type: bool
  sample: true
path:
  description: Alternate path to the authorized_keys file
  returned: success
  type: str
  sample: null
state:
  description: Whether the given key (with the given key_options) should or should not be in the file
  returned: success
  type: str
  sample: present
unique:
  description: Whether the key is unique
  returned: success
  type: bool
  sample: false
user:
  description: The username on the remote host whose authorized_keys file will be modified
  returned: success
  type: str
  sample: user
validate_certs:
  description: This only applies if using a https url as the source of the keys. If set to C(false), the SSL certificates will not be validated.
  returned: success
  type: bool
  sample: true
'''

# Makes sure the public key line is present or absent in the user's .ssh/authorized_keys.
#
# Arguments
# =========
#    user = username
#    key = line to add to authorized_keys for user
#    path = path to the user's authorized_keys file (default: ~/.ssh/authorized_keys)
#    manage_dir = whether to create, and control ownership of the directory (default: true)
#    state = absent|present (default: present)
#
# see example in examples/playbooks

import os
import pwd
import os.path
import tempfile
import re
import shlex
import errno
import traceback
from operator import itemgetter

# TODO(Python2): urllib.parse is available in Python 3. This module may run on
# target hosts with Python 2.7 (e.g., older RHEL systems in CI integration tests).
# Remove the try/except fallback to urlparse when Python 2 support is dropped.
try:
    from urllib.parse import urlparse
except ImportError:
    from urlparse import urlparse

from ansible.module_utils.basic import AnsibleModule
from ansible.module_utils.common.text.converters import to_native
from ansible.module_utils.urls import fetch_url


class keydict(dict):

    """ a dictionary that maintains the order of keys as they are added

    This has become an abuse of the dict interface.  Probably should be
    rewritten to be an entirely custom object with methods instead of
    bracket-notation.

    Our requirements are for a data structure that:
    * Preserves insertion order
    * Can store multiple values for a single key.

    The present implementation has the following functions used by the rest of
    the code:

    * __setitem__(): to add a key=value.  The value can never be disassociated
      with the key, only new values can be added in addition.
    * items(): to retrieve the key, value pairs.

    Other dict methods should work but may be surprising.  For instance, there
    will be multiple keys that are the same in keys() and __getitem__() will
    return a list of the values that have been set via __setitem__.
    """

    # http://stackoverflow.com/questions/2328235/pythonextend-the-dict-class

    def __init__(self, *args, **kw):
        super(keydict, self).__init__(*args, **kw)
        self.itemlist = list(super(keydict, self).keys())

    def __setitem__(self, key, value):
        self.itemlist.append(key)
        if key in self:
            self[key].append(value)
        else:
            super(keydict, self).__setitem__(key, [value])

    def __iter__(self):
        return iter(self.itemlist)

    def keys(self):
        return self.itemlist

    def _item_generator(self):
        indexes = {}
        for key in self.itemlist:
            if key in indexes:
                indexes[key] += 1
            else:
                indexes[key] = 0
            yield key, self[key][indexes[key]]

    def iteritems(self):
        raise NotImplementedError("Do not use this as it's not available on py3")

    def items(self):
        return list(self._item_generator())

    def itervalues(self):
        raise NotImplementedError("Do not use this as it's not available on py3")

    def values(self):
        return [item[1] for item in self.items()]


def _safe_open_write(module, path, follow):
    flags = os.O_WRONLY | os.O_CREAT | os.O_TRUNC
    if not follow and hasattr(os, 'O_NOFOLLOW'):
        flags |= os.O_NOFOLLOW
    try:
        fd = os.open(path, flags, int('0600', 8))
    except OSError as e:
        module.fail_json(msg="File open failed %s : %s" % (path, to_native(e)))
    return fd


def keyfile(module, user, write=False, path=None, manage_dir=True, follow=False):
    """
    Calculate name of authorized keys file, optionally creating the
    directories and file, properly setting permissions.

    :param str user: name of user in passwd file
    :param bool write: if True, write changes to authorized_keys file (creating directories if needed)
    :param str path: if not None, use provided path rather than default of '~user/.ssh/authorized_keys'
    :param bool manage_dir: if True, create and set ownership of the parent dir of the authorized_keys file
    :param bool follow: if True symlinks will be followed and not replaced
    :return: full path string to authorized_keys for user
    """

    if module.check_mode and path is not None:
        keysfile = path

        if follow:
            return os.path.realpath(keysfile)

        return keysfile

    try:
        user_entry = pwd.getpwnam(user)
    except KeyError as e:
        if module.check_mode and path is None:
            module.fail_json(msg="Either user must exist or you must provide full path to key file in check mode")
        module.fail_json(msg="Failed to lookup user %s: %s" % (user, to_native(e)))
    if path is None:
        homedir = user_entry.pw_dir
        sshdir = os.path.join(homedir, ".ssh")
        keysfile = os.path.join(sshdir, "authorized_keys")
    else:
        sshdir = os.path.dirname(path)
        keysfile = path

    if follow:
        keysfile = os.path.realpath(keysfile)

    if not write or module.check_mode:
        return keysfile

    uid = user_entry.pw_uid
    gid = user_entry.pw_gid

    if manage_dir:
        if not os.path.exists(sshdir):
            try:
                os.mkdir(sshdir, int('0700', 8))
            except OSError as e:
                module.fail_json(msg="Failed to create directory %s : %s" % (sshdir, to_native(e)))
            if module.selinux_enabled():
                module.set_default_selinux_context(sshdir, False)
        os.chown(sshdir, uid, gid, follow_symlinks=follow)
        os.chmod(sshdir, int('0700', 8))

    if not os.path.exists(keysfile):
        basedir = os.path.dirname(keysfile)
        if not os.path.exists(basedir):
            os.makedirs(basedir)

        fd = _safe_open_write(module, keysfile, follow)
        os.close(fd)
        if module.selinux_enabled():
            module.set_default_selinux_context(keysfile, False)

    try:
        os.chown(keysfile, uid, gid, follow_symlinks=follow)
        os.chmod(keysfile, int('0600', 8))
    except OSError:
        pass

    return keysfile


def parseoptions(module, options):
    '''
    reads a string containing ssh-key options
    and returns a dictionary of those options
    '''
    options_dict = keydict()  # ordered dict
    if options:
        # the following regex will split on commas while
        # ignoring those commas that fall within quotes
        regex = re.compile(r'''((?:[^,"']|"[^"]*"|'[^']*')+)''')
        parts = regex.split(options)[1:-1]
        for part in parts:
            if "=" in part:
                (key, value) = part.split("=", 1)
                options_dict[key] = value
            elif part != ",":
                options_dict[part] = None

    return options_dict


def parsekey(module, raw_key, rank=None):
    '''
    parses a key, which may or may not contain a list
    of ssh-key options at the beginning

    rank indicates the keys original ordering, so that
    it can be written out in the same order.
    '''

    VALID_SSH2_KEY_TYPES = [
        'sk-ecdsa-sha2-nistp256@openssh.com',
        'sk-ecdsa-sha2-nistp256-cert-v01@openssh.com',
        'webauthn-sk-ecdsa-sha2-nistp256@openssh.com',
        'ecdsa-sha2-nistp256',
        'ecdsa-sha2-nistp256-cert-v01@openssh.com',
        'ecdsa-sha2-nistp384',
        'ecdsa-sha2-nistp384-cert-v01@openssh.com',
        'ecdsa-sha2-nistp521',
        'ecdsa-sha2-nistp521-cert-v01@openssh.com',
        'sk-ssh-ed25519@openssh.com',
        'sk-ssh-ed25519-cert-v01@openssh.com',
        'ssh-ed25519',
        'ssh-ed25519-cert-v01@openssh.com',
        'ssh-dss',
        'ssh-rsa',
        'ssh-xmss@openssh.com',
        'ssh-xmss-cert-v01@openssh.com',
        'rsa-sha2-256',
        'rsa-sha2-512',
        'ssh-rsa-cert-v01@openssh.com',
        'rsa-sha2-256-cert-v01@openssh.com',
        'rsa-sha2-512-cert-v01@openssh.com',
        'ssh-dss-cert-v01@openssh.com',
    ]

    options = None   # connection options
    key = None   # encrypted key string
    key_type = None   # type of ssh key
    type_index = None   # index of keytype in key string|list

    # remove comment yaml escapes
    raw_key = raw_key.replace(r'\#', '#')

    # split key safely
    lex = shlex.shlex(raw_key)
    lex.quotes = []
    lex.commenters = ''  # keep comment hashes
    lex.whitespace_split = True
    key_parts = list(lex)

    if key_parts and key_parts[0] == '#':
        # comment line, invalid line, etc.
        return (raw_key, 'skipped', None, None, rank)

    for i in range(0, len(key_parts)):
        if key_parts[i] in VALID_SSH2_KEY_TYPES:
            type_index = i
            key_type = key_parts[i]
            break

    # check for options
    if type_index is None:
        return None
    elif type_index > 0:
        options = " ".join(key_parts[:type_index])

    # parse the options (if any)
    options = parseoptions(module, options)

    # get key after the type index
    key = key_parts[(type_index + 1)]

    # set comment to everything after the key
    if len(key_parts) > (type_index + 1):
        comment = " ".join(key_parts[(type_index + 2):])

    return (key, key_type, options, comment, rank)


def readfile(module, filename):
    try:
        with open(filename, 'r') as f:
            return f.read()
    except IOError as e:
        if e.errno == errno.EACCES:
            module.fail_json(msg="Permission denied on file or path for authorized keys file: %s" % filename,
                             exception=traceback.format_exc())
        elif e.errno == errno.ENOENT:
            return ''
        else:
            raise


def parsekeys(module, lines):
    keys = {}
    for rank_index, line in enumerate(lines.splitlines(True)):
        key_data = parsekey(module, line, rank=rank_index)
        if key_data:
            # use key as identifier
            keys[key_data[0]] = key_data
        else:
            # for an invalid line, just set the line
            # dict key to the line so it will be re-output later
            keys[line] = (line, 'skipped', None, None, rank_index)
    return keys


def writefile(module, filename, content):
    dummy, tmp_path = tempfile.mkstemp()

    try:
        with open(tmp_path, "w") as f:
            f.write(content)
    except IOError as e:
        module.add_cleanup_file(tmp_path)
        module.fail_json(msg="Failed to write to file %s: %s" % (tmp_path, to_native(e)))
    module.atomic_move(tmp_path, filename)


def serialize(keys):
    lines = []
    new_keys = keys.values()
    # order the new_keys by their original ordering, via the rank item in the tuple
    ordered_new_keys = sorted(new_keys, key=itemgetter(4))

    for key in ordered_new_keys:
        try:
            (keyhash, key_type, options, comment, rank) = key

            option_str = ""
            if options:
                option_strings = []
                for option_key, value in options.items():
                    if value is None:
                        option_strings.append("%s" % option_key)
                    else:
                        option_strings.append("%s=%s" % (option_key, value))
                option_str = ",".join(option_strings)
                option_str += " "

            # comment line or invalid line, just leave it
            if not key_type:
                key_line = key

            if key_type == 'skipped':
                key_line = key[0]
            else:
                key_line = "%s%s %s %s\n" % (option_str, key_type, keyhash, comment)
        except Exception:
            key_line = key
        lines.append(key_line)
    return ''.join(lines)


def enforce_state(module, params):
    """
    Add or remove key.
    """

    user = params["user"]
    key = params["key"]
    path = params.get("path", None)
    manage_dir = params.get("manage_dir", True)
    state = params.get("state", "present")
    key_options = params.get("key_options", None)
    exclusive = params.get("exclusive", False)
    comment = params.get("comment", None)
    follow = params.get('follow', False)
    error_msg = "Error getting key from: %s"

    # if the key is a url or file, request it and use it as key source
    if key.startswith("http"):
        try:
            resp, info = fetch_url(module, key)
            if info['status'] != 200:
                module.fail_json(msg=error_msg % key)
            else:
                key = resp.read()
        except Exception:
            module.fail_json(msg=error_msg % key)

        # resp.read gives bytes on python3, convert to native string type
        key = to_native(key, errors='surrogate_or_strict')

    if key.startswith("file"):
        # if the key is an absolute path, check for existense and use it as a key source
        key_path = urlparse(key).path
        if not os.path.exists(key_path):
            module.fail_json(msg="Path to a key file not found: %s" % key_path)
        if not os.path.isfile(key_path):
            module.fail_json(msg="Path to a key is a directory and must be a file: %s" % key_path)
        try:
            with open(key_path, 'r') as source_fh:
                key = source_fh.read()
        except OSError as e:
            module.fail_json(msg="Failed to read key file %s : %s" % (key_path, to_native(e)))

    # extract individual keys into an array, skipping blank lines and comments
    new_keys = [s for s in key.splitlines() if s and not s.startswith('#')]

    # check current state -- just get the filename, don't create file
    do_write = False
    params["keyfile"] = keyfile(module, user, do_write, path, manage_dir, follow)
    existing_content = readfile(module, params["keyfile"])
    existing_keys = parsekeys(module, existing_content)

    # Add a place holder for keys that should exist in the state=present and
    # exclusive=true case
    keys_to_exist = []

    # we will order any non exclusive new keys higher than all the existing keys,
    # resulting in the new keys being written to the key file after existing keys, but
    # in the order of new_keys
    max_rank_of_existing_keys = len(existing_keys)

    # Check our new keys, if any of them exist we'll continue.
    for rank_index, new_key in enumerate(new_keys):
        parsed_new_key = parsekey(module, new_key, rank=rank_index)

        if not parsed_new_key:
            module.fail_json(msg="invalid key specified: %s" % new_key)

        if key_options is not None:
            parsed_options = parseoptions(module, key_options)
            # rank here is the rank in the provided new keys, which may be unrelated to rank in existing_keys
            parsed_new_key = (parsed_new_key[0], parsed_new_key[1], parsed_options, parsed_new_key[3], parsed_new_key[4])

        if comment is not None:
            parsed_new_key = (parsed_new_key[0], parsed_new_key[1], parsed_new_key[2], comment, parsed_new_key[4])

        matched = False
        non_matching_keys = []

        if parsed_new_key[0] in existing_keys:
            # Then we check if everything (except the rank at index 4) matches, including
            # the key type and options. If not, we append this
            # existing key to the non-matching list
            # We only want it to match everything when the state
            # is present
            if parsed_new_key[:4] != existing_keys[parsed_new_key[0]][:4] and state == "present":
                non_matching_keys.append(existing_keys[parsed_new_key[0]])
            else:
                matched = True

        # handle idempotent state=present
        if state == "present":
            keys_to_exist.append(parsed_new_key[0])
            if len(non_matching_keys) > 0:
                for non_matching_key in non_matching_keys:
                    if non_matching_key[0] in existing_keys:
                        del existing_keys[non_matching_key[0]]
                        do_write = True

            # new key that didn't exist before. Where should it go in the ordering?
            if not matched:
                # We want the new key to be after existing keys if not exclusive (rank > max_rank_of_existing_keys)
                total_rank = max_rank_of_existing_keys + parsed_new_key[4]
                # replace existing key tuple with new parsed key with its total rank
                existing_keys[parsed_new_key[0]] = (parsed_new_key[0], parsed_new_key[1], parsed_new_key[2], parsed_new_key[3], total_rank)
                do_write = True

        elif state == "absent":
            if not matched:
                continue
            del existing_keys[parsed_new_key[0]]
            do_write = True

    # remove all other keys to honor exclusive
    # for 'exclusive', make sure keys are written in the order the new keys were
    if state == "present" and exclusive:
        to_remove = frozenset(existing_keys).difference(keys_to_exist)
        for key in to_remove:
            del existing_keys[key]
            do_write = True

    if do_write:
        filename = keyfile(module, user, do_write, path, manage_dir, follow)
        new_content = serialize(existing_keys)

        diff = None
        if module._diff:
            diff = {
                'before_header': params['keyfile'],
                'after_header': filename,
                'before': existing_content,
                'after': new_content,
            }
            params['diff'] = diff

        if not module.check_mode:
            writefile(module, filename, new_content)
            user_entry = pwd.getpwnam(user)
            uid = user_entry.pw_uid
            gid = user_entry.pw_gid
            os.chown(filename, uid, gid, follow_symlinks=follow)
            os.chmod(filename, int('0600', 8))
        params['changed'] = True

    return params


def main():
    module = AnsibleModule(
        argument_spec=dict(
            user=dict(type='str', required=True),
            key=dict(type='str', required=True, no_log=False),
            path=dict(type='path'),
            manage_dir=dict(type='bool', default=True),
            state=dict(type='str', default='present', choices=['absent', 'present']),
            key_options=dict(type='str', no_log=False),
            exclusive=dict(type='bool', default=False),
            comment=dict(type='str'),
            validate_certs=dict(type='bool', default=True),
            follow=dict(type='bool', default=False),
        ),
        supports_check_mode=True,
    )

    results = enforce_state(module, module.params)
    module.exit_json(**results)


if __name__ == '__main__':
    main()
