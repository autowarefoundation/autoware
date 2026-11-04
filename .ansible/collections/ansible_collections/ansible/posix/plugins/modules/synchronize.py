#!/usr/bin/python
# -*- coding: utf-8 -*-

# Copyright: (c) 2012-2013, Timothy Appnel <tim@appnel.com>
# Copyright: (c) 2017, Ansible Project
# GNU General Public License v3.0+ (see COPYING or https://www.gnu.org/licenses/gpl-3.0.txt)

from __future__ import absolute_import, division, print_function
__metaclass__ = type


DOCUMENTATION = r'''
---
module: synchronize
short_description: A wrapper around rsync to make common tasks in your playbooks quick and easy
description:
    - M(ansible.posix.synchronize) is a wrapper around C(rsync) to make common tasks in your playbooks quick and easy.
    - It is run and originates on the local host where Ansible is being run.
    - Of course, you could just use the M(ansible.builtin.command) action to call C(rsync) yourself, but you also have to add a fair number of
      boilerplate options and host facts.
    - This module is not intended to provide access to the full power of C(rsync), but does make the most common
      invocations easier to implement.
      You I(still) may need to call C(rsync) directly via M(ansible.builtin.command) or M(ansible.builtin.shell) depending on your use case.
version_added: "1.0.0"
options:
  src:
    description:
      - Path on the source host that will be synchronized to the destination.
      - The path can be absolute or relative.
    type: path
    required: true
  dest:
    description:
      - Path on the destination host that will be synchronized from the source.
      - The path can be absolute or relative.
    type: path
    required: true
  dest_port:
    description:
      - Port number for ssh on the destination host.
      - Prior to Ansible 2.0, the C(ansible_ssh_port) inventory var took precedence over this value.
      - This parameter defaults to the value of C(ansible_port), the C(remote_port) config setting
        or the value from ssh client configuration if none of the former have been set.
    type: int
  mode:
    description:
      - Specify the direction of the synchronization.
      - In V(push) mode the localhost or delegate is the source.
      - In V(pull) mode the remote host in context is the source.
    type: str
    choices: [ pull, push ]
    default: push
  archive:
    description:
      - Mirrors the rsync archive flag, enables recursive, links, perms, times, owner, group flags, and C(-D).
    type: bool
    default: true
  checksum:
    description:
      - Skip based on checksum, rather than mod-time & size; Note that that O(archive) option is still enabled by default -
        the O(checksum) option will not disable it.
    type: bool
    default: false
  compress:
    description:
      - Compress file data during the transfer.
      - In most cases, leave this enabled unless it causes problems.
    type: bool
    default: true
  existing_only:
    description:
      - Skip creating new files on receiver.
    type: bool
    default: false
  delete:
    description:
      - Delete files in O(dest) that do not exist (after transfer, not before) in the O(src) path.
      - This option requires O(recursive=true).
      - This option ignores excluded files and behaves like the rsync opt C(--delete-after).
    type: bool
    default: false
  dirs:
    description:
      - Transfer directories without recursing.
    type: bool
    default: false
  recursive:
    description:
      - Recurse into directories.
      - This parameter defaults to the value of the archive option.
    type: bool
  links:
    description:
      - Copy symlinks as symlinks.
      - This parameter defaults to the value of the archive option.
    type: bool
  copy_links:
    description:
      - Copy symlinks as the item that they point to (the referent) is copied, rather than the symlink.
    type: bool
    default: false
  perms:
    description:
      - Preserve permissions.
      - This parameter defaults to the value of the archive option.
    type: bool
  times:
    description:
      - Preserve modification times.
      - This parameter defaults to the value of the archive option.
    type: bool
  owner:
    description:
      - Preserve owner (super user only).
      - This parameter defaults to the value of the archive option.
    type: bool
  group:
    description:
      - Preserve group.
      - This parameter defaults to the value of the archive option.
    type: bool
  rsync_path:
    description:
      - Specify the rsync command to run on the remote host. See C(--rsync-path) on the rsync man page.
      - To specify the rsync command to run on the local host, you need to set this your task var C(ansible_rsync_path).
    type: str
  rsync_timeout:
    description:
      - Specify a C(--timeout) for the rsync command in seconds.
    type: int
    default: 0
  set_remote_user:
    description:
      - Put C(user@) for the remote paths.
      - If you have a custom ssh config to define the remote user for a host
        that does not match the inventory user, you should set this parameter to V(false).
    type: bool
    default: true
  ssh_connection_multiplexing:
    description:
      - SSH connection multiplexing for rsync is disabled by default to prevent misconfigured ControlSockets from resulting in failed SSH connections.
        This is accomplished by setting the SSH C(ControlSocket) to C(none).
      - Set this option to V(true) to allow multiplexing and reduce SSH connection overhead.
      - Note that simply setting this option to V(true) is not enough;
        You must also configure SSH connection multiplexing in your SSH client config by setting values for
        C(ControlMaster), C(ControlPersist) and C(ControlPath).
    type: bool
    default: false
  rsync_opts:
    description:
      - Specify additional rsync options by passing in an array.
      - Note that an empty string in C(rsync_opts) will end up transfer the current working directory.
    type: list
    default: []
    elements: str
  partial:
    description:
      - Tells rsync to keep the partial file which should make a subsequent transfer of the rest of the file much faster.
    type: bool
    default: false
  verify_host:
    description:
      - Verify destination host key.
    type: bool
    default: false
  private_key:
    description:
      - Specify the private key to use for SSH-based rsync connections (e.g. C(~/.ssh/id_rsa)).
    type: path
  link_dest:
    description:
      - Add a destination to hard link against during the rsync.
    type: list
    default:
    elements: path
  delay_updates:
    description:
      - This option puts the temporary file from each updated file into a holding directory until the end of the transfer,
        at which time all the files are renamed into place in rapid succession.
    type: bool
    default: true
    version_added: '1.3.0'
  use_ssh_args:
    description:
      - In Ansible 2.10 and lower, it uses the ssh_args specified in C(ansible.cfg).
      - In Ansible 2.11 and onwards, when set to V(true), it uses all SSH connection configurations like
        C(ansible_ssh_args), C(ansible_ssh_common_args), and C(ansible_ssh_extra_args).
    type: bool
    default: false
  _local_rsync_path:
    description: Internal use only.
    type: path
    default: 'rsync'
    required: false
  _local_rsync_password:
    description: Internal use only, never logged.
    type: str
    required: false
  _substitute_controller:
    description: Internal use only.
    type: bool
    default: false
  _ssh_args:
    description: Internal use only. See O(use_ssh_args) for ssh arg settings.
    type: str
    required: false

notes:
   - C(rsync) must be installed on both the local and remote host.
   - For the M(ansible.posix.synchronize) module, the "local host" is the host I(the synchronize task originates on),
     and the "destination host" is the host I(synchronize is connecting to).
   - The "local host" can be changed to a different host by using C(delegate_to).
     This enables copying between two remote hosts or entirely on one remote machine.
   - >
     The user and permissions for the synchronize O(src) are those of the user running the Ansible task on the local host (or the remote_user for a
     C(delegate_to) host when C(delegate_to) is used).
   - The user and permissions for the synchronize `dest` are those of the `remote_user` on the destination host or the `become_user` if `become=yes` is active.
   - In Ansible 2.0 a bug in the synchronize module made become occur on the "local host".  This was fixed in Ansible 2.0.1.
   - Currently, M(ansible.posix.synchronize) is limited to elevating permissions via passwordless sudo.
     This is because rsync itself is connecting to the remote machine and rsync doesn't give us a way to pass sudo credentials in.
   - Currently there are only a few connection types which support synchronize (ssh, paramiko, local, and docker) because a sync strategy has been
     determined for those connection types.  Note that the connection for these must not need a password as rsync itself is making the connection and
     rsync does not provide us a way to pass a password to the connection.
   - Expect that O(dest=~/x) will be V(~<remote_user>/x) even if using sudo.
   - Inspect the verbose output to validate the destination user/host/path are what was expected.
   - To exclude files and directories from being synchronized, you may add C(.rsync-filter) files to the source directory.
   - rsync daemon must be up and running with correct permission when using rsync protocol in source or destination path.
   - The C(synchronize) module enables C(--delay-updates) by default to avoid leaving a destination in a broken in-between state if the underlying rsync process
     encounters an error. Those synchronizing large numbers of files that are willing to trade safety for performance should disable this option.
   - link_destination is subject to the same limitations as the underlying rsync daemon. Hard links are only preserved if the relative subtrees
     of the source and destination are the same. Attempts to hardlink into a directory that is a subdirectory of the source will be prevented.
seealso:
- module: ansible.builtin.copy
- module: community.windows.win_robocopy
author:
- Timothy Appnel (@tima)
'''

EXAMPLES = r'''
- name: Synchronization of src on the control machine to dest on the remote hosts
  ansible.posix.synchronize:
    src: some/relative/path
    dest: /some/absolute/path

- name: Synchronization using rsync protocol (push)
  ansible.posix.synchronize:
    src: some/relative/path/
    dest: rsync://somehost.com/path/

- name: Synchronization using rsync protocol (pull)
  ansible.posix.synchronize:
    mode: pull
    src: rsync://somehost.com/path/
    dest: /some/absolute/path/

- name: Synchronization using rsync protocol on delegate host (push)
  ansible.posix.synchronize:
    src: /some/absolute/path/
    dest: rsync://somehost.com/path/
  delegate_to: delegate.host

- name: Synchronization using rsync protocol on delegate host (pull)
  ansible.posix.synchronize:
    mode: pull
    src: rsync://somehost.com/path/
    dest: /some/absolute/path/
  delegate_to: delegate.host

- name: Synchronization without any --archive options enabled
  ansible.posix.synchronize:
    src: some/relative/path
    dest: /some/absolute/path
    archive: false

- name: Synchronization with --archive options enabled except for --recursive
  ansible.posix.synchronize:
    src: some/relative/path
    dest: /some/absolute/path
    recursive: false

- name: Synchronization with --archive options enabled except for --times, with --checksum option enabled
  ansible.posix.synchronize:
    src: some/relative/path
    dest: /some/absolute/path
    checksum: true
    times: false

- name: Synchronization without --archive options enabled except use --links
  ansible.posix.synchronize:
    src: some/relative/path
    dest: /some/absolute/path
    archive: false
    links: true

- name: Synchronization of two paths both on the control machine
  ansible.posix.synchronize:
    src: some/relative/path
    dest: /some/absolute/path
  delegate_to: localhost

- name: Synchronization of src on the inventory host to the dest on the localhost in pull mode
  ansible.posix.synchronize:
    mode: pull
    src: some/relative/path
    dest: /some/absolute/path

- name: Synchronization of src on delegate host to dest on the current inventory host.
  ansible.posix.synchronize:
    src: /first/absolute/path
    dest: /second/absolute/path
  delegate_to: delegate.host

- name: Synchronize two directories on one remote host.
  ansible.posix.synchronize:
    src: /first/absolute/path
    dest: /second/absolute/path
  delegate_to: "{{ inventory_hostname }}"

- name: Synchronize and delete files in dest on the remote host that are not found in src of localhost.
  ansible.posix.synchronize:
    src: some/relative/path
    dest: /some/absolute/path
    delete: true
    recursive: true

# This specific command is granted su privileges on the destination
- name: Synchronize using an alternate rsync command
  ansible.posix.synchronize:
    src: some/relative/path
    dest: /some/absolute/path
    rsync_path: su -c rsync

# Example .rsync-filter file in the source directory
# - var       # exclude any path whose last part is 'var'
# - /var      # exclude any path starting with 'var' starting at the source directory
# + /var/conf # include /var/conf even though it was previously excluded

- name: Synchronize passing in extra rsync options
  ansible.posix.synchronize:
    src: /tmp/helloworld
    dest: /var/www/helloworld
    rsync_opts:
      - "--no-motd"
      - "--exclude=.git"

# Hardlink files if they didn't change
- name: Use hardlinks when synchronizing filesystems
  ansible.posix.synchronize:
    src: /tmp/path_a/foo.txt
    dest: /tmp/path_b/foo.txt
    link_dest: /tmp/path_a/

# Specify the rsync binary to use on remote host and on local host
- hosts: groupofhosts
  vars:
    ansible_rsync_path: /usr/gnu/bin/rsync

  tasks:
    - name: copy /tmp/localpath/ to remote location /tmp/remotepath
      ansible.posix.synchronize:
        src: /tmp/localpath/
        dest: /tmp/remotepath
        rsync_path: /usr/gnu/bin/rsync
'''


import os
import errno

# TODO(Python2): shlex.quote was added in Python 3.3. This module may run on
# target hosts with Python 2.7 (e.g., older RHEL systems in CI integration tests).
# Remove the try/except fallback to pipes.quote when Python 2 support is dropped.
try:
    from shlex import quote as shlex_quote
except ImportError:
    from pipes import quote as shlex_quote

from ansible.module_utils.basic import AnsibleModule
from ansible.module_utils.common.text.converters import to_bytes, to_native


client_addr = None


def substitute_controller(path):
    global client_addr
    if not client_addr:
        ssh_env_string = os.environ.get('SSH_CLIENT', None)
        try:
            client_addr, _ = ssh_env_string.split(None, 1)  # pylint: disable=disallowed-name
        except AttributeError:
            ssh_env_string = os.environ.get('SSH_CONNECTION', None)
            try:
                client_addr, _ = ssh_env_string.split(None, 1)  # pylint: disable=disallowed-name
            except AttributeError:
                pass
        if not client_addr:
            raise ValueError

    if path.startswith('localhost:'):
        path = path.replace('localhost', client_addr, 1)
    return path


def is_rsh_needed(source, dest):
    if source.startswith('rsync://') or dest.startswith('rsync://'):
        return False
    if ':' in source or ':' in dest:
        return True
    return False


def main():
    module = AnsibleModule(
        argument_spec=dict(
            src=dict(type='path', required=True),
            dest=dict(type='path', required=True),
            dest_port=dict(type='int'),
            delete=dict(type='bool', default=False),
            private_key=dict(type='path'),
            rsync_path=dict(type='str'),
            _local_rsync_path=dict(type='path', default='rsync'),
            _local_rsync_password=dict(type='str', no_log=True),
            _substitute_controller=dict(type='bool', default=False),
            archive=dict(type='bool', default=True),
            checksum=dict(type='bool', default=False),
            compress=dict(type='bool', default=True),
            existing_only=dict(type='bool', default=False),
            dirs=dict(type='bool', default=False),
            recursive=dict(type='bool'),
            links=dict(type='bool'),
            copy_links=dict(type='bool', default=False),
            perms=dict(type='bool'),
            times=dict(type='bool'),
            owner=dict(type='bool'),
            group=dict(type='bool'),
            set_remote_user=dict(type='bool', default=True),
            rsync_timeout=dict(type='int', default=0),
            rsync_opts=dict(type='list', default=[], elements='str'),
            _ssh_args=dict(type='str'),
            use_ssh_args=dict(type='bool', default=False),
            ssh_connection_multiplexing=dict(type='bool', default=False),
            partial=dict(type='bool', default=False),
            verify_host=dict(type='bool', default=False),
            delay_updates=dict(type='bool', default=True),
            mode=dict(type='str', default='push', choices=['pull', 'push']),
            link_dest=dict(type='list', elements='path'),
        ),
        supports_check_mode=True,
    )

    if module.params['_substitute_controller']:
        try:
            source = substitute_controller(module.params['src'])
            dest = substitute_controller(module.params['dest'])
        except ValueError:
            module.fail_json(msg='Could not determine controller hostname for rsync to send to')
    else:
        source = module.params['src']
        dest = module.params['dest']
    dest_port = module.params['dest_port']
    delete = module.params['delete']
    private_key = module.params['private_key']
    rsync_path = module.params['rsync_path']
    rsync = module.params.get('_local_rsync_path', 'rsync')
    rsync_password = module.params.get('_local_rsync_password')
    rsync_timeout = module.params.get('rsync_timeout', 'rsync_timeout')
    archive = module.params['archive']
    checksum = module.params['checksum']
    compress = module.params['compress']
    existing_only = module.params['existing_only']
    dirs = module.params['dirs']
    partial = module.params['partial']
    # the default of these params depends on the value of archive
    recursive = module.params['recursive']
    links = module.params['links']
    copy_links = module.params['copy_links']
    perms = module.params['perms']
    times = module.params['times']
    owner = module.params['owner']
    group = module.params['group']
    rsync_opts = module.params['rsync_opts']
    ssh_args = module.params['_ssh_args']
    ssh_connection_multiplexing = module.params['ssh_connection_multiplexing']
    verify_host = module.params['verify_host']
    link_dest = module.params['link_dest']
    delay_updates = module.params['delay_updates']

    if '/' not in rsync:
        rsync = module.get_bin_path(rsync, required=True)

    cmd = [rsync]
    _sshpass_pipe = None
    if rsync_password:
        try:
            module.run_command(["sshpass"])
        except OSError:
            module.fail_json(
                msg="to use rsync connection with passwords, you must install the sshpass program"
            )
        _sshpass_pipe = os.pipe()
        cmd = ['sshpass', '-d' + to_native(_sshpass_pipe[0], errors='surrogate_or_strict')] + cmd
    if delay_updates:
        cmd.append('--delay-updates')
        cmd.append('-F')
    if compress:
        cmd.append('--compress')
    if rsync_timeout:
        cmd.append('--timeout=%s' % rsync_timeout)
    if module.check_mode:
        cmd.append('--dry-run')
    if delete:
        cmd.append('--delete-after')
    if existing_only:
        cmd.append('--existing')
    if checksum:
        cmd.append('--checksum')
    if copy_links:
        cmd.append('--copy-links')
    if archive:
        cmd.append('--archive')
        if recursive is False:
            cmd.append('--no-recursive')
        if links is False:
            cmd.append('--no-links')
        if perms is False:
            cmd.append('--no-perms')
        if times is False:
            cmd.append('--no-times')
        if owner is False:
            cmd.append('--no-owner')
        if group is False:
            cmd.append('--no-group')
    else:
        if recursive is True:
            cmd.append('--recursive')
        if links is True:
            cmd.append('--links')
        if perms is True:
            cmd.append('--perms')
        if times is True:
            cmd.append('--times')
        if owner is True:
            cmd.append('--owner')
        if group is True:
            cmd.append('--group')
    if dirs:
        cmd.append('--dirs')

    if source.startswith('rsync://') and dest.startswith('rsync://'):
        module.fail_json(msg='either src or dest must be a localhost', rc=1)

    if is_rsh_needed(source, dest):

        # https://github.com/ansible/ansible/issues/15907
        has_rsh = False
        for rsync_opt in rsync_opts:
            if '--rsh' in rsync_opt:
                has_rsh = True
                break

        # if the user has not supplied an --rsh option go ahead and add ours
        if not has_rsh:
            ssh_cmd = [module.get_bin_path('ssh', required=True)]
            if not ssh_connection_multiplexing:
                ssh_cmd.extend(['-S', 'none'])
            if private_key is not None:
                ssh_cmd.extend(['-i', private_key])
            # If the user specified a port value
            # Note:  The action plugin takes care of setting this to a port from
            # inventory if the user didn't specify an explicit dest_port
            if dest_port is not None:
                ssh_cmd.extend(['-o', 'Port=%s' % dest_port])
            if not verify_host:
                ssh_cmd.extend(['-o', 'StrictHostKeyChecking=no', '-o', 'UserKnownHostsFile=/dev/null'])
            ssh_cmd_str = ' '.join(shlex_quote(arg) for arg in ssh_cmd)
            if ssh_args:
                ssh_cmd_str += ' %s' % ssh_args
            cmd.append('--rsh=%s' % shlex_quote(ssh_cmd_str))

    if rsync_path:
        cmd.append('--rsync-path=%s' % shlex_quote(rsync_path))

    if rsync_opts:
        if '' in rsync_opts:
            module.warn('The empty string is present in rsync_opts which will cause rsync to'
                        ' transfer the current working directory. If this is intended, use "."'
                        ' instead to get rid of this warning. If this is unintended, check for'
                        ' problems in your playbook leading to empty string in rsync_opts.')
        cmd.extend(rsync_opts)

    if partial:
        cmd.append('--partial')

    if link_dest:
        cmd.append('-H')
        # verbose required because rsync does not believe that adding a
        # hardlink is actually a change
        cmd.append('-vv')
        for x in link_dest:
            link_path = os.path.abspath(x)
            destination_path = os.path.abspath(os.path.dirname(dest))
            if destination_path.find(link_path) == 0:
                module.fail_json(msg='Hardlinking into a subdirectory of the source would cause recursion. %s and %s' % (destination_path, dest))
            cmd.append('--link-dest=%s' % link_path)

    changed_marker = '<<CHANGED>>'
    cmd.append('--out-format=%s' % shlex_quote(changed_marker + '%i %n%L'))

    cmd.append(shlex_quote(source))
    cmd.append(shlex_quote(dest))
    cmdstr = ' '.join(cmd)

    # If we are using password authentication, write the password into the pipe
    if rsync_password:
        def _write_password_to_pipe(proc):
            os.close(_sshpass_pipe[0])
            try:
                os.write(_sshpass_pipe[1], to_bytes(rsync_password) + b'\n')
            except OSError as exc:
                # Ignore broken pipe errors if the sshpass process has exited.
                if exc.errno != errno.EPIPE or proc.poll() is None:
                    raise

        (rc, out, err) = module.run_command(
            cmdstr, pass_fds=_sshpass_pipe,
            before_communicate_callback=_write_password_to_pipe)
    else:
        (rc, out, err) = module.run_command(cmdstr)

    if rc:
        return module.fail_json(msg=err, rc=rc, cmd=cmdstr)

    if link_dest:
        # a leading period indicates no change
        changed = (changed_marker + '.') not in out
    else:
        changed = changed_marker in out

    out_clean = out.replace(changed_marker, '')
    out_lines = out_clean.split('\n')
    while '' in out_lines:
        out_lines.remove('')
    if module._diff:
        diff = {'prepared': out_clean}
        return module.exit_json(changed=changed, msg=out_clean,
                                rc=rc, cmd=cmdstr, stdout_lines=out_lines,
                                diff=diff)

    return module.exit_json(changed=changed, msg=out_clean,
                            rc=rc, cmd=cmdstr, stdout_lines=out_lines)


if __name__ == '__main__':
    main()
