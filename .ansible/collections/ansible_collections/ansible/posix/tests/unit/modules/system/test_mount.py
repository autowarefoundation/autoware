from __future__ import absolute_import, division, print_function

__metaclass__ = type

import os
import tempfile

from ansible_collections.ansible.posix.tests.unit.compat import unittest
from ansible_collections.ansible.posix.tests.unit.compat.mock import MagicMock
from ansible.module_utils.common.text.converters import to_bytes

from ansible_collections.ansible.posix.plugins.modules.mount import (
    get_linux_mounts,
    _set_mount_save_old,
    set_mount,
)


class LinuxMountsTestCase(unittest.TestCase):

    def _create_file(self, content):
        tmp_file = tempfile.NamedTemporaryFile(prefix='ansible-test-', delete=False)
        tmp_file.write(to_bytes(content))
        tmp_file.close()
        self.addCleanup(os.unlink, tmp_file.name)
        return tmp_file.name

    def test_code_comment(self):
        path = self._create_file(
            '140 136 253:2 /rootfs / rw - ext4 /dev/sdb2 rw\n'
            '141 140 253:2 /rootfs/tmp/aaa /tmp/bbb rw - ext4 /dev/sdb2 rw\n'
        )
        mounts = get_linux_mounts(None, path)
        self.assertEqual(mounts['/tmp/bbb']['src'], '/tmp/aaa')

    def test_set_mount_save_old(self):
        module = MagicMock(name='AnsibleModule')
        module.check_mode = True
        module.params = {'backup': False}

        fstab_data = [
            'UUID=8ac075e3-1124-4bb6-bef7-a6811bf8b870 / xfs defaults 0 0\n',
            '/swapfile none swap defaults 0 0\n'
        ]
        path = self._create_file("".join(fstab_data))
        args = {
            'fstab': path,
            'name': '/data',
            'src': '/dev/sdb1',
            'fstype': 'ext4',
            'opts': 'defaults',
            'dump': '0',
            'passno': '0',
        }

        name, changed = set_mount(module, args)
        self.assertEqual(name, '/data')
        self.assertTrue(changed)

        name, backup_lines, changed = _set_mount_save_old(module, args)
        self.assertEqual(backup_lines, fstab_data)
        self.assertEqual(name, '/data')
        self.assertTrue(changed)
