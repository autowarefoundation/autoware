# -*- coding: utf-8 -*-

# Copyright: (c) 2021, Felix Fontein <felix@fontein.de>
# GNU General Public License v3.0+ (see COPYING or https://www.gnu.org/licenses/gpl-3.0.txt)

"""Provide version object to compare version numbers."""

from __future__ import absolute_import, division, print_function

__metaclass__ = type


# Once we drop support for Ansible 2.9, ansible-base 2.10, and ansible-core 2.11, we can
# remove the _version.py file, and replace the following import by
#
#     from ansible.module_utils.compat.version import LooseVersion

from ._version import LooseVersion, StrictVersion

__all__ = ['LooseVersion', 'StrictVersion']
