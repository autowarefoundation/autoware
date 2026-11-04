# Copyright (c) 2023 Maxwell G <maxwell@gtmx.me>
# GNU General Public License v3.0+ (see COPYING or https://www.gnu.org/licenses/gpl-3.0.txt)

"""
Helpers to respawn a module to run using the system interpreter
"""

from __future__ import absolute_import, division, print_function

__metaclass__ = type

try:
    from ansible.module_utils.common import respawn
except ImportError:
    HAS_RESPAWN_UTIL = False
else:
    HAS_RESPAWN_UTIL = True


SYSTEM_PYTHON_INTERPRETERS = (
    "/usr/bin/libexec/platform-python",
    "/usr/bin/python3",
    "/usr/bin/python2",
    "/usr/bin/python",
)


def respawn_module(module):
    """
    Respawn an ansible module to using the first interpreter in
    SYSTEM_PYTHON_INTERPRETERS that contains `module`.

    Args:
        module (str): Name of python module to search for

    Returns:
        Returns None if the module cannot be respawned.
    """
    if respawn.has_respawned():
        return
    interpreter = respawn.probe_interpreters_for_module(
        SYSTEM_PYTHON_INTERPRETERS, module
    )
    if interpreter:
        respawn.respawn_module(interpreter)
