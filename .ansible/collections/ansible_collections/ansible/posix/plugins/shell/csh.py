# Copyright (c) 2014, Chris Church <chris@ninemoreminutes.com>
# Copyright (c) 2017 Ansible Project
# GNU General Public License v3.0+ (see COPYING or https://www.gnu.org/licenses/gpl-3.0.txt)
from __future__ import (absolute_import, division, print_function)
__metaclass__ = type

DOCUMENTATION = '''
    name: csh
    short_description: C shell (/bin/csh)
    description:
      - When you have no other option than to use csh
    extends_documentation_fragment:
      - shell_common
'''

from shlex import quote as shlex_quote
from ansible.plugins.shell import ShellBase


class ShellModule(ShellBase):

    # Common shell filenames that this plugin handles
    COMPATIBLE_SHELLS = frozenset(('csh', 'tcsh'))
    # Family of shells this has.  Must match the filename without extension
    SHELL_FAMILY = 'csh'

    # commonly used
    ECHO = 'echo'
    COMMAND_SEP = ';'

    # How to end lines in a python script one-liner
    _SHELL_EMBEDDED_PY_EOL = '\\\n'
    _SHELL_REDIRECT_ALLNULL = '>& /dev/null'
    _SHELL_AND = '&&'
    _SHELL_OR = '||'
    _SHELL_SUB_LEFT = '"`'
    _SHELL_SUB_RIGHT = '`"'
    _SHELL_GROUP_LEFT = '('
    _SHELL_GROUP_RIGHT = ')'

    def env_prefix(self, **kwargs):
        ret = []
        # All the -u options must be first, so we process them first
        ret += ['-u %s' % k for k, v in kwargs.items() if v is None]
        ret += ['%s=%s' % (k, shlex_quote(str(v))) for k, v in kwargs.items() if v is not None]
        return 'env %s' % ' '.join(ret)
