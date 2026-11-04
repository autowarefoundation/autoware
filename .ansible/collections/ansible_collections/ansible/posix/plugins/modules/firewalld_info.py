#!/usr/bin/python
# -*- coding: utf-8 -*-

# Copyright: (c) 2021, Hideki Saito <saito@fgrep.org>
# GNU General Public License v3.0+ (see COPYING or https://www.gnu.org/licenses/gpl-3.0.txt)

from __future__ import (absolute_import, division, print_function)
__metaclass__ = type

DOCUMENTATION = r'''
---
module: firewalld_info
short_description: Gather information about firewalld
description:
    - This module gathers information about firewalld rules.
options:
    active_zones:
        description: Gather information about active zones.
        type: bool
        default: false
    zones:
        description:
            - Gather information about specific zones.
            - If only works if O(active_zones=false).
        required: false
        type: list
        elements: str
requirements:
    - firewalld >= 0.2.11
    - python-firewall
    - python-dbus
author:
    - Hideki Saito (@saito-hideki)
'''

EXAMPLES = r'''
- name: Gather information about active zones
  ansible.posix.firewalld_info:
    active_zones: true
  register: result

- name: Print default zone for debugging
  ansible.builtin.debug:
    var: result.firewalld_info.default_zone

- name: Gather information about specific zones
  ansible.posix.firewalld_info:
    zones:
      - public
      - external
      - internal
  register: result
'''

RETURN = r'''
active_zones:
    description:
      - Gather active zones only if turn it C(true).
    returned: success
    type: bool
    sample: false
collected_zones:
    description:
      - A list of collected zones.
    returned: success
    type: list
    sample: [external, internal]
undefined_zones:
    description:
      - A list of undefined zones in C(zones) option.
      - C(undefined_zones) will be ignored for gathering process.
    returned: success
    type: list
    sample: [foo, bar]
firewalld_info:
    description:
      - Returns various information about firewalld configuration.
    returned: success
    type: complex
    contains:
        version:
            description:
              - The version information of firewalld.
            returned: success
            type: str
            sample: 0.8.2
        default_zone:
            description:
              - The zone name of default zone.
            returned: success
            type: str
            sample: public
        zones:
            description:
              - A dict of zones to gather information.
            returned: success
            type: complex
            contains:
                zone:
                    description:
                      - The zone name registered in firewalld.
                    returned: success
                    type: complex
                    sample: external
                    contains:
                        target:
                            description:
                              - A list of services in the zone.
                            returned: success
                            type: str
                            sample: ACCEPT
                        icmp_block_inversion:
                            description:
                              - The ICMP block inversion to block
                                all ICMP requests.
                            returned: success
                            type: bool
                            sample: false
                        interfaces:
                            description:
                              - A list of network interfaces.
                            returned: success
                            type: list
                            sample:
                              - 'eth0'
                              - 'eth1'
                        sources:
                            description:
                              - A list of source network address.
                            returned: success
                            type: list
                            sample:
                              - '172.16.30.0/24'
                              - '172.16.31.0/24'
                        services:
                            description:
                              - A list of network services.
                            returned: success
                            type: list
                            sample:
                              - 'dhcp'
                              - 'dns'
                              - 'ssh'
                        ports:
                            description:
                              - A list of network port with protocol.
                            returned: success
                            type: list
                            sample:
                              - - "22"
                                - "tcp"
                              - - "80"
                                - "tcp"
                        protocols:
                            description:
                              - A list of network protocol.
                            returned: success
                            type: list
                            sample:
                              - "icmp"
                              - "ipv6-icmp"
                        forward:
                            description:
                              - The network interface forwarding.
                              - This parameter supports on python-firewall
                                0.9.0(or later) and is not collected in earlier
                                versions.
                            returned: success
                            type: bool
                            sample: false
                        masquerade:
                            description:
                              - The network interface masquerading.
                            returned: success
                            type: bool
                            sample: false
                        forward_ports:
                            description:
                              - A list of forwarding port pair with protocol.
                            returned: success
                            type: list
                            sample:
                              - "icmp"
                              - "ipv6-icmp"
                        source_ports:
                            description:
                              - A list of network source port with protocol.
                            returned: success
                            type: list
                            sample:
                              - - "30000"
                                - "tcp"
                              - - "30001"
                                - "tcp"
                        icmp_blocks:
                            description:
                              - A list of blocking icmp protocol.
                            returned: success
                            type: list
                            sample:
                              - "echo-request"
                        rich_rules:
                            description:
                              - A list of rich language rule.
                            returned: success
                            type: list
                            sample:
                              - "rule protocol value=\"icmp\" reject"
                              - "rule priority=\"32767\" reject"
'''

from ansible.module_utils.basic import AnsibleModule, missing_required_lib
from ansible.module_utils.common.text.converters import to_native
from ansible_collections.ansible.posix.plugins.module_utils._respawn import respawn_module, HAS_RESPAWN_UTIL
from ansible_collections.ansible.posix.plugins.module_utils.version import StrictVersion


try:
    import dbus
    HAS_DBUS = True
except ImportError:
    HAS_DBUS = False

try:
    import firewall.client as fw_client
    import firewall.config as fw_config
    HAS_FIREWALLD = True
except ImportError:
    HAS_FIREWALLD = False


def get_version():
    return fw_config.VERSION


def get_active_zones(client):
    return client.getActiveZones().keys()


def get_all_zones(client):
    return client.getZones()


def get_default_zone(client):
    return client.getDefaultZone()


def get_zone_settings(client, zone):
    return client.getZoneSettings(zone)


def get_zone_target(zone_settings):
    return zone_settings.getTarget()


def get_zone_icmp_block_inversion(zone_settings):
    return zone_settings.getIcmpBlockInversion()


def get_zone_interfaces(zone_settings):
    return zone_settings.getInterfaces()


def get_zone_sources(zone_settings):
    return zone_settings.getSources()


def get_zone_services(zone_settings):
    return zone_settings.getServices()


def get_zone_ports(zone_settings):
    return zone_settings.getPorts()


def get_zone_protocols(zone_settings):
    return zone_settings.getProtocols()


# This function supports python-firewall 0.9.0(or later).
def get_zone_forward(zone_settings):
    return zone_settings.getForward()


def get_zone_masquerade(zone_settings):
    return zone_settings.getMasquerade()


def get_zone_forward_ports(zone_settings):
    return zone_settings.getForwardPorts()


def get_zone_source_ports(zone_settings):
    return zone_settings.getSourcePorts()


def get_zone_icmp_blocks(zone_settings):
    return zone_settings.getIcmpBlocks()


def get_zone_rich_rules(zone_settings):
    return zone_settings.getRichRules()


def main():
    module_args = dict(
        active_zones=dict(required=False, type='bool', default=False),
        zones=dict(required=False, type='list', elements='str'),
    )

    module = AnsibleModule(
        argument_spec=module_args,
        supports_check_mode=True,
    )

    firewalld_info = dict()
    result = dict(
        changed=False,
        active_zones=module.params['active_zones'],
        collected_zones=list(),
        undefined_zones=list(),
    )

    # Exit with failure message if requirements modules are not installed.
    if not HAS_DBUS and not HAS_FIREWALLD and HAS_RESPAWN_UTIL:
        # Only respawn the module if both libraries are missing.
        # If only one is available, then usage of the "wrong" (i.e. not the system one)
        # python interpreter is likely not the problem.
        respawn_module("firewall")

    if not HAS_DBUS:
        module.fail_json(msg=missing_required_lib('python-dbus'))
    if not HAS_FIREWALLD:
        module.fail_json(msg=missing_required_lib('python-firewall'))

    try:
        client = fw_client.FirewallClient()

        # Gather general information of firewalld.
        firewalld_info['version'] = get_version()
        firewalld_info['default_zone'] = get_default_zone(client)

        # Gather information for zones.
        zones_info = dict()
        collect_zones = list()
        ignore_zones = list()
        if module.params['active_zones']:
            collect_zones = get_active_zones(client)
        elif module.params['zones']:
            all_zones = get_all_zones(client)
            specified_zones = module.params['zones']
            collect_zones = list(set(specified_zones) & set(all_zones))
            ignore_zones = list(set(specified_zones) - set(collect_zones))
            if ignore_zones:
                module.warn('Please note: zone:(%s) have been ignored in the gathering process.' % ", ".join(ignore_zones))
        else:
            collect_zones = get_all_zones(client)

        for zone in collect_zones:
            # Gather settings for each zone based on the output of
            # 'firewall-cmd --info-zone=<ZONE>' command.
            zone_info = dict()
            zone_settings = get_zone_settings(client, zone)
            zone_info['target'] = get_zone_target(zone_settings)
            zone_info['icmp_block_inversion'] = get_zone_icmp_block_inversion(zone_settings)
            zone_info['interfaces'] = get_zone_interfaces(zone_settings)
            zone_info['sources'] = get_zone_sources(zone_settings)
            zone_info['services'] = get_zone_services(zone_settings)
            zone_info['ports'] = get_zone_ports(zone_settings)
            zone_info['protocols'] = get_zone_protocols(zone_settings)
            zone_info['masquerade'] = get_zone_masquerade(zone_settings)
            zone_info['forward_ports'] = get_zone_forward_ports(zone_settings)
            zone_info['source_ports'] = get_zone_source_ports(zone_settings)
            zone_info['icmp_blocks'] = get_zone_icmp_blocks(zone_settings)
            zone_info['rich_rules'] = get_zone_rich_rules(zone_settings)

            # The 'forward' parameter supports on python-firewall 0.9.0(or later).
            if StrictVersion(firewalld_info['version']) >= StrictVersion('0.9.0'):
                zone_info['forward'] = get_zone_forward(zone_settings)

            zones_info[zone] = zone_info
        firewalld_info['zones'] = zones_info
    except AttributeError as e:
        module.fail_json(msg=('firewalld probably not be running, Or the following method '
                              'is not supported with your python-firewall version. (Error: %s)') % to_native(e))
    except dbus.exceptions.DBusException as e:
        module.fail_json(msg=('Unable to gather firewalld settings.'
                              ' You may need to run as the root user or'
                              ' use become. (Error: %s)' % to_native(e)))

    result['collected_zones'] = collect_zones
    result['undefined_zones'] = ignore_zones
    result['firewalld_info'] = firewalld_info
    module.exit_json(**result)


if __name__ == '__main__':
    main()
