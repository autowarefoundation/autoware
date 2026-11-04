#!/usr/bin/python
# -*- coding: utf-8 -*-

# Copyright: (c) 2013, Adam Miller <maxamillion@fedoraproject.org>
# GNU General Public License v3.0+ (see COPYING or https://www.gnu.org/licenses/gpl-3.0.txt)

from __future__ import absolute_import, division, print_function
__metaclass__ = type

DOCUMENTATION = r'''
---
module: firewalld
short_description: Manage arbitrary ports/services with firewalld
description:
  - This module allows for addition or deletion of services and ports (either TCP or UDP) in either running or permanent firewalld rules.
options:
  service:
    description:
      - Name of a service to add/remove to/from firewalld.
      - The service must be listed in output of C(firewall-cmd --get-services).
    type: str
  protocol:
    description:
      - Name of a protocol to add/remove to/from firewalld.
    type: str
  port:
    description:
      - Name of a port or port range to add/remove to/from firewalld.
      - Must be in the form PORT/PROTOCOL or PORT-PORT/PROTOCOL for port ranges.
    type: str
  port_forward:
    description:
      - Port and protocol to forward using firewalld.
    type: list
    elements: dict
    suboptions:
      port:
        type: str
        required: true
        description:
          - Source port to forward from.
      proto:
        type: str
        required: true
        description:
          - protocol to forward.
        choices: [udp, tcp]
      toport:
        type: str
        required: true
        description:
          - destination port.
      toaddr:
        type: str
        description:
          - Optional address to forward to.
  rich_rule:
    description:
      - Rich rule to add/remove to/from firewalld.
      - See L(Syntax for firewalld rich language rules,https://firewalld.org/documentation/man-pages/firewalld.richlanguage.html).
    type: str
  source:
    description:
      - The source/network you would like to add/remove to/from firewalld.
    type: str
  interface:
    description:
      - The interface you would like to add/remove to/from a zone in firewalld.
    type: str
  icmp_block:
    description:
      - The ICMP block you would like to add/remove to/from a zone in firewalld.
    type: str
  icmp_block_inversion:
    description:
      - Enable/Disable inversion of ICMP blocks for a zone in firewalld.
      - Note that the option type is changed to bool in ansible.posix version 2.0.0 and later.
    type: bool
  zone:
    description:
      - The firewalld zone to add/remove to/from.
      - Note that the default zone can be configured per system but V(public) is default from upstream.
      - Available choices can be extended based on per-system configs, listed here are "out of the box" defaults.
      - Possible values include V(block), V(dmz), V(drop), V(external), V(home), V(internal), V(public), V(trusted), V(work).
    type: str
  permanent:
    description:
      - Whether to apply this change to the permanent firewalld configuration.
      - As of Ansible 2.3, permanent operations can operate on firewalld configs when it is not running (requires firewalld >= 0.3.9).
      - Note that if this is V(false), O(immediate=true) by default.
    type: bool
    default: false
  immediate:
    description:
      - Whether to apply this change to the runtime firewalld configuration.
      - Defaults to V(true) if O(permanent=false).
    type: bool
    default: false
  state:
    description:
      - Enable or disable a setting.
      - 'For ports: Should this port accept (V(enabled)) or reject (V(disabled)) connections.'
      - The states V(present) and V(absent) can only be used in zone level operations (i.e. when no other parameters but zone and state are set).
    type: str
    required: true
    choices: [ absent, disabled, enabled, present ]
  timeout:
    description:
      - The amount of time in seconds the rule should be in effect for when non-permanent.
    type: int
    default: 0
  forward:
    description:
      - The forward setting you would like to enable/disable to/from zones within firewalld.
      - This option only is supported by firewalld v0.9.0 or later.
      - Note that the option type is changed to bool in ansible.posix version 2.0.0 and later.
    type: bool
  masquerade:
    description:
      - The masquerade setting you would like to enable/disable to/from zones within firewalld.
      - Note that the option type is changed to bool in ansible.posix version 2.0.0 and later.
    type: bool
  offline:
    description:
      - Ignores O(immediate) if O(permanent=true) and firewalld is not running.
    type: bool
    default: false
  target:
    description:
      - firewalld Zone target.
      - If O(state=absent), this will reset the target to default.
    choices: [ default, ACCEPT, DROP, "%%REJECT%%" ]
    type: str
    version_added: 1.2.0
notes:
  - Not tested on any Debian based system.
  - Requires the python2 bindings of firewalld, which may not be installed by default.
  - For distributions where the python2 firewalld bindings are unavailable (e.g Fedora 28 and later) you will have to set the
    ansible_python_interpreter for these hosts to the python3 interpreter path and install the python3 bindings.
  - Zone transactions (creating, deleting) can be performed by using only the zone and state parameters "present" or "absent".
    Note that zone transactions must explicitly be permanent. This is a limitation in firewalld.
    This also means that you will have to reload firewalld after adding a zone that you wish to perform immediate actions on.
    The module will not take care of this for you implicitly because that would undo any previously performed immediate actions which were not
    permanent. Therefore, if you require immediate access to a newly created zone it is recommended you reload firewalld immediately after the zone
    creation returns with a changed state and before you perform any other immediate, non-permanent actions on that zone.
  - This module needs C(python-firewall) or C(python3-firewall) on managed nodes.
    It is usually provided as a subset with C(firewalld) from the OS distributor for the OS default Python interpreter.
requirements:
- firewalld >= 0.9.0
- python-firewall >= 0.9.0
author:
- Adam Miller (@maxamillion)
'''

EXAMPLES = r'''
- name: Permanently enable https service, also enable it immediately if possible
  ansible.posix.firewalld:
    service: https
    state: enabled
    permanent: true
    immediate: true
    offline: true

- name: Permit traffic in default zone for https service
  ansible.posix.firewalld:
    service: https
    permanent: true
    state: enabled

- name: Permit ospf traffic
  ansible.posix.firewalld:
    protocol: ospf
    permanent: true
    state: enabled

- name: Do not permit traffic in default zone on port 8081/tcp
  ansible.posix.firewalld:
    port: 8081/tcp
    permanent: true
    state: disabled

- name: Permit traffic in default zone on port 161-162/ucp
  ansible.posix.firewalld:
    port: 161-162/udp
    permanent: true
    state: enabled

- name: Permit traffic in dmz zone on http service
  ansible.posix.firewalld:
    zone: dmz
    service: http
    permanent: true
    state: enabled

- name: Enable FTP service with rate limiting using firewalld rich rule
  ansible.posix.firewalld:
    rich_rule: rule service name="ftp" audit limit value="1/m" accept
    permanent: true
    state: enabled

- name: Allow traffic from 192.0.2.0/24 in internal zone
  ansible.posix.firewalld:
    source: 192.0.2.0/24
    zone: internal
    state: enabled

- name: Assign eth2 interface to trusted zone
  ansible.posix.firewalld:
    zone: trusted
    interface: eth2
    permanent: true
    state: enabled

- name: Enable forwarding in internal zone
  ansible.posix.firewalld:
    forward: true
    state: enabled
    permanent: true
    zone: internal

- name: Enable masquerade in dmz zone
  ansible.posix.firewalld:
    masquerade: true
    state: enabled
    permanent: true
    zone: dmz

- name: Create custom zone if not already present
  ansible.posix.firewalld:
    zone: custom
    state: present
    permanent: true

- name: Enable ICMP block inversion in drop zone
  ansible.posix.firewalld:
    zone: drop
    state: enabled
    permanent: true
    icmp_block_inversion: true

- name: Block ICMP echo requests in drop zone
  ansible.posix.firewalld:
    zone: drop
    state: enabled
    permanent: true
    icmp_block: echo-request

- name: Set internal zone target to ACCEPT
  ansible.posix.firewalld:
    zone: internal
    state: present
    permanent: true
    target: ACCEPT

- name: Redirect port 443 to 8443 with Rich Rule
  ansible.posix.firewalld:
    rich_rule: rule family=ipv4 forward-port port=443 protocol=tcp to-port=8443
    zone: public
    permanent: true
    immediate: true
    state: enabled
'''

from ansible.module_utils.basic import AnsibleModule
from ansible_collections.ansible.posix.plugins.module_utils.firewalld import FirewallTransaction, fw_offline

try:
    from firewall.client import Rich_Rule
    from firewall.client import FirewallClientZoneSettings
except ImportError:
    # The import errors are handled via FirewallTransaction, don't need to
    # duplicate that here
    pass


class IcmpBlockTransaction(FirewallTransaction):
    """
    IcmpBlockTransaction
    """

    def __init__(self, module, action_args=None, zone=None, desired_state=None, permanent=False, immediate=False):
        super(IcmpBlockTransaction, self).__init__(
            module, action_args=action_args, desired_state=desired_state, zone=zone, permanent=permanent, immediate=immediate
        )

    def get_enabled_immediate(self, icmp_block, timeout):
        return icmp_block in self.fw.getIcmpBlocks(self.zone)

    def get_enabled_permanent(self, icmp_block, timeout):
        fw_zone, fw_settings = self.get_fw_zone_settings()
        return icmp_block in fw_settings.getIcmpBlocks()

    def set_enabled_immediate(self, icmp_block, timeout):
        self.fw.addIcmpBlock(self.zone, icmp_block, timeout)

    def set_enabled_permanent(self, icmp_block, timeout):
        fw_zone, fw_settings = self.get_fw_zone_settings()
        fw_settings.addIcmpBlock(icmp_block)
        self.update_fw_settings(fw_zone, fw_settings)

    def set_disabled_immediate(self, icmp_block, timeout):
        self.fw.removeIcmpBlock(self.zone, icmp_block)

    def set_disabled_permanent(self, icmp_block, timeout):
        fw_zone, fw_settings = self.get_fw_zone_settings()
        fw_settings.removeIcmpBlock(icmp_block)
        self.update_fw_settings(fw_zone, fw_settings)


class IcmpBlockInversionTransaction(FirewallTransaction):
    """
    IcmpBlockInversionTransaction
    """

    def __init__(self, module, action_args=None, zone=None, desired_state=None, permanent=False, immediate=False):
        super(IcmpBlockInversionTransaction, self).__init__(
            module, action_args=action_args, desired_state=desired_state, zone=zone, permanent=permanent, immediate=immediate
        )

    def get_enabled_immediate(self):
        if self.fw.queryIcmpBlockInversion(self.zone) is True:
            return True
        else:
            return False

    def get_enabled_permanent(self):
        fw_zone, fw_settings = self.get_fw_zone_settings()
        if fw_settings.getIcmpBlockInversion() is True:
            return True
        else:
            return False

    def set_enabled_immediate(self):
        self.fw.addIcmpBlockInversion(self.zone)

    def set_enabled_permanent(self):
        fw_zone, fw_settings = self.get_fw_zone_settings()
        fw_settings.setIcmpBlockInversion(True)
        self.update_fw_settings(fw_zone, fw_settings)

    def set_disabled_immediate(self):
        self.fw.removeIcmpBlockInversion(self.zone)

    def set_disabled_permanent(self):
        fw_zone, fw_settings = self.get_fw_zone_settings()
        fw_settings.setIcmpBlockInversion(False)
        self.update_fw_settings(fw_zone, fw_settings)


class ServiceTransaction(FirewallTransaction):
    """
    ServiceTransaction
    """

    def __init__(self, module, action_args=None, zone=None, desired_state=None, permanent=False, immediate=False):
        super(ServiceTransaction, self).__init__(
            module, action_args=action_args, desired_state=desired_state, zone=zone, permanent=permanent, immediate=immediate
        )

    def get_enabled_immediate(self, service, timeout):
        if service in self.fw.getServices(self.zone):
            return True
        else:
            return False

    def get_enabled_permanent(self, service, timeout):
        fw_zone, fw_settings = self.get_fw_zone_settings()

        if service in fw_settings.getServices():
            return True
        else:
            return False

    def set_enabled_immediate(self, service, timeout):
        self.fw.addService(self.zone, service, timeout)

    def set_enabled_permanent(self, service, timeout):
        fw_zone, fw_settings = self.get_fw_zone_settings()
        fw_settings.addService(service)
        self.update_fw_settings(fw_zone, fw_settings)

    def set_disabled_immediate(self, service, timeout):
        self.fw.removeService(self.zone, service)

    def set_disabled_permanent(self, service, timeout):
        fw_zone, fw_settings = self.get_fw_zone_settings()
        fw_settings.removeService(service)
        self.update_fw_settings(fw_zone, fw_settings)


class ProtocolTransaction(FirewallTransaction):
    """
    ProtocolTransaction
    """

    def __init__(self, module, action_args=None, zone=None, desired_state=None, permanent=False, immediate=False):
        super(ProtocolTransaction, self).__init__(
            module, action_args=action_args, desired_state=desired_state, zone=zone, permanent=permanent, immediate=immediate
        )

    def get_enabled_immediate(self, protocol, timeout):
        if protocol in self.fw.getProtocols(self.zone):
            return True
        else:
            return False

    def get_enabled_permanent(self, protocol, timeout):
        fw_zone, fw_settings = self.get_fw_zone_settings()

        if protocol in fw_settings.getProtocols():
            return True
        else:
            return False

    def set_enabled_immediate(self, protocol, timeout):
        self.fw.addProtocol(self.zone, protocol, timeout)

    def set_enabled_permanent(self, protocol, timeout):
        fw_zone, fw_settings = self.get_fw_zone_settings()
        fw_settings.addProtocol(protocol)
        self.update_fw_settings(fw_zone, fw_settings)

    def set_disabled_immediate(self, protocol, timeout):
        self.fw.removeProtocol(self.zone, protocol)

    def set_disabled_permanent(self, protocol, timeout):
        fw_zone, fw_settings = self.get_fw_zone_settings()
        fw_settings.removeProtocol(protocol)
        self.update_fw_settings(fw_zone, fw_settings)


class ForwardTransaction(FirewallTransaction):
    """
    ForwardTransaction
    """

    def __init__(self, module, action_args=None, zone=None, desired_state=None, permanent=False, immediate=False):
        super(ForwardTransaction, self).__init__(
            module, action_args=action_args, desired_state=desired_state, zone=zone, permanent=permanent, immediate=immediate
        )

        self.enabled_msg = "Added forward to zone %s" % self.zone
        self.disabled_msg = "Removed forward from zone %s" % self.zone

    def get_enabled_immediate(self):
        if self.fw.queryForward(self.zone) is True:
            return True
        else:
            return False

    def get_enabled_permanent(self):
        fw_zone, fw_settings = self.get_fw_zone_settings()
        if fw_settings.queryForward() is True:
            return True
        else:
            return False

    def set_enabled_immediate(self):
        self.fw.addForward(self.zone)

    def set_enabled_permanent(self):
        fw_zone, fw_settings = self.get_fw_zone_settings()
        fw_settings.setForward(True)
        self.update_fw_settings(fw_zone, fw_settings)

    def set_disabled_immediate(self):
        self.fw.removeForward(self.zone)

    def set_disabled_permanent(self):
        fw_zone, fw_settings = self.get_fw_zone_settings()
        fw_settings.setForward(False)
        self.update_fw_settings(fw_zone, fw_settings)


class MasqueradeTransaction(FirewallTransaction):
    """
    MasqueradeTransaction
    """

    def __init__(self, module, action_args=None, zone=None, desired_state=None, permanent=False, immediate=False):
        super(MasqueradeTransaction, self).__init__(
            module, action_args=action_args, desired_state=desired_state, zone=zone, permanent=permanent, immediate=immediate
        )

        self.enabled_msg = "Added masquerade to zone %s" % self.zone
        self.disabled_msg = "Removed masquerade from zone %s" % self.zone

    def get_enabled_immediate(self):
        if self.fw.queryMasquerade(self.zone) is True:
            return True
        else:
            return False

    def get_enabled_permanent(self):
        fw_zone, fw_settings = self.get_fw_zone_settings()
        if fw_settings.getMasquerade() is True:
            return True
        else:
            return False

    def set_enabled_immediate(self):
        self.fw.addMasquerade(self.zone)

    def set_enabled_permanent(self):
        fw_zone, fw_settings = self.get_fw_zone_settings()
        fw_settings.setMasquerade(True)
        self.update_fw_settings(fw_zone, fw_settings)

    def set_disabled_immediate(self):
        self.fw.removeMasquerade(self.zone)

    def set_disabled_permanent(self):
        fw_zone, fw_settings = self.get_fw_zone_settings()
        fw_settings.setMasquerade(False)
        self.update_fw_settings(fw_zone, fw_settings)


class PortTransaction(FirewallTransaction):
    """
    PortTransaction
    """

    def __init__(self, module, action_args=None, zone=None, desired_state=None, permanent=False, immediate=False):
        super(PortTransaction, self).__init__(
            module, action_args=action_args, desired_state=desired_state, zone=zone, permanent=permanent, immediate=immediate
        )

    def get_enabled_immediate(self, port, protocol, timeout):
        if self.fw_offline:
            dummy, fw_settings = self.get_fw_zone_settings()
            return fw_settings.queryPort(port=port, protocol=protocol)
        return self.fw.queryPort(zone=self.zone, port=port, protocol=protocol)

    def get_enabled_permanent(self, port, protocol, timeout):
        dummy, fw_settings = self.get_fw_zone_settings()
        return fw_settings.queryPort(port=port, protocol=protocol)

    def set_enabled_immediate(self, port, protocol, timeout):
        self.fw.addPort(self.zone, port, protocol, timeout)

    def set_enabled_permanent(self, port, protocol, timeout):
        fw_zone, fw_settings = self.get_fw_zone_settings()
        fw_settings.addPort(port, protocol)
        self.update_fw_settings(fw_zone, fw_settings)

    def set_disabled_immediate(self, port, protocol, timeout):
        self.fw.removePort(self.zone, port, protocol)

    def set_disabled_permanent(self, port, protocol, timeout):
        fw_zone, fw_settings = self.get_fw_zone_settings()
        fw_settings.removePort(port, protocol)
        self.update_fw_settings(fw_zone, fw_settings)


class InterfaceTransaction(FirewallTransaction):
    """
    InterfaceTransaction
    """

    def __init__(self, module, action_args=None, zone=None, desired_state=None, permanent=False, immediate=False):
        super(InterfaceTransaction, self).__init__(
            module, action_args=action_args, desired_state=desired_state, zone=zone, permanent=permanent, immediate=immediate
        )

        self.enabled_msg = "Changed %s to zone %s" % \
            (self.action_args[0], self.zone)

        self.disabled_msg = "Removed %s from zone %s" % \
            (self.action_args[0], self.zone)

    def get_enabled_immediate(self, interface):
        if self.fw_offline:
            fw_zone, fw_settings = self.get_fw_zone_settings()
            interface_list = fw_settings.getInterfaces()
        else:
            interface_list = self.fw.getInterfaces(self.zone)
        if interface in interface_list:
            return True
        else:
            return False

    def get_enabled_permanent(self, interface):
        fw_zone, fw_settings = self.get_fw_zone_settings()

        if interface in fw_settings.getInterfaces():
            return True
        else:
            return False

    def set_enabled_immediate(self, interface):
        self.fw.changeZoneOfInterface(self.zone, interface)

    def set_enabled_permanent(self, interface):
        fw_zone, fw_settings = self.get_fw_zone_settings()
        if self.fw_offline:
            iface_zone_objs = []
            for zone in self.fw.config.get_zones():
                old_zone_obj = self.fw.config.get_zone(zone)
                if interface in old_zone_obj.interfaces:
                    iface_zone_objs.append(old_zone_obj)

            if len(iface_zone_objs) > 1:
                # Even it shouldn't happen, it's actually possible that
                # the same interface is in several zone XML files
                self.module.fail_json(
                    msg='ERROR: interface {0} is in {1} zone XML file, can only be in one'.format(
                        interface,
                        len(iface_zone_objs)
                    )
                )
            elif len(iface_zone_objs) == 1 and iface_zone_objs[0].name != self.zone:
                old_zone_obj = iface_zone_objs[0]
                old_zone_config = self.fw.config.get_zone_config(old_zone_obj)
                old_zone_settings = FirewallClientZoneSettings(list(old_zone_config))
                old_zone_settings.removeInterface(interface)    # remove from old
                self.fw.config.set_zone_config(
                    old_zone_obj,
                    old_zone_settings.settings
                )
            fw_settings.addInterface(interface)             # add to new
            self.fw.config.set_zone_config(fw_zone, fw_settings.settings)
        else:
            old_zone_name = self.fw.config().getZoneOfInterface(interface)
            if old_zone_name != self.zone:
                if old_zone_name:
                    old_zone_obj = self.fw.config().getZoneByName(old_zone_name)
                    old_zone_settings = old_zone_obj.getSettings()
                    old_zone_settings.removeInterface(interface)  # remove from old
                    old_zone_obj.update(old_zone_settings)
                fw_settings.addInterface(interface)              # add to new
                fw_zone.update(fw_settings)

    def set_disabled_immediate(self, interface):
        self.fw.removeInterface(self.zone, interface)

    def set_disabled_permanent(self, interface):
        fw_zone, fw_settings = self.get_fw_zone_settings()
        fw_settings.removeInterface(interface)
        self.update_fw_settings(fw_zone, fw_settings)


class RichRuleTransaction(FirewallTransaction):
    """
    RichRuleTransaction
    """

    def __init__(self, module, action_args=None, zone=None, desired_state=None, permanent=False, immediate=False):
        super(RichRuleTransaction, self).__init__(
            module, action_args=action_args, desired_state=desired_state, zone=zone, permanent=permanent, immediate=immediate
        )

    def get_enabled_immediate(self, rule, timeout):
        # Convert the rule string to standard format
        # before checking whether it is present
        rule = str(Rich_Rule(rule_str=rule))
        if rule in self.fw.getRichRules(self.zone):
            return True
        else:
            return False

    def get_enabled_permanent(self, rule, timeout):
        fw_zone, fw_settings = self.get_fw_zone_settings()
        # Convert the rule string to standard format
        # before checking whether it is present
        rule = str(Rich_Rule(rule_str=rule))
        if rule in fw_settings.getRichRules():
            return True
        else:
            return False

    def set_enabled_immediate(self, rule, timeout):
        self.fw.addRichRule(self.zone, rule, timeout)

    def set_enabled_permanent(self, rule, timeout):
        fw_zone, fw_settings = self.get_fw_zone_settings()
        fw_settings.addRichRule(rule)
        self.update_fw_settings(fw_zone, fw_settings)

    def set_disabled_immediate(self, rule, timeout):
        self.fw.removeRichRule(self.zone, rule)

    def set_disabled_permanent(self, rule, timeout):
        fw_zone, fw_settings = self.get_fw_zone_settings()
        fw_settings.removeRichRule(rule)
        self.update_fw_settings(fw_zone, fw_settings)


class SourceTransaction(FirewallTransaction):
    """
    SourceTransaction
    """

    def __init__(self, module, action_args=None, zone=None, desired_state=None, permanent=False, immediate=False):
        super(SourceTransaction, self).__init__(
            module, action_args=action_args, desired_state=desired_state, zone=zone, permanent=permanent, immediate=immediate
        )

        self.enabled_msg = "Added %s to zone %s" % \
            (self.action_args[0], self.zone)

        self.disabled_msg = "Removed %s from zone %s" % \
            (self.action_args[0], self.zone)

    def get_enabled_immediate(self, source):
        if source in self.fw.getSources(self.zone):
            return True
        else:
            return False

    def get_enabled_permanent(self, source):
        fw_zone, fw_settings = self.get_fw_zone_settings()
        if source in fw_settings.getSources():
            return True
        else:
            return False

    def set_enabled_immediate(self, source):
        self.fw.addSource(self.zone, source)

    def set_enabled_permanent(self, source):
        fw_zone, fw_settings = self.get_fw_zone_settings()
        fw_settings.addSource(source)
        self.update_fw_settings(fw_zone, fw_settings)

    def set_disabled_immediate(self, source):
        self.fw.removeSource(self.zone, source)

    def set_disabled_permanent(self, source):
        fw_zone, fw_settings = self.get_fw_zone_settings()
        fw_settings.removeSource(source)
        self.update_fw_settings(fw_zone, fw_settings)


class ZoneTargetTransaction(FirewallTransaction):
    """
    ZoneTargetTransaction
    """

    def __init__(self, module, action_args=None, zone=None, desired_state=None,
                 permanent=True, immediate=False, enabled_values=None, disabled_values=None):
        super(ZoneTargetTransaction, self).__init__(
            module, action_args=action_args, desired_state=desired_state, zone=zone,
            permanent=permanent, immediate=immediate,
            enabled_values=enabled_values or ["present", "enabled"],
            disabled_values=disabled_values or ["absent", "disabled"])

        self.enabled_msg = "Set zone %s target to %s" % \
            (self.zone, action_args[0])

        self.disabled_msg = "Reset zone %s target to default" % \
            (self.zone)

        self.tx_not_permanent_error_msg = "Zone operations must be permanent. " \
            "Make sure you didn't set the 'permanent' flag to 'false' or the 'immediate' flag to 'true'."

    def get_enabled_immediate(self, target):
        self.module.fail_json(msg=self.tx_not_permanent_error_msg)

    def get_enabled_permanent(self, target):
        fw_zone, fw_settings = self.get_fw_zone_settings()
        current_target = fw_settings.getTarget()
        return (current_target == target)

    def set_enabled_immediate(self, target):
        self.module.fail_json(msg=self.tx_not_permanent_error_msg)

    def set_enabled_permanent(self, target):
        fw_zone, fw_settings = self.get_fw_zone_settings()
        fw_settings.setTarget(target)
        self.update_fw_settings(fw_zone, fw_settings)

    def set_disabled_immediate(self, target):
        self.module.fail_json(msg=self.tx_not_permanent_error_msg)

    def set_disabled_permanent(self, target):
        fw_zone, fw_settings = self.get_fw_zone_settings()
        fw_settings.setTarget("default")
        self.update_fw_settings(fw_zone, fw_settings)


class ZoneTransaction(FirewallTransaction):
    """
    ZoneTransaction
    """

    def __init__(self, module, action_args=None, zone=None, desired_state=None,
                 permanent=True, immediate=False, enabled_values=None, disabled_values=None):
        super(ZoneTransaction, self).__init__(
            module, action_args=action_args, desired_state=desired_state, zone=zone,
            permanent=permanent, immediate=immediate,
            enabled_values=enabled_values or ["present"],
            disabled_values=disabled_values or ["absent"])

        self.enabled_msg = "Added zone %s" % \
            (self.zone)

        self.disabled_msg = "Removed zone %s" % \
            (self.zone)

        self.tx_not_permanent_error_msg = "Zone operations must be permanent. " \
            "Make sure you didn't set the 'permanent' flag to 'false' or the 'immediate' flag to 'true'."

    def get_enabled_immediate(self):
        self.module.fail_json(msg=self.tx_not_permanent_error_msg)

    def get_enabled_permanent(self):
        if self.fw_offline:
            zones = self.fw.config.get_zones()
            zone_names = [self.fw.config.get_zone(z).name for z in zones]
        else:
            zones = self.fw.config().listZones()
            zone_names = [self.fw.config().getZone(z).get_property("name") for z in zones]
        return self.zone in zone_names

    def set_enabled_immediate(self):
        self.module.fail_json(msg=self.tx_not_permanent_error_msg)

    def set_enabled_permanent(self):
        if self.fw_offline:
            self.fw.config.new_zone(self.zone, FirewallClientZoneSettings().settings)
        else:
            self.fw.config().addZone(self.zone, FirewallClientZoneSettings())

    def set_disabled_immediate(self):
        self.module.fail_json(msg=self.tx_not_permanent_error_msg)

    def set_disabled_permanent(self):
        if self.fw_offline:
            zone = self.fw.config.get_zone(self.zone)
            self.fw.config.remove_zone(zone)
        else:
            zone_obj = self.fw.config().getZoneByName(self.zone)
            zone_obj.remove()


class ForwardPortTransaction(FirewallTransaction):
    """
    ForwardPortTransaction
    """

    def __init__(self, module, action_args=None, zone=None, desired_state=None, permanent=False, immediate=False):
        super(ForwardPortTransaction, self).__init__(
            module, action_args=action_args, desired_state=desired_state, zone=zone, permanent=permanent, immediate=immediate
        )

    def get_enabled_immediate(self, port, proto, toport, toaddr, timeout):
        if self.fw_offline:
            dummy, fw_settings = self.get_fw_zone_settings()
            return fw_settings.queryForwardPort(port=port, protocol=proto, to_port=toport, to_addr=toaddr)
        return self.fw.queryForwardPort(zone=self.zone, port=port, protocol=proto, toport=toport, toaddr=toaddr)

    def get_enabled_permanent(self, port, proto, toport, toaddr, timeout):
        dummy, fw_settings = self.get_fw_zone_settings()
        return fw_settings.queryForwardPort(port=port, protocol=proto, to_port=toport, to_addr=toaddr)

    def set_enabled_immediate(self, port, proto, toport, toaddr, timeout):
        self.fw.addForwardPort(self.zone, port, proto, toport, toaddr, timeout)

    def set_enabled_permanent(self, port, proto, toport, toaddr, timeout):
        fw_zone, fw_settings = self.get_fw_zone_settings()
        fw_settings.addForwardPort(port, proto, toport, toaddr)
        self.update_fw_settings(fw_zone, fw_settings)

    def set_disabled_immediate(self, port, proto, toport, toaddr, timeout):
        self.fw.removeForwardPort(self.zone, port, proto, toport, toaddr)

    def set_disabled_permanent(self, port, proto, toport, toaddr, timeout):
        fw_zone, fw_settings = self.get_fw_zone_settings()
        fw_settings.removeForwardPort(port, proto, toport, toaddr)
        self.update_fw_settings(fw_zone, fw_settings)


def main():

    module = AnsibleModule(
        argument_spec=dict(
            icmp_block=dict(type='str'),
            icmp_block_inversion=dict(type='bool'),
            service=dict(type='str'),
            protocol=dict(type='str'),
            port=dict(type='str'),
            port_forward=dict(type='list', elements='dict'),
            rich_rule=dict(type='str'),
            zone=dict(type='str'),
            immediate=dict(type='bool', default=False),
            source=dict(type='str'),
            permanent=dict(type='bool', default=False),
            state=dict(type='str', required=True, choices=['absent', 'disabled', 'enabled', 'present']),
            timeout=dict(type='int', default=0),
            interface=dict(type='str'),
            forward=dict(type='bool'),
            masquerade=dict(type='bool'),
            offline=dict(type='bool', default=False),
            target=dict(type='str', choices=['default', 'ACCEPT', 'DROP', '%%REJECT%%']),
        ),
        supports_check_mode=True,
        required_by=dict(
            interface=('zone',),
            target=('zone',),
            source=('permanent',),
        ),
        mutually_exclusive=[
            ['icmp_block', 'icmp_block_inversion', 'service', 'protocol', 'port', 'port_forward', 'rich_rule',
             'interface', 'forward', 'masquerade', 'source', 'target']
        ],
    )

    permanent = module.params['permanent']
    desired_state = module.params['state']
    immediate = module.params['immediate']
    timeout = module.params['timeout']
    interface = module.params['interface']
    forward = module.params['forward']
    masquerade = module.params['masquerade']
    offline = module.params['offline']

    # Sanity checks
    FirewallTransaction.sanity_check(module)

    # `offline`, `immediate`, and `permanent` have a weird twisty relationship.
    if offline:
        # specifying offline without permanent makes no sense
        if not permanent:
            module.fail_json(msg='offline cannot be enabled unless permanent changes are allowed')

        # offline overrides immediate to false if firewalld is offline
        if fw_offline:
            immediate = False

    # immediate defaults to true if permanent is not enabled
    if not permanent and not immediate:
        immediate = True

    if immediate and fw_offline:
        module.fail_json(msg='firewall is not currently running, unable to perform immediate actions without a running firewall daemon')

    # Verify required params are provided
    changed = False
    msgs = []
    icmp_block = module.params['icmp_block']
    icmp_block_inversion = module.params['icmp_block_inversion']
    service = module.params['service']
    protocol = module.params['protocol']
    rich_rule = module.params['rich_rule']
    source = module.params['source']
    zone = module.params['zone']
    target = module.params['target']

    port = None
    if module.params['port'] is not None:
        if '/' in module.params['port']:
            port, port_protocol = module.params['port'].strip().split('/')
        else:
            port_protocol = None
        if not port_protocol:
            module.fail_json(msg='improper port format (missing protocol?)')
    else:
        port_protocol = None

    port_forward_toaddr = ''
    port_forward = None
    if module.params['port_forward'] is not None:
        if len(module.params['port_forward']) > 1:
            module.fail_json(msg='Only one port forward supported at a time')
        port_forward = module.params['port_forward'][0]
        if 'port' not in port_forward:
            module.fail_json(msg='port must be specified for port forward')
        if 'proto' not in port_forward:
            module.fail_json(msg='proto udp/tcp must be specified for port forward')
        if 'toport' not in port_forward:
            module.fail_json(msg='toport must be specified for port forward')
        if 'toaddr' in port_forward:
            port_forward_toaddr = port_forward['toaddr']

    modification = False
    if any([icmp_block, icmp_block_inversion, service, protocol, port, port_forward, rich_rule,
            interface, forward, masquerade, source, target]):
        modification = True
    if modification and desired_state in ['absent', 'present'] and target is None:
        module.fail_json(
            msg='absent and present state can only be used in zone level operations'
        )

    if icmp_block is not None:

        transaction = IcmpBlockTransaction(
            module,
            action_args=(icmp_block, timeout),
            zone=zone,
            desired_state=desired_state,
            permanent=permanent,
            immediate=immediate,
        )

        changed, transaction_msgs = transaction.run()
        msgs = msgs + transaction_msgs
        if changed is True:
            msgs.append("Changed icmp-block %s to %s" % (icmp_block, desired_state))

    if icmp_block_inversion is not None:
        expected_state = 'enabled' if (desired_state == 'enabled') == icmp_block_inversion else 'disabled'
        transaction = IcmpBlockInversionTransaction(
            module,
            action_args=(),
            zone=zone,
            desired_state=expected_state,
            permanent=permanent,
            immediate=immediate,
        )

        changed, transaction_msgs = transaction.run()
        msgs = msgs + transaction_msgs
        if changed is True:
            msgs.append("Changed icmp-block-inversion %s to %s" % (icmp_block_inversion, desired_state))

    if service is not None:

        transaction = ServiceTransaction(
            module,
            action_args=(service, timeout),
            zone=zone,
            desired_state=desired_state,
            permanent=permanent,
            immediate=immediate,
        )

        changed, transaction_msgs = transaction.run()
        msgs = msgs + transaction_msgs
        if changed is True:
            msgs.append("Changed service %s to %s" % (service, desired_state))

    if protocol is not None:

        transaction = ProtocolTransaction(
            module,
            action_args=(protocol, timeout),
            zone=zone,
            desired_state=desired_state,
            permanent=permanent,
            immediate=immediate,
        )

        changed, transaction_msgs = transaction.run()
        msgs = msgs + transaction_msgs
        if changed is True:
            msgs.append("Changed protocol %s to %s" % (protocol, desired_state))

    if source is not None:

        transaction = SourceTransaction(
            module,
            action_args=(source,),
            zone=zone,
            desired_state=desired_state,
            permanent=permanent,
            immediate=immediate,
        )

        changed, transaction_msgs = transaction.run()
        msgs = msgs + transaction_msgs

    if port is not None:

        transaction = PortTransaction(
            module,
            action_args=(port, port_protocol, timeout),
            zone=zone,
            desired_state=desired_state,
            permanent=permanent,
            immediate=immediate,
        )

        changed, transaction_msgs = transaction.run()
        msgs = msgs + transaction_msgs
        if changed is True:
            msgs.append(
                "Changed port %s to %s" % (
                    "%s/%s" % (port, port_protocol), desired_state
                )
            )

    if port_forward is not None:
        transaction = ForwardPortTransaction(
            module,
            action_args=(str(port_forward['port']), port_forward['proto'],
                         str(port_forward['toport']), port_forward_toaddr, timeout),
            zone=zone,
            desired_state=desired_state,
            permanent=permanent,
            immediate=immediate
        )

        changed, transaction_msgs = transaction.run()
        msgs = msgs + transaction_msgs
        if changed is True:
            msgs.append(
                "Changed port_forward %s to %s" % (
                    "port=%s:proto=%s:toport=%s:toaddr=%s" % (
                        port_forward['port'], port_forward['proto'],
                        port_forward['toport'], port_forward_toaddr
                    ), desired_state
                )
            )

    if rich_rule is not None:

        transaction = RichRuleTransaction(
            module,
            action_args=(rich_rule, timeout),
            zone=zone,
            desired_state=desired_state,
            permanent=permanent,
            immediate=immediate,
        )

        changed, transaction_msgs = transaction.run()
        msgs = msgs + transaction_msgs
        if changed is True:
            msgs.append("Changed rich_rule %s to %s" % (rich_rule, desired_state))

    if interface is not None:

        transaction = InterfaceTransaction(
            module,
            action_args=(interface,),
            zone=zone,
            desired_state=desired_state,
            permanent=permanent,
            immediate=immediate,
        )

        changed, transaction_msgs = transaction.run()
        msgs = msgs + transaction_msgs

    if forward is not None:
        expected_state = 'enabled' if (desired_state == 'enabled') == forward else 'disabled'
        transaction = ForwardTransaction(
            module,
            action_args=(),
            zone=zone,
            desired_state=expected_state,
            permanent=permanent,
            immediate=immediate,
        )

        changed, transaction_msgs = transaction.run()
        msgs = msgs + transaction_msgs

    if masquerade is not None:
        expected_state = 'enabled' if (desired_state == 'enabled') == masquerade else 'disabled'
        transaction = MasqueradeTransaction(
            module,
            action_args=(),
            zone=zone,
            desired_state=expected_state,
            permanent=permanent,
            immediate=immediate,
        )

        changed, transaction_msgs = transaction.run()
        msgs = msgs + transaction_msgs

    if target is not None:

        transaction = ZoneTargetTransaction(
            module,
            action_args=(target,),
            zone=zone,
            desired_state=desired_state,
            permanent=permanent,
            immediate=immediate,
        )

        changed, transaction_msgs = transaction.run()
        msgs = msgs + transaction_msgs

    ''' If there are no changes within the zone we are operating on the zone itself '''
    if not modification and desired_state in ['absent', 'present']:

        transaction = ZoneTransaction(
            module,
            action_args=(),
            zone=zone,
            desired_state=desired_state,
            permanent=permanent,
            immediate=immediate,
        )

        changed, transaction_msgs = transaction.run()
        msgs = msgs + transaction_msgs
        if changed is True:
            msgs.append("Changed zone %s to %s" % (zone, desired_state))

    if fw_offline:
        msgs.append("(offline operation: only on-disk configs were altered)")

    module.exit_json(changed=changed, msg=', '.join(msgs))


if __name__ == '__main__':
    main()
