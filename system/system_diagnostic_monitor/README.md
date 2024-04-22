# system_diagnostic_monitor

This package contains default configurations of diagnostic graph and scripts for system integration.

## Configs

| Name                                              | Description                                                  |
| ------------------------------------------------- | ------------------------------------------------------------ |
| [autoware-main.yaml](./config/autoware-main.yaml) | Diagnostic graphs for basic monitoring of Autoware.          |
| [autoware-psim.yaml](./config/autoware-psim.yaml) | Diagnostic graph with some units disabled for the simulator. |

## Scripts

| Name                                                                   | Description                                         |
| ---------------------------------------------------------------------- | --------------------------------------------------- |
| [component_state_diagnostics](./script/component_state_diagnostics.py) | Node that converts component states to diagnostics. |
