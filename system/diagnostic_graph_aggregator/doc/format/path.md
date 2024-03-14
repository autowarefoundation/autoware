# Path

The path object specifies the file path of the subgraph to be imported.
The structure of the subgraph file should be [graph object](./graph.md).

## Format

| Name   | Type     | Required | Description                    |
| ------ | -------- | -------- | ------------------------------ |
| `path` | `string` | yes      | The file path of the subgraph. |

## Substitutions

File paths can contain substitutions like ROS 2 launch. The supported substitutions are as follows.

| Substitution                  | Description                      |
| ----------------------------- | -------------------------------- |
| `$(dirname)`                  | The path of this file directory. |
| `$(find-pkg-share <package>)` | The path of the package.         |
