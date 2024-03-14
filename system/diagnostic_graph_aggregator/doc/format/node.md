# Node

The `node` is a base object that makes up the diagnostic graph.
Any derived object can be used where a node object is required.

## Format

| Name   | Type     | Required | Description                                       |
| ------ | -------- | -------- | ------------------------------------------------- |
| `type` | `string` | yes      | The string indicating the type of derived object. |
| `path` | `string` | no       | Any string to reference from other nodes.         |

## Derived objects

- [diag](./node/diag.md)
- [and](./node/and.md)
- [or](./node/or.md)
- [remapping](./node/remap.md)
  - warn-to-ok
  - warn-to-error
- [constant](./node/const.md)
  - ok
  - warn
  - error
  - stale
