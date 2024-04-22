# Unit

The `unit` is a base object that makes up the diagnostic graph.
Any derived object can be used where a unit object is required.

## Format

| Name   | Type     | Required | Description                                       |
| ------ | -------- | -------- | ------------------------------------------------- |
| `type` | `string` | yes      | The string indicating the type of derived object. |
| `path` | `string` | no       | Any string to reference from other units.         |

## Derived objects

- [diag](./unit/diag.md)
- [and](./unit/and.md)
- [or](./unit/or.md)
- [remapping](./unit/remap.md)
  - warn-to-ok
  - warn-to-error
- [constant](./unit/const.md)
  - ok
  - warn
  - error
  - stale
