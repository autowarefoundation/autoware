# Edit

The `edit` is a base object that edits the existing diagnostic graph.
Any derived object can be used where a edit object is required.

## Format

| Name   | Type     | Required | Description                                       |
| ------ | -------- | -------- | ------------------------------------------------- |
| `type` | `string` | yes      | The string indicating the type of derived object. |

## Derived objects

- [remove](./edit/remove.md)
