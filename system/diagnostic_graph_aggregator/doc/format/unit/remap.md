# Constant

!!! warning

    This object is under development. It may be removed in the future.

The remapping object is a unit that converts error levels.

## Format

| Name   | Type                            | Required | Description                                    |
| ------ | ------------------------------- | -------- | ---------------------------------------------- |
| `type` | `string`                        | yes      | Specify remapping type when using this object. |
| `item` | <code>[unit](../unit.md)</code> | yes      | Input unit object.                             |

## Remapping types

The supported remapping types are as follows.

- `warn-to-ok`
- `warn-to-error`
