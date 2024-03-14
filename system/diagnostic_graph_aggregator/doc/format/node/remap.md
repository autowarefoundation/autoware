# Constant

!!! warning

    This object is under development. It may be removed in the future.

The remapping object is a node that converts error levels.

## Format

| Name   | Type                                   | Required | Description                                          |
| ------ | -------------------------------------- | -------- | ---------------------------------------------------- |
| `type` | `string`                               | yes      | Specify remapping type when using this object.       |
| `list` | <code>list\[[node](../node.md)]</code> | yes      | List of input node objects. The list size must be 1. |

## Remapping types

The supported remapping types are as follows.

- `warn-to-ok`
- `warn-to-error`
