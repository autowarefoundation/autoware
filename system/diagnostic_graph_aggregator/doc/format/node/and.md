# And

The `and` object is a node that is evaluated as the maximum error level of the input nodes.
Note that error level `stale` is treated as `error`.

## Format

| Name   | Type                                   | Required | Description                                                  |
| ------ | -------------------------------------- | -------- | ------------------------------------------------------------ |
| `type` | <code>string</code>                    | yes      | Specify `and` or `short-circuit-and` when using this object. |
| `list` | <code>list\[[node](../node.md)]</code> | yes      | List of input node objects.                                  |

## Short-circuit evaluation

!!! warning

    The`short-circuit-and` is work in progress (WIP).
