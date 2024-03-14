# Or

The `or` object is a node that is evaluated as the minimum error level of the input nodes.
Note that error level `stale` is treated as `error`.

## Format

| Name   | Type                                   | Required | Description                          |
| ------ | -------------------------------------- | -------- | ------------------------------------ |
| `type` | <code>string</code>                    | yes      | Specify `or` when using this object. |
| `list` | <code>list\[[node](../node.md)]</code> | yes      | List of input node objects.          |
