# Or

The `or` object is a unit that is evaluated as the minimum error level of the input units.
Note that error level `stale` is treated as `error`.

## Format

| Name   | Type                                   | Required | Description                          |
| ------ | -------------------------------------- | -------- | ------------------------------------ |
| `type` | <code>string</code>                    | yes      | Specify `or` when using this object. |
| `list` | <code>list\[[unit](../unit.md)]</code> | yes      | List of input unit objects.          |
