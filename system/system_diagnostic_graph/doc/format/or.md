# Unit

Or is a node that is evaluated as the OR of the input nodes.

## Format

| Name | Type                                       | Required | Description                          |
| ---- | ------------------------------------------ | -------- | ------------------------------------ |
| type | string                                     | yes      | Specify `or` when using this object. |
| name | string                                     | yes      | Name of diagnostic status.           |
| list | List<[Diag](./diag.md)\|[Unit](./unit.md)> | yes      | List of input node references.       |
