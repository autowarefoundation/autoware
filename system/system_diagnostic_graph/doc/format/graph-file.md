# GraphFile

GraphFile is the top level object that makes up the configuration file.

## Format

| Name  | Type                    | Required | Description                    |
| ----- | ----------------------- | -------- | ------------------------------ |
| files | List<[Path](./path.md)> | no       | Paths of the files to include. |
| nodes | List<[Node](./node.md)> | no       | Nodes of the diagnostic graph. |
