# Graph

The graph object is the top level structure that makes up the configuration file.

## Format

| Name    | Type                                   | Required | Description                                       |
| ------- | -------------------------------------- | -------- | ------------------------------------------------- |
| `files` | <code>list\[[path](./path.md)\]</code> | no       | List of path objects for importing subgraphs.     |
| `units` | <code>list\[[unit](./unit.md)\]</code> | no       | List of unit objects that make up the graph.      |
| `edits` | <code>list\[[edit](./edit.md)\]</code> | no       | List of edit objects to partially edit the graph. |
