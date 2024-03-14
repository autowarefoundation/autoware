# Graph

The graph object is the top level structure that makes up the configuration file.

## Format

| Name    | Type                                   | Required | Description                                       |
| ------- | -------------------------------------- | -------- | ------------------------------------------------- |
| `files` | <code>list\[[path](./path.md)\]</code> | no       | List of path objects for importing subgraphs.     |
| `nodes` | <code>list\[[node](./node.md)\]</code> | no       | List of node objects that make up the graph.      |
| `edits` | <code>list\[[edit](./edit.md)\]</code> | no       | List of edit objects to partially edit the graph. |
