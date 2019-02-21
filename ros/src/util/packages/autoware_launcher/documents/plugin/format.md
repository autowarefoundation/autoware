# Autoware Launcher Plugin File Format

The plugin file is yaml file. Currently, there are the following versions.
* Plugin Version 0.1 (experimental)

# Plugin Version 0.1

## Plugin File
The file is the following dictionary.

### Node Type

| Key      | Type   | Value/Comment |
|----------|--------|---------------|
| format   | string | Fixed value: Autoware Launcher Plugin Version 0.1 |
| exts     | [Extended Data List](#extended_data) | Extended Data Definitions |
| rules    | [Children Rule List](#children_rule) | Children Rule Definitions |

### Leaf Type

| Key      | Type   | Value/Comment |
|----------|--------|---------------|
| format   | string | Fixed value: Autoware Launcher Plugin Version 0.1 |
| rosxml   | string | Path to roslaunch xml file  |
| exts     | [Extended Data List](#extended_data) | Extended Data Definitions |
| args     | [Argument Data List](#argument_data) | Argument Data Definitions |


## <a id="argument_data">Argument Data</a> / <a id="extended_data">Extended Data</a>

dictionary

| Key     | Type          | Value/Comment |
|---------|---------------|---------------|
| name    | string        | Data name     |
| type    | string        | Data type (Enum: [str, int, real, bool]) |
| list    | string / null | List format: (Enum: [null, space, yaml]) |
| default |               | Default value |

## <a id="children_rule">Children Rule</a>

dictionary

| Key     | Type          | Value/Comment |
|---------|---------------|---------------|
| name    | string        | Child node name  |
| list    | bool          | Is multiple node |
| plugin  | [Reference Rule](#ref_rule) | Child type |

## <a id="ref_rule">Reference Rule</a>

list (If a type other than the list is specified, it is converted to a list with 1 element.)

Each element of the list must be:
* Path Type
* Scan Type

### Path Type

Type is string. Specify the path to the plugin file.

### Scan Type

Type is dictionary. Scan all plugin files from the specified root path.

| Key     | Type          | Value/Comment   |
|---------|---------------|-----------------|
| scan    | string        | Scan root path  |



