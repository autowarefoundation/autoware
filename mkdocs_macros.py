import json

from tabulate import tabulate

# This file is for defining macros for mkdocs-macros plugin
# Check https://mkdocs-macros-plugin.readthedocs.io/en/latest/macros/ for the details


def format_param_type(param_type):
    if param_type == "number":
        return "float"
    else:
        return param_type


def format_param_range(param):
    list_of_range = []
    if "enum" in param.keys():
        list_of_range.append(param["enum"])
    if "minimum" in param.keys():
        list_of_range.append("≥" + str(param["minimum"]))
    if "exclusiveMinimum" in param.keys():
        list_of_range.append(">" + str(param["exclusiveMinimum"]))
    if "maximum" in param.keys():
        list_of_range.append("≤" + str(param["maximum"]))
    if "exclusiveMaximum" in param.keys():
        list_of_range.append("<" + str(param["exclusiveMaximum"]))
    if "exclusive" in param.keys():
        list_of_range.append("≠" + str(param["exclusive"]))

    if len(list_of_range) == 0:
        return "N/A"
    else:
        range_in_text = ""
        for item in list_of_range:
            if range_in_text != "":
                range_in_text += "<br/>"
            range_in_text += str(item)
        return range_in_text


def extract_parameter_info(parameters, namespace=""):
    params = []
    for k, v in parameters.items():
        if "$ref" in v.keys():
            continue
        if v["type"] != "object":
            param = {}
            param["Name"] = namespace + k
            param["Type"] = format_param_type(v["type"])
            param["Description"] = v["description"]
            param["Default"] = v["default"]
            param["Range"] = format_param_range(v)
            params.append(param)
        else:  # if the object is namespace, then dive deeper in to json value
            params.extend(extract_parameter_info(v["properties"], k + "."))
    return params


def format_json(json_data):
    parameters = list(json_data["definitions"].values())[0]["properties"]
    # cspell: ignore tablefmt
    markdown_table = tabulate(extract_parameter_info(parameters), headers="keys", tablefmt="github")
    return markdown_table


def define_env(env):
    @env.macro
    def json_to_markdown(json_schema_file_path):
        with open(json_schema_file_path) as f:
            data = json.load(f)
            return format_json(data)
