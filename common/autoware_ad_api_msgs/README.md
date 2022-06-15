# autoware_ad_api_msgs

## ResponseStatus

This message is a response status commonly used in the service type API. Each API can define its own status codes.
The status codes are primarily used to indicate the error cause, such as invalid parameter and timeout.
If the API succeeds, set success to true, code to zero, and message to the empty string.
Alternatively, codes and messages can be used for warnings or additional information.
If the API fails, set success to false, code to the related status code, and message to the information.
The status code zero is reserved for success. The status code 50000 or over are also reserved for typical cases.

| Name       |  Code | Description                          |
| ---------- | ----: | ------------------------------------ |
| SUCCESS    |     0 | This API has completed successfully. |
| DEPRECATED | 50000 | This API is deprecated.              |

## InterfaceVersion

Considering the product life cycle, there may be multiple vehicles using different versions of the AD API due to changes in requirements or functional improvements. For example, one vehicle uses `v1` for stability and another vehicle uses `v2` to enable more advanced functionality.

In that situation, the AD API users such as developers of a web service have to switch the application behavior based on the version that each vehicle uses.
The version of AD API follows [Semantic Versioning][semver] in order to provide an intuitive understanding of the changes between versions.

<!-- link -->

[semver]: https://semver.org/
