# autoware_ad_api_msgs

## ResponseStatus

This message is a response status commonly used in the service type API. For details, see [the response status][docs-response-status].
Each API can define its own status codes.
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

This message is for the interface version of the set of AD APIs. For details, see [the interface feature][docs-interface].

<!-- link -->

[docs-response-status]: https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-interfaces/#response-status
[docs-interface]: https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-interfaces/ad-api/features/interface/
