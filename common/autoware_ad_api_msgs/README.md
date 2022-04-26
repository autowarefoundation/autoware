# autoware_ad_api_msgs

## InterfaceVersion

This API provides the interface version of the set of AD APIs.
It follows [Semantic Versioning][semver] in order to provide an intuitive understanding of the changes between versions.

### Use cases

Considering the product life cycle, there will be multiple vehicles that use different versions of the AD API due to changes in requirements or some improvements.
For example, a vehicle uses `v1` for stability and another vehicle uses `v2` for more functionality.

In that situation, the AD API users such as developers of a Web service have to switch the application behavior based on the version that each vehicle uses.

<!-- link -->

[semver]: https://semver.org/
