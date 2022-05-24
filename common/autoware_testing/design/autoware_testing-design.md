# autoware_testing

This is the design document for the `autoware_testing` package.

## Purpose / Use cases

The package aims to provide a unified way to add standard testing functionality to the package, currently supporting:

- Smoke testing (`add_smoke_test`): launch a node with default configuration and ensure that it starts up and does not crash.

## Design

Uses `ros_testing` (which is an extension of `launch_testing`) and provides some parametrized, reusable standard tests to run.

## Assumptions / Known limits

Parametrization is limited to package, executable names, parameters filename and executable arguments. Test namespace is set as 'test'.
Parameters file for the package is expected to be in `param` directory inside package.

## Inputs / Outputs / API

To add a smoke test to your package tests, add test dependency on `autoware_testing` to `package.xml`

```xml
<test_depend>autoware_testing</test_depend>
```

and add the following two lines to `CMakeLists.txt` in the `IF (BUILD_TESTING)` section:

```cmake
find_package(autoware_testing REQUIRED)
add_smoke_test(<package_name> <executable_name> [PARAM_FILENAME <param_filename>] [EXECUTABLE_ARGUMENTS <arguments>])
```

Where

`<package_name>` - [required] tested node package name.

`<executable_name>` - [required] tested node executable name.

`<param_filename>` - [optional] param filename. Default value is `test.param.yaml`. Required mostly in situation where there are multiple smoke tests in a package and each requires different parameters set

`<arguments>` - [optional] arguments passed to executable. By default no arguments are passed.

which adds `<executable_name>_smoke_test` test to suite.

Example test result:

```text
build/<package_name>/test_results/<package_name>/<executable_name>_smoke_test.xunit.xml: 1 test, 0 errors, 0 failures, 0 skipped
```

## References / External links

- <https://en.wikipedia.org/wiki/Smoke_testing_(software)>
- <https://github.com/ros2/ros_testing>
- <https://github.com/ros2/launch/blob/master/launch_testing>

## Future extensions / Unimplemented parts

- Adding more types of standard tests.

## Related issues

- Issue #700: add smoke test
- Issue #1224: Port other packages with smoke tests to use `autoware_testing`
