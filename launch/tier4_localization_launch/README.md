# tier4_localization_launch

## Structure

![tier4_localization_launch](./localization_launch.drawio.svg)

## Package Dependencies

Please see `<exec_depend>` in `package.xml`.

## Usage

You can include as follows in `*.launch.xml` to use `localization.launch.xml`.

```xml
  <include file="$(find-pkg-share tier4_localization_launch)/launch/localization.launch.xml">
  </include>
```

## Notes

There are some `param.yaml` files in `config` directory.

```bash
ndt_scan_matcher.param.yaml
```
