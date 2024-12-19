group "default" {
  targets = [
    "rosdep-depend",
    "rosdep-universe-depend",
    "rosdep-universe-sensing-perception-depend",
    "rosdep-universe-vehicle-system-depend",
    "rosdep-universe-planning-control-depend",
    "rosdep-universe-localization-mapping-depend",
    "core-devel",
    "universe-common-devel",
    "universe-sensing-perception-devel",
    "universe-sensing-perception",
    "universe-localization-mapping-devel",
    "universe-localization-mapping",
    "universe-planning-control-devel",
    "universe-planning-control",
    "universe-vehicle-system-devel",
    "universe-vehicle-system",
    "universe-devel",
    "universe"
  ]
}

// For docker/metadata-action
target "docker-metadata-action-rosdep-depend" {}
target "docker-metadata-action-rosdep-universe-depend" {}
target "docker-metadata-action-rosdep-universe-sensing-perception-depend" {}
target "docker-metadata-action-rosdep-universe-vehicle-system-depend" {}
target "docker-metadata-action-rosdep-universe-planning-control-depend" {}
target "docker-metadata-action-rosdep-universe-localization-mapping-depend" {}
target "docker-metadata-action-core-devel" {}
target "docker-metadata-action-universe-common-devel" {}
target "docker-metadata-action-universe-sensing-perception-devel" {}
target "docker-metadata-action-universe-sensing-perception" {}
target "docker-metadata-action-universe-localization-mapping-devel" {}
target "docker-metadata-action-universe-localization-mapping" {}
target "docker-metadata-action-universe-planning-control-devel" {}
target "docker-metadata-action-universe-planning-control" {}
target "docker-metadata-action-universe-vehicle-system-devel" {}
target "docker-metadata-action-universe-vehicle-system" {}
target "docker-metadata-action-universe-devel" {}
target "docker-metadata-action-universe" {}

target "rosdep-depend" {
  inherits = ["docker-metadata-action-rosdep-depend"]
  dockerfile = "docker/Dockerfile"
  target = "rosdep-depend"
}

target "rosdep-universe-depend" {
  inherits = ["docker-metadata-action-rosdep-universe-depend"]
  dockerfile = "docker/Dockerfile"
  target = "rosdep-universe-depend"
}

target "rosdep-universe-sensing-perception-depend" {
  inherits = ["docker-metadata-action-rosdep-universe-sensing-perception-depend"]
  dockerfile = "docker/Dockerfile"
  target = "rosdep-universe-sensing-perception-depend"
}

target "rosdep-universe-vehicle-system-depend" {
  inherits = ["docker-metadata-action-rosdep-universe-vehicle-system-depend"]
  dockerfile = "docker/Dockerfile"
  target = "rosdep-universe-vehicle-system-depend"
}

target "rosdep-universe-planning-control-depend" {
  inherits = ["docker-metadata-action-rosdep-universe-planning-control-depend"]
  dockerfile = "docker/Dockerfile"
  target = "rosdep-universe-planning-control-depend"
}

target "rosdep-universe-localization-mapping-depend" {
  inherits = ["docker-metadata-action-rosdep-universe-localization-mapping-depend"]
  dockerfile = "docker/Dockerfile"
  target = "rosdep-universe-localization-mapping-depend"
}
target "core-devel" {
  inherits = ["docker-metadata-action-core-devel"]
  dockerfile = "docker/Dockerfile"
  target = "core-devel"
}

target "universe-common-devel" {
  inherits = ["docker-metadata-action-universe-common-devel"]
  dockerfile = "docker/Dockerfile"
  target = "universe-common-devel"
}

target "universe-sensing-perception-devel" {
  inherits = ["docker-metadata-action-universe-sensing-perception-devel"]
  dockerfile = "docker/Dockerfile"
  target = "universe-sensing-perception-devel"
}

target "universe-sensing-perception" {
  inherits = ["docker-metadata-action-universe-sensing-perception"]
  dockerfile = "docker/Dockerfile"
  target = "universe-sensing-perception"
}

target "universe-localization-mapping-devel" {
  inherits = ["docker-metadata-action-universe-localization-mapping-devel"]
  dockerfile = "docker/Dockerfile"
  target = "universe-localization-mapping-devel"
}

target "universe-localization-mapping" {
  inherits = ["docker-metadata-action-universe-localization-mapping"]
  dockerfile = "docker/Dockerfile"
  target = "universe-localization-mapping"
}

target "universe-planning-control-devel" {
  inherits = ["docker-metadata-action-universe-planning-control-devel"]
  dockerfile = "docker/Dockerfile"
  target = "universe-planning-control-devel"
}

target "universe-planning-control" {
  inherits = ["docker-metadata-action-universe-planning-control"]
  dockerfile = "docker/Dockerfile"
  target = "universe-planning-control"
}

target "universe-vehicle-system-devel" {
  inherits = ["docker-metadata-action-universe-vehicle-system-devel"]
  dockerfile = "docker/Dockerfile"
  target = "universe-vehicle-system-devel"
}

target "universe-vehicle-system" {
  inherits = ["docker-metadata-action-universe-vehicle-system"]
  dockerfile = "docker/Dockerfile"
  target = "universe-vehicle-system"
}

target "universe-devel" {
  inherits = ["docker-metadata-action-universe-devel"]
  dockerfile = "docker/Dockerfile"
  target = "universe-devel"
}

target "universe" {
  inherits = ["docker-metadata-action-universe"]
  dockerfile = "docker/Dockerfile"
  target = "universe"
}
