group "default" {
  targets = [
    "core-common-devel",
    "core",
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
    "universe-visualization-devel",
    "universe-visualization",
    "universe-api-devel",
    "universe-api",
    "universe-devel",
    "universe"
  ]
}

// For docker/metadata-action
target "docker-metadata-action-core-common-devel" {}
target "docker-metadata-action-core" {}
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
target "docker-metadata-action-universe-visualization-devel" {}
target "docker-metadata-action-universe-visualization" {}
target "docker-metadata-action-universe-api-devel" {}
target "docker-metadata-action-universe-api" {}
target "docker-metadata-action-universe-devel" {}
target "docker-metadata-action-universe" {}

target "core-common-devel" {
  inherits = ["docker-metadata-action-core-common-devel"]
  dockerfile = "docker/Dockerfile"
  target = "core-common-devel"
}

target "core" {
  inherits = ["docker-metadata-action-core"]
  dockerfile = "docker/Dockerfile"
  target = "core"
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

target "universe-visualization-devel" {
  inherits = ["docker-metadata-action-universe-visualization-devel"]
  dockerfile = "docker/Dockerfile"
  target = "universe-visualization-devel"
}

target "universe-visualization" {
  inherits = ["docker-metadata-action-universe-visualization"]
  dockerfile = "docker/Dockerfile"
  target = "universe-visualization"
}

target "universe-api-devel" {
  inherits = ["docker-metadata-action-universe-api-devel"]
  dockerfile = "docker/Dockerfile"
  target = "universe-api-devel"
}

target "universe-api" {
  inherits = ["docker-metadata-action-universe-api"]
  dockerfile = "docker/Dockerfile"
  target = "universe-api"
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
