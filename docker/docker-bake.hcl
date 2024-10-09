group "default" {
  targets = [
    "base",
    "core-devel",
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
target "docker-metadata-action-base" {}
target "docker-metadata-action-core-devel" {}
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

target "base" {
  inherits = ["docker-metadata-action-base"]
  dockerfile = "docker/Dockerfile"
  target = "base"
}

target "core-devel" {
  inherits = ["docker-metadata-action-core-devel"]
  dockerfile = "docker/Dockerfile"
  target = "core-devel"
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
