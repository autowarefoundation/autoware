group "default" {
  targets = ["base", "core", "universe", "devel", "universe"]
}

// For docker/metadata-action
target "docker-metadata-action-base" {}
target "docker-metadata-action-core-devel" {}
target "docker-metadata-action-universe-devel" {}
target "docker-metadata-action-devel" {}
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

target "universe-devel" {
  inherits = ["docker-metadata-action-universe-devel"]
  dockerfile = "docker/Dockerfile"
  target = "universe-devel"
}

target "devel" {
  inherits = ["docker-metadata-action-devel"]
  dockerfile = "docker/Dockerfile"
  target = "devel"
}

target "universe" {
  inherits = ["docker-metadata-action-universe"]
  dockerfile = "docker/Dockerfile"
  target = "universe"
}
