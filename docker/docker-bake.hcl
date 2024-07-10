group "default" {
  targets = ["base", "autoware-core", "autoware-universe", "devel", "runtime"]
}

// For docker/metadata-action
target "docker-metadata-action-base" {}
target "docker-metadata-action-autoware-core" {}
target "docker-metadata-action-autoware-universe" {}
target "docker-metadata-action-devel" {}
target "docker-metadata-action-runtime" {}

target "base" {
  inherits = ["docker-metadata-action-base"]
  dockerfile = "docker/Dockerfile"
  target = "base"
}

target "autoware-core" {
  inherits = ["docker-metadata-action-autoware-core"]
  dockerfile = "docker/Dockerfile"
  target = "autoware-core"
}

target "autoware-universe" {
  inherits = ["docker-metadata-action-autoware-universe"]
  dockerfile = "docker/Dockerfile"
  target = "autoware-universe"
}

target "devel" {
  inherits = ["docker-metadata-action-devel"]
  dockerfile = "docker/Dockerfile"
  target = "devel"
}

target "runtime" {
  inherits = ["docker-metadata-action-runtime"]
  dockerfile = "docker/Dockerfile"
  target = "runtime"
}
