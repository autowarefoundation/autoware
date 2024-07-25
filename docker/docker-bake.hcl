group "default" {
  targets = ["base", "prebuilt", "devel", "runtime"]
}

// For docker/metadata-action
target "docker-metadata-action-base" {}
target "docker-metadata-action-prebuilt" {}
target "docker-metadata-action-devel" {}
target "docker-metadata-action-runtime" {}

target "base" {
  inherits = ["docker-metadata-action-base"]
  dockerfile = "docker/Dockerfile"
  target = "base"
}

target "prebuilt" {
  inherits = ["docker-metadata-action-prebuilt"]
  dockerfile = "docker/Dockerfile"
  target = "prebuilt"
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
