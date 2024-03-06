group "default" {
  targets = ["prebuilt", "devel", "runtime"]
}

// For docker/metadata-action
target "docker-metadata-action-prebuilt" {}
target "docker-metadata-action-devel" {}
target "docker-metadata-action-runtime" {}

target "prebuilt" {
  inherits = ["docker-metadata-action-prebuilt"]
  dockerfile = "docker/autoware-openadk/Dockerfile"
  target = "prebuilt"
}

target "devel" {
  inherits = ["docker-metadata-action-devel"]
  dockerfile = "docker/autoware-openadk/Dockerfile"
  target = "devel"
}

target "runtime" {
  inherits = ["docker-metadata-action-runtime"]
  dockerfile = "docker/autoware-openadk/Dockerfile"
  target = "runtime"
}
