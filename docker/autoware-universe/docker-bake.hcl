group "default" {
  targets = ["devel", "prebuilt", "runtime"]
}

// For docker/metadata-action
target "docker-metadata-action-devel" {}
target "docker-metadata-action-prebuilt" {}
target "docker-metadata-action-runtime" {}

// For source image
target "devel" {
  inherits = ["docker-metadata-action-devel"]
  dockerfile = "docker/autoware-universe/Dockerfile"
  target = "devel"
}

// For prebuilt image
target "prebuilt" {
  inherits = ["docker-metadata-action-prebuilt"]
  dockerfile = "docker/autoware-universe/Dockerfile"
  target = "prebuilt"
}

// For runtime image
target "runtime" {
  inherits = ["docker-metadata-action-runtime"]
  dockerfile = "docker/autoware-universe/Dockerfile"
  target = "runtime"
}
