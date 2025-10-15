group "default" {
  targets = [
    "base",
    "base-cuda"
  ]
}

// For docker/metadata-action
target "docker-metadata-action-base" {}
target "docker-metadata-action-base-cuda" {}

target "base" {
  inherits = ["docker-metadata-action-base"]
  dockerfile = "docker/Dockerfile.base"
  target = "base"
}

target "base-cuda" {
  inherits = ["docker-metadata-action-base-cuda"]
  dockerfile = "docker/Dockerfile.base"
  target = "base-cuda"
}
