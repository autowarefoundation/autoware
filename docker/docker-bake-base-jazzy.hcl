group "default" {
  targets = [
    "base-jazzy",
    "base-jazzy-cuda"
  ]
}

// For docker/metadata-action
target "docker-metadata-action-base-jazzy" {}
target "docker-metadata-action-base-jazzy-cuda" {}

target "base-jazzy" {
  inherits = ["docker-metadata-action-base-jazzy"]
  dockerfile = "docker/Dockerfile.base-jazzy"
  target = "base"
}

target "base-jazzy-cuda" {
  inherits = ["docker-metadata-action-base-jazzy-cuda"]
  dockerfile = "docker/Dockerfile.base-jazzy"
  target = "base-cuda"
}
