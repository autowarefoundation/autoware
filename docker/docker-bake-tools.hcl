group "default" {
  targets = [
    "simulator",
    "web-visualizer"
  ]
}

// For docker/metadata-action
target "docker-metadata-action-simulator" {}
target "docker-metadata-action-web-visualizer" {}

target "simulator" {
  inherits = ["docker-metadata-action-simulator"]
  dockerfile = "docker/tools/Dockerfile.simulator"
  target = "simulator"
}

target "web-visualizer" {
  inherits = ["docker-metadata-action-web-visualizer"]
  dockerfile = "docker/tools/Dockerfile.web-visualizer"
  target = "web-visualizer"
}
