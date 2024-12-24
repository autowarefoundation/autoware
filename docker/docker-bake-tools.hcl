group "default" {
  targets = [
    "simulator",
    "visualizer"
  ]
}

// For docker/metadata-action
target "docker-metadata-action-simulator" {}
target "docker-metadata-action-visualizer" {}

target "simulator" {
  inherits = ["docker-metadata-action-simulator"]
  dockerfile = "docker/tools/Dockerfile"
  target = "simulator"
}

target "visualizer" {
  inherits = ["docker-metadata-action-visualizer"]
  dockerfile = "docker/tools/Dockerfile"
  target = "visualizer"
}
