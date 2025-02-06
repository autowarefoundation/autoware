group "default" {
  targets = [
    "visualizer"
  ]
}

// For docker/metadata-action
target "docker-metadata-action-visualizer" {}

target "visualizer" {
  inherits = ["docker-metadata-action-visualizer"]
  dockerfile = "docker/tools/visualizer/Dockerfile.visualizer"
  target = "visualizer"
}
