group "default" {
  targets = [
    "visualizer",
    "scenario-simulator",
  ]
}

// For docker/metadata-action
target "docker-metadata-action-visualizer" {}
target "docker-metadata-action-scenario-simulator" {}

target "visualizer" {
  inherits = ["docker-metadata-action-visualizer"]
  dockerfile = "docker/tools/visualizer/Dockerfile"
  target = "visualizer"
}

target "scenario-simulator" {
  inherits = ["docker-metadata-action-scenario-simulator"]
  dockerfile = "docker/tools/scenario-simulator/Dockerfile"
  target = "scenario-simulator"
}
