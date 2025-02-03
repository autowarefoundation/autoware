group "default" {
  targets = [
    "scenario-simulator",
    "visualizer"
  ]
}

// For docker/metadata-action
target "docker-metadata-action-scenario-simulator" {}
target "docker-metadata-action-visualizer" {}

target "scenario-simulator" {
  inherits = ["docker-metadata-action-scenario-simulator"]
  dockerfile = "docker/tools/scenario-simulator/Dockerfile.scenario-simulator"
  target = "scenario-simulator"
}

target "visualizer" {
  inherits = ["docker-metadata-action-visualizer"]
  dockerfile = "docker/tools/visualizer/Dockerfile.visualizer"
  target = "visualizer"
}
