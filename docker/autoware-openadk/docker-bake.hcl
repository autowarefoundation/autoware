group "default" {
  targets = ["base", "planning-control", "visualizer"]
}

// For docker/metadata-action
target "docker-metadata-action-base" {}
target "docker-metadata-action-planning-control" {}
target "docker-metadata-action-visualizer" {}
target "docker-metadata-action-simulator" {}

// Autoware Openadk services
target "base" {
  inherits = ["docker-metadata-action-base"]
  dockerfile = "docker/autoware-openadk/Dockerfile"
  target = "base"
}

target "planning-control" {
  inherits = ["docker-metadata-action-planning-control"]
  dockerfile = "docker/autoware-openadk/services/planning-control/Dockerfile"
  target = "planning-control"
}

target "visualizer" {
  inherits = ["docker-metadata-action-visualizer"]
  dockerfile = "docker/autoware-openadk/services/visualizer/Dockerfile"
  target = "visualizer"
}

target "simulator" {
  inherits = ["docker-metadata-action-simulator"]
  dockerfile = "docker/autoware-openadk/services/simulator/Dockerfile"
  target = "simulator"
}
