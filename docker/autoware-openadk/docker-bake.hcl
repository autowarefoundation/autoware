group "default" {
  targets = ["base", "prebuilt", "devel", "runtime"]
}

// For docker/metadata-action
target "docker-metadata-action-base" {}
target "docker-metadata-action-prebuilt" {}
target "docker-metadata-action-devel" {}
target "docker-metadata-action-runtime" {}
target "docker-metadata-action-planning-control" {}
target "docker-metadata-action-visualizer" {}
target "docker-metadata-action-simulator" {}

target "base" {
  inherits = ["docker-metadata-action-base"]
  dockerfile = "docker/autoware-openadk/Dockerfile"
  target = "base"
}

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
