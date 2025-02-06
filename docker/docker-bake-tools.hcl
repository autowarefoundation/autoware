group "default" {
  targets = [
    "scenario-simulator",
  ]
}

// For docker/metadata-action
target "docker-metadata-action-scenario-simulator" {}

target "scenario-simulator" {
  inherits = ["docker-metadata-action-scenario-simulator"]
  dockerfile = "docker/tools/scenario-simulator/Dockerfile.scenario-simulator"
  target = "scenario-simulator"
}
