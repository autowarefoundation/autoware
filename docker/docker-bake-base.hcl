group "default" {
  targets = [
    "base",
    "base-cuda",
    "jazzy-base"
  ]
}

// For docker/metadata-action
target "docker-metadata-action-base" {}
target "docker-metadata-action-base-cuda" {}
target "docker-metadata-action-base-jazzy" {}

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

target "jazzy-base" {
  inherits = ["docker-metadata-action-base-jazzy"]
  dockerfile = "docker/Dockerfile.base"
  target = "base"
  args = {
    ROS_DISTRO = "jazzy"
    BASE_IMAGE = "ros:jazzy-ros-base-noble"
  }
}
