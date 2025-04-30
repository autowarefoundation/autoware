group "default" {
  targets = [
    "universe-common-devel-cuda",
    "universe-sensing-perception-devel-cuda",
    "universe-sensing-perception-cuda",
    "universe-devel-cuda",
    "universe-cuda"
  ]
}

// For docker/metadata-action
target "docker-metadata-action-universe-common-devel-cuda" {}
target "docker-metadata-action-universe-sensing-perception-devel-cuda" {}
target "docker-metadata-action-universe-sensing-perception-cuda" {}
target "docker-metadata-action-universe-devel-cuda" {}
target "docker-metadata-action-universe-cuda" {}

variable "AUTOWARE_IMAGE_PREFIX" {
  default = ""
}

target "universe-common-devel-cuda" {
  inherits = ["docker-metadata-action-universe-common-devel-cuda"]
  dockerfile = "docker/Dockerfile"
  target = "universe-common-devel-cuda"
  args = {
    AUTOWARE_IMAGE_PREFIX = "${AUTOWARE_IMAGE_PREFIX}"
  }
}

target "universe-sensing-perception-devel-cuda" {
  inherits = ["docker-metadata-action-universe-sensing-perception-devel-cuda"]
  dockerfile = "docker/Dockerfile"
  target = "universe-sensing-perception-devel-cuda"
  args = {
    AUTOWARE_IMAGE_PREFIX = "${AUTOWARE_IMAGE_PREFIX}"
  }
}

target "universe-sensing-perception-cuda" {
  inherits = ["docker-metadata-action-universe-sensing-perception-cuda"]
  dockerfile = "docker/Dockerfile"
  target = "universe-sensing-perception-cuda"
  args = {
    AUTOWARE_IMAGE_PREFIX = "${AUTOWARE_IMAGE_PREFIX}"
  }
}

target "universe-devel-cuda" {
  inherits = ["docker-metadata-action-universe-devel-cuda"]
  dockerfile = "docker/Dockerfile"
  target = "universe-devel-cuda"
  args = {
    AUTOWARE_IMAGE_PREFIX = "${AUTOWARE_IMAGE_PREFIX}"
  }
}

target "universe-cuda" {
  inherits = ["docker-metadata-action-universe-cuda"]
  dockerfile = "docker/Dockerfile"
  target = "universe-cuda"
  args = {
    AUTOWARE_IMAGE_PREFIX = "${AUTOWARE_IMAGE_PREFIX}"
  }
}
