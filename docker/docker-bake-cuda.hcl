group "default" {
  targets = [
    "universe-sensing-perception-devel-cuda",
    "universe-sensing-perception-cuda",
    "universe-devel-cuda",
    "universe-cuda"
  ]
}

// For docker/metadata-action
target "docker-metadata-action-universe-sensing-perception-devel-cuda" {}
target "docker-metadata-action-universe-sensing-perception-cuda" {}
target "docker-metadata-action-universe-devel-cuda" {}
target "docker-metadata-action-universe-cuda" {}

target "universe-sensing-perception-devel-cuda" {
  inherits = ["docker-metadata-action-universe-sensing-perception-devel-cuda"]
  dockerfile = "docker/Dockerfile"
  target = "universe-sensing-perception-devel-cuda"
}

target "universe-sensing-perception-cuda" {
  inherits = ["docker-metadata-action-universe-sensing-perception-cuda"]
  dockerfile = "docker/Dockerfile"
  target = "universe-sensing-perception-cuda"
}

target "universe-devel-cuda" {
  inherits = ["docker-metadata-action-universe-devel-cuda"]
  dockerfile = "docker/Dockerfile"
  target = "universe-devel-cuda"
}

target "universe-cuda" {
  inherits = ["docker-metadata-action-universe-cuda"]
  dockerfile = "docker/Dockerfile"
  target = "universe-cuda"
}
