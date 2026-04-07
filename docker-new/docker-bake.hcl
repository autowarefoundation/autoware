variable "ROS_DISTRO" {
  default = "jazzy"
}

// CI variables: set via environment in GitHub Actions, empty for local builds
variable "REGISTRY" {
  default = ""
}
variable "PLATFORM" {
  default = ""
}
variable "TAG_DATE" {
  default = ""
}
variable "TAG_VERSION" {
  default = ""
}
variable "TAG_REF" {
  default = ""
}

function "tags" {
  params = [name]
  result = compact(concat(
    REGISTRY == "" ? ["autoware:${name}-${ROS_DISTRO}"] : [],
    REGISTRY != "" && TAG_REF == "" ? ["${REGISTRY}:${name}-${ROS_DISTRO}-${PLATFORM}"] : [],
    REGISTRY != "" && TAG_DATE != "" && TAG_REF == "" ? ["${REGISTRY}:${name}-${ROS_DISTRO}-${TAG_DATE}-${PLATFORM}"] : [],
    REGISTRY != "" && TAG_VERSION != "" ? ["${REGISTRY}:${name}-${ROS_DISTRO}-${TAG_VERSION}-${PLATFORM}"] : [],
    REGISTRY != "" && TAG_REF != "" ? ["${REGISTRY}:${name}-${ROS_DISTRO}-${TAG_REF}-${PLATFORM}"] : [],
  ))
}

group "default" {
  targets = ["universe", "universe-cuda"]
}

group "ci-base" {
  targets = ["base"]
}

target "base" {
  dockerfile = "docker-new/base.Dockerfile"
  target     = "base"
  tags       = tags("base")
  args = {
    ROS_DISTRO = ROS_DISTRO
  }
}

target "core-dependencies" {
  dockerfile = "docker-new/core.Dockerfile"
  target     = "core-dependencies"
  tags       = tags("core-dependencies")
  contexts = {
    autoware-base = "target:base"
  }
  args = {
    BASE_IMAGE = "autoware-base"
  }
}

target "core-devel" {
  dockerfile = "docker-new/core.Dockerfile"
  target     = "core-devel"
  tags       = tags("core-devel")
  contexts = {
    autoware-base = "target:base"
  }
  args = {
    BASE_IMAGE = "autoware-base"
  }
}

target "core" {
  dockerfile = "docker-new/core.Dockerfile"
  target     = "core"
  tags       = tags("core")
  contexts = {
    autoware-base = "target:base"
  }
  args = {
    BASE_IMAGE = "autoware-base"
  }
}

target "_universe-base" {
  dockerfile = "docker-new/universe.Dockerfile"
  contexts = {
    autoware-core-devel = "target:core-devel"
    autoware-core       = "target:core"
  }
  args = {
    CORE_DEVEL_IMAGE = "autoware-core-devel"
    CORE_IMAGE       = "autoware-core"
  }
}

target "universe-dependencies" {
  inherits = ["_universe-base"]
  target   = "universe-dependencies"
  tags     = tags("universe-dependencies")
}

target "universe-dependencies-cuda" {
  inherits = ["_universe-base"]
  target   = "universe-dependencies-cuda"
  tags     = tags("universe-dependencies-cuda")
}

target "universe-devel-cuda" {
  inherits = ["_universe-base"]
  target   = "universe-devel-cuda"
  tags     = tags("universe-devel-cuda")
}

target "universe-devel" {
  inherits = ["_universe-base"]
  target   = "universe-devel"
  tags     = tags("universe-devel")
}

target "universe-runtime-dependencies" {
  inherits = ["_universe-base"]
  target   = "universe-runtime-dependencies"
  tags     = tags("universe-runtime-dependencies")
}

target "universe" {
  inherits = ["_universe-base"]
  target   = "universe"
  tags     = tags("universe")
}

target "universe-cuda" {
  inherits = ["_universe-base"]
  target   = "universe-cuda"
  tags     = tags("universe-cuda")
}
