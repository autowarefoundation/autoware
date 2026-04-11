variable "ROS_DISTRO" {
  default = "jazzy"
}

function "tags" {
  params = [name]
  result = ["autoware:${name}-${ROS_DISTRO}"]
}

group "default" {
  targets = ["base",
             "core-dependencies", "core-devel", "core",
             "universe-dependencies", "universe-devel",
             "universe-runtime-dependencies", "universe",
             "universe-dependencies-cuda", "universe-devel-cuda", "universe-cuda"]
}

group "ci-base" {
  targets = ["base"]
}

group "ci-core" {
  targets = ["core-dependencies", "core-devel", "core"]
}

group "ci-universe" {
  targets = ["universe-dependencies", "universe-devel",
             "universe-runtime-dependencies", "universe"]
}

group "ci-universe-cuda" {
  targets = ["universe-dependencies-cuda", "universe-devel-cuda", "universe-cuda"]
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
