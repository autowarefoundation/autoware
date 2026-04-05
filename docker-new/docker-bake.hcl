variable "ROS_DISTRO" {
  default = "jazzy"
}

group "default" {
  targets = ["universe", "universe-cuda"]
}

target "base" {
  dockerfile = "docker-new/base.Dockerfile"
  target     = "base"
  tags       = ["autoware:base-${ROS_DISTRO}"]
  args = {
    ROS_DISTRO = ROS_DISTRO
  }
}

target "core-dependencies" {
  dockerfile = "docker-new/core.Dockerfile"
  target     = "core-dependencies"
  tags       = ["autoware:core-dependencies-${ROS_DISTRO}"]
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
  tags       = ["autoware:core-devel-${ROS_DISTRO}"]
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
  tags       = ["autoware:core-${ROS_DISTRO}"]
  contexts = {
    autoware-base = "target:base"
  }
  args = {
    BASE_IMAGE = "autoware-base"
  }
}

target "universe-dependencies" {
  dockerfile = "docker-new/universe.Dockerfile"
  target     = "universe-dependencies"
  tags       = ["autoware:universe-dependencies-${ROS_DISTRO}"]
  contexts = {
    autoware-core-devel = "target:core-devel"
    autoware-core       = "target:core"
  }
  args = {
    CORE_DEVEL_IMAGE = "autoware-core-devel"
    CORE_IMAGE       = "autoware-core"
  }
}

target "universe-dependencies-cuda" {
  dockerfile = "docker-new/universe.Dockerfile"
  target     = "universe-dependencies-cuda"
  tags       = ["autoware:universe-dependencies-${ROS_DISTRO}-cuda"]
  contexts = {
    autoware-core-devel = "target:core-devel"
    autoware-core       = "target:core"
  }
  args = {
    CORE_DEVEL_IMAGE = "autoware-core-devel"
    CORE_IMAGE       = "autoware-core"
  }
}

target "universe-devel-cuda" {
  dockerfile = "docker-new/universe.Dockerfile"
  target     = "universe-devel-cuda"
  tags       = ["autoware:universe-devel-${ROS_DISTRO}-cuda"]
  contexts = {
    autoware-core-devel = "target:core-devel"
    autoware-core       = "target:core"
  }
  args = {
    CORE_DEVEL_IMAGE = "autoware-core-devel"
    CORE_IMAGE       = "autoware-core"
  }
}

target "universe-devel" {
  dockerfile = "docker-new/universe.Dockerfile"
  target     = "universe-devel"
  tags       = ["autoware:universe-devel-${ROS_DISTRO}"]
  contexts = {
    autoware-core-devel = "target:core-devel"
    autoware-core       = "target:core"
  }
  args = {
    CORE_DEVEL_IMAGE = "autoware-core-devel"
    CORE_IMAGE       = "autoware-core"
  }
}

target "universe-runtime-dependencies" {
  dockerfile = "docker-new/universe.Dockerfile"
  target     = "universe-runtime-dependencies"
  tags       = ["autoware:universe-runtime-dependencies-${ROS_DISTRO}"]
  contexts = {
    autoware-core-devel = "target:core-devel"
    autoware-core       = "target:core"
  }
  args = {
    CORE_DEVEL_IMAGE = "autoware-core-devel"
    CORE_IMAGE       = "autoware-core"
  }
}

target "universe" {
  dockerfile = "docker-new/universe.Dockerfile"
  target     = "universe"
  tags       = ["autoware:universe-${ROS_DISTRO}"]
  contexts = {
    autoware-core-devel = "target:core-devel"
    autoware-core       = "target:core"
  }
  args = {
    CORE_DEVEL_IMAGE = "autoware-core-devel"
    CORE_IMAGE       = "autoware-core"
  }
}

target "universe-cuda" {
  dockerfile = "docker-new/universe.Dockerfile"
  target     = "universe-cuda"
  tags       = ["autoware:universe-${ROS_DISTRO}-cuda"]
  contexts = {
    autoware-core-devel = "target:core-devel"
    autoware-core       = "target:core"
  }
  args = {
    CORE_DEVEL_IMAGE = "autoware-core-devel"
    CORE_IMAGE       = "autoware-core"
  }
}
