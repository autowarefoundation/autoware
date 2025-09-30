variable "ROS_DISTRO" {
  default = "jazzy"
}

variable "RMW_IMPLEMENTATION" {
  default = "rmw_cyclonedds_cpp"
}

variable "CUDA_VERSION" {
  default = "12.8"
}

variable "TENSORRT_VERSION" {
  default = "10.8.0.43-1+cuda12.8"
}

variable "SPCONV_VERSION" {
  default = "2.3.8"
}

variable "CUMM_VERSION" {
  default = "0.5.3"
}

group "default" {
  targets = ["universe", "universe-cuda"]
}

target "base" {
  dockerfile = "docker-new/base.Dockerfile"
  target     = "base"
  tags       = ["autoware:base-${ROS_DISTRO}"]
  args = {
    ROS_DISTRO         = ROS_DISTRO
    RMW_IMPLEMENTATION = RMW_IMPLEMENTATION
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
    CUDA_VERSION     = CUDA_VERSION
    TENSORRT_VERSION = TENSORRT_VERSION
    SPCONV_VERSION   = SPCONV_VERSION
    CUMM_VERSION     = CUMM_VERSION
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
    CUDA_VERSION     = CUDA_VERSION
    TENSORRT_VERSION = TENSORRT_VERSION
    SPCONV_VERSION   = SPCONV_VERSION
    CUMM_VERSION     = CUMM_VERSION
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
    CUDA_VERSION     = CUDA_VERSION
    TENSORRT_VERSION = TENSORRT_VERSION
    SPCONV_VERSION   = SPCONV_VERSION
    CUMM_VERSION     = CUMM_VERSION
  }
}
