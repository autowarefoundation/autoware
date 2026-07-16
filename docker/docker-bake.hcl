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

// When true, cross-group context references pull pre-built images from the
// registry instead of rebuilding via target:. Set in CI where each group
// builds in a separate job and upstream images are already pushed.
variable "USE_REGISTRY_CONTEXTS" {
  default = false
}

// IMPORTANT: The first element must always be the plain name-distro-platform tag
// because ctx() uses tags(name)[0] to construct docker-image:// references.
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

// Returns "docker-image://..." when USE_REGISTRY_CONTEXTS is true (CI),
// or "target:..." when false (local builds).
function "ctx" {
  params = [name]
  result = USE_REGISTRY_CONTEXTS ? "docker-image://${tags(name)[0]}" : "target:${name}"
}

// CUDA architectures by ROS_DISTRO. The same value drives both
// CMAKE_CUDA_ARCHITECTURES env vars in docker/universe-cuda.Dockerfile.
//
//   jazzy  (Ubuntu 24.04 / CUDA 13.0): include sm_110 (Thor Blackwell)
//   humble (Ubuntu 22.04 / CUDA 12.8): omit sm_110 — Thor's compute
//                                       capability was introduced in
//                                       CUDA 13.0 and is not a valid
//                                       target for the 12.8 toolchain.
//
// Compute capabilities: 86=Ampere consumer, 87=Orin, 89=Ada, 90=Hopper,
//                       110=Thor Blackwell (Jetson Thor + DRIVE Thor).
//
// Unknown distros resolve to "" on purpose: an empty CMAKE_CUDA_ARCHITECTURES
// fails the CUDA build loudly rather than silently picking one distro's list.
function "cuda_architectures" {
  params = [distro]
  result = distro == "jazzy" ? "86;87;89;90;110" : (
    distro == "humble" ? "86;89;90" : ""
  )
}

group "default" {
  targets = ["base",
             "core-dependencies", "core-devel", "core",
             "base-cuda-runtime", "base-cuda-devel",
             "universe-dependencies", "universe-devel", "universe",
             "universe-dependencies-cuda", "universe-devel-cuda", "universe-cuda"]
}

group "ci-base" {
  targets = ["base"]
}

group "ci-core" {
  targets = ["core-dependencies", "core-devel", "core"]
}

group "ci-base-cuda" {
  targets = ["base-cuda-runtime", "base-cuda-devel"]
}

group "ci-universe" {
  targets = ["universe-dependencies", "universe-devel", "universe"]
}

group "ci-universe-cuda" {
  targets = ["universe-dependencies-cuda", "universe-devel-cuda", "universe-cuda"]
}

target "base" {
  dockerfile = "docker/base.Dockerfile"
  target     = "base"
  tags       = tags("base")
  args = {
    ROS_DISTRO = ROS_DISTRO
  }
}

target "core-dependencies" {
  dockerfile = "docker/core.Dockerfile"
  target     = "core-dependencies"
  tags       = tags("core-dependencies")
  contexts = {
    autoware-base = ctx("base")
  }
  args = {
    BASE_IMAGE = "autoware-base"
    ROS_DISTRO = ROS_DISTRO
  }
}

target "core-devel" {
  dockerfile = "docker/core.Dockerfile"
  target     = "core-devel"
  tags       = tags("core-devel")
  contexts = {
    autoware-base = ctx("base")
  }
  args = {
    BASE_IMAGE = "autoware-base"
    ROS_DISTRO = ROS_DISTRO
  }
}

target "core" {
  dockerfile = "docker/core.Dockerfile"
  target     = "core"
  tags       = tags("core")
  contexts = {
    autoware-base = ctx("base")
  }
  args = {
    BASE_IMAGE = "autoware-base"
    ROS_DISTRO = ROS_DISTRO
  }
}

target "base-cuda-runtime" {
  dockerfile = "docker/base-cuda.Dockerfile"
  target     = "base-cuda-runtime"
  tags       = tags("base-cuda-runtime")
  contexts = {
    autoware-base = ctx("base")
  }
  args = {
    BASE_IMAGE = "autoware-base"
    ROS_DISTRO = ROS_DISTRO
  }
}

target "base-cuda-devel" {
  dockerfile = "docker/base-cuda.Dockerfile"
  target     = "base-cuda-devel"
  tags       = tags("base-cuda-devel")
  contexts = {
    autoware-base = ctx("base")
  }
  args = {
    BASE_IMAGE = "autoware-base"
    ROS_DISTRO = ROS_DISTRO
  }
}

target "_universe-base" {
  dockerfile = "docker/universe.Dockerfile"
  contexts = {
    autoware-core-devel = ctx("core-devel")
    autoware-core       = ctx("core")
  }
  args = {
    CORE_DEVEL_IMAGE = "autoware-core-devel"
    CORE_IMAGE       = "autoware-core"
    ROS_DISTRO       = ROS_DISTRO
  }
}

target "universe-dependencies" {
  inherits = ["_universe-base"]
  target   = "universe-dependencies"
  tags     = tags("universe-dependencies")
}

target "universe-devel" {
  inherits = ["_universe-base"]
  target   = "universe-devel"
  tags     = tags("universe-devel")
}

target "universe" {
  inherits = ["_universe-base"]
  target   = "universe"
  tags     = tags("universe")
}

target "_universe-cuda-base" {
  dockerfile = "docker/universe-cuda.Dockerfile"
  contexts = {
    autoware-base-cuda-runtime = ctx("base-cuda-runtime")
    autoware-base-cuda-devel   = ctx("base-cuda-devel")
    autoware-core-devel        = ctx("core-devel")
  }
  args = {
    BASE_CUDA_RUNTIME_IMAGE = "autoware-base-cuda-runtime"
    BASE_CUDA_DEVEL_IMAGE   = "autoware-base-cuda-devel"
    ROS_DISTRO              = ROS_DISTRO
    CUDA_ARCHITECTURES      = cuda_architectures(ROS_DISTRO)
  }
}

target "universe-dependencies-cuda" {
  inherits = ["_universe-cuda-base"]
  target   = "universe-dependencies-cuda"
  tags     = tags("universe-dependencies-cuda")
}

target "universe-devel-cuda" {
  inherits = ["_universe-cuda-base"]
  target   = "universe-devel-cuda"
  tags     = tags("universe-devel-cuda")
}

target "universe-cuda" {
  inherits = ["_universe-cuda-base"]
  target   = "universe-cuda"
  tags     = tags("universe-cuda")
}
