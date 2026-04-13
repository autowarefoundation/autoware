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
    autoware-base = ctx("base")
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
    autoware-base = ctx("base")
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
    autoware-base = ctx("base")
  }
  args = {
    BASE_IMAGE = "autoware-base"
  }
}

target "_universe-base" {
  dockerfile = "docker-new/universe.Dockerfile"
  contexts = {
    autoware-core-devel = ctx("core-devel")
    autoware-core       = ctx("core")
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
