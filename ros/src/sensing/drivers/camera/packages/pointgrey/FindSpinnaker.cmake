unset(Spinnaker_FOUND)
unset(Spinnaker_INCLUDE_DIRS)
unset(Spinnaker_LIBRARIES)

find_path(Spinnaker_INCLUDE_DIRS NAMES
  Spinnaker.h
  PATHS
  /usr/include/spinnaker/
  /usr/local/include/spinnaker/
)

find_library(Spinnaker_LIBRARIES NAMES Spinnaker
  PATHS
  /usr/lib
  /usr/local/lib
)

if (Spinnaker_INCLUDE_DIRS AND Spinnaker_LIBRARIES)
  set(Spinnaker_FOUND 1)
endif (Spinnaker_INCLUDE_DIRS AND Spinnaker_LIBRARIES)