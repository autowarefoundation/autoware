cmake_minimum_required(VERSION 2.8.3)
project(jsk_recognition_msgs)
find_package(catkin REQUIRED
  std_msgs sensor_msgs geometry_msgs message_generation pcl_msgs jsk_footstep_msgs)
add_message_files(
  FILES
  Accuracy.msg
  BoundingBoxArray.msg
  BoundingBoxMovement.msg
  BoundingBox.msg
  BoundingBoxArrayWithCameraInfo.msg
  Circle2DArray.msg
  Circle2D.msg
  ClassificationResult.msg
  ClusterPointIndices.msg
  ColorHistogramArray.msg
  ColorHistogram.msg
  DepthCalibrationParameter.msg
  DepthErrorResult.msg
  HeightmapConfig.msg
  Histogram.msg
  HistogramWithRangeBin.msg
  HistogramWithRange.msg
  HistogramWithRangeArray.msg
  ICPResult.msg
  ImageDifferenceValue.msg
  Int32Stamped.msg
  Label.msg
  LabelArray.msg
  LineArray.msg
  Line.msg
  ModelCoefficientsArray.msg
  Object.msg
  ObjectArray.msg
  ParallelEdgeArray.msg
  ParallelEdge.msg
  PlotData.msg
  PointsArray.msg
  PolygonArray.msg
  PosedCameraInfo.msg
  RectArray.msg
  Rect.msg
  RotatedRect.msg
  RotatedRectStamped.msg
  SimpleHandle.msg
  SimpleOccupancyGridArray.msg
  SimpleOccupancyGrid.msg
  SlicedPointCloud.msg
  SnapItRequest.msg
  SparseImage.msg
  SparseOccupancyGridArray.msg
  SparseOccupancyGridCell.msg
  SparseOccupancyGridColumn.msg
  SparseOccupancyGrid.msg
  TimeRange.msg
  TorusArray.msg
  Torus.msg
  TrackerStatus.msg
  TrackingStatus.msg
  BoolStamped.msg
  VectorArray.msg
  WeightedPoseArray.msg
  ContactSensor.msg
  ContactSensorArray.msg
  PlotDataArray.msg
  Segment.msg
  SegmentStamped.msg
  SegmentArray.msg
  PeoplePose.msg
  PeoplePoseArray.msg
  )

# TODO(wkentaro): Most of srv files are duplicated in jsk_pcl_ros,
#   and jsk_perception, so those in the packages should be removed.
add_service_files(FILES
  CallPolygon.srv
  CallSnapIt.srv
  CheckCircle.srv
  CheckCollision.srv
  EnvironmentLock.srv
  EuclideanSegment.srv
  ICPAlign.srv
  ICPAlignWithBox.srv
  NonMaximumSuppression.srv
  PolygonOnEnvironment.srv
  RobotPickupReleasePoint.srv
  SaveMesh.srv
  SetDepthCalibrationParameter.srv
  SetLabels.srv
  SetPointCloud2.srv
  SetTemplate.srv
  SnapFootstep.srv
  SwitchTopic.srv
  TowerPickUp.srv
  TowerRobotMoveCommand.srv
  TransformScreenpoint.srv
  UpdateOffset.srv
  WhiteBalancePoints.srv
  WhiteBalance.srv
  )

generate_messages(
  DEPENDENCIES
  std_msgs sensor_msgs geometry_msgs pcl_msgs jsk_footstep_msgs
)
catkin_package(
  CATKIN_DEPENDS std_msgs sensor_msgs geometry_msgs pcl_msgs jsk_footstep_msgs
)

# From https://github.com/jsk-ros-pkg/jsk_recognition/pull/2345
# Install header files directly into ${CATKIN_PACKAGE_INCLUDE_DESTINATION}.
# If the package has setup.py and modules under src/${PROJECT_NAME}/,
# install python executables directly into ${CATKIN_PACKAGE_BIN_DESTINATION}.
# However, if it doesn't have setup.py, directories including python executables
# should be installed recursively into ${CATKIN_PACKAGE_SHARE_DESTINATION}.
# Also, other directories like 'launch' or 'sample' must be installed
# recursively into ${CATKIN_PACKAGE_SHARE_DESTINATION}.
# Be careful that 'launch' and 'launch/' are different: the former is directory
# and the latter is each content.
install(DIRECTORY sample scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
