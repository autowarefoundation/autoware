cmake_minimum_required(VERSION 2.8.3)
project(jsk_recognition_msgs)
find_package(catkin REQUIRED
  std_msgs sensor_msgs geometry_msgs message_generation pcl_msgs)
add_message_files(
  FILES
  BoundingBoxArray.msg
  BoundingBoxMovement.msg
  BoundingBox.msg
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
  ICPResult.msg
  ImageDifferenceValue.msg
  Int32Stamped.msg
  LineArray.msg
  Line.msg
  ModelCoefficientsArray.msg
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
  WeightedPoseArray.msg
  )

generate_messages(
  DEPENDENCIES
  std_msgs sensor_msgs geometry_msgs pcl_msgs
)
catkin_package(
  CATKIN_DEPENDS std_msgs sensor_msgs geometry_msgs pcl_msgs
)

install(DIRECTORY scripts
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})