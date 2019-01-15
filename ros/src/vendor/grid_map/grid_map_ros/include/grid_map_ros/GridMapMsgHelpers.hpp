/*
 * GridMapMsgHelpers.hpp
 *
 *  Created on: Sep 8, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

// ROS
#include <ros/ros.h>
#include <std_msgs/UInt32MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64MultiArray.h>

// Eigen
#include <Eigen/Core>

namespace grid_map {

/*!
 * Returns the number of dimensions of the grid map.
 * @return number of dimensions.
 */
const int nDimensions();

enum class StorageIndices {
    Column,
    Row
};

//! Holds the names of the storage indeces.
extern std::map<StorageIndices, std::string> storageIndexNames;

/*!
 * Checks if message data is stored in row-major format.
 * @tparam MultiArrayMessageType_ a std_msgs::xxxMultiArray message (e.g. std_msgs::Float32MultiArray).
 * @param[in] message the message data.
 * @return true if is in row-major format, false if is in column-major format.
 */
template<typename MultiArrayMessageType_>
bool isRowMajor(const MultiArrayMessageType_& message)
{
  if (message.layout.dim[0].label == grid_map::storageIndexNames[grid_map::StorageIndices::Column]) return false;
  else if (message.layout.dim[0].label == grid_map::storageIndexNames[grid_map::StorageIndices::Row]) return true;
  ROS_ERROR("isRowMajor() failed because layout label is not set correctly.");
  return false;
}

/*!
 * Returns the number of columns of the message data.
 * @tparam MultiArrayMessageType_ a std_msgs::xxxMultiArray message (e.g. std_msgs::Float32MultiArray).
 * @param[in] message the message data.
 * @return the number of columns.
 */
template<typename MultiArrayMessageType_>
unsigned int getCols(const MultiArrayMessageType_& message)
{
  if (isRowMajor(message)) return message.layout.dim.at(1).size;
  return message.layout.dim.at(0).size;
}

/*!
 * Returns the number of rows of the message data.
 * @tparam MultiArrayMessageType_ a std_msgs::xxxMultiArray message (e.g. std_msgs::Float32MultiArray).
 * @param[in] message the message data.
 * @return the number of rows.
 */
template<typename MultiArrayMessageType_>
unsigned int getRows(const MultiArrayMessageType_& message)
{
  if (isRowMajor(message)) return message.layout.dim.at(0).size;
  return message.layout.dim.at(1).size;
}

/*!
 * Copies an Eigen matrix into a ROS MultiArray message.
 * Both column- and row-major matrices are allowed, and the type
 * will be marked in the layout labels.
 * @tparam MultiArrayMessageType_ a std_msgs::xxxMultiArray message (e.g. std_msgs::Float32MultiArray).
 * @tparam EigenType_ an Eigen matrix with matching Scalar type as that of the multi-array message.
 * @param[in] e the Eigen matrix to be converted.
 * @param[out] m the ROS message to which the data will be copied.
 * @return true if successful.
 */
template<typename EigenType_, typename MultiArrayMessageType_>
bool matrixEigenCopyToMultiArrayMessage(const EigenType_& e, MultiArrayMessageType_& m)
{
  m.layout.dim.resize(nDimensions());
  m.layout.dim[0].stride = e.size();
  m.layout.dim[0].size = e.outerSize();
  m.layout.dim[1].stride = e.innerSize();
  m.layout.dim[1].size = e.innerSize();

  if (e.IsRowMajor) {
    m.layout.dim[0].label = storageIndexNames[StorageIndices::Row];
    m.layout.dim[1].label = storageIndexNames[StorageIndices::Column];
  } else {
    m.layout.dim[0].label = storageIndexNames[StorageIndices::Column];
    m.layout.dim[1].label = storageIndexNames[StorageIndices::Row];
  }

  m.data.insert(m.data.begin() + m.layout.data_offset, e.data(), e.data() + e.size());
  return true;
}

/**
 * Copies a ROS xxxMultiArray message into an Eigen matrix.
 * @tparam MultiArrayMessageType_ a std_msgs::xxxMultiArray message (e.g. std_msgs::Float32MultiArray)
 * @tparam EigenType_ an Eigen matrix with matching Scalar type as that of the multi-array message.
 * @param[in] m the ROS message to which the data will be copied.
 * @param[out] e the Eigen matrix to be converted.
 * @return true if successful.
 */
template<typename EigenType_, typename MultiArrayMessageType_>
bool multiArrayMessageCopyToMatrixEigen(const MultiArrayMessageType_& m, EigenType_& e)
{
  if (e.IsRowMajor != isRowMajor(m)) {
    ROS_ERROR("multiArrayMessageToMatrixEigen() failed because the storage order is not compatible.");
    return false;
  }

  EigenType_ tempE(getRows(m), getCols(m));
  tempE = Eigen::Map<const EigenType_>(m.data.data(), getRows(m), getCols(m));
  e = tempE;
  return true;
}

/*!
 * Maps a ROS xxxMultiArray message into an Eigen matrix.
 * Both column- and row-major message types are allowed.
 * @param[in] m the ROS message to be converted.
 * @param[out] e the Eigen matrix to which the data will be mapped.
 * @return true if successful.
 */
template<typename EigenType_, typename MultiArrayMessageType_>
bool multiArrayMessageMapToMatrixEigen(MultiArrayMessageType_& m, EigenType_& e)
{
  if (e.IsRowMajor != isRowMajor(m)) {
    ROS_ERROR("multiArrayMessageToMatrixEigen() failed because the storage order is not compatible.");
    return false;
  }

  e.resize(getRows(m), getCols(m));
  e = Eigen::Map<EigenType_>(m.data.data(), getRows(m), getCols(m));
  return true;
}

} /* namespace */
