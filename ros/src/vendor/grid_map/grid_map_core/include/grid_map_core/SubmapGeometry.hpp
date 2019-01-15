/*
 * SubmapGeometry.hpp
 *
 *  Created on: Aug 18, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

#include <grid_map_core/GridMap.hpp>

namespace grid_map {

class GridMap;

/*!
 * This class holds information about the geometry of submap
 * region of a grid map. Note that, this class does NOT hold
 * the any data of the grid map.
 */
class SubmapGeometry
{
 public:

  /*!
   * Constructor. Note that the requested position and length
   * of the submap is adapted to fit the geometry of the parent
   * grid map.
   * @param[in] gridMap the parent grid map containing the submap.
   * @param[in] position the requested submap position (center).
   * @param[in] length the requested submap length.
   * @param[out] isSuccess true if successful, false otherwise.
   */
  SubmapGeometry(const GridMap& gridMap, const Position& position, const Length& length,
                 bool& isSuccess);

  virtual ~SubmapGeometry();

  const GridMap& getGridMap() const;
  const Length& getLength() const;
  const Position& getPosition() const;
  const Index& getRequestedIndexInSubmap() const;
  const Size& getSize() const;
  double getResolution() const;
  const Index& getStartIndex() const;

 private:

  //! Parent grid map of the submap.
  const GridMap& gridMap_;

  //! Start index (typically top left) index of the submap.
  Index startIndex_;

  //! Size of the submap.
  Size size_;

  //! Position (center) of the submap.
  Position position_;

  //! Length of the submap.
  Length length_;

  //! Index in the submap that corresponds to the requested
  //! position of the submap.
  Index requestedIndexInSubmap_;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} /* namespace grid_map */
