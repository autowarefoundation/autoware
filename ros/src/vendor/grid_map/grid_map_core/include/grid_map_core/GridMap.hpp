/*
 * GridMap.hpp
 *
 *  Created on: Jul 14, 2014
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

#include "grid_map_core/TypeDefs.hpp"
#include "grid_map_core/SubmapGeometry.hpp"
#include "grid_map_core/BufferRegion.hpp"

// STL
#include <vector>
#include <unordered_map>

// Eigen
#include <Eigen/Core>

namespace grid_map {

class SubmapGeometry;

/*!
 * Grid map managing multiple overlaying maps holding float values.
 * Data structure implemented as two-dimensional circular buffer so map
 * can be moved efficiently.
 *
 * Data is defined with string keys. Examples are:
 * - "elevation"
 * - "variance"
 * - "color"
 * - "quality"
 * - "surface_normal_x", "surface_normal_y", "surface_normal_z"
 * etc.
 */
class GridMap
{
 public:
  // Type traits for use with template methods/classes using GridMap as a template parameter.
  typedef grid_map::DataType DataType;
  typedef grid_map::Matrix Matrix;

  /*!
   * Constructor.
   * @param layers a vector of strings containing the definition/description of the data layer.
   */
  GridMap(const std::vector<std::string>& layers);

  /*!
   * Emtpy constructor.
   */
  GridMap();

  /*!
   * Default copy assign and copy constructors.
   */
  GridMap(const GridMap&) = default;
  GridMap& operator=(const GridMap&) = default;
  GridMap(GridMap&&) = default;
  GridMap& operator=(GridMap&&) = default;

  /*!
   * Destructor.
   */
  virtual ~GridMap();

  /*!
   * Set the geometry of the grid map. Clears all the data.
   * @param length the side lengths in x, and y-direction of the grid map [m].
   * @param resolution the cell size in [m/cell].
   * @param position the 2d position of the grid map in the grid map frame [m].
   */
  void setGeometry(const Length& length, const double resolution,
                   const Position& position = Position::Zero());

  /*!
   * Set the geometry of the grid map from submap geometry information.
   * @param geometry the submap geometry information.
   */
  void setGeometry(const SubmapGeometry& geometry);

  /*!
   * Add a new empty data layer.
   * @param layer the name of the layer.
   * @value value the value to initialize the cells with.
   */
  void add(const std::string& layer, const double value = NAN);

  /*!
   * Add a new data layer (if the layer already exists, overwrite its data, otherwise add layer and data).
   * @param layer the name of the layer.
   * @param data the data to be added.
   */
  void add(const std::string& layer, const Matrix& data);

  /*!
   * Checks if data layer exists.
   * @param layer the name of the layer.
   * @return true if layer exists, false otherwise.
   */
  bool exists(const std::string& layer) const;

  /*!
   * Returns the grid map data for a layer as matrix.
   * @param layer the name of the layer to be returned.
   * @return grid map data as matrix.
   * @throw std::out_of_range if no map layer with name `layer` is present.
   */
  const Matrix& get(const std::string& layer) const;

  /*!
   * Returns the grid map data for a layer as non-const. Use this method
   * with care!
   * @param layer the name of the layer to be returned.
   * @return grid map data.
   * @throw std::out_of_range if no map layer with name `layer` is present.
   */
  Matrix& get(const std::string& layer);

  /*!
   * Returns the grid map data for a layer as matrix.
   * @param layer the name of the layer to be returned.
   * @return grid map data as matrix.
   * @throw std::out_of_range if no map layer with name `layer` is present.
   */
  const Matrix& operator [](const std::string& layer) const;

  /*!
   * Returns the grid map data for a layer as non-const. Use this method
   * with care!
   * @param layer the name of the layer to be returned.
   * @return grid map data.
   * @throw std::out_of_range if no map layer with name `layer` is present.
   */
  Matrix& operator [](const std::string& layer);

  /*!
   * Removes a layer from the grid map.
   * @param layer the name of the layer to be removed.
   * @return true if successful.
   */
  bool erase(const std::string& layer);

  /*!
   * Gets the names of the layers.
   * @return the names of the layers.
   */
  const std::vector<std::string>& getLayers() const;

  /*!
   * Set the basic layers that need to be valid for a cell to be considered as valid.
   * Also, the basic layers are set to NAN when clearing the cells with `clearBasic()`.
   * By default the list of basic layers is empty.
   * @param basicLayers the list of layers that are the basic layers of the map.
   */
  void setBasicLayers(const std::vector<std::string>& basicLayers);

  /*!
   * Gets the names of the basic layers.
   * @return the names of the basic layers.
   */
  const std::vector<std::string>& getBasicLayers() const;

  /*!
   * True if basic layers are defined.
   * @return true if basic layers are defined, false otherwise.
   */
  bool hasBasicLayers() const;

  /*!
   * Checks if another grid map contains the same layers as this grid map.
   * The other grid map could contain more layers than the checked ones.
   * Does not check the selection of basic layers.
   * @param other the other grid map.
   * @return true if the other grid map has the same layers, false otherwise.
   */
  bool hasSameLayers(const grid_map::GridMap& other) const;

  /*!
   * Get cell data at requested position.
   * @param layer the name of the layer to be accessed.
   * @param position the requested position.
   * @return the data of the cell.
   * @throw std::out_of_range if no map layer with name `layer` is present.
   */
  float& atPosition(const std::string& layer, const Position& position);

  /*!
   * Get cell data at requested position. Const version form above.
   * @param layer the name of the layer to be accessed.
   * @param position the requested position.
   * @return the data of the cell.
   * @throw std::out_of_range if no map layer with name `layer` is present.
   * @throw std::runtime_error if the specified interpolation method is not implemented.
   */
  float atPosition(const std::string& layer, const Position& position,
                   InterpolationMethods interpolationMethod = InterpolationMethods::INTER_NEAREST) const;

  /*!
   * Get cell data for requested index.
   * @param layer the name of the layer to be accessed.
   * @param index the requested index.
   * @return the data of the cell.
   * @throw std::out_of_range if no map layer with name `layer` is present.
   */
  float& at(const std::string& layer, const Index& index);

  /*!
   * Get cell data for requested index. Const version form above.
   * @param layer the name of the layer to be accessed.
   * @param index the requested index.
   * @return the data of the cell.
   * @throw std::out_of_range if no map layer with name `layer` is present.
   */
  float at(const std::string& layer, const Index& index) const;

  /*!
   * Gets the corresponding cell index for a position.
   * @param[in] position the requested position.
   * @param[out] index the corresponding index.
   * @return true if successful, false if position outside of map.
   */
  bool getIndex(const Position& position, Index& index) const;

  /*!
   * Gets the 2d position of cell specified by the index (x, y of cell position) in
   * the grid map frame.
   * @param[in] index the index of the requested cell.
   * @param[out] position the position of the data point in the parent frame.
   * @return true if successful, false if index not within range of buffer.
   */
  bool getPosition(const Index& index, Position& position) const;

  /*!
   * Check if position is within the map boundaries.
   * @param position the position to be checked.
   * @return true if position is within map, false otherwise.
   */
  bool isInside(const Position& position) const;

  /*!
   * Checks if the index of all layers defined as basic types are valid,
   * i.e. if all basic types are finite. Returns `false` if no basic types are defined.
   * @param index the index to check.
   * @return true if cell is valid, false otherwise.
   */
  bool isValid(const Index& index) const;

  /*!
   * Checks if cell at index is a valid (finite) for a certain layer.
   * @param index the index to check.
   * @param layer the name of the layer to be checked for validity.
   * @return true if cell is valid, false otherwise.
   */
  bool isValid(const Index& index, const std::string& layer) const;

  /*!
   * Checks if cell at index is a valid (finite) for certain layers.
   * @param index the index to check.
   * @param layers the layers to be checked for validity.
   * @return true if cell is valid, false otherwise.
   */
  bool isValid(const Index& index, const std::vector<std::string>& layers) const;

  /*!
   * Gets the 3d position of a data point (x, y of cell position & cell value as z) in
   * the grid map frame. This is useful for data layers such as elevation.
   * @param layer the name of the layer to be accessed.
   * @param index the index of the requested cell.
   * @param position the position of the data point in the parent frame.
   * @return true if successful, false if no valid data available.
   */
  bool getPosition3(const std::string& layer, const Index& index, Position3& position) const;

  /*!
   * Gets the 3d vector of three layers with suffixes 'x', 'y', and 'z'.
   * @param layerPrefix the prefix for the layer to bet get as vector.
   * @param index the index of the requested cell.
   * @param vector the vector with the values of the data type.
   * @return true if successful, false if no valid data available.
   */
  bool getVector(const std::string& layerPrefix, const Index& index,
                 Eigen::Vector3d& vector) const;

  /*!
   * Gets a submap from the map. The requested submap is specified with the requested
   * location and length.
   * Note: The returned submap may not have the requested length due to the borders
   * of the map and discretization.
   * @param[in] position the requested position of the submap (usually the center).
   * @param[in] length the requested length of the submap.
   * @param[out] isSuccess true if successful, false otherwise.
   * @return submap (is empty if success is false).
   */
  GridMap getSubmap(const Position& position, const Length& length, bool& isSuccess) const;

  /*!
   * Gets a submap from the map. The requested submap is specified with the requested
   * location and length.
   * Note: The returned submap may not have the requested length due to the borders
   * of the map and discretization.
   * @param[in] position the requested position of the submap (usually the center).
   * @param[in] length the requested length of the submap.
   * @param[out] indexInSubmap the index of the requested position in the submap.
   * @param[out] isSuccess true if successful, false otherwise.
   * @return submap (is empty if success is false).
   */
  GridMap getSubmap(const Position& position, const Length& length, Index& indexInSubmap,
                    bool& isSuccess) const;

   /*!
    * Set the position of the grid map.
    * Note: This method does not change the data stored in the grid map and
    * is complementary to the `move(...)` method. For a comparison between
    * the `setPosition` and the `move` method, see the `move_demo_node.cpp`
    * file of the `grid_map_demos` package.
    * @param position the 2d position of the grid map in the grid map frame [m].
    */
   void setPosition(const Position& position);

  /*!
   * Move the grid map w.r.t. to the grid map frame. Use this to move the grid map
   * boundaries without moving the grid map data. Takes care of all the data handling,
   * such that the grid map data is stationary in the grid map frame.
   * Note: For a comparison between the `setPosition` and the `move` method,
   * see the `move_demo_node.cpp` file of the `grid_map_demos` package.
   * @param position the new location of the grid map in the map frame.
   * @param newRegions the regions of the newly covered / previously uncovered regions of the buffer.
   * @return true if map has been moved, false otherwise.
   */
  bool move(const Position& position, std::vector<BufferRegion>& newRegions);

  /*!
   * Move the grid map w.r.t. to the grid map frame. Use this to move the grid map
   * boundaries without moving the grid map data. Takes care of all the data handling,
   * such that the grid map data is stationary in the grid map frame.
   * @param position the new location of the grid map in the map frame.
   * @return true if map has been moved, false otherwise.
   */
  bool move(const Position& position);

  /*!
   * Adds data from an other grid map to this grid map
   * @param other the grid map to take data from.
   * @param extendMap if true the grid map is resized that the other map fits within.
   * @param overwriteData if true the new data replaces the old values, else only invalid cells are updated.
   * @param copyAllLayer if true all layers are used to add data.
   * @param layers the layers that are copied if not all layers are used.
   * @return true if successful.
   */
  bool addDataFrom(const GridMap& other, bool extendMap,
                   bool overwriteData, bool copyAllLayers,
                   std::vector<std::string> layers = std::vector<std::string>());

  /*!
   * Extends the size of the grip map such that the other grid map fits within.
   * @param other the grid map to extend the size to.
   * @return true if successful.
   */
  bool extendToInclude(const GridMap& other);

  /*!
   * Clears all cells (set to NAN) for a layer.
   * @param layer the layer to be cleared.
   */
  void clear(const std::string& layer);

  /*!
   * Clears all cells (set to NAN) for all basic layers.
   * Header information (geometry etc.) remains valid.
   */
  void clearBasic();

  /*!
   * Clears all cells of all layers.
   * If basic layers are used, clearBasic() is preferred as it is more efficient.
   * Header information (geometry etc.) remains valid.
   */
  void clearAll();

  /*!
   * Set the timestamp of the grid map.
   * @param timestamp the timestamp to set (in  nanoseconds).
   */
  void setTimestamp(const Time timestamp);

  /*!
   * Get the timestamp of the grid map.
   * @return timestamp in nanoseconds.
   */
  Time getTimestamp() const;

  /*!
   * Resets the timestamp of the grid map (to zero).
   */
  void resetTimestamp();

  /*!
   * Set the frame id of the grid map.
   * @param frameId the frame id to set.
   */
  void setFrameId(const std::string& frameId);

  /*!
   * Get the frameId of the grid map.
   * @return frameId.
   */
  const std::string& getFrameId() const;

  /*!
   * Get the side length of the grid map.
   * @return side length of the grid map.
   */
  const Length& getLength() const;

  /*!
   * Get the 2d position of the grid map in the grid map frame.
   * @return position of the grid map in the grid map frame.
   */
  const Position& getPosition() const;

  /*!
   * Get the resolution of the grid map.
   * @return resolution of the grid map in the xy plane [m/cell].
   */
  double getResolution() const;

  /*!
   * Get the grid map size (rows and cols of the data structure).
   * @return grid map size.
   */
  const Size& getSize() const;

  /*!
   * Set the start index of the circular buffer.
   * Use this method with caution!
   * @return buffer start index.
   */
  void setStartIndex(const Index& startIndex);

  /*!
   * Get the start index of the circular buffer.
   * @return buffer start index.
   */
  const Index& getStartIndex() const;

  /*!
   * Checks if the buffer is at start index (0,0).
   * @return true if buffer is at default start index.
   */
  bool isDefaultStartIndex() const;

  /*!
   * Rearranges data such that the buffer start index is at (0,0).
   */
  void convertToDefaultStartIndex();

 private:

  /*!
   * Clear a number of columns of the grid map.
   * @param index the left index for the columns to be reset.
   * @param nCols the number of columns to reset.
   */
  void clearCols(unsigned int index, unsigned int nCols);

  /*!
   * Clear a number of rows of the grid map.
   * @param index the upper index for the rows to be reset.
   * @param nRows the number of rows to reset.
   */
  void clearRows(unsigned int index, unsigned int nRows);

  /*!
   * Get cell data at requested position, linearly interpolated from 2x2 cells.
   * @param layer the name of the layer to be accessed.
   * @param position the requested position.
   * @param value the data of the cell.
   * @return true if linear interpolation was successful.
   */
  bool atPositionLinearInterpolated(const std::string& layer, const Position& position, float& value) const;

  /*!
   * Resize the buffer.
   * @param bufferSize the requested buffer size.
   */
  void resize(const Index& bufferSize);

  //! Frame id of the grid map.
  std::string frameId_;

  //! Timestamp of the grid map (nanoseconds).
  Time timestamp_;

  //! Grid map data stored as layers of matrices.
  std::unordered_map<std::string, Matrix> data_;

  //! Names of the data layers.
  std::vector<std::string> layers_;

  //! List of layers from `data_` that are the basic grid map layers.
  //! This means that for a cell to be valid, all basic layers need to be valid.
  //! Also, the basic layers are set to NAN when clearing the map with `clear()`.
  std::vector<std::string> basicLayers_;

  //! Side length of the map in x- and y-direction [m].
  Length length_;

  //! Map resolution in xy plane [m/cell].
  double resolution_;

  //! Map position in the grid map frame [m].
  Position position_;

  //! Size of the buffer (rows and cols of the data structure).
  Size size_;

  //! Circular buffer start indeces.
  Index startIndex_;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} /* namespace */
