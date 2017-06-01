/*
 * GridMap.h
 *
 *  Created on: May 14, 2016
 *      Author: hatem
 */

#ifndef GRIDMAPSA_H_
#define GRIDMAPSA_H_

#include "RoadNetwork.h"

namespace PlannerHNS
{

#define get2dIndex(r,c,w) r*w + c
#define checkGridLimit(r,c,h,w) r >= 0 && c >= 0 && r < h && c < w
#define checkGridIndex(i, nCells) i >= 0 && i < nCells
#define SUBCELL_L 10 //subcell lenth in centimeter


class CELL_Info
{
public:
  int r,c,index;
  GPSPoint center;
  int nCells;
  double heuristic;
  double forwardHeuristic;
  double backwardHeuristic;
  double heuristicValue;
  double forward_heuristicValue;
  double backward_heuristicValue;
  int expanded; // used in path planning
  bool closed;
  double value;
  int action;
  double localize_val;
  double localize_prob;
  std::vector<double> localize_features;
  GPSPoint forwardCenter;
  GPSPoint backwardCenter;
  DIRECTION_TYPE bDir;
  POINT2D bottom_left;
  POINT2D top_right;
  POINT2D bottom_right;
  POINT2D top_left;
  int nStaticPoints;
  int nMovingPoints;

  CELL_Info* pInnerMap;

  std::vector<POINT2D> innerStaticPointsList;
  std::vector<POINT2D> innerMovingPointsList;

  std::vector<GPSPoint> path;


public:
  void InitSubCells(double cell_l, double sub_cell_l);
/**
 * @brief Clear the map contents including obstacle data if bMovingOnly parameter = -1
 * @param bMovingOnly , 1 : clear cell data and moving only points, 0 clear all data including moving and static points, -1 clear data only.
 */
  void Clear(int bMovingOnly);
  void ClearSubCells(bool bMovingOnly);

  CELL_Info();

  virtual ~CELL_Info();

  /*
   * Cell initialization
   */
  void Initialize(POINT2D bottom_left, double cell_l, int row, int col, bool bDefaultEmpty);

  /*
   * assignment operator
   */
  bool operator==(const CELL_Info& cell);

  bool operator!=(const CELL_Info& cell);

  inline bool PointInRect(const POINT2D& p)
   {
     return p.x >= bottom_left.x && p.x <= top_right.x && p.y >= bottom_left.y && p.y <= top_right.y;
   }

  bool TestWithRectangle(RECTANGLE& rec);
  bool TestWithCircle(POINT2D _center,double  width);
   inline bool HitTest(const POINT2D& p);

   void UpdateSubCellCostValue(const std::vector<POINT2D>& ps, const double& cell_l, const double& sub_cell_l);
   void UpdateCostValue(const std::vector<POINT2D>& ps);

   void SaveCell(std::ostream& f);
   void LoadCell(std::ifstream& f);

};

class GridMap
{
  public:

	  pthread_mutex_t update_map_mutex;

    double w, inner_w; // current world width
    double h, inner_h; // current world height
    double cell_l; // cell or block length, if this is an inner cell measurements will be in meter
    double sub_cell_l;
    double origin_x , origin_y;



    int inner_start_row;
    int inner_start_col;
    int inner_end_row;
    int inner_end_col;

    bool m_bEnableInnerMap;
    bool m_bUpdatedMap;



    int wCells, nInnerWCells; // width, number of cells per row
    int hCells, nInnerHCells; // height, number of cells per column
	//POINT2D center;
	int m_MaxHeuristics;

	int m_DisplayResolution;

	POINT2D* delta;


    // This method map obstacles from real world space to Grid space , marking each cell or internal cells as obstacle
	void UpdateMapObstacleValue(const Obstacle& ob);
	void UpdateMapDrivablesValue(const DrivableArea& dr);
	void UpdateMapDrivablesValuePlygon(const std::vector<std::vector<POINT2D> >& points);
	void UpdateMapObstaclesValuePlygon(const std::vector<POINT2D>& poly, std::vector<CELL_Info*>& modifiedCell);

	/**
	 * @brief update cell to indicate that there is an obstacle @ absolute point p
	 * @param p absolute x,y point
	 * @return pointer to the updated cell
	 */
	CELL_Info* UpdateMapObstaclePoint(const POINT2D& p);

	/**
	 * @brief update cell to indicate that there is an obstacle @ absolute point p , and make the map thiner according to a giving threshold
	 * @param p absolute x,y point
	 * @param thiningTHreshold distance to search for old obstacles
	 * @return pointer to the updated cell
	 */
	CELL_Info* UpdateThinMapObstaclePoint(const POINT2D& p, const GPSPoint& carPos, const double& thiningTHreshold);

	/**
	 * @brief update cell to indicate that there is an moving obstacle @ absolute point p
	 * @param p absolute x,y point
	 * @return pointer to the updated cell
	 */
	CELL_Info* UpdateMapMovingObstaclePoint(const POINT2D& p);

	/**
	 * @brief update subcells cost values to reflect the effect of obstacle @ absolute pint p
	 * @param p obstacle point (x,y)
	 * @param currPos current car location to apply the effect of obstacle on p
	 * @return pointer to the updated cell
	 */
	CELL_Info* UpdateMapCostValueRange(const std::vector<POINT2D>& ps, const GPSPoint& currPos, const std::vector<double>& features);
	/**
	 * @brief find the cell @ p then update its localization cost and probability which were read from a map file
	 * @param p absolute position of the cell center
	 * @param localize_val cost value
	 * @param localize_prob probability value (should be zero in case of updating from map file
	 * @return pointer to the updated cell
	 */
	CELL_Info* UpdateSubMapCostValue(const POINT2D& p, const double& localize_val, const double& localize_prob);

	CELL_Info* UpdateMapCostValue(const POINT2D& p, const double& localize_val, const double& localize_prob);

	CELL_Info* GetCellFromPointInnerMap(const POINT2D& p);
	CELL_Info* GetCellFromPoint(const POINT2D& p, bool bExpand = false); // return cell information from (x,y) coordinates
	CELL_Info* GetSubCellFromPoint(const POINT2D& p); // return sub cell information from (x,y) coordinates
	CELL_Info* GetSubCellFromCell(CELL_Info* const parent, const POINT2D& p); // return sub cell information from parent cell

	bool CheckSubCellsInTheWay(const POINT2D& p, const GPSPoint& carPos, const double& thiningTHreshold, std::vector<CELL_Info*>& pSubCellsList);

	/**
	 * @brief Clear the map contents including obstacle data if bMovingOnly parameter = -1
	 * @param bMovingOnly , 1 : clear cell data and moving only points, 0 clear all data including moving and static points, -1 clear data only.
	 */
	void ClearMap(int bMovingOnly);
	void OpenClosedCells();
	void BackupMap();

	GridMap();
    GridMap(double start_x, double start_y, double  map_w, double map_h, double cell_length, bool bDefaultEmpty); // initialize and construct the 2D array of the Grid cells
    void InitInnerMap(double  map_l, GridMap* const pParentMap, const POINT2D& center); // initialize and construct map from another map (cells will point to cells from the other map , width and hight will be maximum available limited by the parameters
    virtual ~GridMap();

    CELL_Info* pCells;
    int nCells;

    void SaveMap(const std::string& mapFilePath, const std::string& mapName);
    void LoadMap(const std::string& mapFilePath, const POINT2D& pos, const double& loadingRadius, const GPSPoint& mapTransformation);

    int GetSurroundingMainCells(const POINT2D& pos, std::vector<CELL_Info*>& cells_list, double max_range=5);
    int GetSurroundingNonObstacleCells(const POINT2D& pos, std::vector<CELL_Info*>& cells_list, double max_range=5);
    int GetSurroundingMainCellsRectangle(const POINT2D& pos,	std::vector<CELL_Info*>& cells_list, RECTANGLE& rect);
    int GetSurroundingMainCellsCircle(const POINT2D& pos,	std::vector<CELL_Info*>& cells_list, double radius);
    int GetSurroundingMainCellsRectangleNoObstacle(const POINT2D& pos,	std::vector<CELL_Info*>& cells_list, RECTANGLE& rect);

    bool IsUpdated()
    {
    	return m_bUpdatedMap;
    }

    void ObservedMap()
    {
    	m_bUpdatedMap = false;
    }

  private:
    int InsidePolygon(const std::vector<POINT2D>& polygon,const POINT2D& p);

    //vector<CELL_Info*> pDrivableCells;
  };

}
#endif /* GRIDMAP_H_ */
