/*
 * GridMap.cpp
 *
 *  Created on: Oct 22, 2013
 *      Author: hatem
 */

#include "GridMap.h"
#include "PlanningHelpers.h"
#include <fstream>

using namespace std;

namespace PlannerHNS
{

GridMap::GridMap(double start_x, double start_y, double map_w, double map_h, double cell_length, bool bDefaultEmpty)
 {
	assert(cell_length > 0);
    assert(map_w>0);
    assert(map_h>0);

    m_bUpdatedMap = false;
    origin_x = start_x ;
    origin_y = start_y;
    m_DisplayResolution = 1;
    sub_cell_l = 0;
	nInnerWCells = nInnerHCells = 0;
	m_bEnableInnerMap = false;
	inner_end_col = inner_end_row = inner_h = inner_start_col = inner_start_row = inner_w = 0;

	w = map_w;
    h = map_h;

    cell_l = cell_length;
    sub_cell_l = cell_l/(double)SUBCELL_L;

    wCells =  w/cell_l;
    hCells =  h/cell_l;

    nCells = wCells*hCells;
	m_MaxHeuristics = w*h*cell_l;

    pCells =  new CELL_Info[nCells];
    POINT2D p;
    int index = 0;

    for(int r=0; r<hCells; r++)
    {
       for(int c=0; c<wCells; c++)
	   {
		 index = get2dIndex(r,c,wCells);
		 p.x = ((double)c * cell_l) + origin_x;
		 p.y = ((double)r * cell_l) + origin_y;
		 pCells[index].Initialize(p, cell_l, r, c,bDefaultEmpty);
		 pCells[index].index = index;

	   }
    }

    double temp[8][3] = { { -1, 0, 1 }, { 0, -1, 1 }, { 1, 0, 1 }, { 0, 1, 1 }, { -1, -1, 1.5 }, { 1, -1, 1.5 }, { 1, 1, 1.5 }, { -1, 1, 1.5 } }; // left, down, right, top, left down, right down, top right, left top
    delta = new POINT2D[8];
    for (int i = 0; i < 8; i++)
    	delta[i] = POINT2D(temp[i][0], temp[i][1]);
 }

int GridMap::InsidePolygon(const vector<POINT2D>& polygon,const POINT2D& p)
{
  int counter = 0;
  int i;
  double xinters;
  POINT2D p1,p2;
  int N = polygon.size();
  if(N <=0 ) return -1;

  p1 = polygon.at(0);
  for (i=1;i<=N;i++)
  {
    p2 = polygon.at(i % N);

    if (p.y > MIN(p1.y,p2.y))
    {
      if (p.y <= MAX(p1.y,p2.y))
      {
        if (p.x <= MAX(p1.x,p2.x))
        {
          if (p1.y != p2.y)
          {
            xinters = (p.y-p1.y)*(p2.x-p1.x)/(p2.y-p1.y)+p1.x;
            if (p1.x == p2.x || p.x <= xinters)
              counter++;
          }
        }
      }
    }
    p1 = p2;
  }

  if (counter % 2 == 0)
    return 0;
  else
    return 1;
}


  void GridMap::OpenClosedCells()
  {
	int loop_size =  nCells;
	int index = 0;
	while(index != loop_size)
	{
			pCells[index].closed = false;
			pCells[index].expanded = -1;
			index++;
	}
  }

  void GridMap::ClearMap(int bMovingOnly)
  {
  	int loop_size =  nCells;
  	int index = 0;
	  while(index != loop_size)
		{
		  pCells[index].Clear(bMovingOnly);
		index++;
	  }
	  m_bUpdatedMap = true;
  }

  void GridMap::UpdateMapObstacleValue(const Obstacle& ob)
  {
	  POINT2D p1, p2, p3, p4;
		p1 = ob.sp;
		p2 = ob.ep;

		if(ob.polygon.size() == 0)
		{
			int loop_size =  nCells;
			int index = 0;
			while(index != loop_size)
			{
					p3 = pCells[index].bottom_left;
					p4 = pCells[index].top_right;

					if(! ( p2.y < p3.y || p1.y > p4.y || p2.x < p3.x || p1.x > p4.x ))
					{
						pCells[index].nStaticPoints++;
						m_bUpdatedMap = true;
					}

					index++;
			}
		}
		else
		{
			vector<CELL_Info*> modList;
			UpdateMapObstaclesValuePlygon(ob.polygon, modList);
		}
  }

  void GridMap::UpdateMapObstaclesValuePlygon(const vector<POINT2D>& poly, vector<CELL_Info*>& modifiedCell)
  {

	  POINT2D minP, maxP;
	  CELL_Info* minC, *maxC;
	  int index = 0;

	  minP = poly[0];
	  maxP = poly[0];

		for(unsigned int j=1; j< poly.size(); j++)
		{
			if(poly[j].x < minP.x) minP.x = poly[j].x;
			if(poly[j].y < minP.y) minP.y = poly[j].y;

			if(poly[j].x > maxP.x) maxP.x = poly[j].x;
			if(poly[j].y > maxP.y) maxP.y = poly[j].y;
		}


		minC = GetCellFromPoint(minP,false);
		maxC = GetCellFromPoint(maxP, false);

		if(!maxC || ! minC)
		{
			printf("Obstacle Polygon is outside the Map !!");
			return;
		}

		for(int r=minC->r; r<=maxC->r; r++)
		{
			for(int c=minC->c; c<=maxC->c; c++)
			{
				index = get2dIndex(r,c,wCells);
				//CELL_Info* pSub = &pCells[index];

//					if(pSub->pInnerMap == 0)
//					{
//						pSub->InitSubCells(cell_l, sub_cell_l);
//					}
//						int index_sub = 0;
//						bool bUpdatedSubCell = false;
//						while(index_sub < pSub->nCells)
//						{
//							if(InsidePolygon(&poly, poly.size(), pSub->pInnerMap[index_sub].bottom_left)==1 || InsidePolygon(&poly, poly.size(), pSub->pInnerMap[index_sub].top_right)==1)
//							{
//								pSub->pInnerMap[index_sub].nStaticPoints = 1;
//								m_bUpdatedMap = true;
//								bUpdatedSubCell = true;
//							}
//							index_sub++;
//						}


				//if(bUpdatedSubCell)
				{
					POINT2D bl= pCells[index].bottom_left;
					bl.x += 0.01;
					bl.y += 0.01;
					POINT2D tr= pCells[index].top_right;
					tr.x -= 0.01;
					tr.y -= 0.01;
					if(InsidePolygon(poly, bl)==1 || InsidePolygon(poly, tr)==1)
					{

							pCells[index].nMovingPoints = 1;
							pCells[index].nStaticPoints = 1;
//							pCells[index].forwardHeuristic = nCells;
//							pCells[index].backwardHeuristic = nCells;
//							pCells[index].heuristic = nCells;
							modifiedCell.push_back(&pCells[index]);
							m_bUpdatedMap = true;
							//pDrivableCells.push_back(&pCells[r][c]);
					}
				}
			}
		}
  }

  void GridMap::UpdateMapDrivablesValuePlygon(const vector<vector<POINT2D> >& points)
  {
//		for(int r=0; r<nColCells; r++)
//		{
//			for(int c=0; c<nRowCells; c++)
//			{
//				if(pCells[r][c].cell.bObstacle)
//				{
//					for(unsigned int i=0; i< points->size(); i++)
//					{
//						vector<POINT2D>* poly = &((*points)[i]);
//						if(InsidePolygon(poly, poly->size(), pCells[r][c].cell.bottom_left)==1 || InsidePolygon(poly, poly->size(), pCells[r][c].cell.top_right)==1)
//								pCells[r][c].cell.bObstacle = false;
//					}
//				}
//
//			}
//		}

	  POINT2D minP, maxP;
	  CELL_Info* minC, *maxC;
	  int index = 0;

	  for(unsigned int i=0; i< points.size(); i++)
		{
			vector<POINT2D> poly = points.at(i);
			maxP = minP = poly[0];

			for(unsigned int j=1; j< poly.size(); j++)
			{
				if(poly[j].x < minP.x) minP.x = poly[j].x;
				if(poly[j].y < minP.y) minP.y = poly[j].y;

				if(poly[j].x > maxP.x) maxP.x = poly[j].x;
				if(poly[j].y > maxP.y) maxP.y = poly[j].y;
			}


			minC = GetCellFromPoint(minP,false);
			maxC = GetCellFromPoint(maxP, false);

			for(int r=minC->r; r<maxC->r; r++)
			{
				for(int c=minC->c; c<maxC->c; c++)
				{
					index = get2dIndex(r,c,wCells);

					CELL_Info* pSub = &pCells[index];

					if(pSub->pInnerMap != 0)
					{
						pSub->InitSubCells(cell_l, sub_cell_l);
					}
						int index_sub = 0;
						//bool bUpdatedSubCell = false;
						while(index_sub < pSub->nCells)
						{
							if(InsidePolygon(poly, pSub->pInnerMap[index_sub].bottom_left)==1 || InsidePolygon(poly, pSub->pInnerMap[index_sub].top_right)==1)
							{
								pSub->pInnerMap[index_sub].nStaticPoints = 0;
								m_bUpdatedMap = true;
								//bUpdatedSubCell = true;
							}
							index_sub++;
						}


					//if(bUpdatedSubCell)
					{
						if(InsidePolygon(poly, pCells[index].bottom_left)==1 || InsidePolygon(poly, pCells[index].top_right)==1)
						{

								pCells[index].nStaticPoints = 0;
								m_bUpdatedMap = true;
								//pDrivableCells.push_back(&pCells[r][c]);
						}
					}
				}
			}
		}
  }

  void GridMap::UpdateMapDrivablesValue(const DrivableArea& dr)
    {
  	  POINT2D p1, p2, p3, p4;
  	  p1 = dr.sp;
  	  p2 = dr.ep;

  		int loop_size =  nCells;
  	int index = 0;
	  while(index != loop_size)
		{
				p3 = pCells[index].bottom_left;
				p4 = pCells[index].top_right;

				if(! ( p2.y < p3.y || p1.y > p4.y || p2.x < p3.x || p1.x > p4.x ))
				{
					pCells[index].nStaticPoints = 0;
					m_bUpdatedMap = true;
				}

				index++;
		}
    }

  void GridMap::InitInnerMap(double  map_l, GridMap* const pParentMap, const POINT2D& center)
  {
	  int min_row, min_col, max_row, max_col;
	  CELL_Info* scell = GetCellFromPoint(center);
		if(!scell) return;

		//Get First Left Cell
		double max_left = scell->c * pParentMap->cell_l;
		if(max_left < map_l)
		{
			min_row = 0;
		}
		else
		{
			POINT2D p(center.x-map_l, center.y);
			min_row = GetCellFromPoint(p)->r;
		}

		double max_right = (pParentMap->hCells - scell->c) * pParentMap->cell_l;
		if(max_right < map_l)
		{
			max_row = pParentMap->hCells;
		}
		else
		{
			POINT2D p(center.x+map_l, center.y);
			max_row = GetCellFromPoint(p)->r;
		}

		double max_bottom = scell->r * pParentMap->cell_l;
		if(max_bottom < map_l)
		{
			min_col = 0;
		}
		else
		{
			POINT2D p(center.x, center.y-map_l);
			min_col = GetCellFromPoint(p)->c;
		}

		double max_top = (pParentMap->wCells- scell->r) * pParentMap->cell_l;
		if(max_top < map_l)
		{
			max_col = pParentMap->wCells;
		}
		else
		{
			POINT2D p(center.x, center.y+map_l);
			max_col = GetCellFromPoint(p)->c;
		}

		inner_start_row = min_row;
		inner_start_col = min_col;
		inner_end_row = max_row;
		inner_end_col = max_col;
		nInnerWCells =max_col-min_col;
		nInnerHCells = max_row-min_row;
		inner_w =  nInnerWCells* pParentMap->cell_l;
		inner_h =  nInnerHCells* pParentMap->cell_l;
		cell_l = pParentMap->cell_l;
		sub_cell_l = pParentMap->sub_cell_l;


  }

 CELL_Info* GridMap::UpdateMapCostValueRange(const vector<POINT2D>& ps, const GPSPoint& currPos, const vector<double>& features)
  {
	 POINT2D pos(currPos.x, currPos.y);
	CELL_Info* pC = GetCellFromPoint(pos);



	if(pC)
	{
		//Update Affected cells value from this new point
		vector<CELL_Info*> cells;
		GetSurroundingMainCells(pos, cells, 1);
		for(unsigned int i=0; i< cells.size(); i++)
		{
			cells[i]->UpdateSubCellCostValue(ps, cell_l, sub_cell_l);
			cells[i]->UpdateCostValue(ps);
			m_bUpdatedMap = true;
		}
		//only one level
//			pC->UpdateSubCellCostValue(p, cell_l, sub_cell_l);

	}

	return pC;
  }

 bool GridMap::CheckSubCellsInTheWay(const POINT2D& p, const GPSPoint& carPos, const double& thiningThreshold, vector<CELL_Info*>& pSubCellsList)
 {
	 POINT2D v(p.x - carPos.x, p.y - carPos.y);
	 double v_norm = pointNorm(v);


	 //Number of search iteration will be a ratio between the thining threshold   and the sub cell length
	 double step_d = sub_cell_l;
	 double start_d = -thiningThreshold/ 2.0;
	 pSubCellsList.clear();
	 CELL_Info* pSubCell = 0;
	 while(start_d < thiningThreshold)
	 {
		 POINT2D p_obstacle = p;
		 p_obstacle.x += v.x/v_norm * start_d;
		 p_obstacle.y += v.y/v_norm * start_d;
		 pSubCell = GetSubCellFromPoint(p_obstacle);
		 if(pSubCell && pSubCell->nStaticPoints>0)
			 return true;

		pSubCellsList.push_back(pSubCell);
		 start_d += step_d;
	 }

	 return false;
 }

 CELL_Info* GridMap::UpdateThinMapObstaclePoint(const POINT2D& p, const GPSPoint& carPos,const double& thiningTHreshold)
 {
	 CELL_Info* pC = GetCellFromPoint(p);
	if(pC)
	{
		vector<CELL_Info*> subcells_list;

		if(!CheckSubCellsInTheWay(p, carPos, thiningTHreshold, subcells_list))
		{
			pC->nStaticPoints++;
			CELL_Info* pSubc = GetSubCellFromCell(pC, p);
			if(pSubc)
			{
				if(pSubc->nStaticPoints<1)
				{
					pSubc->innerStaticPointsList.push_back(p);
					m_bUpdatedMap = true;
				}

				pSubc->nStaticPoints++;
			}
		}
	}
	return pC;
 }

 CELL_Info* GridMap::UpdateMapObstaclePoint(const POINT2D& p)
{
	CELL_Info* pC = GetCellFromPoint(p);
	if(pC)
	{
		if(pC->nStaticPoints < 5)
			pC->nStaticPoints++;
		CELL_Info* pSubc = GetSubCellFromCell(pC, p);
		if(pSubc)
		{
			if(pSubc->nStaticPoints<1)
			{
				//pthread_mutex_lock(&update_map_mutex);
				pSubc->innerStaticPointsList.push_back(p);
				m_bUpdatedMap = true;
				//pthread_mutex_unlock(&update_map_mutex);
			}

			if(pSubc->nStaticPoints < 5)
				pSubc->nStaticPoints++;
		}
	}
	return pC;
}

CELL_Info* GridMap::UpdateMapMovingObstaclePoint(const POINT2D& p)
{
	CELL_Info* pC = GetCellFromPoint(p);
	if(pC)
	{
		if(pC->nMovingPoints < 5)
			pC->nMovingPoints++;
		CELL_Info* pSubc = GetSubCellFromCell(pC, p);
		if(pSubc)
		{
			if(pSubc->nMovingPoints<1)
			{
				//pthread_mutex_lock(&update_map_mutex);
				pSubc->innerMovingPointsList.push_back(p);
				m_bUpdatedMap = true;
				//pthread_mutex_unlock(&update_map_mutex);
			}

			if(pSubc->nMovingPoints<5)
				pSubc->nMovingPoints++;
		}
	}
	return pC;
}

 CELL_Info* GridMap::UpdateMapCostValue(const POINT2D& p, const double& localize_val, const double& localize_prob)
 {
 	CELL_Info* pC = GetCellFromPoint(p);
 	if(pC)
 	{
 		pC->localize_val = localize_val;
 		pC->localize_prob = localize_prob;
 		m_bUpdatedMap = true;
 	}

 	return pC;
 }

 CELL_Info* GridMap::UpdateSubMapCostValue(const POINT2D& p, const double& localize_val, const double& localize_prob)
 {
 	CELL_Info* pC = GetCellFromPoint(p);
 	if(pC)
 	{

 		CELL_Info* pSubc = GetSubCellFromCell(pC, p);
 		if(pSubc)
 		{
 			pSubc->localize_val = localize_val;
 			pSubc->localize_prob = localize_prob;
 			m_bUpdatedMap = true;
 		}
 	}

 	return pC;
 }

CELL_Info* GridMap::GetSubCellFromCell(CELL_Info* const parent, const POINT2D& p)
{
	if(!parent) return 0;

	if(!parent->pInnerMap)
		parent->InitSubCells(cell_l, sub_cell_l);

	int row = floor((p.y - parent->bottom_left.y)/sub_cell_l);
	int col = floor((p.x - parent->bottom_left.x)/sub_cell_l);

	if(row>=0 && row<SUBCELL_L && col >=0 && col < SUBCELL_L)
		return &parent->pInnerMap[get2dIndex(row,col,SUBCELL_L)];
	else
		return 0;
}

CELL_Info* GridMap::GetSubCellFromPoint(const POINT2D& p)
{
	CELL_Info* pMainCell = GetCellFromPoint(p);
	if(pMainCell)
	{
		if(!pMainCell->pInnerMap)
			pMainCell->InitSubCells(cell_l, sub_cell_l);

		int row = floor((p.y - pMainCell->bottom_left.y)/sub_cell_l);
		int col = floor((p.x - pMainCell->bottom_left.x)/sub_cell_l);

		return &pMainCell->pInnerMap[get2dIndex(row,col,SUBCELL_L)];
	}
	else
		return 0;
}

CELL_Info* GridMap::GetCellFromPoint(const POINT2D& p, bool bExpand)
{

	  int row = floor((p.y-origin_y) /cell_l);
	  int col = floor((p.x-origin_x) /cell_l);

	  if(row>=0 && row < hCells && col >=0 && col < wCells)
	  {
//		  POINT2D _p(p.x , p.y );
//		bool exist = pCells[get2dIndex(row,col,nColCells)].PointInRect(p);
//
//		if(!exist)
//			return 0;

		  //retCell = pCells[row][col];
		  //retCell.center.a = p.a;
		  int index = get2dIndex(row,col,wCells);
		  if(index >= 0 && index < nCells)
			  return &pCells[index];
		  else
			  printf("Error Getting Cell with Info: P(%f,%f) , C(%d,%d), index = %d", p.x, p.y, row, col, index);
	  }
	  else if(bExpand)
	  {
		  //first get extend direction and factor
		  double lf=0, rf=0,tf=0,bf=0;
		  int nRC=0,nCC=0;
		  if(fabsf(p.x) >= 0)
			nRC= (fabsf(p.x) - 0)/cell_l + 1;

		  if(fabsf(p.y) >= 0)
			nCC= (fabsf(p.y) - 0)/cell_l + 1;

		  if(p.x > 0)
			  rf = nRC*4.0;
		  else
			  lf = nRC*4.0;

		  if(p.y > 0)
			  tf = nCC*4.0;
		  else
			  bf = nCC*4.0;

	  }

    return 0;
  }


CELL_Info* GridMap::GetCellFromPointInnerMap(const POINT2D& p)
{
  int row = floor((p.y - origin_y) /cell_l);
  int col = floor((p.x - origin_x)/cell_l);

  if(row>=inner_start_row && row < inner_end_row && col >=inner_start_col && col < inner_end_col)
  {

	  POINT2D _p(p.x , p.y );

	bool exist = pCells[get2dIndex(row,col,wCells)].PointInRect(p);

	if(!exist)
		return 0;

	  //retCell = pCells[row][col];
	  //retCell.center.a = p.a;
	  return &pCells[get2dIndex(row,col,wCells)];
  }

  return 0;

}

  void GridMap::BackupMap()
  {
  }

  GridMap::GridMap()
	{
	  //update_map_mutex = PTHREAD_MUTEX_INITIALIZER;
		sub_cell_l = 0;
		nCells = 0;
		nInnerWCells = nInnerHCells = 0;
		m_bEnableInnerMap = false;
		inner_end_col = inner_end_row = inner_h = inner_start_col = inner_start_row = inner_w = 0;
		w = h = cell_l  = wCells = hCells = m_MaxHeuristics = 0;
		pCells = 0;
		m_DisplayResolution = 1;
		delta = 0;
		origin_y = 0;
		origin_x =0;
		m_bUpdatedMap  = false;
	}

  GridMap::~GridMap()
  {
    if(pCells)
    {
      delete [] pCells;
      pCells = 0;
    }
    if(delta)
    {
    	delete [] delta;
    	delta = 0;
    }
  }

  int GridMap::GetSurroundingNonObstacleCells(const POINT2D& pos, vector<CELL_Info*>& cells_list, double max_range)
  {
	  int nMaxLevels = max_range/cell_l;

	int r, c;
	vector<CELL_Info*> nextLevel;
	vector<CELL_Info*> currentLevel;
	vector<CELL_Info*> tempLevel;

	CELL_Info* originalGoal = GetCellFromPoint(pos);
	if(!originalGoal) return 0;

	CELL_Info* tempCell;


	currentLevel.push_back(originalGoal);
	cells_list.push_back(originalGoal);
	int counter = 0;
	int index = 0;

	while (currentLevel.size() > 0 && nMaxLevels>0)
	{
		tempCell = currentLevel.back();
		currentLevel.pop_back();

		for (int i = 0; i < 8; i++)
		{
			counter++;
			r = tempCell->r + delta[i].x;
			c = tempCell->c + delta[i].y;
			index = get2dIndex(r,c,wCells);

			if (r >= 0 && c >= 0 && r < hCells && c < wCells)
			{
				if(pCells[index].nMovingPoints>0 || pCells[index].nStaticPoints > 0 || pCells[index].heuristic == m_MaxHeuristics)
					continue;
				//insert unique
				bool bFound = false;
				for(unsigned int j=0; j< cells_list.size();j++)
				{
					if(cells_list[j] == &pCells[index])
					{
						bFound = true;
						break;
					}
				}
				if(!bFound)
				{
					cells_list.push_back(&pCells[index]);
					nextLevel.push_back(&pCells[index]);
				}
			}
		}

		if (currentLevel.size() == 0 && nextLevel.size() > 0)
		{
			tempLevel = currentLevel;
			currentLevel = nextLevel;
			nextLevel = tempLevel;
			nMaxLevels--;
		}
	}

	return counter;
  }

  int GridMap::GetSurroundingMainCellsRectangleNoObstacle(const POINT2D& pos,	vector<CELL_Info*>& cells_list, RECTANGLE& rect)
    {

    	//calculate the number of levels that satisfy the max range criteria
    	int nMaxLevels = hypot(rect.width, rect.length) / cell_l;

    	CELL_Info* originalGoal = GetCellFromPoint(pos);

    	if (!originalGoal)
    		return 0;

    	cells_list.push_back(originalGoal);

    	if (nMaxLevels <= 1)
    		return 1;

    	nMaxLevels--;

    	vector<pair<CELL_Info*, POINT2D> > straitCells;
    	vector<pair<CELL_Info*, POINT2D> > diagonalCells;
    	vector<pair<CELL_Info*, POINT2D> > straitCellsL2;
    	vector<pair<CELL_Info*, POINT2D> > diagonalCellsL2;
    	int r, c;
    	CELL_Info* tempCell = 0;
    	int counter = 1;
    	int index = 0;

    	POINT2D mask;
    	//first level , // left, down, right, top, left down, right down, top right, left top
    	//strait degree
    	for (unsigned int i = 0; i < 4; i++)
    	{
    		mask.x = delta[i].x;
    		mask.y = delta[i].y;
    		r = originalGoal->r + mask.x;
    		c = originalGoal->c + mask.y;
    		index = get2dIndex(r, c, wCells);

    		if(checkGridIndex(index, nCells) && pCells[index].nMovingPoints == 0 && pCells[index].nStaticPoints == 0 && pCells[index].heuristic != m_MaxHeuristics)
  		{
  			straitCells.push_back(make_pair(&pCells[index], mask));
  			if (!pCells[index].TestWithRectangle(rect))
  				cells_list.push_back(&pCells[index]);
  		}
    	}

    	//diagonal degree
    	for (unsigned int i = 4; i < 8; i++)
    	{
    		mask.x = delta[i].x;
    		mask.y = delta[i].y;
    		r = originalGoal->r + mask.x;
    		c = originalGoal->c + mask.y;
    		index = get2dIndex(r, c, wCells);
    		if(checkGridIndex(index, nCells) && pCells[index].nMovingPoints == 0 && pCells[index].nStaticPoints == 0 && pCells[index].heuristic != m_MaxHeuristics)
    		{
    			diagonalCells.push_back(make_pair(&pCells[index], mask));
  			if (!pCells[index].TestWithRectangle(rect))
  				cells_list.push_back(&pCells[index]);
    		}
    	}

    	nMaxLevels--;
    	counter++;

    	while (nMaxLevels > 0)
    	{
    		straitCellsL2.clear();
    		diagonalCellsL2.clear();
    		while (straitCells.size() > 0)
    		{
    			mask = straitCells.back().second;
    			tempCell = straitCells.back().first;
    			r = tempCell->r + mask.x;
    			c = tempCell->c + mask.y;
    			index = get2dIndex(r, c, wCells);
    			if(checkGridIndex(index, nCells) && pCells[index].nMovingPoints == 0 && pCells[index].nStaticPoints == 0 && pCells[index].heuristic != m_MaxHeuristics)
    			{
  				straitCellsL2.push_back(make_pair(&pCells[index], mask));
  				if (!pCells[index].TestWithRectangle(rect))
  					cells_list.push_back(&pCells[index]);
    			}

    			//diagonal 1
    			if (straitCells.back().second.x == 0)
    				mask.x += 1;
    			else
    				mask.y += 1;
    			r = tempCell->r + mask.x;
    			c = tempCell->c + mask.y;
    			index = get2dIndex(r, c, wCells);
    			if(checkGridIndex(index, nCells) && pCells[index].nMovingPoints == 0 && pCells[index].nStaticPoints == 0 && pCells[index].heuristic != m_MaxHeuristics)
    			{
  				diagonalCellsL2.push_back(make_pair(&pCells[index], mask));
  				if (!pCells[index].TestWithRectangle(rect))
  					cells_list.push_back(&pCells[index]);
    			}

    			//diagonal 2
    			if (straitCells.back().second.x == 0)
    				mask.x += -2;
    			else
    				mask.y += -2;
    			r = tempCell->r + mask.x;
    			c = tempCell->c + mask.y;
    			index = get2dIndex(r, c, wCells);
    			if(checkGridIndex(index, nCells) && pCells[index].nMovingPoints == 0 && pCells[index].nStaticPoints == 0 && pCells[index].heuristic != m_MaxHeuristics)
    			{
  				diagonalCellsL2.push_back(make_pair(&pCells[index], mask));
  				if (!pCells[index].TestWithRectangle(rect))
  					cells_list.push_back(&pCells[index]);
    			}

    			straitCells.pop_back();
    		}

    		//Diagonal
    		while (diagonalCells.size() > 0)
    		{
    			mask = diagonalCells.back().second;
    			tempCell = diagonalCells.back().first;
    			r = tempCell->r + mask.x;
    			c = tempCell->c + mask.y;
    			index = get2dIndex(r, c, wCells);
    			if(checkGridIndex(index, nCells) && pCells[index].nMovingPoints == 0 && pCells[index].nStaticPoints == 0 && pCells[index].heuristic != m_MaxHeuristics)
    			{
  				diagonalCellsL2.push_back(make_pair(&pCells[index], mask));
  				if (!pCells[index].TestWithRectangle(rect))
  					cells_list.push_back(&pCells[index]);
    			}

    			diagonalCells.pop_back();
    		}

    		nMaxLevels--;
    		counter++;
    		if (nMaxLevels <= 0)
    			break;

    		straitCells = straitCellsL2;
    		diagonalCells = diagonalCellsL2;

    	}

    	return counter;

    }

  int GridMap::GetSurroundingMainCellsRectangle(const POINT2D& pos,	vector<CELL_Info*>& cells_list, RECTANGLE& rect)
  {

  	//calculate the number of levels that satisfy the max range criteria
  	int nMaxLevels = hypot(rect.width, rect.length) / cell_l;

  	CELL_Info* originalGoal = GetCellFromPoint(pos);

  	if (!originalGoal)
  		return 0;

  	cells_list.push_back(originalGoal);

  	if (nMaxLevels <= 1)
  		return 1;

  	nMaxLevels--;

  	vector<pair<CELL_Info*, POINT2D> > straitCells;
  	vector<pair<CELL_Info*, POINT2D> > diagonalCells;
  	vector<pair<CELL_Info*, POINT2D> > straitCellsL2;
  	vector<pair<CELL_Info*, POINT2D> > diagonalCellsL2;
  	int r, c;
  	CELL_Info* tempCell = 0;
  	int counter = 1;
  	int index = 0;

  	POINT2D mask;
  	//first level , // left, down, right, top, left down, right down, top right, left top
  	//strait degree
  	for (unsigned int i = 0; i < 4; i++)
  	{
  		mask.x = delta[i].x;
  		mask.y = delta[i].y;
  		r = originalGoal->r + mask.x;
  		c = originalGoal->c + mask.y;
  		index = get2dIndex(r, c, wCells);
  		if(checkGridIndex(index, nCells))
		{
			straitCells.push_back(make_pair(&pCells[index], mask));
			if (!pCells[index].TestWithRectangle(rect))
				cells_list.push_back(&pCells[index]);
		}
  	}

  	//diagonal degree
  	for (unsigned int i = 4; i < 8; i++)
  	{
  		mask.x = delta[i].x;
  		mask.y = delta[i].y;
  		r = originalGoal->r + mask.x;
  		c = originalGoal->c + mask.y;
  		index = get2dIndex(r, c, wCells);
  		if(checkGridIndex(index, nCells))
  		{
  			diagonalCells.push_back(make_pair(&pCells[index], mask));
			if (!pCells[index].TestWithRectangle(rect))
				cells_list.push_back(&pCells[index]);
  		}
  	}

  	nMaxLevels--;
  	counter++;

  	while (nMaxLevels > 0)
  	{
  		straitCellsL2.clear();
  		diagonalCellsL2.clear();
  		while (straitCells.size() > 0)
  		{
  			mask = straitCells.back().second;
  			tempCell = straitCells.back().first;
  			r = tempCell->r + mask.x;
  			c = tempCell->c + mask.y;
  			index = get2dIndex(r, c, wCells);
  			if(checkGridIndex(index, nCells))
  			{
				straitCellsL2.push_back(make_pair(&pCells[index], mask));
				if (!pCells[index].TestWithRectangle(rect))
					cells_list.push_back(&pCells[index]);
  			}

  			//diagonal 1
  			if (straitCells.back().second.x == 0)
  				mask.x += 1;
  			else
  				mask.y += 1;
  			r = tempCell->r + mask.x;
  			c = tempCell->c + mask.y;
  			index = get2dIndex(r, c, wCells);
  			if(checkGridIndex(index, nCells))
  			{
				diagonalCellsL2.push_back(make_pair(&pCells[index], mask));
				if (!pCells[index].TestWithRectangle(rect))
					cells_list.push_back(&pCells[index]);
  			}

  			//diagonal 2
  			if (straitCells.back().second.x == 0)
  				mask.x += -2;
  			else
  				mask.y += -2;
  			r = tempCell->r + mask.x;
  			c = tempCell->c + mask.y;
  			index = get2dIndex(r, c, wCells);
  			if(checkGridIndex(index, nCells))
  			{
				diagonalCellsL2.push_back(make_pair(&pCells[index], mask));
				if (!pCells[index].TestWithRectangle(rect))
					cells_list.push_back(&pCells[index]);
  			}

  			straitCells.pop_back();
  		}

  		//Diagonal
  		while (diagonalCells.size() > 0)
  		{
  			mask = diagonalCells.back().second;
  			tempCell = diagonalCells.back().first;
  			r = tempCell->r + mask.x;
  			c = tempCell->c + mask.y;
  			index = get2dIndex(r, c, wCells);
  			if(checkGridIndex(index, nCells))
  			{
				diagonalCellsL2.push_back(make_pair(&pCells[index], mask));
				if (!pCells[index].TestWithRectangle(rect))
					cells_list.push_back(&pCells[index]);
  			}

  			diagonalCells.pop_back();
  		}

  		nMaxLevels--;
  		counter++;
  		if (nMaxLevels <= 0)
  			break;

  		straitCells = straitCellsL2;
  		diagonalCells = diagonalCellsL2;

  	}

  	return counter;

  }

  int GridMap::GetSurroundingMainCellsCircle(const POINT2D& pos,	vector<CELL_Info*>& cells_list, double radius)
    {

    	//calculate the number of levels that satisfy the max range criteria
    	int nMaxLevels = radius * 2.0 / cell_l;

    	CELL_Info* originalGoal = GetCellFromPoint(pos);

    	if (!originalGoal)
    		return 0;

    	cells_list.push_back(originalGoal);

    	if (nMaxLevels <= 1)
    		return 1;

    	nMaxLevels--;

    	vector<pair<CELL_Info*, POINT2D> > straitCells;
    	vector<pair<CELL_Info*, POINT2D> > diagonalCells;
    	vector<pair<CELL_Info*, POINT2D> > straitCellsL2;
    	vector<pair<CELL_Info*, POINT2D> > diagonalCellsL2;
    	int r, c;
    	CELL_Info* tempCell = 0;
    	int counter = 1;
    	int index = 0;

    	POINT2D mask;
    	//first level , // left, down, right, top, left down, right down, top right, left top
    	//strait degree
    	for (unsigned int i = 0; i < 4; i++)
    	{
    		mask.x = delta[i].x;
    		mask.y = delta[i].y;
    		r = originalGoal->r + mask.x;
    		c = originalGoal->c + mask.y;
    		index = get2dIndex(r, c, wCells);
    		if(checkGridIndex(index, nCells))
  		{
  			straitCells.push_back(make_pair(&pCells[index], mask));
  			if (pCells[index].TestWithCircle(pos, radius))
  				cells_list.push_back(&pCells[index]);
  		}
    	}

    	//diagonal degree
    	for (unsigned int i = 4; i < 8; i++)
    	{
    		mask.x = delta[i].x;
    		mask.y = delta[i].y;
    		r = originalGoal->r + mask.x;
    		c = originalGoal->c + mask.y;
    		index = get2dIndex(r, c, wCells);
    		if(checkGridIndex(index, nCells))
    		{
    			diagonalCells.push_back(make_pair(&pCells[index], mask));
    			if (pCells[index].TestWithCircle(pos, radius))
    				cells_list.push_back(&pCells[index]);
    		}
    	}

    	nMaxLevels--;
    	counter++;

    	while (nMaxLevels > 0)
    	{
    		straitCellsL2.clear();
    		diagonalCellsL2.clear();
    		while (straitCells.size() > 0)
    		{
    			mask = straitCells.back().second;
    			tempCell = straitCells.back().first;
    			r = tempCell->r + mask.x;
    			c = tempCell->c + mask.y;
    			index = get2dIndex(r, c, wCells);
    			if(checkGridIndex(index, nCells))
    			{
    				straitCellsL2.push_back(make_pair(&pCells[index], mask));
    				if (pCells[index].TestWithCircle(pos, radius))
    					cells_list.push_back(&pCells[index]);
    			}

    			//diagonal 1
    			if (straitCells.back().second.x == 0)
    				mask.x += 1;
    			else
    				mask.y += 1;
    			r = tempCell->r + mask.x;
    			c = tempCell->c + mask.y;
    			index = get2dIndex(r, c, wCells);
    			if(checkGridIndex(index, nCells))
    			{
					diagonalCellsL2.push_back(make_pair(&pCells[index], mask));
					if (pCells[index].TestWithCircle(pos, radius))
						cells_list.push_back(&pCells[index]);
    			}

    			//diagonal 2
    			if (straitCells.back().second.x == 0)
    				mask.x += -2;
    			else
    				mask.y += -2;
    			r = tempCell->r + mask.x;
    			c = tempCell->c + mask.y;
    			index = get2dIndex(r, c, wCells);
    			if(checkGridIndex(index, nCells))
    			{
    				diagonalCellsL2.push_back(make_pair(&pCells[index], mask));
    				if (pCells[index].TestWithCircle(pos, radius))
    					cells_list.push_back(&pCells[index]);
    			}

    			straitCells.pop_back();
    		}

    		//Diagonal
    		while (diagonalCells.size() > 0)
    		{
    			mask = diagonalCells.back().second;
    			tempCell = diagonalCells.back().first;
    			r = tempCell->r + mask.x;
    			c = tempCell->c + mask.y;
    			index = get2dIndex(r, c, wCells);
    			if(checkGridIndex(index, nCells))
    			{
					diagonalCellsL2.push_back(make_pair(&pCells[index], mask));
					if (pCells[index].TestWithCircle(pos, radius))
						cells_list.push_back(&pCells[index]);
    			}

    			diagonalCells.pop_back();
    		}

    		nMaxLevels--;
    		counter++;
    		if (nMaxLevels <= 0)
    			break;

    		straitCells = straitCellsL2;
    		diagonalCells = diagonalCellsL2;

    	}

    	return counter;

    }

  int GridMap::GetSurroundingMainCells(const POINT2D& pos, vector<CELL_Info*>& cells_list, double max_range)
  {

	  //calculate the number of levels that satisfy the max range criteria
	int nMaxLevels = max_range/cell_l;

  	int r, c;
  	vector<CELL_Info*> nextLevel;
  	vector<CELL_Info*> currentLevel;
  	vector<CELL_Info*> tempLevel;

  	CELL_Info* originalGoal = GetCellFromPoint(pos);
  	if(!originalGoal) return 0;

  	CELL_Info* tempCell;


  	currentLevel.push_back(originalGoal);
  	cells_list.push_back(originalGoal);
  	int counter = 0;
  	int index = 0;

  	while (currentLevel.size() > 0 && nMaxLevels>0)
  	{
  		tempCell = currentLevel.back();
  		currentLevel.pop_back();

  		for (int i = 0; i < 8; i++)
  		{
  			counter++;
  			r = tempCell->r + delta[i].x;
  			c = tempCell->c + delta[i].y;
  			index = get2dIndex(r,c,wCells);

  			if (r >= 0 && c >= 0 && r < hCells && c < wCells)
  			{
  				//insert unique
  				bool bFound = false;
  				for(unsigned int j=0; j< cells_list.size();j++)
  				{
  					if(cells_list[j] == &pCells[index])
  					{
  						bFound = true;
  						break;
  					}
  				}
  				if(!bFound)
  				{
  					cells_list.push_back(&pCells[index]);
  					nextLevel.push_back(&pCells[index]);
  				}
  			}
  		}

  		if (currentLevel.size() == 0 && nextLevel.size() > 0)
  		{
  			tempLevel = currentLevel;
  			currentLevel = nextLevel;
  			nextLevel = tempLevel;
  			nMaxLevels--;
  		}
  	}

  	return counter;

  }

  void GridMap::SaveMap(const string& mapFilePath, const string& mapName)
  {
	  ofstream f(mapFilePath.c_str(),ios::out);
	  if(!f.is_open())
	  {
		  printf("\n Can't Open Map File to Save!, %s", mapFilePath.c_str());
		  return;
	  }
	  f.precision(8);

	  if(nCells>0)
	  {
		  int loop_size =  nCells;
		  int index = 0;
			while(index != loop_size)
			{
				if(pCells[index].nStaticPoints > 0 )
				{
					int subindex = 0;
					int sub_loop_size = pCells[index].nCells;
					while(subindex != sub_loop_size)
					{
						if(pCells[index].pInnerMap[subindex].nStaticPoints > 0)
						{
							for(unsigned int p=0; p<pCells[index].pInnerMap[subindex].innerStaticPointsList.size(); p++)
							{
								f<<pCells[index].pInnerMap[subindex].innerStaticPointsList[p].x<<","<<pCells[index].pInnerMap[subindex].innerStaticPointsList[p].y<<" ";
							}
							f<<endl;
						}

						subindex++;
					}
				}

			index++;
			}
	  }

	  f.close();

	  //save Values Map
//	  string cost_file = mapFilePath + "_cost.grd";
//	  ofstream f2(cost_file.c_str(),ios::out);
//  	  f2.precision(8);
//  	if(nCells>0)
//	  {
//		  int loop_size =  nCells;
//		  int index = 0;
//			while(index != loop_size)
//			{
//				if(pCells[index].nCells>0)
//				{
//					if(pCells[index].localize_val>0)
//					{
//						f2<<"C,"<<pCells[index].center.p.x<<","<<pCells[index].center.p.y<<","<<pCells[index].localize_val << " ";
//						f2<<endl;
//					}
//
//					int subIndex = 0;
//					while(subIndex != pCells[index].nCells)
//					{
//						f2<<"S,"<<pCells[index].pInnerMap[subIndex].center.p.x<<","<<pCells[index].pInnerMap[subIndex].center.p.y<<","<<pCells[index].pInnerMap[subIndex].localize_val << " ";
//						subIndex++;
//					}
//					f2<<endl;
//				}
//
//				index++;
//			}
//	  }
//
//  	f2.close();

  }

  void GridMap::LoadMap(const string& mapFilePath, const POINT2D& pos, const double& loadingRadius, const GPSPoint& mapTransformation)
  {

//	  GPSPoint point;
//	  ifstream f(mapFilePath.c_str(), ios::in);
//	  if(!f.is_open())
//	  {
//		  printf("\n Can't Open Map File !, %s", mapFilePath.c_str());
//		  return;
//	  }
//
//	  f.precision(8);
//	  string token, temp, innerToken;
//	  string strLine;
//
//	while(!f.eof())
//	{
//		getline(f, strLine);
//		istringstream str_stream(strLine);
//		while(getline(str_stream, innerToken, ' '))
//		{
//
//			string str_x, str_y;
//
//			istringstream ss(innerToken);
//
//			getline(ss, str_x, ',');
//			getline(ss, str_y, ',');
//
//			point.p.x = atof(str_x.c_str());
//			point.p.y = atof(str_y.c_str());
//
//			MathUtil::CoordinateTransform(point, mapTransformation);
//
//			UpdateMapObstaclePoint(point);
//		}
//	}
//
//	  f.close();

//	  string cost_file = mapFilePath + "_cost.grd";
//	  ifstream f2(cost_file.c_str(),ios::in);
//	  f2.precision(8);
//	  double cost_val = 0;
//	  while(!f2.eof())
//	  	{
//	  		getline(f2, strLine);
//	  		istringstream str_stream(strLine);
//
//	  		while(getline(str_stream, innerToken, ' '))
//			{
//				string str_key, str_x, str_y, str_val;
//				istringstream ss(innerToken);
//				getline(ss, str_key, ',');
//				getline(ss, str_x, ',');
//				getline(ss, str_y, ',');
//				getline(ss, str_val, ',');
//
//				point.x = atof(str_x.c_str());
//				point.y = atof(str_y.c_str());
//				cost_val = atof(str_val.c_str());
//
//				if(str_key.compare("S")==0)
//					UpdateSubMapCostValue(point, cost_val, 0);
//				else
//					UpdateMapCostValue(point, cost_val, 0);
//			}
//
//	  	}
//
//	  f2.close();
  }

  CELL_Info::CELL_Info()
    {
		index = 0;
		r=0;
		c=0;

		nCells = 0;
		pInnerMap = 0;
		heuristicValue = 0;
		forwardHeuristic = 0;
		backwardHeuristic = 0;
		heuristic = 0;
		expanded = -1;
		value = 0;
		action = -1;
		forward_heuristicValue = 0;
		backward_heuristicValue = 0;
		bDir = STANDSTILL_DIR;
		closed = false;
		nMovingPoints = 0;
		nStaticPoints = 0;
		localize_val = 0;
		localize_prob = 0;
    }

  CELL_Info::~CELL_Info()
     {
  //     if(pInnerCells)
  //       delete [] pInnerCells;
  	if(pInnerMap)
  		delete [] pInnerMap;
     }

  void CELL_Info::ClearSubCells(bool bMovingOnly)
  {
  	for(int i=0; i<nCells; i++)
  	{
  		if(!bMovingOnly)
  		{
  			pInnerMap[i].nStaticPoints = 0;
  			pInnerMap[i].innerStaticPointsList.clear();
  		}
  		pInnerMap[i].nMovingPoints = 0;
  		pInnerMap[i].innerMovingPointsList.clear();
  	}
  }

  void CELL_Info::Clear(int bMovingOnly)
  {
  	//forwardCenter = center;
  	//backwardCenter = center;

  	heuristicValue = 0;
  	heuristic = 0;
  	forwardHeuristic  = 0;
  	backwardHeuristic  = 0;
  	forward_heuristicValue = 0;
  	backward_heuristicValue = 0;
  	expanded = -1;
  	value = 0;
  	closed = false;
  	action = -1;
  	bDir = STANDSTILL_DIR;
  	if(bMovingOnly == 1)
  	{
  		if(nMovingPoints>0)
  		{
  			nMovingPoints = 0;
  			ClearSubCells(true);
  		}
  	}
  	else if(bMovingOnly == 0)
  	{
  		if(nMovingPoints>0 || nStaticPoints>0)
  		{
  			nMovingPoints = 0;
  			nStaticPoints = 0;
  			ClearSubCells(false);
  		}
  	}

  }

  void CELL_Info::InitSubCells(double cell_l, double sub_cell_l)
  {
  	nCells = SUBCELL_L*SUBCELL_L;
  	pInnerMap =  new CELL_Info[nCells];
  	 POINT2D p;
  	 int index = 0;

  	 for(int rr=0; rr<SUBCELL_L; rr++)
  	 {
  		 for(int cc=0; cc<SUBCELL_L; cc++)
  		   {
  			 index = get2dIndex(rr,cc,SUBCELL_L);
  			 p.x = this->bottom_left.x +  ((double)cc * sub_cell_l );
  			 p.y = this->bottom_left.y + ((double)rr * sub_cell_l );
  			 pInnerMap[index].Initialize(p, sub_cell_l, rr, cc,true);
  			 pInnerMap[index].index = index;
  		   }
  	 }
  }

  void CELL_Info::UpdateCostValue(const vector<POINT2D>& ps)
  {
  	index = 0;
  	double cost = 0;
  	for(unsigned int i=0; i<ps.size() ; i++)
  	{
  		cost += sqrt(distance2points(center, ps[i]));
  	}
  	if(localize_val==0)
  		localize_val = cost;
  	else
  		localize_val = (localize_val + cost) / 2.0;
  }

  void CELL_Info::UpdateSubCellCostValue(const vector<POINT2D>& ps, const double& cell_l, const double& sub_cell_l)
  {
  	if(!pInnerMap)
  		InitSubCells(cell_l, sub_cell_l);

  	index = 0;
  	double cost = 0;
  	while(index < nCells)
  	{
  		//if(pInnerMap[index].localize_val == 0)
  		{
  			cost = 0;
  			for(unsigned int i=0; i<ps.size() ; i++)
  			{
  				//cost += abs(ps[i].x - pInnerMap[index].center.p.x) + abs(ps[i].y - pInnerMap[index].center.p.y);//sqrt(MathUtil::Distance(pInnerMap[index].center.p, ps[i]));
  				cost += sqrt(distance2points(pInnerMap[index].center, ps[i]));
  			}
  			if(pInnerMap[index].localize_val == 0)
  				pInnerMap[index].localize_val = cost;
  			else
  				pInnerMap[index].localize_val = (pInnerMap[index].localize_val+cost)/2.0;
  		}
  		 index++;
  	 }

  }

  void CELL_Info::Initialize(POINT2D bottom_l, double cell_l, int row, int col, bool bDefaultEmpty)
    {
		double half = cell_l / 2.0;
		center.x = bottom_l.x + half;
		center.y = bottom_l.y + half;
		bottom_left = bottom_l;
		top_right.x = bottom_left.x + cell_l;
		top_right.y = bottom_left.y + cell_l;
		bottom_right.x = top_right.x;
		bottom_right.y = bottom_left.y;
		top_left.x = bottom_left.x;
		top_left.y = top_right.y;
		nMovingPoints = !bDefaultEmpty;
		nStaticPoints = !bDefaultEmpty;
		r = row;
		c = col;
    }

bool CELL_Info::operator==(const CELL_Info& cell)
  {
	if((this->r == cell.r && this->c == cell.c) || this->index == cell.index)
	  return true;
	else
	  return false;
  }

bool CELL_Info::operator!=(const CELL_Info& cell)
  {
	if((this->r != cell.r || this->c != cell.c) || this->index != cell.index)
	  return true;
	else
	  return false;
  }


    /*
     * Check for point to lie in the cell and cell this is an obstacle
     */
   inline bool CELL_Info::HitTest(const POINT2D& p)
    {

  	 bool bHit = PointInRect(p);

  	     if(pInnerMap && bHit)
  	       {
  	         for(int i=0; i<nCells; i++)
  	           {
  	             if(pInnerMap[i].PointInRect(p) == true) return true;
  	           }
  	       }

  	     return bHit;
    }

   bool CELL_Info::TestWithRectangle(RECTANGLE& rec)
   {
  	 if(!rec.PointInRect(bottom_left))
  		 return true;
  	 if(!rec.PointInRect(bottom_right))
  		 return true;
  	 if(!rec.PointInRect(top_right))
  		 return true;
  	 if(!rec.PointInRect(top_left))
  		 return true;

  	 return false;
   }

   bool CELL_Info::TestWithCircle(POINT2D _center, double width)
   {
  	 if(distance2points(center, _center) <= width)
  		 return true;
  	 else
  		 return false;
   }
   void CELL_Info::SaveCell(ostream& f)
   {
  //	 f<<"#CELL_Info:"<<r<<c<<index<nPoints<<bottom_left.x<<bottom_left.y<<top_right.x<<top_right.y;
  //	 f<<endl;
  //	 if(pInnerMap)
  //	 {
  //		 f<<"#InnerMap:";
  //		 for(int i=0; i<nCells; i++)
  //		   {
  //			 pInnerMap[i].SaveCell(f);
  //		   }
  //	 }

   }

   void CELL_Info::LoadCell(ifstream& f)
   {

   }

}
