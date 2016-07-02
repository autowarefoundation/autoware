/*
 * RoadNetwork.h
 *
 *  Created on: May 19, 2016
 *      Author: hatem
 */

#ifndef ROADNETWORK_H_
#define ROADNETWORK_H_

#include <string>
#include <vector>


namespace PlannerHNS
{

class Lane;

enum CORRECTION_DIRECTION {	FORWARD, FORWARD_LEFT, FORWARD_RIGHT,
	BACKWARD, BACKWARD_LEFT, BACKWARD_RIGHT, STANDSTILL};

enum OBSTACLE_TYPE {SIDEWALK, TREE, CAR, TRUCK, HOUSE, PEDESTRIAN, CYCLIST, GENERAL_OBSTACLE};
enum DRIVABLE_TYPE {DIRT, TARMAC, PARKINGAREA, INDOOR, GENERAL_AREA};

enum STATE_TYPE {INITIAL_STATE, WAITING_STATE, FORWARD_STATE, STOPPING_STATE, EMERGENCY_STATE,
	TRAFFIC_LIGHT_STOP_STATE, STOP_SIGN_STOP_STATE, FOLLOW_STATE, LANE_CHANGE_STATE, OBSTACLE_AVOIDANCE_STATE, FINISH_STATE};
enum LIGHT_INDICATOR {INDICATOR_LEFT, INDICATOR_RIGHT, INDICATOR_BOTH , INDICATOR_NONE};

enum SHIFT_POS {SHIFT_POS_PP = 0x60, SHIFT_POS_RR = 0x40, SHIFT_POS_NN = 0x20,
	SHIFT_POS_DD = 0x10, SHIFT_POS_BB = 0xA0, SHIFT_POS_SS = 0x0f, SHIFT_POS_UU = 0xff };

class POINT2D
{
public:
    double x;
    double y;
    double z;
    POINT2D()
    {
      x=0;y=0;z=0;
    }
    POINT2D(double px, double py, double pz = 0)
    {
      x = px;
      y = py;
      z = pz;
    }
};

class RECTANGLE

{
public:
  POINT2D bottom_left;
  POINT2D top_right;
  double width;
  double length;
  bool bObstacle;
  inline bool PointInRect(POINT2D p)
  {
    return p.x >= bottom_left.x && p.x <= top_right.x && p.y >= bottom_left.y && p.y <= top_right.y;
  }

  inline bool HitTest(POINT2D p)
  {
    return PointInRect(p) && bObstacle;
  }

  RECTANGLE()
  {
	  width=0;
	  length = 0;
    bObstacle = true;
  }

  virtual ~RECTANGLE(){}
};

class MapItem
{
public:
  int id;
  POINT2D sp; //start point
  POINT2D ep; // end point
  double c; //curvature
  double w; //width
  double l; //length
  std::string fileName; //
  std::vector<POINT2D> polygon;


  MapItem(int ID, POINT2D start, POINT2D end, double curvature, double width, double length, std::string objName)
  {
    id = ID;
    sp = start;
    ep = end;
    c = curvature;
    w = width;
    l = length;
    fileName = objName;

  }

  MapItem()
  {
    id = 0; c = 0; w = 0; l = 0;
  }

  virtual ~MapItem(){}

  MapItem(const MapItem & cmi)
  {
        id = cmi.id;
        sp = cmi.sp;
        ep = cmi.ep;
        c = cmi.c;
        w = cmi.w;
        l = cmi.l;
        fileName = cmi.fileName;
  }
  MapItem &operator=(const MapItem &cmi)
  {
    this->id = cmi.id;
      this->sp = cmi.sp;
      this->ep = cmi.ep;
      this->c = cmi.c;
      this->w = cmi.w;
      this->l = cmi.l;
      this->fileName = cmi.fileName;
      return *this;
  }

  virtual int operator==(const MapItem &mi) const
    {
      return this->id == mi.id;
    }
};

class Obstacle : public MapItem
{
  public:
    OBSTACLE_TYPE t;

    Obstacle(int ID, POINT2D start, POINT2D end, double curvature, double width, double length,OBSTACLE_TYPE type, std::string fileName ) : MapItem(ID, start, end, curvature, width, length, fileName)
  {
      t = type;
  }
    virtual ~Obstacle()
    {
    }

    Obstacle() : MapItem()
       {
      t = SIDEWALK;
       }

    Obstacle(const Obstacle& ob) : MapItem(ob)
      {
        t = ob.t;
      }

    Obstacle& operator=(const Obstacle& ob)
      {
      this->id = ob.id;
      this->sp = ob.sp;
      this->ep = ob.ep;
      this->c = ob.c;
      this->w = ob.w;
      this->l = ob.l;
      this->t = ob.t;
      this->fileName = ob.fileName;
      return *this;
      }

      virtual int operator==(const Obstacle &ob) const
          {
            return this->id == ob.id && this->t == ob.t;
          }
};

class DrivableArea : public MapItem
{
public:
  DRIVABLE_TYPE t; // drivable area type

  DrivableArea(int ID, POINT2D start, POINT2D end, double curvature, double width, double length,DRIVABLE_TYPE type, std::string fileName ) : MapItem( ID, start, end, curvature, width, length, fileName)
  {
    t = type;
  }

  virtual ~DrivableArea()
  {

  }

  DrivableArea() : MapItem()
    {
      t = PARKINGAREA;
    }

  DrivableArea(const DrivableArea& da) : MapItem(da)
  {
    t = da.t;
  }

  DrivableArea& operator=(const DrivableArea& da)
  {
    this->id = da.id;
    this->sp = da.sp;
    this->ep = da.ep;
    this->c = da.c;
    this->w = da.w;
    this->l = da.l;
    this->t = da.t;
    this->fileName = da.fileName;
    return *this;
  }

  virtual int operator==(const DrivableArea &da) const
      {
        return this->id == da.id && this->t == da.t;
      }

};

class GPSPoint
{
public:
	double lat, x;
	double lon, y;
	double alt, z;
	double dir, a;

	GPSPoint()
	{
		lat = x = 0;
		lon = y = 0;
		alt = z = 0;
		dir = a = 0;
	}

	GPSPoint(const double& x, const double& y, const double& z, const double& a)
	{
		this->x = this->lat = x;
		this->y = this->lon = y;
		this->z = this->alt = z;
		this->a = this->dir = a;
	}
};

class Rotation
{
public:
	double x;
	double y;
	double z;
	double w;

	Rotation()
	{
		x = 0;
		y = 0;
		z = 0;
		w = 0;
	}
};

class WayPoint
{
public:
	GPSPoint pos;
	Rotation rot;
	double   v;
	double   cost;
	int 	 laneId;
	int 	 id;
	CORRECTION_DIRECTION bDir;

	Lane* pLane;
	WayPoint* pLeft;
	WayPoint* pRight;
	WayPoint* pFront;
	WayPoint* pBack;

	WayPoint()
	{
		id = 0;
		v = 0;
		cost = 0;
		laneId = -1;
		pLane  = 0;
		pLeft = 0;
		pRight = 0;
		pFront = 0;
		pBack = 0;
		bDir = FORWARD;
	}

	WayPoint(const double& x, const double& y, const double& z, const double& a)
	{
		pos.x = x;
		pos.y = y;
		pos.z = z;
		pos.a = a;

		id = 0;
		v = 0;
		cost = 0;
		laneId = -1;
		pLane  = 0;
		pLeft = 0;
		pRight = 0;
		pFront = 0;
		pBack = 0;
		bDir = FORWARD;
	}
};

class StopLine
{
public:
	int id;
	int laneId;
	int roadId;
	std::vector<GPSPoint> points;
	Lane* pLane;

	StopLine()
	{
		id    = 0;
		laneId =0;
		roadId =0;
		pLane = 0;
	}
};

class WaitingLine
{
public:
	int id;
	int laneId;
	int roadId;
	std::vector<GPSPoint> points;
	Lane* pLane;

	WaitingLine()
	{
		id    = 0;
		laneId =0;
		roadId =0;
		pLane = 0;
	}
};

enum TrafficSignTypes {UNKNOWN_SIGN, STOP_SIGN, MAX_SPEED_SIGN, MIN_SPEED_SIGN};

class TrafficSign
{
public:
	int id;
	int laneId;
	int roadId;

	GPSPoint pos;
	TrafficSignTypes signType;
	double value;
	double fromValue;
	double toValue;
	std::string strValue;
	timespec timeValue;
	timespec fromTimeValue;
	timespec toTimeValue;

	Lane* pLane;

	TrafficSign()
	{
		id    		= 0;
		laneId 		= 0;
		roadId		= 0;
		signType  	= UNKNOWN_SIGN;
		value		= 0;
		fromValue	= 0;
		toValue		= 0;
//		timeValue	= 0;
//		fromTimeValue = 0;
//		toTimeValue	= 0;
		pLane 		= 0;
	}
};

enum TrafficLightState {UNKNOWN_LIGHT, RED_LIGHT, GREEN_LIGHT, YELLOW_LIGHT, LEFT_GREEN, FORWARD_GREEN, RIGHT_GREEN, FLASH_YELLOW, FLAH_RED};

class TrafficLight
{
public:
	int id;
	GPSPoint pos;
	TrafficLightState lightState;

	Lane* pLane;

	TrafficLight()
	{
		id 			= 0;
		lightState	= GREEN_LIGHT;
		pLane 		= 0;
	}
};

enum RoadSegmentType {NORMAL_ROAD, INTERSECTION_ROAD, UTURN_ROAD, EXIT_ROAD, MERGE_ROAD, HIGHWAY_ROAD};

class RoadSegment
{
public:
	int id;
	RoadSegmentType roadType;
	std::vector<int> fromIds;
	std::vector<int> toIds;
	std::vector<Lane> Lanes;


	std::vector<RoadSegment*> fromLanes;
	std::vector<RoadSegment*> toLanes;

	RoadSegment()
	{
		id = 0;
		roadType = NORMAL_ROAD;
	}


};

enum LaneType{NORMAL_LANE, MERGE_LANE, EXIT_LANE, BUS_LANE, BUS_STOP_LANE, EMERGENCY_LANE};

class Lane
{
public:
	int id;
	int roadId;
	std::vector<int> fromIds;
	std::vector<int> toIds;
	int num; //lane number in the road segment from left to right
	double speed;
	double length;
	LaneType type;
	std::vector<TrafficSign> signs;
	std::vector<WayPoint> points;
	TrafficLight trafficlight;
	StopLine stopLine;
	WaitingLine waitingLine;

	std::vector<Lane*> fromLanes;
	std::vector<Lane*> toLanes;
	Lane* pLeftLane;
	Lane* pRightLane;
	RoadSegment * pRoad;

	Lane()
	{
		id 		= 0;
		num		= 0;
		speed 	= 0;
		length 	= 0;
		type 	= NORMAL_LANE;
		pLeftLane = 0;
		pRightLane = 0;
		pRoad	= 0;
		roadId = 0;
	}

};

class RoadNetwork
{
public:
	std::vector<RoadSegment> roadSegments;
};

class VehicleState
{
public:
	double speed;
	double steer;
	SHIFT_POS shift;

	VehicleState()
	{
		speed = 0;
		steer = 0;
		shift = SHIFT_POS_NN;
	}

};

class BehaviorState
{
public:
	STATE_TYPE state;
	double maxVelocity;
	double minVelocity;
	double stopDistance;
	double followVelocity;
	double followDistance;
	LIGHT_INDICATOR indicator;


	BehaviorState()
	{
		state = INITIAL_STATE;
		maxVelocity = 0;
		minVelocity = 0;
		stopDistance = 0;
		followVelocity = 0;
		followDistance = 0;
		indicator  = INDICATOR_NONE;

	}

};

}


#endif /* ROADNETWORK_H_ */

