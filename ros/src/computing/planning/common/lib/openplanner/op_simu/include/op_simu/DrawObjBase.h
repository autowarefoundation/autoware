/*
 * DrawObjBase.h
 *
 *  Created on: Jun 17, 2016
 *      Author: hatem
 */

#ifndef DRAWOBJBASE_H_
#define DRAWOBJBASE_H_

#include <string>
namespace Graphics
{

enum DISPLAY_SHAP {DISP_WHEEL, DISP_PEDAL, DISP_TEXT};
enum SPECIAL_KEYS_TYPE { UP_KEY, DOWN_KEY, LEFT_KEY, RIGHT_KEY, LEFT_CTRL_KEY, RIGHT_CTRL_KEY, LEFT_SHIFT_KEY, RIGHT_SHIFT_KEY, SPACE_KEY, CHAR_KEY,};

class DisplayDataObj
{
public:
	double x;
	double y;
	double value;
	std::string text;
	DISPLAY_SHAP shape;

	DisplayDataObj()
	{
		x = y = value = 0;
		shape = DISP_TEXT;
	}
};

class DrawObjBase
  {
  public:

    virtual void DrawSimu()=0;
    virtual void DrawInfo(const int& centerX, const int& centerY, const int& maxX, const int& maxY)=0;
    virtual void OnLeftClick(const double& x, const double& y) = 0;
    virtual void OnRightClick(const double& x, const double& y) = 0;
    virtual void OnKeyboardPress(const SPECIAL_KEYS_TYPE& sKey, const unsigned char& key) = 0;
    virtual void LoadMaterials() = 0;
    virtual void Reset() = 0;
    virtual bool IsInitState() = 0;
    virtual void UpdatePlaneStartGoal(const double& x1,const double& y1, const double& a1, const double& x2,const double& y2, const double& a2) = 0;
    virtual void AddSimulatedCarPos(const double& x,const double& y, const double& a) = 0;

    DrawObjBase();
    virtual ~DrawObjBase();

    double m_followX;
    double m_followY;
    double m_followZ;
    double m_followA;
  };

} /* namespace Graphics */

#endif /* DRAWOBJBASE_H_ */
