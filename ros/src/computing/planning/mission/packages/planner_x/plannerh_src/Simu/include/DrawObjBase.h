/*
 * DrawObjBase.h
 *
 *  Created on: Jun 17, 2016
 *      Author: hatem
 */

#ifndef DRAWOBJBASE_H_
#define DRAWOBJBASE_H_


namespace Graphics
{

enum SPECIAL_KEYS_TYPE { UP_KEY, DOWN_KEY, LEFT_KEY, RIGHT_KEY, LEFT_CTRL_KEY, RIGHT_CTRL_KEY, LEFT_SHIFT_KEY, RIGHT_SHIFT_KEY, SPACE_KEY, CHAR_KEY,};

class DrawObjBase
  {
  public:

    virtual void DrawSimu()=0;
    virtual void DrawInfo()=0;
    virtual void OnLeftClick(const double& x, const double& y) = 0;
    virtual void OnRightClick(const double& x, const double& y) = 0;
    virtual void OnKeyboardPress(const SPECIAL_KEYS_TYPE& sKey, const unsigned char& key) = 0;
    virtual void LoadMaterials() = 0;
    virtual void Reset() = 0;

    DrawObjBase();
    virtual ~DrawObjBase();

    double m_followX;
    double m_followY;
    double m_followZ;
    double m_followA;
  };

} /* namespace Graphics */

#endif /* DRAWOBJBASE_H_ */
