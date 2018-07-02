/*
 * DrawObjBase.h
 *
 *  Created on: Jun 17, 2016
 *      Author: hatem
 */

#ifndef DRAWOBJBASE_TESTING
#define DRAWOBJBASE_TESTING

#include <string>
namespace OP_TESTING_NS
{

class DrawObjBase
  {
  public:

    virtual void DrawSimu()=0;
    virtual void OnKeyboardPress(const int& sKey, const unsigned char& key) = 0;
    DrawObjBase();
    virtual ~DrawObjBase();
  };

}

#endif
