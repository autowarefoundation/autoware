#ifndef RECTCLASSSCORE_H_
#define RECTCLASSSCORE_H_

#include <sstream>
#include <string>

template<typename _Tp>
class RectClassScore
{
public:
  _Tp x, y, w, h;
  _Tp score;
  unsigned int class_type;
  bool enabled;

  inline std::string toString()
  {
    std::ostringstream out;
    out << "P(" << GetClassString() << ") at " << "(x:" << x << ", y:" << y << ", w:" << w << ", h:" << h << ") ="
        << score;
    return out.str();
  }

  inline std::string GetClassString()
  {
    switch (class_type)
    {
      case 0:
        return "nothing";
      case 1:
        return "plane";
      case 2:
        return "bicycle";
      case 3:
        return "bird";
      case 4:
        return "boat";
      case 5:
        return "bottle";
      case 6:
        return "bus";
      case 7:
        return "car";
      case 8:
        return "cat";
      case 9:
        return "chair";
      case 10:
        return "cow";
      case 11:
        return "table";
      case 12:
        return "dog";
      case 13:
        return "horse";
      case 14:
        return "motorbike";
      case 15:
        return "person";
      case 16:
        return "plant";
      case 17:
        return "sheep";
      case 18:
        return "sofa";
      case 19:
        return "train";
      case 20:
        return "tv";
      default:
        return "error";
    }
  }
};


#endif /* RECTCLASSSCORE_H_ */
