#ifndef RECTCLASSSCORE_H_
#define RECTCLASSSCORE_H_

#include <sstream>
#include <string>

template<typename _Tp> class RectClassScore
{
public:
	_Tp x, y, w, h;
	_Tp score;
	unsigned int class_type;
	bool enabled;

	std::string toString()
	{
		std::ostringstream out;
		out << "P(" << GetClassString() << ") at " << "(x:" << x << ", y:" << y << ", w:" << w << ", h:" << h << ") =" << score;
		return out.str();
	}
	std::string GetClassString()
		{
			switch (class_type)
			{
				case 0:
					return "nothing";
				break;
				case 1:
					return "plane";
				break;
				case 2:
					return "bicycle";
				break;
				case 3:
					return "bird";
				break;
				case 4:
					return "boat";
				break;
				case 5:
					return "bottle";
				break;
				case 6:
					return "bus";
				break;
				case 7:
					return "car";
				break;
				case 8:
					return "cat";
				break;
				case 9:
					return "chair";
				break;
				case 10:
					return "cow";
				break;
				case 11:
					return "table";
				break;
				case 12:
					return "dog";
				break;
				case 13:
					return "horse";
				break;
				case 14:
					return "motorbike";
				break;
				case 15:
					return "person";
				break;
				case 16:
					return "plant";
				break;
				case 17:
					return "sheep";
				break;
				case 18:
					return "sofa";
				break;
				case 19:
					return "train";
				break;
				case 20:
					return "tv";
				break;
				default:
					return "error";
			}
		}
};


#endif /* RECTCLASSSCORE_H_ */
