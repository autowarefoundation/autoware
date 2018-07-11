#ifndef RECTCLASSSCORE_H_
#define RECTCLASSSCORE_H_

#include <sstream>
#include <string>

namespace Yolo2
{
	enum YoloDetectorClasses//using coco for default cfg and weights
	{
		PERSON, BICYCLE, CAR, MOTORBIKE, AEROPLANE, BUS, TRAIN, TRUCK, BOAT, TRAFFIC_LIGHT,
		FIRE_HYDRANT, STOP_SIGN, PARKING_METER, BENCH, BIRD, CAT, DOG, HORSE, SHEEP, COW,
		ELEPHANT, BEAR, ZEBRA, GIRAFFE, BACKPACK, UMBRELLA, HANDBAG, TIE, SUITCASE, FRISBEE,
		SKIS, SNOWBOARD, SPORTS_BALL, KITE, BASEBALL_BAT, BASEBALL_GLOVE, SKATEBOARD, SURFBOARD, TENNIS_RACKET, BOTTLE,
		WINE_GLASS, CUP, FORK, KNIFE, SPOON, BOWL, BANANA, APPLE, SANDWICH, ORANGE,
		BROCCOLI, CARROT, HOT_DOG, PIZZA, DONUT, CAKE, CHAIR, SOFA, POTTEDPLANT, BED,
		DININGTABLE, TOILET, TVMONITOR, LAPTOP, MOUSE, REMOTE, KEYBOARD, CELL_PHONE, MICROWAVE, OVEN,
		TOASTER, SINK, REFRIGERATOR, BOOK, CLOCK, VASE, SCISSORS, TEDDY_BEAR, HAIR_DRIER, TOOTHBRUSH,
	};
}

template<typename _Tp> class RectClassScore
{
public:
	_Tp x, y, w, h;
	_Tp score;
	unsigned int class_type;
	bool enabled;

	inline std::string toString()
	{
		std::ostringstream out;
		out << "P(" << GetClassString() << ") at " << "(x:" << x << ", y:" << y << ", w:" << w << ", h:" << h << ") =" << score;
		return out.str();
	}
	inline std::string GetClassString()
	{
		switch (class_type)
		{
			case Yolo3::PERSON: return "person";
            case Yolo2::BICYCLE: return "bicycle";
            case Yolo2::CAR: return "car";
            case Yolo2::MOTORBIKE: return "motorbike";
            case Yolo2::AEROPLANE: return "aeroplane";
            case Yolo2::BUS: return "bus";
            case Yolo2::TRAIN: return "train";
            case Yolo2::TRUCK: return "truck";
            case Yolo2::BOAT: return "boat";
            case Yolo2::TRAFFIC_LIGHT: return "traffic_lost";
            case Yolo2::FIRE_HYDRANT: return "fire_hydrant";
            case Yolo2::STOP_SIGN: return "stop_sign";
            case Yolo2::PARKING_METER: return "parking_meter";
            case Yolo2::BENCH: return "bench";
            case Yolo2::BIRD: return "bird";
            case Yolo2::CAT: return "cat";
            case Yolo2::DOG: return "dog";
            case Yolo2::HORSE: return "horse";
            case Yolo2::SHEEP: return "sheep";
            case Yolo2::COW: return "cow";
            case Yolo2::ELEPHANT: return "elephant";
            case Yolo2::BEAR: return "bear";
            case Yolo2::ZEBRA: return "zebra";
            case Yolo2::GIRAFFE: return "giraffe";
            case Yolo2::BACKPACK: return "backpack";
            case Yolo2::UMBRELLA: return "umbrella";
            case Yolo2::HANDBAG: return "handbag";
            case Yolo2::TIE: return "tie";
            case Yolo2::SUITCASE: return "suitcase";
            case Yolo2::FRISBEE: return "frisbee";
            case Yolo2::SKIS: return "skis";
            case Yolo2::SNOWBOARD: return "snowboard";
            case Yolo2::SPORTS_BALL: return "sports_ball";
            case Yolo2::KITE: return "kite";
            case Yolo2::BASEBALL_BAT: return "baseball_bat";
            case Yolo2::BASEBALL_GLOVE: return "baseball_glove";
            case Yolo2::SKATEBOARD: return "skateboard";
            case Yolo2::SURFBOARD: return "surfboard";
            case Yolo2::TENNIS_RACKET: return "tennis_racket";
            case Yolo2::BOTTLE: return "bottle";
            case Yolo2::WINE_GLASS: return "wine_glass";
            case Yolo2::CUP: return "cup";
            case Yolo2::FORK: return "fork";
            case Yolo2::KNIFE: return "knife";
            case Yolo2::SPOON: return "spoon";
            case Yolo2::BOWL: return "bowl";
            case Yolo2::BANANA: return "banana";
            case Yolo2::APPLE: return "apple";
            case Yolo2::SANDWICH: return "sandwich";
            case Yolo2::ORANGE: return "orange";
            case Yolo2::BROCCOLI: return "broccoli";
            case Yolo2::CARROT: return "carrot";
            case Yolo2::HOT_DOG: return "hot_dog";
            case Yolo2::PIZZA: return "pizza";
            case Yolo2::DONUT: return "donut";
            case Yolo2::CAKE: return "cake";
            case Yolo2::CHAIR: return "chair";
            case Yolo2::SOFA: return "sofa";
            case Yolo2::POTTEDPLANT: return "potted_plant";
            case Yolo2::BED: return "bed";
            case Yolo2::DININGTABLE: return "dining_table";
            case Yolo2::TOILET: return "toilet";
            case Yolo2::TVMONITOR: return "tv";
            case Yolo2::LAPTOP: return "laptop";
            case Yolo2::MOUSE: return "mouse";
            case Yolo2::REMOTE: return "remote";
            case Yolo2::KEYBOARD: return "keyboard";
            case Yolo2::CELL_PHONE: return "cellphone";
            case Yolo2::MICROWAVE: return "microwave";
            case Yolo2::OVEN: return "over";
            case Yolo2::TOASTER: return "toaster";
            case Yolo2::SINK: return "sink";
            case Yolo2::REFRIGERATOR: return "refrigerator";
            case Yolo2::BOOK: return "book";
            case Yolo2::CLOCK: return "clock";
            case Yolo2::VASE: return "vase";
            case Yolo2::SCISSORS: return "scissors";
            case Yolo2::TEDDY_BEAR: return "teddy_bear";
            case Yolo2::HAIR_DRIER: return "hair_drier";
            case Yolo2::TOOTHBRUSH: return "toothbrush";
			default:return "error";
		}
	}
};


#endif /* RECTCLASSSCORE_H_ */
