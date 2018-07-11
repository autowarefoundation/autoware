#ifndef RECTCLASSSCORE_H_
#define RECTCLASSSCORE_H_

#include <sstream>
#include <string>

namespace Yolo3
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
		out << class_type << "(x:" << x << ", y:" << y << ", w:" << w << ", h:" << h << ") =" << score;
		return out.str();
	}
	inline std::string GetClassString()
	{
		switch (class_type)
		{
            case Yolo3::PERSON: return "person";
            case Yolo3::BICYCLE: return "bicycle";
            case Yolo3::CAR: return "car";
            case Yolo3::MOTORBIKE: return "motorbike";
            case Yolo3::AEROPLANE: return "aeroplane";
            case Yolo3::BUS: return "bus";
            case Yolo3::TRAIN: return "train";
            case Yolo3::TRUCK: return "truck";
            case Yolo3::BOAT: return "boat";
            case Yolo3::TRAFFIC_LIGHT: return "traffic_lost";
            case Yolo3::FIRE_HYDRANT: return "fire_hydrant";
            case Yolo3::STOP_SIGN: return "stop_sign";
            case Yolo3::PARKING_METER: return "parking_meter";
            case Yolo3::BENCH: return "bench";
            case Yolo3::BIRD: return "bird";
            case Yolo3::CAT: return "cat";
            case Yolo3::DOG: return "dog";
            case Yolo3::HORSE: return "horse";
            case Yolo3::SHEEP: return "sheep";
            case Yolo3::COW: return "cow";
            case Yolo3::ELEPHANT: return "elephant";
            case Yolo3::BEAR: return "bear";
            case Yolo3::ZEBRA: return "zebra";
            case Yolo3::GIRAFFE: return "giraffe";
            case Yolo3::BACKPACK: return "backpack";
            case Yolo3::UMBRELLA: return "umbrella";
            case Yolo3::HANDBAG: return "handbag";
            case Yolo3::TIE: return "tie";
            case Yolo3::SUITCASE: return "suitcase";
            case Yolo3::FRISBEE: return "frisbee";
            case Yolo3::SKIS: return "skis";
            case Yolo3::SNOWBOARD: return "snowboard";
            case Yolo3::SPORTS_BALL: return "sports_ball";
            case Yolo3::KITE: return "kite";
            case Yolo3::BASEBALL_BAT: return "baseball_bat";
            case Yolo3::BASEBALL_GLOVE: return "baseball_glove";
            case Yolo3::SKATEBOARD: return "skateboard";
            case Yolo3::SURFBOARD: return "surfboard";
            case Yolo3::TENNIS_RACKET: return "tennis_racket";
            case Yolo3::BOTTLE: return "bottle";
            case Yolo3::WINE_GLASS: return "wine_glass";
            case Yolo3::CUP: return "cup";
            case Yolo3::FORK: return "fork";
            case Yolo3::KNIFE: return "knife";
            case Yolo3::SPOON: return "spoon";
            case Yolo3::BOWL: return "bowl";
            case Yolo3::BANANA: return "banana";
            case Yolo3::APPLE: return "apple";
            case Yolo3::SANDWICH: return "sandwich";
            case Yolo3::ORANGE: return "orange";
            case Yolo3::BROCCOLI: return "broccoli";
            case Yolo3::CARROT: return "carrot";
            case Yolo3::HOT_DOG: return "hot_dog";
            case Yolo3::PIZZA: return "pizza";
            case Yolo3::DONUT: return "donut";
            case Yolo3::CAKE: return "cake";
            case Yolo3::CHAIR: return "chair";
            case Yolo3::SOFA: return "sofa";
            case Yolo3::POTTEDPLANT: return "potted_plant";
            case Yolo3::BED: return "bed";
            case Yolo3::DININGTABLE: return "dining_table";
            case Yolo3::TOILET: return "toilet";
            case Yolo3::TVMONITOR: return "tv";
            case Yolo3::LAPTOP: return "laptop";
            case Yolo3::MOUSE: return "mouse";
            case Yolo3::REMOTE: return "remote";
            case Yolo3::KEYBOARD: return "keyboard";
            case Yolo3::CELL_PHONE: return "cellphone";
            case Yolo3::MICROWAVE: return "microwave";
            case Yolo3::OVEN: return "over";
            case Yolo3::TOASTER: return "toaster";
            case Yolo3::SINK: return "sink";
            case Yolo3::REFRIGERATOR: return "refrigerator";
            case Yolo3::BOOK: return "book";
            case Yolo3::CLOCK: return "clock";
            case Yolo3::VASE: return "vase";
            case Yolo3::SCISSORS: return "scissors";
            case Yolo3::TEDDY_BEAR: return "teddy_bear";
            case Yolo3::HAIR_DRIER: return "hair_drier";
            case Yolo3::TOOTHBRUSH: return "toothbrush";
			default:return "error";
		}
	}
};

#endif /* RECTCLASSSCORE_H_ */
