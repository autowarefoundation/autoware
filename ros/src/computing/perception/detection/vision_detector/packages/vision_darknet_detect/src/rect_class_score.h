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
            case Yolo3::TRAFFIC_LIGHT: return "traffic_light";
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
            case Yolo3::OVEN: return "oven";
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
	inline int GetClassInt()
    {
        switch (class_type)
        {
            case Yolo3::PERSON: return 0;
            case Yolo3::BICYCLE: return 1;
            case Yolo3::CAR: return 2;
            case Yolo3::MOTORBIKE: return 3;
            case Yolo3::AEROPLANE: return 4;
            case Yolo3::BUS: return 5;
            case Yolo3::TRAIN: return 6;
            case Yolo3::TRUCK: return 7;
            case Yolo3::BOAT: return 8;
            case Yolo3::TRAFFIC_LIGHT: return 9;
            case Yolo3::FIRE_HYDRANT: return 10;
            case Yolo3::STOP_SIGN: return 11;
            case Yolo3::PARKING_METER: return 12;
            case Yolo3::BENCH: return 14;
            case Yolo3::BIRD: return 15;
            case Yolo3::CAT: return 16;
            case Yolo3::DOG: return 17;
            case Yolo3::HORSE: return 18;
            case Yolo3::SHEEP: return 19;
            case Yolo3::COW: return 20;
            case Yolo3::ELEPHANT: return 21;
            case Yolo3::BEAR: return 22;
            case Yolo3::ZEBRA: return 23;
            case Yolo3::GIRAFFE: return 24;
            case Yolo3::BACKPACK: return 25;
            case Yolo3::UMBRELLA: return 26;
            case Yolo3::HANDBAG: return 27;
            case Yolo3::TIE: return 28;
            case Yolo3::SUITCASE: return 29;
            case Yolo3::FRISBEE: return 30;
            case Yolo3::SKIS: return 31;
            case Yolo3::SNOWBOARD: return 32;
            case Yolo3::SPORTS_BALL: return 33;
            case Yolo3::KITE: return 34;
            case Yolo3::BASEBALL_BAT: return 35;
            case Yolo3::BASEBALL_GLOVE: return 36;
            case Yolo3::SKATEBOARD: return 37;
            case Yolo3::SURFBOARD: return 38;
            case Yolo3::TENNIS_RACKET: return 39;
            case Yolo3::BOTTLE: return 40;
            case Yolo3::WINE_GLASS: return 41;
            case Yolo3::CUP: return 42;
            case Yolo3::FORK: return 43;
            case Yolo3::KNIFE: return 44;
            case Yolo3::SPOON: return 45;
            case Yolo3::BOWL: return 46;
            case Yolo3::BANANA: return 47;
            case Yolo3::APPLE: return 48;
            case Yolo3::SANDWICH: return 49;
            case Yolo3::ORANGE: return 50;
            case Yolo3::BROCCOLI: return 51;
            case Yolo3::CARROT: return 52;
            case Yolo3::HOT_DOG: return 53;
            case Yolo3::PIZZA: return 54;
            case Yolo3::DONUT: return 55;
            case Yolo3::CAKE: return 56;
            case Yolo3::CHAIR: return 57;
            case Yolo3::SOFA: return 58;
            case Yolo3::POTTEDPLANT: return 59;
            case Yolo3::BED: return 60;
            case Yolo3::DININGTABLE: return 61;
            case Yolo3::TOILET: return 62;
            case Yolo3::TVMONITOR: return 63;
            case Yolo3::LAPTOP: return 64;
            case Yolo3::MOUSE: return 65;
            case Yolo3::REMOTE: return 66;
            case Yolo3::KEYBOARD: return 67;
            case Yolo3::CELL_PHONE: return 68;
            case Yolo3::MICROWAVE: return 69;
            case Yolo3::OVEN: return 70;
            case Yolo3::TOASTER: return 71;
            case Yolo3::SINK: return 72;
            case Yolo3::REFRIGERATOR: return 73;
            case Yolo3::BOOK: return 74;
            case Yolo3::CLOCK: return 75;
            case Yolo3::VASE: return 76;
            case Yolo3::SCISSORS: return 77;
            case Yolo3::TEDDY_BEAR: return 78;
            case Yolo3::HAIR_DRIER: return 79;
            case Yolo3::TOOTHBRUSH: return 13;
            default:return 0;
        }
	}
};

#endif /* RECTCLASSSCORE_H_ */
