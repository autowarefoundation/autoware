#ifndef _Scan_to_image_
#define _Scan_to_image_

#define IMAGE_WIDTH 640
#define IMAGE_HEIGHT 480
#define NO_DATA 0

struct Three_dimensional_vector{
    std::vector<float> x;
    std::vector<float> y;
    std::vector<float> z;
};

struct Two_dimensional_vector{
    std::vector<float> x;
    std::vector<float> y;
};

struct Scan_points_dataset{
    Three_dimensional_vector scan_points;
    std::vector<float> intensity;
};

struct Image_points_dataset{
    Two_dimensional_vector image_points;
    std::vector<float> distance;
    std::vector<float> intensity;
};

struct Scan_image{
    float distance[IMAGE_WIDTH][IMAGE_HEIGHT];
    float intensity[IMAGE_WIDTH][IMAGE_HEIGHT];
    int max_y;
    int min_y;
};
#endif
