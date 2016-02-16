#ifndef SEARCH_DISTANCE_H_
#define SEARCH_DISTANCE_H_

#include <vector>

/* Search the minimum value of candidates */
extern float getShortest(const std::vector<float>& candidates);

/* Search the median value of candidates */
extern float getMedian(const std::vector<float>& candidates);

/* Search the mode (most common) value of candidates */
extern float getMode(const std::vector<float>& candidates);

#endif  // SEARCH_DISTANCE_H_
