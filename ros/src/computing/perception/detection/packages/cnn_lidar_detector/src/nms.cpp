#include "nms.hpp"

std::vector< std::vector<float> > Nms(const std::vector<std::vector<float> > &in_boxes,
                               const float &in_threshold)
{
	if (in_boxes.empty())
		{return std::vector< std::vector<float> >();}

	// grab the coordinates of the bounding in_boxes CCW
	std::vector<float> x1 = get_point_from_vector(in_boxes, X_FrontLeft);
	std::vector<float> y1 = get_point_from_vector(in_boxes, Y_FrontLeft);

	std::vector<float> x2 = get_point_from_vector(in_boxes, X_BackLeft);
	std::vector<float> y2 = get_point_from_vector(in_boxes, Y_BackLeft);

	std::vector<float> x3 = get_point_from_vector(in_boxes, X_BackRight);
	std::vector<float> y3 = get_point_from_vector(in_boxes, Y_BackRight);

	std::vector<float> x4 = get_point_from_vector(in_boxes, X_FrontRight);
	std::vector<float> y4 = get_point_from_vector(in_boxes, Y_FrontRight);

	// compute the area of the bounding in_boxes and sort the bounding
	std::vector<float> ww = euclidean_distance(x1, x2);
	std::vector<float> hh = euclidean_distance(x2, x3);
	std::vector<float> area = multiply_element_wise(ww,hh);//get_polygon_area(x1, y1, x2, y2, x3, y3, x4, y4);
	//sort the bounding in_boxes by the bottom-right y-coordinate of the bounding box
	//first get the largest Y point
	std::vector<float> max_y = get_largest_y(y1, y2, y3, y4);
	//std::vector<size_t> idxs = argsort(y2);
	std::vector<size_t> indices = argsort(max_y);

	size_t last_index;
	size_t current_index;
	std::vector<size_t> selected_indices;

	// keep looping while some indexes still remain in the indexes list
	while (indices.size() > 0)
	{
		// grab the last index in the indexes list and add the
		// index value to the list of picked indexes
		last_index = indices.size() - 1;
		current_index    = indices[last_index];
		selected_indices.push_back(current_index);

		// find the largest (x, y) coordinates for the start of
		// the bounding box and the smallest (x, y) coordinates
		// for the end of the bounding box
		auto indices_except_last = remove_last_element(indices);

		auto xx1 = set_elements_to_min_value(x1[current_index], copy_by_indices(x1, indices_except_last));
		auto yy1 = set_elements_to_min_value(y1[current_index], copy_by_indices(y1, indices_except_last));
		auto xx3 = set_elements_to_max_value(x3[current_index], copy_by_indices(x3, indices_except_last));
		auto yy3 = set_elements_to_max_value(y3[current_index], copy_by_indices(y3, indices_except_last));

		// compute the width and height of the bounding box
		auto w = euclidean_distance(xx1, xx3);
		auto h = euclidean_distance(yy1, yy3);

		// compute the ratio of overlap
		auto overlap = divide_element_wise(multiply_element_wise(w, h), copy_by_indices(area, indices_except_last));

		// delete all indexes from the index list that have
		auto delete_indices = keep_by_threshold(overlap, in_threshold);
		delete_indices.push_back(last_index);
		indices = remove_elements_by_index(indices, delete_indices);
	}

	return keep_only_indices(in_boxes, selected_indices);
}

std::vector<float> euclidean_distance(const std::vector<float> &a,
                                      const std::vector<float> &b)
{
	std::vector<float> distances;
	distances.resize(a.size());
	for(size_t i=0 ; i < a.size(); i++)
	{
		distances[i] = sqrt( (a[i]-b[i])*(a[i]-b[i]) );
	}
	return distances;
}

std::vector<float> get_largest_y(const std::vector<float> &y1,
                                 const std::vector<float> &y2,
                                 const std::vector<float> &y3,
                                 const std::vector<float> &y4)
{
	std::vector<float> largest_y;
	largest_y.resize(y1.size());
	for(size_t i=0; i<y1.size(); i++)
	{
		std::vector<float> tmp = {y1[i], y2[i], y3[i], y4[i]};
		largest_y[i] = *std::max_element(tmp.begin(), tmp.end());
	}
	return largest_y;
}

std::vector<float> get_point_from_vector(const std::vector< std::vector<float> > &rect,
                                         const PointInRectangle &pos)
{
	std::vector<float> points;

	for (const auto & p: rect)
	points.push_back(p[pos]);

	return points;
}

std::vector<float> get_polygon_area(const std::vector<float> &x1,
                                    const std::vector<float> &y1,
                                    const std::vector<float> &x2,
                                    const std::vector<float> &y2,
                                    const std::vector<float> &x3,
                                    const std::vector<float> &y3,
                                    const std::vector<float> &x4,
                                    const std::vector<float> &y4)
{
	std::vector<float> areas;

	for (size_t i = 0; i < x1.size(); i++)
	{
		boost_polygon polygon;
		std::vector< boost_point_xy > polygon_points = {boost_point_xy(x1[i],y1[i]),
		                                                boost_point_xy(x2[i],y2[i]),
		                                                boost_point_xy(x3[i],y3[i]),
		                                                boost_point_xy(x4[i],y4[i])};

		boost::geometry::assign_points(polygon, polygon_points);
		areas.push_back(boost::geometry::area(polygon));
	}

	return areas;
}


/*!
 * Returns the indices that would sort @param in_vector
 * @tparam T Type of the vector
 * @param in_vector Vector to be ordered
 * @return The ordered indices
 */
std::vector<size_t> argsort(std::vector<float> & in_vector)
{
	// initialize original index locations
	std::vector<size_t> idx(in_vector.size());
	std::iota(idx.begin(), idx.end(), 0);

	// sort indexes based on comparing values in v
	std::sort(idx.begin(), idx.end(),
	          [&](size_t i1, size_t i2) {return in_vector[i1] < in_vector[i2];});


	return idx;
}

std::vector<float> set_elements_to_min_value(const float &in_min_value,
                                             const std::vector<float> &in_vector)
{
	auto maxVec = in_vector;
	auto len = in_vector.size();

	for (decltype(len) idx = 0; idx < len; ++idx)
		if (in_vector[idx] < in_min_value)
			maxVec[idx] = in_min_value;

	return maxVec;
}

std::vector<float> set_elements_to_max_value(const float &in_max_value,
                                             const std::vector<float> &in_vector)
{
	auto minVec = in_vector;
	auto len = in_vector.size();

	for (decltype(len) idx = 0; idx < len; ++idx)
		if (in_vector[idx] > in_max_value)
			minVec[idx] = in_max_value;

	return minVec;
}

std::vector<float> copy_by_indices(const std::vector<float> &in_vector,
                                   const std::vector<size_t> &in_indices)
{
	std::vector<float> resultVec;

	for (const auto & idx : in_indices)
		{resultVec.push_back(in_vector[idx]);}

	return resultVec;
}

std::vector<size_t> remove_last_element(const std::vector<size_t> &in_vector)
{
	auto resultVec = in_vector;
	resultVec.erase(resultVec.end()-1);
	return resultVec;
}

std::vector<float> subtract_element_wise(const std::vector<float> &vec1,
                                         const std::vector<float> &vec2)
{
	std::vector<float> result;
	auto len = vec1.size();

	for (decltype(len) idx = 0; idx < len; ++idx)
		{result.push_back(vec1[idx] - vec2[idx] + 1);}

	return result;
}

std::vector<float> multiply_element_wise(const std::vector<float> &vec1,
                                         const std::vector<float> &vec2)
{
	std::vector<float> resultVec;
	auto len = vec1.size();

	for (decltype(len) idx = 0; idx < len; ++idx)
		{resultVec.push_back(vec1[idx] * vec2[idx]);}

	return resultVec;
}

std::vector<float> divide_element_wise(const std::vector<float> &vec1,
                                       const std::vector<float> &vec2)
{
	std::vector<float> resultVec;
	auto len = vec1.size();

	for (decltype(len) idx = 0; idx < len; ++idx)
		{resultVec.push_back(vec1[idx] / vec2[idx]);}

	return resultVec;
}

std::vector<size_t> keep_by_threshold(const std::vector<float> &in_vector,
                                   const float &in_threshold)
{
	std::vector<size_t> resultVec;
	auto len = in_vector.size();

	for (decltype(len) index = 0; index < len; ++index)
	{
		if (in_vector[index] > in_threshold)
			{resultVec.push_back(index);}
	}

	return resultVec;
}

std::vector<size_t> remove_elements_by_index(const std::vector<size_t> &in_vector,
                                          const std::vector<size_t> &indices)
{
	auto resultVec = in_vector;
	auto offset = 0;

	for (const auto & index : indices)
	{
		resultVec.erase(resultVec.begin() + index + offset);
		offset -= 1;
	}

	return resultVec;
}

std::vector< std::vector<float> > BoxesToRectangles(const std::vector< std::vector<float> > & in_boxes)
{
	std::vector< std::vector<float> > rectangles;
	std::vector<float> box;

	for (const auto & box: in_boxes)
		{rectangles.push_back({box[0], box[1], box[2], box[3]});}

	return rectangles;
}

template <typename T>
std::vector<T> keep_only_indices(const std::vector<T> &in_vector,
                                 const std::vector<size_t> &in_indices)
{
	std::vector<T> resultVec;
  
	for (const auto & idx: in_indices)
		{resultVec.push_back(in_vector[idx]);}

	return resultVec;
}
