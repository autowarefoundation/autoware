// file: cvmat_serialization.h


#ifndef _CV_MAT_SERIALIZER
#define _CV_MAT_SERIALIZER

#include <opencv2/opencv.hpp>
#include <boost/serialization/split_free.hpp>
#include <boost/serialization/vector.hpp>

BOOST_SERIALIZATION_SPLIT_FREE(::cv::Mat)
namespace boost {
  namespace serialization {

    /** Serialization support for cv::Mat */
    template <class Archive>
    void save(Archive & ar, const cv::Mat& m, const unsigned int version)
    {
      size_t elem_size = m.elemSize();
      size_t elem_type = m.type();

      ar & m.cols;
      ar & m.rows;
      ar & elem_size;
      ar & elem_type;

      const size_t data_size = m.cols * m.rows * elem_size;
      ar & boost::serialization::make_array(m.ptr(), data_size);
    }

    /** Serialization support for cv::Mat */
    template <class Archive>
    void load(Archive & ar, cv::Mat& m, const unsigned int version)
    {
      int cols, rows;
      size_t elem_size, elem_type;

      ar & cols;
      ar & rows;
      ar & elem_size;
      ar & elem_type;

      m.create(rows, cols, elem_type);

      size_t data_size = m.cols * m.rows * elem_size;
      ar & boost::serialization::make_array(m.ptr(), data_size);
    }

  }
}



BOOST_SERIALIZATION_SPLIT_FREE(cv::KeyPoint)
namespace boost{
	namespace serialization {

		template <class Archive>
		void save (Archive &ar, const cv::KeyPoint &kp, const unsigned int version)
		{
			float kx = kp.pt.x,
				ky = kp.pt.y;

			ar & kx;
			ar & ky;
			ar & kp.size;
			ar & kp.angle;
			ar & kp.response;
			ar & kp.octave;
			ar & kp.class_id;
		}

		template <class Archive>
		void load (Archive &ar, cv::KeyPoint &kp, const unsigned int version)
		{
			float kx, ky;

			ar & kx;
			ar & ky;
			ar & kp.size;
			ar & kp.angle;
			ar & kp.response;
			ar & kp.octave;
			ar & kp.class_id;
			kp.pt.x = kx;
			kp.pt.y = ky;
		}
	}
}


#endif // _CV_MAT_SERIALIZER
