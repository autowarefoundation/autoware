/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */

#ifndef FAST_PCL_REGISTRATION_CORRESPONDENCE_REJECTION_H_
#define FAST_PCL_REGISTRATION_CORRESPONDENCE_REJECTION_H_

//#include <pcl/registration/correspondence_types.h>
//#include <pcl/registration/correspondence_sorting.h>
#include "fast_pcl/registration/correspondence_types.h"
#include "fast_pcl/registration/correspondence_sorting.h"
#include <pcl/console/print.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>

namespace pcl
{
  namespace registration
  {
    /** @b CorrespondenceRejector represents the base class for correspondence rejection methods
      * \author Dirk Holz
      * \ingroup registration
      */
    class CorrespondenceRejector
    {
      public:
        typedef boost::shared_ptr<CorrespondenceRejector> Ptr;
        typedef boost::shared_ptr<const CorrespondenceRejector> ConstPtr;

        /** \brief Empty constructor. */
        CorrespondenceRejector () 
          : rejection_name_ ()
          , input_correspondences_ () 
        {}

        /** \brief Empty destructor. */
        virtual ~CorrespondenceRejector () {}

        /** \brief Provide a pointer to the vector of the input correspondences.
          * \param[in] correspondences the const boost shared pointer to a correspondence vector
          */
        virtual inline void 
        setInputCorrespondences (const CorrespondencesConstPtr &correspondences) 
        { 
          input_correspondences_ = correspondences; 
        };

        /** \brief Get a pointer to the vector of the input correspondences.
          * \return correspondences the const boost shared pointer to a correspondence vector
          */
        inline CorrespondencesConstPtr 
        getInputCorrespondences () { return input_correspondences_; };

        /** \brief Run correspondence rejection
          * \param[out] correspondences Vector of correspondences that have not been rejected.
          */
        inline void 
        getCorrespondences (pcl::Correspondences &correspondences)
        {
          if (!input_correspondences_ || (input_correspondences_->empty ()))
            return;

          applyRejection (correspondences);
        }

        /** \brief Get a list of valid correspondences after rejection from the original set of correspondences.
          * Pure virtual. Compared to \a getCorrespondences this function is
          * stateless, i.e., input correspondences do not need to be provided beforehand,
          * but are directly provided in the function call.
          * \param[in] original_correspondences the set of initial correspondences given
          * \param[out] remaining_correspondences the resultant filtered set of remaining correspondences
          */
        virtual inline void 
        getRemainingCorrespondences (const pcl::Correspondences& original_correspondences, 
                                     pcl::Correspondences& remaining_correspondences) = 0;

        /** \brief Determine the indices of query points of
          * correspondences that have been rejected, i.e., the difference
          * between the input correspondences (set via \a setInputCorrespondences)
          * and the given correspondence vector.
          * \param[in] correspondences Vector of correspondences after rejection
          * \param[out] indices Vector of query point indices of those correspondences
          * that have been rejected.
          */
        inline void 
        getRejectedQueryIndices (const pcl::Correspondences &correspondences, 
                                 std::vector<int>& indices)
        {
          if (!input_correspondences_ || input_correspondences_->empty ())
          {
            PCL_WARN ("[pcl::registration::%s::getRejectedQueryIndices] Input correspondences not set (lookup of rejected correspondences _not_ possible).\n", getClassName ().c_str ());
            return;
          }

          pcl::getRejectedQueryIndices(*input_correspondences_, correspondences, indices);
        }

        /** \brief Get a string representation of the name of this class. */
        inline const std::string& 
        getClassName () const { return (rejection_name_); }


        /** \brief See if this rejector requires source points */
        virtual bool
        requiresSourcePoints () const
        { return (false); }

        /** \brief Abstract method for setting the source cloud */
        virtual void
        setSourcePoints (pcl::PCLPointCloud2::ConstPtr /*cloud2*/)
        {
          PCL_WARN ("[pcl::registration::%s::setSourcePoints] This class does not require an input source cloud", getClassName ().c_str ());
        }
        
        /** \brief See if this rejector requires source normals */
        virtual bool
        requiresSourceNormals () const
        { return (false); }

        /** \brief Abstract method for setting the source normals */
        virtual void
        setSourceNormals (pcl::PCLPointCloud2::ConstPtr /*cloud2*/)
        { 
          PCL_WARN ("[pcl::registration::%s::setSourceNormals] This class does not require input source normals", getClassName ().c_str ());
        }
        /** \brief See if this rejector requires a target cloud */
        virtual bool
        requiresTargetPoints () const
        { return (false); }

        /** \brief Abstract method for setting the target cloud */
        virtual void
        setTargetPoints (pcl::PCLPointCloud2::ConstPtr /*cloud2*/)
        {
          PCL_WARN ("[pcl::registration::%s::setTargetPoints] This class does not require an input target cloud", getClassName ().c_str ());
        }
        
        /** \brief See if this rejector requires target normals */
        virtual bool
        requiresTargetNormals () const
        { return (false); }

        /** \brief Abstract method for setting the target normals */
        virtual void
        setTargetNormals (pcl::PCLPointCloud2::ConstPtr /*cloud2*/)
        {
          PCL_WARN ("[pcl::registration::%s::setTargetNormals] This class does not require input target normals", getClassName ().c_str ());
        }

      protected:

        /** \brief The name of the rejection method. */
        std::string rejection_name_;

        /** \brief The input correspondences. */
        CorrespondencesConstPtr input_correspondences_;

        /** \brief Abstract rejection method. */
        virtual void 
        applyRejection (Correspondences &correspondences) = 0;
    };

    /** @b DataContainerInterface provides a generic interface for computing correspondence scores between correspondent
      * points in the input and target clouds
      * \ingroup registration
      */
    class DataContainerInterface
    {
      public:
        virtual ~DataContainerInterface () {}
        virtual double getCorrespondenceScore (int index) = 0;
        virtual double getCorrespondenceScore (const pcl::Correspondence &) = 0;
     };

    /** @b DataContainer is a container for the input and target point clouds and implements the interface 
      * to compute correspondence scores between correspondent points in the input and target clouds
      * \ingroup registration
      */
    template <typename PointT, typename NormalT = pcl::PointNormal>
    class DataContainer : public DataContainerInterface
    {
      typedef pcl::PointCloud<PointT> PointCloud;
      typedef typename PointCloud::Ptr PointCloudPtr;
      typedef typename PointCloud::ConstPtr PointCloudConstPtr;

      typedef typename pcl::search::KdTree<PointT>::Ptr KdTreePtr;
      
      typedef pcl::PointCloud<NormalT> Normals;
      typedef typename Normals::Ptr NormalsPtr;
      typedef typename Normals::ConstPtr NormalsConstPtr;

      public:

        /** \brief Empty constructor. */
        DataContainer (bool needs_normals = false) 
          : input_ ()
          , input_transformed_ ()
          , target_ ()
          , input_normals_ ()
          , input_normals_transformed_ ()
          , target_normals_ ()
          , tree_ (new pcl::search::KdTree<PointT>)
          , class_name_ ("DataContainer")
          , needs_normals_ (needs_normals)
          , target_cloud_updated_ (true)
          , force_no_recompute_ (false)
        {
        }
      
        /** \brief Empty destructor */
        virtual ~DataContainer () {}

        /** \brief Provide a source point cloud dataset (must contain XYZ
          * data!), used to compute the correspondence distance.  
          * \param[in] cloud a cloud containing XYZ data
          */
        PCL_DEPRECATED ("[pcl::registration::DataContainer::setInputCloud] setInputCloud is deprecated. Please use setInputSource instead.")
        void
        setInputCloud (const PointCloudConstPtr &cloud);

        /** \brief Get a pointer to the input point cloud dataset target. */
        PCL_DEPRECATED ("[pcl::registration::DataContainer::getInputCloud] getInputCloud is deprecated. Please use getInputSource instead.")
        PointCloudConstPtr const
        getInputCloud ();

        /** \brief Provide a source point cloud dataset (must contain XYZ
          * data!), used to compute the correspondence distance.  
          * \param[in] cloud a cloud containing XYZ data
          */
        inline void 
        setInputSource (const PointCloudConstPtr &cloud)
        {
          input_ = cloud;
        }

        /** \brief Get a pointer to the input point cloud dataset target. */
        inline PointCloudConstPtr const 
        getInputSource () { return (input_); }

        /** \brief Provide a target point cloud dataset (must contain XYZ
          * data!), used to compute the correspondence distance.  
          * \param[in] target a cloud containing XYZ data
          */
        inline void 
        setInputTarget (const PointCloudConstPtr &target)
        {
          target_ = target;
          target_cloud_updated_ = true;
        }

        /** \brief Get a pointer to the input point cloud dataset target. */
        inline PointCloudConstPtr const 
        getInputTarget () { return (target_); }
        
        /** \brief Provide a pointer to the search object used to find correspondences in
          * the target cloud.
          * \param[in] tree a pointer to the spatial search object.
          * \param[in] force_no_recompute If set to true, this tree will NEVER be 
          * recomputed, regardless of calls to setInputTarget. Only use if you are 
          * confident that the tree will be set correctly.
          */
        inline void
        setSearchMethodTarget (const KdTreePtr &tree, 
                               bool force_no_recompute = false) 
        { 
          tree_ = tree; 
          if (force_no_recompute)
          {
            force_no_recompute_ = true;
          }
          target_cloud_updated_ = true;
        }

        /** \brief Set the normals computed on the input point cloud
          * \param[in] normals the normals computed for the input cloud
          */
        inline void
        setInputNormals (const NormalsConstPtr &normals) { input_normals_ = normals; }

        /** \brief Get the normals computed on the input point cloud */
        inline NormalsConstPtr
        getInputNormals () { return (input_normals_); }

        /** \brief Set the normals computed on the target point cloud
          * \param[in] normals the normals computed for the input cloud
          */
        inline void
        setTargetNormals (const NormalsConstPtr &normals) { target_normals_ = normals; }
        
        /** \brief Get the normals computed on the target point cloud */
        inline NormalsConstPtr
        getTargetNormals () { return (target_normals_); }

        /** \brief Get the correspondence score for a point in the input cloud
          * \param[in] index index of the point in the input cloud
          */
        inline double 
        getCorrespondenceScore (int index)
        {
          if ( target_cloud_updated_ && !force_no_recompute_ )
          {
            tree_->setInputCloud (target_);
          }
          std::vector<int> indices (1);
          std::vector<float> distances (1);
          if (tree_->nearestKSearch (input_->points[index], 1, indices, distances))
            return (distances[0]);
          else
            return (std::numeric_limits<double>::max ());
        }

        /** \brief Get the correspondence score for a given pair of correspondent points
          * \param[in] corr Correspondent points
          */
        inline double 
        getCorrespondenceScore (const pcl::Correspondence &corr)
        {
          // Get the source and the target feature from the list
          const PointT &src = input_->points[corr.index_query];
          const PointT &tgt = target_->points[corr.index_match];

          return ((src.getVector4fMap () - tgt.getVector4fMap ()).squaredNorm ());
        }
        
        /** \brief Get the correspondence score for a given pair of correspondent points based on 
          * the angle betweeen the normals. The normmals for the in put and target clouds must be 
          * set before using this function
          * \param[in] corr Correspondent points
          */
        inline double
        getCorrespondenceScoreFromNormals (const pcl::Correspondence &corr)
        {
          //assert ( (input_normals_->points.size () != 0) && (target_normals_->points.size () != 0) && "Normals are not set for the input and target point clouds");
          assert (input_normals_ && target_normals_ && "Normals are not set for the input and target point clouds");
          const NormalT &src = input_normals_->points[corr.index_query];
          const NormalT &tgt = target_normals_->points[corr.index_match];
          return (double ((src.normal[0] * tgt.normal[0]) + (src.normal[1] * tgt.normal[1]) + (src.normal[2] * tgt.normal[2])));
        }

     private:
        /** \brief The input point cloud dataset */
        PointCloudConstPtr input_;

        /** \brief The input transformed point cloud dataset */
        PointCloudPtr input_transformed_;

        /** \brief The target point cloud datase. */
        PointCloudConstPtr target_;

        /** \brief Normals to the input point cloud */
        NormalsConstPtr input_normals_;

        /** \brief Normals to the input point cloud */
        NormalsPtr input_normals_transformed_;

        /** \brief Normals to the target point cloud */
        NormalsConstPtr target_normals_;

        /** \brief A pointer to the spatial search object. */
        KdTreePtr tree_;

        /** \brief The name of the rejection method. */
        std::string class_name_;

        /** \brief Should the current data container use normals? */
        bool needs_normals_;

        /** \brief Variable that stores whether we have a new target cloud, meaning we need to pre-process it again.
         * This way, we avoid rebuilding the kd-tree */
        bool target_cloud_updated_;

        /** \brief A flag which, if set, means the tree operating on the target cloud 
         * will never be recomputed*/
        bool force_no_recompute_;



        /** \brief Get a string representation of the name of this class. */
        inline const std::string& 
        getClassName () const { return (class_name_); }
    };
  }
}

      //#include <pcl/registration/impl/correspondence_rejection.hpp>
#include "fast_pcl/registration/impl/correspondence_rejection.hpp"

#endif /* FAST_PCL_REGISTRATION_CORRESPONDENCE_REJECTION_H_ */

