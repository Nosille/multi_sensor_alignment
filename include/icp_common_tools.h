/** ROS node 

/* 

Copyright (c) 2017

*/

#ifndef ICP_COMMON_TOOL_H
#define ICP_COMMON_TOOL_H

#include <iostream>
#include <vector> 
#include <sstream>

#include <mutex>

#include <ros/ros.h>

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/common/intersections.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/impl/extract_indices.hpp>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/impl/project_inliers.hpp> 
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/impl/sac_segmentation.hpp>
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/concave_hull.h>

#include <pcl/point_cloud.h>
#include <pcl/type_traits.h>
#include <pcl/point_representation.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/conditional_removal.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/ndt.h>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/rolling_mean.hpp>

#include "std_msgs/String.h"

//convenient typedefs
typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

using namespace boost::accumulators;
typedef accumulator_set<double, stats<tag::rolling_mean > > window_acc;

/**
 * \brief Cross-registers pointcloud2 topics for use in alignment
 */

namespace Multi_Sensor_Alignment
{
    
    //Returns true if the two input quaternions are close to each other. This can
    //be used to check whether or not one of two quaternions which are supposed to
    //be very similar but has its component signs reversed (q has the same rotation as
    //-q)
    static bool AreQuaternionsClose(tf2::Quaternion q1, tf2::Quaternion q2)
    {

        float dot = q1.dot(q2);
        
        if(dot < 0.0f)
        {

            return false;                   
        }

        else
        {

            return true;
        }
    }
    
    geometry_msgs::Quaternion AverageQuaternion(const geometry_msgs::Quaternion& _newRotation, 
    window_acc &_qx_array, window_acc &_qy_array, window_acc &_qz_array, window_acc &_qw_array, 
    double &_current_qx, double &_current_qy, double &_current_qz, double &_current_qw)
    {
        //ROS_INFO_STREAM_NAMED(node_name, current_qx_ << " " << current_qy_ << " " << current_qz_ << " " << current_qw_);
        tf2::Quaternion lastRotation(_current_qx, _current_qy, _current_qz, _current_qw);
        tf2::Quaternion currRotation; 
        tf2::convert(_newRotation, currRotation);
        //On first pass lastRotation will be zero length
        if(abs(lastRotation.length()) < 0.1)
        {
        ROS_INFO_STREAM("AverageQuaternion initialized");

        //Add new values to accumulators
        _qx_array(currRotation.x()); _current_qx = currRotation.x();
        _qy_array(currRotation.y()); _current_qy = currRotation.y();
        _qz_array(currRotation.z()); _current_qz = currRotation.z();
        _qw_array(currRotation.w()); _current_qw = currRotation.w();

        return _newRotation;
        }

        //Before we add the new rotation to the average (mean), we have to check whether the quaternion has to be inverted. Because
        //q and -q are the same rotation, but cannot be averaged, we have to make sure they are all the same.
        // if(AreQuaternionsClose(currRotation, lastRotation))
        // {
        //     ROS_INFO_STREAM_NAMED(node_name, "flip quaternion");
        //     ROS_INFO_STREAM_NAMED(node_name, currRotation.x() << " " << currRotation.y() << " " << currRotation.z() << " " << currRotation.w());
        //     ROS_INFO_STREAM_NAMED(node_name, lastRotation.x() << " " << lastRotation.y() << " " << lastRotation.z() << " " << lastRotation.w());
        //     currRotation = tf2::Quaternion(-currRotation.x(), -currRotation.y(), -currRotation.z(), -currRotation.w());
        // }
        _current_qx = currRotation.x();
        _current_qy = currRotation.y();
        _current_qz = currRotation.z();
        _current_qw = currRotation.w();
        
        //Add new values to accumulators
        _qx_array(currRotation.x());
        _qy_array(currRotation.y());
        _qz_array(currRotation.z());
        _qw_array(currRotation.w());
        float w = rolling_mean(_qw_array);
        float x = rolling_mean(_qx_array);
        float y = rolling_mean(_qy_array);
        float z = rolling_mean(_qz_array);

        //Convert back to quaternion
        tf2::Quaternion mean(x, y, z, w);

        geometry_msgs::Quaternion result;
        tf2::convert(mean.normalize(), result);

        //note: if speed is an issue, you can skip the normalization step
        return result;
    }

    void DownsampleCloud(const pcl::PointCloud<PointT>::Ptr in_cloud, pcl::PointCloud<PointT>::Ptr out_cloud, double in_leaf_size,
        double i_min, double i_max, 
        double x_min, double x_max, 
        double y_min, double y_max, 
        double z_min, double z_max)
    {
        pcl::PointCloud<PointT>::Ptr filtered_ptr(new pcl::PointCloud<PointT>);
    
        // build the condition
        pcl::ConditionAnd<PointT>::Ptr range_cond (new pcl::ConditionAnd<PointT> ());
        range_cond->addComparison (pcl::FieldComparison<PointT>::ConstPtr (new pcl::FieldComparison<PointT> ("intensity", pcl::ComparisonOps::GE, i_min)));
        range_cond->addComparison (pcl::FieldComparison<PointT>::ConstPtr (new pcl::FieldComparison<PointT> ("intensity", pcl::ComparisonOps::LE, i_max)));
        range_cond->addComparison (pcl::FieldComparison<PointT>::ConstPtr (new pcl::FieldComparison<PointT> ("x",         pcl::ComparisonOps::GE, x_min)));
        range_cond->addComparison (pcl::FieldComparison<PointT>::ConstPtr (new pcl::FieldComparison<PointT> ("x",         pcl::ComparisonOps::LE, x_max)));
        range_cond->addComparison (pcl::FieldComparison<PointT>::ConstPtr (new pcl::FieldComparison<PointT> ("y",         pcl::ComparisonOps::GE, y_min)));
        range_cond->addComparison (pcl::FieldComparison<PointT>::ConstPtr (new pcl::FieldComparison<PointT> ("y",         pcl::ComparisonOps::LE, y_max)));
        range_cond->addComparison (pcl::FieldComparison<PointT>::ConstPtr (new pcl::FieldComparison<PointT> ("z",         pcl::ComparisonOps::GE, z_min)));
        range_cond->addComparison (pcl::FieldComparison<PointT>::ConstPtr (new pcl::FieldComparison<PointT> ("z",         pcl::ComparisonOps::LE, z_max)));
        // build the filter
        pcl::ConditionalRemoval<PointT> condrem;
        condrem.setCondition (range_cond);
        condrem.setInputCloud (in_cloud);
        condrem.setKeepOrganized(false);
        // apply filter
        condrem.filter (*filtered_ptr);

        if(in_leaf_size > 0.001)
        {
        pcl::VoxelGrid<PointT> voxelized;
        voxelized.setInputCloud(filtered_ptr);
        voxelized.setLeafSize((float)in_leaf_size, (float)in_leaf_size, (float)in_leaf_size);
        voxelized.filter(*out_cloud);
        }
        else
        {
        pcl::copyPointCloud(*filtered_ptr, *out_cloud);
        }
    }
    
    void PerformRegistration(pcl::PointCloud<PointT>::ConstPtr _cloud0, pcl::PointCloud<PointT>::ConstPtr _cloud1,
         Eigen::Matrix4f &_current_registration, int _Method, int _MaxIterations, double _Epsilon,  
         int _KSearch, double _RadiusSearch, double _MaxCorrespondenceDistance, double _StepSize, double _Resolution)
    {
    // ICP Nonlinear with scaling CorDist
        if(_Method == 0)
        {
            // Compute surface normals and curvature
            PointCloudWithNormals::Ptr points_with_normals0 (new PointCloudWithNormals);
            PointCloudWithNormals::Ptr points_with_normals1 (new PointCloudWithNormals);

            pcl::NormalEstimation<PointT, PointNormalT> norm_est;
            pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI> ());
            norm_est.setSearchMethod (tree);
            if(_KSearch > 0) norm_est.setKSearch(_KSearch);
            else norm_est.setRadiusSearch(_RadiusSearch);
            
            norm_est.setInputCloud (_cloud0);
            norm_est.compute (*points_with_normals0);
            pcl::copyPointCloud (*_cloud0, *points_with_normals0);

            norm_est.setInputCloud (_cloud1);
            norm_est.compute (*points_with_normals1);
            pcl::copyPointCloud (*_cloud1, *points_with_normals1);

            // Align
            pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg;
            reg.setTransformationEpsilon (_Epsilon);
            // Set the maximum distance between two correspondences (cloud0<->cloud1) to user input
            // Note: adjust this based on the size of your datasets
            reg.setMaxCorrespondenceDistance (_MaxCorrespondenceDistance);  
            // Set the first pointcloud as the target
            reg.setInputTarget (points_with_normals0);

            // Run the optimization in a loop and visualize the results
            PointCloudWithNormals::Ptr reg_result = points_with_normals1;
            reg.setMaximumIterations (2);
            Eigen::Matrix4f prev;
            for (int i = 0; i < _MaxIterations; ++i)
            {
                ROS_DEBUG_STREAM("Iteration Nr. " << i << " maxCorrespondenceDistance=" << reg.getMaxCorrespondenceDistance () << ".\n");

                // save previous cloud
                points_with_normals1 = reg_result;

                // Estimate
                reg.setInputSource (points_with_normals1);
                reg.align (*reg_result, _current_registration);
            
                //accumulate transformation between each Iteration
                if(reg.hasConverged ())
                {
                _current_registration = reg.getFinalTransformation ();

                //if the difference between this transformation and the previous one
                //is smaller than the threshold, refine the process by reducing
                //the maximal correspondence distance
                if (std::abs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ())
                                reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () * 0.99);
                }
                
                prev = reg.getLastIncrementalTransformation();
            }

            ROS_INFO_STREAM("ICP Nonlinear Transform converged:" << reg.hasConverged ()
                    << " score: " << reg.getFitnessScore () << " epsilon:" << reg.getTransformationEpsilon());

        }
        // ICP Nonlinear
        else if(_Method == 1)
        {
            // Compute surface normals and curvature
            PointCloudWithNormals::Ptr points_with_normals0 (new PointCloudWithNormals);
            PointCloudWithNormals::Ptr points_with_normals1 (new PointCloudWithNormals);

            pcl::NormalEstimation<PointT, PointNormalT> norm_est;
            pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI> ());
            norm_est.setSearchMethod (tree);
            if(_KSearch > 0) norm_est.setKSearch(_KSearch);
            else norm_est.setRadiusSearch(_RadiusSearch);
            
            norm_est.setInputCloud (_cloud0);
            norm_est.compute (*points_with_normals0);
            pcl::copyPointCloud (*_cloud0, *points_with_normals0);

            norm_est.setInputCloud (_cloud1);
            norm_est.compute (*points_with_normals1);
            pcl::copyPointCloud (*_cloud1, *points_with_normals1);

            // Align
            pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg;
            reg.setTransformationEpsilon (_Epsilon);
            // Set the maximum distance between two correspondences (cloud0<->cloud1) to user input
            // Note: adjust this based on the size of your datasets
            reg.setMaxCorrespondenceDistance (_MaxCorrespondenceDistance);  
            reg.setMaximumIterations(_MaxIterations);
            // Set the first pointcloud as the target
            reg.setInputTarget (points_with_normals0);
            reg.setInputSource (points_with_normals1);

            PointCloudWithNormals::Ptr reg_result = points_with_normals1;
            
            //Get Results
            reg.align (*reg_result, _current_registration);

            ROS_INFO_STREAM("ICP Nonlinear Transform converged:" << reg.hasConverged ()
                    << " score: " << reg.getFitnessScore () << " epsilon:" << reg.getTransformationEpsilon());
            
            if(reg.hasConverged() ) _current_registration = reg.getFinalTransformation();

        }
        // Normal Distributions Transform
        else if(_Method == 2)
        {
            // Initializing Normal Distributions Transform (NDT).
            pcl::NormalDistributionsTransform<PointT, PointT> ndt;

            ndt.setTransformationEpsilon(_Epsilon);
            ndt.setStepSize(_StepSize);
            ndt.setResolution(_Resolution);
            ndt.setMaximumIterations(_MaxIterations);

            // Set the first pointcloud as the target
            ndt.setInputTarget(_cloud0);
            ndt.setInputSource(_cloud1);

            //Get Results
            pcl::PointCloud<PointT>::Ptr reg_result;
            ndt.align(*reg_result, _current_registration);
            
            ROS_INFO_STREAM("Normal Distributions Transform converged:" << ndt.hasConverged ()
                        << " score: " << ndt.getFitnessScore () << " prob:" << ndt.getTransformationProbability());

            if(ndt.hasConverged() ) _current_registration = ndt.getFinalTransformation();

        }
        // ICP with Normals
        else if(_Method == 3)
        {
            #if (defined(PCL_VERSION) && PCL_VERSION_COMPARE(<, 1, 10, 1))
                throw std::runtime_error("PCL must be at or above version 1.10.1 for this method=3");
            #else

                // Compute surface normals and curvature
                PointCloudWithNormals::Ptr points_with_normals0 (new PointCloudWithNormals);
                PointCloudWithNormals::Ptr points_with_normals1 (new PointCloudWithNormals);

                pcl::NormalEstimation<PointT, PointNormalT> norm_est;
                pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI> ());
                norm_est.setSearchMethod (tree);
                
                if(_KSearch > 0) norm_est.setKSearch (_KSearch);
                else norm_est.setRadiusSearch(_RadiusSearch);
                
                norm_est.setInputCloud (_cloud0);
                norm_est.compute (*points_with_normals0);
                pcl::copyPointCloud (*_cloud0, *points_with_normals0);

                norm_est.setInputCloud (_cloud1);
                norm_est.compute (*points_with_normals1);
                pcl::copyPointCloud (*_cloud1, *points_with_normals1);

                // Align
                pcl::IterativeClosestPointWithNormals<PointNormalT, PointNormalT> reg;
                reg.setTransformationEpsilon (_Epsilon);
                // Set the maximum distance between two correspondences (cloud0<->cloud1) to user input
                // Note: adjust this based on the size of your datasets
                reg.setMaxCorrespondenceDistance (_MaxCorrespondenceDistance);  
                reg.setMaximumIterations(_MaxIterations);
                reg.setUseSymmetricObjective(true);
                reg.setEnforceSameDirectionNormals(true);
                // Set the first pointcloud as the target
                reg.setInputTarget (points_with_normals0);
                reg.setInputSource (points_with_normals1);

                PointCloudWithNormals::Ptr reg_result = points_with_normals1;
                
                //Get Results
                reg.align (*reg_result, _current_registration);

                ROS_INFO_STREAM("ICP with Normals converged:" << reg.hasConverged ()
                        << " score: " << reg.getFitnessScore () << " epsilon:" << reg.getTransformationEpsilon());
                
                if(reg.hasConverged() ) _current_registration = reg.getFinalTransformation();


            #endif
        }
    }
} // namespace Multi_Sensor_Alignment

#endif  // ICP_COMMON_TOOL_H

