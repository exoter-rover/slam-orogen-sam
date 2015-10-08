#ifndef sam_TYPES_HPP
#define sam_TYPES_HPP

/* If you need to define types specific to your oroGen components, define them
 * here. Required headers must be included explicitly
 *
 * However, it is common that you will only import types from your library, in
 * which case you do not need this file
 */

#include <base/time.h>
#include <base/eigen.h>
#include <base/samples/RigidBodyState.hpp>
#include <base/samples/Pointcloud.hpp>

namespace sam {

    enum OutlierFilterType
    {
        NONE,
        STATISTICAL,
        RADIUS
    };

    struct BilateralFilterConfiguration
    {
        bool filterOn;
        float spatial_width; //size of the window bilateral filter
        float range_sigma; // the standard deviation of the Gaussian for the intensity difference
    };

    struct OutlierRemovalFilterConfiguration
    {
        OutlierFilterType type;

        //STATISTICAL: the number of nearest neighbors to use for mean distance estimation (nr_k)
        //RADIUS: Get the radius of the sphere that will determine which points are neighbors (radiu).
        float parameter_one;

        //STATISTICAL: the standard deviation multiplier for the distance threshold calculation.(stddev_null)
        //RADIUS: number of neighbors that need to be present in order to be classified as an inlier(min_pts)
        float parameter_two;
    };

    /** Output port type **/
    struct Information
    {
        base::Time time; //time-stamp
        base::Time predict_execution_time;
        std::vector< base::samples::RigidBodyState > pose_nodes_rbs; //Rbs with the orientation and position of the contact point
        std::vector< base::samples::RigidBodyState > landmark_nodes_rbs; //Rbs with the orientation and position of the contact point
        std::vector< base::samples::Pointcloud > point_clouds; //Rbs with the orientation and position of the contact point
    };

}

#endif

