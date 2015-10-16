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


    /** Pose correspondences **/
    struct PoseCorrespondence
    {
        int node_idx;
        std::vector<int> correspondences_idx;
    };

    /** Output port type **/
    struct Information
    {
        base::Time time; //time-stamp
        base::Time predict_execution_time;
        double accumulated_distance;
        std::vector< base::samples::RigidBodyState > pose_nodes_rbs; //Rbs of the poses in the node graph
        std::vector< base::samples::RigidBodyState > landmark_nodes_rbs; //Rbs of the landmarks in the node graph
        std::vector< PoseCorrespondence > poses_correspondences; //Idx of the poses correspondences to find matches
    };

}

#endif

