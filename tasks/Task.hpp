/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef SAM_TASK_TASK_HPP
#define SAM_TASK_TASK_HPP

#include "sam/TaskBase.hpp"

/** STD **/
#include <vector>
#include <cstdlib>
#include <cmath>
#include <time.h>

/** Boost **/
#include <boost/shared_ptr.hpp> /** shared pointers **/

/** Eigen **/
#include <Eigen/Core> /** Core */
#include <Eigen/StdVector> /** For STL container with Eigen types **/

/** Base Types **/
#include <base/samples/RigidBodyState.hpp>
#include <base/samples/Pointcloud.hpp>

/** MTK **/
#include <mtk/types/pose.hpp>
#include <mtk/types/SOn.hpp>
#include <mtk/build_manifold.hpp>
#include <ukfom/ukf.hpp>
#include <ukfom/mtkwrap.hpp>

/** Envire **/
#include <envire_core/all>

/** SAM **/
#include <envire_sam/ESAM.hpp>

namespace sam {

    /** MTK TYPES **/
    // We can't use types having a comma inside AutoConstruct macros :(
    typedef MTK::vect<3, double> vec3;
    typedef MTK::SO3<double> SO3;

    MTK_BUILD_MANIFOLD ( MTKState ,
    (( vec3, pos ))
    (( SO3, orient ))
    (( vec3, velo ))
    (( vec3, angvelo ))
    );

    typedef ukfom::mtkwrap<MTKState> WMTKState;
    typedef ukfom::ukf<WMTKState> UKF;

    /*! \class Task 
     * \brief The task context provides and requires services. It uses an ExecutionEngine to perform its functions.
     * Essential interfaces are operations, data flow ports and properties. These interfaces have been defined using the oroGen specification.
     * In order to modify the interfaces you should (re)use oroGen and rely on the associated workflow.
     * Declare the State Optimization class
     * \details
     * The name of a TaskContext is primarily defined via:
     \verbatim
     deployment 'deployment_name'
         task('custom_task_name','sam::Task')
     end
     \endverbatim
     *  It can be dynamically adapted when the deployment is called with a prefix argument. 
     */
    class Task : public TaskBase
    {
	friend class TaskBase;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW //Structures having Eigen members

    protected:
        /******************************/
        /*** Control Flow Variables ***/
        /******************************/
        bool initFilter;

        /**************************/
        /*** Property Variables ***/
        /**************************/

        /** Bilateral filter Configuration **/
        envire::sam::BilateralFilterParams bfilter_config;

        /** Outlier Filter Removal Configuration **/
        envire::sam::OutlierRemovalParams outlier_config;

        /** SIFT Keypoints **/
        envire::sam::SIFTKeypointParams sift_config;

        /** Feature descriptor **/
        envire::sam::PFHFeatureParams feature_config;

        /******************************************/
        /*** General Internal Storage Variables ***/
        /******************************************/

        /** State of the filter **/
        base::TransformWithCovariance pose_with_cov;

        /** State of the filter **/
        WMTKState pose_state;

        /** The pose prediction in an UT form **/
        boost::shared_ptr<UKF> filter;

        /** Envire Smoothing and Mapping **/
        boost::shared_ptr<envire::sam::ESAM> esam;

        /** Norm of the covariance in segment **/
        double velocity_norm_cov;

        int velocity_cov_counts;

        double accumulated_segment;

        base::Time start_distance_segment_time;

        /**************************/
        /** Input port variables **/
        /**************************/

        /** Delta Pose estimation **/
        ::base::samples::RigidBodyState delta_pose;

        /***************************/
        /** Output port variables **/
        /***************************/

        base::samples::RigidBodyState sam_pose_out;

        base::samples::RigidBodyState odo_pose_out;

        /** Current map in point cloud form **/
        base::samples::Pointcloud base_point_cloud_map_out;

        /** Debug filter info **/
        sam::Information info;

    protected:

        virtual void delta_pose_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &delta_pose_samples_sample);

        virtual void point_cloud_samplesTransformerCallback(const base::Time &ts, const ::base::samples::Pointcloud &point_cloud_samples_sample);

    public:
        /** TaskContext constructor for Task
         * \param name Name of the task. This name needs to be unique to make it identifiable via nameservices.
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        Task(std::string const& name = "sam::Task");

        /** TaskContext constructor for Task 
         * \param name Name of the task. This name needs to be unique to make it identifiable for nameservices. 
         * \param engine The RTT Execution engine to be used for this task, which serialises the execution of all commands, programs, state machines and incoming events for a task. 
         * 
         */
        Task(std::string const& name, RTT::ExecutionEngine* engine);

        /** Default deconstructor of Task
         */
	~Task();

        /** This hook is called by Orocos when the state machine transitions
         * from PreOperational to Stopped. If it returns false, then the
         * component will stay in PreOperational. Otherwise, it goes into
         * Stopped.
         *
         * It is meaningful only if the #needs_configuration has been specified
         * in the task context definition with (for example):
         \verbatim
         task_context "TaskName" do
           needs_configuration
           ...
         end
         \endverbatim
         */
        bool configureHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to Running. If it returns false, then the component will
         * stay in Stopped. Otherwise, it goes into Running and updateHook()
         * will be called.
         */
        bool startHook();

        /** This hook is called by Orocos when the component is in the Running
         * state, at each activity step. Here, the activity gives the "ticks"
         * when the hook should be called.
         *
         * The error(), exception() and fatal() calls, when called in this hook,
         * allow to get into the associated RunTimeError, Exception and
         * FatalError states. 
         *
         * In the first case, updateHook() is still called, and recover() allows
         * you to go back into the Running state.  In the second case, the
         * errorHook() will be called instead of updateHook(). In Exception, the
         * component is stopped and recover() needs to be called before starting
         * it again. Finally, FatalError cannot be recovered.
         */
        void updateHook();

        /** This hook is called by Orocos when the component is in the
         * RunTimeError state, at each activity step. See the discussion in
         * updateHook() about triggering options.
         *
         * Call recover() to go back in the Runtime state.
         */
        void errorHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Running to Stopped after stop() has been called.
         */
        void stopHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to PreOperational, requiring the call to configureHook()
         * before calling start() again.
         */
        void cleanupHook();

        /**@brief initialization
         */
        void initialization(Eigen::Affine3d &tf);

        /**@brief Initialize the UKF
         */
        void initUKF(WMTKState &statek, UKF::cov &statek_cov);

        /**@brief Update
         */
        void updateESAM();

        /**@brief Reset the UKF
         */
        void resetUKF(::base::Pose &current_delta_pose, ::base::Matrix6d &cov_current_delta_pose);

        /**@brief Initialize the envire SAM
         */
        void initESAM(base::TransformWithCovariance &tf_cov);

        /**@brief Check the velocity uncertainty over the last segment velocity
         */
        bool checkSegmentVelocityCov(const base::Time &start, const base::Time &end, const double &segment);

        /**@brief Check the uncertainty over the last segment
         */
        bool checkSegmentCov(const double &current_segment);

        /** @brief Port out the values
        */
        void outputPortSamples(const base::Time &timestamp);

        /** @brief Port out the Point Cloud map
        */
        void outputPortPointCloud(const base::Time &timestamp);

        /** @brief Port out the Point Cloud map
        */
        void updateInformation(const int &current_segment);


    };
}

#endif

