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

namespace sam {

    // We can't use types having a comma inside AutoConstruct macros :(
    typedef MTK::vect<3, double> vec3;
    typedef MTK::SO3<double> SO3;


    MTK_BUILD_MANIFOLD ( MTKState ,
    (( vec3, pos ))
    (( SO3, orient ))
    (( vec3, vel ))
    );

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

    protected:
        /**************************/
        /*** Property Variables ***/
        /**************************/

        /******************************************/
        /*** General Internal Storage Variables ***/
        /******************************************/

        envire::core::TransformGraph envire_graph; //! The environment in a graph structure

        boost::shared_ptr< ukfom::ukf<MTKState> > pose_ukf; //! The pose prediction in a UT form

        /**************************/
        /** Input port variables **/
        /**************************/


        /***************************/
        /** Output port variables **/
        /***************************/
        base::samples::RigidBodyState pose_out;
        base::samples::Pointcloud point_cloud_map;

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
    };
}

#endif

