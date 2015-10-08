/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

#ifndef D2R
#define D2R M_PI/180.00 /** Convert degree to radian **/
#endif
#ifndef R2D
#define R2D 180.00/M_PI /** Convert radian to degree **/
#endif

#define DEBUG_PRINTS 1
#define DEBUG_EXECUTION_TIME 1

using namespace sam;

/** Process model when accumulating delta poses **/
WMTKState processModel (const WMTKState &state,  const Eigen::Vector3d &linear_velocity, const Eigen::Vector3d &angular_velocity, const double delta_t)
{
    WMTKState s2; /** Propagated state */

    /** Update rotation rate **/
    s2.angvelo = angular_velocity;

    /** Apply Rotation **/
    ::sam::vec3 scaled_axis = state.angvelo * delta_t;
    s2.orient = state.orient * sam::SO3::exp(scaled_axis);

    /** Update the velocity (position rate) **/
    s2.velo = state.orient * linear_velocity;

    /** Apply Translation **/
    s2.pos = state.pos + (state.velo * delta_t);

    return s2;
};



Task::Task(std::string const& name)
    : TaskBase(name)
{
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine)
{
    /******************************/
    /*** Control Flow Variables ***/
    /******************************/
    initFilter = false;

    /**************************/
    /** Input port variables **/
    /**************************/
    this->delta_pose.invalidate();

    //this->pcl_point_cloud_in.reset(new PCLPointCloud);


}

Task::~Task()
{
    //this->pcl_point_cloud_in.reset();
}

void Task::delta_pose_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &delta_pose_samples_sample)
{
    if(!initFilter)
    {
        /***************************/
        /** Filter Initialization **/
        /***************************/

        Eigen::Affine3d tf; /** Transformer transformation **/

        /** Get the transformation **/
        if (_navigation_frame.value().compare(_world_frame.value()) == 0)
        {
            tf.setIdentity();
        }
        else if (!_navigation2world.get(ts, tf, false))
        {
           RTT::log(RTT::Fatal)<<"[SAM FATAL ERROR]  No transformation provided."<<RTT::endlog();
           return;
        }

        #ifdef DEBUG_PRINTS
        RTT::log(RTT::Warning)<<"[SAM POSE_SAMPLES] - Initializing Filter..."<<RTT::endlog();
        #endif

        /** Initialization of the back-end **/
        this->initialization(tf);

        /** Set the delta_pose **/
        this->delta_pose = delta_pose_samples_sample;

        #ifdef DEBUG_PRINTS
        RTT::log(RTT::Warning)<<"[DONE]\n";
        #endif

        initFilter = true;
    }

    const double predict_delta_t = delta_pose_samples_sample.time.toSeconds() - this->delta_pose.time.toSeconds();
    #ifdef DEBUG_PRINTS
    RTT::log(RTT::Warning)<<"[SAM POSE_SAMPLES] Received new samples at "<<delta_pose_samples_sample.time.toString()<<RTT::endlog();
    RTT::log(RTT::Warning)<<"[SAM POSE_SAMPLES] delta_t: "<<predict_delta_t<<RTT::endlog();
    #endif

    /** A new sample arrived to the input port **/
    this->delta_pose = delta_pose_samples_sample;

    /********************/
    /** Filter Predict **/
    /********************/
    #ifdef DEBUG_EXECUTION_TIME
    clock_t start = clock();
    #endif

    /** Process Model Uncertainty **/
    UKF::cov cov_process; cov_process.setZero();
    MTK::subblock (cov_process, &WMTKState::velo, &WMTKState::velo) = this->delta_pose.cov_velocity;
    MTK::subblock (cov_process, &WMTKState::angvelo, &WMTKState::angvelo) = this->delta_pose.cov_angular_velocity;

    /** Predict the filter state **/
    filter->predict(boost::bind(processModel, _1 ,
                            static_cast<const Eigen::Vector3d>(this->delta_pose.velocity),
                            static_cast<const Eigen::Vector3d>(this->delta_pose.angular_velocity),
                            predict_delta_t),
                            cov_process);

    /** Current delta segment displacement (previous - current pose) in last
     * pose frame **/
    double current_segment = (this->last_state.orient.inverse()*(this->last_state.pos - this->filter->mu().pos)).norm();

    std::cout<<"CURRENT SEGMENT: "<<current_segment;
    /** Set new nodes in envire SAM **/
    if(current_segment > _distance_segment.value())
    {
        std::cout<<" IS BIGGER THAN "<<_distance_segment.value()<<"\n";
        this->updateESAM();

        if (this->esam->currentPoseId().compare("x2") == 0)
        {
            std::cout<<"FACTOR-GRAPH!!!\n";
            this->esam->printFactorGraph("\nFACTOR GRAPH\n");
            std::cout<<"OPTIMIZE!!!\n";
            this->esam->optimize();
            this->esam->printMarginals();
        }
    }

    #ifdef DEBUG_EXECUTION_TIME
    clock_t end = clock();
    double cpu_time_used = ((double) (end - start)) / CLOCKS_PER_SEC;
    this->info.predict_execution_time = base::Time::fromMicroseconds(cpu_time_used*1000000.00);
    #endif

    this->outputPortSamples(this->delta_pose.time);
}

void Task::point_cloud_samplesTransformerCallback(const base::Time &ts, const ::base::samples::Pointcloud &point_cloud_samples_sample)
{
    ::base::samples::Pointcloud base_point_cloud_in;
    Eigen::Affine3d tf; /** Transformer transformation **/

    /** Get the transformation **/
    if (_sensor_frame.value().compare(_body_frame.value()) == 0)
    {
        tf.setIdentity();
    }
    else if (!_sensor2body.get(ts, tf, false))
    {
       RTT::log(RTT::Fatal)<<"[SAM FATAL ERROR]  No transformation provided."<<RTT::endlog();
       return;
    }

    #ifdef DEBUG_PRINTS
    RTT::log(RTT::Warning)<<"[SAM POINT_CLOUD_SAMPLES] Received new samples at "<<point_cloud_samples_sample.time.toString()<<RTT::endlog();
    #endif

    /** Get the new point clouds **/
    base_point_cloud_in = point_cloud_samples_sample;

    /** Transform the point cloud in body frame **/
    if (_sensor_frame.value().compare(_body_frame.value()) != 0)
    {
        //::sam::transformPointCloud(base_point_cloud_in, tf);
    }

    /** Convert to pcl point clouds **/

}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    if (! TaskBase::configureHook())
        return false;

    /***********************/
    /** Configure Values  **/
    /***********************/

    initFilter = false;

    /** Output port **/
    pose_out.invalidate();
    pose_out.sourceFrame = _localization_source_frame.value();

    /** Relative Frame to port out the samples **/
    pose_out.targetFrame = _world_frame.value();

    /***********************/
    /** Info and Warnings **/
    /***********************/
    RTT::log(RTT::Warning)<<"[SAM TASK] DESIRED TARGET FRAME IS: "<<pose_out.targetFrame<<RTT::endlog();
    RTT::log(RTT::Warning)<<"[SAM TASK] ERROR PER DISTANCE TRAVELED: "<<_error_per_distance_traveled.value() *100<<"%"<<RTT::endlog();

    return true;
}
bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;
    return true;
}
void Task::updateHook()
{
    TaskBase::updateHook();
}
void Task::errorHook()
{
    TaskBase::errorHook();
}
void Task::stopHook()
{
    TaskBase::stopHook();
}
void Task::cleanupHook()
{
    TaskBase::cleanupHook();

    /** Reset prediction filter **/
    this->filter.reset();

    /** Reset envire_sam **/
    this->esam.reset();
}

void Task::initialization(Eigen::Affine3d &tf)
{
    /** The filter vector state variables for the navigation quantities **/
    WMTKState statek_0;

    /** Set the current pose to initialize the filter structure **/
    statek_0.pos = tf.translation(); //!Initial position
    statek_0.orient = Eigen::Quaternion<double>(tf.rotation());

    /** Set the initial velocities in the state vector **/
    statek_0.velo.setZero(); //!Initial linear velocity
    statek_0.angvelo.setZero(); //!Initial angular velocity

    /******************************/
    /** Initialize the Back-End  **/
    /******************************/

    /** Initial covariance matrix **/
    UKF::cov P0; /** Initial P(0) for the state **/
    P0.setZero();
    MTK::setDiagonal (P0, &WMTKState::pos, 1e-06);
    MTK::setDiagonal (P0, &WMTKState::orient, 1e-06);
    MTK::setDiagonal (P0, &WMTKState::velo, 1e-10);
    MTK::setDiagonal (P0, &WMTKState::angvelo, 1e-10);

    /** Initialization for UKF **/
    this->initUKF(statek_0, P0);

    /** Initialization for ESAM **/
    base::TransformWithCovariance pose(statek_0.pos, statek_0.orient, P0.block<6,6>(0,0));
    this->initESAM(pose);

    /** Last state is init state **/
    this->last_state = statek_0;

    return;
}

void Task::initUKF(WMTKState &statek, UKF::cov &statek_cov)
{
    /** Create the filter **/
    this->filter.reset (new UKF (statek, statek_cov));

    #ifdef DEBUG_PRINTS
    WMTKState state = filter->mu();
    RTT::log(RTT::Warning)<< RTT::endlog();
    RTT::log(RTT::Warning)<<"[SAM INIT] State P0|0 is of size " <<filter->sigma().rows()<<" x "<<filter->sigma().cols()<< RTT::endlog();
    RTT::log(RTT::Warning)<<"[SAM INIT] State P0|0:\n"<<filter->sigma()<< RTT::endlog();
    RTT::log(RTT::Warning)<<"[SAM INIT] state:\n"<<state<< RTT::endlog();
    RTT::log(RTT::Warning)<<"[SAM INIT] position:\n"<<state.pos<< RTT::endlog();
    RTT::log(RTT::Warning)<<"[SAM INIT] orientation Roll: "<< base::getRoll(state.orient)*R2D
        <<" Pitch: "<< base::getPitch(state.orient)*R2D<<" Yaw: "<< base::getYaw(state.orient)*R2D<< RTT::endlog();
    RTT::log(RTT::Warning)<<"[SAM INIT] velocity:\n"<<state.velo<<"\n";
    RTT::log(RTT::Warning)<<"[SAM INIT] angular velocity:\n"<<state.angvelo<<"\n";
    RTT::log(RTT::Warning)<< RTT::endlog();
    #endif

    return;
}

void Task::updateESAM()
{
    /** Reset the UKF **/
    ::base::Pose current_delta_pose;
    ::base::Vector6d cov_current_delta_pose;
    this->resetUKF(current_delta_pose, cov_current_delta_pose);

    /** Set a new Factor in ESAM **/
    this->esam->addDeltaPoseFactor(this->delta_pose.time, current_delta_pose, cov_current_delta_pose);

    /** Set a new Value in ESAM **/
    std::string frame_id = this->esam->currentPoseId();
    std::cout<<"[SAM] CURRENT POSE ID: "<<frame_id<<"\n";
    std::cout<<"[SAM] CURRENT DELTA POSITION:\n"<<current_delta_pose.position<<"\n";
    std::cout<<"[SAM] CURRENT DELTA ORIENTATION ROLL: "<< base::getRoll(current_delta_pose.orientation)*R2D
        <<" PITCH: "<< base::getPitch(current_delta_pose.orientation)*R2D<<" YAW: "<< base::getYaw(current_delta_pose.orientation)*R2D<<std::endl;
    std::cout<<"[SAM] CURRENT DELTA COVARIANCE:\n"<<cov_current_delta_pose<<"\n";

    ::base::TransformWithCovariance current_pose_with_cov;
    current_pose_with_cov.translation = this->last_state.pos;
    current_pose_with_cov.orientation = this->last_state.orient;
    current_pose_with_cov.cov = this->last_state_cov.block<6,6>(0,0);
    this->esam->insertValue(frame_id, current_pose_with_cov);

    std::cout<<"********************************************\n";
    std::cout<<"[SAM] CURRENT POSITION:\n"<<current_pose_with_cov.translation<<"\n";
    std::cout<<"[SAM] CURRENT ORIENTATION ROLL: "<< base::getRoll(current_pose_with_cov.orientation)*R2D
        <<" PITCH: "<< base::getPitch(current_pose_with_cov.orientation)*R2D<<" YAW: "<< base::getYaw(current_pose_with_cov.orientation)*R2D<<std::endl;
    std::cout<<"[SAM] CURRENT COVARIANCE:\n"<<current_pose_with_cov.cov<<"\n";

    this->esam->graphViz("esam_graph.dot");

    return;
}

void Task::resetUKF(::base::Pose &current_delta_pose, ::base::Vector6d &cov_current_delta_pose)
{

    /** Compute the delta pose since last time we reset the filter **/
    current_delta_pose.position = this->filter->mu().pos - this->last_state.pos;// Delta position origin frame
    current_delta_pose.position = this->last_state.orient.inverse() * current_delta_pose.position;// Delta position in last frame
    current_delta_pose.orientation = this->last_state.orient.inverse() * this->filter->mu().orient;// Delta orientation

    /** Compute the delta covariance since last time we reset the filter **/
    /** The covariance is reset every step, then tehre is not need to compute
     * the difference between last and current **/
    ::base::Matrix3d cov_delta_position, cov_delta_orientation;
    cov_delta_position = this->filter->sigma().block<3,3>(0,0) - this->last_state_cov.block<3,3>(0,0);// Delta position covariance
    cov_delta_position = this->last_state.orient.inverse().toRotationMatrix() * cov_delta_position * this->last_state.orient.inverse().toRotationMatrix().transpose();// Delta covariance in last frame
    cov_delta_orientation = this->filter->sigma().block<3,3>(3,3) - this->last_state_cov.block<3,3>(3,3);// Delta orientation covariance
    cov_current_delta_pose << cov_delta_position.diagonal(), cov_delta_orientation.diagonal();

    /** Update the last state **/
    this->last_state = this->filter->mu();
    this->last_state_cov = this->filter->sigma();

    /** Reset covariance matrix **/
    UKF::cov P(UKF::cov::Zero());
    MTK::setDiagonal (P, &WMTKState::pos, 1e-06);
    MTK::setDiagonal (P, &WMTKState::orient, 1e-06);
    MTK::setDiagonal (P, &WMTKState::velo, 1e-10);
    MTK::setDiagonal (P, &WMTKState::angvelo, 1e-10);

    /** Remove the filter **/
    this->filter.reset();

    /** Create and reset a new filter **/
    this->initUKF(this->last_state, P);

    return;
}

void Task::initESAM(base::TransformWithCovariance &tf_cov)
{
    /*********************************************/
    /** Initialize the Back-End Graph Optimizer **/
    /*********************************************/

    /** ESAM constructor set the prior factor **/
    this->esam.reset(new envire::sam::ESAM(tf_cov, 'x', 'l'));

    /** Set the initial pose value **/
    this->esam->addPoseValue(tf_cov);

    /** Graph Viz export **/
    this->esam->graphViz("esam_graph.dot");

    return;
}


void Task::outputPortSamples(const base::Time &timestamp)
{
    WMTKState statek = this->filter->mu();

    this->pose_out.time = timestamp;
    this->pose_out.position = statek.pos;
    this->pose_out.cov_position = this->filter->sigma().block<3,3>(0,0);
    this->pose_out.orientation = statek.orient;
    this->pose_out.cov_orientation = this->filter->sigma().block<3,3>(3,3);
    this->pose_out.velocity = statek.velo;
    this->pose_out.cov_velocity =  this->filter->sigma().block<3,3>(6,6);
    this->pose_out.angular_velocity = statek.angvelo;
    this->pose_out.cov_angular_velocity =  this->filter->sigma().block<3,3>(9,9);
    _pose_samples_out.write(pose_out);

    if (_output_debug.value())
    {
        if (_output_debug.value())
        {
            this->info.time = timestamp;
            _task_info_out.write(info);
        }
    }

    return;
}

