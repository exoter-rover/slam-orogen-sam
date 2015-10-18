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
        //RTT::log(RTT::Warning)<<"[SAM POSE_SAMPLES] - Initializing Filter..."<<RTT::endlog();
        #endif

        /** Initialization of the back-end **/
        this->initialization(tf);

        /** Set the delta_pose **/
        this->delta_pose = delta_pose_samples_sample;

        /** Set the variable to compute the segments **/
        this->start_distance_segment_time = delta_pose_samples_sample.time;
        this->velocity_norm_cov = 0.00;
        this->accumulated_segment = 0.00;
        this->velocity_cov_counts = 0;

        #ifdef DEBUG_PRINTS
        //RTT::log(RTT::Warning)<<"[DONE]\n";
        #endif

        initFilter = true;
    }

    const double predict_delta_t = delta_pose_samples_sample.time.toSeconds() - this->delta_pose.time.toSeconds();
    #ifdef DEBUG_PRINTS
    //RTT::log(RTT::Warning)<<"[SAM POSE_SAMPLES] Received new samples at "<<delta_pose_samples_sample.time.toString()<<RTT::endlog();
    //RTT::log(RTT::Warning)<<"[SAM POSE_SAMPLES] delta_t: "<<predict_delta_t<<RTT::endlog();
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
    double distance_segment = std::fabs(this->filter->mu().pos.norm() - this->accumulated_segment);

    this->velocity_norm_cov = std::max(this->velocity_norm_cov, this->delta_pose.cov_velocity.norm());
    this->velocity_cov_counts++;

    //std::cout<<"ACCUMULATED SEGMENT: "<<accumulated_segment<<"\n";
    //std::cout<<"DISTANCE SEGMENT: "<<distance_segment<<"\n";
    //std::cout<<"MAXIMUM NORM COVARIANCE: "<<this->velocity_norm_cov<<"\n";

    /** Check the distance traveled **/
    if(distance_segment > _distance_segment.value())
    {
        //std::cout<<" IS BIGGER THAN "<<_distance_segment.value()<<"\n";

        /** Only add a new node in case the covariance is bigger than the
         * percentage error defined in the task property **/
        if (this->checkSegmentVelocityCov(start_distance_segment_time,
                    this->delta_pose.time, distance_segment))
        {
            /** Update ESAM **/
            this->updateESAM();
            //this->esam->printFactorGraph("\nFACTOR GRAPH!!!!\n");

            /** Compute Features Keypoints **/
            std::cout<<"[SAM] COMPUTE KEYPOINTS AND FEATURES\n";
            this->esam->computeKeypoints();

            /** Detect Landmarks **/
            std::cout<<"[SAM] DETECT LANDMARKS\n";
            this->esam->detectLandmarks(this->delta_pose.time);

            /** Write graph into GraphViz **/
            this->esam->graphViz("esam_graph.dot");

            /** Write local point cloud **/
            this->esam->currentPointCloudtoPLY("point_cloud_", true);

            /** Update Information **/
            this->updateInformation(accumulated_segment+distance_segment);
        }

        /** Reset accumulated segment, velocity covariance and the time for the next iteration **/
        this->accumulated_segment = this->filter->mu().pos.norm();
        this->velocity_norm_cov = 0.00;
        this->start_distance_segment_time = this->delta_pose.time;
        this->velocity_cov_counts = 0;
    }

//    std::cout<<"\n";

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

    if(!initFilter)
    {
        return;
    }

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
    //RTT::log(RTT::Warning)<<"[SAM POINT_CLOUD_SAMPLES] Received new samples at "<<point_cloud_samples_sample.time.toString()<<RTT::endlog();
    #endif

    /** Get the new point clouds **/
    base_point_cloud_in = point_cloud_samples_sample;

    /** Transform the point cloud in body frame (in case of needed) **/
    if (_sensor_frame.value().compare(_body_frame.value()) != 0)
    {
        this->esam->transformPointCloud(base_point_cloud_in, tf);
    }
    #ifdef DEBUG_PRINTS
    //RTT::log(RTT::Warning)<<"[SAM POINT_CLOUD_SAMPLES] Transformed Point cloud "<<RTT::endlog();
    #endif

    /** Transform point cloud to the current node frame **/
    Eigen::Affine3d delta_tf (this->filter->mu().orient);
    delta_tf.translation() = this->filter->mu().pos;
    this->esam->transformPointCloud(base_point_cloud_in, delta_tf);

    /** Push the point cloud in ESAM **/
    this->esam->pushPointCloud(base_point_cloud_in,
            static_cast<int>(_sensor_point_cloud_height.value()),
            static_cast<int>(_sensor_point_cloud_width.value()));

    /** Port out the point cloud **/
    this->outputPortPointCloud(base_point_cloud_in.time);
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

    this->initFilter = false;

    this->bfilter_config = _bfilter_config.get();
    this->outlier_config = _outlier_config.get();
    this->sift_config = _sift_config.get();
    this->feature_config = _feature_config.get();

    /** SAM Output port **/
    this->sam_pose_out.invalidate();
    this->sam_pose_out.sourceFrame = _sam_localization_source_frame.value();

    /** Relative Frame to port out the SAM pose samples **/
    this->sam_pose_out.targetFrame = _world_frame.value();

    /** Odometry Output port **/
    this->odo_pose_out.invalidate();
    this->odo_pose_out.sourceFrame = _odometry_localization_source_frame.value();

    /** Relative Frame to port out the Odometry pose samples **/
    this->odo_pose_out.targetFrame = this->sam_pose_out.sourceFrame;

    /** Reset info information **/
    this->info.accumulated_distance = 0.00;

    /***********************/
    /** Info and Warnings **/
    /***********************/
    RTT::log(RTT::Warning)<<"[SAM TASK] DESIRED TARGET FRAME IS: "<<sam_pose_out.targetFrame<<RTT::endlog();
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

    this->esam->writePlyFile(this->base_point_cloud_map_out, _output_ply.value());
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

    /******************************/
    /** Initialize the Back-End  **/
    /******************************/

    /** Initial covariance matrix **/
    UKF::cov P0; /** Initial P(0) for the state **/
    P0.setZero();
    MTK::setDiagonal (P0, &WMTKState::pos, 1e-10);
    MTK::setDiagonal (P0, &WMTKState::orient, 1e-10);
    MTK::setDiagonal (P0, &WMTKState::velo, 1e-12);
    MTK::setDiagonal (P0, &WMTKState::angvelo, 1e-12);

    /** Initialization for UKF **/
    this->initUKF(statek_0, P0);

    /** Initialization for ESAM **/
    base::TransformWithCovariance pose(tf.translation(), Eigen::Quaternion<double>(tf.rotation()), P0.block<6,6>(0,0));
    this->initESAM(pose);

    /** Accumulate pose in MTK state form **/
    this->pose_state.pos = tf.translation(); //!Initial position
    this->pose_state.orient = Eigen::Quaternion<double>(tf.rotation());

    /** Set the initial velocities in the state vector **/
    this->pose_state.velo.setZero(); //!Initial linear velocity
    this->pose_state.angvelo.setZero(); //!Initial angular velocity

    /** Accumulate pose in TWC form **/
    this->pose_with_cov.translation = tf.translation();
    this->pose_with_cov.orientation = this->pose_state.orient;

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
    std::cout<<"[SAM] RESET UKF\n";
    ::base::Pose delta_pose;
    ::base::Matrix6d cov_delta_pose;
    this->resetUKF(delta_pose, cov_delta_pose);

    /**************************/
    /** DELTA POSE IN FACTOR **/
    /**************************/

    /** Compute variance **/
    Eigen::SelfAdjointEigenSolver<base::Matrix6d> ev(cov_delta_pose);
    base::Vector6d var_delta_pose = ev.eigenvalues();

    /** Set a new Factor in ESAM **/
    this->esam->addDeltaPoseFactor(this->delta_pose.time, delta_pose, var_delta_pose);

    std::string frame_id = this->esam->currentPoseId();
    std::cout<<"[SAM] CURRENT POSE ID: "<<frame_id<<"\n";
    std::cout<<"[SAM] CURRENT DELTA POSITION:\n"<<delta_pose.position<<"\n";
    std::cout<<"[SAM] CURRENT DELTA ORIENTATION ROLL: "<< base::getRoll(delta_pose.orientation)*R2D
        <<" PITCH: "<< base::getPitch(delta_pose.orientation)*R2D<<" YAW: "<< base::getYaw(delta_pose.orientation)*R2D<<std::endl;
    std::cout<<"[SAM] CURRENT DELTA COVARIANCE:\n"<<cov_delta_pose<<"\n";

    /**************************/
    /**  POSE IN ESTIMATES   **/
    /**************************/

    /** Compute the pose estimate **/
    ::base::TransformWithCovariance delta_pose_with_cov(delta_pose.position, delta_pose.orientation, cov_delta_pose);
    this->pose_with_cov =  this->pose_with_cov * delta_pose_with_cov;

    /** Update the pose in pose state **/
    this->pose_state.pos = this->pose_with_cov.translation;
    this->pose_state.orient = static_cast<Eigen::Quaterniond>(this->pose_with_cov.orientation);

    /** Insert the pose estimated value into ESAM **/
    this->esam->insertPoseValue(frame_id, this->pose_with_cov);

    std::cout<<"********************************************\n";
    std::cout<<"[SAM] CURRENT POSITION:\n"<<this->pose_with_cov.translation<<"\n";
    std::cout<<"[SAM] CURRENT ORIENTATION ROLL: "<< base::getRoll(this->pose_with_cov.orientation)*R2D
        <<" PITCH: "<< base::getPitch(this->pose_with_cov.orientation)*R2D<<" YAW: "<< base::getYaw(this->pose_with_cov.orientation)*R2D<<std::endl;
    std::cout<<"[SAM] CURRENT COVARIANCE:\n"<<this->pose_with_cov.cov<<"\n";

    return;
}

void Task::resetUKF(::base::Pose &current_delta_pose, ::base::Matrix6d &cov_current_delta_pose)
{

    /** Compute the delta pose since last time we reset the filter **/
    current_delta_pose.position = this->filter->mu().pos;// Delta position
    current_delta_pose.orientation = this->filter->mu().orient;// Delta orientation

    /** Compute the delta covariance since last time we reset the filter **/
    cov_current_delta_pose = this->filter->sigma().block<6,6>(0,0);

    /** Update the velocity in the pose state **/
    this->pose_state.velo = this->pose_state.orient * this->filter->mu().velo;// current linear velocity
    this->pose_state.angvelo = this->pose_state.orient * this->filter->mu().angvelo;// current angular velocity

    /** Reset covariance matrix **/
    UKF::cov P(UKF::cov::Zero());
    MTK::setDiagonal (P, &WMTKState::pos, 1e-10);
    MTK::setDiagonal (P, &WMTKState::orient, 1e-10);
    MTK::setDiagonal (P, &WMTKState::velo, 1e-12);
    MTK::setDiagonal (P, &WMTKState::angvelo, 1e-12);

    /** Remove the filter **/
    this->filter.reset();

    /** Create and reset a new filter **/
    WMTKState statek;
    this->initUKF(statek, P);

    return;
}

void Task::initESAM(base::TransformWithCovariance &tf_cov)
{
    /*********************************************/
    /** Initialize the Back-End Graph Optimizer **/
    /*********************************************/

    /** ESAM constructor set the prior factor **/
    this->esam.reset(new envire::sam::ESAM(tf_cov, 'x', 'l',
                _downsample_size.value(),
                this->bfilter_config, this->outlier_config,
                this->sift_config, this->feature_config,
                _point_variance.value()));

    /** Set the initial pose value **/
    this->esam->addPoseValue(tf_cov);

    /** Graph Viz export **/
    this->esam->graphViz("esam_graph.dot");

    return;
}

bool Task::checkSegmentVelocityCov(const base::Time &start, const base::Time &end,
                                const double &segment)
{
    const float segment_velo = segment / (end - start).toSeconds();
    //this->velocity_norm_cov /= this->velocity_cov_counts;

    //std::cout<<"DELTA TIME: "<<(end-start).toSeconds()<<" [SECONDS]\n";
    //std::cout<<"SEGMENT VELOCITY: "<<segment_velo<<"\n";
    //std::cout<<"NORM COVARIANCE: "<<this->velocity_norm_cov<<"\n";
    //std::cout<<"TARGET NORM COVARIANCE: "<<(segment_velo * _error_per_distance_traveled.value())<<"\n";

    if (this->velocity_norm_cov >= (segment_velo * _error_per_distance_traveled.value()))
    {
        return true;
    }

    return false;
}

bool Task::checkSegmentCov(const double &current_segment)
{
    Eigen::Matrix3d const &cov_segment_position(this->filter->sigma().block<3,3>(0,0));

    /** Compute eigenvalues **/
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> ev(cov_segment_position);
    Eigen::Vector3d e_val = ev.eigenvalues();
    const double error = Eigen::Vector3d(e_val.array().sqrt()).norm();

    //std::cout<<"[SAM] CURRENT DELTA COVARIANCE:\n"<<cov_segment_position<<"\n";
   // std::cout<<"[SAM] CURRENT NORM ERROR: "<<error<<"\n";
   // std::cout<<"[SAM] TARGET NORM ERROR: "<<_error_per_distance_traveled.value() * current_segment<<"\n";
    if (error > _error_per_distance_traveled.value() * current_segment)
    {
        return true;
    }

    return false;
}

void Task::outputPortSamples(const base::Time &timestamp)
{
    WMTKState statek = this->filter->mu();

    /** Out port the last pose in SAM values **/
    this->sam_pose_out = this->esam->getRbsPose(this->esam->currentPoseId());
    this->sam_pose_out.sourceFrame = _sam_localization_source_frame.value();
    this->sam_pose_out.targetFrame = _world_frame.value();
    this->sam_pose_out.time = timestamp;
    this->sam_pose_out.velocity = this->sam_pose_out.orientation * statek.velo;
    base::Matrix3d rotation_in_matrix = this->sam_pose_out.orientation.toRotationMatrix();
    this->sam_pose_out.cov_velocity =  rotation_in_matrix * this->filter->sigma().block<3,3>(6,6) * rotation_in_matrix.transpose();
    this->sam_pose_out.angular_velocity = this->sam_pose_out.orientation * statek.angvelo;
    this->sam_pose_out.cov_angular_velocity =  rotation_in_matrix * this->filter->sigma().block<3,3>(9,9) * rotation_in_matrix.transpose();
    _sam_pose_samples_out.write(this->sam_pose_out);

    /** Out port the last odometry pose **/
    this->odo_pose_out.position = statek.pos;
    this->odo_pose_out.cov_position = this->filter->sigma().block<3,3>(0,0);
    this->odo_pose_out.orientation = statek.orient;
    this->odo_pose_out.cov_orientation = this->filter->sigma().block<3,3>(3,3);
    this->odo_pose_out.velocity = statek.velo;
    this->odo_pose_out.cov_velocity =  this->filter->sigma().block<3,3>(6,6);
    this->odo_pose_out.angular_velocity = statek.angvelo;
    this->odo_pose_out.cov_angular_velocity =  this->filter->sigma().block<3,3>(9,9);
    _odo_pose_samples_out.write(this->odo_pose_out);

    /** Debug information **/
    if (_output_debug.value())
    {
        if (_output_debug.value())
        {
            this->info.time = timestamp;
            this->info.pose_nodes_rbs = this->esam->getRbsPoses();
            _task_info_out.write(info);
        }
    }

    return;
}

void Task::outputPortPointCloud(const base::Time &timestamp)
{
    /** Marge all point cloud in esam **/
    this->esam->mergePointClouds(this->base_point_cloud_map_out, true);

    /** Port out the point cloud **/
    this->base_point_cloud_map_out.time = timestamp;
    _point_cloud_samples_out.write(this->base_point_cloud_map_out);

    return;
}

void Task::updateInformation(const int &current_segment)
{
    /** Update pose with cov from the last SAM result **/
    this->pose_with_cov = this->esam->getTransformPose(this->esam->currentPoseId());

    /** Update accumulated distance **/
    this->info.accumulated_distance += current_segment;

    /** Update pose correspondences for landmarks **/
    sam::PoseCorrespondence pose;
    pose.node_idx = this->esam->getPoseCorrespodences(pose.correspondences_idx);
    this->info.poses_correspondences.push_back(pose);
}
