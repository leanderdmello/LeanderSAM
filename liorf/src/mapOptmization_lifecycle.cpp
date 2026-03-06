// Copyright 2025 Avular B.V.
// All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula

#include "utility_lifecycle.h"
#include "liorf/msg/cloud_info.hpp"
#include "liorf/srv/save_map.hpp"
#include <autonomy_msgs/srv/get_state.hpp>
#include <interface_msgs/msg/point_cloud_available.hpp>

#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>

#include <gtsam/nonlinear/ISAM2.h>

#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>

#include <filesystem>

#include "Scancontext.h"

using namespace gtsam;

using symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)
using symbol_shorthand::G; // GPS pose
using symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)

/*
 * A point cloud type that has 6D pose info ([x,y,z,roll,pitch,yaw] intensity is time stamp)
 */
struct PointXYZIRPYT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY; // preferred way of adding a XYZ+padding
    float  roll;
    float  pitch;
    float  yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT(
    PointXYZIRPYT,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(float, roll, roll)(
        float,
        pitch,
        pitch)(float, yaw, yaw)(double, time, time))

typedef PointXYZIRPYT PointTypePose;

enum class SCInputType
{
    SINGLE_SCAN_FULL,
    SINGLE_SCAN_FEAT,
    MULTI_SCAN_FEAT
};

class mapOptimization : public ParamServerLifecycle
{
public:
    // gtsam
    NonlinearFactorGraph   gtSAMgraph;
    Values                 initialEstimate;
    std::unique_ptr<ISAM2> isam;
    Values                 isamCurrentEstimate;
    Eigen::MatrixXd        poseCovariance;
    ISAM2Params            parameters;

    rclcpp::Subscription<liorf::msg::CloudInfo>::SharedPtr                         subCloud;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr                   subGPS;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr              subLoop;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subInitialPose;
    rclcpp::Subscription<interface_msgs::msg::PointCloudAvailable>::SharedPtr      subMapLocation;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr        pubLaserCloudSurround;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr              pubLaserOdometryGlobal;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr              pubLaserOdometryIncremental;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr        pubKeyPoses;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr                  pubPath;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr        pubHistoryKeyFrames;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr        pubIcpKeyFrames;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr        pubRecentKeyFrames;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr        pubRecentKeyFrame;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr        pubCloudRegisteredRaw;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubLoopConstraintEdge;
    rclcpp::Publisher<liorf::msg::CloudInfo>::SharedPtr                pubSLAMInfo;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr              pubGpsOdom;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr        pubLocalizationMap;

    rclcpp::Service<liorf::srv::SaveMap>::SharedPtr         srvSaveMap;
    rclcpp::Client<autonomy_msgs::srv::GetState>::SharedPtr slam_mode_client_;

    std::deque<nav_msgs::msg::Odometry> gpsQueue;
    liorf::msg::CloudInfo               cloudInfo;

    vector<pcl::PointCloud<PointType>::Ptr> surfCloudKeyFrames;

    pcl::PointCloud<PointType>::Ptr     cloudKeyPoses3D;
    pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D;
    pcl::PointCloud<PointType>::Ptr     copy_cloudKeyPoses3D;
    pcl::PointCloud<PointTypePose>::Ptr copy_cloudKeyPoses6D;

    pcl::PointCloud<PointType>::Ptr laserCloudSurfLast; // surf feature set from odoOptimization
    pcl::PointCloud<PointType>::Ptr
        laserCloudSurfLastDS; // downsampled surf feature set from odoOptimization

    pcl::PointCloud<PointType>::Ptr laserCloudOri;
    pcl::PointCloud<PointType>::Ptr coeffSel;

    std::vector<PointType> laserCloudOriSurfVec; // surf point holder for parallel computation
    std::vector<PointType> coeffSelSurfVec;
    std::vector<bool>      laserCloudOriSurfFlag;

    map<int, pair<pcl::PointCloud<PointType>, pcl::PointCloud<PointType>>> laserCloudMapContainer;
    pcl::PointCloud<PointType>::Ptr                                        laserCloudSurfFromMap;
    pcl::PointCloud<PointType>::Ptr                                        laserCloudSurfFromMapDS;
    pcl::PointCloud<PointType>::Ptr                                        localizationMap;
    pcl::PointCloud<PointType>::Ptr                                        localizationMapDS;

    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap;

    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurroundingKeyPoses;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeHistoryKeyPoses;

    pcl::VoxelGrid<PointType> downSizeFilterSurf;
    pcl::VoxelGrid<PointType> downSizeFilterLocalMapSurf;
    pcl::VoxelGrid<PointType> downSizeFilterICP;
    pcl::VoxelGrid<PointType>
        downSizeFilterSurroundingKeyPoses; // for surrounding key poses of scan-to-map optimization

    rclcpp::Time timeLaserInfoStamp;
    double       timeLaserInfoCur;

    float transformTobeMapped[6];

    std::mutex mtx;
    std::mutex mtxLoopInfo;

    bool    isDegenerate;
    cv::Mat matP;

    int laserCloudSurfFromMapDSNum;
    int laserCloudSurfLastDSNum;
    int localizationMapDSNum;

    bool                   aLoopIsClosed;
    map<int, int>          loopIndexContainer; // from new to old
    vector<pair<int, int>> loopIndexQueue;
    vector<gtsam::Pose3>   loopPoseQueue;
    // vector<gtsam::noiseModel::Diagonal::shared_ptr> loopNoiseQueue;
    vector<gtsam::SharedNoiseModel>         loopNoiseQueue;
    deque<std_msgs::msg::Float64MultiArray> loopInfoVec;

    nav_msgs::msg::Path globalPath;

    Eigen::Affine3f transPointAssociateToMap;
    Eigen::Affine3f incrementalOdometryAffineFront;
    Eigen::Affine3f incrementalOdometryAffineBack;

    GeographicLib::LocalCartesian gps_trans_;

    // scancontext loop closure
    std::unique_ptr<SCManager> scManager_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> br;

    std::atomic<bool>            threadRunning_;
    std::unique_ptr<std::thread> loopClosureThread_;
    std::unique_ptr<std::thread> visualizeMapThread_;

    bool lastIncreOdomPubFlag;
    bool lastImuPreTransAvailable;

    bool  systemInitialized;
    bool  hasInitializePose;
    bool  foundMapAllignment;
    float initializePose[6];
    float localizationPose[6];
    float localizationFitness;

    std::string slamModeCache_;
    bool        waitingForSlamMode_;

    mapOptimization(const rclcpp::NodeOptions &options)
        : ParamServerLifecycle("liorf_mapOptimization", options)
    {
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
        rclcpp_lifecycle::State const &state)
    {
        // Downsize parameters
        downSizeFilterSurf.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize,
                                       mappingSurfLeafSize);
        downSizeFilterLocalMapSurf.setLeafSize(surroundingKeyframeMapLeafSize,
                                               surroundingKeyframeMapLeafSize,
                                               surroundingKeyframeMapLeafSize);
        downSizeFilterICP.setLeafSize(loopClosureICPSurfLeafSize, loopClosureICPSurfLeafSize,
                                      loopClosureICPSurfLeafSize);
        downSizeFilterSurroundingKeyPoses.setLeafSize(
            surroundingKeyframeDensity, surroundingKeyframeDensity,
            surroundingKeyframeDensity); // for surrounding key poses of scan-to-map optimization

        // Setup params
        isDegenerate             = false;
        aLoopIsClosed            = false;
        lastIncreOdomPubFlag     = false;
        lastImuPreTransAvailable = false;
        hasInitializePose        = false;
        systemInitialized        = false;

        slamModeCache_      = "unknown";
        waitingForSlamMode_ = false;

        threadRunning_.store(false);

        laserCloudSurfFromMapDSNum = 0;
        laserCloudSurfLastDSNum    = 0;
        localizationMapDSNum       = 0;

        // ISAM parameters
        parameters.relinearizeThreshold = 0.1;
        parameters.relinearizeSkip      = 1;

        br = std::make_unique<tf2_ros::TransformBroadcaster>(this);

        rclcpp::QoS subMapLocationQoS = rclcpp::QoS(1);
        subMapLocationQoS.durability(rclcpp::DurabilityPolicy::TransientLocal);
        subMapLocation = create_subscription<interface_msgs::msg::PointCloudAvailable>(
            "/interface/mapping_point_cloud_available", subMapLocationQoS,
            std::bind(&mapOptimization::mapLocationHandler, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(), "%s configured", this->get_name());
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
        rclcpp_lifecycle::State const &state)
    {
        isam = std::make_unique<ISAM2>(parameters);
        allocateMemory();

        // Initialize parameters
        laserCloudOriSurfVec.resize(N_SCAN * Horizon_SCAN);
        coeffSelSurfVec.resize(N_SCAN * Horizon_SCAN);
        laserCloudOriSurfFlag.resize(N_SCAN * Horizon_SCAN);

        std::fill(laserCloudOriSurfFlag.begin(), laserCloudOriSurfFlag.end(), false);

        for(int i = 0; i < 6; ++i)
        {
            transformTobeMapped[i] = 0;
        }

        matP = cv::Mat(6, 6, CV_32F, cv::Scalar::all(0));

        // Publishers
        pubLaserCloudSurround = create_publisher<sensor_msgs::msg::PointCloud2>(
            "mapping/map_global", QosPolicy(history_policy, reliability_policy));
        pubLaserOdometryGlobal = create_publisher<nav_msgs::msg::Odometry>(
            "mapping/odometry", QosPolicy(history_policy, reliability_policy));
        pubLaserOdometryIncremental = create_publisher<nav_msgs::msg::Odometry>(
            "mapping/odometry_incremental", QosPolicy(history_policy, reliability_policy));
        pubPath = create_publisher<nav_msgs::msg::Path>(
            "mapping/path", QosPolicy(history_policy, reliability_policy));

        // Debug Publisher
        if(debugFlag)
        {
            pubKeyPoses = create_publisher<sensor_msgs::msg::PointCloud2>(
                "mapping/trajectory", QosPolicy(history_policy, reliability_policy));
            pubHistoryKeyFrames = create_publisher<sensor_msgs::msg::PointCloud2>(
                "mapping/icp_loop_closure_history_cloud",
                QosPolicy(history_policy, reliability_policy));
            pubIcpKeyFrames = create_publisher<sensor_msgs::msg::PointCloud2>(
                "mapping/icp_loop_closure_corrected_cloud",
                QosPolicy(history_policy, reliability_policy));
            pubLoopConstraintEdge = create_publisher<visualization_msgs::msg::MarkerArray>(
                "mapping/loop_closure_constraints", QosPolicy(history_policy, reliability_policy));
            pubRecentKeyFrames = create_publisher<sensor_msgs::msg::PointCloud2>(
                "mapping/map_local", QosPolicy(history_policy, reliability_policy));
            pubRecentKeyFrame = create_publisher<sensor_msgs::msg::PointCloud2>(
                "mapping/cloud_registered", QosPolicy(history_policy, reliability_policy));
            pubCloudRegisteredRaw = create_publisher<sensor_msgs::msg::PointCloud2>(
                "mapping/cloud_registered_raw", QosPolicy(history_policy, reliability_policy));
            pubSLAMInfo = create_publisher<liorf::msg::CloudInfo>(
                "mapping/slam_info", QosPolicy(history_policy, reliability_policy));
            pubGpsOdom = create_publisher<nav_msgs::msg::Odometry>(
                "mapping/gps_odom", QosPolicy(history_policy, reliability_policy));
            pubLocalizationMap = create_publisher<sensor_msgs::msg::PointCloud2>(
                "localization/map", QosPolicy(history_policy, reliability_policy));
        }

        // Subscribers
        subCloud = create_subscription<liorf::msg::CloudInfo>(
            "deskew/cloud_info", QosPolicy(history_policy, reliability_policy),
            std::bind(&mapOptimization::laserCloudInfoHandler, this, std::placeholders::_1));
        subGPS = create_subscription<sensor_msgs::msg::NavSatFix>(
            gpsTopic, QosPolicy(history_policy, reliability_policy),
            std::bind(&mapOptimization::gpsHandler, this, std::placeholders::_1));
        subLoop = create_subscription<std_msgs::msg::Float64MultiArray>(
            "lio_loop/loop_closure_detection", QosPolicy(history_policy, reliability_policy),
            std::bind(&mapOptimization::loopInfoHandler, this, std::placeholders::_1));
        subInitialPose = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", QosPolicy(history_policy, reliability_policy),
            std::bind(&mapOptimization::initialPoseHandler, this, std::placeholders::_1));

        // Services
        srvSaveMap = create_service<liorf::srv::SaveMap>(
            "save_map", std::bind(&mapOptimization::saveMapService, this, std::placeholders::_1,
                                  std::placeholders::_2));
        slam_mode_client_ = create_client<autonomy_msgs::srv::GetState>("get_state");

        // Threads
        threadRunning_.store(true);
        loopClosureThread_ =
            std::make_unique<std::thread>(&mapOptimization::loopClosureThread, this);
        visualizeMapThread_ =
            std::make_unique<std::thread>(&mapOptimization::visualizeGlobalMapThread, this);

        LifecycleNode::on_activate(state);
        RCLCPP_INFO(get_logger(), "%s activated", this->get_name());
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
        rclcpp_lifecycle::State const &state)
    {
        resetInternalState();
        LifecycleNode::on_deactivate(state);
        RCLCPP_INFO(get_logger(), "%s deactivated", this->get_name());
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(
        rclcpp_lifecycle::State const &state)
    {
        resetInternalState();
        RCLCPP_INFO(get_logger(), "%s cleaned up", this->get_name());
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(
        rclcpp_lifecycle::State const &state)
    {
        resetInternalState();
        RCLCPP_INFO(get_logger(), "%s shut down", this->get_name());
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    void allocateMemory()
    {
        cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
        cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());
        copy_cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
        copy_cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());

        kdtreeSurroundingKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());
        kdtreeHistoryKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());

        laserCloudSurfLast.reset(
            new pcl::PointCloud<PointType>()); // surf feature set from odoOptimization
        laserCloudSurfLastDS.reset(
            new pcl::PointCloud<PointType>()); // downsampled surf featuer set from odoOptimization

        laserCloudOri.reset(new pcl::PointCloud<PointType>());
        coeffSel.reset(new pcl::PointCloud<PointType>());

        laserCloudSurfFromMap.reset(new pcl::PointCloud<PointType>());
        laserCloudSurfFromMapDS.reset(new pcl::PointCloud<PointType>());

        localizationMap.reset(new pcl::PointCloud<PointType>());
        localizationMapDS.reset(new pcl::PointCloud<PointType>());

        kdtreeSurfFromMap.reset(new pcl::KdTreeFLANN<PointType>());

        scManager_ = std::make_unique<SCManager>();
    }

    void resetInternalState()
    {
        // Stop threads
        threadRunning_.store(false);
        if(loopClosureThread_ && loopClosureThread_->joinable())
        {
            RCLCPP_INFO(get_logger(), "Joining LoopClosure thread");
            loopClosureThread_->join();
            loopClosureThread_.reset();
        }
        if(visualizeMapThread_ && visualizeMapThread_->joinable())
        {
            RCLCPP_INFO(get_logger(), "Joining VisualizeMap thread");
            visualizeMapThread_->join();
            visualizeMapThread_.reset();
        }

        // Clear subscribers and publishers
        subCloud.reset();
        subGPS.reset();
        subLoop.reset();
        subInitialPose.reset();
        pubKeyPoses.reset();
        pubLaserCloudSurround.reset();
        pubLaserOdometryGlobal.reset();
        pubLaserOdometryIncremental.reset();
        pubPath.reset();
        pubHistoryKeyFrames.reset();
        pubIcpKeyFrames.reset();
        pubLoopConstraintEdge.reset();
        pubRecentKeyFrames.reset();
        pubRecentKeyFrame.reset();
        pubCloudRegisteredRaw.reset();
        pubSLAMInfo.reset();
        pubGpsOdom.reset();
        pubLocalizationMap.reset();

        // Clear queues
        gpsQueue.clear();
        loopIndexQueue.clear();
        loopPoseQueue.clear();
        loopNoiseQueue.clear();

        // Clear storage objects
        laserCloudMapContainer.clear();
        loopIndexContainer.clear();
        loopInfoVec.clear();
        globalPath.poses.clear();
        surfCloudKeyFrames.clear();

        // Clear GTSAM
        gtSAMgraph.resize(0);
        initialEstimate.clear();

        // Clear ISAM
        isam.reset();

        // Clear flags
        lastIncreOdomPubFlag     = false;
        lastImuPreTransAvailable = false;
        isDegenerate             = false;
        aLoopIsClosed            = false;
        hasInitializePose        = false;
        systemInitialized        = false;

        slam_mode_client_.reset();
        slamModeCache_      = "unknown";
        waitingForSlamMode_ = false;
    }

    void laserCloudInfoHandler(const liorf::msg::CloudInfo::SharedPtr msgIn)
    {
        // extract time stamp
        timeLaserInfoStamp = msgIn->header.stamp;
        timeLaserInfoCur   = ROS_TIME(msgIn->header.stamp);

        // extract info and feature cloud
        cloudInfo = *msgIn;
        pcl::fromROSMsg(msgIn->cloud_deskewed, *laserCloudSurfLast);

        // TODO
        // ......
        // remapping
        // ......
        // END

        std::lock_guard<std::mutex> lock(mtx);

        static double timeLastProcessing   = -1;
        static double timeLastICPAlignment = -1;
        if(timeLaserInfoCur - timeLastProcessing >= mappingProcessInterval)
        {
            timeLastProcessing = timeLaserInfoCur;

            if(!systemInitialized)
            {
                getSlamMode();

                if(initialPoseAtOrigin && slamModeCache_ == "mapping")
                {
                    initializePose[0] = 0.0;
                    initializePose[1] = 0.0;
                    initializePose[2] = 0.0;
                    initializePose[3] = 0.0;
                    initializePose[4] = 0.0;
                    initializePose[5] = 0.0;

                    hasInitializePose = true;
                }

                if(!hasInitializePose)
                {
                    RCLCPP_WARN(get_logger(), "An initial pose is required to continue.");
                    return;
                }

                if(!systemInitialize())
                {
                    RCLCPP_WARN(get_logger(), "System initialization failed!");
                    return;
                }
            }

            updateInitialGuess();

            extractSurroundingKeyFrames();

            downsampleCurrentScan();

            scan2MapOptimization();

            // Periodic ICP alignment for localization mode
            foundMapAllignment = false;
            if(slamModeCache_ == "localization")
            {
                if(timeLaserInfoCur - timeLastICPAlignment >= icpAlignmentInterval)
                {
                    timeLastICPAlignment = timeLaserInfoCur;
                    if(performICPAlignment())
                        foundMapAllignment = true;
                }
            }

            saveKeyFramesAndFactor();

            correctPoses();

            publishOdometry();

            publishFrames();
        }
    }

    void gpsHandler(const sensor_msgs::msg::NavSatFix::SharedPtr gpsMsg)
    {
        if(gpsMsg->status.status != 0)
            return;

        Eigen::Vector3d trans_local_;
        static bool     first_gps = false;
        if(!first_gps)
        {
            first_gps = true;
            gps_trans_.Reset(gpsMsg->latitude, gpsMsg->longitude, gpsMsg->altitude);
        }

        gps_trans_.Forward(gpsMsg->latitude, gpsMsg->longitude, gpsMsg->altitude, trans_local_[0],
                           trans_local_[1], trans_local_[2]);

        nav_msgs::msg::Odometry gps_odom;
        gps_odom.header               = gpsMsg->header;
        gps_odom.header.frame_id      = "map";
        gps_odom.pose.pose.position.x = trans_local_[0];
        gps_odom.pose.pose.position.y = trans_local_[1];
        gps_odom.pose.pose.position.z = trans_local_[2];
        tf2::Quaternion quat_tf;
        quat_tf.setRPY(0.0, 0.0, 0.0);
        geometry_msgs::msg::Quaternion quat_msg;
        tf2::convert(quat_tf, quat_msg);
        gps_odom.pose.pose.orientation = quat_msg;
        if(debugFlag)
        {
            pubGpsOdom->publish(gps_odom);
        }
        size_t gpsQueue_before = gpsQueue.size();
        gpsQueue.push_back(gps_odom);
    }

    void pointAssociateToMap(PointType const *const pi, PointType *const po)
    {
        po->x = transPointAssociateToMap(0, 0) * pi->x + transPointAssociateToMap(0, 1) * pi->y +
                transPointAssociateToMap(0, 2) * pi->z + transPointAssociateToMap(0, 3);
        po->y = transPointAssociateToMap(1, 0) * pi->x + transPointAssociateToMap(1, 1) * pi->y +
                transPointAssociateToMap(1, 2) * pi->z + transPointAssociateToMap(1, 3);
        po->z = transPointAssociateToMap(2, 0) * pi->x + transPointAssociateToMap(2, 1) * pi->y +
                transPointAssociateToMap(2, 2) * pi->z + transPointAssociateToMap(2, 3);
        po->intensity = pi->intensity;
    }

    pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn,
                                                        PointTypePose                  *transformIn)
    {
        pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

        int cloudSize = cloudIn->size();
        cloudOut->resize(cloudSize);

        Eigen::Affine3f transCur =
            pcl::getTransformation(transformIn->x, transformIn->y, transformIn->z,
                                   transformIn->roll, transformIn->pitch, transformIn->yaw);

#pragma omp parallel for num_threads(numberOfCores)
        for(int i = 0; i < cloudSize; ++i)
        {
            const auto &pointFrom = cloudIn->points[i];
            cloudOut->points[i].x = transCur(0, 0) * pointFrom.x + transCur(0, 1) * pointFrom.y +
                                    transCur(0, 2) * pointFrom.z + transCur(0, 3);
            cloudOut->points[i].y = transCur(1, 0) * pointFrom.x + transCur(1, 1) * pointFrom.y +
                                    transCur(1, 2) * pointFrom.z + transCur(1, 3);
            cloudOut->points[i].z = transCur(2, 0) * pointFrom.x + transCur(2, 1) * pointFrom.y +
                                    transCur(2, 2) * pointFrom.z + transCur(2, 3);
            cloudOut->points[i].intensity = pointFrom.intensity;
        }
        return cloudOut;
    }

    gtsam::Pose3 pclPointTogtsamPose3(PointTypePose thisPoint)
    {
        return gtsam::Pose3(
            gtsam::Rot3::RzRyRx(double(thisPoint.roll), double(thisPoint.pitch),
                                double(thisPoint.yaw)),
            gtsam::Point3(double(thisPoint.x), double(thisPoint.y), double(thisPoint.z)));
    }

    gtsam::Pose3 trans2gtsamPose(float transformIn[])
    {
        return gtsam::Pose3(gtsam::Rot3::RzRyRx(transformIn[0], transformIn[1], transformIn[2]),
                            gtsam::Point3(transformIn[3], transformIn[4], transformIn[5]));
    }

    Eigen::Affine3f pclPointToAffine3f(PointTypePose thisPoint)
    {
        return pcl::getTransformation(thisPoint.x, thisPoint.y, thisPoint.z, thisPoint.roll,
                                      thisPoint.pitch, thisPoint.yaw);
    }

    Eigen::Affine3f trans2Affine3f(float transformIn[])
    {
        return pcl::getTransformation(transformIn[3], transformIn[4], transformIn[5],
                                      transformIn[0], transformIn[1], transformIn[2]);
    }

    PointTypePose trans2PointTypePose(float transformIn[])
    {
        PointTypePose thisPose6D;
        thisPose6D.x     = transformIn[3];
        thisPose6D.y     = transformIn[4];
        thisPose6D.z     = transformIn[5];
        thisPose6D.roll  = transformIn[0];
        thisPose6D.pitch = transformIn[1];
        thisPose6D.yaw   = transformIn[2];
        return thisPose6D;
    }


    bool saveMapService(const std::shared_ptr<liorf::srv::SaveMap::Request> req,
                        std::shared_ptr<liorf::srv::SaveMap::Response>      res)
    {
        string saveMapDirectory;

        cout << "****************************************************" << endl;
        cout << "Saving map to pcd files ..." << endl;
        if(req->destination.empty())
            saveMapDirectory = std::getenv("HOME") + savePCDDirectory;
        else
            saveMapDirectory = std::getenv("HOME") + req->destination;
        cout << "Save destination: " << saveMapDirectory << endl;
        // create directory and remove old files;
        int unused = system((std::string("exec rm -r ") + saveMapDirectory).c_str());
        unused     = system((std::string("mkdir -p ") + saveMapDirectory).c_str());
        // save key frame transformations
        pcl::io::savePCDFileBinary(saveMapDirectory + "/trajectory.pcd", *cloudKeyPoses3D);
        pcl::io::savePCDFileBinary(saveMapDirectory + "/transformations.pcd", *cloudKeyPoses6D);
        // extract global point cloud map

        pcl::PointCloud<PointType>::Ptr globalSurfCloud(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalSurfCloudDS(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalMapCloud(new pcl::PointCloud<PointType>());
        for(int i = 0; i < (int)cloudKeyPoses3D->size(); i++)
        {
            *globalSurfCloud +=
                *transformPointCloud(surfCloudKeyFrames[i], &cloudKeyPoses6D->points[i]);
            cout << "\r" << std::flush << "Processing feature cloud " << i << " of "
                 << cloudKeyPoses6D->size() << " ...";
        }

        if(req->resolution != 0)
        {
            cout << "\n\nSave resolution: " << req->resolution << endl;
            // down-sample and save surf cloud
            downSizeFilterSurf.setInputCloud(globalSurfCloud);
            downSizeFilterSurf.setLeafSize(req->resolution, req->resolution, req->resolution);
            downSizeFilterSurf.filter(*globalSurfCloudDS);
            pcl::io::savePCDFileBinary(saveMapDirectory + "/SurfMap.pcd", *globalSurfCloudDS);
        }
        else
        {
            // save surf cloud
            pcl::io::savePCDFileBinary(saveMapDirectory + "/SurfMap.pcd", *globalSurfCloud);
        }

        // save global point cloud map
        *globalMapCloud += *globalSurfCloud;

        int ret = pcl::io::savePCDFileBinary(saveMapDirectory + "/GlobalMap.pcd", *globalMapCloud);
        res->success = ret == 0;

        downSizeFilterSurf.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize,
                                       mappingSurfLeafSize);

        cout << "****************************************************" << endl;
        cout << "Saving map to pcd files completed\n" << endl;

        return true;
    }

    void visualizeGlobalMapThread()
    {
        rclcpp::Rate rate(0.2);
        while(rclcpp::ok() and threadRunning_)
        {
            rate.sleep();
            publishGlobalMap();
        }

        if(savePCD == false)
            return;

        std::shared_ptr<liorf::srv::SaveMap::Request> req =
            std::make_unique<liorf::srv::SaveMap::Request>();
        std::shared_ptr<liorf::srv::SaveMap::Response> res =
            std::make_unique<liorf::srv::SaveMap::Response>();

        if(!saveMapService(req, res))
        {
            cout << "Fail to save map" << endl;
        }
    }

    void publishGlobalMap()
    {
        if(pubLaserCloudSurround->get_subscription_count() == 0)
            return;

        if(cloudKeyPoses3D->points.empty() == true)
            return;

        pcl::KdTreeFLANN<PointType>::Ptr kdtreeGlobalMap(new pcl::KdTreeFLANN<PointType>());
        ;
        pcl::PointCloud<PointType>::Ptr globalMapKeyPoses(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalMapKeyPosesDS(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalMapKeyFrames(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalMapKeyFramesDS(new pcl::PointCloud<PointType>());

        // kd-tree to find near key frames to visualize
        std::vector<int>   pointSearchIndGlobalMap;
        std::vector<float> pointSearchSqDisGlobalMap;
        // search near key frames to visualize
        mtx.lock();
        kdtreeGlobalMap->setInputCloud(cloudKeyPoses3D);
        kdtreeGlobalMap->radiusSearch(cloudKeyPoses3D->back(), globalMapVisualizationSearchRadius,
                                      pointSearchIndGlobalMap, pointSearchSqDisGlobalMap, 0);
        mtx.unlock();

        for(int i = 0; i < (int)pointSearchIndGlobalMap.size(); ++i)
            globalMapKeyPoses->push_back(cloudKeyPoses3D->points[pointSearchIndGlobalMap[i]]);
        // downsample near selected key frames
        pcl::VoxelGrid<PointType> downSizeFilterGlobalMapKeyPoses; // for global map visualization
        downSizeFilterGlobalMapKeyPoses.setLeafSize(
            globalMapVisualizationPoseDensity, globalMapVisualizationPoseDensity,
            globalMapVisualizationPoseDensity); // for global map visualization
        downSizeFilterGlobalMapKeyPoses.setInputCloud(globalMapKeyPoses);
        downSizeFilterGlobalMapKeyPoses.filter(*globalMapKeyPosesDS);
        for(auto &pt : globalMapKeyPosesDS->points)
        {
            kdtreeGlobalMap->nearestKSearch(pt, 1, pointSearchIndGlobalMap,
                                            pointSearchSqDisGlobalMap);
            pt.intensity = cloudKeyPoses3D->points[pointSearchIndGlobalMap[0]].intensity;
        }

        // extract visualized and downsampled key frames
        for(int i = 0; i < (int)globalMapKeyPosesDS->size(); ++i)
        {
            if(common_lib_->pointDistance(globalMapKeyPosesDS->points[i], cloudKeyPoses3D->back()) >
               globalMapVisualizationSearchRadius)
                continue;
            int thisKeyInd = (int)globalMapKeyPosesDS->points[i].intensity;
            *globalMapKeyFrames += *transformPointCloud(surfCloudKeyFrames[thisKeyInd],
                                                        &cloudKeyPoses6D->points[thisKeyInd]);
        }
        // downsample visualized points
        pcl::VoxelGrid<PointType> downSizeFilterGlobalMapKeyFrames; // for global map visualization
        downSizeFilterGlobalMapKeyFrames.setLeafSize(
            globalMapVisualizationLeafSize, globalMapVisualizationLeafSize,
            globalMapVisualizationLeafSize); // for global map visualization
        downSizeFilterGlobalMapKeyFrames.setInputCloud(globalMapKeyFrames);
        downSizeFilterGlobalMapKeyFrames.filter(*globalMapKeyFramesDS);
        publishCloud(pubLaserCloudSurround, globalMapKeyFramesDS, timeLaserInfoStamp,
                     odometryFrame);
    }


    void loopClosureThread()
    {
        if(loopClosureEnableFlag == false)
            return;

        rclcpp::Rate rate(loopClosureFrequency);
        while(rclcpp::ok() and threadRunning_)
        {
            rate.sleep();
            performRSLoopClosure();
            performSCLoopClosure();
            visualizeLoopClosure();
        }
    }

    void loopInfoHandler(const std_msgs::msg::Float64MultiArray::SharedPtr loopMsg)
    {
        std::lock_guard<std::mutex> lock(mtxLoopInfo);
        if(loopMsg->data.size() != 2)
            return;

        size_t loopInfoVec_before = loopInfoVec.size();
        loopInfoVec.push_back(*loopMsg);

        while(loopInfoVec.size() > 5)
            loopInfoVec.pop_front();
    }

    void performRSLoopClosure()
    {
        if(cloudKeyPoses3D->points.empty() == true)
            return;

        mtx.lock();
        *copy_cloudKeyPoses3D = *cloudKeyPoses3D;
        *copy_cloudKeyPoses6D = *cloudKeyPoses6D;
        mtx.unlock();

        // find keys
        int loopKeyCur;
        int loopKeyPre;
        if(detectLoopClosureExternal(&loopKeyCur, &loopKeyPre) == false)
            if(detectLoopClosureDistance(&loopKeyCur, &loopKeyPre) == false)
                return;

        // extract cloud
        pcl::PointCloud<PointType>::Ptr cureKeyframeCloud(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr prevKeyframeCloud(new pcl::PointCloud<PointType>());
        {
            loopFindNearKeyframes(cureKeyframeCloud, loopKeyCur, 0, -1);
            loopFindNearKeyframes(prevKeyframeCloud, loopKeyPre, historyKeyframeSearchNum, -1);
            if(cureKeyframeCloud->size() < 300 || prevKeyframeCloud->size() < 1000)
                return;
            if(debugFlag && pubHistoryKeyFrames->get_subscription_count() != 0)
                publishCloud(pubHistoryKeyFrames, prevKeyframeCloud, timeLaserInfoStamp,
                             odometryFrame);
        }

        // ICP Settings
        pcl::IterativeClosestPoint<PointType, PointType> icp;
        icp.setMaxCorrespondenceDistance(historyKeyframeSearchRadius * 2);
        icp.setMaximumIterations(100);
        icp.setTransformationEpsilon(1e-6);
        icp.setEuclideanFitnessEpsilon(1e-6);
        icp.setRANSACIterations(0);

        // Align clouds
        icp.setInputSource(cureKeyframeCloud);
        icp.setInputTarget(prevKeyframeCloud);
        pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
        icp.align(*unused_result);

        if(icp.hasConverged() == false || icp.getFitnessScore() > historyKeyframeFitnessScore)
            return;

        // publish corrected cloud
        if(debugFlag && pubIcpKeyFrames->get_subscription_count() != 0)
        {
            pcl::PointCloud<PointType>::Ptr closed_cloud(new pcl::PointCloud<PointType>());
            pcl::transformPointCloud(*cureKeyframeCloud, *closed_cloud,
                                     icp.getFinalTransformation());
            publishCloud(pubIcpKeyFrames, closed_cloud, timeLaserInfoStamp, odometryFrame);
        }

        // Get pose transformation
        float           x, y, z, roll, pitch, yaw;
        Eigen::Affine3f correctionLidarFrame;
        correctionLidarFrame = icp.getFinalTransformation();
        // transform from world origin to wrong pose
        Eigen::Affine3f tWrong = pclPointToAffine3f(copy_cloudKeyPoses6D->points[loopKeyCur]);
        // transform from world origin to corrected pose
        Eigen::Affine3f tCorrect =
            correctionLidarFrame *
            tWrong; // pre-multiplying -> successive rotation about a fixed frame
        pcl::getTranslationAndEulerAngles(tCorrect, x, y, z, roll, pitch, yaw);
        gtsam::Pose3  poseFrom = Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));
        gtsam::Pose3  poseTo   = pclPointTogtsamPose3(copy_cloudKeyPoses6D->points[loopKeyPre]);
        gtsam::Vector Vector6(6);
        float         noiseScore = icp.getFitnessScore();
        Vector6 << noiseScore, noiseScore, noiseScore, noiseScore, noiseScore, noiseScore;
        noiseModel::Diagonal::shared_ptr constraintNoise = noiseModel::Diagonal::Variances(Vector6);

        // Add pose constraint
        mtx.lock();
        size_t loopIndexQueue_before = loopIndexQueue.size();
        size_t loopPoseQueue_before  = loopPoseQueue.size();
        size_t loopNoiseQueue_before = loopNoiseQueue.size();

        loopIndexQueue.push_back(make_pair(loopKeyCur, loopKeyPre));
        loopPoseQueue.push_back(poseFrom.between(poseTo));
        loopNoiseQueue.push_back(constraintNoise);

        mtx.unlock();

        // add loop constriant
        loopIndexContainer[loopKeyCur] = loopKeyPre;
    }

    // copy from sc-lio-sam
    void performSCLoopClosure()
    {
        if(cloudKeyPoses3D->points.empty() == true)
            return;

        mtx.lock();
        *copy_cloudKeyPoses3D = *cloudKeyPoses3D;
        *copy_cloudKeyPoses6D = *cloudKeyPoses6D;
        mtx.unlock();

        // find keys
        // first: nn index, second: yaw diff
        auto detectResult = scManager_->detectLoopClosureID();
        int  loopKeyCur   = copy_cloudKeyPoses3D->size() - 1;
        ;
        int   loopKeyPre = detectResult.first;
        float yawDiffRad =
            detectResult.second; // not use for v1 (because pcl icp withi initial somthing wrong...)
        if(loopKeyPre == -1)
            return;

        auto it = loopIndexContainer.find(loopKeyCur);
        if(it != loopIndexContainer.end())
            return;

        // std::cout << "SC loop found! between " << loopKeyCur << " and " << loopKeyPre << "." <<
        // std::endl; // giseop

        // extract cloud
        pcl::PointCloud<PointType>::Ptr cureKeyframeCloud(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr prevKeyframeCloud(new pcl::PointCloud<PointType>());
        {
            int base_key = 0;
            loopFindNearKeyframes(cureKeyframeCloud, loopKeyCur, 0, base_key); // giseop
            loopFindNearKeyframes(prevKeyframeCloud, loopKeyPre, historyKeyframeSearchNum,
                                  base_key); // giseop

            if(cureKeyframeCloud->size() < 300 || prevKeyframeCloud->size() < 1000)
                return;
            if(debugFlag && pubHistoryKeyFrames->get_subscription_count() != 0)
                publishCloud(pubHistoryKeyFrames, prevKeyframeCloud, timeLaserInfoStamp,
                             odometryFrame);
        }

        // ICP Settings
        pcl::IterativeClosestPoint<PointType, PointType> icp;
        icp.setMaxCorrespondenceDistance(historyKeyframeSearchRadius * 2);
        icp.setMaximumIterations(100);
        icp.setTransformationEpsilon(1e-6);
        icp.setEuclideanFitnessEpsilon(1e-6);
        icp.setRANSACIterations(0);

        // Align clouds
        icp.setInputSource(cureKeyframeCloud);
        icp.setInputTarget(prevKeyframeCloud);
        pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
        icp.align(*unused_result);

        if(icp.hasConverged() == false || icp.getFitnessScore() > historyKeyframeFitnessScore)
            return;

        // publish corrected cloud
        if(debugFlag && pubIcpKeyFrames->get_subscription_count() != 0)
        {
            pcl::PointCloud<PointType>::Ptr closed_cloud(new pcl::PointCloud<PointType>());
            pcl::transformPointCloud(*cureKeyframeCloud, *closed_cloud,
                                     icp.getFinalTransformation());
            publishCloud(pubIcpKeyFrames, closed_cloud, timeLaserInfoStamp, odometryFrame);
        }

        // Get pose transformation
        float           x, y, z, roll, pitch, yaw;
        Eigen::Affine3f correctionLidarFrame;
        correctionLidarFrame = icp.getFinalTransformation();

        // // transform from world origin to wrong pose
        // Eigen::Affine3f tWrong = pclPointToAffine3f(copy_cloudKeyPoses6D->points[loopKeyCur]);
        // // transform from world origin to corrected pose
        // Eigen::Affine3f tCorrect = correctionLidarFrame * tWrong;// pre-multiplying -> successive
        // rotation about a fixed frame pcl::getTranslationAndEulerAngles (tCorrect, x, y, z, roll,
        // pitch, yaw); gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y,
        // z)); gtsam::Pose3 poseTo =
        // pclPointTogtsamPose3(copy_cloudKeyPoses6D->points[loopKeyPre]);

        // gtsam::Vector Vector6(6);
        // float noiseScore = icp.getFitnessScore();
        // Vector6 << noiseScore, noiseScore, noiseScore, noiseScore, noiseScore, noiseScore;
        // noiseModel::Diagonal::shared_ptr constraintNoise =
        // noiseModel::Diagonal::Variances(Vector6);

        // giseop
        pcl::getTranslationAndEulerAngles(correctionLidarFrame, x, y, z, roll, pitch, yaw);
        gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));
        gtsam::Pose3 poseTo   = Pose3(Rot3::RzRyRx(0.0, 0.0, 0.0), Point3(0.0, 0.0, 0.0));

        // giseop, robust kernel for a SC loop
        float         robustNoiseScore = 0.5; // constant is ok...
        gtsam::Vector robustNoiseVector6(6);
        robustNoiseVector6 << robustNoiseScore, robustNoiseScore, robustNoiseScore,
            robustNoiseScore, robustNoiseScore, robustNoiseScore;
        noiseModel::Base::shared_ptr robustConstraintNoise;
        robustConstraintNoise = gtsam::noiseModel::Robust::Create(
            gtsam::noiseModel::mEstimator::Cauchy::Create(
                1), // optional: replacing Cauchy by DCS or GemanMcClure, but with a good front-end
                    // loop detector, Cauchy is empirically enough.
            gtsam::noiseModel::Diagonal::Variances(
                robustNoiseVector6)); // - checked it works. but with robust kernel, map
                                      // modification may be delayed (i.e,. requires more
                                      // true-positive loop factors)

        // Add pose constraint
        mtx.lock();
        size_t loopIndexQueue_before2 = loopIndexQueue.size();
        size_t loopPoseQueue_before2  = loopPoseQueue.size();
        size_t loopNoiseQueue_before2 = loopNoiseQueue.size();

        loopIndexQueue.push_back(make_pair(loopKeyCur, loopKeyPre));
        loopPoseQueue.push_back(poseFrom.between(poseTo));
        loopNoiseQueue.push_back(robustConstraintNoise);

        mtx.unlock();

        // add loop constriant
        loopIndexContainer[loopKeyCur] = loopKeyPre;
    }

    bool detectLoopClosureDistance(int *latestID, int *closestID)
    {
        int loopKeyCur = copy_cloudKeyPoses3D->size() - 1;
        int loopKeyPre = -1;

        // check loop constraint added before
        auto it = loopIndexContainer.find(loopKeyCur);
        if(it != loopIndexContainer.end())
            return false;

        // find the closest history key frame
        std::vector<int>   pointSearchIndLoop;
        std::vector<float> pointSearchSqDisLoop;
        kdtreeHistoryKeyPoses->setInputCloud(copy_cloudKeyPoses3D);
        kdtreeHistoryKeyPoses->radiusSearch(copy_cloudKeyPoses3D->back(),
                                            historyKeyframeSearchRadius, pointSearchIndLoop,
                                            pointSearchSqDisLoop, 0);

        for(int i = 0; i < (int)pointSearchIndLoop.size(); ++i)
        {
            int id = pointSearchIndLoop[i];
            if(abs(copy_cloudKeyPoses6D->points[id].time - timeLaserInfoCur) >
               historyKeyframeSearchTimeDiff)
            {
                loopKeyPre = id;
                break;
            }
        }

        if(loopKeyPre == -1 || loopKeyCur == loopKeyPre)
            return false;

        *latestID  = loopKeyCur;
        *closestID = loopKeyPre;

        return true;
    }

    bool detectLoopClosureExternal(int *latestID, int *closestID)
    {
        // this function is not used yet, please ignore it
        int loopKeyCur = -1;
        int loopKeyPre = -1;

        std::lock_guard<std::mutex> lock(mtxLoopInfo);
        if(loopInfoVec.empty())
            return false;

        double loopTimeCur = loopInfoVec.front().data[0];
        double loopTimePre = loopInfoVec.front().data[1];
        loopInfoVec.pop_front();

        if(abs(loopTimeCur - loopTimePre) < historyKeyframeSearchTimeDiff)
            return false;

        int cloudSize = copy_cloudKeyPoses6D->size();
        if(cloudSize < 2)
            return false;

        // latest key
        loopKeyCur = cloudSize - 1;
        for(int i = cloudSize - 1; i >= 0; --i)
        {
            if(copy_cloudKeyPoses6D->points[i].time >= loopTimeCur)
                loopKeyCur = round(copy_cloudKeyPoses6D->points[i].intensity);
            else
                break;
        }

        // previous key
        loopKeyPre = 0;
        for(int i = 0; i < cloudSize; ++i)
        {
            if(copy_cloudKeyPoses6D->points[i].time <= loopTimePre)
                loopKeyPre = round(copy_cloudKeyPoses6D->points[i].intensity);
            else
                break;
        }

        if(loopKeyCur == loopKeyPre)
            return false;

        auto it = loopIndexContainer.find(loopKeyCur);
        if(it != loopIndexContainer.end())
            return false;

        *latestID  = loopKeyCur;
        *closestID = loopKeyPre;

        return true;
    }

    void loopFindNearKeyframes(pcl::PointCloud<PointType>::Ptr &nearKeyframes,
                               const int                       &key,
                               const int                       &searchNum,
                               const int                       &loop_index)
    {
        // extract near keyframes
        nearKeyframes->clear();
        int cloudSize = copy_cloudKeyPoses6D->size();
        for(int i = -searchNum; i <= searchNum; ++i)
        {
            int keyNear = key + i;
            if(keyNear < 0 || keyNear >= cloudSize)
                continue;

            int select_loop_index = (loop_index != -1) ? loop_index : key + i;
            *nearKeyframes += *transformPointCloud(
                surfCloudKeyFrames[keyNear], &copy_cloudKeyPoses6D->points[select_loop_index]);
        }

        if(nearKeyframes->empty())
            return;

        // downsample near keyframes
        pcl::PointCloud<PointType>::Ptr cloud_temp(new pcl::PointCloud<PointType>());
        downSizeFilterICP.setInputCloud(nearKeyframes);
        downSizeFilterICP.filter(*cloud_temp);
        *nearKeyframes = *cloud_temp;
    }

    void visualizeLoopClosure()
    {
        if(loopIndexContainer.empty())
            return;

        visualization_msgs::msg::MarkerArray markerArray;
        // loop nodes
        visualization_msgs::msg::Marker markerNode;
        markerNode.header.frame_id    = odometryFrame;
        markerNode.header.stamp       = timeLaserInfoStamp;
        markerNode.action             = visualization_msgs::msg::Marker::ADD;
        markerNode.type               = visualization_msgs::msg::Marker::SPHERE_LIST;
        markerNode.ns                 = "loop_nodes";
        markerNode.id                 = 0;
        markerNode.pose.orientation.w = 1;
        markerNode.scale.x            = 0.3;
        markerNode.scale.y            = 0.3;
        markerNode.scale.z            = 0.3;
        markerNode.color.r            = 0;
        markerNode.color.g            = 0.8;
        markerNode.color.b            = 1;
        markerNode.color.a            = 1;
        // loop edges
        visualization_msgs::msg::Marker markerEdge;
        markerEdge.header.frame_id    = odometryFrame;
        markerEdge.header.stamp       = timeLaserInfoStamp;
        markerEdge.action             = visualization_msgs::msg::Marker::ADD;
        markerEdge.type               = visualization_msgs::msg::Marker::LINE_LIST;
        markerEdge.ns                 = "loop_edges";
        markerEdge.id                 = 1;
        markerEdge.pose.orientation.w = 1;
        markerEdge.scale.x            = 0.1;
        markerEdge.color.r            = 0.9;
        markerEdge.color.g            = 0.9;
        markerEdge.color.b            = 0;
        markerEdge.color.a            = 1;

        for(auto it = loopIndexContainer.begin(); it != loopIndexContainer.end(); ++it)
        {
            int                       key_cur = it->first;
            int                       key_pre = it->second;
            geometry_msgs::msg::Point p;
            p.x = copy_cloudKeyPoses6D->points[key_cur].x;
            p.y = copy_cloudKeyPoses6D->points[key_cur].y;
            p.z = copy_cloudKeyPoses6D->points[key_cur].z;
            markerNode.points.push_back(p);
            markerEdge.points.push_back(p);
            p.x = copy_cloudKeyPoses6D->points[key_pre].x;
            p.y = copy_cloudKeyPoses6D->points[key_pre].y;
            p.z = copy_cloudKeyPoses6D->points[key_pre].z;
            markerNode.points.push_back(p);
            markerEdge.points.push_back(p);
        }

        markerArray.markers.push_back(markerNode);
        markerArray.markers.push_back(markerEdge);
        if(debugFlag)
        {
            pubLoopConstraintEdge->publish(markerArray);
        }
    }

    void updateInitialGuess()
    {
        // save current transformation before any processing
        incrementalOdometryAffineFront = trans2Affine3f(transformTobeMapped);

        static Eigen::Affine3f lastImuTransformation;
        // initialization
        if(cloudKeyPoses3D->points.empty())
        {
            lastImuTransformation =
                pcl::getTransformation(0, 0, 0, cloudInfo.imurollinit, cloudInfo.imupitchinit,
                                       cloudInfo.imuyawinit); // save imu before return;
            return;
        }

        // use imu pre-integration estimation for pose guess
        static Eigen::Affine3f lastImuPreTransformation;
        if(cloudInfo.odomavailable == true)
        {
            Eigen::Affine3f transBack = pcl::getTransformation(
                cloudInfo.initialguessx, cloudInfo.initialguessy, cloudInfo.initialguessz,
                cloudInfo.initialguessroll, cloudInfo.initialguesspitch, cloudInfo.initialguessyaw);
            if(lastImuPreTransAvailable == false)
            {
                lastImuPreTransformation = transBack;
                lastImuPreTransAvailable = true;
            }
            else
            {
                Eigen::Affine3f transIncre = lastImuPreTransformation.inverse() * transBack;
                Eigen::Affine3f transTobe  = trans2Affine3f(transformTobeMapped);
                Eigen::Affine3f transFinal = transTobe * transIncre;
                pcl::getTranslationAndEulerAngles(transFinal, transformTobeMapped[3],
                                                  transformTobeMapped[4], transformTobeMapped[5],
                                                  transformTobeMapped[0], transformTobeMapped[1],
                                                  transformTobeMapped[2]);

                lastImuPreTransformation = transBack;

                lastImuTransformation =
                    pcl::getTransformation(0, 0, 0, cloudInfo.imurollinit, cloudInfo.imupitchinit,
                                           cloudInfo.imuyawinit); // save imu before return;
                return;
            }
        }

        // use imu incremental estimation for pose guess (only rotation)
        if(cloudInfo.imuavailable == true && imuType)
        {
            Eigen::Affine3f transBack = pcl::getTransformation(
                0, 0, 0, cloudInfo.imurollinit, cloudInfo.imupitchinit, cloudInfo.imuyawinit);
            Eigen::Affine3f transIncre = lastImuTransformation.inverse() * transBack;

            Eigen::Affine3f transTobe  = trans2Affine3f(transformTobeMapped);
            Eigen::Affine3f transFinal = transTobe * transIncre;
            pcl::getTranslationAndEulerAngles(
                transFinal, transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5],
                transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);

            lastImuTransformation =
                pcl::getTransformation(0, 0, 0, cloudInfo.imurollinit, cloudInfo.imupitchinit,
                                       cloudInfo.imuyawinit); // save imu before return;
            return;
        }
    }

    void extractForLoopClosure()
    {
        pcl::PointCloud<PointType>::Ptr cloudToExtract(new pcl::PointCloud<PointType>());
        int                             numPoses = cloudKeyPoses3D->size();
        for(int i = numPoses - 1; i >= 0; --i)
        {
            if((int)cloudToExtract->size() <= surroundingKeyframeSize)
                cloudToExtract->push_back(cloudKeyPoses3D->points[i]);
            else
                break;
        }

        extractCloud(cloudToExtract);
    }

    void extractNearby()
    {
        pcl::PointCloud<PointType>::Ptr surroundingKeyPoses(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr surroundingKeyPosesDS(new pcl::PointCloud<PointType>());
        std::vector<int>                pointSearchInd;
        std::vector<float>              pointSearchSqDis;

        // extract all the nearby key poses and downsample them
        kdtreeSurroundingKeyPoses->setInputCloud(cloudKeyPoses3D); // create kd-tree
        kdtreeSurroundingKeyPoses->radiusSearch(cloudKeyPoses3D->back(),
                                                (double)surroundingKeyframeSearchRadius,
                                                pointSearchInd, pointSearchSqDis);
        for(int i = 0; i < (int)pointSearchInd.size(); ++i)
        {
            int id = pointSearchInd[i];
            surroundingKeyPoses->push_back(cloudKeyPoses3D->points[id]);
        }

        downSizeFilterSurroundingKeyPoses.setInputCloud(surroundingKeyPoses);
        downSizeFilterSurroundingKeyPoses.filter(*surroundingKeyPosesDS);
        for(auto &pt : surroundingKeyPosesDS->points)
        {
            kdtreeSurroundingKeyPoses->nearestKSearch(pt, 1, pointSearchInd, pointSearchSqDis);
            pt.intensity = cloudKeyPoses3D->points[pointSearchInd[0]].intensity;
        }

        // also extract some latest key frames in case the robot rotates in one position
        int numPoses = cloudKeyPoses3D->size();
        for(int i = numPoses - 1; i >= 0; --i)
        {
            if(timeLaserInfoCur - cloudKeyPoses6D->points[i].time < 10.0)
                surroundingKeyPosesDS->push_back(cloudKeyPoses3D->points[i]);
            else
                break;
        }

        extractCloud(surroundingKeyPosesDS);
    }

    void extractCloud(pcl::PointCloud<PointType>::Ptr cloudToExtract)
    {
        // fuse the map
        laserCloudSurfFromMap->clear();
        for(int i = 0; i < (int)cloudToExtract->size(); ++i)
        {
            if(common_lib_->pointDistance(cloudToExtract->points[i], cloudKeyPoses3D->back()) >
               surroundingKeyframeSearchRadius)
                continue;

            int thisKeyInd = (int)cloudToExtract->points[i].intensity;
            if(laserCloudMapContainer.find(thisKeyInd) != laserCloudMapContainer.end())
            {
                // transformed cloud available
                *laserCloudSurfFromMap += laserCloudMapContainer[thisKeyInd].second;
            }
            else
            {
                // transformed cloud not available
                pcl::PointCloud<PointType> laserCloudCornerTemp;
                pcl::PointCloud<PointType> laserCloudSurfTemp = *transformPointCloud(
                    surfCloudKeyFrames[thisKeyInd], &cloudKeyPoses6D->points[thisKeyInd]);
                *laserCloudSurfFromMap += laserCloudSurfTemp;
                laserCloudMapContainer[thisKeyInd] =
                    make_pair(laserCloudCornerTemp, laserCloudSurfTemp);
            }
        }

        // Downsample the surrounding surf key frames (or map)
        downSizeFilterLocalMapSurf.setInputCloud(laserCloudSurfFromMap);
        downSizeFilterLocalMapSurf.filter(*laserCloudSurfFromMapDS);
        laserCloudSurfFromMapDSNum = laserCloudSurfFromMapDS->size();

        // clear map cache if too large
        if(laserCloudMapContainer.size() > 1000)
            laserCloudMapContainer.clear();
    }

    void extractSurroundingKeyFrames()
    {
        if(cloudKeyPoses3D->points.empty() == true)
            return;

        // if (loopClosureEnableFlag == true)
        // {
        //     extractForLoopClosure();
        // } else {
        //     extractNearby();
        // }

        extractNearby();
    }

    void downsampleCurrentScan()
    {
        laserCloudSurfLastDS->clear();
        downSizeFilterSurf.setInputCloud(laserCloudSurfLast);
        downSizeFilterSurf.filter(*laserCloudSurfLastDS);
        laserCloudSurfLastDSNum = laserCloudSurfLastDS->size();
    }

    void updatePointAssociateToMap()
    {
        transPointAssociateToMap = trans2Affine3f(transformTobeMapped);
    }

    void surfOptimization()
    {
        updatePointAssociateToMap();

#pragma omp parallel for num_threads(numberOfCores)
        for(int i = 0; i < laserCloudSurfLastDSNum; i++)
        {
            PointType          pointOri, pointSel, coeff;
            std::vector<int>   pointSearchInd;
            std::vector<float> pointSearchSqDis;

            pointOri = laserCloudSurfLastDS->points[i];
            pointAssociateToMap(&pointOri, &pointSel);
            kdtreeSurfFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

            Eigen::Matrix<float, 5, 3> matA0;
            Eigen::Matrix<float, 5, 1> matB0;
            Eigen::Vector3f            matX0;

            matA0.setZero();
            matB0.fill(-1);
            matX0.setZero();

            if(pointSearchSqDis[4] < 1.0)
            {
                for(int j = 0; j < 5; j++)
                {
                    matA0(j, 0) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].x;
                    matA0(j, 1) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].y;
                    matA0(j, 2) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].z;
                }

                matX0 = matA0.colPivHouseholderQr().solve(matB0);

                float pa = matX0(0, 0);
                float pb = matX0(1, 0);
                float pc = matX0(2, 0);
                float pd = 1;

                float ps = sqrt(pa * pa + pb * pb + pc * pc);
                pa /= ps;
                pb /= ps;
                pc /= ps;
                pd /= ps;

                bool planeValid = true;
                for(int j = 0; j < 5; j++)
                {
                    if(fabs(pa * laserCloudSurfFromMapDS->points[pointSearchInd[j]].x +
                            pb * laserCloudSurfFromMapDS->points[pointSearchInd[j]].y +
                            pc * laserCloudSurfFromMapDS->points[pointSearchInd[j]].z + pd) > 0.2)
                    {
                        planeValid = false;
                        break;
                    }
                }

                if(planeValid)
                {
                    float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;

                    float s = 1 - 0.9 * fabs(pd2) /
                                      sqrt(sqrt(pointOri.x * pointOri.x + pointOri.y * pointOri.y +
                                                pointOri.z * pointOri.z));

                    coeff.x         = s * pa;
                    coeff.y         = s * pb;
                    coeff.z         = s * pc;
                    coeff.intensity = s * pd2;

                    if(s > 0.1)
                    {
                        laserCloudOriSurfVec[i]  = pointOri;
                        coeffSelSurfVec[i]       = coeff;
                        laserCloudOriSurfFlag[i] = true;
                    }
                }
            }
        }
    }

    void combineOptimizationCoeffs()
    {
        // combine surf coeffs
        for(int i = 0; i < laserCloudSurfLastDSNum; ++i)
        {
            if(laserCloudOriSurfFlag[i] == true)
            {
                laserCloudOri->push_back(laserCloudOriSurfVec[i]);
                coeffSel->push_back(coeffSelSurfVec[i]);
            }
        }
        // reset flag for next iteration
        std::fill(laserCloudOriSurfFlag.begin(), laserCloudOriSurfFlag.end(), false);
    }

    bool LMOptimization(int iterCount)
    {
        // This optimization is from the original loam_velodyne by Ji Zhang, need to cope with
        // coordinate transformation lidar <- camera      ---     camera <- lidar x = z ---     x =
        // y y = x                ---     y = z z = y                ---     z = x roll = yaw ---
        // roll = pitch pitch = roll         ---     pitch = yaw yaw = pitch          ---     yaw =
        // roll

        // lidar -> camera
        float srx = sin(transformTobeMapped[2]);
        float crx = cos(transformTobeMapped[2]);
        float sry = sin(transformTobeMapped[1]);
        float cry = cos(transformTobeMapped[1]);
        float srz = sin(transformTobeMapped[0]);
        float crz = cos(transformTobeMapped[0]);

        int laserCloudSelNum = laserCloudOri->size();
        if(laserCloudSelNum < 50)
        {
            return false;
        }

        cv::Mat matA(laserCloudSelNum, 6, CV_32F, cv::Scalar::all(0));
        cv::Mat matAt(6, laserCloudSelNum, CV_32F, cv::Scalar::all(0));
        cv::Mat matAtA(6, 6, CV_32F, cv::Scalar::all(0));
        cv::Mat matB(laserCloudSelNum, 1, CV_32F, cv::Scalar::all(0));
        cv::Mat matAtB(6, 1, CV_32F, cv::Scalar::all(0));
        cv::Mat matX(6, 1, CV_32F, cv::Scalar::all(0));

        PointType pointOri, coeff;

        for(int i = 0; i < laserCloudSelNum; i++)
        {
            // lidar -> camera
            pointOri.x = laserCloudOri->points[i].x;
            pointOri.y = laserCloudOri->points[i].y;
            pointOri.z = laserCloudOri->points[i].z;
            // lidar -> camera
            coeff.x         = coeffSel->points[i].x;
            coeff.y         = coeffSel->points[i].y;
            coeff.z         = coeffSel->points[i].z;
            coeff.intensity = coeffSel->points[i].intensity;
            // in camera
            /*             float arx = (crx*sry*srz*pointOri.x + crx*crz*sry*pointOri.y -
               srx*sry*pointOri.z) * coeff.x
                                  + (-srx*srz*pointOri.x - crz*srx*pointOri.y - crx*pointOri.z) *
               coeff.y
                                  + (crx*cry*srz*pointOri.x + crx*cry*crz*pointOri.y -
               cry*srx*pointOri.z) * coeff.z;

                        float ary = ((cry*srx*srz - crz*sry)*pointOri.x
                                  + (sry*srz + cry*crz*srx)*pointOri.y + crx*cry*pointOri.z) *
               coeff.x
                                  + ((-cry*crz - srx*sry*srz)*pointOri.x
                                  + (cry*srz - crz*srx*sry)*pointOri.y - crx*sry*pointOri.z) *
               coeff.z;

                        float arz = ((crz*srx*sry - cry*srz)*pointOri.x +
               (-cry*crz-srx*sry*srz)*pointOri.y)*coeff.x
                                  + (crx*crz*pointOri.x - crx*srz*pointOri.y) * coeff.y
                                  + ((sry*srz + cry*crz*srx)*pointOri.x +
               (crz*sry-cry*srx*srz)*pointOri.y)*coeff.z;
                         */

            float arx = (-srx * cry * pointOri.x - (srx * sry * srz + crx * crz) * pointOri.y +
                         (crx * srz - srx * sry * crz) * pointOri.z) *
                            coeff.x +
                        (crx * cry * pointOri.x - (srx * crz - crx * sry * srz) * pointOri.y +
                         (crx * sry * crz + srx * srz) * pointOri.z) *
                            coeff.y;

            float ary =
                (-crx * sry * pointOri.x + crx * cry * srz * pointOri.y +
                 crx * cry * crz * pointOri.z) *
                    coeff.x +
                (-srx * sry * pointOri.x + srx * sry * srz * pointOri.y +
                 srx * cry * crz * pointOri.z) *
                    coeff.y +
                (-cry * pointOri.x - sry * srz * pointOri.y - sry * crz * pointOri.z) * coeff.z;

            float arz = ((crx * sry * crz + srx * srz) * pointOri.y +
                         (srx * crz - crx * sry * srz) * pointOri.z) *
                            coeff.x +
                        ((-crx * srz + srx * sry * crz) * pointOri.y +
                         (-srx * sry * srz - crx * crz) * pointOri.z) *
                            coeff.y +
                        (cry * crz * pointOri.y - cry * srz * pointOri.z) * coeff.z;

            // camera -> lidar
            matA.at<float>(i, 0) = arz;
            matA.at<float>(i, 1) = ary;
            matA.at<float>(i, 2) = arx;
            matA.at<float>(i, 3) = coeff.x;
            matA.at<float>(i, 4) = coeff.y;
            matA.at<float>(i, 5) = coeff.z;
            matB.at<float>(i, 0) = -coeff.intensity;
        }

        cv::transpose(matA, matAt);
        matAtA = matAt * matA;
        matAtB = matAt * matB;
        cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);

        if(iterCount == 0)
        {
            cv::Mat matE(1, 6, CV_32F, cv::Scalar::all(0));
            cv::Mat matV(6, 6, CV_32F, cv::Scalar::all(0));
            cv::Mat matV2(6, 6, CV_32F, cv::Scalar::all(0));

            cv::eigen(matAtA, matE, matV);
            matV.copyTo(matV2);

            isDegenerate      = false;
            float eignThre[6] = {100, 100, 100, 100, 100, 100};
            for(int i = 5; i >= 0; i--)
            {
                if(matE.at<float>(0, i) < eignThre[i])
                {
                    for(int j = 0; j < 6; j++)
                    {
                        matV2.at<float>(i, j) = 0;
                    }
                    isDegenerate = true;
                }
                else
                {
                    break;
                }
            }
            matP = matV.inv() * matV2;
        }

        if(isDegenerate)
        {
            cv::Mat matX2(6, 1, CV_32F, cv::Scalar::all(0));
            matX.copyTo(matX2);
            matX = matP * matX2;
        }

        transformTobeMapped[0] += matX.at<float>(0, 0);
        transformTobeMapped[1] += matX.at<float>(1, 0);
        transformTobeMapped[2] += matX.at<float>(2, 0);
        transformTobeMapped[3] += matX.at<float>(3, 0);
        transformTobeMapped[4] += matX.at<float>(4, 0);
        transformTobeMapped[5] += matX.at<float>(5, 0);

        float deltaR = sqrt(pow(pcl::rad2deg(matX.at<float>(0, 0)), 2) +
                            pow(pcl::rad2deg(matX.at<float>(1, 0)), 2) +
                            pow(pcl::rad2deg(matX.at<float>(2, 0)), 2));
        float deltaT =
            sqrt(pow(matX.at<float>(3, 0) * 100, 2) + pow(matX.at<float>(4, 0) * 100, 2) +
                 pow(matX.at<float>(5, 0) * 100, 2));

        if(deltaR < 0.05 && deltaT < 0.05)
        {
            return true; // converged
        }
        return false; // keep optimizing
    }

    void scan2MapOptimization()
    {
        if(cloudKeyPoses3D->points.empty())
            return;

        if(laserCloudSurfLastDSNum > 30)
        {
            kdtreeSurfFromMap->setInputCloud(laserCloudSurfFromMapDS);

            for(int iterCount = 0; iterCount < 30; iterCount++)
            {
                laserCloudOri->clear();
                coeffSel->clear();

                surfOptimization();

                combineOptimizationCoeffs();

                if(LMOptimization(iterCount) == true)
                {
                    if(debugFlag)
                    {
                        RCLCPP_INFO(get_logger(), "Scan to map optimization with %d iterations.",
                                    iterCount);
                    }
                    break;
                }

                if(iterCount >= 29)
                {
                    RCLCPP_WARN(get_logger(),
                                "Scan to map optimization reached maximum iterations.");
                }
            }

            transformUpdate();
        }
        else
        {
            RCLCPP_WARN(get_logger(), "Not enough features! Only %d planar features available.",
                        laserCloudSurfLastDSNum);
        }
    }

    void transformUpdate()
    {
        if(cloudInfo.imuavailable == true && imuType)
        {
            if(std::abs(cloudInfo.imupitchinit) < 1.4)
            {
                double          imuWeight = imuRPYWeight;
                tf2::Quaternion imuQuaternion;
                tf2::Quaternion transformQuaternion;
                double          rollMid, pitchMid, yawMid;

                // slerp roll
                transformQuaternion.setRPY(transformTobeMapped[0], 0, 0);
                imuQuaternion.setRPY(cloudInfo.imurollinit, 0, 0);
                tf2::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight))
                    .getRPY(rollMid, pitchMid, yawMid);
                transformTobeMapped[0] = rollMid;

                // slerp pitch
                transformQuaternion.setRPY(0, transformTobeMapped[1], 0);
                imuQuaternion.setRPY(0, cloudInfo.imupitchinit, 0);
                tf2::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight))
                    .getRPY(rollMid, pitchMid, yawMid);
                transformTobeMapped[1] = pitchMid;
            }
        }

        transformTobeMapped[0] =
            constraintTransformation(transformTobeMapped[0], rotation_tollerance);
        transformTobeMapped[1] =
            constraintTransformation(transformTobeMapped[1], rotation_tollerance);
        transformTobeMapped[5] = constraintTransformation(transformTobeMapped[5], z_tollerance);

        incrementalOdometryAffineBack = trans2Affine3f(transformTobeMapped);
    }

    float constraintTransformation(float value, float limit)
    {
        if(value < -limit)
            value = -limit;
        if(value > limit)
            value = limit;

        return value;
    }

    bool saveFrame()
    {
        if(cloudKeyPoses3D->points.empty())
            return true;

        Eigen::Affine3f transStart = pclPointToAffine3f(cloudKeyPoses6D->back());
        Eigen::Affine3f transFinal = pcl::getTransformation(
            transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5],
            transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
        Eigen::Affine3f transBetween = transStart.inverse() * transFinal;
        float           x, y, z, roll, pitch, yaw;
        pcl::getTranslationAndEulerAngles(transBetween, x, y, z, roll, pitch, yaw);

        if(abs(roll) < surroundingkeyframeAddingAngleThreshold &&
           abs(pitch) < surroundingkeyframeAddingAngleThreshold &&
           abs(yaw) < surroundingkeyframeAddingAngleThreshold &&
           sqrt(x * x + y * y + z * z) < surroundingkeyframeAddingDistThreshold)
            return false;

        return true;
    }

    void addOdomFactor()
    {
        if(cloudKeyPoses3D->points.empty())
        {
            noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Variances(
                (gtsam::Vector(6) << 1e-2, 1e-2, M_PI * M_PI, 1e8, 1e8, 1e8)
                    .finished()); // rad*rad, meter*meter
            size_t gtSAMgraph_before_prior      = gtSAMgraph.size();
            size_t initialEstimate_before_prior = initialEstimate.size();
            gtSAMgraph.add(PriorFactor<Pose3>(0, trans2gtsamPose(transformTobeMapped), priorNoise));
            initialEstimate.insert(0, trans2gtsamPose(transformTobeMapped));
        }
        else
        {
            noiseModel::Diagonal::shared_ptr odometryNoise = noiseModel::Diagonal::Variances(
                (gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());
            gtsam::Pose3 poseFrom = pclPointTogtsamPose3(cloudKeyPoses6D->points.back());
            gtsam::Pose3 poseTo   = trans2gtsamPose(transformTobeMapped);
            size_t       gtSAMgraph_before_between      = gtSAMgraph.size();
            size_t       initialEstimate_before_between = initialEstimate.size();
            gtSAMgraph.add(BetweenFactor<Pose3>(cloudKeyPoses3D->size() - 1,
                                                cloudKeyPoses3D->size(), poseFrom.between(poseTo),
                                                odometryNoise));
            initialEstimate.insert(cloudKeyPoses3D->size(), poseTo);
        }
    }

    void addGPSFactor()
    {
        if(gpsQueue.empty())
            return;

        // wait for system initialized and settles down
        if(cloudKeyPoses3D->points.empty())
            return;
        else
        {
            if(common_lib_->pointDistance(cloudKeyPoses3D->front(), cloudKeyPoses3D->back()) < 5.0)
                return;
        }

        // pose covariance small, no need to correct
        if(poseCovariance(3, 3) < poseCovThreshold && poseCovariance(4, 4) < poseCovThreshold)
            return;

        // last gps position
        static PointType lastGPSPoint;

        while(!gpsQueue.empty())
        {
            if(ROS_TIME(gpsQueue.front().header.stamp) < timeLaserInfoCur - 0.2)
            {
                // message too old
                size_t gpsQueue_before_pop1 = gpsQueue.size();
                gpsQueue.pop_front();
            }
            else if(ROS_TIME(gpsQueue.front().header.stamp) > timeLaserInfoCur + 0.2)
            {
                // message too new
                break;
            }
            else
            {
                nav_msgs::msg::Odometry thisGPS              = gpsQueue.front();
                size_t                  gpsQueue_before_pop2 = gpsQueue.size();
                gpsQueue.pop_front();

                // GPS too noisy, skip
                float noise_x = thisGPS.pose.covariance[0];
                float noise_y = thisGPS.pose.covariance[7];
                float noise_z = thisGPS.pose.covariance[14];
                if(noise_x > gpsCovThreshold || noise_y > gpsCovThreshold)
                    continue;

                float gps_x = thisGPS.pose.pose.position.x;
                float gps_y = thisGPS.pose.pose.position.y;
                float gps_z = thisGPS.pose.pose.position.z;
                if(!useGpsElevation)
                {
                    gps_z   = transformTobeMapped[5];
                    noise_z = 0.01;
                }

                // GPS not properly initialized (0,0,0)
                if(abs(gps_x) < 1e-6 && abs(gps_y) < 1e-6)
                    continue;

                // Add GPS every a few meters
                PointType curGPSPoint;
                curGPSPoint.x = gps_x;
                curGPSPoint.y = gps_y;
                curGPSPoint.z = gps_z;
                if(common_lib_->pointDistance(curGPSPoint, lastGPSPoint) < 5.0)
                    continue;
                else
                    lastGPSPoint = curGPSPoint;

                gtsam::Vector Vector3(3);
                Vector3 << max(noise_x, 1.0f), max(noise_y, 1.0f), max(noise_z, 1.0f);
                noiseModel::Diagonal::shared_ptr gps_noise =
                    noiseModel::Diagonal::Variances(Vector3);
                gtsam::GPSFactor gps_factor(cloudKeyPoses3D->size(),
                                            gtsam::Point3(gps_x, gps_y, gps_z), gps_noise);
                size_t           gtSAMgraph_before_gps = gtSAMgraph.size();
                gtSAMgraph.add(gps_factor);

                aLoopIsClosed = true;
                break;
            }
        }
    }

    void addLoopFactor()
    {
        if(loopIndexQueue.empty())
            return;

        for(int i = 0; i < (int)loopIndexQueue.size(); ++i)
        {
            int          indexFrom   = loopIndexQueue[i].first;
            int          indexTo     = loopIndexQueue[i].second;
            gtsam::Pose3 poseBetween = loopPoseQueue[i];
            // gtsam::noiseModel::Diagonal::shared_ptr noiseBetween = loopNoiseQueue[i];
            auto   noiseBetween           = loopNoiseQueue[i];
            size_t gtSAMgraph_before_loop = gtSAMgraph.size();
            gtSAMgraph.add(BetweenFactor<Pose3>(indexFrom, indexTo, poseBetween, noiseBetween));
        }

        loopIndexQueue.clear();
        loopPoseQueue.clear();
        loopNoiseQueue.clear();
        aLoopIsClosed = true;
    }

    void saveKeyFramesAndFactor()
    {
        // mapAllignmentFactor
        if(foundMapAllignment)
            addMapAllignmentFactor();

        if(saveFrame() == false)
            return;

        // odom factor
        addOdomFactor();

        // gps factor
        addGPSFactor();

        // loop factor
        addLoopFactor();

        // cout << "****************************************************" << endl;
        // gtSAMgraph.print("GTSAM Graph:\n");

        // update iSAM
        isam->update(gtSAMgraph, initialEstimate);
        isam->update();

        if(aLoopIsClosed == true)
        {
            isam->update();
            isam->update();
            isam->update();
            isam->update();
            isam->update();
        }

        gtSAMgraph.resize(0);
        initialEstimate.clear();

        // save key poses
        PointType     thisPose3D;
        PointTypePose thisPose6D;
        Pose3         latestEstimate;

        isamCurrentEstimate = isam->calculateEstimate();
        latestEstimate      = isamCurrentEstimate.at<Pose3>(isamCurrentEstimate.size() - 1);
        // cout << "****************************************************" << endl;
        // isamCurrentEstimate.print("Current estimate: ");

        thisPose3D.x                  = latestEstimate.translation().x();
        thisPose3D.y                  = latestEstimate.translation().y();
        thisPose3D.z                  = latestEstimate.translation().z();
        thisPose3D.intensity          = cloudKeyPoses3D->size(); // this can be used as index
        size_t cloudKeyPoses3D_before = cloudKeyPoses3D->size();
        cloudKeyPoses3D->push_back(thisPose3D);

        thisPose6D.x                  = thisPose3D.x;
        thisPose6D.y                  = thisPose3D.y;
        thisPose6D.z                  = thisPose3D.z;
        thisPose6D.intensity          = thisPose3D.intensity; // this can be used as index
        thisPose6D.roll               = latestEstimate.rotation().roll();
        thisPose6D.pitch              = latestEstimate.rotation().pitch();
        thisPose6D.yaw                = latestEstimate.rotation().yaw();
        thisPose6D.time               = timeLaserInfoCur;
        size_t cloudKeyPoses6D_before = cloudKeyPoses6D->size();
        cloudKeyPoses6D->push_back(thisPose6D);

        // cout << "****************************************************" << endl;
        // cout << "Pose covariance:" << endl;
        // cout << isam->marginalCovariance(isamCurrentEstimate.size()-1) << endl << endl;
        poseCovariance = isam->marginalCovariance(isamCurrentEstimate.size() - 1);

        // save updated transform
        transformTobeMapped[0] = latestEstimate.rotation().roll();
        transformTobeMapped[1] = latestEstimate.rotation().pitch();
        transformTobeMapped[2] = latestEstimate.rotation().yaw();
        transformTobeMapped[3] = latestEstimate.translation().x();
        transformTobeMapped[4] = latestEstimate.translation().y();
        transformTobeMapped[5] = latestEstimate.translation().z();

        // save all the received edge and surf points
        pcl::PointCloud<PointType>::Ptr thisSurfKeyFrame(new pcl::PointCloud<PointType>());
        pcl::copyPointCloud(*laserCloudSurfLastDS, *thisSurfKeyFrame);

        // save key frame cloud
        size_t surfCloudKeyFrames_before = surfCloudKeyFrames.size();
        surfCloudKeyFrames.push_back(thisSurfKeyFrame);

        // The following code is copy from sc-lio-sam
        // Scan Context loop detector - giseop
        // - SINGLE_SCAN_FULL: using downsampled original point cloud (/full_cloud_projected +
        // downsampling)
        // - SINGLE_SCAN_FEAT: using surface feature as an input point cloud for scan context
        // (2020.04.01: checked it works.)
        // - MULTI_SCAN_FEAT: using NearKeyframes (because a MulRan scan does not have beyond
        // region, so to solve this issue ... )
        const SCInputType sc_input_type = SCInputType::SINGLE_SCAN_FULL; // change this

        if(sc_input_type == SCInputType::SINGLE_SCAN_FULL)
        {
            pcl::PointCloud<PointType>::Ptr thisRawCloudKeyFrame(new pcl::PointCloud<PointType>());
            pcl::fromROSMsg(cloudInfo.cloud_deskewed, *thisRawCloudKeyFrame);

            scManager_->makeAndSaveScancontextAndKeys(*thisRawCloudKeyFrame);
        }
        else if(sc_input_type == SCInputType::SINGLE_SCAN_FEAT)
        {
            scManager_->makeAndSaveScancontextAndKeys(*thisSurfKeyFrame);
        }
        else if(sc_input_type == SCInputType::MULTI_SCAN_FEAT)
        {
            pcl::PointCloud<PointType>::Ptr multiKeyFrameFeatureCloud(
                new pcl::PointCloud<PointType>());
            loopFindNearKeyframes(multiKeyFrameFeatureCloud, cloudKeyPoses6D->size() - 1,
                                  historyKeyframeSearchNum, -1);
            scManager_->makeAndSaveScancontextAndKeys(*multiKeyFrameFeatureCloud);
        }

        // save path for visualization
        updatePath(thisPose6D);
    }

    void correctPoses()
    {
        if(cloudKeyPoses3D->points.empty())
            return;

        if(aLoopIsClosed == true)
        {
            // clear map cache
            laserCloudMapContainer.clear();
            // clear path
            globalPath.poses.clear();
            // update key poses
            int numPoses = isamCurrentEstimate.size();
            for(int i = 0; i < numPoses; ++i)
            {
                cloudKeyPoses3D->points[i].x = isamCurrentEstimate.at<Pose3>(i).translation().x();
                cloudKeyPoses3D->points[i].y = isamCurrentEstimate.at<Pose3>(i).translation().y();
                cloudKeyPoses3D->points[i].z = isamCurrentEstimate.at<Pose3>(i).translation().z();

                cloudKeyPoses6D->points[i].x = cloudKeyPoses3D->points[i].x;
                cloudKeyPoses6D->points[i].y = cloudKeyPoses3D->points[i].y;
                cloudKeyPoses6D->points[i].z = cloudKeyPoses3D->points[i].z;
                cloudKeyPoses6D->points[i].roll =
                    isamCurrentEstimate.at<Pose3>(i).rotation().roll();
                cloudKeyPoses6D->points[i].pitch =
                    isamCurrentEstimate.at<Pose3>(i).rotation().pitch();
                cloudKeyPoses6D->points[i].yaw = isamCurrentEstimate.at<Pose3>(i).rotation().yaw();

                updatePath(cloudKeyPoses6D->points[i]);
            }

            aLoopIsClosed = false;
        }
    }

    void updatePath(const PointTypePose &pose_in)
    {
        geometry_msgs::msg::PoseStamped pose_stamped;
        rclcpp::Time                    t(static_cast<uint32_t>(pose_in.time * 1e9));
        pose_stamped.header.stamp    = t;
        pose_stamped.header.frame_id = odometryFrame;
        pose_stamped.pose.position.x = pose_in.x;
        pose_stamped.pose.position.y = pose_in.y;
        pose_stamped.pose.position.z = pose_in.z;
        tf2::Quaternion q;
        q.setRPY(pose_in.roll, pose_in.pitch, pose_in.yaw);
        pose_stamped.pose.orientation.x = q.x();
        pose_stamped.pose.orientation.y = q.y();
        pose_stamped.pose.orientation.z = q.z();
        pose_stamped.pose.orientation.w = q.w();

        size_t globalPath_before = globalPath.poses.size();
        globalPath.poses.push_back(pose_stamped);
    }

    void publishOdometry()
    {
        // Publish odometry for ROS (global)
        nav_msgs::msg::Odometry laserOdometryROS;
        laserOdometryROS.header.stamp         = timeLaserInfoStamp;
        laserOdometryROS.header.frame_id      = odometryFrame;
        laserOdometryROS.child_frame_id       = "odom_mapping";
        laserOdometryROS.pose.pose.position.x = transformTobeMapped[3];
        laserOdometryROS.pose.pose.position.y = transformTobeMapped[4];
        laserOdometryROS.pose.pose.position.z = transformTobeMapped[5];
        // Ref: http://wiki.ros.org/tf2/Tutorials/Migration/DataConversions
        tf2::Quaternion quat_tf;
        quat_tf.setRPY(transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
        geometry_msgs::msg::Quaternion quat_msg;
        tf2::convert(quat_tf, quat_msg);
        laserOdometryROS.pose.pose.orientation = quat_msg;
        pubLaserOdometryGlobal->publish(laserOdometryROS);

        // Publish TF
        quat_tf.setRPY(transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
        tf2::Transform t_odom_to_lidar = tf2::Transform(
            quat_tf,
            tf2::Vector3(transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5]));
        tf2::TimePoint               time_point = tf2_ros::fromRclcpp(timeLaserInfoStamp);
        tf2::Stamped<tf2::Transform> temp_odom_to_lidar(t_odom_to_lidar, time_point, odometryFrame);
        geometry_msgs::msg::TransformStamped trans_odom_to_lidar;
        tf2::convert(temp_odom_to_lidar, trans_odom_to_lidar);
        trans_odom_to_lidar.child_frame_id = "lidar_link";
        br->sendTransform(trans_odom_to_lidar);

        // Publish odometry for ROS (incremental)
        static nav_msgs::msg::Odometry laserOdomIncremental; // incremental odometry msg
        static Eigen::Affine3f         increOdomAffine;      // incremental odometry in affine
        if(lastIncreOdomPubFlag == false)
        {
            lastIncreOdomPubFlag = true;
            laserOdomIncremental = laserOdometryROS;
            increOdomAffine      = trans2Affine3f(transformTobeMapped);
        }
        else
        {
            Eigen::Affine3f affineIncre =
                incrementalOdometryAffineFront.inverse() * incrementalOdometryAffineBack;
            increOdomAffine = increOdomAffine * affineIncre;
            float x, y, z, roll, pitch, yaw;
            pcl::getTranslationAndEulerAngles(increOdomAffine, x, y, z, roll, pitch, yaw);
            if(cloudInfo.imuavailable == true && imuType)
            {
                if(std::abs(cloudInfo.imupitchinit) < 1.4)
                {
                    double          imuWeight = 0.1;
                    tf2::Quaternion imuQuaternion;
                    tf2::Quaternion transformQuaternion;
                    double          rollMid, pitchMid, yawMid;

                    // slerp roll
                    transformQuaternion.setRPY(roll, 0, 0);
                    imuQuaternion.setRPY(cloudInfo.imurollinit, 0, 0);
                    tf2::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight))
                        .getRPY(rollMid, pitchMid, yawMid);
                    roll = rollMid;

                    // slerp pitch
                    transformQuaternion.setRPY(0, pitch, 0);
                    imuQuaternion.setRPY(0, cloudInfo.imupitchinit, 0);
                    tf2::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight))
                        .getRPY(rollMid, pitchMid, yawMid);
                    pitch = pitchMid;
                }
            }
            laserOdomIncremental.header.stamp         = timeLaserInfoStamp;
            laserOdomIncremental.header.frame_id      = odometryFrame;
            laserOdomIncremental.child_frame_id       = "odom_mapping";
            laserOdomIncremental.pose.pose.position.x = x;
            laserOdomIncremental.pose.pose.position.y = y;
            laserOdomIncremental.pose.pose.position.z = z;
            tf2::Quaternion quat_tf;
            quat_tf.setRPY(roll, pitch, yaw);
            geometry_msgs::msg::Quaternion quat_msg;
            tf2::convert(quat_tf, quat_msg);
            laserOdomIncremental.pose.pose.orientation = quat_msg;
            if(isDegenerate)
                laserOdomIncremental.pose.covariance[0] = 1;
            else
                laserOdomIncremental.pose.covariance[0] = 0;
        }
        pubLaserOdometryIncremental->publish(laserOdomIncremental);
    }

    void publishFrames()
    {
        if(cloudKeyPoses3D->points.empty())
            return;

        if(debugFlag)
        {
            // publish key poses
            publishCloud(pubKeyPoses, cloudKeyPoses3D, timeLaserInfoStamp, odometryFrame);
            // Publish surrounding key frames
            publishCloud(pubRecentKeyFrames, laserCloudSurfFromMapDS, timeLaserInfoStamp,
                         odometryFrame);
        }
        // publish registered key frame
        if(debugFlag && pubRecentKeyFrame->get_subscription_count() != 0)
        {
            pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
            PointTypePose                   thisPose6D = trans2PointTypePose(transformTobeMapped);
            *cloudOut += *transformPointCloud(laserCloudSurfLastDS, &thisPose6D);
            publishCloud(pubRecentKeyFrame, cloudOut, timeLaserInfoStamp, odometryFrame);
        }
        // publish registered high-res raw cloud
        if(debugFlag && pubCloudRegisteredRaw->get_subscription_count() != 0)
        {
            pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
            pcl::fromROSMsg(cloudInfo.cloud_deskewed, *cloudOut);
            PointTypePose thisPose6D = trans2PointTypePose(transformTobeMapped);
            *cloudOut                = *transformPointCloud(cloudOut, &thisPose6D);
            publishCloud(pubCloudRegisteredRaw, cloudOut, timeLaserInfoStamp, odometryFrame);
        }
        // publish path
        if(pubPath->get_subscription_count() != 0)
        {
            globalPath.header.stamp    = timeLaserInfoStamp;
            globalPath.header.frame_id = odometryFrame;
            pubPath->publish(globalPath);
        }
        // publish SLAM infomation for 3rd-party usage
        static int lastSLAMInfoPubSize = -1;
        if(debugFlag && pubSLAMInfo->get_subscription_count() != 0)
        {
            // if (lastSLAMInfoPubSize != cloudKeyPoses6D->size())
            // {
            //     liorf::msg::CloudInfo slamInfo;
            //     slamInfo.header.stamp = timeLaserInfoStamp;
            //     pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
            //     *cloudOut += *laserCloudSurfLastDS;
            //     slamInfo.key_frame_cloud = publishCloud(rclcpp::Publisher(), cloudOut,
            //     timeLaserInfoStamp, lidarFrame); slamInfo.key_frame_poses =
            //     publishCloud(rclcpp::Publisher(), cloudKeyPoses6D, timeLaserInfoStamp,
            //     odometryFrame); pcl::PointCloud<PointType>::Ptr localMapOut(new
            //     pcl::PointCloud<PointType>()); *localMapOut += *laserCloudSurfFromMapDS;
            //     slamInfo.key_frame_map = publishCloud(rclcpp::Publisher(), localMapOut,
            //     timeLaserInfoStamp, odometryFrame); pubSLAMInfo->publish(slamInfo);
            //     lastSLAMInfoPubSize = cloudKeyPoses6D->size();
            // }
        }
    }

    void initialPoseHandler(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msgIn)
    {
        tf2::Quaternion quat(msgIn->pose.pose.orientation.x, msgIn->pose.pose.orientation.y,
                             msgIn->pose.pose.orientation.z, msgIn->pose.pose.orientation.w);
        double          roll, pitch, yaw;
        tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

        initializePose[0] = roll;
        initializePose[1] = pitch;
        initializePose[2] = yaw;

        initializePose[3] = msgIn->pose.pose.position.x;
        initializePose[4] = msgIn->pose.pose.position.y;
        initializePose[5] = msgIn->pose.pose.position.z;

        RCLCPP_INFO(get_logger(),
                    "Received initial pose: x: %f, y: %f, z: %f, roll: %f, pitch: %f, yaw: %f",
                    initializePose[3], initializePose[4], initializePose[5], pcl::rad2deg(roll),
                    pcl::rad2deg(pitch), pcl::rad2deg(yaw));
        hasInitializePose = true;
    }

    void mapLocationHandler(const interface_msgs::msg::PointCloudAvailable::SharedPtr msgIn)
    {
        loadMapLocation = msgIn->floor_path;

        RCLCPP_INFO(get_logger(), "Set map PCD directory to: %s", loadMapLocation.c_str());
    }

    bool loadGlobalMap()
    {
        std::string global_map_file = loadMapLocation + "/map.pcd";

        if(!std::filesystem::exists(global_map_file))
        {
            RCLCPP_WARN(get_logger(), "Global map file not found: %s", global_map_file.c_str());
            return false;
        }

        if(pcl::io::loadPCDFile<PointType>(global_map_file, *localizationMap) == -1)
        {
            RCLCPP_ERROR(get_logger(), "Failed to load global map from: %s",
                         global_map_file.c_str());
            return false;
        }

        downSizeFilterLocalMapSurf.setInputCloud(localizationMap);
        downSizeFilterLocalMapSurf.filter(*localizationMapDS);
        localizationMapDSNum = localizationMapDS->size();

        // Threshold of 1000 points in the global map has been copied from original Liorf
        // Localization Code, but can be optimized
        if(localizationMapDSNum < 1000)
        {
            RCLCPP_WARN(get_logger(), "Not enough features in global map: %s",
                        global_map_file.c_str());
            return false;
        }

        RCLCPP_INFO(get_logger(), "Global map loaded with a size of: %d", localizationMapDSNum);

        if(debugFlag)
        {
            RCLCPP_INFO(get_logger(), "Publishing localization map...");
            publishCloud(pubLocalizationMap, localizationMapDS, rclcpp::Time(), odometryFrame);
        }
        return true;
    }

    bool systemInitialize()
    {
        transformTobeMapped[0] = initializePose[0];
        transformTobeMapped[1] = initializePose[1];
        transformTobeMapped[2] = initializePose[2];
        transformTobeMapped[3] = initializePose[3];
        transformTobeMapped[4] = initializePose[4];
        transformTobeMapped[5] = initializePose[5];

        if(slamModeCache_ == "mapping")
        {
            RCLCPP_INFO(get_logger(), "SLAM mode: %s", slamModeCache_.c_str());
            systemInitialized = true;
            return true;
        }

        else if(slamModeCache_ == "localization")
        {
            RCLCPP_INFO(get_logger(), "SLAM mode: %s", slamModeCache_.c_str());

            if(!loadGlobalMap())
            {
                RCLCPP_WARN(get_logger(),
                            "A loaded global map is required to initialize the system.");
                return false;
            }

            if(!performICPAlignment())
            {
                RCLCPP_WARN(get_logger(), "ICP alignment failed during system initialization.");
                hasInitializePose = false;
                return false;
            }

            transformTobeMapped[0] = localizationPose[0];
            transformTobeMapped[1] = localizationPose[1];
            transformTobeMapped[2] = localizationPose[2];
            transformTobeMapped[3] = localizationPose[3];
            transformTobeMapped[4] = localizationPose[4];
            transformTobeMapped[5] = localizationPose[5];

            RCLCPP_INFO(get_logger(), "Initial pose successful for localization");
            systemInitialized = true;
            return true;
        }

        else
        {
            RCLCPP_WARN(get_logger(), "SLAM mode is unknown during system initialization.");
            return false;
        }
    }

    void getSlamMode()
    {
        if(!slam_mode_client_->service_is_ready())
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                 "get_state service not available");
            slamModeCache_ = "unknown";
            return;
        }

        auto request        = std::make_shared<autonomy_msgs::srv::GetState::Request>();
        waitingForSlamMode_ = true;

        slam_mode_client_->async_send_request(
            request,
            [this](rclcpp::Client<autonomy_msgs::srv::GetState>::SharedFuture response)
            {
                waitingForSlamMode_ = false;
                try
                {
                    slamModeCache_ = response.get()->state;
                }
                catch(const std::exception &e)
                {
                    RCLCPP_WARN(get_logger(), "Failed to get SLAM mode: %s", e.what());
                    slamModeCache_ = "unknown";
                }
            });
    }

    bool performICPAlignment()
    {
        // All icp parameters have been set after manual testing
        // and can be made into configurable parameters or optimized if needed
        pcl::IterativeClosestPoint<PointType, PointType> icp;
        icp.setMaxCorrespondenceDistance(icpMaxCorrespondenceDistance);
        icp.setMaximumIterations(icpMaximumIterations);
        icp.setTransformationEpsilon(icpTransformationEpsilon);
        icp.setEuclideanFitnessEpsilon(icpEuclideanFitnessEpsilon);
        icp.setRANSACIterations(icpRANSACIterations);

        Eigen::Affine3f initialize_affine = trans2Affine3f(transformTobeMapped);

        pcl::PointCloud<PointType>::Ptr out_cloud(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr result(new pcl::PointCloud<PointType>());
        pcl::transformPointCloud(*laserCloudSurfLast, *out_cloud, initialize_affine);

        // Align clouds
        icp.setInputSource(out_cloud);
        icp.setInputTarget(localizationMapDS);
        icp.align(*result);

        Eigen::Affine3f correctionLidarFrame;
        float           x, y, z, roll, pitch, yaw;
        correctionLidarFrame = icp.getFinalTransformation();

        if(!icp.hasConverged() || icp.getFitnessScore() >= icpFitnessThreshold)
        {
            RCLCPP_WARN(get_logger(), "ICP alignment failed: converged=%d, fitness score=%.6f",
                        icp.hasConverged() ? 1 : 0, icp.getFitnessScore());
            return false;
        }

        localizationFitness = icp.getFitnessScore();

        Eigen::Affine3f tCorrect = correctionLidarFrame * initialize_affine;
        pcl::getTranslationAndEulerAngles(tCorrect, x, y, z, roll, pitch, yaw);

        RCLCPP_INFO(get_logger(), "ICP alignment succeeded!");
        RCLCPP_INFO(get_logger(),
                    "Initial guessed pose for localization: x: %f, y: %f, z: %f, roll: %f, pitch: "
                    "%f, yaw: %f",
                    transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5],
                    pcl::rad2deg(transformTobeMapped[0]), pcl::rad2deg(transformTobeMapped[1]),
                    pcl::rad2deg(transformTobeMapped[2]));

        localizationPose[0] = roll;
        localizationPose[1] = pitch;
        localizationPose[2] = yaw;
        localizationPose[3] = x;
        localizationPose[4] = y;
        localizationPose[5] = z;

        RCLCPP_INFO(get_logger(),
                    "Found localization pose: x: %f, y: %f, z: %f, roll: %f, pitch: %f, yaw: %f",
                    localizationPose[3], localizationPose[4], localizationPose[5],
                    pcl::rad2deg(localizationPose[0]), pcl::rad2deg(localizationPose[1]),
                    pcl::rad2deg(localizationPose[2]));

        return true;
    }

    void addMapAllignmentFactor()
    {
        // wait for system initialized and settles down
        if(cloudKeyPoses3D->points.empty())
        {
            return;
        }

        float        map_roll  = localizationPose[0];
        float        map_pitch = localizationPose[1];
        float        map_yaw   = localizationPose[2];
        float        map_x     = localizationPose[3];
        float        map_y     = localizationPose[4];
        float        map_z     = localizationPose[5];
        gtsam::Pose3 mapPose =
            Pose3(Rot3::RzRyRx(map_roll, map_pitch, map_yaw), Point3(map_x, map_y, map_z));

        gtsam::Vector Vector6(6);
        float         linNoiseScore = std::max(localizationFitness, minimalCovariance_linear);
        float         angNoiseScore = std::max(localizationFitness, minimalCovariance_angular);
        Vector6 << linNoiseScore, linNoiseScore, linNoiseScore, angNoiseScore, angNoiseScore,
            angNoiseScore;
        noiseModel::Diagonal::shared_ptr mapNoise = noiseModel::Diagonal::Variances(Vector6);

        // option 1
        //  gtsam::GPSFactor gps_factor(cloudKeyPoses3D->size(), map_pose, mapNoise);
        //  gtSAMgraph.add(gps_factor);

        // option 2
        RCLCPP_INFO(get_logger(), "Right before adding the factor");
        gtSAMgraph.add(PriorFactor<Pose3>(cloudKeyPoses3D->size(), mapPose, mapNoise));

        RCLCPP_INFO(get_logger(), "Added Map Alignment Factor!");

        aLoopIsClosed = true;
    }
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    // options.use_intra_process_comms(true);
    rclcpp::executors::SingleThreadedExecutor exec;

    auto MO = std::make_shared<mapOptimization>(options);
    exec.add_node(MO->get_node_base_interface());

    exec.spin();

    rclcpp::shutdown();
    return 0;
}
