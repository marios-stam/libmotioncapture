#pragma once
#include "libmotioncapture/motioncapture.h"
#include <ros/ros.h>
#include <tf/transform_listener.h>

namespace libmotioncapture
{

    class MotionCaptureArucoImpl;

    class MotionCaptureAruco : public MotionCapture
    {
    public:
        MotionCaptureAruco(const std::string &hostname);

        virtual ~MotionCaptureAruco();

        const std::string &version() const;

        // implementations for MotionCapture interface
        virtual void waitForNextFrame();
        virtual const std::map<std::string, RigidBody> &rigidBodies() const;
        // virtual RigidBody rigidBodyByName(const std::string &name) const;
        virtual const PointCloud &pointCloud() const;
        virtual const std::vector<LatencyInfo> &latency() const;
        virtual uint64_t timeStamp() const;

        virtual bool supportsRigidBodyTracking() const { return true; } // TODO: implement

        virtual bool supportsPointCloud() const { return false; }

        virtual bool supportsLatencyEstimate() const { return false; }

        virtual bool supportsTimeStamp() const { return false; } // TODO: implement

    private:
        MotionCaptureArucoImpl *pImpl;
        std::vector<std::string> tf_ids;
        ros::NodeHandle nh;
        tf::TransformListener listener;
        std::vector<std::string> ids;
    };

} // namespace libobjecttracker
