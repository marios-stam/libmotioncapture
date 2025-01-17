#pragma once
#include "libmotioncapture/motioncapture.h"
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>

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

        // OpenCV variables
        cv::VideoCapture in_video;

        cv::Mat image, image_copy;
        cv::Mat camera_matrix, dist_coeffs;
        std::ostringstream vector_to_marker;
        cv::Ptr<cv::aruco::Dictionary> dictionary;

        cv::FileStorage fs;
        float marker_length_m;

        std::map<int, int> ids_dict; // connects aruco id to Rigid_body id
    };

} // namespace libobjecttracker
