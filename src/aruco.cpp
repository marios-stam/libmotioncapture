#include "libmotioncapture/aruco.h"
#include <filesystem>

namespace libmotioncapture
{
    // constexpr int MAX_PACKETSIZE = 65503; // max size of packet (actual packet size is dynamic)
    // constexpr int MAX_NAMELENGTH = 256;

    class MotionCaptureArucoImpl
    {
    public:
        MotionCaptureArucoImpl() : version(), data() {}

        void parseModelDef(const char *data)
        {
            // pass
        }

    public:
        std::string version;

        uint64_t clockFrequency; // ticks/second for timestamps
        std::vector<char> data;

        struct rigidBody
        {
            int ID;
            float x;
            float y;
            float z;
            float qx;
            float qy;
            float qz;
            float qw;
            float fError; // mean marker error
            bool bTrackingValid;
        };
        std::vector<rigidBody> rigidBodies;

        struct marker
        {
            float x;
            float y;
            float z;
        };
        std::vector<marker> markers;

        struct rigidBodyDefinition
        {
            std::string name;
            int ID;
            int parentID;
            float xoffset;
            float yoffset;
            float zoffset;
        };
        std::map<int, rigidBodyDefinition> rigidBodyDefinitions;
    };

    MotionCaptureAruco::MotionCaptureAruco(const std::string &hostname)
    {
        marker_length_m = 0.08; // meters

        // SETUP ARUCO ID-RIGIDBODY ID
        ids_dict.insert(std::make_pair(0, 0));

        pImpl = new MotionCaptureArucoImpl;
        int dictionaryId = 0;
        dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));

        try
        {
            cv::FileStorage fs("/home/marios/crazyswarm/calibration_params.yml", cv::FileStorage::READ);

            fs["camera_matrix"] >> camera_matrix;
            fs["distortion_coefficients"] >> dist_coeffs;
        }
        catch (cv::Exception &e)
        {
            const char *err_msg = e.what();
            std::cout << "Exception: " << err_msg << std::endl;
        }
        catch (const std::exception &e)
        {
            std::cout << e.what() << std::endl;
        }

        std::cout << "camera_matrix " << camera_matrix << std::endl;
        std::cout << "dist coeffs " << dist_coeffs << std::endl;

        in_video.open(0);
        if (!in_video.isOpened())
        {
            std::cerr << "failed to open video input: " << std::endl;
            // return 1;
        }
    }

    const std::string &MotionCaptureAruco::version() const { return pImpl->version; }

    void MotionCaptureAruco::waitForNextFrame()
    {
        in_video.grab();
        in_video.retrieve(image);
        image.copyTo(image_copy);

        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        cv::aruco::detectMarkers(image, dictionary, corners, ids);

        // if at least one marker detected
        if (ids.size() > 0)
        {
            cv::aruco::drawDetectedMarkers(image_copy, corners, ids);

            std::vector<cv::Vec3d> rvecs, tvecs;
            cv::aruco::estimatePoseSingleMarkers(corners, marker_length_m, camera_matrix, dist_coeffs, rvecs, tvecs);

            std::cout << "Translation: " << tvecs[0] << "\tRotation: " << rvecs[0] << std::endl;
            int aruco_id, rigid_body_id;
            // Draw axis for each marker
            for (int i = 0; i < ids.size(); i++)
            {
                aruco_id = ids[i];

                std::cout << "aruco_id: " << aruco_id << std::endl;
                std::map<int, int>::iterator pos = ids_dict.find(aruco_id);
                if (pos == ids_dict.end())
                {
                    std::cout << "Could't find the aruco_id:" << aruco_id << "in dict" << std::endl;
                }
                else
                {
                    rigid_body_id = pos->second;
                    std::cout << "rigid_body_id: " << rigid_body_id << std::endl;
                }

                cv::aruco::drawAxis(image_copy, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], 0.1);
                // This section is going to print the data for all the detected
                // markers. If you have more than a single marker, it is
                // recommended to change the below section so that either you
                // only print the data for a specific marker, or you print the
                // data for each marker separately.
                vector_to_marker.str(std::string());
                vector_to_marker << std::setprecision(4) << "x: " << std::setw(8) << tvecs[0](0);
                cv::putText(image_copy, vector_to_marker.str(), cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 252, 124), 1, CV_AVX);

                vector_to_marker.str(std::string());
                vector_to_marker << std::setprecision(4) << "y: " << std::setw(8) << tvecs[0](1);
                cv::putText(image_copy, vector_to_marker.str(), cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 252, 124), 1, CV_AVX);

                vector_to_marker.str(std::string());
                vector_to_marker << std::setprecision(4) << "z: " << std::setw(8) << tvecs[0](2);
                cv::putText(image_copy, vector_to_marker.str(), cv::Point(10, 70), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 252, 124), 1, CV_AVX);

                pImpl->rigidBodies[rigid_body_id].x = tvecs[i](0);
                pImpl->rigidBodies[rigid_body_id].y = tvecs[i](1);
                pImpl->rigidBodies[rigid_body_id].z = tvecs[i](2);
                pImpl->rigidBodies[rigid_body_id].qx = 0;
                pImpl->rigidBodies[rigid_body_id].qy = 0;
                pImpl->rigidBodies[rigid_body_id].qz = 0;
                pImpl->rigidBodies[rigid_body_id].qw = 1;
            }
        }

        imshow("Pose estimation", image_copy);

        int wait_time = 10;

        char key = (char)cv::waitKey(wait_time);
        if (key == 27)
        {
            // break;
            // TODO : add a way to exit
        }

        // Rigid body position and orientation
        if (pImpl->rigidBodies.size() == 0)
        {
            pImpl->rigidBodies.push_back({0, 0, 0, 0, 0, 0, 0, 0, 0, true});
        }
        else
        {
        }

        // Calculate latenc>ies (ROS communication,Camera processsing,etc)

        // const uint64_t cameraLatencyTicks =
        //     cameraDataReceivedTimestamp - cameraMidExposureTimestamp;
        // const double cameraLatencySeconds = cameraLatencyTicks /
        // (double)pImpl->clockFrequency; latencies_.emplace_back(LatencyInfo("Camera",
        // cameraLatencySeconds));

        // const uint64_t swLatencyTicks = transmitTimestamp - cameraDataReceivedTimestamp;
        // const double swLatencySeconds = swLatencyTicks / (double)pImpl->clockFrequency;
        // latencies_.emplace_back(LatencyInfo("Motive", swLatencySeconds));
    }

    const std::map<std::string, RigidBody> &MotionCaptureAruco::rigidBodies() const
    {
        // TODO: avoid copies here...
        rigidBodies_.clear();
        for (const auto &rb : pImpl->rigidBodies)
        {
            if (rb.bTrackingValid)
            {
                const auto &def = pImpl->rigidBodyDefinitions[rb.ID];

                Eigen::Vector3f position(rb.x + def.xoffset, rb.y + def.yoffset, rb.z + def.zoffset);

                Eigen::Quaternionf rotation(rb.qw, // w
                                            rb.qx, // x
                                            rb.qy, // y
                                            rb.qz  // z
                );
                rigidBodies_.emplace(def.name, RigidBody(def.name, position, rotation));
            }
        }
        return rigidBodies_;
    }

    const PointCloud &MotionCaptureAruco::pointCloud() const
    {
        // TODO: avoid copies here...
        pointcloud_.resize(pImpl->markers.size(), Eigen::NoChange);
        for (size_t r = 0; r < pImpl->markers.size(); ++r)
        {
            const auto &marker = pImpl->markers[r];
            pointcloud_.row(r) << marker.x, marker.y, marker.z;
        }
        return pointcloud_;
    }

    const std::vector<LatencyInfo> &MotionCaptureAruco::latency() const { return latencies_; }

    uint64_t MotionCaptureAruco::timeStamp() const { return timestamp_; }

    MotionCaptureAruco::~MotionCaptureAruco() { delete pImpl; }
}
