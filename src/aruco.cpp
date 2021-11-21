#include "libmotioncapture/aruco.h"

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
        pImpl = new MotionCaptureArucoImpl;

        // Setup Topic listener

        // query model config(# of drones,rate of data,etc)

        // connect to topic to receive 1st mocap data
    }

    const std::string &MotionCaptureAruco::version() const { return pImpl->version; }

    void MotionCaptureAruco::waitForNextFrame()
    {
        // use a loop to get latest data
        do
        {
            // wait for new transform
        } while (/*not new transform*/ true);

        // Rigid body position and orientation
        // memcpy(&pImpl->rigidBodies[j].ID, ptr, 4);
        // ptr += 4;
        // memcpy(&pImpl->rigidBodies[j].x, ptr, 4);
        // ptr += 4;
        // memcpy(&pImpl->rigidBodies[j].y, ptr, 4);
        // ptr += 4;
        // memcpy(&pImpl->rigidBodies[j].z, ptr, 4);
        // ptr += 4;
        // memcpy(&pImpl->rigidBodies[j].qx, ptr, 4);
        // ptr += 4;
        // memcpy(&pImpl->rigidBodies[j].qy, ptr, 4);
        // ptr += 4;
        // memcpy(&pImpl->rigidBodies[j].qz, ptr, 4);
        // ptr += 4;
        // memcpy(&pImpl->rigidBodies[j].qw, ptr, 4);
        // ptr += 4;

        // Calculate latencies (ROS communication,Camera processsing,etc)

        // const uint64_t cameraLatencyTicks =
        //     cameraDataReceivedTimestamp - cameraMidExposureTimestamp;
        // const double cameraLatencySeconds = cameraLatencyTicks / (double)pImpl->clockFrequency;
        // latencies_.emplace_back(LatencyInfo("Camera", cameraLatencySeconds));

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

                Eigen::Vector3f position(rb.x + def.xoffset, rb.y + def.yoffset,
                                         rb.z + def.zoffset);

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
