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

        // start ROS node
        int argc;
        char **argv;
        ros::init(argc, argv, "aruco_listener");

        // Setup Transform listener

        // query model config(# of drones,rate of data,etc)
        listener.getFrameStrings(ids);
        for (uint8_t i = 0; i < ids.size(); i++)
        {
            std::string s = ids[i];
            if (s.rfind("/aruco_", 0) == 0) // if frame name starts with /aruco_
            {
                char id_string = s.at(7);
                int id = id_string - '0'; // convert to int
                std ::cout << "id: " << id << std::endl;

                tf_ids.push_back(s);
                pImpl->rigidBodies.push_back({id, 0, 0, 0, 0, 0, 0, 1, 0.0, true});
            }
        }

        // connect to topic to receive 1st mocap data
    }

    const std::string &MotionCaptureAruco::version() const { return pImpl->version; }

    void MotionCaptureAruco::waitForNextFrame()
    {
        bool found_transform[tf_ids.size()];

        tf::StampedTransform transform;
        uint8_t i = 0;
        for (auto tf_id : tf_ids)
        {
            try
            {
                listener.lookupTransform("/world", tf_id, ros::Time(0), transform);
            }
            catch (tf::TransformException ex)
            {
                ROS_ERROR("Could not find transform : %s", tf_id);
                ROS_ERROR("%s", ex.what());
            }

            pImpl->rigidBodies[i].x = transform.getOrigin().x();
            pImpl->rigidBodies[i].y = transform.getOrigin().y();
            pImpl->rigidBodies[i].z = transform.getOrigin().z();

            pImpl->rigidBodies[i].qx = transform.getRotation().x();
            pImpl->rigidBodies[i].qy = transform.getRotation().y();
            pImpl->rigidBodies[i].qz = transform.getRotation().z();
            pImpl->rigidBodies[i].qw = transform.getRotation().w();

            // Printing the values of the transform
            ROS_INFO("Transform %s received", tf_id.c_str());
            ROS_INFO("x:%f,y:%f,z:%f", transform.getOrigin().x(), transform.getOrigin().y(),
                     transform.getOrigin().z());

            ROS_INFO("qx:%f,qy:%f,qz:%f,qw:%f", transform.getRotation().x(),
                     transform.getRotation().y(), transform.getRotation().z(),
                     transform.getRotation().w());

            i++;
        }

        // Rigid body position and orientation

        // Calculate latencies (ROS communication,Camera processsing,etc)

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
