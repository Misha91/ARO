#include <laser_geometry/laser_geometry.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <ros/ros.h>

namespace point_cloud_color {

    using sensor_msgs::LaserScan;
    using sensor_msgs::PointCloud2;

    /**
     * @brief A nodelet converting laser scans to point clouds.
     *
     * This nodelet converts scans (i.e., sensor_msgs::LaserScan messages) to point clouds (i.e., to
     * sensor_msgs::PointCloud2 messages).
     *
     */
    class ScanToPointCloud : public nodelet::Nodelet {
    public:
        ScanToPointCloud();

        virtual ~ScanToPointCloud();

        void onInit();

        void scanCallback(const LaserScan::ConstPtr &scan);

    private:
        std::string fixed_frame;
        /**
         * @brief channelOptions Channels to extract.
         *
         * Channels to extract, see laser_geometry.h for details.
         * 0x00 - no channels enabled,
         * 0x01 - enable intensity (default),
         * 0x02 - enable index (default),
         * 0x04 - enable distance,
         * 0x08 - enable stamps,
         * 0x10 - enable viewpoint.
         */
        int channel_options;
        int scan_queue_size;
        int cloud_queue_size;
        double tf_cache_sec;
        tf::TransformListener tf_listener;
        double tf_timeout;
        ros::Subscriber scan_sub;
        laser_geometry::LaserProjection projector;
        ros::Publisher points_pub;
    };

    ScanToPointCloud::ScanToPointCloud():
            fixed_frame("odom"),
            channel_options(laser_geometry::channel_option::Default),
            scan_queue_size(2),
            cloud_queue_size(2),
            tf_cache_sec(10.0),
            tf_listener(ros::Duration(tf_cache_sec)),
            tf_timeout(1.0) {
    }

    ScanToPointCloud::~ScanToPointCloud() {
    }

    void ScanToPointCloud::onInit() {
        NODELET_INFO("ScanToPointCloud::onInit: Initializing...");

        ros::NodeHandle &nh = getNodeHandle();
        ros::NodeHandle &pnh = getPrivateNodeHandle();

        // Process parameters.
        pnh.param("fixed_frame", fixed_frame, fixed_frame);
        NODELET_INFO("Using fixed frame: %s.", fixed_frame.c_str());
        pnh.param("tf_timeout", tf_timeout, tf_timeout);
        NODELET_INFO("Transform timeout: %.3g s.", tf_timeout);
        pnh.param("channel_options", channel_options, channel_options);
        NODELET_INFO("Channel options: %#x.", channel_options);
        pnh.param("scan_queue_size", scan_queue_size, scan_queue_size);
        NODELET_INFO("Scan queue size: %i.", scan_queue_size);
        pnh.param("cloud_queue_size", cloud_queue_size, cloud_queue_size);
        NODELET_INFO("Point cloud queue size: %i.", cloud_queue_size);

        // Advertise scan point cloud.
        std::string cloud_topic = nh.resolveName("cloud", true);
        points_pub = nh.advertise<PointCloud2>(
                cloud_topic,
                static_cast<uint32_t>(cloud_queue_size),
                false);
        NODELET_INFO("Point cloud advertised: %s.", cloud_topic.data());

        // Subscribe scan topic.
        std::string scan_topic = nh.resolveName("scan", true);
        scan_sub = nh.subscribe<LaserScan>(
                scan_topic,
                static_cast<uint32_t>(scan_queue_size),
                &ScanToPointCloud::scanCallback,
                this);
        NODELET_INFO("Scan subscribed: %s.", scan_topic.data());
    }

    void ScanToPointCloud::scanCallback(const LaserScan::ConstPtr &scan) {
        ros::Time t_last = scan->header.stamp
                           + ros::Duration().fromSec(scan->ranges.size() * scan->time_increment);
        if (!tf_listener.waitForTransform(
                scan->header.frame_id,
                fixed_frame,
                t_last,
                ros::Duration(tf_timeout))) {
            NODELET_WARN_THROTTLE(
                    1.0,
                    "Could not transform from %s to %s at %.3f.",
                    scan->header.frame_id.c_str(),
                    fixed_frame.c_str(),
                    scan->header.stamp.toSec());
            return;
        }

        PointCloud2::Ptr cloud(new PointCloud2);
        try {
            projector.transformLaserScanToPointCloud(
                    fixed_frame,
                    *scan,
                    *cloud,
                    tf_listener,
                    -1.0,
                    channel_options);
            points_pub.publish(cloud);
        } catch (tf::TransformException &ex) {
            ROS_ERROR_THROTTLE(1.0, "Transform exception: %s.", ex.what());
            return;
        }
    }

} /* namespace point_cloud_color */

PLUGINLIB_EXPORT_CLASS(point_cloud_color::ScanToPointCloud, nodelet::Nodelet)
