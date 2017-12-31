// save_cloud.cpp

#include <ros/ros.h>
#include <boost/thread/thread.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <sensor_msgs/image_encodings.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <sensor_msgs/PointCloud2.h>
#include <pcl/common/common_headers.h>
#include <pcl/console/parse.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/mls.h>
#include <pcl/visualization/boost.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <vector>

// #define RGB_CLOUD // comment out if you want to use colored point cloud

ros::Publisher cloud_pub, image_pub;

using namespace std;

static const string IMAGE_WINDOW  = "Image Viewer";
static const string CLOUD_WINDOW  = "Cloud Viewer";
static const string CLOUD_ID      = "cloud1";
cv::Mat g_color_img;

#ifdef RGB_CLOUD
pcl::PointCloud<pcl::PointXYZRGB>::Ptr g_cloud;
#else
pcl::PointCloud<pcl::PointXYZ>::Ptr g_cloud;
#endif

boost::shared_ptr<pcl::visualization::PCLVisualizer> cloud_viewer(new pcl::visualization::PCLVisualizer(CLOUD_WINDOW));
//pcl::visualization::CloudViewer cloud_viewer(CLOUD_WINDOW);

void  keyboard_cb(const pcl::visualization::KeyboardEvent& event, void*)
{
    static int save_count = 0;

    if (event.getKeySym() == "s" && event.keyDown ()) {
        //* save
        std::stringstream filename_image;
        filename_image << save_count << ".png";
        cv::imwrite( filename_image.str(), g_color_img );
        std::cout << filename_image.str() << " saved." << std::endl;
        std::stringstream filename_cloud;
        filename_cloud << save_count << ".pcd";
        pcl::io::savePCDFileBinary(filename_cloud.str(), *g_cloud);
        // pcl::io::savePCDFile( filename2.str(), input_cloud );
        std::cout << filename_cloud.str() << " saved." << std::endl;
        save_count++;
    }

    if (event.getKeySym() == "z" && event.keyDown ()) {
        exit(1);
    }
}

void mouse_cb(const pcl::visualization::MouseEvent& mouse_event, void*)
{
    if (mouse_event.getType () == pcl::visualization::MouseEvent::MouseButtonPress && mouse_event.getButton () == pcl::visualization::MouseEvent::LeftButton) {
        cout << "left button pressed @ " << mouse_event.getX () << " , " << mouse_event.getY () << endl;
    }
}


void image_cb(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;

    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    g_color_img = cv_ptr->image;
    cv::imshow(IMAGE_WINDOW, cv_ptr->image);
    cv::waitKey(1);
}


void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input)
{
    if ((input->width * input->height) == 0) return;

#ifdef RGB_CLOUD
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
#else
    pcl::PointCloud<pcl::PointXYZ> cloud;
#endif

    pcl::fromROSMsg (*input, cloud);

    //vector<int> indicies;
    //pcl::removeNaNFromPointCloud(cloud, cloud, indicies);

    if (!cloud_viewer->wasStopped()) {
        //cloud_viewer.showCloud(cloud.makeShared());
        cloud_viewer->spinOnce();

        boost::this_thread::sleep(boost::posix_time::microseconds(10000));

        // ポイントクラウドの更新
        // false if the point cloud doesn't exist, true if the pose was successfully updated.
        g_cloud = cloud.makeShared();
        if (!cloud_viewer->updatePointCloud(cloud.makeShared(), CLOUD_ID)) {
            cloud_viewer->addPointCloud(cloud.makeShared(), CLOUD_ID);
            cloud_viewer->setCameraPosition (0, 0, 0, 0, 0, 1, 0, -1, 0); // Position, Viewpoint, Up
        }
    }
}

int main (int argc, char** argv)
{
    std::cout << "******** Save cloud **********" << std::endl;
    std::cout << "s: Save a cloud and image file" << std::endl;
    std::cout << "z: Quit" << std::endl;
    std::cout << "******************************" << std::endl;


#ifdef RGB_CLOUD
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
#else
    pcl::PointCloud<pcl::PointXYZ> cloud;
#endif

    // Initialize ROS
    ros::init (argc, argv, "save_cloud");
    ros::NodeHandle nh;
    // サブスクライバーの作成
    ros::Subscriber cloud_sub = nh.subscribe ("/kinect2/qhd/points", 1, cloud_cb);
    ros::Subscriber image_sub = nh.subscribe ("/kinect2/qhd/image_color", 1, image_cb);

    cv::namedWindow(IMAGE_WINDOW);

    cloud_viewer->registerMouseCallback(mouse_cb);
    cloud_viewer->registerKeyboardCallback(keyboard_cb);


    double bcolor[3] = {0, 0, 0}; // background color 0<= (r,g,b) < = 1
    cloud_viewer->setBackgroundColor (bcolor[0], bcolor[1], bcolor[2]);

#ifdef RGB_CLOUD
    cloud_viewer->addPointCloud<pcl::PointXYZRGB> (cloud.makeShared(), CLOUD_ID);
#else
    cloud_viewer->addPointCloud<pcl::PointXYZ> (cloud.makeShared(), CLOUD_ID);
#endif

    cloud_viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, CLOUD_ID);
    //Adds 3D axes describing a coordinate system to screen at 0,0,0.
    //viewer->addCoordinateSystem(1.0); // scale x:red, y:green, z:blue
    cloud_viewer->initCameraParameters();
    cloud_viewer->setCameraPosition(0, 0, 0, 0, 0, 1, 0, -1, 0);

    ros::spin();

    return 1;
}
