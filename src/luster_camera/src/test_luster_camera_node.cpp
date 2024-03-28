#include <thread>
#include "luster_camera/luster_camera.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_luster_camera_node");
    ros::NodeHandle nh;
    LusterCamera luster_camera;
    cv::Mat ptz_img;
    image_transport::ImageTransport image_transport(nh);
    image_transport::Publisher image_transport_publisher;
    image_transport_publisher = image_transport.advertise("/luster_img", 1);

    // 子线程，循环采集图像
    std::thread loop_execute(&LusterCamera::grabImage, &luster_camera);
    sleep(2);  // 等待2秒，让子线程先执行

    // 主线程，处理图像
    ros::Rate rate(luster_camera.getFps());
    while (ros::ok())
    {
        luster_camera.returnImage(ptz_img);
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", ptz_img).toImageMsg();
        image_transport_publisher.publish(msg);
        rate.sleep();
    }

    // 终止处理
    ros::shutdown();
    luster_camera.shutDown();
    loop_execute.join();


    return 0;
}