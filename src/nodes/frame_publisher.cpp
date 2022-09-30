#include <ros/ros.h>
#include <ros/time.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include "depthai/depthai.hpp"


static constexpr int fps = 35;
static constexpr auto monoRes = dai::MonoCameraProperties::SensorResolution::THE_480_P;


int main(int argc, char **argv)
{
    // Set up ROS.
    ros::init(argc, argv, "frame_publisher");
    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);

    image_transport::Publisher rgb_pub = it.advertise("/camera/rgb/image_raw", 1);
    image_transport::Publisher depth_pub = it.advertise("/camera/depth/image_raw", 1);

    // setup cameras
    dai::Pipeline pipeline;
    std::vector<std::string> queueNames;
  
    // Define sources and outputs
    auto controlIn = pipeline.create<dai::node::XLinkIn>();
    auto camRgb = pipeline.create<dai::node::ColorCamera>();
    auto left = pipeline.create<dai::node::MonoCamera>();
    auto right = pipeline.create<dai::node::MonoCamera>();
    auto stereo = pipeline.create<dai::node::StereoDepth>();

    auto rgbOut = pipeline.create<dai::node::XLinkOut>();
    auto depthOut = pipeline.create<dai::node::XLinkOut>();

    controlIn->setStreamName("control");
    
    rgbOut->setStreamName("rgb");
    queueNames.push_back("rgb");

    depthOut->setStreamName("depth");
    queueNames.push_back("depth");

    camRgb->setBoardSocket(dai::CameraBoardSocket::RGB);
    camRgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    camRgb->setFps(fps);
    camRgb->setColorOrder(dai::ColorCameraProperties::ColorOrder::RGB);
    camRgb->setIspScale(2, 3, 8, 9);

    left->setResolution(monoRes);
    left->setBoardSocket(dai::CameraBoardSocket::LEFT);
    left->setFps(fps);

    right->setResolution(monoRes);
    right->setBoardSocket(dai::CameraBoardSocket::RIGHT);
    right->setFps(fps);

    stereo->setDefaultProfilePreset(dai::node::StereoDepth::PresetMode::HIGH_DENSITY);
    stereo->setLeftRightCheck(true);
    stereo->setExtendedDisparity(true);
    stereo->setSubpixel(false);
    stereo->setDepthAlign(dai::CameraBoardSocket::RGB);

    camRgb->isp.link(rgbOut->input);
    left->out.link(stereo->left);
    right->out.link(stereo->right);

    controlIn->out.link(camRgb->inputControl);

    stereo->depth.link(depthOut->input);

    dai::Device device(pipeline);

    // Sets queues size and behavior
    for (const auto &name : queueNames)
    {
        device.getOutputQueue(name, 4, false);
    }

    std::unordered_map<std::string, cv::Mat> frame;
    sensor_msgs::ImagePtr msg;

    std::cout << "Camera node started" << std::endl;

    while (ros::ok())
    {
        std::unordered_map<std::string, std::shared_ptr<dai::ImgFrame>> latestPacket;

        auto queueEvents = device.getQueueEvents(queueNames);
        for (const auto &name : queueEvents)
        {
            auto packets = device.getOutputQueue(name)->tryGetAll<dai::ImgFrame>();
            auto count = packets.size();
            if (count > 0)
            {
                latestPacket[name] = packets[count - 1];
            }
        }

        for (const auto &name : queueNames)
        {
            if (latestPacket.find(name) != latestPacket.end())
            {
                if (name == "depth")
                {
                    frame[name] = latestPacket[name]->getFrame();
                    frame[name].convertTo(frame[name], CV_16UC1);

                    msg = cv_bridge::CvImage(std_msgs::Header(), "mono16", frame[name]).toImageMsg();
                    depth_pub.publish(msg);
                }
                else
                {
                    frame[name] = latestPacket[name]->getCvFrame();
                    frame[name].convertTo(frame[name], CV_8UC3);

                    cv::Mat dst;
                    cv::flip(frame[name], dst, -1); 
                    cv::cvtColor(dst, dst, CV_BGR2RGB);

                    msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", dst).toImageMsg();
                    rgb_pub.publish(msg);
                }
            }
        }

        if(frame.find("rgb") != frame.end() && frame.find("depth") != frame.end()) {
            frame.clear();
        }
    }

    // Let ROS handle all callbacks.
    ros::spin();

    return 0;
} // end main()