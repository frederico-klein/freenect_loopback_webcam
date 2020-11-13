#include <opencv2/opencv.hpp>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#ifndef VID_WIDTH
#define VID_WIDTH 640
#endif

#ifndef VID_HEIGHT
#define VID_HEIGHT 480
#endif

//#define VIDEO_IN  "/dev/video0"
#define VIDEO_OUT "/dev/video0"

cv_bridge::CvImagePtr cv_ptr;

// open output device needs to be shared
int output;
size_t framesize = VID_WIDTH * VID_HEIGHT * 3;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    // convert  because we need RGB24 and we have BGR
    cv::Mat result;
    cv::cvtColor(cv_ptr->image, result, cv::COLOR_BGR2RGB);

    // write frame to output device
    size_t written = write(output, result.data, framesize);
    if (written < 0) {
        std::cerr << "ERROR: could not write to output device!\n";
        close(output);
    }
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int
main(int argc, char* argv[]) {

    //opoening video output
    output = open(VIDEO_OUT, O_RDWR);
    if(output < 0) {
        std::cerr << "ERROR: could not open output device!\n" <<
        strerror(errno); return -2;
    }

    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/camera/rgb/image_raw", 1, imageCallback);

    // configure params for output device
    struct v4l2_format vid_format;
    memset(&vid_format, 0, sizeof(vid_format));
    vid_format.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;

    if (ioctl(output, VIDIOC_G_FMT, &vid_format) < 0) {
        std::cerr << "ERROR: unable to get video format!\n" <<
        strerror(errno); return -1;
    }

    //erm this is wrong. fixit
    vid_format.fmt.pix.width = 640;//cam.get(cv::CAP_PROP_FRAME_WIDTH);
    vid_format.fmt.pix.height = 480;//cam.get(cv::CAP_PROP_FRAME_HEIGHT);

    // NOTE: change this according to below filters...
    // Chose one from the supported formats on Chrome:
    // - V4L2_PIX_FMT_YUV420,
    // - V4L2_PIX_FMT_Y16,
    // - V4L2_PIX_FMT_Z16,
    // - V4L2_PIX_FMT_INVZ,
    // - V4L2_PIX_FMT_YUYV,
    // - V4L2_PIX_FMT_RGB24,
    // - V4L2_PIX_FMT_MJPEG,
    // - V4L2_PIX_FMT_JPEG
    vid_format.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB24;

    vid_format.fmt.pix.sizeimage = framesize;
    vid_format.fmt.pix.field = V4L2_FIELD_NONE;

    if (ioctl(output, VIDIOC_S_FMT, &vid_format) < 0) {
        std::cerr << "ERROR: unable to set video format!\n" <<
        strerror(errno); return -1;
    }

    // loop over these actions:
    ros::spin();

    std::cout << "\n\nFinish, bye!\n";
}
