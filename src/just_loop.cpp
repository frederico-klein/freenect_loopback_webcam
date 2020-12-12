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
//#define VIDEO_OUT "/dev/video0"

std::string default_param_video_out;

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
    //size_t written = write(output, cv_ptr->image.data, framesize);
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

    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    nh.param<std::string>("video_out", default_param_video_out, "/dev/video0");
    image_transport::Subscriber sub = it.subscribe("image_in", 1, imageCallback);

    //opoening video output
    output = open(default_param_video_out.c_str(), O_RDWR);
    if(output < 0) {
        std::cerr << "ERROR: could not open output device!\n" <<
        strerror(errno); return -2;
    }


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
    //if I do this I can have other people handle the conversion for me.

    //vid_format.fmt.pix.pixelformat = V4L2_PIX_FMT_BGR24;
    //vid_format.fmt.pix.pixelformat = V4L2_PIX_FMT_XBGR32;


    vid_format.fmt.pix.field = V4L2_FIELD_NONE;

    //so i forgot some stuff and maybe zoom is complaing about this
    //https://www.kernel.org/doc/html/v4.10/media/uapi/v4l/pixfmt-002.html#c.v4l2_pix_format
    vid_format.fmt.pix.bytesperline = 640;
    vid_format.fmt.pix.sizeimage = framesize;
    vid_format.fmt.pix.colorspace = V4L2_COLORSPACE_DEFAULT;
    //vid_format.fmt.pix.colorspace = V4L2_COLORSPACE_SRGB; //since we are emulating a webcam...
    //vid_format.fmt.pix.colorspace = V4L2_COLORSPACE_REC709;
    //vid_format.fmt.pix.colorspace = V4L2_COLORSPACE_RAW;
    // vid_format.fmt.pix.priv = ;
    // vid_format.fmt.pix.flags = ;
    // vid_format.fmt.pix.ycvcr_enc = V4L2_YCBCR_ENC_DEFAULT;
    // vid_format.fmt.pix.hsv_enc = V4L2_HSV_ENC_180;
    // vid_format.fmt.pix.hsv_enc = V4L2_HSV_ENC_256;
    vid_format.fmt.pix.quantization = V4L2_QUANTIZATION_DEFAULT;
    //vid_format.fmt.pix.quantization = V4L2_QUANTIZATION_FULL_RANGE;
    //vid_format.fmt.pix.quantization = V4L2_QUANTIZATION_LIM_RANGE;
    vid_format.fmt.pix.xfer_func = V4L2_XFER_FUNC_DEFAULT;
    //vid_format.fmt.pix.xfer_func = V4L2_XFER_FUNC_709;
    //vid_format.fmt.pix.xfer_func = V4L2_XFER_FUNC_SRGB;
    //vid_format.fmt.pix.xfer_func = V4L2_XFER_FUNC_ADOBERGB;
    //vid_format.fmt.pix.xfer_func = V4L2_XFER_FUNC_SMPTE240M;
    //vid_format.fmt.pix.xfer_func = V4L2_XFER_FUNC_NONE;
    //vid_format.fmt.pix.xfer_func = V4L2_XFER_FUNC_DCI_P3;
    //vid_format.fmt.pix.xfer_func = V4L2_XFER_FUNC_SMPTE2084;

    if (ioctl(output, VIDIOC_S_FMT, &vid_format) < 0) {
        std::cerr << "ERROR: unable to set video format!\n" <<
        strerror(errno); return -1;
    }

    // loop over these actions:
    ros::spin();

    std::cout << "\n\nFinish, bye!\n";
}
