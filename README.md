# freenect_loopback_webcam

This is a loopback webcam implementation of freenect using ROS and the v4l2loopback (https://github.com/umlaeute/v4l2loopback/) module. It is a basic bastardization of this very nice tutorial https://arcoresearchgroup.wordpress.com/2020/06/02/virtual-camera-for-opencv-using-v4l2loopback/

It should be straightforward to ditch ros and have this as simple driver based on libfreenect. 

This is a better version that almost corrects the color problems I had in the past, but it still has some issues. 

I believe the color conversion is incorrect, or something is incomplete, because, even though the device works in Cheese, it has wrong color (it shows as BGR) in VLC, in Zoom it shows a black screen unless adjust for low light is set to auto.

It should be a starting point for something functional though. 


## getting started

### Step one: Install v4l2loopback somehow.

In ubuntu it is `sudo apt-get install v4l2loopback`.

Run it:

    sudo modprobe v4l2loopback devices=2 card_label="Test source, ROS fake camera"
    
*Note you need to add 2 devices here, or zoom, for instance will not show it. No idea why, probably has to do with some incomplete setup.

Test if it worked with:

    gst-launch-1.0 videotestsrc ! v4l2sink device=/dev/video0

Now open cheese or vlc. You should see an old style rgb test image with some fake static on one side. 

### Step two: Install ros freenect.

like `sudo apt-get install ros-<DISTRO>-freenect-stack.`
  
Check if it worked with a `source /opt/ros/<DISTRO>/setup.bash` and run the freenect with `roslaunch freenect_launch freenect.launch`. Use image_viewer or rqt_image_view and inspect /camera/rgb/image_raw. There should be an image there. 
  
### Step three: get this package and compile it.  

Create a catkin_ws directory with a src subdir. 

Do a `git clone https://github.com/frederico-klein/freenect_loopback_webcam.git`

Compile it with a `catkin_make`.   
  
## Running

You will need to have a v4l loopback device running each time. If you didn't put this on your startup you need to do it before you run this thing. 

    sudo modprobe v4l2loopback devices=2 card_label="Test source, ROS fake camera"
    
Run the freenect as well:

    roslaunch freenect_launch freenect.launch

Now run the freenect_loopback_webcam node with:

    rosrun freenect_loopback_webcam loop_node
    
    
You should have a fake kinect webcam. Note you can publish anything as an image, since we are using ros. 


