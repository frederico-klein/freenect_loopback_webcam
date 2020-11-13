# freenect_loopback_webcam

This is a loopback webcam implementation of freenect using ROS and the v4l2loopback (https://github.com/umlaeute/v4l2loopback/) module. It is a basic bastardization of this very nice tutorial https://arcoresearchgroup.wordpress.com/2020/06/02/virtual-camera-for-opencv-using-v4l2loopback/

It should be straightforward to ditch ros and have this as simple driver based on libfreenect. 

This is a better version that almost corrects the color problems I had in the past, but it still has some issues. 

I believe the color conversion is incorrect, or something is incomplete, because, even though the device works in Cheese, it has wrong color (it shows as BGR) in VLC, in Zoom it shows a black screen unless adjust for low light is set to auto.

It should be a starting point for something functional though. 
