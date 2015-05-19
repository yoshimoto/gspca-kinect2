# gspca-kinect2

Linux kernel driver for the "Kinect for Windows 2" sensor.

## requirements ##
* Linux 3.x+

## build & install

```
$ make -C /lib/modules/`uname -r`/build  SUBDIRS=`pwd` SRCROOT=`pwd` modules  
$ sudo /sbin/insmod ./gspca_main.ko  
$ sudo /sbin/insmod ./gspca_kinect2.ko  
```

## usage

This driver provides v4l2 interface; color stream is mapped to /dev/video0, and depth is mapped to /dev/video1.
If you have two or more devices, they shall be mappaed to /dev/videoX, where X are (2*n) for n-th color stream
and (2*n+1) for n-th depth stream.

### capture color video

```
$ ffmpeg  -framerate 30 -video_size 640x480 -i /dev/video0  test.avi  
$ mplayer test.avi  
```
### misc

```
$ v4l-info /dev/video0  
$ v4l-info /dev/video1  
$ v4l2ucp preview  
```

# acknowledgements

This driver is based on libfreenect2

Special thanks to the people in the OpenKinect project!!!
