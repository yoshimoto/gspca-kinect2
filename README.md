# gspca-kinect2

Linux kernel driver for the "Kinect for Windows 2" sensor.

## Requirements ##
* Linux 3.x+

## Build & install

```
$ make -C /lib/modules/`uname -r`/build  SUBDIRS=`pwd` SRCROOT=`pwd` modules  
$ sudo /sbin/modprobe videodev
$ sudo /sbin/insmod ./gspca_main.ko  
$ sudo /sbin/insmod ./gspca_kinect2.ko  
```

## Usage

This driver provides two v4l2 interfaces per a single kinect sensor; color camera is mapped to /dev/video0, and depth camera is mapped to /dev/video1.
If you have two or more sensors, they shall be mappaed to /dev/videoX, where X are (2*n) for n-th color and (2*n+1) for n-th depth.

### Capture color video

```
$ ffmpeg  -framerate 30 -video_size 640x480 -i /dev/video0  test.avi  
$ mplayer test.avi  
```

### View live color/depth video

I wrote an open source library, named libk4w2. See https://github.com/yoshimoto/libk4w2/ for details.

### Misc

```
$ v4l-info /dev/video0  
$ v4l-info /dev/video1  
$ v4l2ucp preview  
```

# Acknowledgements

This driver is based on the following discussions and source codes;
- libfreenect2, https://github.com/OpenKinect/libfreenect2
- Analyzing Kinect 2 ISO data, https://groups.google.com/forum/#!topic/openkinect/8Aa8ab3aAs4

Special thanks to the people in the OpenKinect project!!!


Hiromasa YOSHIMOTO
