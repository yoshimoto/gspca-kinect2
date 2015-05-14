# gspca-kinect2

Linux kernel driver for the "Kinect for Windows 2" sensor.

## Requirements ##
* Linux 3.x+

## Build & install

```
$ make -C /lib/modules/`uname -r`/build  SUBDIRS=`pwd` SRCROOT=`pwd` modules  
$ sudo /sbin/modprobe gspca_main  
$ sudo /sbin/insmod ./kinect2.ko  
```

## Usage

This driver provides v4l2 interface; color stream is mapped to /dev/video0, and depth is mapped to /dev/video1.

### Captures color video

```
$ ffmpeg  -framerate 30 -video_size 640x480 -i /dev/video0  test.avi  
$ mplayer test.avi  
```
### misc.

```
$ v4l-info /dev/video0  
$ v4l-info /dev/video1  
$ v4l2ucp preview  
```
