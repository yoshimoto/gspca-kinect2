# gspca-kinect2
GSPCA/Kinect2 Sensor Device USB Camera Driver

## Build & install

$ make -C /lib/modules/`uname -r`/build  SUBDIRS=`pwd` SRCROOT=`pwd` modules

$ sudo /sbin/modprobe gspca_main
$ sudo /sbin/insmod ./kinect2.ko

## Usage

### Captures color video
$ ffmpeg  -framerate 30 -video_size 640x480 -i /dev/video0  test.avi
$ mplayer test.avi

### misc.
$ v4l-info /dev/video0
$ v4l-info /dev/video1
$ v4l2ucp preview
