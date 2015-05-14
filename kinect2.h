/*
 * This file holds Kinect2-related constans and structures.
 * These are based on the OpenKinect project and libfreenect2.
 * See the following URLs for details;
 * - http://openkinect.org/wiki/
 *
 * Copyright (C) 2015  Hiromasa Yoshimoto <hrmsysmt@gmail.com>
 */

#ifndef _UAPI__LINUX_KINECT2_H
#define _UAPI__LINUX_KINECT2_H

#include <linux/types.h> /* __u8 etc */

struct kinect2_color_camera_param
{
     /* unknown, always seen as 1 so far */
     __u8 table_id;

     /* color -> depth mapping parameters */
     float f;
     float cx;
     float cy;

     float shift_d;
     float shift_m;

     float mx_x3y0; /* xxx */
     float mx_x0y3; /* yyy */
     float mx_x2y1; /* xxy */
     float mx_x1y2; /* yyx */
     float mx_x2y0; /* xx */
     float mx_x0y2; /* yy */
     float mx_x1y1; /* xy */
     float mx_x1y0; /* x */
     float mx_x0y1; /* y */
     float mx_x0y0; /* 1 */

     float my_x3y0; /* xxx */
     float my_x0y3; /* yyy */
     float my_x2y1; /* xxy */
     float my_x1y2; /* yyx */
     float my_x2y0; /* xx */
     float my_x0y2; /* yy */
     float my_x1y1; /* xy */
     float my_x1y0; /* x */
     float my_x0y1; /* y */
     float my_x0y0; /* 1 */

     /* perhaps related to xtable/ztable in the deconvolution code. */
     /* data seems to be arranged into two tables of 28*23, which */
     /* matches the depth image aspect ratio of 512*424 very closely */
     float table1[28 * 23 * 4];
     float table2[28 * 23];
} __attribute__((packed));


/* Depth camera's intrinsic & distortion parameters */
struct kinect2_depth_camera_param
{
     /* intrinsics (this is pretty certain) */
     float fx;
     float fy;
     float unknown0; /* assumed to be always zero */
     float cx;
     float cy;

     /* radial distortion (educated guess based on calibration data from Kinect SDK) */
     float k1;
     float k2;
     float p1; /* always seen as zero so far, so purely a guess */
     float p2; /* always seen as zero so far, so purely a guess */
     float k3;

     float unknown1[13]; /* assumed to be always zero */
} __attribute__((packed));


/* "P0" coefficient tables, input to the deconvolution code */
struct kinect2_p0table
{
     __u32 headersize;
     __u32 unknown1;
     __u32 unknown2;
     __u32 tablesize;
     __u32 unknown3;
     __u32 unknown4;
     __u32 unknown5;
     __u32 unknown6;

     __u16 unknown7;
     __u16 p0table0[512*424]; /* row[0] == row[511] == 0x2c9a */
     __u16 unknown8;

     __u16 unknown9;
     __u16 p0table1[512*424]; /* row[0] == row[511] == 0x08ec */
     __u16 unknownA;

     __u16 unknownB;
     __u16 p0table2[512*424]; /* row[0] == row[511] == 0x42e8 */
     __u16 unknownC;

     __u8  unknownD[];
} __attribute__((packed));

struct kinect2_depth_footer {
     __u32 magic0;
     __u32 magic1;
     __u32 timestamp;
     __u32 sequence;
     __u32 subsequence;
     __u32 length;
     __u32 fields[32];
} __attribute__((packed));

struct kinect2_color_header {
     __u32 sequence;
     __u32 magic; /* 0x42424242 */
     __u8 image[];
} __attribute__((packed));

struct kinect2_color_footer {
     __u32 sequence;
     __u32 unknownA[3];
     __u32 timestamp; 
     __u32 unknownB[2];
     __u32 magic;       /* 0x42424242 */
     __u32 length;
     __u32 unknownC[4];
} __attribute__((packed));

#define KINECT2_DEPTH_IMAGE_SIZE (512*424*11/8)
#define KINECT2_DEPTH_FRAME_SIZE (KINECT2_DEPTH_IMAGE_SIZE + sizeof(struct kinect2_depth_footer))

#define KINECT2_GET_DEPTH_FOOTER(p) ((struct kinect2_depth_footer*)((char*)(p) + KINECT2_DEPTH_FRAME_SIZE*10 - sizeof(struct kinect2_depth_footer)))
#define KINECT2_GET_COLOR_FOOTER(ptr,len) (struct kinect2_color_footer*)((char*)ptr + (len) - sizeof(struct kinect2_color_footer))

#include <linux/videodev2.h>


struct kinect2_ioctl_req {
    int   len;
    void *ptr;
} __attribute__((packed));

#define VIDIOC_KINECT2_COLOR_PARAM _IOWR('V', BASE_VIDIOC_PRIVATE + 0, struct kinect2_ioctl_req)
#define VIDIOC_KINECT2_DEPTH_PARAM _IOWR('V', BASE_VIDIOC_PRIVATE + 1, struct kinect2_ioctl_req)
#define VIDIOC_KINECT2_P0TABLE    _IOWR('V', BASE_VIDIOC_PRIVATE + 2, struct kinect2_ioctl_req)


#endif /* _UAPI__LINUX_KINECT2_H */
