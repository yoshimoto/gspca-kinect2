/*
 * kinect2 sensor device camera, gspca driver
 *
 * Copyright (C) 2015  Hiromasa Yoshimoto <hrmsysmt@gmail.com>
 *
 * Based on the OpenKinect project and libfreenect2
 * http://openkinect.org/wiki/Init_Analysis
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#define MODULE_NAME "kinect2"

#include <media/v4l2-ioctl.h>
#include "gspca.h"

#include "kinect2.h"

MODULE_AUTHOR("Hiromasa YOSHIMOTO <hrmsysmt@gmail.com>");
MODULE_DESCRIPTION("GSPCA/Kinect2 Sensor Device USB Camera Driver");
MODULE_LICENSE("GPL");


#define COLOR_IF 0
#define DEPTH_IF 1

/*
 * based on the OpenKinect project and libfreenect2
 */
#define REQUEST_MAGIC   0x06022009
#define RESPONSE_MAGIC  0x0A6FE000

#define KCMD_READ_FIRMWARE_VERSIONS 0x02
#define KCMD_INIT_STREAMS 0x09
#define KCMD_READ_DATA_0x14 0x14
#define KCMD_READ_STATUS 0x16
#define KCMD_READ_DATA_PAGE 0x22
#define KCMD_READ_DATA_0x26 0x26

#define KCMD_SET_STREAMING 0x2B
#define KCMD_SET_MODE 0x4B

#define KCMD_0x46 0x46
#define KCMD_0x47 0x47

#define KCMD_0x0A 0x0A

#define BULK_SIZE 0x8000

#define CHECK(r) do { if (!(r)) PDEBUG(D_USBO, __FILE__":%d failed\n", __LINE__); } while (0)

struct request {
    u32 magic;
    u32 cmdseq;
    u32 reply_len;
    u32 cmd;
    u32 reserved0;
    u32 param[8];
} __attribute__((packed));

/* specific camera descriptor */
struct sd {
     struct gspca_dev gspca_dev; /* !! must be the first item */

     u32 cmdseq;              /* a sequence number for control commands */
     struct request request;  /* a buffer for sending control commands */
     u32 response[32];        /* a buffer for receiving response */
     u8  synced;              /* used by sd_depth_pkt_scan() */

     struct v4l2_ioctl_ops ioctl_ops;
};

/* Grey bit-packed formats */
#define V4L2_PIX_FMT_Y11BPACK    v4l2_fourcc('Y', '1', '1', 'B')

static const struct v4l2_pix_format color_mode[] = {
     {1920, 1080, V4L2_PIX_FMT_JPEG, V4L2_FIELD_NONE,
      .bytesperline = 1920,
      .sizeimage = 1920 * 1080,
      .colorspace = V4L2_COLORSPACE_JPEG},
};

static const struct v4l2_pix_format depth_mode[] = {
     {512, 424, V4L2_PIX_FMT_Y11BPACK, V4L2_FIELD_NONE,
      .bytesperline = 512 * 11 / 8,
      .sizeimage = KINECT2_DEPTH_FRAME_SIZE*10,
      .colorspace = V4L2_COLORSPACE_SRGB},
};

static const u8 depth_rates[] = {30};
static const u8 color_rates[] = {30};
static const struct framerates color_framerates[] = {
	{ 	.rates = color_rates,
		.nrates = ARRAY_SIZE(color_rates),
	},
};
static const struct framerates depth_framerates[] = {
	{ 	.rates = depth_rates,
		.nrates = ARRAY_SIZE(depth_rates),
	},
};

/*
static void debug_dump(void *ptr, int len)
{
     u8 *p = (u8*)ptr;
     int i;
     for (i=0; i<len; i+=8) {
	  pr_warn("0x%04x: %02x %02x %02x %02x %02x %02x %02x %02x\n", i,
		  p[i+0], p[i+1], p[i+2], p[i+3],
		  p[i+4], p[i+5], p[i+6], p[i+7]);
     }
     }*/

static void sd_stopN(struct gspca_dev *gspca_dev);
static long sd_private_ioctl(struct file *file, void *fh,
			     bool valid_prio, unsigned int cmd, void *arg);

/**
 * @retval >=0 length of response, <0 error code
 */
static int send_cmd(struct gspca_dev *gspca_dev, u32 cmd,
		    const u32 *param, unsigned int num_param,
		    void *replybuf, unsigned int reply_len)
{
	struct sd *sd = (struct sd *) gspca_dev;
	struct usb_device *udev = gspca_dev->dev;
	struct request *req = &sd->request;
	int actual_len, result = 0;
	int res, i;

	if (num_param > ARRAY_SIZE(req->param)) {
	     PDEBUG(D_USBO, "send_cmd: too many params (%d)\n", num_param);
	     return -1;
	}

	req->magic = cpu_to_le32(REQUEST_MAGIC);
	req->cmdseq = cpu_to_le32(sd->cmdseq);
	req->reply_len = cpu_to_le32(reply_len);
	req->cmd = cpu_to_le32(cmd);
	for (i=0; i<num_param; ++i)
	     req->param[i] = cpu_to_le32(param[i]);

	res = usb_bulk_msg(udev, usb_sndbulkpipe(udev, 0x002),
			   req, sizeof(*req) - sizeof(req->param) + num_param*4,
			   &actual_len, USB_CTRL_SET_TIMEOUT);
	/*PDEBUG(D_USBO, "Control cmd=%04x seq=%04x len=%04x actual_len:%04x: %d", cmd,
	  sd->cmdseq, num_param, actual_len, res);*/
	if (res < 0) {
	     PDEBUG(D_USBO, "send_cmd: send failed (%d)\n", res);
	     return res;
	}

	if (reply_len) {
	     res = usb_bulk_msg(udev, usb_rcvbulkpipe(udev, 0x081),
				replybuf, reply_len,
				&actual_len, USB_CTRL_SET_TIMEOUT);
	     if (res < 0) {
		  PDEBUG(D_USBO, "send_cmd: recv failed (%d)\n", res);
		  return res;
	     }
	     result = actual_len;
	}

	/* Received completion packet */
	res = usb_bulk_msg(udev, usb_rcvbulkpipe(udev, 0x081),
			   sd->response, sizeof(sd->response),
			   &actual_len, USB_CTRL_SET_TIMEOUT);
	if (res < 0) {
	     PDEBUG(D_USBO, "send_cmd: read failed (%d)\n", res);
	     return res;
	}

	if (cpu_to_le32(RESPONSE_MAGIC) != sd->response[0]) {
	     PDEBUG(D_USBO, "send_cmd: Bad magic %08x\n", sd->response[0]);
	     return -1;
	} else if (cpu_to_le32(sd->cmdseq) != sd->response[1]) {
	     PDEBUG(D_USBO, "send_cmd: Bad cmd seq %08x\n", sd->response[1]);
	     return -1;
	}

	sd->cmdseq++;

	return result;
}

static inline void sd_color_pkt_scan(struct gspca_dev *gspca_dev,
				     u8 *data, int datalen)
{
    int type;

    if (gspca_dev->image_len == 0) {
	struct kinect2_color_header *h = (struct kinect2_color_header*)data;
	if (0x42424242 != h->magic) {
	    PDEBUG(D_STREAM, "bad magic\n");
	    return ;
	}
    }

    if (BULK_SIZE != datalen) 
	type = LAST_PACKET;
    else 
	type = gspca_dev->image_len?INTER_PACKET:FIRST_PACKET;

    gspca_frame_add(gspca_dev, type, data, datalen);
}

static inline void sd_depth_pkt_scan(struct gspca_dev *gspca_dev, u8 *data, int datalen)
{
     struct sd *sd = (struct sd *) gspca_dev;

     PDEBUG(D_STREAM, "pktscan\n");

     if (gspca_dev->pkt_size != datalen) {
	  struct kinect2_depth_footer *f;
	  f = (struct kinect2_depth_footer*)(data + datalen - sizeof(*f));
	  if (0x00 != f->magic0) {
	       PDEBUG(D_PACK, " bad footer %d/%d\n", datalen, gspca_dev->pkt_size);
	       goto discard;
	  } else if (KINECT2_DEPTH_IMAGE_SIZE != f->length) {
	       PDEBUG(D_PACK, " wrong length\n");
	       goto discard;
	  } else {
	       if (sd->synced) {
		    gspca_frame_add(gspca_dev,
				    (9==f->subsequence)?LAST_PACKET:INTER_PACKET,
				    data, datalen);
	       } else {
		    if (9==f->subsequence) 
			 sd->synced = 1;
	       }
	  }
     } else {
	  if (sd->synced) {
	       gspca_frame_add(gspca_dev,
			       (0==gspca_dev->image_len)?FIRST_PACKET:INTER_PACKET,
			       data, datalen);
	  } else {
	       goto discard;
	  }
     }
     return;

discard:
     /* Discard data until a new frame starts. */
     gspca_dev->last_packet_type = DISCARD_PACKET;
     sd->synced = 0;
}

static inline void sd_pkt_scan(struct gspca_dev *gspca_dev, u8 *data, int datalen)
{
    switch (gspca_dev->iface) {
    case COLOR_IF: sd_color_pkt_scan(gspca_dev, data, datalen); break;
    case DEPTH_IF: sd_depth_pkt_scan(gspca_dev, data, datalen); break;
    }
}

static int
get_iso_max_packet_size(struct gspca_dev *gspca_dev,
			int iface, int alt, int endpoint)
{
     int i;
     int sz = 1024;
     struct usb_interface *intf;
     struct usb_host_interface *host;
     intf = usb_ifnum_to_if(gspca_dev->dev, iface);
     if (!intf) {
	  PDEBUG(D_PROBE, "usb_ifnum_to_if(%d) failed", iface);
	  goto exit;
     }
     host = usb_altnum_to_altsetting(intf, alt);
     if (!host) {
	  PDEBUG(D_PROBE, "usb_altnum_to_altsetting(%d,%d) failed", iface, alt);
	  goto exit;
     }
     for (i = 0; i < host->desc.bNumEndpoints; ++i) {
	  if (host->endpoint[i].desc.bEndpointAddress == endpoint &&
	      (host->endpoint[i].desc.bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) == USB_ENDPOINT_XFER_ISOC) {
	       sz = host->endpoint[i].ss_ep_comp.wBytesPerInterval;
	  }
     }
exit:
     return sz;
}

static int suspend_interface(struct usb_device *udev, int suspend) {
     return usb_control_msg(udev, usb_sndctrlpipe(udev, 0),
			    USB_REQ_SET_FEATURE,
			    USB_RECIP_INTERFACE,
			    USB_INTRF_FUNC_SUSPEND,
			    (suspend)?(USB_INTRF_FUNC_SUSPEND_LP|
				       USB_INTRF_FUNC_SUSPEND_RW):0, NULL, 0,
			    USB_CTRL_SET_TIMEOUT);
}

static int power_device(struct usb_device *udev, int onoff) {
     u8 request = onoff?USB_REQ_SET_FEATURE:USB_REQ_CLEAR_FEATURE;

     int r = usb_control_msg(udev, usb_sndctrlpipe(udev, 0),
			     request,
			     USB_RECIP_DEVICE,
			     USB_DEVICE_U1_ENABLE,
			     0, NULL, 0,
			     USB_CTRL_SET_TIMEOUT);
     if (0==r) {
	  r = usb_control_msg(udev, usb_sndctrlpipe(udev, 0),
			      request,
			      USB_RECIP_DEVICE,
			      USB_DEVICE_U2_ENABLE,
			      0, NULL, 0,
			      USB_CTRL_SET_TIMEOUT);
     }
     return r;
}

// if no super speed hubs in between, then it is equal to tTPTransmissionDelay(=40ns)
static int set_isochronous_delay(struct usb_device *udev, int nanosec)
{
     // for details see USB 3.1 r1 spec section 9.4.11
     return usb_control_msg(udev, usb_sndctrlpipe(udev,0),
			    USB_REQ_SET_ISOCH_DELAY, USB_RECIP_DEVICE,
			    nanosec, 0,
			    NULL, 0, USB_CTRL_SET_TIMEOUT);
}


/* This function is called at probe time just before sd_init */
static int sd_config(struct gspca_dev *gspca_dev,
		     const struct usb_device_id *id)
{
     
     struct sd *sd = (struct sd *) gspca_dev;
     struct cam *cam = &gspca_dev->cam;

     sd->cmdseq = 0;

     switch (gspca_dev->iface) {
     case COLOR_IF:
	  cam->cam_mode = color_mode;
	  cam->mode_framerates = color_framerates;
	  cam->nmodes = ARRAY_SIZE(color_mode);

	  /* setup bulk transfer */
	  cam->bulk = 1;
	  cam->bulk_size = BULK_SIZE;
	  cam->bulk_nurbs = MAX_NURBS;
	  gspca_dev->xfer_ep = 0x083;
	  /*cam->needs_full_bandwidth = 1;*/
	  break;

     case DEPTH_IF:
	  cam->cam_mode = depth_mode;
	  cam->mode_framerates = depth_framerates;
	  cam->nmodes = ARRAY_SIZE(depth_mode);
	  /* setup isoc transfer */
	  gspca_dev->xfer_ep = 0x084;
	  gspca_dev->pkt_size = get_iso_max_packet_size(gspca_dev,
							DEPTH_IF, 1, 0x84);
	  PDEBUG(D_PROBE, "isoc packet size: %d", gspca_dev->pkt_size);
	  cam->bulk = 0;
	  cam->npkt = 16;
	  cam->needs_full_bandwidth = 1;
	  break;
     default:
	  PDEBUG(D_PROBE, "iface is %d, 0 or 1 expected\n", gspca_dev->iface);
	  return -1;
     }

     /* Replaces vdev.ioctl_ops to override vidioc_default() */
     memcpy(&sd->ioctl_ops, gspca_dev->vdev.ioctl_ops, sizeof(sd->ioctl_ops));
     sd->ioctl_ops.vidioc_default = sd_private_ioctl;
     gspca_dev->vdev.ioctl_ops = &sd->ioctl_ops;

     return 0;
}


/* this function is called at probe and resume time */
static int sd_init(struct gspca_dev *gspca_dev)
{
     struct usb_device *udev = gspca_dev->dev;
     int r;

     PDEBUG(D_PROBE, "init; iface: %d\n", gspca_dev->iface);

     r = power_device(udev, 1);
     CHECK(r==0);
     r = suspend_interface(udev, 1);
     CHECK(r==0);

     return 0;
}


static int sd_isoc_init(struct gspca_dev *gspca_dev)
{
     int r;
     struct usb_device *udev = gspca_dev->dev;
     PDEBUG(D_PROBE, "isoc_init; %d\n", gspca_dev->iface);
     switch (gspca_dev->iface) {
     case DEPTH_IF:
	 r = set_isochronous_delay(udev, 40);
	 CHECK(r==0);

	 break;
     }
     return 0;
}


static int sd_start(struct gspca_dev *gspca_dev)
{
     struct usb_device *udev = gspca_dev->dev;
     struct sd *sd = (struct sd *) gspca_dev;
     int r;
     static const u32 start_cmd = 0x01;

     sd->synced = 0;
     PDEBUG(D_PROBE, "sd_start iface:%d\n", gspca_dev->iface);

     r = power_device(udev, 1);
     CHECK(r==0);
     r = suspend_interface(udev, 0);
     CHECK(r==0);

     r = send_cmd(gspca_dev, KCMD_INIT_STREAMS, NULL, 0, NULL, 0);
     CHECK(0==r);
     r = send_cmd(gspca_dev, KCMD_SET_STREAMING, &start_cmd, 1, NULL, 0);
     CHECK(0==r);

     switch (gspca_dev->iface) {
     case COLOR_IF:
	  r = send_cmd(gspca_dev, KCMD_0x0A, NULL, 0, NULL, 0);
	  CHECK(0==r);
	  break;
     }

     return 0;
}


static void sd_stopN(struct gspca_dev *gspca_dev)
{
     int r;
     static const u32 stop_cmd = 0x00;

     PDEBUG(D_PROBE, "Kinect2 stopN; iface: %d\n", gspca_dev->iface);

     r = send_cmd(gspca_dev, KCMD_SET_STREAMING, &stop_cmd, 1, NULL, 0);
     CHECK(0==r);
}

static void sd_stop0(struct gspca_dev *gspca_dev)
{
     struct usb_device *udev = gspca_dev->dev;
     int r;
     r = suspend_interface(udev, 1);
     CHECK(r==0);
     r = power_device(udev, 0);
     CHECK(r==0);

}

static long sd_private_ioctl(struct file *file, void *fh,
			     bool valid_prio, unsigned int cmd, void *arg)
{
     struct gspca_dev *gspca_dev = video_drvdata(file);

     static const struct entry {
	 u32 cmd;
	 int len;
     } table[] = {
	 {0x04, sizeof(struct kinect2_color_camera_param)},
	 {0x03, sizeof(struct kinect2_depth_camera_param)},
	 {0x02, sizeof(struct kinect2_p0table)},
     };

     int r;
     void *buf = NULL;
     struct kinect2_ioctl_req * req = arg;
     int num = _IOC_NR(cmd) - BASE_VIDIOC_PRIVATE;

     if (num < 0 || ARRAY_SIZE(table) <= num) {
	 r = -EOPNOTSUPP;
	 goto out;
     }
     if (req->len != table[num].len) {
	 r = -EINVAL;
	 goto out;
     }

     buf = kmalloc(table[num].len, GFP_KERNEL);
     if (!buf) {
	 r = -ENOMEM;
	 goto out;
     }

     r = send_cmd(gspca_dev, KCMD_READ_DATA_PAGE, &table[num].cmd, 1,
		  buf, table[num].len);
     if (table[num].len == r) {
	  r = copy_to_user((void __user *)req->ptr, buf, table[num].len);
	  if (r) {
	      PDEBUG(D_PROBE, "copy_to_user() failed\n");
	      r = -EFAULT;
	  }
     } else {
	 PDEBUG(D_PROBE, "send_cmd() returns %d, expected %d",
		r, table[num].len);
	 r = -EFAULT;
     }
out:
     if (buf)
	  kfree(buf);
     return r;
}


/* sub-driver description */
static const struct sd_desc sd_desc = {
	.name      = MODULE_NAME,
	.config    = sd_config,
	.init      = sd_init,
	.isoc_init = sd_isoc_init,
	.start     = sd_start,
	.stopN     = sd_stopN,
	.stop0     = sd_stop0,
	.pkt_scan  = sd_pkt_scan,
};

static const struct usb_device_id device_table[] = {
     /* kinect for windows 2 */
     {USB_DEVICE(0x045e, 0x02d8)},
     /* kinect for windows 2 preview? */
     {USB_DEVICE(0x045e, 0x02c4)}, 
     {}
};

MODULE_DEVICE_TABLE(usb, device_table);

static int sd_probe(struct usb_interface *intf, const struct usb_device_id *id)
{
     return gspca_dev_probe2(intf, id, &sd_desc,
			     sizeof(struct sd), THIS_MODULE);
}

static struct usb_driver sd_driver = {
     .name       = KBUILD_MODNAME,
     .id_table   = device_table,
     .probe      = sd_probe,
     .disconnect = gspca_disconnect,
#ifdef CONFIG_PM
     .suspend    = gspca_suspend,
     .resume     = gspca_resume,
     .reset_resume = gspca_resume,
#endif
};

module_usb_driver(sd_driver);
