/***************************************************************************
 * Plug-in for OV7648 image sensor connected to the SN9C1xx PC Camera      *
 * Controllers                                                             *
 *                                                                         *
 * Copyright (C) 2007-2008 by Luca Risolia <luca.risolia@studio.unibo.it>  *
 *                                                                         *
 * This program is free software; you can redistribute it and/or modify    *
 * it under the terms of the GNU General Public License as published by    *
 * the Free Software Foundation; either version 2 of the License, or       *
 * (at your option) any later version.                                     *
 *                                                                         *
 * This program is distributed in the hope that it will be useful,         *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of          *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the           *
 * GNU General Public License for more details.                            *
 *                                                                         *
 * You should have received a copy of the GNU General Public License       *
 * along with this program; if not, write to the Free Software             *
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.               *
 ***************************************************************************/

#include "sn9c102_sensor.h"


static int ov7648_init(struct sn9c102_device* cam)
{
	int err = 0;

	err = sn9c102_write_const_regs(cam, {0x40, 0x02}, {0x00, 0x03},
	                               {0x1a, 0x04}, {0x03, 0x10},
	                               {0x0a, 0x14}, {0xe2, 0x17},
	                               {0x0b, 0x18}, {0x00, 0x19},
	                               {0x1d, 0x1a}, {0x10, 0x1b},
	                               {0x02, 0x1c}, {0x03, 0x1d},
	                               {0x0f, 0x1e}, {0x0c, 0x1f},
	                               {0x00, 0x20}, {0x24, 0x21},
	                               {0x3b, 0x22}, {0x47, 0x23},
	                               {0x60, 0x24}, {0x71, 0x25},
	                               {0x80, 0x26}, {0x8f, 0x27},
	                               {0x9d, 0x28}, {0xaa, 0x29},
	                               {0xb8, 0x2a}, {0xc4, 0x2b},
	                               {0xd1, 0x2c}, {0xdd, 0x2d},
	                               {0xe8, 0x2e}, {0xf4, 0x2f},
	                               {0xff, 0x30}, {0x00, 0x3f},
	                               {0xc7, 0x40}, {0x01, 0x41},
	                               {0x44, 0x42}, {0x00, 0x43},
	                               {0x44, 0x44}, {0x00, 0x45},
	                               {0x44, 0x46}, {0x00, 0x47},
	                               {0xc7, 0x48}, {0x01, 0x49},
	                               {0xc7, 0x4a}, {0x01, 0x4b},
	                               {0xc7, 0x4c}, {0x01, 0x4d},
	                               {0x44, 0x4e}, {0x00, 0x4f},
	                               {0x44, 0x50}, {0x00, 0x51},
	                               {0x44, 0x52}, {0x00, 0x53},
	                               {0xc7, 0x54}, {0x01, 0x55},
	                               {0xc7, 0x56}, {0x01, 0x57},
	                               {0xc7, 0x58}, {0x01, 0x59},
	                               {0x44, 0x5a}, {0x00, 0x5b},
	                               {0x44, 0x5c}, {0x00, 0x5d},
	                               {0x44, 0x5e}, {0x00, 0x5f},
	                               {0xc7, 0x60}, {0x01, 0x61},
	                               {0xc7, 0x62}, {0x01, 0x63},
	                               {0xc7, 0x64}, {0x01, 0x65},
	                               {0x44, 0x66}, {0x00, 0x67},
	                               {0x44, 0x68}, {0x00, 0x69},
	                               {0x44, 0x6a}, {0x00, 0x6b},
	                               {0xc7, 0x6c}, {0x01, 0x6d},
	                               {0xc7, 0x6e}, {0x01, 0x6f},
	                               {0xc7, 0x70}, {0x01, 0x71},
	                               {0x44, 0x72}, {0x00, 0x73},
	                               {0x44, 0x74}, {0x00, 0x75},
	                               {0x44, 0x76}, {0x00, 0x77},
	                               {0xc7, 0x78}, {0x01, 0x79},
	                               {0xc7, 0x7a}, {0x01, 0x7b},
	                               {0xc7, 0x7c}, {0x01, 0x7d},
	                               {0x44, 0x7e}, {0x00, 0x7f},
	                               {0x17, 0x84}, {0x00, 0x85},
	                               {0x2e, 0x86}, {0x00, 0x87},
	                               {0x09, 0x88}, {0x00, 0x89},
	                               {0xe8, 0x8a}, {0x0f, 0x8b},
	                               {0xda, 0x8c}, {0x0f, 0x8d},
	                               {0x40, 0x8e}, {0x00, 0x8f},
	                               {0x37, 0x90}, {0x00, 0x91},
	                               {0xcf, 0x92}, {0x0f, 0x93},
	                               {0xfa, 0x94}, {0x0f, 0x95},
	                               {0x00, 0x96}, {0x00, 0x97},
	                               {0x00, 0x98}, {0x66, 0x99},
	                               {0x00, 0x9a}, {0x40, 0x9b},
	                               {0x20, 0x9c}, {0x00, 0x9d},
	                               {0x00, 0x9e}, {0x00, 0x9f},
	                               {0x2d, 0xc0}, {0x2d, 0xc1},
	                               {0x3a, 0xc2}, {0x00, 0xc3},
	                               {0x04, 0xc4}, {0x3f, 0xc5},
	                               {0x00, 0xc6}, {0x00, 0xc7},
	                               {0x50, 0xc8}, {0x3c, 0xc9},
	                               {0x28, 0xca}, {0xd8, 0xcb},
	                               {0x14, 0xcc}, {0xec, 0xcd},
	                               {0x32, 0xce}, {0xdd, 0xcf},
	                               {0x32, 0xd0}, {0xdd, 0xd1},
	                               {0x6a, 0xd2}, {0x50, 0xd3},
	                               {0x60, 0xd4}, {0x00, 0xd5},
	                               {0x00, 0xd6});

	err += sn9c102_i2c_write(cam, 0x12, 0x80);
	err += sn9c102_i2c_write(cam, 0x12, 0x0c);
	err += sn9c102_i2c_write(cam, 0x19, 0x02);
	err += sn9c102_i2c_write(cam, 0x28, 0xa2);
	err += sn9c102_i2c_write(cam, 0x03, 0xa4);
	err += sn9c102_i2c_write(cam, 0x04, 0x30);
	err += sn9c102_i2c_write(cam, 0x05, 0x88);
	err += sn9c102_i2c_write(cam, 0x06, 0x60);
	err += sn9c102_i2c_write(cam, 0x24, 0xa0);
	err += sn9c102_i2c_write(cam, 0x25, 0x80);
	err += sn9c102_i2c_write(cam, 0x2a, 0x11);
	err += sn9c102_i2c_write(cam, 0x2d, 0x05);
	err += sn9c102_i2c_write(cam, 0x60, 0xa6);
	err += sn9c102_i2c_write(cam, 0x6d, 0x33);
	err += sn9c102_i2c_write(cam, 0x6e, 0x22);

	return err;
}


static int ov7648_get_ctrl(struct sn9c102_device* cam,
                           struct v4l2_control* ctrl)
{
	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		if ((ctrl->value = sn9c102_i2c_read(cam, 0x10)) < 0)
			return -EIO;
		break;
	case V4L2_CID_DO_WHITE_BALANCE:
		if ((ctrl->value = sn9c102_read_reg(cam, 0x02)) < 0)
			return -EIO;
		ctrl->value = (ctrl->value & 0x04) ? 1 : 0;
		break;
	case V4L2_CID_RED_BALANCE:
		ctrl->value = sn9c102_read_reg(cam, 0x05);
		break;
	case V4L2_CID_BLUE_BALANCE:
		ctrl->value = sn9c102_read_reg(cam, 0x06);
		break;
	case SN9C102_V4L2_CID_GREEN_BALANCE:
		ctrl->value = sn9c102_read_reg(cam, 0x07);
		break;
	case V4L2_CID_GAIN:
		if ((ctrl->value = sn9c102_i2c_read(cam, 0x00)) < 0)
			return -EIO;
		ctrl->value &= 0x3f;
		break;
	case V4L2_CID_AUTOGAIN:
		if ((ctrl->value = sn9c102_i2c_read(cam, 0x13)) < 0)
			return -EIO;
		ctrl->value &= 0x01;
		break;
	case V4L2_CID_VFLIP:
		if ((ctrl->value = sn9c102_i2c_read(cam, 0x75)) < 0)
			return -EIO;
		ctrl->value = (ctrl->value & 0x80) ? 1 : 0;
		break;
	case SN9C102_V4L2_CID_BAND_FILTER:
		if ((ctrl->value = sn9c102_i2c_read(cam, 0x2d)) < 0)
			return -EIO;
		ctrl->value = (ctrl->value & 0x02) ? 1 : 0;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}


static int ov7648_set_ctrl(struct sn9c102_device* cam,
                           const struct v4l2_control* ctrl)
{
	int err = 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		err += sn9c102_i2c_write(cam, 0x10, ctrl->value);
		break;
	case V4L2_CID_DO_WHITE_BALANCE:
		err += sn9c102_write_reg(cam, 0x43 | (ctrl->value << 2), 0x02);
		break;
	case V4L2_CID_RED_BALANCE:
		err += sn9c102_write_reg(cam, ctrl->value, 0x05);
		break;
	case V4L2_CID_BLUE_BALANCE:
		err += sn9c102_write_reg(cam, ctrl->value, 0x06);
		break;
	case SN9C102_V4L2_CID_GREEN_BALANCE:
		err += sn9c102_write_reg(cam, ctrl->value, 0x07);
		break;
	case V4L2_CID_GAIN:
		err += sn9c102_i2c_write(cam, 0x00, ctrl->value);
		break;
	case V4L2_CID_AUTOGAIN:
		err += sn9c102_i2c_write(cam, 0x13, 0xa0 | ctrl->value |
		                                    (ctrl->value << 1));
		break;
	case V4L2_CID_VFLIP:
		err += sn9c102_i2c_write(cam, 0x75, 0x0e | (ctrl->value << 7));
		break;
	case SN9C102_V4L2_CID_BAND_FILTER:
		err += sn9c102_i2c_write(cam, 0x2d, ctrl->value << 2);
		break;
	default:
		return -EINVAL;
	}

	return err ? -EIO : 0;
}


static int ov7648_set_crop(struct sn9c102_device* cam,
                           const struct v4l2_rect* rect)
{
	struct sn9c102_sensor* s = sn9c102_get_sensor(cam);
	int err = 0;
	u8 h_start = (u8)(rect->left - s->cropcap.bounds.left) + 2,
	   v_start = (u8)(rect->top - s->cropcap.bounds.top) + 1;

	err += sn9c102_write_reg(cam, h_start, 0x12);
	err += sn9c102_write_reg(cam, v_start, 0x13);

	return err;
}


static int ov7648_set_pix_format(struct sn9c102_device* cam,
                                 const struct v4l2_pix_format* pix)
{
	int err = 0;

	if (pix->pixelformat == V4L2_PIX_FMT_SBGGR8) {
		err += sn9c102_write_reg(cam, 0xe5, 0x17);
		err += sn9c102_i2c_write(cam, 0x11, 0x04);
	} else {
		err += sn9c102_write_reg(cam, 0xe2, 0x17);
		err += sn9c102_i2c_write(cam, 0x11, 0x01);
	}

	return err;
}


static const struct sn9c102_sensor ov7648 = {
	.name = "OV7648",
	.maintainer = "Luca Risolia <luca.risolia@studio.unibo.it>",
	.supported_bridge = BRIDGE_SN9C105 | BRIDGE_SN9C120,
	.sysfs_ops = SN9C102_I2C_READ | SN9C102_I2C_WRITE,
	.frequency = SN9C102_I2C_100KHZ,
	.interface = SN9C102_I2C_2WIRES,
	.i2c_slave_id = 0x21,
	.init = &ov7648_init,
	.qctrl = {
		{
			.id = V4L2_CID_GAIN,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "global gain",
			.minimum = 0x00,
			.maximum = 0x3f,
			.step = 0x01,
			.default_value = 0x14,
			.flags = 0,
		},
		{
			.id = V4L2_CID_EXPOSURE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "exposure",
			.minimum = 0x00,
			.maximum = 0xff,
			.step = 0x01,
			.default_value = 0x60,
			.flags = 0,
		},
		{
			.id = V4L2_CID_DO_WHITE_BALANCE,
			.type = V4L2_CTRL_TYPE_BOOLEAN,
			.name = "led",
			.minimum = 0x00,
			.maximum = 0x01,
			.step = 0x01,
			.default_value = 0x00,
			.flags = 0,
		},
		{
			.id = V4L2_CID_RED_BALANCE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "red balance",
			.minimum = 0x00,
			.maximum = 0x7f,
			.step = 0x01,
			.default_value = 0x20,
			.flags = 0,
		},
		{
			.id = V4L2_CID_BLUE_BALANCE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "blue balance",
			.minimum = 0x00,
			.maximum = 0x7f,
			.step = 0x01,
			.default_value = 0x20,
			.flags = 0,
		},
		{
			.id = V4L2_CID_AUTOGAIN,
			.type = V4L2_CTRL_TYPE_BOOLEAN,
			.name = "auto gain-exposure",
			.minimum = 0x00,
			.maximum = 0x01,
			.step = 0x01,
			.default_value = 0x01,
			.flags = 0,
		},
		{
			.id = V4L2_CID_VFLIP,
			.type = V4L2_CTRL_TYPE_BOOLEAN,
			.name = "vertical flip",
			.minimum = 0x00,
			.maximum = 0x01,
			.step = 0x01,
			.default_value = 0x00,
			.flags = 0,
		},
		{
			.id = SN9C102_V4L2_CID_GREEN_BALANCE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "green balance",
			.minimum = 0x00,
			.maximum = 0x7f,
			.step = 0x01,
			.default_value = 0x20,
			.flags = 0,
		},
		{
			.id = SN9C102_V4L2_CID_BAND_FILTER,
			.type = V4L2_CTRL_TYPE_BOOLEAN,
			.name = "band filter",
			.minimum = 0x00,
			.maximum = 0x01,
			.step = 0x01,
			.default_value = 0x00,
			.flags = 0,
		},
	},
	.get_ctrl = &ov7648_get_ctrl,
	.set_ctrl = &ov7648_set_ctrl,
	.cropcap = {
		.bounds = {
			.left = 0,
			.top = 0,
			.width = 640,
			.height = 480,
		},
		.defrect = {
			.left = 0,
			.top = 0,
			.width = 640,
			.height = 480,
		},
	},
	.set_crop = &ov7648_set_crop,
	.pix_format = {
		.width = 640,
		.height = 480,
		.pixelformat = V4L2_PIX_FMT_JPEG,
		.priv = 8,
	},
	.set_pix_format = &ov7648_set_pix_format
};


int sn9c102_probe_ov7648(struct sn9c102_device* cam)
{
	int pid, ver, err;

	err = sn9c102_write_const_regs(cam, {0x01, 0xf1}, {0x00, 0xf1},
	                               {0x29, 0x01}, {0x74, 0x02},
	                               {0x0e, 0x01}, {0x44, 0x01});

	pid = sn9c102_i2c_try_read(cam, &ov7648, 0x0a);
	ver = sn9c102_i2c_try_read(cam, &ov7648, 0x0b);
	if (err || pid < 0 || ver < 0)
		return -EIO;
	if (pid != 0x76 || ver != 0x48)
		return -ENODEV;

	sn9c102_attach_sensor(cam, &ov7648);

	return 0;
}
