/*
 * Tarf_ar0330_stereo_camera.c - Dual ar0330 sensor driver
 * Copyright (c) 2015-2016, e-con Systems.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/of_graph.h>


#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/module.h>

#include <linux/seq_file.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>

#include "camera/camera_gpio.h"

#include "Tarf_ar0330_stereo_camera.h"



static const struct v4l2_ctrl_ops ar0330_ctrl_ops = {
	.g_volatile_ctrl = ar0330_g_volatile_ctrl,
	.s_ctrl = ar0330_s_ctrl,
};

static int ar0330_bulk_write(struct camera_common_data *s_data, u16 addr,
			     u8 * val, size_t count)
{
	int err;
	int retry_cnt = 0;
	struct ar0330 *priv = (struct ar0330 *)s_data->priv;
	pr_err("%s : addr = 0x%x, size = %u\n", __func__, addr, count);
	retry:
	err = regmap_bulk_write(priv->b_regmap, addr, val, count);
	if (err) {
		retry_cnt++;
		pr_err("%s:i2c write failed, %x = %x count=%d retry=%d\n",
		       __func__, addr, *val, (int)count,retry_cnt);
		
		if(retry_cnt <= 20)		
			goto retry;
	}
	return err;
}

static int test_mode = 0;
static int ctrl_defaults_not_set = 1;
/* functions for reg read and write*/
static inline int ar0330_read_8b_reg(struct camera_common_data *s_data,
				     u16 addr, u8 * val)
{
	int err;
	int retry_cnt = 0;
	struct ar0330 *priv = (struct ar0330 *)s_data->priv;
	retry:
	err = regmap_read(priv->b_regmap, addr, (unsigned int *)val);
	if (err) {
		retry_cnt++;
		pr_err("%s:i2c read failed, %x = %x retry=%d\n",
		       __func__, addr, *val,retry_cnt);
			
		if(retry_cnt <= 10)		
			goto retry;
	}
	return err;

}

static int ar0330_read_reg(struct i2c_client *client, u16 reg, u16 *val)
{
	struct i2c_msg msg[2];
	u8 buf[2];
	int ret;

	buf[0] = (reg >> 8) & 0xff;
	buf[1] = reg & 0xFF;

	msg[0].addr = client->addr;
	msg[0].flags = client->flags;
	msg[0].buf = buf;
	msg[0].len = sizeof(buf);

	msg[1].addr = client->addr;
	msg[1].flags = client->flags | I2C_M_RD;
	msg[1].buf = buf;
	msg[1].len = 2;

	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret >= 0) {
		*val = (buf[0] << 8) | buf[1];
		return 0;
	}

	dev_err(&client->dev,
		"ar0330 read reg:0x%x failed !\n", reg);

	return ret;
}

static inline int ar0330_read_16b_reg(struct camera_common_data *s_data,
				      u16 addr, u16 * val)
{
	int err;
	int retry_cnt = 0;
	struct ar0330 *priv = (struct ar0330 *)s_data->priv;
pr_err("Reading 16b reg: 0x%x\n", *val);
	retry:
//	err = ar0330_read_reg(priv->i2c_client, addr, (unsigned int *)val);
	err = regmap_read(priv->w_regmap, addr, (unsigned int *)val);
pr_err("Reading 16b reg DONE: 0x%x\n", *val);
	if (err) {
		retry_cnt++;
		pr_err("%s:i2c read failed, %x = %x retry=%d\n",
		       __func__, addr, *val,retry_cnt);
		if((addr == 0x0000) && (retry_cnt == 5)) 
			return err;		
		
		if(retry_cnt <= 10)		
			goto retry;

	}
	return err;

}

static inline int ar0330_read_32b_reg(struct camera_common_data *s_data,
				      u16 addr, u32 * val)
{
	int err;
	int retry_cnt = 0;
	struct ar0330 *priv = (struct ar0330 *)s_data->priv;
	retry:
	err = regmap_read(priv->dw_regmap, addr, (unsigned int *)val);
	if (err) {
		retry_cnt++;
		pr_err("%s:i2c read failed, %x = %x retry=%d\n",
		       __func__, addr, *val,retry_cnt);
			
		if(retry_cnt <= 10)		
			goto retry;
	}
	return err;
}

static int ar0330_write_8b_reg(struct camera_common_data *s_data, u16 addr,
			       u8 val)
{
	int err;
	int retry_cnt = 0;

	struct ar0330 *priv = (struct ar0330 *)s_data->priv;
	retry:
	err = regmap_write(priv->b_regmap, addr, val);
	if (err) {
		retry_cnt++;
		pr_err("%s:i2c write failed, %x = %x retry=%d\n",
		       __func__, addr, val,retry_cnt);
			
		if(retry_cnt <= 10)		
			goto retry;
	}

	return err;
}

static int ar0330_write_16b_reg(struct camera_common_data *s_data, u16 addr,
				u16 val)
{
	int err;
	int retry_cnt = 0;
	struct ar0330 *priv = (struct ar0330 *)s_data->priv;
	retry:
	err = regmap_write(priv->w_regmap, addr, val);
	if (err) {
		retry_cnt++;
		pr_err("%s:i2c write failed, %x = %x retry=%d\n",
		       __func__, addr, val,retry_cnt);
			
		if(retry_cnt <= 10)		
			goto retry;
	}

	return err;
}

static int ar0330_write_32b_reg(struct camera_common_data *s_data, u16 addr,
				u32 val)
{
	int err;
	int retry_cnt = 0;
	struct ar0330 *priv = (struct ar0330 *)s_data->priv;
	retry:
	err = regmap_write(priv->dw_regmap, addr, val);
	if (err) {
		retry_cnt++;
		pr_err("%s:i2c write failed, %x = %x retry=%d\n",
		       __func__, addr, val,retry_cnt);
			
		if(retry_cnt <= 10)		
			goto retry;
	}

	return err;
}

static int ar0330_write_table(struct ar0330 *priv, const struct ar0330_reg table[])
{
	return regmap_util_write_table_16_as_8(priv->b_regmap,
					       table,
					       NULL, 0,
					       AR0330_TABLE_WAIT_MS,
					       AR0330_TABLE_END);
}

/* functions for reg read and write ends*/

/* function to toggle gpio*/

static void ar0330_gpio_set(struct ar0330 *priv,
			    unsigned int gpio, int val)
{
	int i = ctrlvals[0].step;
	if (priv->pdata && priv->pdata->use_cam_gpio)
		cam_gpio_ctrl(&priv->i2c_client->dev, gpio, val, 1);
	else {
		if (gpio_cansleep(gpio)){
			gpio_direction_output(gpio,val);
			gpio_set_value_cansleep(gpio, val);
		}
		else{
			gpio_direction_output(gpio,val);
			gpio_set_value(gpio, val);
		}
	}
}

static int ar0330_power_on(struct camera_common_data *s_data)
{
	int err = 0;
	struct ar0330 *priv = (struct ar0330 *)s_data->priv;
	struct camera_common_power_rail *pw = &priv->power;

	dev_dbg(&priv->i2c_client->dev, "%s: power on\n", __func__);

	if (priv->pdata && priv->pdata->power_on) {
		err = priv->pdata->power_on(pw);
		if (err)
			pr_err("%s failed.\n", __func__);
		else
			pw->state = SWITCH_ON;
		return err;
	}

	/* sleeps calls in the sequence below are for internal device
	 * signal propagation as specified by sensor vendor */

	if (pw->avdd)
		err = regulator_enable(pw->avdd);
	if (err)
		goto ar0330_avdd_fail;

	if (pw->iovdd)
		err = regulator_enable(pw->iovdd);
	if (err)
		goto ar0330_iovdd_fail;

	if (pw->reset_gpio)
		ar0330_gpio_set(priv, pw->reset_gpio, 0);

	/* datasheet 2.9: reset requires ~2ms settling time */
	usleep_range(2000, 2010);	

	usleep_range(1, 2);
	if (pw->pwdn_gpio)
		ar0330_gpio_set(priv, pw->pwdn_gpio, 0);

	/* datasheet 2.9: reset requires ~2ms settling time
	 * a power on reset is generated after core power becomes stable */
	usleep_range(2000, 2010);

	if (pw->reset_gpio)
		ar0330_gpio_set(priv, pw->reset_gpio, 1);

	/* datasheet fig 2-9: t3 */
	usleep_range(1350, 1360);

	pw->state = SWITCH_ON;
	return 0;

 ar0330_iovdd_fail:
	regulator_disable(pw->avdd);

 ar0330_avdd_fail:
	pr_err("%s failed.\n", __func__);
	return -ENODEV;
}

#if 1
static int ar0330_power_off(struct camera_common_data *s_data)
{
	int err = 0;
	struct ar0330 *priv = (struct ar0330 *)s_data->priv;
	struct camera_common_power_rail *pw = &priv->power;

	dev_dbg(&priv->i2c_client->dev, "%s: power off\n", __func__);

	if (priv->pdata && priv->pdata->power_on) {
		err = priv->pdata->power_off(pw);
		if (!err)
			pw->state = SWITCH_OFF;
		else
			pr_err("%s failed.\n", __func__);
		return err;
	}

	/* sleeps calls in the sequence below are for internal device
	 * signal propagation as specified by sensor vendor */

	usleep_range(21, 25);
	if (pw->pwdn_gpio)
		ar0330_gpio_set(priv, pw->pwdn_gpio, 0);
	usleep_range(1, 2);
	if (pw->reset_gpio)
		ar0330_gpio_set(priv, pw->reset_gpio, 0);

	/* datasheet 2.9: reset requires ~2ms settling time */
	usleep_range(2000, 2010);

	if (pw->iovdd)
		regulator_disable(pw->iovdd);
	if (pw->avdd)
		regulator_disable(pw->avdd);

	return 0;
}
#endif

static int ar0330_power_put(struct ar0330 *priv)
{
	struct camera_common_power_rail *pw = &priv->power;

	if (unlikely(!pw))
		return -EFAULT;

	if (likely(pw->avdd))
		regulator_put(pw->avdd);

	if (likely(pw->iovdd))
		regulator_put(pw->iovdd);

	pw->avdd = NULL;
	pw->iovdd = NULL;

	if (priv->pdata && priv->pdata->use_cam_gpio)
		cam_gpio_deregister(&priv->i2c_client->dev, pw->pwdn_gpio);
	else {
		gpio_free(pw->pwdn_gpio);
		gpio_free(pw->reset_gpio);
	}

	return 0;
}

static int ar0330_power_get(struct ar0330 *priv)
{
	struct camera_common_power_rail *pw = &priv->power;
	struct camera_common_pdata *pdata = priv->pdata;
	const char *mclk_name;
	const char *parentclk_name;
	struct clk *parent;
	int err = 0 ,ret = 0;

	mclk_name = priv->pdata->mclk_name ?
	    priv->pdata->mclk_name : "cam_mclk1";
	pw->mclk = devm_clk_get(&priv->i2c_client->dev, mclk_name);
	if (IS_ERR(pw->mclk)) {
		dev_err(&priv->i2c_client->dev,
			"unable to get clock %s\n", mclk_name);
		return PTR_ERR(pw->mclk);
	}

	parentclk_name = priv->pdata->parentclk_name;
	if (parentclk_name) {
		parent = devm_clk_get(&priv->i2c_client->dev, parentclk_name);
		if (IS_ERR(parent)) {
			dev_err(&priv->i2c_client->dev,
				"unable to get parent clock %s",
				parentclk_name);
		} else
			clk_set_parent(pw->mclk, parent);
	}

	/* analog 2.8v */
	err |= camera_common_regulator_get(&priv->i2c_client->dev,
					   &pw->avdd, pdata->regulators.avdd);
	/* IO 1.8v */
	err |= camera_common_regulator_get(&priv->i2c_client->dev,
					   &pw->iovdd, pdata->regulators.iovdd);

	if (!err) {
		pw->reset_gpio = pdata->reset_gpio;
		pw->pwdn_gpio = pdata->pwdn_gpio;
	}

	if (pdata->use_cam_gpio) {
		err = cam_gpio_register(&priv->i2c_client->dev, pw->pwdn_gpio);
		if (err)
			dev_err(&priv->i2c_client->dev,
				"%s ERR can't register cam gpio %u!\n",
				 __func__, pw->pwdn_gpio);
	} else {
		ret = gpio_request(pw->pwdn_gpio, "cam_pwdn_gpio");
		if (ret < 0)
			dev_dbg(&priv->i2c_client->dev,
				"%s can't request pwdn_gpio %d\n",
				__func__, ret);
		ret = gpio_request(pw->reset_gpio, "cam_reset_gpio");
		if (ret < 0)
			dev_dbg(&priv->i2c_client->dev,
				"%s can't request reset_gpio %d\n",
				__func__, ret);
	}


	pw->state = SWITCH_OFF;
	return err;
}

static int ar0330_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
    	struct camera_common_data *s_data = to_camera_common_data(&client->dev);
	struct ar0330 *priv = (struct ar0330 *)s_data->priv;
	struct camera_common_data *cam_data = priv->s_data;
	int err = 1;
	
	dev_dbg(&client->dev, "%s++\n", __func__);
	pr_err("ar0330_s_stream enable mode table = %u\n", enable);

	if (!enable) {
		return ar0330_write_table(priv, mode_table[AR0330_MODE_STOP_STREAM]);
	} else {
	pr_err("ar0330_s_stream starting test mode\n");
//		return ar0330_write_table(priv,
//			  mode_table[AR0330_MODE_STOP_STREAM]);

		return ar0330_write_table(priv, mode_table[AR0330_MODE_TEST_PATTERN]);
	}


	pr_err("Frame format mode = %u\n", priv->frmfmt_mode);
	if(priv->frmfmt_mode == AR0330_MODE_HD)	{
		pr_err("Frame rate index = %u\n", priv->frate_index);
		if(priv->frate_index == AR0330_60_FPS_INDEX) {
			err = ar0330_write_table(priv, mode_table[AR0330_720P_60FPS]);
	
			if(priv->exposure_mode == V4L2_EXPOSURE_AUTO) {
				ar0330_write_32b_reg(cam_data, 0x2024, AR0330_60_FPS_EXPOSURE_VAL);	/*   Upper Et val in microsec */
				ar0330_write_32b_reg(cam_data, 0x2028, AR0330_60_FPS_EXPOSURE_VAL);	/*   Max Et val in microsec */
			}
			pr_err("%s++  AR0330_720P_60FPS \n",__func__);
		}
		else if(priv->frate_index == AR0330_30_FPS_INDEX) {
			err = ar0330_write_table(priv, mode_table[AR0330_720P_30FPS]);
			if(priv->exposure_mode == V4L2_EXPOSURE_AUTO) {
				ar0330_write_32b_reg(cam_data, 0x2024, AR0330_30_FPS_EXPOSURE_VAL);	/*   Upper Et val in microsec */
				ar0330_write_32b_reg(cam_data, 0x2028, AR0330_30_FPS_EXPOSURE_VAL);	/*   Max Et val in microsec */
			}//printk("%s++  AR0330_720P_30FPS \n",__func__);
		}
		else
			printk("Set Resolution failed - invalid FrameRate Index = %d",priv->frate_index);
	}
	else if(priv->frmfmt_mode == AR0330_MODE_VGA) {
		if(priv->frate_index == AR0330_60_FPS_INDEX) {
			err = ar0330_write_table(priv, mode_table[AR0330_480P_60FPS]);
			if(priv->exposure_mode == V4L2_EXPOSURE_AUTO) {
				ar0330_write_32b_reg(cam_data, 0x2024, AR0330_60_FPS_EXPOSURE_VAL);	/*   Upper Et val in microsec */
				ar0330_write_32b_reg(cam_data, 0x2028, AR0330_60_FPS_EXPOSURE_VAL);	/*   Max Et val in microsec */
			}//printk("%s++ AR0330_480P_60FPS \n",__func__);
		}
		else if(priv->frate_index == AR0330_30_FPS_INDEX) {
			err = ar0330_write_table(priv, mode_table[AR0330_480P_30FPS]);
			if(priv->exposure_mode == V4L2_EXPOSURE_AUTO) {
				ar0330_write_32b_reg(cam_data, 0x2024, AR0330_30_FPS_EXPOSURE_VAL);	/*   Upper Et val in microsec */
				ar0330_write_32b_reg(cam_data, 0x2028, AR0330_30_FPS_EXPOSURE_VAL);	/*   Max Et val in microsec */
			}
		//	printk("%s++ AR0330_480P_30FPS \n",__func__);
		}
		else
			printk("Set Resolution failed - invalid FrameRate Index = %d",priv->frate_index);
	}
	else if(priv->frmfmt_mode == AR0330_MODE_NHD) {
		if(priv->frate_index == AR0330_60_FPS_INDEX) {
			err = ar0330_write_table(priv, mode_table[AR0330_360P_60FPS]);
			if(priv->exposure_mode == V4L2_EXPOSURE_AUTO) {
				ar0330_write_32b_reg(cam_data, 0x2024, AR0330_60_FPS_EXPOSURE_VAL);	/*   Upper Et val in microsec */
				ar0330_write_32b_reg(cam_data, 0x2028, AR0330_60_FPS_EXPOSURE_VAL);	/*   Max Et val in microsec */
			}//printk("%s++ AR0330_480P_60FPS \n",__func__);
		}
		else if(priv->frate_index == AR0330_30_FPS_INDEX) {
			err = ar0330_write_table(priv, mode_table[AR0330_360P_30FPS]);
			if(priv->exposure_mode == V4L2_EXPOSURE_AUTO) {
				ar0330_write_32b_reg(cam_data, 0x2024, AR0330_30_FPS_EXPOSURE_VAL);	/*   Upper Et val in microsec */
				ar0330_write_32b_reg(cam_data, 0x2028, AR0330_30_FPS_EXPOSURE_VAL);	/*   Max Et val in microsec */
			}
		//	printk("%s++ AR0330_480P_30FPS \n",__func__);
		}
		else
			printk("Set Resolution failed - invalid FrameRate Index = %d",priv->frate_index);
	}
	if (err)
		goto exit;

	if (test_mode) {
		pr_err("[[[ TEST MODE ON ]]]\n");
		err = ar0330_write_table(priv,
					 mode_table[AR0330_MODE_TEST_PATTERN]);
	}
	return 0;
	
 exit:
  //    printk("Exit with error = %d",err);
	dev_dbg(&client->dev, "%s: error setting stream\n", __func__);
	return err;
}

static int ar0330_g_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *param)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct camera_common_data *s_data = to_camera_common_data(&client->dev);
	struct ar0330 *priv = (struct ar0330 *)s_data->priv;

	pr_err("[[%s]]", __func__);

	if (!priv || !priv->pdata) {
		return -EINVAL;
	}
	param->parm.capture.capability |= V4L2_CAP_TIMEPERFRAME;

	pr_err("FR index = %d", priv->frate_index);
	pr_err("FR mode = %d", priv->frmfmt_mode);

	param->parm.capture.timeperframe.denominator =
	    ar0330_frmfmt[priv->frmfmt_mode].framerates[priv->frate_index];

	pr_err("priv->denom = %d\n",param->parm.capture.timeperframe.denominator);
	param->parm.capture.timeperframe.numerator = 1;
	return 0;
}



static int ar0330_s_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *param)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
     	struct camera_common_data *s_data = to_camera_common_data(&client->dev);
	struct ar0330 *priv = (struct ar0330 *)s_data->priv;
	int ret = 0;
	int err = 1;
    
	if (!priv || !priv->pdata) {
		return -EINVAL;
	}
	for (ret = 0; ret < ar0330_frmfmt[priv->frmfmt_mode].num_framerates;
	     ret++) {
		if ((ar0330_frmfmt[priv->frmfmt_mode].framerates[ret] ==
					param->parm.capture.timeperframe.denominator)) {

			/*Stop the Streaming - Not implemented*/			
			err = ar0330_write_table(priv,mode_table[AR0330_MODE_STOP_STREAM]);
			if (err) {
				dev_err(&client->dev, "%s: Failed stream_config \n", __func__);
				return err;
			}
			priv->frate_index = ret;
		//	printk("%s++ ,priv->frate_index = %d \n",__func__,priv->frate_index);
			return 0;
		}
	}
    	printk("S_parm with error -- param->parm.capture.timeperframe.denominator = %d \n",param->parm.capture.timeperframe.denominator);
	return -EINVAL;
}

static int ar0330_g_input_status(struct v4l2_subdev *sd, u32 * status)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct camera_common_data *s_data = to_camera_common_data(&client->dev);
	struct ar0330 *priv = (struct ar0330 *)s_data->priv;
	struct camera_common_power_rail *pw = &priv->power;

	*status = pw->state == SWITCH_ON;
	return 0;
}

void init_private_data(struct ar0330 *p)
{
	p->frate_index 		= 0;
	p->brightness		= ctrl(BRIGHTNESS, def);
	p->contrast		= ctrl(CONTRAST, def);
	p->saturation		= ctrl(SATURATION, def);
	p->sharpness		= ctrl(SHARPNESS, def);
	p->gain			= ctrl(GAIN, def);
	p->gamma		= ctrl(GAMMA, def);
	p->awb_mode		= ctrl(AUTO_WHITE_BALANCE, def);
	p->awb_value		= ctrl(WHITE_BALANCE_TEMPERATURE, def);
	p->vflip		= ctrl(VFLIP, def);
	p->hflip		= ctrl(HFLIP, def);
	p->exposure_mode	= ctrl(EXPOSURE_AUTO, def);
	p->exposure_value	= ctrl(EXPOSURE_ABSOLUTE, def);
	return ;
}

static struct v4l2_subdev_video_ops ar0330_subdev_video_ops = {
	.s_stream = ar0330_s_stream,
	.g_mbus_config = camera_common_g_mbus_config,
	.g_input_status = ar0330_g_input_status,
	.g_parm = ar0330_g_parm,
	.s_parm = ar0330_s_parm,
};

static struct v4l2_subdev_core_ops ar0330_subdev_core_ops = {
	.s_power = camera_common_s_power,
};

static int ar0330_get_fmt(struct v4l2_subdev *sd,
			struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *format)
{
	return camera_common_g_fmt(sd, &format->format);
}

static int ar0330_set_fmt(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *format)
{
	int ret;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct camera_common_data *s_data = to_camera_common_data(&client->dev);
	struct ar0330 *priv = (struct ar0330 *)s_data->priv;
	pr_err("[[[%s]]]", __func__);

	if (!priv || !priv->pdata)
		return -EINVAL;

	switch (format->format.code) {
		case MEDIA_BUS_FMT_UYVY8_1X16:
			priv->format_fourcc = V4L2_PIX_FMT_UYVY;
			break;

		default:
			pr_err("[[[%s]]] => FORMAT.CODE = %d, only %d is accepted", __func__, format->format.code, MEDIA_BUS_FMT_UYVY8_1X16);
			/* Not Implemented */
			//if (format->which != V4L2_SUBDEV_FORMAT_TRY) 		
				return -EINVAL;
		}

	for (ret = 0; ret < s_data->numfmts ; ret++) {
		pr_err("Width %d, Req: %d", ar0330_frmfmt[ret].size.width, format->format.width);
		pr_err("Height %d, Req: %d", ar0330_frmfmt[ret].size.height, format->format.height);
		if ((ar0330_frmfmt[ret].size.width == format->format.width)
				&& (ar0330_frmfmt[ret].size.height ==
					format->format.height)) {
			priv->frmfmt_mode = ar0330_frmfmt[ret].mode;
			printk("%s++ , frmfmt_mode %d \n",__func__,priv->frmfmt_mode);
			//flag = 1;
			break;
		}
	}	
	
	if (format->which == V4L2_SUBDEV_FORMAT_TRY) {
		ret = camera_common_try_fmt(sd, &format->format);
	} else {
		ret = camera_common_s_fmt(sd, &format->format);
	}
    	printk("%s => ret = %d\n",__func__, ret);
	return ret;
}
#if 0
static int camera_common_enum_frameintervals(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_frame_interval_enum *fie)
{
	
}
#endif

static struct v4l2_subdev_pad_ops ar0330_subdev_pad_ops = {
	.enum_mbus_code = camera_common_enum_mbus_code,
	.set_fmt = ar0330_set_fmt,
	.get_fmt = ar0330_get_fmt,
	.enum_frame_size = camera_common_enum_framesizes,
	.enum_frame_interval = camera_common_enum_frameintervals,
};

static struct v4l2_subdev_ops ar0330_subdev_ops = {
	.core = &ar0330_subdev_core_ops,
	.video = &ar0330_subdev_video_ops,
	.pad = &ar0330_subdev_pad_ops,
};

static struct of_device_id ar0330_of_match[] = {
	{.compatible = "nvidia,ar0330",},
	{},
};

static struct camera_common_sensor_ops ar0330_common_ops = {
//        .power_on = ar0330_power_on,
//        .power_off = ar0330_power_off,
	.write_reg = ar0330_write_8b_reg,
	.read_reg = ar0330_read_8b_reg,
};


static int ar0330_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ar0330 *priv =
	    container_of(ctrl->handler, struct ar0330, ctrl_handler);
	int err = 0;

	pr_err("ar0330_g_volatile_ctrl ar0330_g_volatile_ctrl\n");

	if (priv->power.state == SWITCH_OFF)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_BRIGHTNESS:
		ctrl->val = priv->brightness;
		break;
	case V4L2_CID_CONTRAST:
		ctrl->val = priv->contrast;
		break;
	case V4L2_CID_SATURATION:
		ctrl->val = priv->saturation;
		break;
	case V4L2_CID_SHARPNESS:
		ctrl->val = priv->sharpness;
		break;
	case V4L2_CID_GAIN:
		ctrl->val = priv->gain;
		break;
	case V4L2_CID_GAMMA:
		ctrl->val = priv->gamma;
		break;
	case V4L2_CID_AUTO_WHITE_BALANCE:
		ctrl->val = priv->awb_mode;
		break;
	case V4L2_CID_WHITE_BALANCE_TEMPERATURE:
		ctrl->val = priv->awb_value;
		break;
	case V4L2_CID_VFLIP:
		ctrl->val = priv->vflip;
		break;
	case V4L2_CID_HFLIP:
		ctrl->val = priv->hflip;
		break;
	case V4L2_CID_EXPOSURE_AUTO:
		ctrl->val = priv->exposure_mode;
		break;
	case V4L2_CID_EXPOSURE_ABSOLUTE:
		ctrl->val = priv->exposure_value;
		break;
	default:
		pr_err("%s: unknown ctrl id.\n", __func__);
		return -EINVAL;
	}

	return err;
}

static int ar0330_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ar0330 *priv =
	    container_of(ctrl->handler, struct ar0330, ctrl_handler);
	struct camera_common_data *cam_data = priv->s_data;
	u16 data = 0;
	int err = 0, mode = 0;

pr_err("ar0330_s_ctrl ar0330_s_ctrl\n");
	
	if (priv->power.state == SWITCH_OFF)
		return 0;
	mode = cam_data->mode;

	switch (ctrl->id) {
	case V4L2_CID_BRIGHTNESS:
		if ((ctrl->val >= ctrl(BRIGHTNESS, min)) &&
		    (ctrl->val <= ctrl(BRIGHTNESS, max))) {
			ar0330_write_16b_reg(cam_data, 0x7000,
					     (ctrl->val) << 8);
			priv->brightness = (ctrl->val);
		} else {
			return -EINVAL;
		}
		break;
	case V4L2_CID_CONTRAST:
		if ((ctrl->val >= ctrl(CONTRAST, min)) &&
		    (ctrl->val <= ctrl(CONTRAST, max))) {
			//data = RANGE_MAP(ctrl(CONTRAST, min), ctrl(CONTRAST, max), REGISTER_CONTRAST_MIN, REGISTER_CONTRAST_MAX, ctrl->val);
			//printk("Range converted contrast = 0x%x\n", data);
			//ar0330_write_16b_reg(cam_data, 0x52DA, data);
			ar0330_write_16b_reg(cam_data, 0x52DA, ctrl->val << 8);
			priv->contrast = (ctrl->val);
		} else {
			return -EINVAL;
		}
		break;

	case V4L2_CID_SATURATION:
		if ((ctrl->val >= ctrl(SATURATION, min)) &&
		    (ctrl->val <= ctrl(SATURATION, max))) {
			ar0330_write_16b_reg(cam_data, 0x7006, ctrl->val << 8);
			priv->saturation = ctrl->val;
		} else
			return -EINVAL;
		break;
	case V4L2_CID_SHARPNESS:
		if ((ctrl->val >= ctrl(SHARPNESS, min)) &&
		    (ctrl->val <= ctrl(SHARPNESS, max))) {
			data = ((ctrl->val & 0x000F) << 4) << 8; //FIXME:?
			//data = RANGE_MAP(ctrl(SHARPNESS, min), ctrl(SHARPNESS, max), REGISTER_SHARPNESS_MIN, REGISTER_SHARPNESS_MAX, ctrl->val);
			//printk("Range converted sharpness = 0x%x\n", data);
			ar0330_write_16b_reg(cam_data, 0x7010, data);
			priv->sharpness = ctrl->val;
		} else
			return -EINVAL;
		break;
	case V4L2_CID_GAIN:		
         	if((priv->exposure_mode != V4L2_EXPOSURE_AUTO) || (ctrl_defaults_not_set)) {
			if ((ctrl->val >= ctrl(GAIN, min)) &&
		    	(ctrl->val <= ctrl(GAIN, max))) {
				data = (ctrl->val & 0x00FF) << 8;
				//data >>= 1; /* In Tarf Bootdata gain max limit is 0x2d00 so max gain 100 i.e 0x6400 is mapped to 0x3200 */
				//data = RANGE_MAP(ctrl(GAIN, min), ctrl(GAIN, max), REGISTER_GAIN_MIN, REGISTER_GAIN_MAX, ctrl->val);
				//printk("Range converted gain = 0x%x\n", data);
				ar0330_write_16b_reg(cam_data, 0x5006, data);
				priv->gain = ctrl->val;
			} 	
			else
				return -EINVAL;
		}
		else
			return -EINVAL;
		break;
	case V4L2_CID_GAMMA:
		if ((ctrl->val >= ctrl(GAMMA, min)) &&
		    (ctrl->val <= ctrl(GAMMA, max))) {
			data = ((((ctrl->val / 100) & 0x0F) << 12) & 0xF000);
			data = (data | (((ctrl->val % 100) * 4096 / 100) & 0x0FFF));

			ar0330_write_16b_reg(cam_data, 0x700A, data);
			priv->gamma = ctrl->val;
		} else
			return -EINVAL;
		break;
	case V4L2_CID_AUTO_WHITE_BALANCE:
		/* Skip setting this during initialization by v4l2_ctrl_handler_setup.
		 * Otherwise causes WB controls to fail with no error. */

		if (ctrl_defaults_not_set) {
			priv->awb_mode = ctrl->val;
			break;
		}

		/*  Disable AWB, Enable Manual TEMP */
		if (ctrl->val == ctrl(AUTO_WHITE_BALANCE,min)) {
			ar0330_write_16b_reg(cam_data, 0x5100, 0x1157);
		//	break;
		} else if(ctrl->val == ctrl(AUTO_WHITE_BALANCE,max)) {
			/*  Enable AWB */
			ar0330_write_16b_reg(cam_data, 0x5100, 0x115F);
		} else {
			return -EINVAL;
		}
		priv->awb_mode = ctrl->val;
		break;
	case V4L2_CID_WHITE_BALANCE_TEMPERATURE:
		if((priv->awb_mode != ctrl(AUTO_WHITE_BALANCE,max)) || (ctrl_defaults_not_set)) {
			if ((ctrl->val >= ctrl(WHITE_BALANCE_TEMPERATURE, min)) &&
		    	(ctrl->val <= ctrl(WHITE_BALANCE_TEMPERATURE, max))) {
				ar0330_write_16b_reg(cam_data, 0x510A, ctrl->val);
				priv->awb_value = ctrl->val;
			}
			else
				return -EINVAL;
		} 
		else
			return -EINVAL;

		break;
	case V4L2_CID_VFLIP:
		ar0330_read_16b_reg(cam_data, 0x100C, &data);
		if (ctrl->val == ctrl(VFLIP,max)) {
			data &= 0x1;
			ar0330_write_16b_reg(cam_data, 0x100C, data);
		} else if(ctrl->val == ctrl(VFLIP, min)) {
			data |= 0x2;
			ar0330_write_16b_reg(cam_data, 0x100C, data);
		} else {
			return -EINVAL;
		}		
		priv->vflip = ctrl->val;
		break;
	case V4L2_CID_HFLIP:
		ar0330_read_16b_reg(cam_data, 0x100C, &data);
		if (ctrl->val == ctrl(HFLIP,max)) {
			data &= 0x2;
			ar0330_write_16b_reg(cam_data, 0x100C, data);
		} else if(ctrl->val == ctrl(HFLIP, min)) {
			data |= 0x1;
			ar0330_write_16b_reg(cam_data, 0x100C, data);
		} else {
			return -EINVAL;
		}
		priv->hflip = ctrl->val;
		break;
	case V4L2_CID_EXPOSURE_AUTO:
		if (ctrl->val == V4L2_EXPOSURE_AUTO ) {
			if(priv->frate_index == AR0330_60_FPS_INDEX) {
			//	printk("set Expos priv->frate_index = %d\n",priv->frate_index);
				ar0330_write_32b_reg(cam_data, 0x2024, AR0330_60_FPS_EXPOSURE_VAL);	/*   Upper Et val in microsec */
				ar0330_write_32b_reg(cam_data, 0x2028, AR0330_60_FPS_EXPOSURE_VAL);	/*   Max Et val in microsec */
			} else if(priv->frate_index == AR0330_30_FPS_INDEX) {
				ar0330_write_32b_reg(cam_data, 0x2024, AR0330_30_FPS_EXPOSURE_VAL);	/*   Upper Et val in microsec */
				ar0330_write_32b_reg(cam_data, 0x2028, AR0330_30_FPS_EXPOSURE_VAL);	/*   Max Et val in microsec */
			}
			ar0330_write_16b_reg(cam_data,0x5002,0x002c);//Auto exp
			ar0330_write_16b_reg(cam_data,0x52E4,0x0040);//local-tone-mapping default
			priv->exposure_mode = (ctrl->val);
		} else if (ctrl->val == V4L2_EXPOSURE_MANUAL) {
			ar0330_write_16b_reg(cam_data,0x5002,0x0020);//Manual exp
			ar0330_write_16b_reg(cam_data,0x52E4,0x0000);//local-tone-mapping clear
			priv->exposure_mode = (ctrl->val);
		} else
			return -EINVAL;
		break;
	case V4L2_CID_EXPOSURE_ABSOLUTE:
		if((priv->exposure_mode != V4L2_EXPOSURE_AUTO) || (ctrl_defaults_not_set )) {
	       		if ((ctrl->val >= ctrl(EXPOSURE_ABSOLUTE, min)) &&
				(ctrl->val <= ctrl(EXPOSURE_ABSOLUTE, max))) {
				ar0330_write_32b_reg(cam_data, 0x2024, ctrl->val * 100);	/*   Upper Et val in microsec */
				ar0330_write_32b_reg(cam_data, 0x2028, ctrl->val * 100);	/*   Max Et val in microsec */
				ar0330_write_32b_reg(cam_data, 0x500c, ctrl->val * 100);	/*   Manual exp val in microsec */
				priv->exposure_value = (ctrl->val);
			} 
			else 
				return -EINVAL;
		}
		else
			return -EINVAL;
		break;
	default:
		pr_err("%s: unknown ctrl id.\n", __func__);
		return -EINVAL;
	}

	return err;
}

static int ar0330_set_defaults(struct camera_common_data *cam_data)
{
	/* Dynamically configure ISP to 2 or 4 lanes from board file */
	/*   Set spoof mode and 4 lane */
	if (cam_data->numlanes == 2) {
		ar0330_write_16b_reg(cam_data, 0x2030, 0x12);
		printk("Configuring sensor to two lane MIPI\n");
	} else if (cam_data->numlanes == 4) {
		ar0330_write_16b_reg(cam_data, 0x2030, 0x14);
		printk("Configuring sensor to four lane MIPI\n");
	} else
		return -1;

	/* Default settings */
	ar0330_write_16b_reg(cam_data, 0x2012, 0x30);	/*  YUYV422 format */

	ar0330_write_16b_reg(cam_data, 0x5058, 0); /*  AF marker disable	 */
	ar0330_write_16b_reg(cam_data, 0x2010, 0x86);	/*  Detection algs disable (face mark disable) */
//	ar0330_write_16b_reg(cam_data, 0x100C, 0x03);	/*   HFLIP & VFLIP enabled on default to correct orientation issue */

	return 0;
}

#define ADD_CTRL(nctrl) priv->ctrls[V4L2_CTRL_##nctrl] = v4l2_ctrl_new_std(&priv->ctrl_handler, 			\
							&ar0330_ctrl_ops, V4L2_CID_##nctrl, 				\
							ctrl(nctrl, min), 						\
							ctrl(nctrl, max), 						\
							ctrl(nctrl, step), 						\
							ctrl(nctrl, def)); 						\
//							pr_err(STR(V4L2_CID_##nctrl));					\
							if (priv->ctrl_handler.error) { 				\
								printk("%s %s %d %s\n", __FILE__, __func__, __LINE__, #nctrl); 	\
							}

#define ADD_MENU_CTRL(nctrl) priv->ctrls[V4L2_CTRL_##nctrl] = v4l2_ctrl_new_std_menu(&priv->ctrl_handler, 	\
							&ar0330_ctrl_ops, V4L2_CID_##nctrl, 				\
							ctrl(nctrl, max),							\
							0, ctrl(nctrl, def));						\
							if (priv->ctrl_handler.error) { 					\
								printk("%s %s %d %s\n", __FILE__, __func__, __LINE__, #nctrl); 	\
							}			

#define ADD_CUSTM_CTRL(nctrl) 	ctrl_config_list[V4L2_CTRL_CUSTM_CTRL_##nctrl].min = ctrl(nctrl, min);		\
				ctrl_config_list[V4L2_CTRL_CUSTM_CTRL_##nctrl].max = ctrl(nctrl, max);		\
				ctrl_config_list[V4L2_CTRL_CUSTM_CTRL_##nctrl].step = ctrl(nctrl, step);	\
				ctrl_config_list[V4L2_CTRL_CUSTM_CTRL_##nctrl].def = ctrl(nctrl, def);		\
				priv->ctrls[V4L2_CTRL_##nctrl] = v4l2_ctrl_new_custom(&priv->ctrl_handler,	\
						&ctrl_config_list[V4L2_CTRL_CUSTM_CTRL_##nctrl], NULL);		\
				if (priv->ctrls[V4L2_CTRL_##nctrl] == NULL) {					\
					dev_err(&client->dev, "Failed to init %s ctrl %d \n",			\
					ctrl_config_list[V4L2_CTRL_CUSTM_CTRL_##nctrl].name, 			\
					priv->ctrl_handler.error);						\
				}									
				
static int ar0330_ctrls_init(struct ar0330 *priv)
{
	struct i2c_client *client = priv->i2c_client;
	int numctrls;
	int err = 0;
	dev_dbg(&client->dev, "%s++\n", __func__);
	numctrls = AR0330_NUM_CONTROLS;

	v4l2_ctrl_handler_init(&priv->ctrl_handler, numctrls);
	priv->subdev->ctrl_handler = &priv->ctrl_handler;

/*  User class */
	ADD_CTRL(BRIGHTNESS);
	ADD_CTRL(CONTRAST);
	ADD_CTRL(SATURATION);
	ADD_CTRL(AUTO_WHITE_BALANCE);
	ADD_CTRL(GAMMA);
	ADD_CTRL(GAIN);
	ADD_CTRL(HFLIP);
	ADD_CTRL(VFLIP);
	ADD_CTRL(WHITE_BALANCE_TEMPERATURE);
	ADD_CTRL(SHARPNESS);
/*  Camera class */
	ADD_MENU_CTRL(EXPOSURE_AUTO);
	ADD_CTRL(EXPOSURE_ABSOLUTE);
	
	priv->numctrls = numctrls;
	priv->subdev->ctrl_handler = &priv->ctrl_handler;
	if (priv->ctrl_handler.error) {
		dev_err(&client->dev, "Error %d adding controls\n",
			priv->ctrl_handler.error);
		err = priv->ctrl_handler.error;
		goto error;
	}

	err = v4l2_ctrl_handler_setup(&priv->ctrl_handler);
	if (err) {
		dev_err(&client->dev,
			"Error %d setting default controls\n", err);
		goto error;
	}
	ctrl_defaults_not_set = 0;
	return 0;

 error:
	v4l2_ctrl_handler_free(&priv->ctrl_handler);
	return err;
}

MODULE_DEVICE_TABLE(of, ar0330_of_match);

static struct camera_common_pdata *ar0330_parse_dt(struct i2c_client *client)
{
	struct device_node *node = client->dev.of_node;
	struct camera_common_pdata *board_priv_pdata;
	const struct of_device_id *match;
	int gpio;
	int err;

	if (!node)
		return NULL;

	match = of_match_device(ar0330_of_match, &client->dev);
	if (!match) {
		dev_err(&client->dev, "Failed to find matching dt id\n");
		return NULL;
	}

	board_priv_pdata = devm_kzalloc(&client->dev,
					sizeof(*board_priv_pdata), GFP_KERNEL);
	if (!board_priv_pdata)
		return NULL;

	err = camera_common_parse_clocks(&client->dev, board_priv_pdata);
	if (err) {
		dev_err(&client->dev, "Failed to find clocks\n");
		goto error;
	}

	gpio = of_get_named_gpio(node, "pwdn-gpios", 0);
	if (gpio < 0) {
		dev_err(&client->dev, "pwdn gpios not in DT\n");
		goto error;
	}
	board_priv_pdata->pwdn_gpio = (unsigned int)gpio;

	gpio = of_get_named_gpio(node, "reset-gpios", 0);
	if (gpio < 0) {
		/* reset-gpio is not absoluctly needed */
		dev_dbg(&client->dev, "reset gpios not in DT\n");
		gpio = 0;
	}
	board_priv_pdata->reset_gpio = (unsigned int)gpio;

	board_priv_pdata->use_cam_gpio =
	    of_property_read_bool(node, "cam,use-cam-gpio");

	err = of_property_read_string(node, "avdd-reg",
				      &board_priv_pdata->regulators.avdd);
	if (err) {
		dev_err(&client->dev, "avdd-reg not in DT\n");
		goto error;
	}
	err = of_property_read_string(node, "iovdd-reg",
				      &board_priv_pdata->regulators.iovdd);
	if (err) {
		dev_err(&client->dev, "iovdd-reg not in DT\n");
		goto error;
	}

	board_priv_pdata->has_eeprom =
	    of_property_read_bool(node, "has-eeprom");

	return board_priv_pdata;

 error:
	devm_kfree(&client->dev, board_priv_pdata);
	return NULL;
}

static int get_error_status_3mp(struct camera_common_data *common_data)
{
	u16 status = 0;
	ar0330_read_16b_reg(common_data, 0x0006, &status);
	return status;
}

static int ar0330_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	dev_dbg(&client->dev, "%s:\n", __func__);
	return 0;
}

static const struct v4l2_subdev_internal_ops ar0330_subdev_internal_ops = {
	.open = ar0330_open,
};

static const struct media_entity_operations ar0330_media_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static int ar0330_load_boot_data(struct camera_common_data *common_data)
{
	u32 i, sizeof_boot_data = ARRAY_SIZE(AR0330_BOOT_DATA); //0x3f4
	int flag = 0, bcount = 0, ret = 0;
	u16 boot_base_addr = AR0330_BOOT_BASE_ADDRESS;	/* Boot reg address space : 0x8000 to 0x9FFF */
	u8 *boot_data, *pll_init_data;
	u16 crc = 0;
	u16 boot_stage = 0;
	u16 warning0 = 0;

	//memset(AR0330_BOOT_DATA, 0, sizeof AR0330_BOOT_DATA);
	//memcpy(AR0330_BOOT_DATA, AR0330_BOOT_DATA_INT, sizeof AR0330_BOOT_DATA / 2);

	boot_data = (u8 *) kmalloc(BURST_SIZE, GFP_KERNEL);
	if (!boot_data) {
		pr_err("%s: kmalloc error..\n", __func__);
		return -1;
	}

	pll_init_data = (u8 *) kmalloc(PLL_INIT_SIZE, GFP_KERNEL);
	if (!pll_init_data) {
		pr_err("%s: kmalloc error..\n", __func__);
		kfree(boot_data);
		return -1;
	}
	ar0330_write_16b_reg(common_data, 0xF052, 0xffff);	/* Writing sips_crc register */

	printk(" Size of Boot Data = %u Version = %u \n", sizeof_boot_data, BOOT_DATA_VERSION);

	for (i = 0; i < sizeof_boot_data;) {
	//	udelay(10000);
		/* LOADING PLL_INIT_DATA  INTIALLY (once) */
		if (!flag) {
			memset(pll_init_data, 0, PLL_INIT_SIZE);
			memcpy(pll_init_data, AR0330_BOOT_DATA, PLL_INIT_SIZE);
			ret =
			    ar0330_bulk_write(common_data, boot_base_addr,
					      pll_init_data, PLL_INIT_SIZE);
			if (ret < 0) {
				pr_err("%s: Failed to load pll_init_data\n",
				       __func__);
				ret = -1;
				goto end;
			}

			/* Setting bootdata_stage basic register */
			ar0330_write_16b_reg(common_data, 0x6002, 0x0002);

			msleep(1);

			flag = 1;
			boot_base_addr += PLL_INIT_SIZE;
			i += PLL_INIT_SIZE;
		}

		/* CONDITION : LOAD PARTIAL BUFFER IF 0X9FFF IS REACHED */
		else if ((boot_base_addr + BURST_SIZE) > 0x9FFF) {
			bcount = (0x9FFF - boot_base_addr) + 1;

			memset(boot_data, 0, BURST_SIZE);
			memcpy(boot_data, (AR0330_BOOT_DATA + i), bcount);
			ret =
			    ar0330_bulk_write(common_data, boot_base_addr,
					      boot_data, bcount);
			if (ret < 0) {
				pr_err("%s:%d: Failed to load boot_data\n",
				       __func__, __LINE__);
				ret = -1;
				goto end;
			}
			boot_base_addr = 0x8000;
			i += bcount;
		}

		/* CONDITION : REACHED END OF AR0330_BOOT_DATA array, Loading final bootdata buffer */
		else if ((i + BURST_SIZE) >= sizeof_boot_data) {
			bcount = sizeof_boot_data - i;

			memset(boot_data, 0, BURST_SIZE);
			memcpy(boot_data, (AR0330_BOOT_DATA + i), bcount);

			ret =
			    ar0330_bulk_write(common_data, boot_base_addr,
					      boot_data, bcount);
			if (ret < 0) {
				pr_err("%s:%d: Failed to load boot_data\n",
				       __func__, __LINE__);
				ret = -1;
				goto end;
			}
			break;
		}

		/* CONDITION: LOADING BOOT DATA NORMALLY WITH BURST_SIZE */
		else {
			memset(boot_data, 0, BURST_SIZE);
			memcpy(boot_data, (AR0330_BOOT_DATA + i), BURST_SIZE);
			ret =
			    ar0330_bulk_write(common_data, boot_base_addr,
					      boot_data, BURST_SIZE);
			if (ret < 0) {
				pr_err("%s:%d: Failed to load boot_data\n",
				       __func__, __LINE__);
				ret = -1;
				goto end;
			}
			boot_base_addr += BURST_SIZE;
			i += BURST_SIZE;
		}
	}
	pr_debug("%s():%d BOOT DATA DONE\n", __func__, __LINE__);

	/*  check crc */
	ar0330_read_16b_reg(common_data, 0xf052, &crc);
	if (crc != BOOT_DATA_CRC) {
		pr_err("Init setting dump failure. CRC = 0x%X \n", crc);
//		ret = -1;
		goto end;
	} else {
		printk("Init setting dump success. CRC = 0x%X Verified\n", crc);
	}


	/*  Set bootdata_stage to indicate full bootdata is loaded */
	ar0330_write_16b_reg(common_data, 0x6002, 0xffff);
	
   	msleep(1);

	/*  Get bootdata_stage to check whether isp boot is successful */
	ar0330_read_16b_reg(common_data, 0x6002, &boot_stage);
	pr_err("Boot stage = %u\n", boot_stage);

	if (boot_stage != 0xffff) {
		pr_err("ISP Boot Failure. boot_stage = 0x%X \n", boot_stage);

		/*  Get warning0 to check whether any sensor is disconnected */
		ar0330_read_16b_reg(common_data, 0x6004, &warning0);
		
		if (warning0 & PRIMARY_SENSOR_DISCONNECTED_MASK) 
			pr_err("Primary camera disconnected. warning0 = 0x%X \n", warning0);
		else if (warning0 & SECONDARY_SENSOR_DISCONNECTED_MASK) 
			pr_err("Secondary camera disconnected. warning0 = 0x%X \n", warning0);
		else
			pr_err("warning0 = 0x%X \n", warning0);
	}

	if (get_error_status_3mp(common_data)) {
		pr_err("ISP error \n");
		ret = -1;
		goto end;
	}
 end:
	kfree(boot_data);
	kfree(pll_init_data);
	return ret;
}


int camera_common_parse_ports(struct device *dev,
			      struct camera_common_data *s_data)
{
	struct device_node *node = dev->of_node;
	struct device_node *ep = NULL;
	struct device_node *next;
	int bus_width = 0;
	int err = 0;
	int port = 0;

	/* Parse all the remote entities and put them into the list */
	next = of_graph_get_next_endpoint(node, ep);
	if (!next) {
		printk("[AR0330]: no data?.\n");
		return -ENODATA;
	} else {
		printk("[AR0330]: data ok.\n");
	}
	of_node_put(ep);
	ep = next;

	err = of_property_read_u32(ep, "bus-width", &bus_width);
	if (err) {
		printk(dev,
			"Failed to find num of lanes\n");
		return err;
	} else {
		printk("[AR0330]: read u32 ok.\n");
	}
	s_data->numlanes = bus_width;

	err = of_property_read_u32(ep, "csi-port", &port);
	if (err) {
		printk(dev,
			"Failed to find CSI port\n");
		return err;
	} else {
		printk("[AR0330]: read 2 u32 ok.\n");
	}
	s_data->csi_port = port;

	dev_dbg(dev, "%s: csi port %d num of lanes %d\n",
		__func__, s_data->csi_port, s_data->numlanes);
	return 0;

}

/* sensor register write */
static int ar0330_write(struct i2c_client *client, u16 reg, u16 val)
{
	struct i2c_msg msg;
	u8 buf[4];
	int ret;

	dev_dbg(&client->dev, "write reg(0x%x val:0x%x)!\n", reg, val);

	buf[0] = reg >> 8;
	buf[1] = reg & 0xFF;
	buf[2] = val >> 8 & 0xff;
	buf[3] = val & 0xff;

	msg.addr = client->addr;
	msg.flags = client->flags;
	msg.buf = buf;
	msg.len = sizeof(buf);

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret >= 0)
		return 0;

	dev_err(&client->dev,
		"ov5640 write reg(0x%x val:0x%x) failed !\n", reg, val);

	return ret;
}

static int ar0330_write_array(struct i2c_client *client,
			      const struct ar0330_reg *regs)
{
	int i, delay_ms, ret = 0;

	i = 0;
	while (regs[i].addr != REG_NULL) {
		if (regs[i].addr == REG_DELAY) {
			delay_ms = regs[i].val;
			dev_info(&client->dev, "delay(%d) ms !\n", delay_ms);
			usleep_range(1000 * delay_ms, 1000 * delay_ms + 100);
			i++;
			continue;
		}
		ret = ar0330_write(client, regs[i].addr, regs[i].val);
		if (ret) {
			dev_err(&client->dev, "%s failed !\n", __func__);
			break;
		}
		i++;
	}

	return ret;
}


static int ar0330_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct camera_common_data *common_data;
	struct device_node *node = client->dev.of_node;
	struct ar0330 *priv;
	//FIXME: unused char debugfs_name[10];
	u16 boot_stage;
	int err;

	printk("[AR0330]: probing v4l2 sensor.\n");

	if (!IS_ENABLED(CONFIG_OF) || !node)
		return -EINVAL;
#if 0 
	void *adr1;
	void *adr2;
	static int once;
/* Increase Drive strength to solve the GEN_I2C not working issue, 
 * mentioned as "e-CAM30_HEXCUTX2_KI_01" in Confluence
 * 
 * Confluence link:
 * http://192.168.6.10:8090/display/CT/e-CAM30_HEXCUTX2+List+of+Known+Issues
 */
	if (once == 0) {


		adr1 = ioremap(PADCTL_AO_CFG2TMC_GEN8_I2C_SCL_0, 4);
		if (adr1 == NULL)
			printk("error");
		adr2 = ioremap(PADCTL_AO_CFG2TMC_GEN8_I2C_SDA_0, 4);
		if (adr2 == NULL)
			printk("error");

		once++;
		__raw_writel(0x0, adr1);
		__raw_writel(0x0, adr2);
		iounmap(adr1);
		iounmap(adr2);
	}
#endif
	common_data = devm_kzalloc(&client->dev,
				   sizeof(struct camera_common_data),
				   GFP_KERNEL);

	if (!common_data)
		return -ENOMEM;

	priv = devm_kzalloc(&client->dev,
			    sizeof(struct ar0330) + sizeof(struct v4l2_ctrl) *
			    (AR0330_NUM_CONTROLS), GFP_KERNEL);

	if (!priv)
		return -ENOMEM;

	priv->b_regmap = devm_regmap_init_i2c(client, &ar0330_b_regmap_config);
	if (IS_ERR(priv->b_regmap)) {
		dev_err(&client->dev,
			"8bit regmap init failed: %ld\n",
			PTR_ERR(priv->b_regmap));
		return -ENODEV;
	} else {
		printk("[AR0330]: remap init ok.\n");
	}

	priv->w_regmap = devm_regmap_init_i2c(client, &ar0330_w_regmap_config);
	if (IS_ERR(priv->w_regmap)) {
		dev_err(&client->dev,
			"16bit regmap init failed: %ld\n",
			PTR_ERR(priv->w_regmap));
		return -ENODEV;
	}

	unsigned int val = 0;
	pr_err("Reading w reg map: %u\n", val);
	err = regmap_read(priv->w_regmap, 0, &val);
	pr_err("Reading addr 0x0000, err = %d, val = %x\n", err, val);
	err = regmap_read(priv->w_regmap, 0x3000, &val);
	pr_err("Reading addr 0x3000, err = %d, val = %x\n", err, val);
	pr_err("Reading w reg map: %u\n", val);

	priv->dw_regmap =
	    devm_regmap_init_i2c(client, &ar0330_dw_regmap_config);
	if (IS_ERR(priv->dw_regmap)) {
		dev_err(&client->dev,
			"32bit regmap init failed: %ld\n",
			PTR_ERR(priv->dw_regmap));
		return -ENODEV;
	}

	priv->pdata = ar0330_parse_dt(client);
	if (!priv->pdata) {
		dev_err(&client->dev, "unable to get platform data\n");
		return -EFAULT;
	}

	pr_err("Common data address: 0x%lx\n", (unsigned long)common_data);

	common_data->ops = &ar0330_common_ops;
	common_data->ctrl_handler = &priv->ctrl_handler;
	common_data->dev = &client->dev;

	common_data->frmfmt = ar0330_frmfmt;
	common_data->colorfmt =
		camera_common_find_datafmt(AR0330_DEFAULT_DATAFMT);
	pr_err("Data fmt = %u\n", camera_common_find_datafmt(AR0330_DEFAULT_DATAFMT));
	common_data->power = &priv->power;
	common_data->ctrls = priv->ctrls;
	common_data->priv = (void *)priv;
	common_data->numctrls = AR0330_NUM_CONTROLS;
	common_data->numfmts = ARRAY_SIZE(ar0330_frmfmt);
	common_data->def_mode = AR0330_DEFAULT_MODE;
	common_data->def_width = AR0330_DEFAULT_WIDTH;
	common_data->def_height = AR0330_DEFAULT_HEIGHT;
	common_data->fmt_width = common_data->def_width;
	common_data->fmt_height = common_data->def_height;
	common_data->def_clk_freq = 24000000;


        pr_err("[[[nm = 0x%lu ]]]\n", (unsigned long)common_data->sensor_props.num_modes);
        pr_err("[[[sm = 0x%lx ]]]\n", (unsigned long)common_data->sensor_props.sensor_modes);

	priv->i2c_client = client;
	priv->s_data = common_data;
	pr_err("Common data subdev address: 0x%lx\n", (unsigned long)&common_data->subdev);
	priv->subdev = &common_data->subdev;
	priv->subdev->dev = &client->dev;
	priv->s_data->dev = &client->dev;

	err = ar0330_power_get(priv);
	if (err)
		return err;
	else
		printk("[AR0330]: power ok.\n");

	err = camera_common_initialize(common_data, "ar0330");
        pr_err("[[[nm = 0x%lu ]]]\n", (unsigned long)common_data->sensor_props.num_modes);
        pr_err("[[[sm = 0x%lx ]]]\n", (unsigned long)common_data->sensor_props.sensor_modes);
        pr_err("[[[spa = 0x%lx ]]]\n", (unsigned long)&common_data->sensor_props);


        if (err) {
                dev_err(&client->dev, "Failed to initialize ar0330.\n");
                return err;
        } else {
		printk("[AR0330]: Init success.\n");
	}

	err = ar0330_power_on(common_data);
	if (err)
		return err;
	else
		dev_err(&client->dev, "[AR0330]: power on success.\n");


	/* Probe for sensor */
	pr_err("Trying simple read reg\n");
	unsigned int chipid;
	ar0330_read_reg(client, 0x0000, &chipid);
	
	err = ar0330_read_16b_reg(common_data, 0x0000, &priv->chip_id);
	pr_err("ChipID = 0x%x\n", chipid);
	pr_err("ChipID = 0x%x\n", priv->chip_id);
	if (err) {
		printk("ERROR no =%d \n",err);
		dev_err(&client->dev, "AR0330 H/W module not present\n");
		return -ENODEV;
	}
	else
	{
		printk("[AR0330]: Reg 16b read ok.\n");
	}

	printk(" CHIP ID detected 0x%04x !! \n", priv->chip_id);

	if (priv->chip_id != 0x0265) {
		dev_err(&client->dev, "CHIP ID %xd not supported!\n",
			priv->chip_id);
		return -EINVAL;
	}


	/* Initialise the sensor after making sure the hardware is not already initialised */
	ar0330_read_16b_reg(common_data, 0x6002, &boot_stage);
pr_err("Boot stage = %u\n", boot_stage);
	if (boot_stage != 0xFFFF) {
		pr_err("Loading boot data\n");
		if (ar0330_load_boot_data(common_data) == -1) {
			printk("Exiting probe\n");
			return -EIO;
		}
	} else
		printk("Skipping boot data\n");
#if 0
	int bootdata_retry = 0;
        bootdata_start:
        /* Initialise the sensor after making sure the hardware is not already initialised */
        ar0330_read_16b_reg(common_data, 0x6002, &boot_stage);
       // if (boot_stage != 0xFFFF) {
                if (ar0330_load_boot_data(common_data) == -1) {
                        bootdata_retry++;
                        goto bootdata_start;
                        printk("Exiting probe\n");
                        return -EIO;
                }
        //} else
          //      printk("Skipping boot data\n"); 

        printk("bootdata_retry = %d\n", bootdata_retry);
#endif
	if (ar0330_set_defaults(common_data) == -1) {
		printk("Exiting Probe...Wrong lane configuration\n");
		return -EINVAL;
	}

	/* End of load boot data */
pr_err("INITING SUBDEVS\n");
	v4l2_i2c_subdev_init(priv->subdev, client, &ar0330_subdev_ops);
pr_err("SUBDEVS DONE\n");

pr_err("INITING PRIVATE DATA\n");
	init_private_data(priv);
pr_err("INITING PRIVATE DATA DONE\n");
pr_err("INITING ar0330_ctrls_init\n");
	err = ar0330_ctrls_init(priv);
pr_err("INITING ar0330_ctrls_init DONE\n");
	if (err)
		return err;

	priv->subdev->internal_ops = &ar0330_subdev_internal_ops;
	priv->subdev->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
	    V4L2_SUBDEV_FL_HAS_EVENTS;

#if defined(CONFIG_MEDIA_CONTROLLER)
pr_err("INITING MEDIA ENTITY\n");
	priv->pad.flags = MEDIA_PAD_FL_SOURCE;
	priv->subdev->entity.type = MEDIA_ENT_T_V4L2_SUBDEV_SENSOR;
	priv->subdev->entity.ops = &ar0330_media_ops;
	err = media_entity_init(&priv->subdev->entity, 1, &priv->pad, 0);
	if (err < 0) {
		dev_err(&client->dev, "unable to init media entity\n");
		return err;
	}
#endif

	struct camera_common_data *ss_data = to_camera_common_data(&client->dev);
	
	//dev_set_drvdata(&client->dev, priv);
	void *dd = dev_get_drvdata(&client->dev);

        pr_err("[[[drvdata = 0x%lx ]]]\n", (unsigned long)dd);
        pr_err("[[[spa = 0x%lx ]]]\n", (unsigned long)&ss_data->sensor_props);
	pr_err("[[[common data addr = 0x%lx ]]]\n", (unsigned long)ss_data);
        pr_err("[[[nm = 0x%lu ]]]\n", (unsigned long)ss_data->sensor_props.num_modes);
        pr_err("[[[sm = 0x%lx ]]]\n", (unsigned long)ss_data->sensor_props.sensor_modes);


pr_err("XX INITING v4l2_async_register_subdev\n");
	ss_data = to_camera_common_data(&client->dev);
pr_err("XX INITING v4l2_async_register_subdev\n");
	ss_data = to_camera_common_data(&client->dev);
	err = v4l2_async_register_subdev(priv->subdev);
pr_err("v4l2_async_register_subdev DONE\n");
	if (err)
		return err;

	printk("Detected AR0330 sensor\n");

	return 0;
}

static int ar0330_remove(struct i2c_client *client)
{
	struct camera_common_data *s_data = to_camera_common_data(&client->dev);
	struct ar0330 *priv = (struct ar0330 *)s_data->priv;

	v4l2_async_unregister_subdev(priv->subdev);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&priv->subdev->entity);
#endif

	v4l2_ctrl_handler_free(&priv->ctrl_handler);
	ar0330_power_put(priv);
	camera_common_remove_debugfs(s_data);

	return 0;
}

static const struct i2c_device_id ar0330_id[] = {
	{"ar0330", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, ar0330_id);

static struct i2c_driver ar0330_i2c_driver = {
	.driver = {
		   .name = "ar0330",
		   .owner = THIS_MODULE,
		   .of_match_table = of_match_ptr(ar0330_of_match),
		   },
	.probe = ar0330_probe,
	.remove = ar0330_remove,
	.id_table = ar0330_id,
};

module_i2c_driver(ar0330_i2c_driver);
MODULE_DESCRIPTION("Media CTL - V4L2 driver for OnSemi AR0330-Stereo");
MODULE_AUTHOR("e-consystems");
MODULE_LICENSE("GPL v2");
MODULE_VERSION(AR0330_STEREO_DRIVER_VERSION);

