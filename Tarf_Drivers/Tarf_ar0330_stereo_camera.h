#ifndef __Tarf_ar0330_stereo_camera_h__
#define __Tarf_ar0330_stereo_camera_h__

#include "modes.h"
#include "boot_data.h"

#include <linux/regmap.h>

#define RANGE_MAP(a, b, c, d, e) a


#define STR(x) #x


static int ar0330_g_volatile_ctrl(struct v4l2_ctrl *ctrl);

static int ar0330_s_ctrl(struct v4l2_ctrl *ctrl);


#define AR0330_REG_CHIP_ID		0x3000


#define AR0330_STEREO_DRIVER_VERSION "1.0.0"
#define AR0330_DEFAULT_HEIGHT 480				// Ok
#define AR0330_DEFAULT_WIDTH 1280				// Ok
#define AR0330_DEFAULT_DATAFMT MEDIA_BUS_FMT_UYVY8_1X16 //MEDIA_BUS_FMT_UYVY8_2X8 //,MEDIA_BUS_FMT_UYVY8_1X16 //V4L2_PIX_FMT_UYVY//9 //13 //MEDIA_BUS_FMT_UYVY8_1X16		// Probably Ok
#define AR0330_NUM_CONTROLS 12					// Probably Ok
#define AR0330_BOOT_BASE_ADDRESS 0x8000				// Ok
#define AR0330_DEFAULT_MODE 0					// Ok
#define AR0330_MODE_HD 0
#define AR0330_60_FPS_INDEX 0

#define AR0330_720P_60FPS 0 //This

#define AR0330_60_FPS_EXPOSURE_VAL 0
#define AR0330_30_FPS_INDEX 0
#define AR0330_720P_30FPS 0
#define AR0330_30_FPS_EXPOSURE_VAL 0
#define AR0330_MODE_VGA 0
#define AR0330_480P_60FPS 0
#define AR0330_480P_30FPS 0
#define AR0330_MODE_NHD 0
#define AR0330_360P_60FPS 0
#define AR0330_360P_30FPS 0


#define PRIMARY_SENSOR_DISCONNECTED_MASK 0
#define SECONDARY_SENSOR_DISCONNECTED_MASK 0

#define HTS_DEF 2496
#define VTS_DEF 1308
#define MAX_FPS 30


static const int imx268_30fps[] = {
        30, 60
};

static const struct camera_common_frmfmt ar0330_frmfmt[] = {
        {{1280, 360}, imx268_30fps, 2, 0, AR0330_MODE_TEST_PATTERN},
};

extern const struct regmap_config ar0330_dw_regmap_config = {
        .reg_bits = 16, //16
        .val_bits = 32
};
extern const struct regmap_config ar0330_b_regmap_config = {
        .reg_bits = 16, //16
        .val_bits = 8
};
extern const struct regmap_config ar0330_w_regmap_config = {
        .reg_bits = 16,
        .val_bits = 16
};

enum {
BRIGHTNESS,
CONTRAST,
SATURATION,
SHARPNESS,
GAIN,
GAMMA,
AUTO_WHITE_BALANCE,
WHITE_BALANCE_TEMPERATURE,
VFLIP,
HFLIP,
EXPOSURE_AUTO,
EXPOSURE_ABSOLUTE,
};

struct ctrl_vals {
	int min;
	int max;
	int step;
	int def;
};


static struct ctrl_vals ctrlvals[] = {
        [BRIGHTNESS] = {-15, 15, 1, 0},
        [CONTRAST] = {0, 60, 1, 10},
        [SATURATION] = {0, 90, 1, 20},
        [SHARPNESS] = {0, 7, 1, 0},
        [GAIN] = {1, 100, 1, 3},
        [GAMMA] = {40, 500, 1, 220},
        [AUTO_WHITE_BALANCE] = {0, 1, 1, 1},
        [WHITE_BALANCE_TEMPERATURE] = {10, 10000, 10, 4600},
        [VFLIP] = {0, 1, 1, 0},
        [HFLIP] = {0, 1, 1, 0},
        [EXPOSURE_AUTO] = {0, 1, 0, 0},
        [EXPOSURE_ABSOLUTE] = {1, 10000, 1, 156},
};


#define ctrl(index, name) ctrlvals[index].name


#define V4L2_CTRL_BRIGHTNESS 0
#define V4L2_CTRL_CONTRAST 1
#define V4L2_CTRL_SATURATION 2
#define V4L2_CTRL_AUTO_WHITE_BALANCE 4
#define V4L2_CTRL_GAMMA 5
#define V4L2_CTRL_GAIN 6
#define V4L2_CTRL_HFLIP 7
#define V4L2_CTRL_VFLIP 8
#define V4L2_CTRL_WHITE_BALANCE_TEMPERATURE 9
#define V4L2_CTRL_SHARPNESS 10
#define V4L2_CTRL_EXPOSURE_AUTO 11
#define V4L2_CTRL_EXPOSURE_ABSOLUTE 12


#define REGISTER_SHARPNESS_MIN 0
#define REGISTER_SHARPNESS_MAX 0

struct ar0330 {
	int numctrls;					// Ok
	u32 format_fourcc;
	u32 frmfmt_mode;
	u32 frate_index;
	struct regmap *b_regmap;			// Ok
	struct regmap *w_regmap;			// Ok
	struct regmap *dw_regmap;			// Ok
	int brightness;
	int contrast;
	int saturation;
	int sharpness;
	int gain;
	int gamma;
	int awb_mode;
	int awb_value;
	int vflip;
	int hflip;
	int exposure_mode;
	int exposure_value;
	struct v4l2_subdev *subdev;			// Ok
	struct media_pad pad;				// Ok
	struct camera_common_power_rail power;		// Ok
	unsigned int chip_id;
	struct v4l2_ctrl_handler ctrl_handler;		// Ok
	struct i2c_client *i2c_client;
	struct camera_common_pdata *pdata;		// Ok
	struct camera_common_data *s_data;		// Ok
	struct v4l2_ctrl *ctrls[];			// Ok
};

#endif
