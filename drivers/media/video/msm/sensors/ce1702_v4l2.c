/* Copyright (c) 2011-2012, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include "msm_sensor.h"
#include "ce1702_interface.h"

#define SENSOR_NAME "ce1702"
#define PLATFORM_DRIVER_NAME "msm_camera_ce1702"
#define ce1702_obj ce1702_##obj

#define CE1702_MULTI_IMAGE_PREVIEW_MODE

#define CE1702_STATUS_ON	1
#define CE1702_STATUS_OFF	0

/*LGE_UPDATE_S Color Engine Switch for camera, 2012.11.19, elin.lee@lge.com*/
#ifdef CONFIG_MACH_LGE
#if defined (CONFIG_FB_MSM_MIPI_LGIT_VIDEO_FHD_INVERSE_PT) || defined (CONFIG_FB_MSM_MIPI_LGIT_VIDEO_FHD_INVERSE_PT_PANEL)
#define LGIT_COLOR_ENGINE_SWITCH

#ifdef LGIT_COLOR_ENGINE_SWITCH
extern int mipi_lgit_lcd_color_engine_off(void);
extern int mipi_lgit_lcd_color_engine_on(void);
#endif
#endif
#endif
/*LGE_UPDATE_E Color Engine Switch for camera, 2012.11.19, elin.lee@lge.com*/

/* LGE_CHANGE_S, Define For CE1702 Sensor To Understand Easier, 2012.10.22, jungki.kim[Start] */
static bool ce1702_is_doing_touchaf = false;
static int ce1702_focus_mode = -1;
static int ce1702_manual_focus_val = -1;
static int ce1702_wb_mode = 0;
static int ce1702_brightness = 0;
static int ce1702_led_flash_mode = -1;
static int ce1702_iso_value = -1;
static int ce1702_scene_mode = CAMERA_SCENE_OFF;
static int ce1702_aec_awb_lock = -1;
static int ce1702_cam_preview = -1;
static int ce1702_rotation = -1;
static int ce1702_zoom_ratio = 255;
static int ce1702_size_info = 0x1c;
/* LGE_CHANGE_E, Define For CE1702 Sensor To Understand Easier, 2012.10.22, jungki.kim[End] */

static int ce1702_i2c_probe(struct i2c_client *client,const struct i2c_device_id *id);
static int ce1702_i2c_remove(struct i2c_client *client);
static int8_t ce1702_set_zoom_ratio(struct msm_sensor_ctrl_t *s_ctrl, int32_t zoom);
static int8_t ce1702_set_iso(struct msm_sensor_ctrl_t *s_ctrl, int32_t iso);

DEFINE_MUTEX(ce1702_mut);
extern struct msm_sensor_ctrl_t* ce1702_s_interface_ctrl;
static struct msm_sensor_ctrl_t ce1702_s_ctrl;

static struct delayed_work      ce1702_ISP_down_delayed_wq ;
static struct workqueue_struct  *wq_ce1702_ISP_down_work_queue;

static int ce1702_frame_mode = CE1702_FRAME_MAX;
/* LGE_CHANGE_S, Define For CE1702 output mode, 2012.11.10, elin.lee*/
static int isPreviewMode = 0;
static int isSingleCaptureMode = 0;
static int isTMSMode = 0;
static int isBurstMode = 0;
/* LGE_CHANGE_E, Define For CE1702 output mode, 2012.11.10, elin.lee*/

/* LGE_CHANGE_S, add the changing image size for GK project, 2012.10.19 youngil.yun@lge.com */
static struct dimen_t size_info;
/* LGE_CHANGE_E, add the changing image size for GK project, 2012.10.19 youngil.yun@lge.com */

static struct msm_camera_i2c_reg_conf ce1702_start_settings[] = {
#ifdef CE1702_YUV_IMAGE_PREVIEW_MODE
	{0x6B, 0x01},
#endif
#ifdef CE1702_MULTI_IMAGE_PREVIEW_MODE
	{0x63, 0x01},
#endif
};

static struct msm_camera_i2c_reg_conf ce1702_stop_settings[] = {
#ifdef CE1702_YUV_IMAGE_PREVIEW_MODE
		{0x6B, 0x00},
#endif
#ifdef CE1702_MULTI_IMAGE_PREVIEW_MODE
		{0x63, 0x00},
#endif
};

static struct msm_camera_i2c_reg_conf ce1702_prev_settings[] = {
	{0x54,	0x1C01, MSM_CAMERA_I2C_WORD_DATA},	//preview setting
	{0x11,		0x00},	  //AE/AWB unlock
};

static struct msm_camera_i2c_reg_conf ce1702_recommend_settings[] = {
	{0xF0,	0x00},		// FW start cmd
};

static struct v4l2_subdev_info ce1702_subdev_info[] = {
	{
	.code   = V4L2_MBUS_FMT_YUYV8_2X8,//V4L2_MBUS_FMT_SBGGR10_1X10,
	.colorspace = V4L2_COLORSPACE_JPEG,
	.fmt    = 1,
	.order    = 0,
	},
	/* more can be supported, to be added later */
};

static struct msm_camera_i2c_conf_array ce1702_init_conf[] = {
	{&ce1702_recommend_settings[0],
	ARRAY_SIZE(ce1702_recommend_settings), 0, MSM_CAMERA_I2C_BYTE_DATA}
};

static struct msm_camera_i2c_conf_array ce1702_confs[] = {
	{&ce1702_prev_settings[0],
	ARRAY_SIZE(ce1702_prev_settings), 0, MSM_CAMERA_I2C_BYTE_DATA},
	{&ce1702_prev_settings[0],
	ARRAY_SIZE(ce1702_prev_settings), 0, MSM_CAMERA_I2C_BYTE_DATA},
	{&ce1702_prev_settings[0],
	ARRAY_SIZE(ce1702_prev_settings), 0, MSM_CAMERA_I2C_BYTE_DATA},
	{&ce1702_prev_settings[0],
	ARRAY_SIZE(ce1702_prev_settings), 0, MSM_CAMERA_I2C_BYTE_DATA},
	{&ce1702_prev_settings[0],
	ARRAY_SIZE(ce1702_prev_settings), 0, MSM_CAMERA_I2C_BYTE_DATA},
	{&ce1702_prev_settings[0],
	ARRAY_SIZE(ce1702_prev_settings), 0, MSM_CAMERA_I2C_BYTE_DATA},
	{&ce1702_prev_settings[0],
	ARRAY_SIZE(ce1702_prev_settings), 0, MSM_CAMERA_I2C_BYTE_DATA},
	{&ce1702_prev_settings[0],
	ARRAY_SIZE(ce1702_prev_settings), 0, MSM_CAMERA_I2C_BYTE_DATA},
	{&ce1702_prev_settings[0],
	ARRAY_SIZE(ce1702_prev_settings), 0, MSM_CAMERA_I2C_BYTE_DATA},
	{&ce1702_prev_settings[0],
	ARRAY_SIZE(ce1702_prev_settings), 0, MSM_CAMERA_I2C_BYTE_DATA},
	{&ce1702_prev_settings[0],
	ARRAY_SIZE(ce1702_prev_settings), 0, MSM_CAMERA_I2C_BYTE_DATA},
};

static struct msm_sensor_output_info_t ce1702_dimensions[] = {
/* LGE_CHANGE_S, Change Support Sensor Info For GK/GV, 2012.10.25, jungki.kim[Start] */
	{
		/* full size  15fps*/
		.x_output = 0x1070, /* 4208 */
		.y_output = 0x0C30, /* 3120 */
		.line_length_pclk = 0x1070,
		.frame_length_lines = 0x0C30,
		.vt_pixel_clk = 225600000,
		.op_pixel_clk = 225600000,
		.binning_factor = 1,
	},
	{
		/* preview  30fps*/
		.x_output = 0x0500, /* 1280 */
		.y_output = 0x03C0, /* 960 */
		.line_length_pclk = 0x0500,
		.frame_length_lines = 0x03C0,
		.vt_pixel_clk = 225600000,
		.op_pixel_clk = 225600000,
		.binning_factor = 1,
	},
	{
		/* Video 30fps*/
		.x_output = 0x0780, /* 1920 */
		.y_output = 0x0440, /* 1088 */
		.line_length_pclk = 0x0780,
		.frame_length_lines = 0x0440,
		.vt_pixel_clk = 225600000,
		.op_pixel_clk = 225600000,
		.binning_factor = 1,
	},
	{
		/* preview 30fps*/
		.x_output = 0x0500, /* 1280 */
		.y_output = 0x02D0, /* 720 */
		.line_length_pclk = 0x0500,
		.frame_length_lines = 0x02D0,
		.vt_pixel_clk = 225600000,
		.op_pixel_clk = 225600000,
		.binning_factor = 1,
	},
	{
		/* preview 30fps*/
		.x_output = 0x0780, /* 1920 */
		.y_output = 0x0438, /* 1080 */
		.line_length_pclk = 0x0780,
		.frame_length_lines = 0x0438,
		.vt_pixel_clk = 225600000,
		.op_pixel_clk = 225600000,
		.binning_factor = 1,
	},
	{
		/* preview 30fps*/
		.x_output = 0x02D0, /* 720 */
		.y_output = 0x01E0, /* 480 */
		.line_length_pclk = 0x02D0,
		.frame_length_lines = 0x01E0,
		.vt_pixel_clk = 225600000,
		.op_pixel_clk = 225600000,
		.binning_factor = 1,
	},
	{
		/* preview 30fps*/
		.x_output = 0x0B0, /* 176 */
		.y_output = 0x0090, /* 144 */
		.line_length_pclk = 0x0B0,
		.frame_length_lines = 0x0090,
		.vt_pixel_clk = 225600000,
		.op_pixel_clk = 225600000,
		.binning_factor = 1,
	},
	{
		/* EIS FHD size*/
		.x_output = 0x0960, /* 2400 */
		.y_output = 0x0546, /* 1350 */
		.line_length_pclk = 0x0960,
		.frame_length_lines = 0x0546,
		.vt_pixel_clk = 225600000,
		.op_pixel_clk = 225600000,
		.binning_factor = 1,
	},
	{
		/* EIS HD size*/
		.x_output = 0x0640, /* 1600 */
		.y_output = 0x0384, /* 900 */
		.line_length_pclk = 0x0640,
		.frame_length_lines = 0x0384,
		.vt_pixel_clk = 225600000,
		.op_pixel_clk = 225600000,
		.binning_factor = 1,
	},
/* LGE_CHANGE_S, Add for TMC and Burst Shot ISP mode, 2012.11.10, elin.lee*/
      {
		/* zsl*/
		.x_output = 0x0500, /* 1280 */
		.y_output = 0x03C0, /* 960 */
		.line_length_pclk = 0x0500,
		.frame_length_lines = 0x03C0,
		.vt_pixel_clk = 225600000,
		.op_pixel_clk = 225600000,
		.binning_factor = 1,
	},
	{
		/* burst shot*/
		.x_output = 0x0500, /* 1280 */
		.y_output = 0x03C0, /* 960 */
		.line_length_pclk = 0x0500,
		.frame_length_lines = 0x03C0,
		.vt_pixel_clk = 225600000,
		.op_pixel_clk = 225600000,
		.binning_factor = 1,
	},
/* LGE_CHANGE_E, Add for TMC and Burst Shot ISP mode, 2012.11.10, elin.lee*/
/* LGE_CHANGE_E, Change Support Sensor Info For GK/GV, 2012.10.25, jungki.kim[End] */
};

static struct msm_sensor_output_reg_addr_t ce1702_reg_addr = {
	.x_output = 0x034C,
	.y_output = 0x034E,
	.line_length_pclk = 0x0342,
	.frame_length_lines = 0x0340,
};

static struct msm_sensor_id_info_t ce1702_id_info = {
	.sensor_id_reg_addr = 0x0000,
	.sensor_id = 0x0000,	//0x0091,
};

static struct msm_sensor_exp_gain_info_t ce1702_exp_gain_info = {
	.coarse_int_time_addr = 0x0202,
	.global_gain_addr = 0x0204,
	.vert_offset = 5,
};

static enum msm_camera_vreg_name_t ce1702_veg_seq[] = {
	CAM_VIO,
	CAM_VDIG,
	CAM_VANA,
	CAM_VAF,
	CAM_ISP_CORE,
	CAM_ISP_HOST,
	CAM_ISP_RAM,
	CAM_ISP_CAMIF,
	CAM_ISP_SYS,
};

static const struct i2c_device_id ce1702_i2c_id[] = {
	{SENSOR_NAME, (kernel_ulong_t)&ce1702_s_ctrl},
	{ }
};

static struct i2c_driver ce1702_i2c_driver = {
	.id_table = ce1702_i2c_id,
	.probe	= ce1702_i2c_probe,
	.remove = ce1702_i2c_remove,
	.driver = {
		.name = SENSOR_NAME,
	},
};

static struct pm_gpio gpio20_param = {
		.direction      = PM_GPIO_DIR_OUT,
		.output_buffer  = PM_GPIO_OUT_BUF_CMOS,
		.output_value   = 1,
		.pull           = PM_GPIO_PULL_NO,
		.vin_sel	= PM_GPIO_VIN_S4,
		.out_strength   = PM_GPIO_STRENGTH_MED,
		.function       = PM_GPIO_FUNC_NORMAL,
};
static struct pm_gpio gpio13_param = {
		.direction      = PM_GPIO_DIR_OUT,
		.output_buffer  = PM_GPIO_OUT_BUF_CMOS,
		.output_value   = 1,
		.pull           = PM_GPIO_PULL_NO,
		.vin_sel	= PM_GPIO_VIN_S4,
		.out_strength   = PM_GPIO_STRENGTH_MED,
		.function       = PM_GPIO_FUNC_NORMAL,
};
/* LGE_CHANGE_S, For GK/GV Rev.E bring-up, 2012.10.26, gayoung85.lee[Start] */
static struct pm_gpio gpio27_param = {
		.direction      = PM_GPIO_DIR_OUT,
		.output_buffer  = PM_GPIO_OUT_BUF_CMOS,
		.output_value   = 1,
		.pull           = PM_GPIO_PULL_NO,
		.vin_sel	= PM_GPIO_VIN_S4,
		.out_strength   = PM_GPIO_STRENGTH_MED,
		.function       = PM_GPIO_FUNC_NORMAL,
};
/* LGE_CHANGE_E, For GK/GV Rev.E bring-up, 2012.10.26, gayoung85.lee[End] */
static struct msm_cam_clk_info ce1702_cam_clk_info[] = {
	{"cam_clk", MSM_SENSOR_MCLK_24HZ},
};

static struct msm_camera_i2c_client ce1702_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
};

static int __init ce1702_sensor_init_module(void)
{
	LDBGI("%s: ENTER\n", __func__);
	return i2c_add_driver(&ce1702_i2c_driver);
}

static struct v4l2_subdev_core_ops ce1702_subdev_core_ops = {
	.ioctl = msm_sensor_subdev_ioctl,
	.s_power = msm_sensor_power,
};

static struct v4l2_subdev_video_ops ce1702_subdev_video_ops = {
	.enum_mbus_fmt = msm_sensor_v4l2_enum_fmt,
};

static struct v4l2_subdev_ops ce1702_subdev_ops = {
	.core = &ce1702_subdev_core_ops,
	.video  = &ce1702_subdev_video_ops,
};

/* LGE_CHANGE_S, Support Size Table, 2012.11.15, jungki.kim[Start] */

static struct ce1702_size_type ce1702_picture_table[] = {
	{4208, 3120, CE1702_SIZE_13MP},
	{4160, 3120, CE1702_SIZE_13MP},
	{4160, 2340, CE1702_SIZE_W10MP},
	{3264, 2448, CE1702_SIZE_8MP},
	{3264, 1836, CE1702_SIZE_W6MP},
	{3200, 1920, CE1702_SIZE_W6MP},
	{2560, 1920, CE1702_SIZE_5MP},
	{2304, 1296, CE1702_SIZE_W3MP},
	{2240, 1344, CE1702_SIZE_W3MP},
	{2048, 1536, CE1702_SIZE_3MP},
	{1920, 1088, CE1702_SIZE_FHD},
	{1920, 1080, CE1702_SIZE_FHD1},
	{1600, 1200, CE1702_SIZE_2MP},
	{1536, 864, CE1702_SIZE_W1MP},
	{1280, 960, CE1702_SIZE_1MP},
	{1280, 768, CE1702_SIZE_W1MP},
	{1280, 720, CE1702_SIZE_HD},
	{720, 480, CE1702_SIZE_D1},
	{640, 480, CE1702_SIZE_VGA},
	{352, 288, CE1702_SIZE_CIF},
	{320, 240, CE1702_SIZE_QVGA},
	{176, 144, CE1702_SIZE_QCIF},
};

static struct ce1702_size_type ce1702_postview_table[] = {
	{1280, 960, CE1702_SIZE_1MP},
	{1280, 720, CE1702_SIZE_HD},
};

static struct ce1702_size_type ce1702_preview_table[] = {
	{1920, 1088, CE1702_SIZE_FHD},
	{1920, 1080, CE1702_SIZE_FHD1},
	{1280, 960, CE1702_SIZE_1MP},
	{1280, 720, CE1702_SIZE_HD},
};

static struct ce1702_size_type ce1702_video_table[] = {
	{2400, 1350, CE1702_SIZE_EIS_FHD},
	{1920, 1088, CE1702_SIZE_FHD},
	{1600, 900, CE1702_SIZE_EIS_HD},
	{1280, 960, CE1702_SIZE_1MP},
	{1280, 720, CE1702_SIZE_HD},
	{720, 480, CE1702_SIZE_D1},
	{176, 144, CE1702_SIZE_QCIF},
};

uint16_t ce1702_get_supported_size(uint16_t width, uint16_t height, struct ce1702_size_type *table, int tbl_size)
{
	int  i;
	uint16_t rc = 0;

	LDBGI("%s: input size = [%d x %d] size=[%d]\n", __func__, width, height, tbl_size);
	for(i=0;i<tbl_size;i++) {
		if( (table[i].width == width) && (table[i].height == height) ) {
			rc = table[i].size_val;
			LDBGI("%s:1 supported size = [%d x %d] val=0x%02x\n", __func__,
					table[i].width, table[i].height, rc);
			break;
		}
	}
	if(rc == 0) {
		if( (width*3) == (height*4) ) {
			for(i=0;i<tbl_size;i++) {
				if( (table[i].width*3) == (table[i].height*4) ) {
					rc = table[i].size_val;
					LDBGI("%s:2 supported size = [%d x %d](4:3) val=0x%02x\n", __func__,
						table[i].width, table[i].height, rc);
					break;
				}
			}
		} else if( (width*9) == (height*16) ) {
			for(i=0;i<tbl_size;i++) {
				if( (table[i].width*9) == (table[i].height*16) ) {
					rc = table[i].size_val;
					LDBGI("%s:3 supported size = [%d x %d](16:9) val=0x%02x\n", __func__,
						table[i].width, table[i].height, rc);
					break;
				}
			}
		} else {
			rc = CE1702_SIZE_1MP;
			LDBGI("%s:4 supported size = [%d x %d](default) val=0x%02x\n", __func__,
					1280, 960, rc);
		}

	}

	if(rc == 0) {
		//Even though rc = 0,
		rc = CE1702_SIZE_1MP;
		LDBGI("%s: Not find any size. use this! = [%d x %d](default) val=0x%02x\n", __func__,
					1280, 960, rc);
	}

	return rc;
}
/* LGE_CHANGE_E, Support Size Table, 2012.11.15, jungki.kim[End] */

// Start LGE_BSP_CAMERA::kyounghoon.noh@lge.com 2012-06-25
int32_t ce1702_sensor_setting(struct msm_sensor_ctrl_t *s_ctrl,
			int update_type, int res)
{
	int32_t rc = 0;
	uint8_t wdata[20];

#ifndef STOP_LENS_WORKAROUND_1205
	uint8_t rdata[2];
	uint8_t result =0;
	int cnt;
#endif

       LDBGI("%s: update_type = %d, i2c_add = %d, res=%d \n", __func__, update_type, ce1702_s_interface_ctrl->sensor_i2c_addr, res);

	if (update_type == MSM_SENSOR_REG_INIT) {
		//s_ctrl->func_tbl->sensor_stop_stream(s_ctrl);
		//msm_sensor_write_init_settings(s_ctrl);
		//mdelay(500);
		CE_FwStart();
		ce1702_set_iso(s_ctrl, CAMERA_ISO_TYPE_DEFAULT);
		ce1702_set_model_name();
	} else if (update_type == MSM_SENSOR_UPDATE_PERIODIC) {
		//msm_sensor_write_res_settings(s_ctrl, res);
		if(res == MSM_SENSOR_RES_QTR || res ==  MSM_SENSOR_RES_3  || res ==  MSM_SENSOR_RES_2 || res == MSM_SENSOR_RES_4
			 || res == MSM_SENSOR_RES_5 || res == MSM_SENSOR_RES_6 || res == MSM_SENSOR_RES_7|| res == MSM_SENSOR_RES_8)
		{
			ce1702_frame_mode = CE1702_MODE_PREVIEW;
		}
		else if(res == MSM_SENSOR_RES_FULL)
		{
			//s_ctrl->func_tbl->sensor_stop_stream(s_ctrl);

			ce1702_frame_mode = CE1702_MODE_SINGLE_SHOT;
#ifndef STOP_LENS_WORKAROUND_1205
			if(ce1702_focus_mode == AF_MODE_AUTO) {
				wdata[0] = 0x00;
				ce1702_i2c_write(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x35, wdata,  1);	// Stop VCM
				cnt = 0;
				LDBGI("%s: stop lens ce1702_i2c_read successed, result=%d\n", __func__, result);
				do {
					//rdata[0] = 0x00;
					ce1702_i2c_read(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x24, rdata, 1, &result,  1);
					if ((result & 0x01) == 0){
						break;
					}
					cnt++;
					mdelay(5); //yt.jeon 1115 optimize delay time
				} while (cnt < 100); 	//lens running
			}
#endif
// Mode 1: Single Shot
#if 1
#if 0
			cnt = sizeof(ce1702_postview_table) / sizeof(ce1702_postview_table[0]);
			wdata[0] = ce1702_get_supported_size(size_info.picture_width, size_info.picture_height, ce1702_postview_table, cnt);

			wdata[1] = 0x05;
			ce1702_i2c_write(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x55, wdata,  2);

			LDBGI("%s: Capture Size:: picture width = %d , picture height = %d\n", __func__,size_info.picture_width,size_info.picture_height);

			cnt = sizeof(ce1702_picture_table) / sizeof(ce1702_picture_table[0]);
			wdata[0] = ce1702_get_supported_size(size_info.picture_width, size_info.picture_height, ce1702_picture_table, cnt);

			wdata[1] = 0x00;
			wdata[2] = 0x00;
			wdata[3] = 0x81;
			ce1702_i2c_write(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x73, wdata,  4);

			wdata[0] = 0x21;
			wdata[1] = 0x00;
			ce1702_i2c_write(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x07, wdata,  2);
#endif   //fix zoom capture

#endif

		}
/* LGE_CHANGE_S, time machine shot ISP setting for GK project, 2012.11.07 elin.lee@lge.com */
		else if(res == MSM_SENSOR_RES_ZSL){
			LDBGE("%s: E, Time machine shot setting\n", __func__);
			// Multi-Image output setting
			ce1702_frame_mode = CE1702_MODE_TMS_SHOT;

#ifndef STOP_LENS_WORKAROUND_1205
			if(ce1702_focus_mode == AF_MODE_AUTO) {
				wdata[0] = 0x00;
				ce1702_i2c_write(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x35, wdata,  1);	// Stop VCM
				cnt = 0;
				LDBGI("%s: stop lens ce1702_i2c_read successed, result=%d\n", __func__, result);
				do {
					//rdata[0] = 0x00;
					ce1702_i2c_read(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x24, rdata, 1, &result,  1);
					if ((result & 0x01) == 0){
						break;
					}
					cnt++;
					mdelay(5); //yt.jeon 1115 optimize delay time
				} while (cnt < 100); 	//lens running
			}
#endif

#if 0
// LGE_CHANGE_S, rotation fixed, 2012.10.24 elin.lee@lge.com
			wdata[0] = 0x03;
			ce1702_i2c_write(ce1702_s_interface_ctrl->sensor_i2c_addr, 0xAE, wdata,  1);
// LGE_CHANGE_E, rotation fixed, 2012.10.24 elin.lee@lge.com

			wdata[0] = 0x1C;
			wdata[1] = 0x31;
			wdata[2] = 0x00;
			wdata[3] = 0x00;
			wdata[4] = 0x00;
			wdata[5] = 0x01;
			wdata[6] = 0x00;

			ce1702_i2c_write(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x62, wdata,  7);
			// Multi-Image output setting
			wdata[0] = 0x00;
			wdata[1] = 0x60;
			wdata[2] = 0x09;
			wdata[3] = 0xD0;
			wdata[4] = 0x07;
			wdata[5] = 0x01;
			wdata[6] = 0x20;

			ce1702_i2c_write(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x90, wdata,  7);
 #endif
		}
/* LGE_CHANGE_E, time machine shot ISP setting for GK project, 2012.11.07 elin.lee@lge.com */
/* LGE_CHANGE_S, burst shot ISP setting for GK project, 2012.11.07 elin.lee@lge.com */
		else if(res == MSM_SENSOR_RES_BURST){
			// Mode 1: Burst Shot
			LDBGE("%s: E, Burst shot setting\n", __func__);
			ce1702_frame_mode = CE1702_MODE_BURST_SHOT;

#ifndef STOP_LENS_WORKAROUND_1205
			if(ce1702_focus_mode == AF_MODE_AUTO) {
				wdata[0] = 0x00;
				ce1702_i2c_write(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x35, wdata,  1);	// Stop VCM
				cnt = 0;
				LDBGI("%s: stop lens ce1702_i2c_read successed, result=%d\n", __func__, result);
				do {
					//rdata[0] = 0x00;
					ce1702_i2c_read(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x24, rdata, 1, &result,  1);
					if ((result & 0x01) == 0){
						break;
					}
					cnt++;
					mdelay(5); //yt.jeon 1115 optimize delay time
				} while (cnt < 100); 	//lens running
			}
#endif

		}
/* LGE_CHANGE_E, burst shot ISP setting for GK project, 2012.11.07 elin.lee@lge.com */

		LDBGI("%s: ce1702_frame_mode = %d, res=%d \n", __func__, ce1702_frame_mode, res);

		v4l2_subdev_notify(&s_ctrl->sensor_v4l2_subdev,
		NOTIFY_PCLK_CHANGE, &s_ctrl->msm_sensor_reg->
		output_settings[res].op_pixel_clk);

		//msleep(60); //yt.jeon 1115 optimize delay time
	}
	return rc;
}

static int ce1702_i2c_probe(struct i2c_client *client,const struct i2c_device_id *id)
{
	int rc = 0;
       struct msm_sensor_ctrl_t *s_ctrl;
	LDBGI("%s :%s_i2c_probe called\n", __func__, client->name);
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		LDBGE("%s %s i2c_check_functionality failed\n",
			__func__, client->name);
		rc = -EFAULT;
		return rc;
	}

	s_ctrl = (struct msm_sensor_ctrl_t *)(id->driver_data);
	ce1702_s_interface_ctrl = s_ctrl;
	if (s_ctrl->sensor_i2c_client != NULL) {
		s_ctrl->sensor_i2c_client->client = client;
		if (s_ctrl->sensor_i2c_addr != 0)
			s_ctrl->sensor_i2c_client->client->addr =
				s_ctrl->sensor_i2c_addr;
	} else {
		LDBGE("%s %s sensor_i2c_client NULL\n",
			__func__, client->name);
		rc = -EFAULT;
		return rc;
	}

	s_ctrl->sensordata = client->dev.platform_data;
	if (s_ctrl->sensordata == NULL) {
		LDBGE("%s %s NULL sensor data\n", __func__, client->name);
		return -EFAULT;
	}

	init_suspend(); //LGE_CHANGE_S [muhan2k] 2009-11-9 Android PJT
	ce1702_sysfs_add(&client->dev.kobj);

	wq_ce1702_ISP_down_work_queue = create_singlethread_workqueue("wq_ce1702_ISP_down_work_queue");
	if (wq_ce1702_ISP_down_work_queue == NULL) {
		LDBGE("wq_ce1702_ISP_down_work_queue fail	\n");
		rc = -ENOMEM;
		goto probe_failure;
	}

	LDBGI("%s: Get Mutex to start delayed workqueue for ISP Bin Download !!!!![%lu]\n", __func__, jiffies);

	INIT_DELAYED_WORK(&ce1702_ISP_down_delayed_wq, ce1702_wq_ISP_upload);

    if(wq_ce1702_ISP_down_work_queue)
        queue_delayed_work(wq_ce1702_ISP_down_work_queue, &ce1702_ISP_down_delayed_wq, HZ );

	snprintf(s_ctrl->sensor_v4l2_subdev.name,
		sizeof(s_ctrl->sensor_v4l2_subdev.name), "%s", id->name);

	if (!s_ctrl->wait_num_frames)
			s_ctrl->wait_num_frames = 1 * Q10;

	LDBGE("%s %s probe succeeded\n", __func__, client->name);

	v4l2_i2c_subdev_init(&s_ctrl->sensor_v4l2_subdev, client,
		s_ctrl->sensor_v4l2_subdev_ops);
	s_ctrl->sensor_v4l2_subdev.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	media_entity_init(&s_ctrl->sensor_v4l2_subdev.entity, 0, NULL, 0);
	s_ctrl->sensor_v4l2_subdev.entity.type = MEDIA_ENT_T_V4L2_SUBDEV;
	s_ctrl->sensor_v4l2_subdev.entity.group_id = SENSOR_DEV;
	s_ctrl->sensor_v4l2_subdev.entity.name =
		s_ctrl->sensor_v4l2_subdev.name;
	msm_sensor_register(&s_ctrl->sensor_v4l2_subdev);
	s_ctrl->sensor_v4l2_subdev.entity.revision =
		s_ctrl->sensor_v4l2_subdev.devnode->num;

	LDBGI("ce1702_i2c_probe ends successfully\n");

	return 0;
probe_failure:
//	kfree(ce1702_sensorw);
//	ce1702_sensorw = NULL;
	LDBGE("ce1702_probe failed!\n");
	return rc;

}

static int ce1702_i2c_remove(struct i2c_client *client)
{
	/* TODO: this function is called twice. Handle It! */

	struct ce1702_work *sensorw = i2c_get_clientdata(client);

	LDBGI("%s: called\n", __func__);

	ce1702_sysfs_rm(&client->dev.kobj);
	free_irq(client->irq, sensorw);
	deinit_suspend();
	kfree(sensorw);
	return 0;
}


int32_t ce1702_msm_sensor_enable_i2c_mux(struct msm_camera_i2c_conf *i2c_conf)
{
	struct v4l2_subdev *i2c_mux_sd =
		dev_get_drvdata(&i2c_conf->mux_dev->dev);
	v4l2_subdev_call(i2c_mux_sd, core, ioctl,
		VIDIOC_MSM_I2C_MUX_INIT, NULL);
	v4l2_subdev_call(i2c_mux_sd, core, ioctl,
		VIDIOC_MSM_I2C_MUX_CFG, (void *)&i2c_conf->i2c_mux_mode);
	return 0;
}

int32_t ce1702_msm_sensor_disable_i2c_mux(struct msm_camera_i2c_conf *i2c_conf)
{
	struct v4l2_subdev *i2c_mux_sd =
		dev_get_drvdata(&i2c_conf->mux_dev->dev);
	v4l2_subdev_call(i2c_mux_sd, core, ioctl,
				VIDIOC_MSM_I2C_MUX_RELEASE, NULL);
	return 0;
}

// Initiate Parameters by jungki.kim@lge.com
void ce1702_param_init(void)
{
	ce1702_is_doing_touchaf = false;
	ce1702_wb_mode = 0;
	ce1702_brightness = 0;
	ce1702_focus_mode = -1;
	ce1702_manual_focus_val = -1;
	ce1702_led_flash_mode = -1;
	ce1702_aec_awb_lock = -1;
	ce1702_cam_preview = -1;
	ce1702_iso_value = -1;
	ce1702_scene_mode = CAMERA_SCENE_OFF;
	ce1702_rotation = -1;
	ce1702_zoom_ratio = 255;
	ce1702_size_info = 0x1c;
}

int32_t ce1702_power_up(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;
	struct msm_camera_sensor_info *data = s_ctrl->sensordata;
	LDBGI("%s: %d\n", __func__, __LINE__);

	ce1702_param_init();	// init parameters by jungki.kim
	ce1702_frame_mode = CE1702_FRAME_MAX;
	if(isPreviewMode == 1 || isSingleCaptureMode == 1 || isTMSMode==1 || isBurstMode==1){
		LDBGE("%s: Preview is still running!!! isPreviewMode = %d, isSingleCaptureMode = %d, isTMSMode=%d, isBurstMode=%d \n", __func__, isPreviewMode, isSingleCaptureMode, isTMSMode, isBurstMode);
		isPreviewMode = 0;
		isSingleCaptureMode = 0;
		isTMSMode = 0;
		isBurstMode = 0;
	}

	s_ctrl->reg_ptr = kzalloc(sizeof(struct regulator *)
			* data->sensor_platform_info->num_vreg, GFP_KERNEL);
	if (!s_ctrl->reg_ptr) {
		LDBGE("%s: could not allocate mem for regulators\n",
			__func__);
		return -ENOMEM;
	}

	LDBGE("%s: before request gpio, sensor name : %s", __func__, s_ctrl->sensordata->sensor_name);
	rc = msm_camera_request_gpio_table(data, 1);
	if (rc < 0) {
		LDBGE("%s: request gpio failed\n", __func__);
		goto request_gpio_failed;
	}

	rc = gpio_request(ISP_HOST_INT, "ISP_HOST_INT");
	rc = gpio_request(ISP_STBY, "ISP_STBY");
/* LGE_CHANGE_S, For GK/GV Rev.E bring-up, 2012.10.26, gayoung85.lee[Start] */
#if defined(CONFIG_MACH_APQ8064_GKKT) || defined(CONFIG_MACH_APQ8064_GKSK) || defined(CONFIG_MACH_APQ8064_GKU) || defined(CONFIG_MACH_APQ8064_GKATT)
	if(lge_get_board_revno() == HW_REV_E ||lge_get_board_revno() == HW_REV_C ||lge_get_board_revno() == HW_REV_D){
		LDBGI("%s: Revision check! ISP_RST GPIO No.%d\n",__func__,ISP_RST );
		rc = gpio_request(ISP_RST, "ISP_RST");
	}
	if (rc) {
		LDBGE("%s: PM request gpio failed\n", __func__);
	}
#elif defined(CONFIG_MACH_APQ8064_GVDCM)
	if(lge_get_board_revno() >= HW_REV_C){
		LDBGI("%s: Revision check! ISP_RST GPIO No.%d\n",__func__,ISP_RST );
		rc = gpio_request(ISP_RST, "ISP_RST");
	}
	if (rc) {
		LDBGE("%s: PM request gpio failed\n", __func__);
	}
#endif
/* LGE_CHANGE_E, For GK/GV Rev.E bring-up, 2012.10.26, gayoung85.lee[End] */

	rc = msm_camera_config_vreg(&s_ctrl->sensor_i2c_client->client->dev,
			s_ctrl->sensordata->sensor_platform_info->cam_vreg,
			s_ctrl->sensordata->sensor_platform_info->num_vreg,
			s_ctrl->vreg_seq,
			s_ctrl->num_vreg_seq,
			s_ctrl->reg_ptr, 1);

	if (rc < 0) {
		LDBGE("%s: regulator on failed\n", __func__);
		goto config_vreg_failed;
	}

	rc = msm_camera_enable_vreg(&s_ctrl->sensor_i2c_client->client->dev,
			s_ctrl->sensordata->sensor_platform_info->cam_vreg,
			s_ctrl->sensordata->sensor_platform_info->num_vreg,
			s_ctrl->vreg_seq,
			s_ctrl->num_vreg_seq,
			s_ctrl->reg_ptr, 1);
	if (rc < 0) {
		LDBGE("%s: enable regulator failed\n", __func__);
		goto enable_vreg_failed;
	}

	mdelay(1);

	if (s_ctrl->clk_rate != 0)
		ce1702_cam_clk_info->clk_rate = s_ctrl->clk_rate;

	rc = msm_cam_clk_enable(&s_ctrl->sensor_i2c_client->client->dev,
		ce1702_cam_clk_info, s_ctrl->cam_clk, ARRAY_SIZE(ce1702_cam_clk_info), 1);
	if (rc < 0) {
		LDBGE("%s: clk enable failed\n", __func__);
		goto enable_clk_failed;
	}
	mdelay(5);

	rc =pm8xxx_gpio_config(ISP_HOST_INT, &gpio20_param);
	rc =pm8xxx_gpio_config(ISP_STBY, &gpio13_param);
/* LGE_CHANGE_S, For GK/GV Rev.E bring-up, 2012.10.26, gayoung85.lee[Start] */
#if defined(CONFIG_MACH_APQ8064_GKKT) || defined(CONFIG_MACH_APQ8064_GKSK) || defined(CONFIG_MACH_APQ8064_GKU) || defined(CONFIG_MACH_APQ8064_GKATT)
	if(lge_get_board_revno() == HW_REV_E ||lge_get_board_revno() == HW_REV_C ||lge_get_board_revno() == HW_REV_D){
		LDBGI("%s: Revision check! ISP_RST GPIO No.%d\n",__func__,ISP_RST );
		rc =pm8xxx_gpio_config(ISP_RST, &gpio27_param);
	}
	if (rc) {
		LDBGE("%s: pm8xxx_gpio_config on failed\n", __func__);
	}
#elif defined(CONFIG_MACH_APQ8064_GVDCM)
	if(lge_get_board_revno() >= HW_REV_C){
		LDBGI("%s: Revision check! ISP_RST GPIO No.%d\n",__func__,ISP_RST );
		rc =pm8xxx_gpio_config(ISP_RST, &gpio27_param);
	}
	if (rc) {
		LDBGE("%s: pm8xxx_gpio_config on failed\n", __func__);
	}
#endif
/* LGE_CHANGE_E, For GK/GV Rev.E bring-up, 2012.10.26, gayoung85.lee[End] */

	rc = gpio_direction_output(ISP_STBY, 1);

	mdelay(1);

/* LGE_CHANGE_S, For GK/GV Rev.E bring-up, 2012.10.26, gayoung85.lee[Start] */
#if defined(CONFIG_MACH_APQ8064_GKKT) || defined(CONFIG_MACH_APQ8064_GKSK) || defined(CONFIG_MACH_APQ8064_GKU) || defined(CONFIG_MACH_APQ8064_GKATT)
	if(lge_get_board_revno() == HW_REV_E ||lge_get_board_revno() == HW_REV_C ||lge_get_board_revno() == HW_REV_D)
	{
		LDBGI("%s: Revision check! ISP_RST GPIO No.%d\n",__func__,ISP_RST );
		rc = gpio_direction_output(ISP_RST, 1);
		if (rc < 0) {
			LDBGE("%s: config gpio failed\n", __func__);
			goto config_gpio_failed;
		}
	}else
#elif defined(CONFIG_MACH_APQ8064_GVDCM)
	if(lge_get_board_revno() >= HW_REV_C)
	{
		LDBGI("%s: Revision check! ISP_RST GPIO No.%d\n",__func__,ISP_RST );
		rc = gpio_direction_output(ISP_RST, 1);
		if (rc < 0) {
			LDBGE("%s: config gpio failed\n", __func__);
			goto config_gpio_failed;
		}
	}else
#endif
/* LGE_CHANGE_E, For GK/GV Rev.E bring-up, 2012.10.26, gayoung85.lee[End] */
	{
		rc = msm_camera_config_gpio_table(data, 1);
		if (rc < 0) {
			LDBGE("%s: config gpio failed\n", __func__);
			goto config_gpio_failed;
		}
	}

	usleep_range(1000, 2000);
	if (data->sensor_platform_info->ext_power_ctrl != NULL)
		data->sensor_platform_info->ext_power_ctrl(1);

	if (data->sensor_platform_info->i2c_conf &&
		data->sensor_platform_info->i2c_conf->use_i2c_mux)
		ce1702_msm_sensor_enable_i2c_mux(data->sensor_platform_info->i2c_conf);

	return rc;

enable_clk_failed:
/* LGE_CHANGE_S, For GK/GV Rev.E bring-up, 2012.10.26, gayoung85.lee[Start] */
#if defined(CONFIG_MACH_APQ8064_GKKT) || defined(CONFIG_MACH_APQ8064_GKSK) || defined(CONFIG_MACH_APQ8064_GKU) || defined(CONFIG_MACH_APQ8064_GKATT)
	if(lge_get_board_revno() == HW_REV_E ||lge_get_board_revno() == HW_REV_C ||lge_get_board_revno() == HW_REV_D){
		LDBGI("%s: Revision check! ISP_RST GPIO No.%d\n",__func__,ISP_RST );
		rc = gpio_direction_output(ISP_RST, 0);
	}else
#elif defined(CONFIG_MACH_APQ8064_GVDCM)
	if(lge_get_board_revno() >= HW_REV_C){
		LDBGI("%s: Revision check! ISP_RST GPIO No.%d\n",__func__,ISP_RST );
		rc = gpio_direction_output(ISP_RST, 0);
	}else
#endif
/* LGE_CHANGE_E, For GK/GV Rev.E bring-up, 2012.10.26, gayoung85.lee[End] */
	{
		msm_camera_config_gpio_table(data, 0);
	}
config_gpio_failed:
	msm_camera_enable_vreg(&s_ctrl->sensor_i2c_client->client->dev,
			s_ctrl->sensordata->sensor_platform_info->cam_vreg,
			s_ctrl->sensordata->sensor_platform_info->num_vreg,
			s_ctrl->vreg_seq,
			s_ctrl->num_vreg_seq,
			s_ctrl->reg_ptr, 0);

enable_vreg_failed:
	msm_camera_config_vreg(&s_ctrl->sensor_i2c_client->client->dev,
		s_ctrl->sensordata->sensor_platform_info->cam_vreg,
		s_ctrl->sensordata->sensor_platform_info->num_vreg,
		s_ctrl->vreg_seq,
		s_ctrl->num_vreg_seq,
		s_ctrl->reg_ptr, 0);
config_vreg_failed:
	msm_camera_request_gpio_table(data, 0);
request_gpio_failed:
	kfree(s_ctrl->reg_ptr);
	s_ctrl->reg_ptr = NULL;
	return rc;
}

int32_t ce1702_power_down(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;
	struct msm_camera_sensor_info *data = s_ctrl->sensordata;
	LDBGI("%s\n", __func__);

/*LGE_UPDATE_S Color Engine Switch for camera, 2012.11.19, elin.lee@lge.com*/
#ifdef LGIT_COLOR_ENGINE_SWITCH
    if(system_state != SYSTEM_BOOTING) {
      mipi_lgit_lcd_color_engine_on();
    }
#endif
/*LGE_UPDATE_E Color Engine Switch for camera, 2012.11.19, elin.lee@lge.com*/

	rc = ce1702_set_VCM_default_position(s_ctrl);//20121212, hyungmoo.huh@lge.com, for Tick noise

	if (data->sensor_platform_info->i2c_conf &&
		data->sensor_platform_info->i2c_conf->use_i2c_mux)
		ce1702_msm_sensor_disable_i2c_mux(
			data->sensor_platform_info->i2c_conf);
/* LGE_CHANGE_S, For GK/GV Rev.E bring-up, 2012.10.26, gayoung85.lee[Start] */
#if defined(CONFIG_MACH_APQ8064_GKKT) || defined(CONFIG_MACH_APQ8064_GKSK) || defined(CONFIG_MACH_APQ8064_GKU) || defined(CONFIG_MACH_APQ8064_GKATT)
 	if(lge_get_board_revno() == HW_REV_E ||lge_get_board_revno() == HW_REV_C ||lge_get_board_revno() == HW_REV_D){
		LDBGI("%s: Revision check! ISP_RST GPIO No.%d\n",__func__,ISP_RST );
		rc = gpio_direction_output(ISP_RST, 0);
 	}else
#elif defined(CONFIG_MACH_APQ8064_GVDCM)
	if(lge_get_board_revno() >= HW_REV_C){
		LDBGI("%s: Revision check! ISP_RST GPIO No.%d\n",__func__,ISP_RST );
		rc = gpio_direction_output(ISP_RST, 0);
	}else
#endif
/* LGE_CHANGE_E, For GK/GV Rev.E bring-up, 2012.10.26, gayoung85.lee[End] */
 	{
		msm_camera_config_gpio_table(data, 0);
 	}
	if (data->sensor_platform_info->ext_power_ctrl != NULL)
		data->sensor_platform_info->ext_power_ctrl(0);
	msm_cam_clk_enable(&s_ctrl->sensor_i2c_client->client->dev,
		ce1702_cam_clk_info, s_ctrl->cam_clk, ARRAY_SIZE(ce1702_cam_clk_info), 0);

/* LGE_CHANGE_S, Reduce Power Consumption, 2012.11.20, jungki.kim[Start] */
#if 0
	rc =pm8xxx_gpio_config(ISP_HOST_INT, &gpio20_param);
	rc =pm8xxx_gpio_config(ISP_STBY, &gpio13_param);

/* LGE_CHANGE_S, For GK/GV Rev.E bring-up, 2012.10.26, gayoung85.lee[Start] */
#if defined(CONFIG_MACH_APQ8064_GKKT) || defined(CONFIG_MACH_APQ8064_GKSK) || defined(CONFIG_MACH_APQ8064_GKU) || defined(CONFIG_MACH_APQ8064_GKATT)
 	if(lge_get_board_revno() == HW_REV_E){
		LDBGI("%s: Revision check! ISP_RST GPIO No.%d\n",__func__,ISP_RST );
		rc =pm8xxx_gpio_config(ISP_RST, &gpio27_param);
	}
	if (rc) {
		LDBGE("%s: pm8xxx_gpio_config on failed\n", __func__);
	}
#elif defined(CONFIG_MACH_APQ8064_GVDCM)
	if(lge_get_board_revno() >= HW_REV_C){
		LDBGI("%s: Revision check! ISP_RST GPIO No.%d\n",__func__,ISP_RST );
		rc =pm8xxx_gpio_config(ISP_RST, &gpio27_param);
	}
	if (rc) {
		LDBGE("%s: pm8xxx_gpio_config on failed\n", __func__);
	}
#endif
/* LGE_CHANGE_E, For GK/GV Rev.E bring-up, 2012.10.26, gayoung85.lee[End] */
#endif
/* LGE_CHANGE_E, Reduce Power Consumption, 2012.11.20, jungki.kim[End] */

	mdelay(1);

	rc = gpio_direction_output(ISP_HOST_INT, 0);
	rc = gpio_direction_output(ISP_STBY, 0);
	if (rc) {
		LDBGE("%s: gpio_direction_output enable failed\n", __func__);
	}

/* LGE_CHANGE_S, Avoid Kernel Panic, 2012.11.12, jungki.kim[Start] */
	if(s_ctrl->reg_ptr != NULL) {
		msm_camera_enable_vreg(&s_ctrl->sensor_i2c_client->client->dev,
			s_ctrl->sensordata->sensor_platform_info->cam_vreg,
			s_ctrl->sensordata->sensor_platform_info->num_vreg,
			s_ctrl->vreg_seq,
			s_ctrl->num_vreg_seq,
			s_ctrl->reg_ptr, 0);
		msm_camera_config_vreg(&s_ctrl->sensor_i2c_client->client->dev,
			s_ctrl->sensordata->sensor_platform_info->cam_vreg,
			s_ctrl->sensordata->sensor_platform_info->num_vreg,
			s_ctrl->vreg_seq,
			s_ctrl->num_vreg_seq,
			s_ctrl->reg_ptr, 0);
		msm_camera_request_gpio_table(data, 0);
		kfree(s_ctrl->reg_ptr);
	} else {
		// NULL!
		LDBGE("%s: No Regulator Pointer!\n", __func__);
	}
/* LGE_CHANGE_E, Avoid Kernel Panic, 2012.11.12, jungki.kim[End] */

	gpio_free(ISP_HOST_INT);
	gpio_free(ISP_STBY);
/* LGE_CHANGE_S, For GK/GV Rev.E bring-up, 2012.10.26, gayoung85.lee[Start] */
#if defined(CONFIG_MACH_APQ8064_GKKT) || defined(CONFIG_MACH_APQ8064_GKSK) || defined(CONFIG_MACH_APQ8064_GKU) || defined(CONFIG_MACH_APQ8064_GKATT)
 	if(lge_get_board_revno() == HW_REV_E ||lge_get_board_revno() == HW_REV_C ||lge_get_board_revno() == HW_REV_D){
		LDBGI("%s: Revision check! ISP_RST GPIO No.%d\n",__func__,ISP_RST );
		gpio_free(ISP_RST);
	}
#elif defined(CONFIG_MACH_APQ8064_GVDCM)
if(lge_get_board_revno() >= HW_REV_C){
	LDBGI("%s: Revision check! ISP_RST GPIO No.%d\n",__func__,ISP_RST );
	gpio_free(ISP_RST);
}
#endif
/* LGE_CHANGE_E, For GK/GV Rev.E bring-up, 2012.10.26, gayoung85.lee[End] */
	ce1702_frame_mode = CE1702_FRAME_MAX;
	if(isPreviewMode == 1 || isSingleCaptureMode == 1 || isTMSMode==1 || isBurstMode==1){
		LDBGE("%s: Preview is still running!!! isPreviewMode = %d, isSingleCaptureMode = %d, isTMSMode=%d, isBurstMode=%d \n", __func__, isPreviewMode, isSingleCaptureMode, isTMSMode, isBurstMode);
		isPreviewMode = 0;
		isSingleCaptureMode = 0;
		isTMSMode = 0;
		isBurstMode = 0;
	}
    ce1702_focus_mode = -1;

	return 0;
}

int32_t ce1702_match_id(struct msm_sensor_ctrl_t *s_ctrl)
{
	LDBGI("%s\n", __func__);

	//couldn't detect the match id
	if(dest_location_firmware == CE1702_SDCARD2){
		LDBGI("CE1702 firmware update !! \n");
		ce1702_isp_fw_full_upload();
		ce1702_check_flash_version();
		dest_location_firmware = CE1702_NANDFLASH; // only one-time firmware update...
	}
	return CE1702_OK ;
}
//youngil.yun@lge.com 2012-07-09 - end


//We need to excute these function lately because these have to try under previewing condition.
void ce1702_sensor_set_param_lately(struct msm_sensor_ctrl_t *s_ctrl)
{
	/*
		I found that focus mode does not work on the first time.
		Then I check whether it is previewing or not and it is not on previewing!!
		So I need it try later.
	*/

	if( ce1702_aec_awb_lock >= 10) {
		LDBGI("%s: ce1702_aec_awb_lock=[%d] EXTRA WORK TO DO!\n", __func__, ce1702_aec_awb_lock);
		ce1702_set_aec_awb_lock(s_ctrl, ce1702_aec_awb_lock);
	}

	if ( ((ce1702_frame_mode == CE1702_MODE_PREVIEW) && (isSingleCaptureMode == 0))
		|| ((ce1702_frame_mode == CE1702_MODE_TMS_SHOT) && (isTMSMode == 1)) ) {
		LDBGI("%s: ce1702_focus_mode=[%d] EXTRA WORK TO DO!\n", __func__, ce1702_focus_mode);
		if(ce1702_focus_mode == AF_MODE_CAF_PICTURE) {
			ce1702_set_caf(AF_MODE_CAF_PICTURE);
		} else if(ce1702_focus_mode == AF_MODE_CAF_VIDEO) {
			ce1702_set_caf(AF_MODE_CAF_VIDEO);
		}
	}

	if(ce1702_manual_focus_val != -1) {
		LDBGI("%s: ce1702_manual_focus_val=[%d] EXTRA WORK TO DO!\n", __func__, ce1702_manual_focus_val);
		ce1702_set_manual_focus_length(s_ctrl, ce1702_manual_focus_val);
		ce1702_manual_focus_val = -1;
	}
}

void ce1702_sensor_start_stream(struct msm_sensor_ctrl_t *s_ctrl)
{
	uint8_t result;
	int retry_cnt;
	uint8_t rdata[2];
	uint8_t wdata[7];
	int32_t rc = 0;
	int cnt = 0;
//	int dim_rate;

	retry_cnt = 0;
/*LGE_UPDATE_S Color Engine Switch for camera, 2012.11.19, elin.lee@lge.com*/
#ifdef LGIT_COLOR_ENGINE_SWITCH
    if(system_state != SYSTEM_BOOTING) {
      mipi_lgit_lcd_color_engine_off();
    }
#endif
/*LGE_UPDATE_E Color Engine Switch for camera, 2012.11.19, elin.lee@lge.com*/

	LDBGI("%s: isPreviewMode=%d,  isSingleCaptureMode =%d, isTMSMode=%d, isBurstMode =%d\n", __func__, isPreviewMode, isSingleCaptureMode, isTMSMode, isBurstMode);

	if(ce1702_frame_mode == CE1702_MODE_PREVIEW && isPreviewMode == 0){
// 1207 yt.jeon@lge.com set various parameters only if initial preview.
		rdata[0] = 0x00;
		rc = ce1702_i2c_read(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x6C, rdata, 0,  &result,  1);
		if (rc < 0)
			LDBGE("%s: ce1702_i2c_read failed \n", __func__);
		LDBGE("%s: preview status = %d \n", __func__,result);

		if (result == 0x00) {//if stopping
			LDBGE("%s: CE1702_MODE_PREVIEW initial start\n", __func__);
			LDBGI("%s: video_width = [%d], video_height = [%d]\n", __func__, size_info.video_width,size_info.video_height);

			if(ce1702_cam_preview == PREVIEW_MODE_CAMCORDER)
			{
				retry_cnt = sizeof(ce1702_video_table) / sizeof(ce1702_video_table[0]);
				wdata[0] = ce1702_get_supported_size(size_info.video_width, size_info.video_height, ce1702_video_table, retry_cnt);
			}
			else
			{
				retry_cnt = sizeof(ce1702_preview_table) / sizeof(ce1702_preview_table[0]);
				wdata[0] = ce1702_get_supported_size(size_info.preview_width, size_info.preview_height, ce1702_preview_table, retry_cnt);
			}

			wdata[1] = 0x01;
			ce1702_i2c_write(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x54, wdata,  2);
			cnt = sizeof(ce1702_postview_table) / sizeof(ce1702_postview_table[0]);
/* LGE_CHANGE_S, fix zoom capture, 2012.11.21, gayoung85.lee[Start] */
			wdata[0] = ce1702_get_supported_size(size_info.picture_width, size_info.picture_height, ce1702_postview_table, cnt);

			wdata[1] = 0x05;
			ce1702_i2c_write(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x55, wdata,  2);

			LDBGI("%s: Capture Size:: picture width = %d , picture height = %d\n", __func__,size_info.picture_width,size_info.picture_height);

			cnt = sizeof(ce1702_picture_table) / sizeof(ce1702_picture_table[0]);
			wdata[0] = ce1702_get_supported_size(size_info.picture_width, size_info.picture_height, ce1702_picture_table, cnt);
			ce1702_size_info = wdata[0];
			wdata[1] = 0x00;
			wdata[2] = 0x00;
			wdata[3] = 0x81;
			LDBGI("%s: wdata[0] = %d\n", __func__,wdata[0]);
			ce1702_i2c_write(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x73, wdata,  4);

			wdata[0] = 0x21;
			wdata[1] = 0x00;
			ce1702_i2c_write(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x07, wdata,  2);

			ce1702_set_zoom_ratio(s_ctrl, ce1702_zoom_ratio);

/* LGE_CHANGE_E, fix zoom capture, 2012.11.21, gayoung85.lee[End] */

			wdata[0] = 0x14;
			wdata[1] = 0x01;
			ce1702_i2c_write(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x05, wdata,  2);

			wdata[0] = 0x03;
			ce1702_i2c_write(ce1702_s_interface_ctrl->sensor_i2c_addr, 0xAE, wdata,  1);

			wdata[0] = 0x00;
			ce1702_i2c_write(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x11, wdata,  1);
		}

		wdata[0] = 0x01;
		ce1702_i2c_write(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x6B, wdata,  1);
		if (rc < 0)
			LDBGE("%s: ce1702_i2c_write failed \n", __func__);

		retry_cnt = 0;
		do
		{
			mdelay(10);
			rdata[0] = 0x00;
			ce1702_i2c_read(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x6C, rdata, 0,  &result,  1);
			if (rc < 0)
				LDBGE("%s: ce1702_i2c_read failed \n", __func__);
			retry_cnt++;
		} while ((result != 0x08) && (retry_cnt < 100));

		if(result == 0x08){
			isPreviewMode = 1;
			isSingleCaptureMode = 0;
			isTMSMode = 0;
			isBurstMode = 0;
		}else{
			isPreviewMode = 0;
		}

		LDBGI("%s: ce1702_i2c_read successed, CE1702_MODE_PREVIEW result=%d, isPreviewMode=%d \n", __func__, result, isPreviewMode);
	}
	else if(ce1702_frame_mode == CE1702_MODE_SINGLE_SHOT && isSingleCaptureMode == 0){
		LDBGE("%s: CE1702_MODE_SINGLE_SHOT start\n", __func__);

#if 1 // Mode 1: Single Shot
		//set compression rate
		wdata[0] = 0x00;
		wdata[1] = 0x80; //1207 yt.jeon@lge.com increase compression rate;
		wdata[2] = 0x0C; //0xC80(3200)
		wdata[3] = 0xB0; //0x4B0(1200)
		wdata[4] = 0x04;
		wdata[5] = 0x01;
		wdata[6] = 0x21; //0x20
		ce1702_i2c_write(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x90, wdata,  7);

		/* LGE_CHANGE_S, For Snapshot Flash, 2012.10.22, jungki.kim[Start] */
		ce1702_sensor_set_led_flash_mode_snapshot(ce1702_led_flash_mode);
		/* LGE_CHANGE_E, For Snapshot Flash, 2012.10.22, jungki.kim[End] */

		wdata[0] = 0x11;	//aewb lock
		ce1702_i2c_write(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x11, wdata,  1);

		ce1702_set_exif_rotation_to_isp();

		ce1702_i2c_read(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x24, NULL, 0, &result,  1);
		LDBGI("%s: Lens Status: [0x%02x] \n", __func__, result);

		wdata[0] = 0x00;	//Buffering Capture
		ce1702_i2c_write(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x74, wdata,  1);

		retry_cnt = 0;
		do {
			mdelay(10);
			rdata[0] = 0x00;
			ce1702_i2c_read(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x6C, rdata, 1,  &result,  1);
			retry_cnt++;
		} while ((result != 0x01) && (retry_cnt < 100));

		if(retry_cnt >= 100) {
			LDBGE("%s: %d: error to capture with mode 1\n", __func__, __LINE__);
			return;
		}

		if(result == 0x01){
			isPreviewMode = 0;
			isSingleCaptureMode = 1;
			isTMSMode = 0;
			isBurstMode = 0;
		}else{
			isSingleCaptureMode = 0;
		}
#endif

		//mdelay(40); //yt.jeon 1114 optimize delay time

		LDBGI("%s: ce1702_i2c_read successed, CE1702_MODE_SINGLE_SHOT result=%d, isSingleCaptureMode=%d \n", __func__, result, isSingleCaptureMode);
	}
	else if(ce1702_frame_mode == CE1702_MODE_TMS_SHOT && isTMSMode ==0){
		LDBGI("%s: CE1702_MODE_TMS_SHOT start\n", __func__);
        // LGE_CHANGE_S, rotation fixed, 2012.10.24 elin.lee@lge.com
        wdata[0] = 0x03;
        ce1702_i2c_write(ce1702_s_interface_ctrl->sensor_i2c_addr, 0xAE, wdata,  1);
        // LGE_CHANGE_E, rotation fixed, 2012.10.24 elin.lee@lge.com

        cnt = sizeof(ce1702_postview_table) / sizeof(ce1702_postview_table[0]);
    /* LGE_CHANGE_S, fix zoom capture, 2012.11.21, gayoung85.lee[Start] */
        wdata[0] = ce1702_get_supported_size(size_info.picture_width, size_info.picture_height, ce1702_postview_table, cnt);
       cnt = sizeof(ce1702_picture_table) / sizeof(ce1702_picture_table[0]);
    /* LGE_CHANGE_S, fix zoom capture, 2012.11.21, gayoung85.lee[Start] */
        wdata[1] = ce1702_get_supported_size(size_info.picture_width, size_info.picture_height, ce1702_picture_table, cnt);
        wdata[2] = 0x00;
        wdata[3] = 0x00;
        wdata[4] = 0x00;
        wdata[5] = 0x01;
        wdata[6] = 0x00;

        ce1702_i2c_write(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x62, wdata,  7);

        LDBGI("%s: TMC Size:: %d\n", __func__, wdata[0]);

        // Multi-Image output setting
        wdata[0] = 0x00;
	wdata[1] = 0x80; //1207 yt.jeon@lge.com increase compression rate. 0x60
	wdata[2] = 0x0C; //0x09;
	wdata[3] = 0xB0;//0x4B0(1200), 0xD0
	wdata[4] = 0x04; //0x07
	wdata[5] = 0x01;
	wdata[6] = 0x21; //0x20

        ce1702_i2c_write(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x90, wdata,  7);

        wdata[0] = 0x01;
		ce1702_i2c_write(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x63, wdata,  1);

		if (rc < 0)
			LDBGE("%s: ce1702_i2c_write failed \n", __func__);

		do
		{
			mdelay(10);
			rdata[0] = 0x00;
			ce1702_i2c_read(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x6C, rdata, 0,  &result,  1);
			if (rc < 0)
				LDBGE("%s: ce1702_i2c_read failed \n", __func__);
			retry_cnt++;
		} while ((result != 0x18) && (retry_cnt < 100));

		if(result == 0x18){
			isPreviewMode = 0;
			isSingleCaptureMode = 0;
			isTMSMode = 1;
			isBurstMode = 0;
		}else{
			isTMSMode = 0;
		}

		LDBGI("%s: ce1702_i2c_read successed, CE1702_MODE_TMS_SHOT result=%d, isTMSMode=%d \n", __func__, result, isTMSMode);
	}
	else if(ce1702_frame_mode ==  CE1702_MODE_BURST_SHOT && isBurstMode ==0){
		LDBGI("%s: CE1702_MODE_BURST_SHOT start\n", __func__);

    // LGE_CHANGE_S, rotation fixed, 2012.10.24 elin.lee@lge.com
          wdata[0] = 0x03;
          ce1702_i2c_write(ce1702_s_interface_ctrl->sensor_i2c_addr, 0xAE, wdata,  1);
    // LGE_CHANGE_E, rotation fixed, 2012.10.24 elin.lee@lge.com

          cnt = sizeof(ce1702_postview_table) / sizeof(ce1702_postview_table[0]);
        /* LGE_CHANGE_S, fix zoom capture, 2012.11.21, gayoung85.lee[Start] */
          wdata[0] = ce1702_get_supported_size(size_info.picture_width, size_info.picture_height, ce1702_postview_table, cnt);
           cnt = sizeof(ce1702_picture_table) / sizeof(ce1702_picture_table[0]);
        /* LGE_CHANGE_S, fix zoom capture, 2012.11.21, gayoung85.lee[Start] */
          wdata[1] = ce1702_get_supported_size(size_info.picture_width, size_info.picture_height, ce1702_picture_table, cnt);
          wdata[2] = 0x00;
          wdata[3] = 0x00;
          wdata[4] = 0x00;
          wdata[5] = 0x01;
          wdata[6] = 0x00;

          ce1702_i2c_write(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x62, wdata,  7);

          LDBGI("%s: Burst Shot Size:: picture width = %d , picture height = %d\n", __func__,size_info.picture_width,size_info.picture_height);

          // Multi-Image output setting
        wdata[0] = 0x00;
	wdata[1] = 0x80; //1207 yt.jeon@lge.com increase compression rate. 0x60
	wdata[2] = 0x0C; //0x09;
	wdata[3] = 0xB0;//0x4B0(1200), 0xD0
	wdata[4] = 0x04; //0x07
	wdata[5] = 0x01;
	wdata[6] = 0x21; //0x20

          ce1702_i2c_write(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x90, wdata,  7);


	wdata[0] = 0x01;
	ce1702_i2c_write(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x63, wdata,  1);

	if (rc < 0)
		LDBGE("%s: ce1702_i2c_write failed \n", __func__);

	do
	{
		mdelay(10);
		rdata[0] = 0x00;
		ce1702_i2c_read(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x6C, rdata, 0,  &result,  1);
		if (rc < 0)
			LDBGE("%s: ce1702_i2c_read failed \n", __func__);
		retry_cnt++;
	} while ((result != 0x18) && (retry_cnt < 100));

	if(result == 0x18){
		isPreviewMode = 0;
		isSingleCaptureMode = 0;
		isTMSMode = 0;
		isBurstMode = 1;
	}else{
		isBurstMode = 0;
	}
		LDBGI("%s: ce1702_i2c_read successed, CE1702_MODE_BURST_SHOT result=%d, isBurstMode=%d \n", __func__, result, isBurstMode);
	}
	/*
		Comment by jungki.kim
		We need to excute these function lately because these have to try under previewing condition.
	*/
	ce1702_sensor_set_param_lately(s_ctrl);

}

void ce1702_sensor_stop_stream(struct msm_sensor_ctrl_t *s_ctrl)
{
	uint8_t result =0;
	int retry_cnt = 0;
	uint8_t rdata[2];
	uint8_t wdata[4];
	int32_t rc = 0;

	LDBGE("%s: isPreviewMode=%d,  isSingleCaptureMode =%d, isTMSMode=%d, isBurstMode =%d\n", __func__, isPreviewMode, isSingleCaptureMode, isTMSMode, isBurstMode);

	if(ce1702_frame_mode == CE1702_MODE_PREVIEW && isPreviewMode == 1){
		LDBGI("%s: CE1702_MODE_PREVIEW stop\n", __func__);

#if 0
		 if((ce1702_focus_mode == AF_MODE_CAF_VIDEO || ce1702_focus_mode == AF_MODE_CAF_PICTURE)) {
#else
		ce1702_i2c_read(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x24, NULL, 0, &result,  1);
		if ((result & 0x01) == 0x01) {
#endif
			 wdata[0] = 0x00;
			 ce1702_i2c_write(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x35, wdata,  1);	 // Stop VCM
			 mdelay(33); //yt.jeon 1115 optimize delay time
			 retry_cnt = 0;
			 do {
				 //rdata[0] = 0x00;
				 ce1702_i2c_read(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x24, NULL, 0, &result,  1);
				 if ((result & 0x01) == 0){
					 LDBGI("%s: stop lens ce1702_i2c_read successed, result=%d\n", __func__, result);
					 break;
				 }
				 retry_cnt++;
				 mdelay(5); //yt.jeon 1115 optimize delay time
			 } while (retry_cnt < 100);	 //lens running
		 }

	 	wdata[0] = 0x00;
		ce1702_i2c_write(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x6B, wdata,  1);
		if (rc < 0)
			LDBGE("%s: ce1702_i2c_write failed \n", __func__);

		retry_cnt = 0;
		do
		{
			//rdata[0] = 0x00;
			rc = ce1702_i2c_read(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x6C, rdata, 0,  &result,  1);
			if (rc < 0)
				LDBGE("%s: ce1702_i2c_read failed \n", __func__);
			if (result == 0x00)
				break;

			mdelay(10);
			retry_cnt++;
			} while (retry_cnt < 100);

		if(result == 0x00){
			isPreviewMode = 0;
		}

		LDBGI("%s: ce1702_i2c_read successed, result=%d, isPreviewMode=%d \n", __func__, result, isPreviewMode);
	}
	if(ce1702_frame_mode == CE1702_MODE_SINGLE_SHOT && isSingleCaptureMode == 1) {


#if 0 // Reduce s2s by yt.jeon 1205
		LDBGE("%s: CE1702_MODE_SINGLE_SHOT stop \n", __func__);
		wdata[0] = 0x00;
		ce1702_i2c_write(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x75, wdata,  1);
		retry_cnt = 0;
		do
		{
			//rdata[0] = 0x00;
			rc = ce1702_i2c_read(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x6C, rdata, 0,  &result,  1);
			if (rc < 0)
				LDBGE("%s: ce1702_i2c_read failed \n", __func__);
			if (result == 0x00)
				break;

			mdelay(10);
			retry_cnt++;
		} while (retry_cnt < 100);

		if(result == 0x00){
			isSingleCaptureMode  = 0;
		}
		LDBGE("%s: ce1702_i2c_read successed, result=%d, isSingleCaptureMode =%d \n", __func__, result, isSingleCaptureMode );
		isSingleCaptureMode  = 0;
#endif
	}
	else if(ce1702_frame_mode == CE1702_MODE_TMS_SHOT && isTMSMode ==1){

		ce1702_i2c_read(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x24, NULL, 0, &result,  1);
		if ((result & 0x01) == 0x01) {
			 wdata[0] = 0x00;
			 ce1702_i2c_write(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x35, wdata,  1);	 // Stop VCM
			 mdelay(33); //yt.jeon 1115 optimize delay time
			 retry_cnt = 0;
			 do {
				 //rdata[0] = 0x00;
				 ce1702_i2c_read(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x24, NULL, 0, &result,  1);
				 if ((result & 0x01) == 0){
					 LDBGI("%s: stop lens ce1702_i2c_read successed, result=%d\n", __func__, result);
					 break;
				 }
				 retry_cnt++;
				 mdelay(5); //yt.jeon 1115 optimize delay time
			 } while (retry_cnt < 100);	 //lens running
		 }

		wdata[0] = 0x00;
		rc = ce1702_i2c_write(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x63, wdata,  1);
		if (rc < 0)
			LDBGE("%s: ce1702_i2c_write failed \n", __func__);
		retry_cnt = 0;
		do
		{
			//rdata[0] = 0x00;
			rc = ce1702_i2c_read(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x6C, rdata, 0,	&result,  1);
			if (rc < 0)
				LDBGE("%s: ce1702_i2c_read failed \n", __func__);
			if (result == 0x00)
				break;

			mdelay(10);
			retry_cnt++;
		} while (retry_cnt < 100);

		if(result == 0x00){
			isTMSMode= 0;
		}
		LDBGE("%s: ce1702_i2c_read successed, result=%d, isTMSMode =%d \n", __func__, result, isTMSMode );
	}

	else if(ce1702_frame_mode == CE1702_MODE_BURST_SHOT && isBurstMode ==1){

		ce1702_i2c_read(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x24, NULL, 0, &result,  1);
		if ((result & 0x01) == 0x01) {
			 wdata[0] = 0x00;
			 ce1702_i2c_write(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x35, wdata,  1);	 // Stop VCM
			 mdelay(33); //yt.jeon 1115 optimize delay time
			 retry_cnt = 0;
			 do {
				 //rdata[0] = 0x00;
				 ce1702_i2c_read(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x24, NULL, 0, &result,  1);
				 if ((result & 0x01) == 0){
					 LDBGI("%s: stop lens ce1702_i2c_read successed, result=%d\n", __func__, result);
					 break;
				 }
				 retry_cnt++;
				 mdelay(5); //yt.jeon 1115 optimize delay time
			 } while (retry_cnt < 100);	 //lens running
		 }

		wdata[0] = 0x00;
		rc = ce1702_i2c_write(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x75, wdata,  1);
		if (rc < 0)
			LDBGE("%s: ce1702_i2c_write failed \n", __func__);
		retry_cnt = 0;
		do
		{
			//rdata[0] = 0x00;
			rc = ce1702_i2c_read(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x6C, rdata, 0,	&result,  1);
			if (rc < 0)
				LDBGE("%s: ce1702_i2c_read failed \n", __func__);
			if (result == 0x00)
				break;

			mdelay(10);
			retry_cnt++;
		} while (retry_cnt < 100);

		if(result == 0x00){
			isBurstMode= 0;
		}
		LDBGE("%s: ce1702_i2c_read successed, result=%d, isBurstMode =%d \n", __func__, result, isBurstMode );
	}
	/*
		TODO: Need to know which preview mode is
		because 'Preview Assist Mode' command have to be excuted before previewing...
		Comment by jungki.kim

		Temporarily set to preview mode.
	*/
	ce1702_set_preview_assist_mode(s_ctrl, PREVIEW_MODE_CAMERA);

}

void ce1702_set_preview_assist_mode(struct msm_sensor_ctrl_t *s_ctrl, uint8_t mode)
{
	unsigned char data[10];
	LDBGI("%s: mode = %d\n", __func__, mode);

	switch(mode) {
		case PREVIEW_MODE_CAMERA:
			data[0] = 0x00;
			break;

		case PREVIEW_MODE_CAMCORDER:
			data[0] = 0x01;
			break;

		default:
			data[0] = 0x00;
			LDBGE("%s: Error. No preview mode was set! [%d]\n", __func__, mode);
			break;
	}
	ce1702_i2c_write(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x40, data, 1);
}

// Color Effect by Jungki.kim@lge.com
// CMD : 0x3D
// DATA : 0x05
int32_t ce1702_set_special_effect(struct msm_sensor_ctrl_t *s_ctrl, uint8_t effect)
{
	int32_t rc = TRUE;
	unsigned char data[10];
	uint8_t rdata[2];
	uint8_t res;
	int cnt;
	LDBGI("%s: effect=%d\n", __func__, effect);

	data[0] = 0x05; // effect

	switch(effect) {
		case CAMERA_EFFECT_OFF :
			data[1] = 0x00;
			break;
		case CAMERA_EFFECT_MONO :
			data[1] = 0x01;
			break;
		case CAMERA_EFFECT_NEGATIVE :
			data[1] = 0x02;
			break;
		case CAMERA_EFFECT_SEPIA :
			data[1] = 0x03;
			break;
		default :
			data[1] = 0x00;
			break;
	}
	rc = ce1702_i2c_write(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x3D, data, 2);
	if(rc < 0) {
		LDBGE("%s: Fail to apply effect [%d]!!\n", __func__, effect);
		return FALSE;
	}
	data[0] = 0x00;
	rc = ce1702_i2c_write(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x01, data, 0); // Apply!!

	cnt = 0;
	do {
		mdelay(5); //yt.jeon 1115 optimize delay time
		rdata[0] = 0x00;
		rc = ce1702_i2c_read(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x02, rdata, 0, &res, 1);
		LDBGI("%s: verifying... cnt=%d rc=%d\n", __func__, cnt, rc);
		cnt++;
	} while ( (res != 0) && (cnt < 100) );

	if(cnt >= 100) {
		LDBGE("%s: Fail to read isp status \n", __func__);
		rc = - EIO;
	}

	LDBGI("%s: SUCCESS! rc=%d\n", __func__, rc);
	return rc;
}

// Brightness by Jungki.kim@lge.com
// CMD : 0x04
// DATA : 0x02
//To-Do : very little change. Need to improve!
int32_t ce1702_set_exposure_compensation(struct msm_sensor_ctrl_t *s_ctrl, uint8_t exposure)
{

	int32_t rc = 0;
	unsigned char data[10];
	uint8_t rdata[2];
	uint8_t res;
	int cnt;

	int8_t value = (int8_t)exposure;
	int8_t adj = (int8_t)(value/2)+6;

	LDBGI("%s: exposure = %d adj=[%d] rc=%d\n", __func__, value, adj, rc);

	if(ce1702_brightness == adj) {
		LDBGI("%s: just before value[%d] = requested value[%d] (SAME!)\n", __func__, ce1702_brightness, adj);
		return rc;
	}

	data[0] = 0x02; 	// brightness??
	data[1] = adj;	// value

	rc = ce1702_i2c_write(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x04, data, 2);
	if(rc < 0) {
		LDBGE("%s: Fail to apply brightness [%d]!!\n", __func__,adj);
		return FALSE;
	}
	data[0] = 0x00;
	rc = ce1702_i2c_write(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x01, data, 0); // Apply!!

	cnt = 0;
	do {
		mdelay(5); //yt.jeon 1115 optimize delay time
		rdata[0] = 0x00;
		rc = ce1702_i2c_read(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x02, rdata, 0, &res, 1);
		LDBGI("%s: verifying... cnt=%d rc=%d\n", __func__, cnt, rc);
		cnt++;
	} while ( (res != 0) && (cnt < 100) );

	if(cnt >= 100) {
		LDBGE("%s: Fail to read isp status \n", __func__);
		rc = - EIO;
	}
	ce1702_brightness = adj;

	return rc;
}

// CAF Setting for CE1702 by jungki.kim
int8_t ce1702_set_caf(int mode)
{
	int8_t rc = 0;
	unsigned char data[10];
	uint8_t rdata[2];
	uint8_t res=0;
	int cnt;

	LDBGI("%s: Enter with [%d]\n",__func__, mode);

	switch(mode) {
		case AF_MODE_CAF_VIDEO:
			LDBGI("%s: start [ AF_MODE_CAF_VIDEO ]\n", __func__);

			// 1. Setting AF mode (CMD: 0x20)
			data[0] = 0x02; // CAF
			rc = ce1702_i2c_write(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x20, data, 1);
			LDBGI("%s: %d: check rc = [%d]\n",__func__, __LINE__, rc);

			// 2. Check AF Searching Status (CMD:0x24)
			cnt=0;
			do {
				mdelay(2); //yt.jeon 1115 optimize delay time
				rdata[0] = 0x00; //NULL
				rc = ce1702_i2c_read(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x24, rdata, 0, &res, 1);
				LDBGI("%s: %d: Cont-AF state check[count %d]  = %x \n", __func__, __LINE__, cnt, res);
				cnt++;
			} while ((res != 0x00) && (res != 0x02) && (res != 0x04) && (cnt < 500));

			// 3. Start CAF(CMD:0x23)
			data[0] = 0x00; //NULL
			rc = ce1702_i2c_write(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x23, data, 0);
			mdelay(33); //yt.jeon 1115 optimize delay time

			// 4. Check AF Searching Status (CMD:0x24)
			cnt=0;
			do {
				//mdelay(33); //yt.jeon 1115 optimize delay time
				rdata[0] = 0x00; //NULL
				rc = ce1702_i2c_read(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x24, rdata, 0, &res, 1);
				LDBGI("%s: %d: Cont-AF state check[count %d]  = %x \n", __func__, __LINE__, cnt, res);
				res = res & 0xC0;
				if( (res == 0x80) ||(res == 0x40) ) break;
				cnt++;
				mdelay(5); //yt.jeon 1115 optimize delay time
			} while (cnt < 10);

			break;

		case AF_MODE_CAF_PICTURE:
			LDBGI("%s: start [ AF_MODE_CAF_PICTURE ]\n", __func__);

			data[0] = 0x01;
			rc = ce1702_i2c_write(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x2C, data, 1);
			cnt = 0;
			do {
				data[0] = 0x00;
				rc = ce1702_i2c_read(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x2D, data, 0, &res,  1);
				if (res  == 0x01) {
					break;
				}
				mdelay(10);
				cnt++;
			} while (cnt < 100);

			break;

		default:
			break;
	}

	return rc;
}

//Start AF for CE1702 by jungki.kim
int8_t ce1702_start_af(struct msm_sensor_ctrl_t *s_ctrl)
{
	int8_t rc = 0;
	int cnt = 0;
	uint8_t rdata[10], res=0;
	uint8_t data=0;

	if(!ce1702_is_doing_touchaf) {
		LDBGI("%s: Start [AF]\n", __func__);
		rc = ce1702_i2c_write(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x23, NULL, 0);
		cnt = 0;
		do {
			mdelay(10);
			rdata[0] = 0x00;
			rc = ce1702_i2c_read(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x24, rdata, 0, &res,  1);
			//LDBGI("%s: stop check [count %d]	= %x \n", __func__, cnt, res);
			cnt++;
		} while ((cnt < 100) && (res == 0x01)); 	//lens running
	} else {
		LDBGI("%s: Start [TOUCH AF]\n", __func__);
		data = 0x05; // Area ON!
		rc = ce1702_i2c_write(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x42, &data, 1); // control preview assist
		cnt = 0;
		do {
			rdata[0] = 0x00;
			rc = ce1702_i2c_read(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x43, rdata, 0, &res,  1);
			if(res == 0x05) break;	// check sync area
			mdelay(5); //yt.jeon 1115 optimize delay time
			cnt++;
		} while (cnt < 100);
	}
	LDBGI("%s: res = [0x%02x] cnt=[%d]\n", __func__, res, cnt);

	return rc;
}

//Stop AF for CE1702 by jungki.kim
int8_t ce1702_stop_af(struct msm_sensor_ctrl_t *s_ctrl)
{
	int8_t rc = 0;
	uint8_t data;
	uint8_t rdata = 0;
	int cnt;
	int16_t window[4] = {0};

	if( ce1702_is_doing_touchaf == true) {
		LDBGI("%s: Stop [TOUCH AF]\n", __func__);
		rc = ce1702_set_window(s_ctrl, window, SET_AREA_AF_OFF);
		if(rc < 0)
			LDBGE("%s: AF Area does NOT set... [ SET_AREA_AF_OFF ]\n", __func__);
	} else {
		LDBGI("%s: Stop [AF]\n", __func__);
		ce1702_i2c_write(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x35, &data, 0); // stop
		cnt = 0;
		do {
			data = 0x00;
			rc = ce1702_i2c_read(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x24, &data, 0, &rdata, 1);
			if ((rdata & 0x01) == 0)
				break;
			mdelay(10);
			LDBGI("%s: stop check [count %d]	= %x \n", __func__, cnt, rdata);
			cnt++;
		} while (cnt < 100);	//lens running
	}
	ce1702_is_doing_touchaf = false;

	return rc;
}

uint8_t ce1702_check_af_status(bool ispolling)
{
	uint8_t rdata = 0;
	uint8_t data;
	int cnt;

	switch(ispolling) {
		case TRUE:
			cnt = 0;
			LDBGI("%s: Polling Start\n", __func__);
			do {
				data = 0x00;
				ce1702_i2c_read(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x24, &data, 0, &rdata,  1);
				if ((rdata & 0x01) == 0x00)
					break;
				mdelay(10);
				cnt++;
			} while (cnt < 100); 	//lens running
			LDBGI("%s: Polling End Count=[%d]\n", __func__, cnt);
			return TRUE;
			break;

		case FALSE:
			data = 0x00;
			ce1702_i2c_read(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x24, &data, 0, &rdata,  1);
			LDBGI("%s: stop check [single count] = 0x%02x \n", __func__, rdata);
			return rdata;
			break;
	}
	return 0;
}

int8_t ce1702_set_window(struct msm_sensor_ctrl_t *s_ctrl, int16_t *window, int16_t sw)
{
	int8_t rc = 0;
	unsigned char data[10];
	uint8_t rdata[2];
	uint8_t res=0;
	int cnt;

	LDBGI("%s: x=[%d] y=[%d] rx=[%d] ry=[%d] MODE=[%d]\n", __func__, window[0], window[1], window[2], window[3], sw);

	// Check if it is previewing or not.
	rdata[0] = 0x00; //NULL
	rc = ce1702_i2c_read(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x6C, rdata, 0, &res, 1);
	if( (res != 0x08) &&(res != 0x18) ) {
		LDBGI("%s: Sensor is not previewing... value=[%x] return\n", __func__, res);
		return -1;
	}

	if((ce1702_focus_mode != AF_MODE_CAF_VIDEO) || (sw >= SET_AREA_AF_OFF) ) {
		switch(sw) {
			case SET_AREA_AF:
				LDBGI("%s: Mode Set= SET_AREA_AF\n",__func__);
				data[0] = 0x20;	// AF Area
				data[1] = 0xB2; 	// Enable Area
				break;
			case SET_AREA_AE:
				LDBGI("%s: Mode Set= SET_AREA_AE\n",__func__);
				data[0] = 0x10;	// AE Area
				data[1] = 0xB2; 	// Enable Area
				break;
			case SET_AREA_AF_OFF:
				LDBGI("%s: Mode Set= SET_AREA_AF_OFF\n",__func__);
				data[0] = 0x20;	//AF Area
				data[1] = 0x80;	//Disable Area
				break;
			case SET_AREA_AE_OFF:
				LDBGI("%s: Mode Set= SET_AREA_AE_OFF\n",__func__);
				data[0] = 0x10;	//AE Area
				data[1] = 0x80;	//Disable Area
				break;
			default:
				LDBGI("%s: Invalid parameter [%d] return\n",__func__, sw);
				return -1;
				break;
		}

		data[2] = window[0] & 0xFF; // x lower
		data[3] = window[0] >>0x08; // x upper
		data[4] = window[1] & 0xFF; // y lower
		data[5] = window[1] >>0x08; // y upper
		data[6] = window[2] & 0xFF; // x lower
		data[7] = window[2] >>0x08; // x upper
		data[8] = window[3] & 0xFF; // y lower
		data[9] = window[3] >>0x08; // y upper
		rc = ce1702_i2c_write(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x41, data, 10); // preview assist

		if( sw == SET_AREA_AF) {
			// Set Area for Exposure
			data[0] = 0x10;
			data[1] = 0xB2;
			rc = ce1702_i2c_write(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x41, data, 10); // preview assist
		}
/* LGE_CHANGE_S, Move To Start AF Function, 2012.12.4, jungki.kim[Start] */
#if 0
		data[0] = 0x05; // Area ON!
		rc = ce1702_i2c_write(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x42, data, 1); // control preview assist

		cnt = 0;
		do {
			rdata[0] = 0x00;
			rc = ce1702_i2c_read(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x43, rdata, 0, &res,  1);
			mdelay(5); //yt.jeon 1115 optimize delay time
			if(res == 0x05) break;	// check sync area
			//LDBGI("%s: check preview assist cmd [count %d] = 0x%02X \n", __func__, cnt, res);
			cnt++;
		} while (cnt < 100);
#endif
/* LGE_CHANGE_E, Move To Start AF Function, 2012.12.4, jungki.kim[End] */
		if(sw >= SET_AREA_AF_OFF) {
			data[0] = 0x00; // OFF
			rc = ce1702_i2c_write(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x42, data, 1); // control preview assist
			cnt = 0;
			do {
				mdelay(5); //yt.jeon 1115 optimize delay time
				rdata[0] = 0x00;
				rc = ce1702_i2c_read(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x43, rdata, 0, &res,  1);
				//LDBGI("%s: check preview assist cmd [count %d] = 0x%02X \n", __func__, cnt, res);
				cnt++;
				if(res == 0x00) break;	// check
			} while (cnt < 100);
		}
	}

	LDBGI("%s: X\n", __func__);
	return rc;
}

//Set AE Window for CE1702 by jungki.kim
int8_t ce1702_set_ae_window(struct msm_sensor_ctrl_t *s_ctrl, int16_t *window)
{
	int8_t rc = 0;

	//LDBGI("%s: LT[%d,%d] RB[%d,%d]\n", __func__, window[0], window[1], window[2], window[3]);

	//rc = ce1702_set_window(s_ctrl, window, SET_AREA_AE);
	if(rc < 0) {
		LDBGE("%s: AE Area does NOT set...\n", __func__);
	}

	return rc;
}

//Set AF Window for CE1702 by jungki.kim
int8_t ce1702_set_af_window(struct msm_sensor_ctrl_t *s_ctrl, int16_t *window)
{
	int8_t rc = 0;

	//LDBGI("%s: LT[%d,%d] RB[%d,%d]\n", __func__, window[0], window[1], window[2], window[3]);

	ce1702_is_doing_touchaf = true;
	rc = ce1702_set_window(s_ctrl, window, SET_AREA_AF);
	if(rc < 0) {
		LDBGE("%s: AF Area does NOT set... [ SET_AREA_AF ]\n", __func__);
	}

#if 0
	rc = ce1702_set_window(s_ctrl, window, SET_AREA_AF_OFF);
	if(rc < 0) {
		LDBGE("%s: AF Area does NOT set... [ SET_AREA_AF_OFF ]\n", __func__);
	}
#endif

	return rc;
}

#if 0
int8_t ce1702_set_af_mode_auto(void)
{
	int8_t rc = 0;
	unsigned char data[10];
	uint8_t rdata[2];
	uint8_t res;
	int cnt;

	data[0] = 0x00;	// Normal
	rc = ce1702_i2c_write(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x20, data, 1);
	cnt = 0;
	res = 1;
	do {
		mdelay(2); //yt.jeon 1115 optimize delay time
		rdata[0] = 0x00;	// NULL
		rc = ce1702_i2c_read(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x24, rdata, 0, &res,  1);
		//LDBGI("%s: Auto mode setting state check[%d ms]  = %x\n", __func__, cnt*25, res);
		cnt++;
	} while ((res & 0x01) && (cnt < 80));

	return rc;
}
#endif

//AF Mode Settings for CE1702 by jungki.kim
int8_t ce1702_set_focus_mode_setting(struct msm_sensor_ctrl_t *s_ctrl, int32_t afmode)
{
	int8_t rc = 0;
	uint8_t data;

	LDBGI("%s: afmode: [%d] \n", __func__, afmode);

	switch(afmode) {
		case AF_MODE_MACRO:
			ce1702_stop_af(s_ctrl);
			data = 0x01;
			ce1702_i2c_write(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x20, &data, 1);
			ce1702_check_af_status(FALSE);
			break;
		case AF_MODE_NORMAL: // Manual Focus
			ce1702_stop_af(s_ctrl);
			//ce1702_set_manual_focus_length(s_ctrl, 0);
			break;
#if 1
		case AF_MODE_AUTO:
			if(ce1702_focus_mode == AF_MODE_CAF_VIDEO) {
				ce1702_stop_af(s_ctrl);
			}
			if( ce1702_is_doing_touchaf == false) {
				data = 0x00;
				ce1702_i2c_write(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x20, &data, 1);
				ce1702_check_af_status(FALSE);
			}
			break;
		case AF_MODE_CAF_VIDEO:
		case AF_MODE_CAF_PICTURE:
			if(ce1702_focus_mode != AF_MODE_AUTO) {
				LDBGI("%s: Before Mode was NOT [ AF_MODE_AUTO ]\n", __func__);
				ce1702_stop_af(s_ctrl);
				data = 0x00;
				ce1702_i2c_write(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x20, &data, 1);
				ce1702_check_af_status(FALSE);
				ce1702_set_caf(afmode);
			}
			break;
#else	// Should be resolved!
		case AF_MODE_AUTO:
			ce1702_stop_af(s_ctrl);
			data = 0x00;
			ce1702_i2c_write(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x20, &data, 1);
			ce1702_check_af_status(FALSE);
			break;
		case AF_MODE_CAF_VIDEO:
		case AF_MODE_CAF_PICTURE:
			ce1702_stop_af(s_ctrl);
			//data = 0x00;
			//ce1702_i2c_write(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x20, &data, 1);
			//ce1702_check_af_status(FALSE);
			ce1702_set_caf(afmode);
			break;
#endif
		case AF_MODE_INFINITY:
		default:
			ce1702_stop_af(s_ctrl);
			data = 0x00;
			ce1702_i2c_write(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x20, &data, 1);
			ce1702_check_af_status(FALSE);
			break;
	}

	ce1702_focus_mode = afmode;

	return rc;

}

//White Balance Settings for CE1702 by jungki.kim
int8_t ce1702_set_wb_setting(struct msm_sensor_ctrl_t *s_ctrl, uint8_t wb)
{
	int8_t rc = 0;
	unsigned char data[10];
	uint8_t rdata[2];
	uint8_t res, wb_mode;
	int cnt;

	LDBGI("%s: cee1702_wb_mode:[%d] wb: [%d] \n", __func__, ce1702_wb_mode, wb);
	if(ce1702_wb_mode == wb) {
		LDBGE("%s: just before value[%d] = requested value[%d] (SAME!)\n", __func__, ce1702_wb_mode, wb);
		return rc;
	}

	switch (wb) {
		case CAMERA_WB_AUTO:
			LDBGI("%s: setting CAMERA_WB_AUTO\n", __func__);
			wb_mode = 0x00;
			data[0] = 0x11;
			data[1] = 0x00;
			break;
		case CAMERA_WB_INCANDESCENT:
			LDBGI("%s: setting CAMERA_WB_INCANDESCENT\n", __func__);
			wb_mode = 0x01;
			data[0] = 0x10;
			data[1] = 0x04;
			break;
		case CAMERA_WB_DAYLIGHT:
			LDBGI("%s: setting CAMERA_WB_DAYLIGHT\n", __func__);
			wb_mode = 0x01;
			data[0] = 0x10;
			data[1] = 0x01;
			break;
		case CAMERA_WB_FLUORESCENT:
			LDBGI("%s: setting CAMERA_WB_FLUORESCENT\n", __func__);
			wb_mode = 0x01;
			data[0] = 0x10;
			data[1] = 0x03;
			break;
		case CAMERA_WB_CLOUDY_DAYLIGHT:
			LDBGI("%s: setting CAMERA_WB_CLOUDY_DAYLIGHT\n", __func__);
			wb_mode = 0x01;
			data[0] = 0x10;
			data[1] = 0x02;
			break;
		case CAMERA_WB_OFF:
			LDBGI("%s: setting CAMERA_WB_OFF (means WB auto!?)\n", __func__);
			wb_mode = 0x00;
			data[0] = 0x11;
			data[1] = 0x00;
			break;
		default:
			LDBGI("%s: setting default\n", __func__);
			wb_mode = 0x00;
			data[0] = 0x11;
			data[1] = 0x00;
			break;
	}

	rc = ce1702_i2c_write(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x1A, &wb_mode, 1);	//WB Mode Setting
	rc = ce1702_i2c_write(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x04, data, 2);	//preset

	data[0]=0x00; //NULL
	rc = ce1702_i2c_write(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x01, data, 0);
	cnt = 0;
	do {
		mdelay(5); //yt.jeon 1115 optimize delay time
		rdata[0] = 0x00;
		rc = ce1702_i2c_read(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x02, rdata, 0, &res, 1);
		LDBGI("%s: WB setting check[%d] = %x \n", __func__, cnt, res);
		cnt++;
	} while((res != 0) && (cnt < 500));

	if (cnt >=500)
		rc = -EIO;

	ce1702_wb_mode = wb;
	return rc;
}

//Zoom Ratio Settings for CE1702 by jungki.kim
int8_t ce1702_set_zoom_ratio(struct msm_sensor_ctrl_t *s_ctrl, int32_t zoom)
{
	int8_t rc = 0;
	unsigned char data[10];
	uint8_t rdata[2] = {0, 0};
//	uint8_t res = 0;
	int cnt;
//	bool af_pause = false;
	if(isSingleCaptureMode == 1){
		return TRUE;
	}

	LDBGI("%s: zoom ratio=[%d] focus_mode=[%d]\n", __func__, zoom, ce1702_focus_mode);

#ifdef CAF_STOP_HERE
	af_pause = false;
	if(ce1702_focus_mode == AF_MODE_CAF_PICTURE) {
		data[0] = 0x00;
		rc = ce1702_i2c_read(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x2D, data, 0, &res, 1);
		if(res == 0x01) {
			data[0] = 0x02;
			rc = ce1702_i2c_write(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x2C, data, 1);	// pause
			cnt = 0;
			do {
				data[0] = 0x00;
				rc = ce1702_i2c_read(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x2D, data, 0, &res,  1);
				if (res  == 0x02) {
					LDBGI("%s: Pause AF-T\n", __func__);
					af_pause = true;
					break;
				}
				mdelay(10);
				cnt++;
			} while (cnt < 100);
		}
	}
#endif

	data[0] = zoom;
	rc = ce1702_i2c_write(ce1702_s_interface_ctrl->sensor_i2c_addr, 0xB9, data, 1);	// zoom ratio
	if(rc < 0) {
		LDBGE("%s: Fail to apply zoom [%d]!!\n", __func__, zoom);
		return FALSE;
	}

	cnt = 0;
	LDBGI("%s: [%d]: now polling START!\n", __func__, cnt);
	do {
		data[0] = 0x00;
		rc = ce1702_i2c_read(ce1702_s_interface_ctrl->sensor_i2c_addr, 0xBA, data, 0, rdata, 2);
		if (rdata[1] == 0)
			break;
		mdelay(5); // Do not reduce delay!
		//LDBGI("%s: verifying... cnt=%d rc=%d rdata[1]=[%d]\n", __func__, cnt, rc, rdata[1]);
		cnt++;
	} while ( cnt < 100 );
	LDBGI("%s: [%d]: now polling END!\n", __func__, cnt);

	if(cnt >= 100) {
		LDBGE("%s: Fail to read isp status \n", __func__);
		rc = - EIO;
	}

#ifdef CAF_STOP_HERE
	/* Restart AF-T on Snapshot*/
	if(af_pause) {
		data[0] = 0x01;
		rc = ce1702_i2c_write(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x2C, data, 1);	// restart
		cnt = 0;
		do {
			data[0] = 0x00;
			rc = ce1702_i2c_read(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x2D, data, 0, &res,  1);
			if (res  == 0x01) {
				LDBGI("%s: Restart AF-T\n", __func__);
				af_pause = false;
				break;
			}
			mdelay(10);
			cnt++;
		} while (cnt < 100);
		LDBGI("%s: [%d]: now polling END!\n", __func__, cnt);
	}
#endif
	ce1702_zoom_ratio = zoom;
	return rc;
}

/* LGE_CHANGE_S, Split Manual Focus To 60 Seteps, 2012.10.29, jungki.kim[Start] */
#if 0
//Support Manual Focus by jungki.kim@lge.com
static uint8_t MF_val[9][2] = {
	{0x00, 0x03},		// 0. macro	10cm
	{0x00, 0x02},		// 1. 14.9cm
	{0xc0, 0x01}, 	// 2. 28.1cm
	{0xa0, 0x01},		// 3. 37.3cm
	{0x80, 0x01},		// 4. 47.9cm
	{0x60, 0x01}, 	// 5. 66.9cm
	{0x40, 0x01},		// 6. 1.11M
	{0x20, 0x01},		// 7. 1.66M
	{0x0C, 0x00},		// 8. 3.32M
};
#endif
/* LGE_CHANGE_E, Split Manual Focus To 60 Seteps, 2012.10.29, jungki.kim[End] */

int8_t ce1702_set_manual_focus_length(struct msm_sensor_ctrl_t *s_ctrl, int32_t focus_val)
{
	int8_t rc = 0;
	unsigned char data[10];
	uint8_t rdata[2] = {0, 0};

	LDBGI("%s: manual focus value=%d\n", __func__, focus_val);
	// Check whether previewing or not
	data[0] = 0x00;
	rc = ce1702_i2c_read(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x6C, data, 0, rdata, 1);
	if( (rdata[0] == 0x08) ||(rdata[0] == 0x18) ) {
		// on preview
		focus_val = 610 - (10*focus_val);  //20121212, hyungmoo.huh@lge.com, Need to use EEPROM data for accuracy
		data[0] = 0x01;	// absolut position
		data[1] = focus_val & 0xFF; // x lower
		data[2] = focus_val >>0x08; // x upper
		mdelay(5);
		LDBGI("%s: CMD=0x%02X, 0x%02X, 0x%02X, 0x%02X\n", __func__, 0x33, data[0], data[1], data[2]);
		rc = ce1702_i2c_write(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x33, data, 3);
		ce1702_check_af_status(TRUE);
	} else {
		/* Not Preview Mode */
		LDBGI("%s: Skip to apply because of not previewing\n", __func__);
	}

	ce1702_manual_focus_val = focus_val;

	return rc;
}

/* LGE_CHANGE_S, Get Battery Temperature For Flash, 2012.12.7, jungki.kim[Start] */
int get_pm8921_batt_temp(void)
{
	int rc = 0;
	struct pm8xxx_adc_chan_result result;

	rc = pm8xxx_adc_read(CHANNEL_BATT_THERM, &result);
	if (rc) {
		LDBGE("%s: Error reading battery temperature!\n", __func__);
		return rc;
	}
	rc = (int)((int)result.physical / 10);
	return rc;
}
/* LGE_CHANGE_E, Get Battery Temperature For Flash, 2012.12.7, jungki.kim[End] */

/* LGE_CHANGE_S, Set Flash LED Mode For Preview, 2012.10.22, jungki.kim[Start] */
int8_t ce1702_sensor_set_led_flash_mode(struct msm_sensor_ctrl_t *s_ctrl, int32_t led_mode)
{
	int8_t rc = 0;
	unsigned char data[10];
	unsigned char led_power = 0x1A;
	int batt_temp = 0;

	batt_temp = get_pm8921_batt_temp();
	LDBGI("%s: led_mode = [ %d ] battery = [%d]\n", __func__, led_mode, batt_temp);
	if(batt_temp > -10) {
		led_power = 0x1A;	// 1237.5mA
	} else {
		led_power = 0x03;	// 225mA
	}

	if(ce1702_led_flash_mode == led_mode) {
		LDBGE("%s: just before value[%d] = requested value[%d] (SAME!)\n", __func__, ce1702_led_flash_mode, led_mode);
		return rc;
	}

	switch(led_mode) {
		case FLASH_LED_OFF:
			data[0] = 0x03; 	//Snapshot
			data[1] = 0x00;	//on
			rc = ce1702_i2c_write(ce1702_s_interface_ctrl->sensor_i2c_addr, 0xB2, data, 2);	// Set Strobe
			data[0] = 0x00; 	//LED
			data[1] = 0x00;	//OFF
			rc = ce1702_i2c_write(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x06, data, 2);	// CMD:ONOFF!(Manually)
		break;

#ifdef PREVIEW_AF_FLASH_ON
		case FLASH_LED_AUTO:	//auto for on AF
			data[0] = 0x01; 	//AF
			data[1] = 0x02;	//auto
			rc = ce1702_i2c_write(ce1702_s_interface_ctrl->sensor_i2c_addr, 0xB2, data, 2);	// Set Strobe
			data[0] = 0x01; 	//AF
			data[1] = 0x00;
			data[2] = led_power;
			data[3] = 0x00;
			rc = ce1702_i2c_write(ce1702_s_interface_ctrl->sensor_i2c_addr, 0xB3, data, 4);	// How Strong?
		break;

		case FLASH_LED_ON:	//on for on AF
			data[0] = 0x01; 	//AF
			data[1] = 0x01;	//on
			rc = ce1702_i2c_write(ce1702_s_interface_ctrl->sensor_i2c_addr, 0xB2, data, 2);	// Set Strobe
			data[0] = 0x01; 	//AF
			data[1] = 0x00;
			data[2] = led_power;
			data[3] = 0x00;
			rc = ce1702_i2c_write(ce1702_s_interface_ctrl->sensor_i2c_addr, 0xB3, data, 4);	// How Strong?
		break;
#else
		case FLASH_LED_AUTO:	//auto for on AF
			data[0] = 0x03; 	//Snapshot
			data[1] = 0x02; //auto
			rc = ce1702_i2c_write(ce1702_s_interface_ctrl->sensor_i2c_addr, 0xB2, data, 2); // Set Strobe
			data[0] = 0x03; 	//Snapshot
			data[1] = 0x00;
			data[2] = led_power;
			data[3] = 0x00;
			rc = ce1702_i2c_write(ce1702_s_interface_ctrl->sensor_i2c_addr, 0xB3, data, 4); // How Strong?
		break;

		case FLASH_LED_ON:	//on for on AF
			data[0] = 0x03; 	//SnapShot
			data[1] = 0x01; //on
			rc = ce1702_i2c_write(ce1702_s_interface_ctrl->sensor_i2c_addr, 0xB2, data, 2); // Set Strobe
			data[0] = 0x03; 	//Snapshot
			data[1] = 0x00;
			data[2] = led_power;
			data[3] = 0x00;
			rc = ce1702_i2c_write(ce1702_s_interface_ctrl->sensor_i2c_addr, 0xB3, data, 4); // How Strong?
		break;

#endif

		case FLASH_LED_TORCH:	//torch
			data[0] = 0x00;	//Manual
			data[1] = 0x00;
			data[2] = 0x03;	//Max Current on Snapshot
			data[3] = 0x00;	//Max Current on Recording
			rc = ce1702_i2c_write(ce1702_s_interface_ctrl->sensor_i2c_addr, 0xB3, data, 4);	// How Strong?
			data[0] = 0x00; 	//LED
			data[1] = 0x01;	//ON
			rc = ce1702_i2c_write(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x06, data, 2);	// CMD:ONOFF!(Manually)
		break;

		default:
			data[0] = 0x03; 	//Snapshot
			data[1] = 0x00;	//on
			rc = ce1702_i2c_write(ce1702_s_interface_ctrl->sensor_i2c_addr, 0xB2, data, 2);	// Set Strobe
			data[0] = 0x00; 	//LED
			data[1] = 0x00;	//OFF
			rc = ce1702_i2c_write(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x06, data, 2);	// CMD:ONOFF!(Manually)

			led_mode = 0;
		break;
	}

	ce1702_led_flash_mode = led_mode;
	return rc;
}
/* LGE_CHANGE_E, Set Flash LED Mode For Preview, 2012.10.22, jungki.kim[End] */

/* LGE_CHANGE_S, Set Flash LED Mode For Snapshot, 2012.10.22, jungki.kim[Start] */
int8_t ce1702_sensor_set_led_flash_mode_snapshot(int32_t led_mode)
{
	int8_t rc = 0;
	unsigned char data[10];
	unsigned char led_power = 0x1A;
	int batt_temp = 0;

	batt_temp = get_pm8921_batt_temp();
	LDBGI("%s: led_mode = [ %d ] battery = [%d]\n", __func__, led_mode, batt_temp);
	if(batt_temp > -10) {
		led_power = 0x1A;	// 1237.5mA
	} else {
		led_power = 0x03;	// 225mA
	}

	LDBGI("%s: led_mode=%d\n", __func__, led_mode);

	switch(led_mode) {
		case FLASH_LED_AUTO:	//auto for snapshot
			data[0] = 0x03; 	//snapshot
			data[1] = 0x02;	//auto
			rc = ce1702_i2c_write(ce1702_s_interface_ctrl->sensor_i2c_addr, 0xB2, data, 2);	// Set Strobe
			data[0] = 0x03; 	//snapshot
			data[1] = 0x00;
			data[2] = led_power;
			data[3] = 0x00;
			rc = ce1702_i2c_write(ce1702_s_interface_ctrl->sensor_i2c_addr, 0xB3, data, 4);	// How Strong?
		break;

		case FLASH_LED_ON:	//on for snapshot
			data[0] = 0x03; 	//snapshot
			data[1] = 0x01;	//on
			rc = ce1702_i2c_write(ce1702_s_interface_ctrl->sensor_i2c_addr, 0xB2, data, 2);	// Set Strobe
			data[0] = 0x03; 	//snapshot
			data[1] = 0x00;
			data[2] = led_power;
			data[3] = 0x00;
			rc = ce1702_i2c_write(ce1702_s_interface_ctrl->sensor_i2c_addr, 0xB3, data, 4);	// How Strong?
		break;

		case FLASH_LED_TORCH:
		case FLASH_LED_OFF:
		default:
		break;
	}

	return rc;
}
/* LGE_CHANGE_E, Set Flash LED Mode For Snapshot, 2012.10.22, jungki.kim[End] */

//Set Antibanding for CE1702 by jungki.kim
int8_t ce1702_sensor_set_antibanding(struct msm_sensor_ctrl_t *s_ctrl, int32_t antibanding)
{
	int8_t rc = 0;
	unsigned char data[10];

	LDBGI("%s: antibanding=%d\n", __func__, antibanding);

	switch(antibanding) {
		case 0:	//off
			data[0] = 0x00;
			break;

		case 1:	// 50hz
			data[0] = 0x02;
			break;

		case 2:	//60hz
			data[0] = 0x03;
			break;

		case 3:	// auto
			data[0] = 0x01;
			break;
		default:
			data[0] = 0x01;
			break;
	}

	rc = ce1702_i2c_write(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x14, data, 1);

	return rc;
}

/* LGE_CHANGE_S, add the object tracking method for GK project, 2012.10.19 youngil.yun@lge.com */
int32_t ce1702_object_tracking(struct msm_sensor_ctrl_t *s_ctrl, struct rec_t * rect_info)
{
	uint8_t wdata[10];
	uint16_t center_x, center_y;

	LDBGI("%s mode = %d\n", __func__,rect_info->mode);

	if(rect_info->mode) //on
	{
		center_x=(rect_info->x+(rect_info->dx/2));
		center_y=(rect_info->y+(rect_info->dy/2));

		LDBGI("%s x = %d, dx = %d, y = %d, dy = %d\n", __func__,rect_info->x,rect_info->dx,rect_info->y,rect_info->dy);
		LDBGI("%s center_x = %d, center_y = %d\n", __func__,center_x,center_y);

		wdata[0] = 0x04;
		wdata[1] = 0x00;
		wdata[2] = center_x&0xFF;  //lower
		wdata[3] = center_x>>0x8; //upper
		wdata[4] = center_y&0xFF; //lower
		wdata[5] = center_y>>0x8; //upper
		wdata[6] = 0x00;
		wdata[7] = 0x00;
		wdata[8] = 0x00;
		wdata[9] = 0x00;
		ce1702_i2c_write(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x41, wdata,  10);	//set Object tracking area
		wdata[0] = 0x04;
		ce1702_i2c_write(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x42, wdata,  1);	//Object tracking on
	}
	else //off
	{
		wdata[0] = 0x00;
		ce1702_i2c_write(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x42, wdata,  1);	//Object tracking off
	}

	return CE1702_OK ;
}
/* LGE_CHANGE_E, add the object tracking method for GK project, 2012.10.19 youngil.yun@lge.com */

/* LGE_CHANGE_S, Support AEC/AWB Lock For CE1702, 2012.10.22, jungki.kim[Start] */
int8_t ce1702_set_aec_awb_lock(struct msm_sensor_ctrl_t *s_ctrl, int32_t islock)
{
	int8_t rc = 0;
	unsigned char data[10];
	uint8_t rdata[2];
	uint8_t res = 0;

	/* Is it previewing? */
	rdata[0] = 0x00; //NULL
	rc = ce1702_i2c_read(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x6C, rdata, 0, &res, 1);
	LDBGI("%s: %d: Preview state check = %x \n", __func__, __LINE__, res);
	if( (res != 0x08) &&(res != 0x18)) {
		LDBGI("%s: Sensor is not previewing... value=[%x]\n", __func__, res);
		/*
			Comment by Jungki.kim
			After previewing is success, this has to be excuted lately.
		*/
		ce1702_aec_awb_lock = 10 + islock;
		return rc;
	}

	/* Digest late start function */
	if(ce1702_aec_awb_lock >= 10) {
		ce1702_aec_awb_lock = 10 - ce1702_aec_awb_lock;
		islock = ce1702_aec_awb_lock;
		LDBGI("%s: Check_Result: islock = [%d]\n", __func__, islock);
	}

	if(islock) {	// Test point!
		// AEC/AWB Lock
		data[0] = 0x13;
		rc = ce1702_i2c_write(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x11, data, 1);
		LDBGI("%s: AEC/AWB Lock :: Success\n", __func__);
	} else {
		//AEC/AWB Unlock
		data[0] = 0x00;
		rc = ce1702_i2c_write(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x11, data, 1);
		LDBGI("%s: AEC/AWB UnLock :: Success\n", __func__);
	}

	ce1702_aec_awb_lock = islock;
	return rc;
}
/* LGE_CHANGE_E, Support AEC/AWB Lock For CE1702, 2012.10.22, jungki.kim[End] */

/* LGE_CHANGE_S, Get Current Previewing Mode, 2012.10.25, jungki.kim[Start] */
int8_t ce1702_get_cam_open_mode(struct msm_sensor_ctrl_t *s_ctrl, int32_t cam_op_mode)
{
	int8_t rc = 0;

	ce1702_cam_preview = cam_op_mode;

	LDBGI("%s: ### current opened preview for [%s]\n", __func__,
		(ce1702_cam_preview == PREVIEW_MODE_CAMERA)?"SNAPSHOT":"RECORDING");

	return rc;

}
/* LGE_CHANGE_E, Get Current Previewing Mode, 2012.10.25, jungki.kim[End] */

/* LGE_CHANGE_S, add the changing image size for GK project, 2012.10.19 youngil.yun@lge.com */
int32_t ce1702_dim_info(struct msm_sensor_ctrl_t *s_ctrl, struct dimen_t* dimension_info)
{
	size_info.preview_width = dimension_info->preview_width;
	size_info.preview_height = dimension_info->preview_height;
	size_info.picture_width = dimension_info->picture_width;
	size_info.picture_height = dimension_info->picture_height;
	size_info.video_width = dimension_info->video_width;
	size_info.video_height = dimension_info->video_height;

	LDBGI("%s: preview_width = %d, preview_height = %d, picture_width = %d, picture_height = %d\n", __func__,
			size_info.preview_width,size_info.preview_height,size_info.picture_width,size_info.picture_height);

	return CE1702_OK;
}
/* LGE_CHANGE_E, add the changing image size for GK project, 2012.10.19 youngil.yun@lge.com */
/* LGE_CHANGE_S, Add ISO setting for GK/GV, 2012.10.28, gayoung85.lee[Start] */
int8_t ce1702_set_iso(struct msm_sensor_ctrl_t *s_ctrl, int32_t iso)
{
	uint8_t data[10];
	uint8_t rdata[2];
	uint8_t res;
	int retry_cnt;
	uint8_t rc = 0;
	LDBGI("%s\n", __func__);

	if(ce1702_cam_preview == PREVIEW_MODE_CAMCORDER)
	{
		iso = CAMERA_ISO_TYPE_DEFAULT;
		LDBGI("%s: set default camcorder mode EV preset [%d]\n", __func__, iso);
	}

	if(ce1702_iso_value == iso) {
		LDBGI("%s: just before value[%d] = requested value[%d] (SAME!)\n", __func__, ce1702_iso_value, iso);
		return rc;
	}

	data[0] = 0x01;

	switch (iso)
	{
	case CAMERA_ISO_TYPE_AUTO:
		data[1] = 0x00;
		break;

	case CAMERA_ISO_TYPE_100:
		data[1] = 0x02;
		break;

	case CAMERA_ISO_TYPE_200:
		data[1] = 0x03;
		break;

	case CAMERA_ISO_TYPE_400:
		data[1] =0x04;
		break;

	case CAMERA_ISO_TYPE_800:
		data[1] = 0x05;
		break;

	case CAMERA_ISO_TYPE_1600:
		data[1] = 0x06;
		break;

	case CAMERA_ISO_TYPE_DEFAULT:
		data[1] = 0x06;
		break;

	default:
		LDBGI("Not support iso setting \n");
		data[1] = 0x00;
		break;
	}

	LDBGI("ce1702_set_iso : %d	==> 0x04h value : %x, %x \n", iso, data[0], data[1]);

	ce1702_i2c_write(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x04, data, 2);

	data[0]=0x00; //NULL
	ce1702_i2c_write(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x01, data, 0);

	retry_cnt = 0;

	do{
		mdelay(5); //yt.jeon 1115 optimize delay time
		rdata[0] = 0x00;
		rc = ce1702_i2c_read(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x02, rdata, 0, &res, 1);
		LDBGI(">>>> iso reflection check[%d] = %x \n", retry_cnt, res);
		retry_cnt++;
	} while((res != 0) && (retry_cnt < 500));

	if (retry_cnt >=500) rc = -EIO;

	ce1702_iso_value = iso;

	return rc;

}
/* LGE_CHANGE_E, Add ISO setting for GK/GV, 2012.10.28, gayoung85.lee[End] */

/* LGE_CHANGE_S, Add ManualSceneMode for GK/GV, 2012.10.28, gayoung85.lee[Start] */
int8_t ce1702_set_manual_scene_mode(struct msm_sensor_ctrl_t *s_ctrl, int32_t scene_mode)
{
	int8_t rc = TRUE;
	unsigned char data[10];

	LDBGI("%s: scene_mode=%d, ce1702_scene_mode = %d\n", __func__, scene_mode, ce1702_scene_mode);

	data[0] = 0x02; // auto scene detection off
	data[2] = 0x00;
	data[3] = 0x00;
	data[4] = 0x00;
	data[5] = 0x00;
	if(ce1702_scene_mode != scene_mode) {
		switch(scene_mode) {
			case CAMERA_SCENE_OFF:
				data[0] = 0x00;
				data[1] = 0x00;
			LDBGI("%s: CAMERA_SCENE_OFF=%d\n", __func__, scene_mode);
				break;
			case CAMERA_SCENE_LANDSCAPE :
				data[1] = 0x01;
			LDBGI("%s: CAMERA_SCENE_LANDSCAPE=%d\n", __func__, scene_mode);
				break;
			case CAMERA_SCENE_PORTRAIT :
				data[1] = 0x02;
			LDBGI("%s: CAMERA_SCENE_PORTRAIT=%d\n", __func__, scene_mode);
				break;
			case CAMERA_SCENE_NIGHT_PORTRAIT :
				data[1] = 0x03;
			LDBGI("%s: CAMERA_SCENE_NIGHT_PORTRAIT=%d\n", __func__, scene_mode);
				break;
			case CAMERA_SCENE_SPORTS:
				data[1] = 0x04;
			LDBGI("%s: CAMERA_SCENE_SPORTS=%d\n", __func__, scene_mode);
				break;
			case CAMERA_SCENE_SUNSET :
				data[1] = 0x05;
			LDBGI("%s: CAMERA_SCENE_SUNSET=%d\n", __func__, scene_mode);
				break;
			default :
				data[0] = 0x00;
				data[1] = 0x00;
				break;
		}
		rc = ce1702_i2c_write(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x80, data, 6);
		if(rc < 0) {
			LDBGI("%s: Fail to apply scene mode [%d]!!\n", __func__, scene_mode);
			return FALSE;
		}
		ce1702_scene_mode = scene_mode;
	}
	return rc;
}
/* LGE_CHANGE_E, Add ManualSceneMode for GK/GV, 2012.10.28, gayoung85.lee[End] */

/* LGE_CHANGE_S, Insert Model Name And Maker Name In JPEG, 2012.10.31, jungki.kim[Start] */
int8_t ce1702_set_model_name(void)
{
	int8_t rc = 0;
	char *maker_name = "LG Electronics";
	char *model_name;
	unsigned char data[MAX_NUMBER_CE1702+2];
	int i;
#ifdef CONFIG_MACH_APQ8064_GKKT
	model_name = "LG-F240K";
#endif
#ifdef CONFIG_MACH_APQ8064_GKSK
	model_name = "LG-F240S";
#endif
#ifdef CONFIG_MACH_APQ8064_GKU
	model_name = "LG-F240L";
#endif
#ifdef CONFIG_MACH_APQ8064_GKATT
	model_name = "LG-E940";
#endif
#ifdef CONFIG_MACH_APQ8064_GVDCM
	model_name = "DS1201";
#endif

	memset(data, 0, sizeof(data));
	data[0] = 0x06;	// Model
	data[1] = strlen(model_name);
	for(i=0;i<strlen(model_name);i++)
		data[2+i] = model_name[i];
	data[2+i+1] ='\0';

	rc = ce1702_i2c_write(ce1702_s_interface_ctrl->sensor_i2c_addr, 0xA1, data, MAX_NUMBER_CE1702+2);
	LDBGI("%s: model = [%s] rc=[%d]\n", __func__, model_name, rc);

	memset(data, 0, sizeof(data));
	data[0] = 0x05;	// Maker
	data[1] = strlen(maker_name);
	for(i=0;i<strlen(maker_name);i++)
		data[2+i] = maker_name[i];
	data[2+i+1] ='\0';

	rc = ce1702_i2c_write(ce1702_s_interface_ctrl->sensor_i2c_addr, 0xA1, data, MAX_NUMBER_CE1702+2);
	LDBGI("%s: maker = [%s] rc=[%d]\n", __func__, maker_name, rc);

	ce1702_set_time();

	return rc;
}
/* LGE_CHANGE_E, Insert Model Name And Maker Name In JPEG, 2012.10.31, jungki.kim[End] */

int8_t ce1702_set_time(void)
{
	int8_t rc = 0;
	unsigned char cmd;
	unsigned char data[10];
	struct timespec time;
	struct tm tmresult;

	time = __current_kernel_time();
	time_to_tm(time.tv_sec,sys_tz.tz_minuteswest * 60* (-1),&tmresult);

	cmd = 0x0E;

	data[0] = (uint8_t)((tmresult.tm_year + 1900) & 0xFF);
	data[1] = (uint8_t)((tmresult.tm_year + 1900) >> 0x08);
	data[2] = tmresult.tm_mon + 1;
	data[3] = tmresult.tm_mday;
	data[4] = tmresult.tm_hour;
	data[5] = tmresult.tm_min;
	data[6] = tmresult.tm_sec+1;
	rc = ce1702_i2c_write(ce1702_s_interface_ctrl->sensor_i2c_addr, cmd, data, 7);

	//LDBGI("%s: %02x %02x-%02d-%02d %02d:%02d:%02d\n", __func__,
	//	data[0], data[1], data[2], data[3], data[4], data[5], data[6]);

	return rc;
}

/* LGE_CHANGE_S, Set Gyro Data, 2012.10.30, jungki.kim[Start] */
int32_t ce1702_set_gyro_data(struct msm_sensor_ctrl_t *s_ctrl, uint8_t *data)
{
	uint8_t data2[32];

	LDBGI("%s\n", __func__);

	if (copy_from_user(&data2, (void *)data, 32))
		return -EFAULT;

	if(data[0]) // gyro
	{
		ce1702_i2c_write(ce1702_s_interface_ctrl->sensor_i2c_addr, 0xB1, &data[1], 11);
	}

	if(data[12]) // gravity
	{
		ce1702_i2c_write(ce1702_s_interface_ctrl->sensor_i2c_addr, 0xB1, &data[13], 11);
	}

	return CE1702_OK ;
}
/* LGE_CHANGE_E, Set Gyro Data, 2012.10.30, jungki.kim[End] */

/* LGE_CHANGE_S, Add WDR mode for GK project, 2012.10.25, gayoung85.lee[Start] */
int8_t ce1702_set_WDR(struct msm_sensor_ctrl_t *s_ctrl, int32_t wdr_mode)
{
	int8_t rc = TRUE;
	unsigned char data[10];
	uint8_t pdata[2];
	uint8_t res = 0;
	uint8_t wdrdata[3];
	uint8_t rdata[2];
	int cnt = 0;
	if(ce1702_cam_preview != PREVIEW_MODE_CAMCORDER){
		return rc ;
	}

	LDBGI("%s: WDR = %d\n", __func__, wdr_mode);

	pdata[0] = 0x20;
	pdata[1] = 0x01;
	rc = ce1702_i2c_write(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x04, pdata, 2);

	data[0] =0x00;
	rc =  ce1702_i2c_write(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x01, data, 0);

	do {
		mdelay(5); //yt.jeon 1115 optimize delay time
		rdata[0] = 0x00;
		rc = ce1702_i2c_read(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x02, rdata, 0, &res, 1);
		LDBGI("%s: verifying... cnt=%d rc=%d\n", __func__, cnt, rc);
		cnt++;
	} while ( (res != 0) && (cnt < 100) );

	if(cnt >= 100) {
		LDBGI("%s: Fail to read isp status \n", __func__);
		rc = - EIO;
	}
	switch(wdr_mode) {
		case CE1702_STATUS_OFF:
			data[0] = 0x00;
			LDBGI("ce1702_set_WDR : %d \n", wdr_mode);
			break;

		case CE1702_STATUS_ON:
			data[0] = 0x01;
			LDBGI("ce1702_set_WDR : %d \n", wdr_mode);
			break;
		}

	// 0 : off, 1: on
	LDBGI("ce1702_set_WDR : %d \n", wdr_mode);

	rc = ce1702_i2c_write(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x88, data, 1);

	if(rc<0) {
		LDBGI("%s: Fail to apply WDR[%d]\n", __func__, wdr_mode);
		return FALSE;
		}
	rdata[0] = 0x00;
	rc = ce1702_i2c_read(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x87, rdata, 0, wdrdata, 3);
	LDBGI("%s : status = %d, hilight = %d, shadow = %d\n",__func__, wdrdata[0], wdrdata[1],wdrdata[2]);

	return rc;
}
/* LGE_CHANGE_E, Add WDR mode for GK project, 2012.10.25, gayoung85.lee[End] */


/* LGE_CHANGE_S, Insert Rotation Information In EXIF, 2012.10.31, jungki.kim[Start] */
int8_t ce1702_set_exif_rotation(struct msm_sensor_ctrl_t *s_ctrl, int rotation)
{
	int8_t rc = 0;

	ce1702_rotation = rotation;
	LDBGI("%s: rotation=[%d]\n", __func__, ce1702_rotation);

	return rc;
}

int8_t ce1702_set_exif_rotation_to_isp(void)
{
	int8_t rc = 0;
	unsigned char cmd;
	unsigned char data[10];

	memset(data, 0, sizeof(data));

	if(ce1702_rotation != -1) {
		cmd = 0xA2;
		data[0] = 0x00;
		switch(ce1702_rotation) {
			case 0:
				data[1] = 0x01;
				break;
			case 90:
				data[1] = 0x06;
				break;
			case 180:
				data[1] = 0x03;
				break;
			case 270:
				data[1] = 0x08;
				break;
			default:
				data[1] = 0x01;
				LDBGE("%s: Bug!!\n",__func__);
				break;
		}
		rc = ce1702_i2c_write(ce1702_s_interface_ctrl->sensor_i2c_addr, cmd, data, 5);
		if(rc != CE1702_OK)
			LDBGE("%s: Send Command Error!\n", __func__);
	}

	return rc;
}
/* LGE_CHANGE_E, Insert Rotation Information In EXIF, 2012.10.31, jungki.kim[End] */

/* LGE_CHANGE_S, Set GPS Exif Tags For GK/GV, 2012.11.7, jungki.kim[Start] */
int8_t ce1702_set_exif_gps(struct msm_sensor_ctrl_t *s_ctrl, struct k_exif_gps_t *gps_info)
{
	int8_t rc = 0;
	unsigned char cmd;
	unsigned char data[10];
	uint32_t flt;
	//int i;

	cmd = 0xA3;

	// Latitude
	data[0] = 0x80;
	if(gps_info->latRef == 'N')
					data[1] = 0x00;
	else
					data[1] = 0x01;

	data[2] = 0x02;
	data[3] = gps_info->latitude[0];
	data[4] = gps_info->latitude[1];

	data[5] = (uint32_t)(gps_info ->latitude[2] / 10000);
	flt = (gps_info->latitude[2]) % 10000;
	flt = flt * 10;

	data[6] = flt & 0xFF;
	flt = (uint32_t)(flt >> 0x08);
	data[7] = flt & 0xFF;
	flt = (uint32_t)(flt >> 0x08);
	data[8] = flt & 0xFF;

	//for(i=0;i<9;i++)
	//	LDBGI("%s: data[%d]=0x%02x\n", __func__, i, data[i]);
	rc = ce1702_i2c_write(ce1702_s_interface_ctrl->sensor_i2c_addr, cmd, data, 9);
	if(rc != CE1702_OK)
		LDBGE("%s: Error to insert GPS(latitude)!!\n", __func__);

	// Longitude
	data[0] = 0x81;
	if(gps_info->lonRef == 'E')
					data[1] = 0x00;
	else
					data[1] = 0x01;

	data[2] = 0x02;
	data[3] = gps_info->longitude[0];
	data[4] = gps_info->longitude[1];

	data[5] = (uint32_t)(gps_info ->longitude[2] / 10000);
	flt = (gps_info->longitude[2]) % 10000;
	flt = flt*10;

	data[6] = flt & 0xFF;
	flt = (uint32_t)(flt >> 0x08);
	data[7] = flt & 0xFF;
	flt = (uint32_t)(flt >> 0x08);
	data[8] = flt & 0xFF;

	//for(i=0;i<9;i++)
	//	LDBGI("%s: data[%d]=0x%02x\n", __func__, i, data[i]);
	rc = ce1702_i2c_write(ce1702_s_interface_ctrl->sensor_i2c_addr, cmd, data, 9);
	if(rc != CE1702_OK)
		LDBGE("%s: Error to insert GPS(longitude)!!\n", __func__);

	// Altitude
	data[0] = 0x02;
	if(gps_info->altiRef == 0)
					data[1] = 0x00;
	else
					data[1] = 0x01;

	flt = (gps_info->altitude) % 1000;
	gps_info->altitude = (uint32_t)(gps_info->altitude / 1000);

	data[2] = (gps_info->altitude) & 0xFF;
	gps_info->altitude = (uint32_t)((gps_info->altitude) >> 0x08);
	data[3] = (gps_info->altitude) & 0xFF;
	gps_info->altitude = (uint32_t)((gps_info->altitude) >> 0x08);
	data[4] = (gps_info->altitude) & 0xFF;
	gps_info->altitude = (uint32_t)((gps_info->altitude) >> 0x08);
	data[5] = (gps_info->altitude) & 0xFF;

	data[6] = (uint16_t)(flt / 10);

	data[7] = 0x00;
	data[8] = 0x00;

	//for(i=0;i<9;i++)
	//	LDBGI("%s: data[%d]=0x%02x\n", __func__, i, data[i]);
	rc = ce1702_i2c_write(ce1702_s_interface_ctrl->sensor_i2c_addr, cmd, data, 9);
	if(rc != CE1702_OK)
		LDBGE("%s: Error to insert GPS(altitude)!!\n", __func__);


	// Date And Time
	data[0] = 0x20;
	data[1] = (gps_info->gpsDateStamp[0]) & 0xFF; // x lower
	data[2] = (gps_info->gpsDateStamp[0]) >>0x08; // x upper
	data[3] = gps_info->gpsDateStamp[1];
	data[4] = gps_info->gpsDateStamp[2];
	data[5] = gps_info->gpsTimeStamp[0];
	data[6] = gps_info->gpsTimeStamp[1];
	data[7] = gps_info->gpsTimeStamp[2];
	data[8] = 0x00;

	//for(i=0;i<9;i++)
	//	LDBGI("%s: data[%d]=0x%02x\n", __func__, i, data[i]);
	rc = ce1702_i2c_write(ce1702_s_interface_ctrl->sensor_i2c_addr, cmd, data, 9);
	if(rc != CE1702_OK)
		LDBGE("%s: Error to insert GPS(date and time)!!\n", __func__);

	return rc;
}
/* LGE_CHANGE_E, Set GPS Exif Tags For GK/GV, 2012.11.7, jungki.kim[End] */

/* LGE_CHANGE_S, Add Auto Scene Detection for GK project, 2012.11.7, gayoung85.lee[Start] */
int8_t ce1702_asd_enable(struct msm_sensor_ctrl_t *s_ctrl, int32_t asd_onoff)
{
	int8_t rc = TRUE;
	unsigned char wdata[6];
	uint8_t data;

	LDBGI("%s: asd_onoff = [%d], ce1702_scene_mode = [%d], ce1702_is_doing_touchaf = [%d]\n", __func__,
		asd_onoff, ce1702_scene_mode, ce1702_is_doing_touchaf);
	if( ((asd_onoff == 0) && (ce1702_scene_mode != 0)) ||(ce1702_is_doing_touchaf == true) ) {
		LDBGI("%s normal scene mode = [%d] ce1702_is_doing_touchaf = [%s]\n",__func__,
			ce1702_scene_mode, (ce1702_is_doing_touchaf==true)?"true":"false");
		return rc;
	} else {
		LDBGI("%s asd_mode = %d [start]\n", __func__, asd_onoff);
		if(asd_onoff){

		//set preview assist (41H) for face detection
			wdata[0] = 0x00;
			wdata[1] = 0x01;
			wdata[2] = 0x03;
			wdata[3] = 0x00;
			wdata[4] = 0x00;
			wdata[5] = 0x00;

			rc = ce1702_i2c_write(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x41, wdata, 6);
			LDBGI("%s apply preview assist 'on' for face detection\n", __func__);
			//apply preview assist (42H) for face detection
			wdata[0] = 0x01;
			rc = ce1702_i2c_write(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x42, wdata, 1);
		} else {
			/* ASD Disable */
			wdata[0] = 0x00;
			rc = ce1702_i2c_write(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x42, wdata, 1);
			LDBGI("%s apply preview assist 'off'  for face detection\n", __func__);
		}

		wdata[0] = asd_onoff;
		wdata[1] = 0x00;
		wdata[2] = 0x00;
		wdata[3] = 0x00;
		wdata[4] = 0x00;
		wdata[5] = 0x00;

		data = 0x01;
		rc = ce1702_i2c_write(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x14, &data, 1);
		rc = ce1702_i2c_write(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x80, wdata, 6 );

		LDBGI("%s [end]\n", __func__);
	}
	return rc;
}
/* LGE_CHANGE_E, Add Auto Scene Detection for GK project, 2012.11.7, gayoung85.lee[End] */



//20121212, hyungmoo.huh@lge.com, description [START]
int8_t ce1702_set_manual_VCM_position(struct msm_sensor_ctrl_t *s_ctrl, int32_t focus_val)
{
	int8_t rc = 0;
	unsigned char data[3];

	LDBGI("%s: manual focus value=%d\n", __func__, focus_val);

		data[0] = 0x01;	// absolute position
		data[1] = (unsigned char)(focus_val & 0xFF); // x lower
		data[2] = (unsigned char)(focus_val >>0x08); // x upper
		LDBGI("%s: CMD=0x%02X, 0x%02X, 0x%02X, 0x%02X\n", __func__, 0x33, data[0], data[1], data[2]);
		rc = ce1702_i2c_write(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x33, data, 3);
		rc = ce1702_check_af_status(TRUE);

	return rc;
}

int8_t ce1702_set_VCM_default_position(struct msm_sensor_ctrl_t *s_ctrl)
{
	int8_t rc = 0, count = 0;
	unsigned char data[3] = {0, };
	uint8_t rdata[2] = {0, };
	int32_t focus_val = 0;

	data[0] = 0x00;
	rc = ce1702_i2c_read(ce1702_s_interface_ctrl->sensor_i2c_addr, 0x34, data, 0, rdata, 2);

	focus_val = (int32_t)((0xFF00 &(rdata[1] <<0x08)) | rdata[0]);
	LDBGI("%s: focus value=%d\n", __func__, focus_val);

	//20121212, hyungmoo.huh@lge.com, This position value need to adjust if tick noise happens.

	if (focus_val > 200)
	{
		rc = ce1702_set_manual_VCM_position(s_ctrl, 180);
		focus_val = 180;
	}

	if (focus_val < 100)
		rc = ce1702_set_manual_VCM_position(s_ctrl, 0);
	else
	{
		do{
			count++;
			focus_val = focus_val -70;
			rc = ce1702_set_manual_VCM_position(s_ctrl, focus_val);

		}while (focus_val > 80 && count <5);

		rc = ce1702_set_manual_VCM_position(s_ctrl, 0);
	}

	return rc;
}
//20121212, hyungmoo.huh@lge.com, description [END]




static struct msm_sensor_fn_t ce1702_func_tbl = {
	.sensor_start_stream = ce1702_sensor_start_stream, //msm_sensor_start_stream,
	.sensor_stop_stream = ce1702_sensor_stop_stream,//msm_sensor_stop_stream,
	.sensor_group_hold_on = msm_sensor_group_hold_on,
	.sensor_group_hold_off = msm_sensor_group_hold_off,
	.sensor_set_fps = msm_sensor_set_fps,
	.sensor_write_exp_gain = msm_sensor_write_exp_gain1,
	.sensor_write_snapshot_exp_gain = msm_sensor_write_exp_gain1,
#if 0
	.sensor_setting = msm_sensor_setting,
#else
	.sensor_setting = ce1702_sensor_setting,
#endif
	.sensor_csi_setting = msm_sensor_setting1,
	.sensor_set_sensor_mode = msm_sensor_set_sensor_mode,
	.sensor_mode_init = msm_sensor_mode_init,
	.sensor_get_output_info = msm_sensor_get_output_info,
	.sensor_config = msm_sensor_config,
#if 1
	.sensor_power_up = ce1702_power_up,
	.sensor_power_down = ce1702_power_down,
#else
	.sensor_power_up = msm_sensor_power_up,
	.sensor_power_down = msm_sensor_power_down,
#endif
	.sensor_adjust_frame_lines = msm_sensor_adjust_frame_lines1,
	.sensor_get_csi_params = msm_sensor_get_csi_params,
	.sensor_match_id = ce1702_match_id,
	.sensor_special_effect = ce1702_set_special_effect,						// Color effect by Jungki.kim@lge.com
	.sensor_exposure_compensation = ce1702_set_exposure_compensation,	// Adjust Exposure by Jungki.kim@lge.com
	.sensor_set_focus_mode_setting = ce1702_set_focus_mode_setting,		//AF Mode Settings for CE1702 by jungki.kim@lge.com
	.sensor_start_af = ce1702_start_af,									// Start AF for CE1702 by jungki.kim
	.sensor_stop_af = ce1702_stop_af,									// Stop AF for CE1702 by jungki.kim
	.sensor_set_af_window = ce1702_set_af_window,						//Set AF Window for CE1702 by jungki.kim
	.sensor_whitebalance_setting = ce1702_set_wb_setting,					//White Balance Settings for CE1702 by jungki.kim@lge.com
	.sensor_set_zoom_ratio = ce1702_set_zoom_ratio,						//Zoom Ratio Settings for CE1702 by jungki.kim@lge.com
	.sensor_set_manual_focus_length = ce1702_set_manual_focus_length,		//Support Manual Focus by jungki.kim@lge.com
	.sensor_set_led_flash_mode = ce1702_sensor_set_led_flash_mode,		//Support LED Flash only for CE1702 by jungki.kim
	.sensor_set_antibanding_ce1702 = ce1702_sensor_set_antibanding,		//Set Antibanding for CE1702 by jungki.kim
	.sensor_set_ae_window = ce1702_set_ae_window,						//Set AE Window for CE1702 by jungki.kim
	.sensor_object_tracking = ce1702_object_tracking,						//add the object tracking method for GK project, 2012.10.19 youngil.yun@lge.com
	.sensor_set_aec_awb_lock = ce1702_set_aec_awb_lock,					//Support AEC/AWB Lock for CE1702 by jungki.kim
	.sensor_dim_info = ce1702_dim_info,									//add the sensor setting function for GK project, 2012.10.19 youngil.yun@lge.com
	.sensor_get_cam_open_mode = ce1702_get_cam_open_mode,			//Get Current Previewing Mode by jungki.kim@lge.com
	.sensor_set_iso = ce1702_set_iso,									//Support ISO setting for CE1702 by gayoung85.lee
	.sensor_set_manual_scene_mode = ce1702_set_manual_scene_mode,		//Support ManualSceneMode for CE1702 by gayoung85.lee
	.sensor_set_gyro_data = ce1702_set_gyro_data,						//Set Gyro Data by junghee.eim@lge.com
	.sensor_set_wdr = ce1702_set_WDR,									//Set WDR mode for CE1702 by gayoung85.lee
	.sensor_set_exif_rotation = ce1702_set_exif_rotation,					//Insert Rotation Information In EXIF by jungki.kim@lge.com
	.sensor_set_exif_gps = ce1702_set_exif_gps,							//Set GPS Exif Tags For GK/GV by jungki.kim@lge.com
	.sensor_set_asd_enable = ce1702_asd_enable,							//Support ASD for CE1702 by gayoung85.lee
};

static struct msm_sensor_reg_t ce1702_regs = {
	.default_data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.start_stream_conf = ce1702_start_settings,
	.start_stream_conf_size = ARRAY_SIZE(ce1702_start_settings),
	.stop_stream_conf = ce1702_stop_settings,
	.stop_stream_conf_size = ARRAY_SIZE(ce1702_stop_settings),
	.init_settings = &ce1702_init_conf[0],
	.init_size = ARRAY_SIZE(ce1702_init_conf),
	.mode_settings = &ce1702_confs[0],
	.output_settings = &ce1702_dimensions[0],
	.num_conf = ARRAY_SIZE(ce1702_confs),
};

static struct msm_sensor_ctrl_t ce1702_s_ctrl = {
	.msm_sensor_reg = &ce1702_regs,
	.sensor_i2c_client = &ce1702_sensor_i2c_client,
	.sensor_i2c_addr = 0x78,	//0x3c,
	.vreg_seq = ce1702_veg_seq,
	.num_vreg_seq = ARRAY_SIZE(ce1702_veg_seq),
	.sensor_output_reg_addr = &ce1702_reg_addr,
	.sensor_id_info = &ce1702_id_info,
	.sensor_exp_gain_info = &ce1702_exp_gain_info,
	.cam_mode = MSM_SENSOR_MODE_INVALID,
	.msm_sensor_mutex = &ce1702_mut,
	.sensor_i2c_driver = &ce1702_i2c_driver,
	.sensor_v4l2_subdev_info = ce1702_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(ce1702_subdev_info),
	.sensor_v4l2_subdev_ops = &ce1702_subdev_ops,
	.func_tbl = &ce1702_func_tbl,
	.clk_rate = MSM_SENSOR_MCLK_24HZ,
};

module_init(ce1702_sensor_init_module);

MODULE_AUTHOR("LGE.com ");
MODULE_DESCRIPTION("NEC CE1702 Driver");
MODULE_LICENSE("GPL v2");
