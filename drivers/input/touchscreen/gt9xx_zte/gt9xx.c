/* drivers/input/touchscreen/gt9xx.c
 *
 * 2010 - 2013 Goodix Technology.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be a reference
 * to you, when you are integrating the GOODiX's CTP IC into your system,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * Version: 2.2
 * Authors: andrew@goodix.com, meta@goodix.com
 * Release Date: 2014/01/14
 * Revision record:
 *		V1.0:
 *			first Release. By Andrew, 2012/08/31
 *		V1.2:
 *			modify gtp_reset_guitar,
			slot report,tracking_id & 0x0F. By Andrew, 2012/10/15
 *		V1.4:
 *			modify gt9xx_update.c. By Andrew, 2012/12/12
 *		V1.6:
 *			1. new heartbeat/esd_protect mechanism(add external watchdog)
 *			2. doze mode, sliding wakeup
 *			3. 3 more cfg_group(GT9 Sensor_ID: 0~5)
 *			3. config length verification
 *			4. names & comments
 *					By Meta, 2013/03/11
 *		V1.8:
 *			1. pen/stylus identification
 *			2. read double check & fixed config support
 *			3. new esd & slide wakeup optimization
 *					By Meta, 2013/06/08
 *		V2.0:
 *			1. compatible with GT9XXF
 *			2. send config after resume
 *					By Meta, 2013/08/06
 *		V2.2:
 *			1. gt9xx_config for debug
 *			2. gesture wakeup
 *			3. pen separate input device, active-pen button support
 *			4. coordinates & keys optimization
 *					By Meta, 2014/01/14
 */

#include <linux/irq.h>
#include "gt9xx.h"
#include "../touchscreen_fw.h"
#include <linux/regulator/consumer.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>

#if GTP_ICS_SLOT_REPORT
	#include <linux/input/mt.h>
#endif
char *gtp_fwfile_table[GTP_MOUDLE_NUM_MAX] = {
GTP_SENSOR_ID_1_FW_NAME,
GTP_SENSOR_ID_2_FW_NAME,
GTP_SENSOR_ID_3_FW_NAME,
GTP_SENSOR_ID_4_FW_NAME,
GTP_SENSOR_ID_5_FW_NAME,
GTP_SENSOR_ID_6_FW_NAME,
};
int touch_moudle;
#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self,
				unsigned long event, void *data);
static int goodix_ts_suspend(struct device *dev);
static int goodix_ts_resume(struct device *dev);
#endif

static const char *goodix_ts_name = "goodix-touchscreen";
static struct workqueue_struct *goodix_wq;
struct i2c_client *i2c_connect_client;
u8 config[GTP_CONFIG_MAX_LENGTH + GTP_ADDR_LENGTH]
	= {GTP_REG_CONFIG_DATA >> 8, GTP_REG_CONFIG_DATA & 0xff};

#if GTP_HAVE_TOUCH_KEY
	static const u16 touch_key_array[] = GTP_KEY_TAB;
	#define GTP_MAX_KEY_NUM  (sizeof(touch_key_array)/sizeof(touch_key_array[0]))

#if GTP_DEBUG_ON
/* static const int  key_codes[] = {KEY_HOME, KEY_BACK, KEY_MENU, KEY_SEARCH}; */
/* static const char *key_names[] = {"Key_Home", "Key_Back", "Key_Menu", "Key_Search"}; */
#endif

#endif
#define VREG_VDD	"vdd_ana"
#define VREG_VBUS	"vcc_i2c"
/* #define GPIO_TS_IRQ		13 */
/* #define GPIO_TS_RST		12 */
/* static struct goodix_ts_data *goodix_ts; */


struct goodix_ts_data *gtp_ts;
static struct regulator *vdd, *vbus;

static s8 gtp_i2c_test(struct i2c_client *client);
void gtp_reset_guitar(struct i2c_client *client, s32 ms);
s32 gtp_send_cfg(struct i2c_client *client);
void gtp_int_sync(s32 ms);


#ifdef CONFIG_HAS_EARLYSUSPEND
static void goodix_ts_early_suspend(struct early_suspend *h);
static void goodix_ts_late_resume(struct early_suspend *h);
#endif

#if GTP_CREATE_WR_NODE
extern s32 init_wr_node(struct i2c_client *);
extern void uninit_wr_node(void);
#endif

#if GTP_AUTO_UPDATE
extern u8 gup_init_update_proc(struct goodix_ts_data *);
extern s32 gup_update_proc(void *dir);
#endif

#if GTP_ESD_PROTECT
static struct delayed_work gtp_esd_check_work;
static struct workqueue_struct *gtp_esd_check_workqueue;
static void gtp_esd_check_func(struct work_struct *);
static s32 gtp_init_ext_watchdog(struct i2c_client *client);
void gtp_esd_switch(struct i2c_client *, s32);
#endif

/*********** For GT9XXF Start **********/
#if GTP_COMPATIBLE_MODE
extern s32 i2c_read_bytes(struct i2c_client *client, u16 addr, u8 *buf, s32 len);
extern s32 i2c_write_bytes(struct i2c_client *client, u16 addr, u8 *buf, s32 len);
extern s32 gup_clk_calibration(void);
extern s32 gup_fw_download_proc(void *dir, u8 dwn_mode);
extern u8 gup_check_fs_mounted(char *path_name);

void gtp_recovery_reset(struct i2c_client *client);
static s32 gtp_esd_recovery(struct i2c_client *client);
s32 gtp_fw_startup(struct i2c_client *client);
static s32 gtp_main_clk_proc(struct goodix_ts_data *ts);
static s32 gtp_bak_ref_proc(struct goodix_ts_data *ts, u8 mode);
#endif
/********** For GT9XXF End **********/

#if GTP_GESTURE_WAKEUP
typedef enum {
	DOZE_DISABLED = 0,
	DOZE_ENABLED = 1,
	DOZE_WAKEUP = 2,
} DOZE_T;
static DOZE_T doze_status = DOZE_DISABLED;
static s8 gtp_enter_doze(struct goodix_ts_data *ts);
#endif


u8 grp_cfg_version = 0;

/*******************************************************
Function:
	Read data from the i2c slave device.
Input:
	client:		i2c device.
	buf[0~1]:	read start address.
	buf[2~len-1]:	read data buffer.
	len:	GTP_ADDR_LENGTH + read bytes count
Output:
	numbers of i2c_msgs to transfer:
	2: succeed, otherwise: failed
*********************************************************/
s32 gtp_i2c_read(struct i2c_client *client, u8 *buf, s32 len)
{
	struct i2c_msg msgs[2];
	s32 ret = -1;
	s32 retries = 0;

	GTP_DEBUG_FUNC();

	msgs[0].flags = !I2C_M_RD;
	msgs[0].addr  = client->addr;
	msgs[0].len   = GTP_ADDR_LENGTH;
	msgs[0].buf   = &buf[0];

	msgs[1].flags = I2C_M_RD;
	msgs[1].addr  = client->addr;
	msgs[1].len   = len - GTP_ADDR_LENGTH;
	msgs[1].buf   = &buf[GTP_ADDR_LENGTH];

	while (retries < 5) {
		ret = i2c_transfer(client->adapter, msgs, 2);
		if (ret == 2)
			break;
		retries++;
	}
	if ((retries >= 5)) {
	#if GTP_COMPATIBLE_MODE
		struct goodix_ts_data *ts = i2c_get_clientdata(client);
	#endif

	#if GTP_GESTURE_WAKEUP
		if (DOZE_ENABLED == doze_status)
			return ret;
	#endif
		GTP_ERROR("I2C Read: 0x%04X, %d bytes failed, errcode: %d! Process reset.",
			(((u16)(buf[0] << 8)) | buf[1]), len-2, ret);
	#if GTP_COMPATIBLE_MODE
		if (CHIP_TYPE_GT9F == ts->chip_type) {
			gtp_recovery_reset(client);
		} else
	#endif
		{
			gtp_reset_guitar(client, 10);
		}
	}
	return ret;
}



/*******************************************************
Function:
	Write data to the i2c slave device.
Input:
	client:		i2c device.
	buf[0~1]:	write start address.
	buf[2~len-1]:	data buffer
	len:	GTP_ADDR_LENGTH + write bytes count
Output:
	numbers of i2c_msgs to transfer:
	1: succeed, otherwise: failed
*********************************************************/
s32 gtp_i2c_write(struct i2c_client *client, u8 *buf, s32 len)
{
	struct i2c_msg msg;
	s32 ret = -1;
	s32 retries = 0;

	GTP_DEBUG_FUNC();

	msg.flags = !I2C_M_RD;
	msg.addr  = client->addr;
	msg.len   = len;
	msg.buf   = buf;

	while (retries < 5) {
		ret = i2c_transfer(client->adapter, &msg, 1);
		if (ret == 1)
			break;
		retries++;
	}
	if ((retries >= 5)) {
	#if GTP_COMPATIBLE_MODE
		struct goodix_ts_data *ts = i2c_get_clientdata(client);
	#endif

	#if GTP_GESTURE_WAKEUP
		if (DOZE_ENABLED == doze_status)
			return ret;
	#endif
		GTP_ERROR("I2C Write: 0x%04X, %d bytes failed, errcode: %d! Process reset.",
			(((u16)(buf[0] << 8)) | buf[1]), len-2, ret);
	#if GTP_COMPATIBLE_MODE
		if (CHIP_TYPE_GT9F == ts->chip_type) {
			gtp_recovery_reset(client);
		} else
	#endif
		{
			gtp_reset_guitar(client, 10);
		}
	}
	return ret;
}


/*******************************************************
Function:
	i2c read twice, compare the results
Input:
	client:  i2c device
	addr:	 operate address
	rxbuf:	 read data to store, if compare successful
	len:	 bytes to read
Output:
	FAIL:	 read failed
	SUCCESS: read successful
*********************************************************/
s32 gtp_i2c_read_dbl_check(struct i2c_client *client,
	u16 addr, u8 *rxbuf, int len)
{
	u8 buf[16] = {0};
	u8 confirm_buf[16] = {0};
	u8 retry = 0;

	while (retry++ < 3) {
		memset(buf, 0xAA, 16);
		buf[0] = (u8)(addr >> 8);
		buf[1] = (u8)(addr & 0xFF);
		gtp_i2c_read(client, buf, len + 2);

		memset(confirm_buf, 0xAB, 16);
		confirm_buf[0] = (u8)(addr >> 8);
		confirm_buf[1] = (u8)(addr & 0xFF);
		gtp_i2c_read(client, confirm_buf, len + 2);

		if (!memcmp(buf, confirm_buf, len+2)) {
			memcpy(rxbuf, confirm_buf+2, len);
			return SUCCESS_OK;
		}
	}
	GTP_ERROR("I2C read 0x%04X, %d bytes, double check failed!", addr, len);
	return FAIL;
}

/*******************************************************
Function:
	Send config.
Input:
	client: i2c device.
Output:
	result of i2c write operation.
		1: succeed, otherwise: failed
*********************************************************/

s32 gtp_send_cfg(struct i2c_client *client)
{
	s32 ret = 2;

#if GTP_DRIVER_SEND_CFG
	s32 retry = 0;
	struct goodix_ts_data *ts = i2c_get_clientdata(client);

	if (ts->fixed_cfg) {
		GTP_INFO("Ic fixed config, no config sent!");
		return 0;
	} else if (ts->pnl_init_error) {
		GTP_INFO("Error occurred in init_panel, no config sent");
		return 0;
	}

	GTP_INFO("Driver send config.");
	for (retry = 0; retry < 5; retry++) {
		ret = gtp_i2c_write(client, config , GTP_CONFIG_MAX_LENGTH + GTP_ADDR_LENGTH);
		if (ret > 0)
			break;
	}
#endif
	return ret;
}
/*******************************************************
Function:
	Disable irq function
Input:
	ts: goodix i2c_client private data
Output:
	None.
*********************************************************/
void gtp_irq_disable(struct goodix_ts_data *ts)
{
	unsigned long irqflags;

	GTP_DEBUG_FUNC();

	spin_lock_irqsave(&ts->irq_lock, irqflags);
	if (!ts->irq_is_disable) {
		ts->irq_is_disable = 1;
		disable_irq_nosync(ts->client->irq);
	}
	spin_unlock_irqrestore(&ts->irq_lock, irqflags);
}

/*******************************************************
Function:
	Enable irq function
Input:
	ts: goodix i2c_client private data
Output:
	None.
*********************************************************/
void gtp_irq_enable(struct goodix_ts_data *ts)
{
	unsigned long irqflags = 0;

	GTP_DEBUG_FUNC();

	spin_lock_irqsave(&ts->irq_lock, irqflags);
	if (ts->irq_is_disable) {
		enable_irq(ts->client->irq);
		ts->irq_is_disable = 0;
	}
	spin_unlock_irqrestore(&ts->irq_lock, irqflags);
}


/*******************************************************
Function:
	Report touch point event
Input:
	ts: goodix i2c_client private data
	id: trackId
	x:	input x coordinate
	y:	input y coordinate
	w:	input pressure
Output:
	None.
*********************************************************/
static void gtp_touch_down(struct goodix_ts_data *ts,
	s32 id, s32 x, s32 y, s32 w)
{
#if GTP_CHANGE_X2Y
	GTP_SWAP(x, y);
#endif

#if GTP_ICS_SLOT_REPORT
	if (id >= GTP_MAX_TOUCH)
		printk("gtp ID:%d, X:%d, Y:%d, W:%d\n", id, x, y, w);

	input_mt_slot(ts->input_dev, id);
	input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 1);
	input_report_key(ts->input_dev, BTN_TOUCH, 1);
	input_report_key(ts->input_dev, BTN_TOOL_FINGER, 1);
	input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x);
	input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y);
	input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, w);
	input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, w);
#else
	input_report_key(ts->input_dev, BTN_TOUCH, 1);
	input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x);
	input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y);
	input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, w);
	input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, w);
	input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, id);
	input_mt_sync(ts->input_dev);
#endif

}

/*******************************************************
Function:
	Report touch release event
Input:
	ts: goodix i2c_client private data
Output:
	None.
*********************************************************/
static void gtp_touch_up(struct goodix_ts_data *ts, s32 id)
{
#if GTP_ICS_SLOT_REPORT
	if (id >= GTP_MAX_TOUCH)
		printk("gtp Touch id[%2d] too big!\n", id);

	input_mt_slot(ts->input_dev, id);
	input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, false);

#else
	input_report_key(ts->input_dev, BTN_TOUCH, 0);
#endif
}

#if GTP_WITH_PEN

static void gtp_pen_init(struct goodix_ts_data *ts)
{
	s32 ret = 0;

	GTP_INFO("Request input device for pen/stylus.");

	ts->pen_dev = input_allocate_device();
	if (ts->pen_dev == NULL) {
		GTP_ERROR("Failed to allocate input device for pen/stylus.");
		return;
	}

	ts->pen_dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS) ;

#if GTP_ICS_SLOT_REPORT
	input_mt_init_slots(ts->pen_dev, 16);
#else
	ts->pen_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
#endif

	set_bit(BTN_TOOL_PEN, ts->pen_dev->keybit);
	set_bit(INPUT_PROP_DIRECT, ts->pen_dev->propbit);

#if GTP_PEN_HAVE_BUTTON
	input_set_capability(ts->pen_dev, EV_KEY, BTN_STYLUS);
	input_set_capability(ts->pen_dev, EV_KEY, BTN_STYLUS2);
#endif

	input_set_abs_params(ts->pen_dev,
		ABS_MT_POSITION_X, 0, ts->abs_x_max, 0, 0);
	input_set_abs_params(ts->pen_dev,
		ABS_MT_POSITION_Y, 0, ts->abs_y_max, 0, 0);
	input_set_abs_params(ts->pen_dev, ABS_MT_PRESSURE, 0, 255, 0, 0);
	input_set_abs_params(ts->pen_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(ts->pen_dev, ABS_MT_TRACKING_ID, 0, 255, 0, 0);

	ts->pen_dev->name = "goodix-pen";
	ts->pen_dev->id.bustype = BUS_I2C;

	ret = input_register_device(ts->pen_dev);
	if (ret) {
		GTP_ERROR("Register %s input device failed", ts->pen_dev->name);
		return;
	}
}

static void gtp_pen_down(s32 x, s32 y, s32 w, s32 id)
{
	struct goodix_ts_data *ts = i2c_get_clientdata(i2c_connect_client);

#if GTP_CHANGE_X2Y
	GTP_SWAP(x, y);
#endif

	input_report_key(ts->pen_dev, BTN_TOOL_PEN, 1);
#if GTP_ICS_SLOT_REPORT
	input_mt_slot(ts->pen_dev, id);
	input_report_abs(ts->pen_dev, ABS_MT_TRACKING_ID, id);
	input_report_abs(ts->pen_dev, ABS_MT_POSITION_X, x);
	input_report_abs(ts->pen_dev, ABS_MT_POSITION_Y, y);
	input_report_abs(ts->pen_dev, ABS_MT_PRESSURE, w);
	input_report_abs(ts->pen_dev, ABS_MT_TOUCH_MAJOR, w);
#else
	input_report_key(ts->pen_dev, BTN_TOUCH, 1);
	input_report_abs(ts->pen_dev, ABS_MT_POSITION_X, x);
	input_report_abs(ts->pen_dev, ABS_MT_POSITION_Y, y);
	input_report_abs(ts->pen_dev, ABS_MT_PRESSURE, w);
	input_report_abs(ts->pen_dev, ABS_MT_TOUCH_MAJOR, w);
	input_report_abs(ts->pen_dev, ABS_MT_TRACKING_ID, id);
	input_mt_sync(ts->pen_dev);
#endif
	GTP_DEBUG("(%d)(%d, %d)[%d]", id, x, y, w);
}

static void gtp_pen_up(s32 id)
{
	struct goodix_ts_data *ts = i2c_get_clientdata(i2c_connect_client);

	input_report_key(ts->pen_dev, BTN_TOOL_PEN, 0);

#if GTP_ICS_SLOT_REPORT
	input_mt_slot(ts->pen_dev, id);
	input_report_abs(ts->pen_dev, ABS_MT_TRACKING_ID, -1);
#else

	input_report_key(ts->pen_dev, BTN_TOUCH, 0);
#endif

}
#endif

/*******************************************************
Function:
	Goodix touchscreen work function
Input:
	work: work struct of goodix_workqueue
Output:
	None.
*********************************************************/
static void goodix_ts_work_func(struct work_struct *work)
{
	u8	end_cmd[3] = {GTP_READ_COOR_ADDR >> 8, GTP_READ_COOR_ADDR & 0xFF, 0};
	u8	point_data[2 + 1 + 8 * GTP_MAX_TOUCH + 1] = {GTP_READ_COOR_ADDR >> 8,
			GTP_READ_COOR_ADDR & 0xFF};
	u8	touch_num = 0;
	u8	finger = 0;
	u16 pre_touch = 0;
	u8 pre_key = 0;
#if GTP_WITH_PEN
	u8 pen_active = 0;
	u8 pre_pen = 0;
#endif
	u8	key_value = 0;
	u8 *coor_data = NULL;
	s32 input_x = 0;
	s32 input_y = 0;
	s32 input_w = 0;
	s32 id = 0;
	s32 i  = 0;
	s32 ret = -1;
	struct goodix_ts_data *ts = NULL;
	static bool have_finger = false;

#if GTP_COMPATIBLE_MODE
	u8 rqst_buf[3] = {0x80, 0x43};
#endif

#if GTP_GESTURE_WAKEUP
	u8 doze_buf[3] = {0x81, 0x4B};
#endif

	GTP_DEBUG_FUNC();
	ts = container_of(work, struct goodix_ts_data, work);
	if (ts->enter_update)
		return;
#if GTP_GESTURE_WAKEUP
	if (DOZE_ENABLED == doze_status) {
		ret = gtp_i2c_read(i2c_connect_client, doze_buf, 3);
		GTP_DEBUG("0x814B = 0x%02X", doze_buf[2]);
		if (ret > 0) {
			if ((doze_buf[2] == 'a') || (doze_buf[2] == 'b') ||
				(doze_buf[2] == 'c') || (doze_buf[2] == 'd') ||
				(doze_buf[2] == 'e') || (doze_buf[2] == 'g') ||
				(doze_buf[2] == 'h') || (doze_buf[2] == 'm') ||
				(doze_buf[2] == 'o') || (doze_buf[2] == 'q') ||
				(doze_buf[2] == 's') || (doze_buf[2] == 'v') ||
				(doze_buf[2] == 'w') || (doze_buf[2] == 'y') ||
				(doze_buf[2] == 'z') || (doze_buf[2] == 0x5E)) {
				if (doze_buf[2] != 0x5E)
					GTP_INFO("Wakeup by gesture(%c), light up the screen!",
						doze_buf[2]);
				else
					GTP_INFO("Wakeup by gesture(^), light up the screen!");

				doze_status = DOZE_WAKEUP;
				input_report_key(ts->input_dev, KEY_POWER, 1);
				input_sync(ts->input_dev);
				input_report_key(ts->input_dev, KEY_POWER, 0);
				input_sync(ts->input_dev);
				doze_buf[2] = 0x00;
				gtp_i2c_write(i2c_connect_client, doze_buf, 3);

	else if ((doze_buf[2] == 0xAA) || (doze_buf[2] == 0xBB) ||
		(doze_buf[2] == 0xAB) || (doze_buf[2] == 0xBA)) {
				char *direction[4] = {"Right", "Down", "Up", "Left"};
				u8 type = ((doze_buf[2] & 0x0F) - 0x0A) + (((doze_buf[2] >> 4) & 0x0F) - 0x0A) * 2;

				GTP_INFO("%s slide to light up the screen!",
					direction[type]);
				doze_status = DOZE_WAKEUP;
				input_report_key(ts->input_dev, KEY_POWER, 1);
				input_sync(ts->input_dev);
				input_report_key(ts->input_dev, KEY_POWER, 0);
				input_sync(ts->input_dev);
				doze_buf[2] = 0x00;
				gtp_i2c_write(i2c_connect_client, doze_buf, 3);
			} else if (0xC0 == (doze_buf[2] & 0xC0)) {
				GTP_INFO("Double click to light up the screen!");
				doze_status = DOZE_WAKEUP;
				input_report_key(ts->input_dev, KEY_POWER, 1);
				input_sync(ts->input_dev);
				input_report_key(ts->input_dev, KEY_POWER, 0);
				input_sync(ts->input_dev);
				doze_buf[2] = 0x00;
				gtp_i2c_write(i2c_connect_client, doze_buf, 3);
			} else {
				doze_buf[2] = 0x00;
				gtp_i2c_write(i2c_connect_client, doze_buf, 3);
				gtp_enter_doze(ts);
			}
		}
		if (ts->use_irq)
			gtp_irq_enable(ts);

		return;
	}
#endif

	ret = gtp_i2c_read(ts->client, point_data, 12);
	if (ret < 0) {
		GTP_ERROR("I2C transfer error. errno:%d\n ", ret);
		if (ts->use_irq)
			gtp_irq_enable(ts);
		return;
	}

	finger = point_data[GTP_ADDR_LENGTH];

#if GTP_COMPATIBLE_MODE
	if ((finger == 0x00) && (CHIP_TYPE_GT9F == ts->chip_type)) {
		ret = gtp_i2c_read(ts->client, rqst_buf, 3);
		if (ret < 0) {
		GTP_ERROR("Read request status error!");
		goto exit_work_func;
		}

		switch (rqst_buf[2]) {
		case GTP_RQST_CONFIG:
			GTP_INFO("Request for config.");
			ret = gtp_send_cfg(ts->client);
			if (ret < 0)
				GTP_ERROR("Request for config unresponded!");
			else {
				rqst_buf[2] = GTP_RQST_RESPONDED;
				gtp_i2c_write(ts->client, rqst_buf, 3);
				GTP_INFO("Request for config responded!");
			}
			break;

		case GTP_RQST_BAK_REF:
			GTP_INFO("Request for backup reference.");
			ts->rqst_processing = 1;
			ret = gtp_bak_ref_proc(ts, GTP_BAK_REF_SEND);
			if (SUCCESS_OK == ret) {
				rqst_buf[2] = GTP_RQST_RESPONDED;
				gtp_i2c_write(ts->client, rqst_buf, 3);
				ts->rqst_processing = 0;
				GTP_INFO("Request for backup reference responded!");
			}
			break;

		case GTP_RQST_RESET:
			GTP_INFO("Request for reset.");
			gtp_recovery_reset(ts->client);
			break;

		case GTP_RQST_MAIN_CLOCK:
			GTP_INFO("Request for main clock.");
			ts->rqst_processing = 1;
			ret = gtp_main_clk_proc(ts);
			if (FAIL == ret)
				GTP_ERROR("Request for main clock unresponded!");
			else {
				GTP_INFO("Request for main clock responded!");
				rqst_buf[2] = GTP_RQST_RESPONDED;
				gtp_i2c_write(ts->client, rqst_buf, 3);
				ts->rqst_processing = 0;
				ts->clk_chk_fs_times = 0;
			}
			break;

		default:
			GTP_INFO("Undefined request: 0x%02X", rqst_buf[2]);
			rqst_buf[2] = GTP_RQST_RESPONDED;
			gtp_i2c_write(ts->client, rqst_buf, 3);
			break;
		}
	}
#endif
	if (finger == 0x00) {
	if (ts->use_irq) {
		gtp_irq_enable(ts);
	}
	return;
}

if ((finger & 0x80) == 0)
	goto exit_work_func;

touch_num = finger & 0x0f;
if (touch_num > GTP_MAX_TOUCH)
	goto exit_work_func;

if (touch_num > 1) {
	u8 buf[8 * GTP_MAX_TOUCH] = {(GTP_READ_COOR_ADDR + 10) >> 8, (GTP_READ_COOR_ADDR + 10) & 0xff};

	ret = gtp_i2c_read(ts->client, buf, 2 + 8 * (touch_num - 1));
	memcpy(&point_data[12], &buf[2], 8 * (touch_num - 1));
}

#if (GTP_HAVE_TOUCH_KEY || GTP_PEN_HAVE_BUTTON)
	if ((touch_num == 0) && (pre_touch == 0))
		key_value = point_data[3 + 8 * touch_num];
	else {
		#ifdef CONFIG_BOARD_GRUIS
		if (touch_num == 1) {
			if ((point_data[3 + 3] | (point_data[3 + 4] << 8)) >= 795)
				key_value = point_data[3 + 8 * touch_num];
			else
				key_value = 0;
		}
		#else
		key_value = 0;
		#endif
	}

	if (key_value || pre_key) {
		/*for (i = 0; i < GTP_MAX_KEY_NUM; i++)
		{
		#if GTP_DEBUG_ON
			for (ret = 0; ret < 4; ++ret)
			{
				if (key_codes[ret] == touch_key_array[i])
				{
					GTP_DEBUG("Key: %s %s", key_names[ret], (key_value & (0x01 << i)) ? "Down" : "Up");
					break;
				}
			}
		#endif
			input_report_key(ts->input_dev, touch_key_array[i], key_value & (0x01<<i));
		}*/

		switch (key_value) {
			#if 0
		case 1:
			point_data[3] = 0x0;
			point_data[4] = 50;
			point_data[5] = 0x0;
			point_data[6] = 0x08;
			point_data[7] = 0x2;
			point_data[8] = 0xFF;
			point_data[9] = 0;
			touch_num = 1;
			break;
		case 2:
			point_data[3] = 0x0;
			point_data[4] = 160;
			point_data[5] = 0x0;
			point_data[6] = 0x08;
			point_data[7] = 0x2;
			point_data[8] = 0xFF;
			point_data[9] = 0;
			touch_num = 1;
			break;
		case 4:
			point_data[3] = 0x0;
			point_data[4] = 0x0e;
			point_data[5] = 0x1;
			point_data[6] = 0x08;
			point_data[7] = 0x2;
			point_data[8] = 0xFF;
			point_data[9] = 0;
			touch_num = 1;
			break;
			#endif
		#ifdef CONFIG_BOARD_SIF
		case 1:
			input_x  = 180;
			input_y  = 1380;
			input_w  = 255;
			gtp_touch_down(ts, id, input_x, input_y, input_w);
			break;
		case 2:
			input_x  = 360;
			input_y  = 1380;
			input_w  = 255;
			gtp_touch_down(ts, id, input_x, input_y, input_w);
			break;

		case 4:
			input_x  = 540;
			input_y  = 1380;
			input_w  = 255;
			gtp_touch_down(ts, id, input_x, input_y, input_w);
			break;
		#else
		case 1:
			input_x  = 80;
			input_y  = 900;
			input_w  = 255;
			gtp_touch_down(ts, id, input_x, input_y, input_w);
			break;
		case 2:
			input_x  = 240;
			input_y  = 900;
			input_w  = 255;
			gtp_touch_down(ts, id, input_x, input_y, input_w);
			break;

		case 4:
			input_x  = 400;
			input_y  = 900;
			input_w  = 255;
			gtp_touch_down(ts, id, input_x, input_y, input_w);
			break;
		#endif
		case 8:
		default:
			break;
		}
		if ((pre_key != 0) && (key_value == 0))
			gtp_touch_up(ts, 0);
		input_sync(ts->input_dev);

	}
#endif
	pre_key = key_value;

	GTP_DEBUG("pre_touch:%02x, finger:%02x.", pre_touch, finger);

#if GTP_ICS_SLOT_REPORT

#if GTP_WITH_PEN
	if (pre_pen && (touch_num == 0)) {
		GTP_DEBUG("Pen touch UP(Slot)!");
		gtp_pen_up(0);
		pen_active = 1;
		pre_pen = 0;
	}
#endif
	if (pre_touch || touch_num) {
		s32 pos = 0;
		u16 touch_index = 0;
		u8 report_num = 0;
		coor_data = &point_data[3];

		if (touch_num) {
			id = coor_data[pos] & 0x0F;

		#if GTP_WITH_PEN
			id = coor_data[pos];
			if ((id & 0x80)) {
				GTP_DEBUG("Pen touch DOWN(Slot)!");
				input_x  = coor_data[pos + 1] |
					(coor_data[pos + 2] << 8);
				input_y  = coor_data[pos + 3] |
					(coor_data[pos + 4] << 8);
				input_w  = coor_data[pos + 5] |
					(coor_data[pos + 6] << 8);

				gtp_pen_down(input_x, input_y, input_w, 0);
				pre_pen = 1;
				pre_touch = 0;
				pen_active = 1;
			}
		#endif

			touch_index |= (0x01<<id);
		}

		GTP_DEBUG("id = %d, touch_index = 0x%x, pre_touch = 0x%x\n",
			id, touch_index, pre_touch);

		if ((have_finger == false) && (touch_num != 0))
			GTP_INFO("touch down\n");

		if (touch_num != 0)
			have_finger = true;

		for (i = 0; i < GTP_MAX_TOUCH; i++) {
		#if GTP_WITH_PEN
			if (pre_pen == 1)
				break;
		#endif

			if ((touch_index & (0x01<<i))) {
				input_x  = coor_data[pos + 1] |
					(coor_data[pos + 2] << 8);
				input_y  = coor_data[pos + 3] |
					(coor_data[pos + 4] << 8);
				input_w  = coor_data[pos + 5] |
					(coor_data[pos + 6] << 8);

				#ifdef CONFIG_BOARD_GRUIS
				if (input_y < 795) {
					gtp_touch_down(ts, id,
						input_x, input_y, input_w);
					pre_touch |= 0x01 << i;
				}
				#else
				gtp_touch_down(ts, id,
						input_x, input_y, input_w);
				pre_touch |= 0x01 << i;
				#endif

				report_num++;
				if (report_num < touch_num) {
					pos += 8;
					id = coor_data[pos] & 0x0F;
					touch_index |= (0x01<<id);
				}
			} else {
				gtp_touch_up(ts, i);
				pre_touch &= ~(0x01 << i);
			}
		}
	}

	if ((touch_num == 0) && (have_finger == true)) {
		have_finger = false;
		GTP_INFO("touch up\n");
		input_report_key(ts->input_dev, BTN_TOUCH, 0);
		input_report_key(ts->input_dev,BTN_TOOL_FINGER, 0);
	}
#else

	if (touch_num) {
		for (i = 0; i < touch_num; i++) {
			coor_data = &point_data[i * 8 + 3];

			id = coor_data[0] & 0x0F;
			input_x  = coor_data[1] | (coor_data[2] << 8);
			input_y  = coor_data[3] | (coor_data[4] << 8);
			input_w  = coor_data[5] | (coor_data[6] << 8);

		#if GTP_WITH_PEN
			id = coor_data[0];
			if (id & 0x80) {
				GTP_DEBUG("Pen touch DOWN!");
				gtp_pen_down(input_x, input_y, input_w, 0);
				pre_pen = 1;
				pen_active = 1;
				break;
			} else
		#endif
			{
				gtp_touch_down(ts, id,
					input_x, input_y, input_w);
			}
		}
	} else if (pre_touch) {
	#if GTP_WITH_PEN
		if (pre_pen == 1) {
			GTP_DEBUG("Pen touch UP!");
			gtp_pen_up(0);
			pre_pen = 0;
			pen_active = 1;
		} else
	#endif
		{
			GTP_DEBUG("Touch Release!");
			gtp_touch_up(ts, 0);
		}
	}

	pre_touch = touch_num;
#endif

#if GTP_WITH_PEN
	if (pen_active) {
		pen_active = 0;
		input_sync(ts->pen_dev);
	} else
#endif
	{
		input_sync(ts->input_dev);
	}

exit_work_func:
	if (!ts->gtp_rawdiff_mode) {
		ret = gtp_i2c_write(ts->client, end_cmd, 3);
		if (ret < 0)
			GTP_INFO("I2C write end_cmd error!");
	}
	if (ts->use_irq)
		gtp_irq_enable(ts);
}

/*******************************************************
Function:
	Timer interrupt service routine for polling mode.
Input:
	timer: timer struct pointer
Output:
	Timer work mode.
	HRTIMER_NORESTART: no restart mode
*********************************************************/
static enum hrtimer_restart goodix_ts_timer_handler(struct hrtimer *timer)
{
	struct goodix_ts_data *ts = container_of(timer, struct goodix_ts_data, timer);

	GTP_DEBUG_FUNC();

	queue_work(goodix_wq, &ts->work);
	hrtimer_start(&ts->timer,
		ktime_set(0, (GTP_POLL_TIME+6)*1000000), HRTIMER_MODE_REL);
	return HRTIMER_NORESTART;
}

/*******************************************************
Function:
	External interrupt service routine for interrupt mode.
Input:
	irq:  interrupt number.
	dev_id: private data pointer
Output:
	Handle Result.
	IRQ_HANDLED: interrupt handled successfully
*********************************************************/
static irqreturn_t goodix_ts_irq_handler(int irq, void *dev_id)
{
	struct goodix_ts_data *ts = dev_id;

	GTP_DEBUG_FUNC();

	gtp_irq_disable(ts);

	queue_work(goodix_wq, &ts->work);

	return IRQ_HANDLED;
}
/*******************************************************
Function:
	Synchronization.
Input:
	ms: synchronization time in millisecond.
Output:
	None.
*******************************************************/
void gtp_int_sync(s32 ms)
{
	GTP_GPIO_OUTPUT(gtp_ts->irq_gpio, 0);
	msleep(ms);
	GTP_GPIO_AS_INT(gtp_ts->irq_gpio);
}


/*******************************************************
Function:
	Reset chip.
Input:
	ms: reset time in millisecond
Output:
	None.
*******************************************************/
void gtp_reset_guitar(struct i2c_client *client, s32 ms)
{
#if GTP_COMPATIBLE_MODE
	struct goodix_ts_data *ts = i2c_get_clientdata(client);
#endif

	GTP_DEBUG_FUNC();
	GTP_INFO("Guitar reset");
	GTP_GPIO_OUTPUT(gtp_ts->reset_gpio, 0);   /* begin select I2C slave addr */
	msleep(ms);
	GTP_GPIO_OUTPUT(gtp_ts->irq_gpio, client->addr == 0x14);

	msleep(2);
	GTP_GPIO_OUTPUT(gtp_ts->reset_gpio, 1);

	msleep(6);

	GTP_GPIO_AS_INPUT(gtp_ts->reset_gpio);	  /* end select I2C slave addr */

#if GTP_COMPATIBLE_MODE
	if (CHIP_TYPE_GT9F == ts->chip_type)
		return;
#endif

	gtp_int_sync(50);
#if GTP_ESD_PROTECT
	gtp_init_ext_watchdog(client);
#endif
}

#if GTP_GESTURE_WAKEUP
/*******************************************************
Function:
	Enter doze mode for sliding wakeup.
Input:
	ts: goodix tp private data
Output:
	1: succeed, otherwise failed
*******************************************************/
static s8 gtp_enter_doze(struct goodix_ts_data *ts)
{
	s8 ret = -1;
	s8 retry = 0;
	u8 i2c_control_buf[3] = {(u8)(GTP_REG_SLEEP >> 8), (u8)GTP_REG_SLEEP, 8};

	GTP_DEBUG_FUNC();

	GTP_DEBUG("Entering gesture mode.");
	while (retry++ < 5) {
		i2c_control_buf[0] = 0x80;
		i2c_control_buf[1] = 0x46;
		ret = gtp_i2c_write(ts->client, i2c_control_buf, 3);
		if (ret < 0) {
			GTP_DEBUG("failed to set doze flag into 0x8046, %d", retry);
			continue;
		}
		i2c_control_buf[0] = 0x80;
		i2c_control_buf[1] = 0x40;
		ret = gtp_i2c_write(ts->client, i2c_control_buf, 3);
		if (ret > 0) {
			doze_status = DOZE_ENABLED;
			GTP_INFO("Gesture mode enabled.");
			return ret;
		}
		msleep(10);
	}
	GTP_ERROR("GTP send gesture cmd failed.");
	return ret;
}
#else
/*******************************************************
Function:
	Enter sleep mode.
Input:
	ts: private data.
Output:
	Executive outcomes.
	1: succeed, otherwise failed.
*******************************************************/
static s8 gtp_enter_sleep(struct goodix_ts_data *ts)
{
	s8 ret = -1;
	s8 retry = 0;
	u8 i2c_control_buf[3] = {(u8)(GTP_REG_SLEEP >> 8), (u8)GTP_REG_SLEEP, 5};

#if GTP_COMPATIBLE_MODE
	u8 status_buf[3] = {0x80, 0x44};
#endif

	GTP_DEBUG_FUNC();

#if GTP_COMPATIBLE_MODE
	if (CHIP_TYPE_GT9F == ts->chip_type) {
		ret = gtp_i2c_read(ts->client, status_buf, 3);
		if (ret < 0)
			GTP_ERROR("failed to get backup-reference status");

		if (status_buf[2] & 0x80) {
			ret = gtp_bak_ref_proc(ts, GTP_BAK_REF_STORE);
			if (FAIL == ret)
				GTP_ERROR("failed to store bak_ref");
		}
	}
#endif

	GTP_GPIO_OUTPUT(gtp_ts->irq_gpio, 0);
	msleep(5);

	while (retry++ < 5) {
		ret = gtp_i2c_write(ts->client, i2c_control_buf, 3);
		if (ret > 0) {
			GTP_INFO("GTP enter sleep!");

			return ret;
		}
		msleep(10);
	}
	GTP_ERROR("GTP send sleep cmd failed.");
	return ret;
}
#endif
/*******************************************************
Function:
	Wakeup from sleep.
Input:
	ts: private data.
Output:
	Executive outcomes.
	>0: succeed, otherwise: failed.
*******************************************************/
static s8 gtp_wakeup_sleep(struct goodix_ts_data *ts)
{
	u8 retry = 0;
	s8 ret = -1;

	GTP_DEBUG_FUNC();

#if GTP_COMPATIBLE_MODE
	if (CHIP_TYPE_GT9F == ts->chip_type) {
		u8 opr_buf[3] = {0x41, 0x80};

		GTP_GPIO_OUTPUT(gtp_ts->irq_gpio, 1);
		msleep(5);

		for (retry = 0; retry < 20; ++retry) {
			opr_buf[2] = 0x0C;
			ret = gtp_i2c_write(ts->client, opr_buf, 3);
			if (FAIL == ret) {
				GTP_ERROR("failed to hold ss51 & dsp!");
				continue;
			}
			opr_buf[2] = 0x00;
			ret = gtp_i2c_read(ts->client, opr_buf, 3);
			if (FAIL == ret) {
				GTP_ERROR("failed to get ss51 & dsp status!");
				continue;
			}
			if (0x0C != opr_buf[2]) {
				GTP_DEBUG("ss51 & dsp not been hold, %d", retry+1);
				continue;
			}
			GTP_DEBUG("ss51 & dsp confirmed hold");

			ret = gtp_fw_startup(ts->client);
			if (FAIL == ret) {
				GTP_ERROR("failed to startup GT9XXF, process recovery");
				gtp_esd_recovery(ts->client);
			}
			break;
		}
		if (retry >= 10) {
			GTP_ERROR("failed to wakeup, processing esd recovery");
			gtp_esd_recovery(ts->client);
		} else {
			GTP_INFO("GT9XXF gtp wakeup success");
		}
		return ret;
	}
#endif

#if GTP_POWER_CTRL_SLEEP
	while (retry++ < 5) {
		gtp_reset_guitar(ts->client, 20);

		GTP_INFO("GTP wakeup sleep.");
		return 1;
	}
#else
	while (retry++ < 10) {
	#if GTP_GESTURE_WAKEUP
		if (DOZE_WAKEUP != doze_status)
			GTP_INFO("Powerkey wakeup.");
		else
			GTP_INFO("Gesture wakeup.");

		doze_status = DOZE_DISABLED;
		gtp_irq_disable(ts);
		gtp_reset_guitar(ts->client, 10);
		gtp_irq_enable(ts);

	#else
			GTP_GPIO_OUTPUT(gtp_ts->irq_gpio, 1);
		msleep(10);
	#endif

		ret = gtp_i2c_test(ts->client);
		if (ret > 0) {
			GTP_INFO("GTP wakeup sleep.");

		#if (!GTP_GESTURE_WAKEUP)
			{
				gtp_int_sync(40);
			#if GTP_ESD_PROTECT
				gtp_init_ext_watchdog(ts->client);
			#endif
			}
		#endif

			return ret;
		}
		gtp_reset_guitar(ts->client, 20);
	}
#endif

	GTP_ERROR("GTP wakeup sleep failed.");
	return ret;
}
#if GTP_DRIVER_SEND_CFG
static s32 gtp_get_info(struct goodix_ts_data *ts)
{
	u8 opr_buf[6] = {0};
	s32 ret = 0;

	ts->abs_x_max = GTP_MAX_WIDTH;
	ts->abs_y_max = GTP_MAX_HEIGHT;
	ts->int_trigger_type = GTP_INT_TRIGGER;

	opr_buf[0] = (u8)((GTP_REG_CONFIG_DATA+1) >> 8);
	opr_buf[1] = (u8)((GTP_REG_CONFIG_DATA+1) & 0xFF);

	ret = gtp_i2c_read(ts->client, opr_buf, 6);
	if (ret < 0)
		return FAIL;

	ts->abs_x_max = (opr_buf[3] << 8) + opr_buf[2];
	ts->abs_y_max = (opr_buf[5] << 8) + opr_buf[4];

	opr_buf[0] = (u8)((GTP_REG_CONFIG_DATA+6) >> 8);
	opr_buf[1] = (u8)((GTP_REG_CONFIG_DATA+6) & 0xFF);

	ret = gtp_i2c_read(ts->client, opr_buf, 3);
	if (ret < 0)
		return FAIL;

	ts->int_trigger_type = opr_buf[2] & 0x03;

	GTP_INFO("X_MAX = %d, Y_MAX = %d, TRIGGER = 0x%02x",
			ts->abs_x_max, ts->abs_y_max, ts->int_trigger_type);

	return SUCCESS_OK;
}
#endif

/*******************************************************
Function:
	Initialize gtp.
Input:
	ts: goodix private data
Output:
	Executive outcomes.
	0: succeed, otherwise: failed
*******************************************************/
static s32 gtp_init_panel(struct goodix_ts_data *ts)
{
	s32 ret = -1;

#if GTP_DRIVER_SEND_CFG
	s32 i = 0;
	u8 check_sum = 0;
	u8 opr_buf[16] = {0};
	u8 sensor_id = 0;

	u8 cfg_info_group1[] = CTP_CFG_GROUP1;
	u8 cfg_info_group2[] = CTP_CFG_GROUP2;
	u8 cfg_info_group3[] = CTP_CFG_GROUP3;
	u8 cfg_info_group4[] = CTP_CFG_GROUP4;
	u8 cfg_info_group5[] = CTP_CFG_GROUP5;
	u8 cfg_info_group6[] = CTP_CFG_GROUP6;
	u8 *send_cfg_buf[] = {cfg_info_group1, cfg_info_group2, cfg_info_group3,
						cfg_info_group4, cfg_info_group5, cfg_info_group6};
	u8 cfg_info_len[] = { CFG_GROUP_LEN(cfg_info_group1),
						CFG_GROUP_LEN(cfg_info_group2),
						CFG_GROUP_LEN(cfg_info_group3),
						CFG_GROUP_LEN(cfg_info_group4),
						CFG_GROUP_LEN(cfg_info_group5),
						CFG_GROUP_LEN(cfg_info_group6)};

	GTP_DEBUG_FUNC();
	GTP_DEBUG("Config Groups\' Lengths: %d, %d, %d, %d, %d, %d",
		cfg_info_len[0], cfg_info_len[1], cfg_info_len[2], cfg_info_len[3],
		cfg_info_len[4], cfg_info_len[5]);


#if GTP_COMPATIBLE_MODE
	if (CHIP_TYPE_GT9F == ts->chip_type)
		ts->fw_error = 0;
	else
#endif
	{
		ret = gtp_i2c_read_dbl_check(ts->client, 0x41E4, opr_buf, 1);
		if (SUCCESS_OK == ret) {
			if (opr_buf[0] != 0xBE) {
				ts->fw_error = 1;
				GTP_ERROR("Firmware error, no config sent!");
				return -EINVAL;
			}
		}
	}

	if ((!cfg_info_len[1]) && (!cfg_info_len[2]) &&
		(!cfg_info_len[3]) && (!cfg_info_len[4]) &&
		(!cfg_info_len[5]))
		sensor_id = 0;
	else {
	#if GTP_COMPATIBLE_MODE
		msleep(50);
	#endif
		ret = gtp_i2c_read_dbl_check(ts->client, GTP_REG_SENSOR_ID, &sensor_id, 1);
		if (SUCCESS_OK == ret) {
			if (sensor_id >= 0x06) {
				GTP_ERROR("Invalid sensor_id(0x%02X), No Config Sent!", sensor_id);
				ts->pnl_init_error = 1;
				return -EINVAL;
			}
		} else {
			GTP_ERROR("Failed to get sensor_id, No config sent!");
			ts->pnl_init_error = 1;
			return -EINVAL;
		}
		GTP_INFO("Sensor_ID: %d", sensor_id);
	}
	ts->sensor_id = sensor_id;
	ts->gtp_cfg_len = cfg_info_len[sensor_id];
	GTP_INFO("CTP_CONFIG_GROUP%d used, config length: %d", sensor_id + 1, ts->gtp_cfg_len);

	if (ts->gtp_cfg_len < GTP_CONFIG_MIN_LENGTH) {
		GTP_ERROR("Config Group%d is INVALID CONFIG GROUP(Len: %d)! NO Config Sent! You need to check you header file CFG_GROUP section!", sensor_id+1, ts->gtp_cfg_len);
		ts->pnl_init_error = 1;
		return -EINVAL;
	}

#if GTP_COMPATIBLE_MODE
	if (CHIP_TYPE_GT9F == ts->chip_type)
		ts->fixed_cfg = 0;
	else
#endif
	{
		ret = gtp_i2c_read_dbl_check(ts->client, GTP_REG_CONFIG_DATA, &opr_buf[0], 1);

		if (ret == SUCCESS_OK) {
			GTP_DEBUG("CFG_GROUP%d Config Version: %d, 0x%02X; IC Config Version: %d, 0x%02X", sensor_id+1,
						send_cfg_buf[sensor_id][0], send_cfg_buf[sensor_id][0], opr_buf[0], opr_buf[0]);

			if (opr_buf[0] < 90) {
				grp_cfg_version = send_cfg_buf[sensor_id][0];
				send_cfg_buf[sensor_id][0] = 0x00;
				ts->fixed_cfg = 0;
			} else {
				GTP_INFO("Ic fixed config with config version(%d, 0x%02X)", opr_buf[0], opr_buf[0]);
				ts->fixed_cfg = 1;
				gtp_get_info(ts);
				return 0;
			}
		} else {
			GTP_ERROR("Failed to get ic config version!No config sent!");
			return -EINVAL;
		}
	}

	memset(&config[GTP_ADDR_LENGTH], 0, GTP_CONFIG_MAX_LENGTH);
	memcpy(&config[GTP_ADDR_LENGTH], send_cfg_buf[sensor_id], ts->gtp_cfg_len);

#if GTP_CUSTOM_CFG
	config[RESOLUTION_LOC]	   = (u8)GTP_MAX_WIDTH;
	config[RESOLUTION_LOC + 1] = (u8)(GTP_MAX_WIDTH>>8);
	config[RESOLUTION_LOC + 2] = (u8)GTP_MAX_HEIGHT;
	config[RESOLUTION_LOC + 3] = (u8)(GTP_MAX_HEIGHT>>8);

	if (GTP_INT_TRIGGER == 0)
		config[TRIGGER_LOC] &= 0xfe;
	else if (GTP_INT_TRIGGER == 1)
		config[TRIGGER_LOC] |= 0x01;
#endif

	check_sum = 0;
	for (i = GTP_ADDR_LENGTH; i < ts->gtp_cfg_len; i++) {
		check_sum += config[i];
	}
	config[ts->gtp_cfg_len] = (~check_sum) + 1;

#else

	ts->gtp_cfg_len = GTP_CONFIG_MAX_LENGTH;
	ret = gtp_i2c_read(ts->client, config, ts->gtp_cfg_len + GTP_ADDR_LENGTH);
	if (ret < 0) {
		GTP_ERROR("Read Config Failed, Using Default Resolution & INT Trigger!");
		ts->abs_x_max = GTP_MAX_WIDTH;
		ts->abs_y_max = GTP_MAX_HEIGHT;
		ts->int_trigger_type = GTP_INT_TRIGGER;
	}

#endif

	if ((ts->abs_x_max == 0) && (ts->abs_y_max == 0)) {
		ts->abs_x_max = (config[RESOLUTION_LOC + 1] << 8) + config[RESOLUTION_LOC];
		ts->abs_y_max = (config[RESOLUTION_LOC + 3] << 8) + config[RESOLUTION_LOC + 2];
		ts->int_trigger_type = (config[TRIGGER_LOC]) & 0x03;
	}

#if GTP_COMPATIBLE_MODE
	if (CHIP_TYPE_GT9F == ts->chip_type) {
		u8 sensor_num = 0;
		u8 driver_num = 0;
		u8 have_key = 0;

		have_key = (config[GTP_REG_HAVE_KEY - GTP_REG_CONFIG_DATA + 2] & 0x01);

		if (1 == ts->is_950) {
			driver_num = config[GTP_REG_MATRIX_DRVNUM - GTP_REG_CONFIG_DATA + 2];
			sensor_num = config[GTP_REG_MATRIX_SENNUM - GTP_REG_CONFIG_DATA + 2];
			if (have_key)
				driver_num--;
			ts->bak_ref_len = (driver_num * (sensor_num - 1) + 2) * 2 * 6;
		} else {
			driver_num = (config[CFG_LOC_DRVA_NUM] & 0x1F) + (config[CFG_LOC_DRVB_NUM]&0x1F);
			if (have_key)
				driver_num--;
			sensor_num = (config[CFG_LOC_SENS_NUM] & 0x0F) + ((config[CFG_LOC_SENS_NUM] >> 4) & 0x0F);
			ts->bak_ref_len = (driver_num * (sensor_num - 2) + 2) * 2;
		}

		GTP_INFO("Drv * Sen: %d * %d(key: %d), X_MAX: %d, Y_MAX: %d, TRIGGER: 0x%02x",
		driver_num, sensor_num, have_key, ts->abs_x_max, ts->abs_y_max, ts->int_trigger_type);
		return 0;
	} else
#endif
	{
	#if GTP_DRIVER_SEND_CFG
		ret = gtp_send_cfg(ts->client);
		if (ret < 0)
			GTP_ERROR("Send config error.");
		config[GTP_ADDR_LENGTH] = grp_cfg_version;
		check_sum = 0;
		for (i = GTP_ADDR_LENGTH; i < ts->gtp_cfg_len; i++) {
			check_sum += config[i];
		}
		config[ts->gtp_cfg_len] = (~check_sum) + 1;
	#endif
		GTP_INFO("X_MAX: %d, Y_MAX: %d, TRIGGER: 0x%02x",
			ts->abs_x_max, ts->abs_y_max, ts->int_trigger_type);
	}

	msleep(10);
	return 0;
}

/*******************************************************
Function:
	Read chip version.
Input:
	client:  i2c device
	version: buffer to keep ic firmware version
Output:
	read operation return.
	2: succeed, otherwise: failed
*******************************************************/
s32 gtp_read_version(struct i2c_client *client, u16 *version)
{
	s32 ret = -1;
	u8 buf[8] = {GTP_REG_VERSION >> 8, GTP_REG_VERSION & 0xff};

	GTP_DEBUG_FUNC();

	ret = gtp_i2c_read(client, buf, sizeof(buf));
	if (ret < 0) {
		GTP_ERROR("GTP read version failed");
		return ret;
	}

	if (version)
		*version = (buf[7] << 8) | buf[6];

	if (buf[5] == 0x00)
		GTP_INFO("IC Version: %c%c%c_%02x%02x", buf[2], buf[3], buf[4], buf[7], buf[6]);
	else
		GTP_INFO("IC Version: %c%c%c%c_%02x%02x", buf[2], buf[3], buf[4], buf[5], buf[7], buf[6]);

	return ret;
}

s32 gtp_get_ic_version(struct i2c_client *client, u8 *ic_version, int lenth)
{
	s32 ret = -1;

	GTP_DEBUG_FUNC();
	if (ic_version) {
		ic_version[0] = GTP_REG_VERSION >> 8;
		ic_version[1] = GTP_REG_VERSION & 0xff;
		ret = gtp_i2c_read(client, ic_version, lenth);
		if (ret < 0) {
			GTP_ERROR("GTP read version failed");
			return ret;
		}
	}

	return ret;
}
/*******************************************************
Function:
	I2c test Function.
Input:
	client:i2c client.
Output:
	Executive outcomes.
		2: succeed, otherwise failed.
*******************************************************/
static s8 gtp_i2c_test(struct i2c_client *client)
{
	u8 test[3] = {GTP_REG_CONFIG_DATA >> 8, GTP_REG_CONFIG_DATA & 0xff};
	u8 retry = 0;
	s8 ret = -1;

	GTP_DEBUG_FUNC();

	while (retry++ < 5) {
		ret = gtp_i2c_read(client, test, 3);
		if (ret > 0)
			return ret;

		GTP_ERROR("GTP i2c test failed time %d.", retry);
		msleep(10);
	}
	return ret;
}
static int touchscreen_power_init(struct device *dev, int flag, char *vreg_vdd, char *vreg_vbus)
{
	if (flag == 1) {
		vdd = vbus = NULL;

		vdd = regulator_get(dev, vreg_vdd);
		if (IS_ERR(vdd)) {
			pr_err("%s get failed\n", vreg_vdd);
			return -EINVAL;
		}
		if (regulator_set_voltage(vdd, 2850000, 2850000)) {
			pr_err("%s set failed\n", vreg_vdd);
			return -EINVAL;
		}

		vbus = regulator_get(dev, vreg_vbus);
		if (IS_ERR(vbus)) {
			pr_err("%s get failed\n", vreg_vbus);
			return -EINVAL;
		}

		if (regulator_set_voltage(vbus, 1800000, 1800000))
			pr_err(" %s set failed\n", vreg_vbus);


	}

	if (flag == 0) {
		regulator_put(vdd);
		regulator_put(vbus);
	}

	return 0;

}

static void touchscreen_power(int on)
{
	int rc = -EINVAL;

	pr_info("goodix %s: on=%d\n", __func__, on);

	if (!vdd || !vbus)
		return;

	if (on) {
		rc = regulator_enable(vdd);
		if (rc) {
			pr_err("vdd enable failed\n");
			return;
		}
		rc = regulator_enable(vbus);
		if (rc) {
			pr_err("vbus enable failed\n");
			return;
		}
	} else {
		rc = regulator_disable(vdd);
		if (rc) {
			pr_err("vdd disable failed\n");
			return;
		}
		rc = regulator_disable(vbus);
		if (rc) {
			pr_err("vbus disable failed\n");
			return;
		}
	}

}


/*******************************************************
Function:
	Request gpio(INT & RST) ports.
Input:
	ts: private data.
Output:
	Executive outcomes.
	>= 0: succeed, < 0: failed
*******************************************************/
static s8 gtp_request_io_port(struct goodix_ts_data *ts)
{
	s32 ret = 0;

	GTP_DEBUG_FUNC();
	ret = GTP_GPIO_REQUEST(ts->irq_gpio, "GTP_INT_IRQ");
	if (ret < 0) {
		GTP_ERROR("Failed to request GPIO:%d, ERRNO:%d",
			(s32)ts->irq_gpio, ret);
		ret = -ENODEV;
		return ret;
	} else {
		GTP_GPIO_AS_INT(ts->irq_gpio);
		ts->client->irq = gpio_to_irq(ts->irq_gpio);
	}

	ret = GTP_GPIO_REQUEST(ts->reset_gpio, "GTP_RST_PORT");
	if (ret < 0) {
		GTP_ERROR("Failed to request GPIO:%d, ERRNO:%d",
			(s32)ts->reset_gpio, ret);
		ret = -ENODEV;
		GTP_GPIO_FREE(gtp_ts->irq_gpio);
		return ret;
	} else {
		GTP_GPIO_AS_INPUT(ts->reset_gpio);
		gtp_reset_guitar(ts->client, 20);
	}


	return ret;
}
static void gtp_free_io_port(struct goodix_ts_data *ts)
{
	GTP_GPIO_FREE(gtp_ts->reset_gpio);
	GTP_GPIO_FREE(gtp_ts->irq_gpio);

}

/*******************************************************
Function:
	Request interrupt.
Input:
	ts: private data.
Output:
	Executive outcomes.
	0: succeed, -1: failed.
*******************************************************/
static s8 gtp_request_irq(struct goodix_ts_data *ts)
{
	s32 ret = -1;
	const u8 irq_table[] = GTP_IRQ_TAB;

	GTP_DEBUG_FUNC();
	GTP_DEBUG("INT trigger type:%x", ts->int_trigger_type);

	ret  = request_irq(ts->client->irq,
					goodix_ts_irq_handler,
					irq_table[ts->int_trigger_type],
					ts->client->name,
					ts);
	if (ret) {
		GTP_ERROR("Request IRQ failed!ERRNO:%d.", ret);
		GTP_GPIO_AS_INPUT(gtp_ts->irq_gpio);
		GTP_GPIO_FREE(gtp_ts->irq_gpio);

		hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		ts->timer.function = goodix_ts_timer_handler;
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
		return -EINVAL;
	} else {
		gtp_irq_disable(ts);
		ts->use_irq = 1;
		return 0;
	}
}

/*******************************************************
Function:
	Request input device Function.
Input:
	ts:private data.
Output:
	Executive outcomes.
	0: succeed, otherwise: failed.
*******************************************************/
static s8 gtp_request_input_dev(struct goodix_ts_data *ts)
{
	s8 ret = -1;
	s8 phys[32];
#if GTP_HAVE_TOUCH_KEY
	u8 index = 0;
#endif

	GTP_DEBUG_FUNC();

	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		GTP_ERROR("Failed to allocate input device.");
		return -ENOMEM;
	}

	ts->input_dev->evbit[0] = BIT_MASK(EV_SYN) |
		BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS) ;
#if GTP_ICS_SLOT_REPORT
	__set_bit(INPUT_PROP_DIRECT, ts->input_dev->propbit);
	input_mt_init_slots(ts->input_dev, 16, 0);
	set_bit(BTN_TOUCH, ts->input_dev->keybit);
	set_bit(BTN_TOOL_FINGER, ts->input_dev->keybit);
#else
	ts->input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
#endif

#if GTP_HAVE_TOUCH_KEY
	for (index = 0; index < GTP_MAX_KEY_NUM; index++)
		input_set_capability(ts->input_dev, EV_KEY, touch_key_array[index]);
#endif

#if GTP_GESTURE_WAKEUP
	input_set_capability(ts->input_dev, EV_KEY, KEY_POWER);
#endif

#if GTP_CHANGE_X2Y
	GTP_SWAP(ts->abs_x_max, ts->abs_y_max);
#endif

	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X,
		0, ts->abs_x_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y,
		0, ts->abs_y_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID, 0, 255, 0, 0);

	snprintf(phys, sizeof(phys), "input/ts");
	ts->input_dev->name = goodix_ts_name;
	ts->input_dev->phys = phys;
	ts->input_dev->id.bustype = BUS_I2C;
	ts->input_dev->id.vendor = 0xDEAD;
	ts->input_dev->id.product = 0xBEEF;
	ts->input_dev->id.version = 10427;

	ret = input_register_device(ts->input_dev);
	if (ret) {
		GTP_ERROR("Register %s input device failed",
			ts->input_dev->name);
		return -ENODEV;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ts->early_suspend.suspend = goodix_ts_early_suspend;
	ts->early_suspend.resume = goodix_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif

#if GTP_WITH_PEN
	gtp_pen_init(ts);
#endif

	return 0;
}

/************** For GT9XXF Start *************/
#if GTP_COMPATIBLE_MODE

s32 gtp_fw_startup(struct i2c_client *client)
{
	u8 opr_buf[4];
	s32 ret = 0;

	opr_buf[0] = 0xAA;
	ret = i2c_write_bytes(client, 0x8041, opr_buf, 1);
	if (ret < 0)
		return FAIL;

	opr_buf[0] = 0x00;
	ret = i2c_write_bytes(client, 0x4180, opr_buf, 1);
	if (ret < 0)
		return FAIL;

	gtp_int_sync(25);

	ret = i2c_read_bytes(client, 0x8041, opr_buf, 1);
	if (ret < 0)
		return FAIL;

	if (0xAA == opr_buf[0]) {
		GTP_ERROR("IC works abnormally,startup failed.");
		return FAIL;
	} else {
		GTP_INFO("IC works normally, Startup success.");
		opr_buf[0] = 0xAA;
		i2c_write_bytes(client, 0x8041, opr_buf, 1);
		return SUCCESS_OK;
	}
}

static s32 gtp_esd_recovery(struct i2c_client *client)
{
	s32 retry = 0;
	s32 ret = 0;
	struct goodix_ts_data *ts;

	ts = i2c_get_clientdata(client);

	gtp_irq_disable(ts);

	GTP_INFO("GT9XXF esd recovery mode");
	gtp_reset_guitar(client, 20);
	for (retry = 0; retry < 5; ++retry) {
		ret = gup_fw_download_proc(NULL, GTP_FL_ESD_RECOVERY);
		if (FAIL == ret) {
			GTP_ERROR("esd recovery failed %d", retry+1);
			continue;
		}
		ret = gtp_fw_startup(ts->client);
		if (FAIL == ret) {
			GTP_ERROR("GT9XXF start up failed %d", retry+1);
			continue;
		}
		break;
	}
	gtp_irq_enable(ts);

	if (retry >= 5) {
		GTP_ERROR("failed to esd recovery");
		return FAIL;
	}

	GTP_INFO("Esd recovery successful");
	return SUCCESS_OK;
}

void gtp_recovery_reset(struct i2c_client *client)
{
#if GTP_ESD_PROTECT
 gtp_esd_switch(client, SWITCH_OFF);
#endif
	GTP_DEBUG_FUNC();

	gtp_esd_recovery(client);

#if GTP_ESD_PROTECT
	gtp_esd_switch(client, SWITCH_ON);
#endif
}

static s32 gtp_bak_ref_proc(struct goodix_ts_data *ts, u8 mode)
{
	s32 ret = 0;
	s32 i = 0;
	s32 j = 0;
	u16 ref_sum = 0;
	u16 learn_cnt = 0;
	u16 chksum = 0;
	s32 ref_seg_len = 0;
	s32 ref_grps = 0;
	struct file *ref_filp = NULL;
	u8 *p_bak_ref;

	ret = gup_check_fs_mounted("/data");
	if (FAIL == ret) {
		ts->ref_chk_fs_times++;
		GTP_DEBUG("Ref check /data times/MAX_TIMES: %d / %d",
			ts->ref_chk_fs_times, GTP_CHK_FS_MNT_MAX);
		if (ts->ref_chk_fs_times < GTP_CHK_FS_MNT_MAX) {
			msleep(50);
			GTP_INFO("/data not mounted.");
			return FAIL;
		}
		GTP_INFO("check /data mount timeout...");
	} else {
		GTP_INFO("/data mounted!!!(%d/%d)", ts->ref_chk_fs_times, GTP_CHK_FS_MNT_MAX);
	}

	p_bak_ref = (u8 *)kzalloc(ts->bak_ref_len, GFP_KERNEL);

	if (NULL == p_bak_ref) {
		GTP_ERROR("Allocate memory for p_bak_ref failed!");
		return FAIL;
	}

	if (ts->is_950) {
		ref_seg_len = ts->bak_ref_len / 6;
		ref_grps = 6;
	} else {
		ref_seg_len = ts->bak_ref_len;
		ref_grps = 1;
	}
	ref_filp = filp_open(GTP_BAK_REF_PATH, O_RDWR | O_CREAT, 0666);
	if (IS_ERR(ref_filp)) {
		GTP_ERROR("Failed to open/create %s.", GTP_BAK_REF_PATH);
		if (GTP_BAK_REF_SEND == mode)
			goto bak_ref_default;
		else
			goto bak_ref_exit;
	}

	switch (mode) {
	case GTP_BAK_REF_SEND:
		GTP_INFO("Send backup-reference");
		ref_filp->f_op->llseek(ref_filp, 0, SEEK_SET);
		ret = ref_filp->f_op->read(ref_filp, (char *)p_bak_ref, ts->bak_ref_len, &ref_filp->f_pos);
		if (ret < 0) {
			GTP_ERROR("failed to read bak_ref info from file, sending default bak_ref");
			goto bak_ref_default;
		}
		for (j = 0; j < ref_grps; ++j) {
			ref_sum = 0;
			for (i = 0; i < (ref_seg_len); i += 2) {
				ref_sum += (p_bak_ref[i + j * ref_seg_len] << 8) + p_bak_ref[i+1 + j * ref_seg_len];
			}
			learn_cnt = (p_bak_ref[j * ref_seg_len + ref_seg_len - 4] << 8) + (p_bak_ref[j * ref_seg_len + ref_seg_len - 3]);
			chksum = (p_bak_ref[j * ref_seg_len + ref_seg_len - 2] << 8) + (p_bak_ref[j * ref_seg_len + ref_seg_len - 1]);
			GTP_DEBUG("learn count = %d", learn_cnt);
			GTP_DEBUG("chksum = %d", chksum);
			GTP_DEBUG("ref_sum = 0x%04X", ref_sum & 0xFFFF);
			if (1 != ref_sum) {
				GTP_INFO("wrong chksum for bak_ref, reset to 0x00 bak_ref");
				memset(&p_bak_ref[j * ref_seg_len], 0, ref_seg_len);
				p_bak_ref[ref_seg_len + j * ref_seg_len - 1] = 0x01;
			} else {
				if (j == (ref_grps - 1))
					GTP_INFO("backup-reference data in %s used", GTP_BAK_REF_PATH);
			}
		}
		ret = i2c_write_bytes(ts->client, GTP_REG_BAK_REF, p_bak_ref, ts->bak_ref_len);
		if (FAIL == ret) {
			GTP_ERROR("failed to send bak_ref because of iic comm error");
			goto bak_ref_exit;
		}
		break;

	case GTP_BAK_REF_STORE:
		GTP_INFO("Store backup-reference");
		ret = i2c_read_bytes(ts->client, GTP_REG_BAK_REF, p_bak_ref, ts->bak_ref_len);
		if (ret < 0) {
			GTP_ERROR("failed to read bak_ref info, sending default back-reference");
			goto bak_ref_default;
		}
		ref_filp->f_op->llseek(ref_filp, 0, SEEK_SET);
		ref_filp->f_op->write(ref_filp, (char *)p_bak_ref, ts->bak_ref_len, &ref_filp->f_pos);
		break;

	default:
		GTP_ERROR("invalid backup-reference request");
		break;
	}
	ret = SUCCESS_OK;
	goto bak_ref_exit;

bak_ref_default:

	for (j = 0; j < ref_grps; ++j) {
		memset(&p_bak_ref[j * ref_seg_len], 0, ref_seg_len);
		p_bak_ref[j * ref_seg_len + ref_seg_len - 1] = 0x01;
	}
	ret = i2c_write_bytes(ts->client, GTP_REG_BAK_REF,
			p_bak_ref, ts->bak_ref_len);
	if (!IS_ERR(ref_filp)) {
		GTP_INFO("write backup-reference data into %s", GTP_BAK_REF_PATH);
		ref_filp->f_op->llseek(ref_filp, 0, SEEK_SET);
		ref_filp->f_op->write(ref_filp, (char *)p_bak_ref, ts->bak_ref_len, &ref_filp->f_pos);
	}
	if (ret == FAIL)
		GTP_ERROR("failed to load the default backup reference");

bak_ref_exit:

	if (p_bak_ref)
		kfree(p_bak_ref);
	if (ref_filp && !IS_ERR(ref_filp))
		filp_close(ref_filp, NULL);
	return ret;
}


static s32 gtp_verify_main_clk(u8 *p_main_clk)
{
	u8 chksum = 0;
	u8 main_clock = p_main_clk[0];
	s32 i = 0;

	if (main_clock < 50 || main_clock > 120)
		return FAIL;

	for (i = 0; i < 5; ++i) {
		if (main_clock != p_main_clk[i])
			return FAIL;
		chksum += p_main_clk[i];
	}
	chksum += p_main_clk[5];
	if ((chksum) == 0)
		return SUCCESS_OK;
	else
		return FAIL;
}

static s32 gtp_main_clk_proc(struct goodix_ts_data *ts)
{
	s32 ret = 0;
	s32 i = 0;
	s32 clk_chksum = 0;
	struct file *clk_filp = NULL;
	u8 p_main_clk[6] = {0};

	ret = gup_check_fs_mounted("/data");
	if (FAIL == ret) {
		ts->clk_chk_fs_times++;
		GTP_DEBUG("Clock check /data times/MAX_TIMES: %d / %d",
			ts->clk_chk_fs_times, GTP_CHK_FS_MNT_MAX);
		if (ts->clk_chk_fs_times < GTP_CHK_FS_MNT_MAX) {
			msleep(50);
			GTP_INFO("/data not mounted.");
			return FAIL;
		}
		GTP_INFO("Check /data mount timeout!");
	} else {
		GTP_INFO("/data mounted!!!(%d/%d)", ts->clk_chk_fs_times, GTP_CHK_FS_MNT_MAX);
	}

	clk_filp = filp_open(GTP_MAIN_CLK_PATH, O_RDWR | O_CREAT, 0666);
	if (IS_ERR(clk_filp))
		GTP_ERROR("%s is unavailable, calculate main clock", GTP_MAIN_CLK_PATH);
	else {
		clk_filp->f_op->llseek(clk_filp, 0, SEEK_SET);
		clk_filp->f_op->read(clk_filp, (char *)p_main_clk, 6, &clk_filp->f_pos);

		ret = gtp_verify_main_clk(p_main_clk);
		if (FAIL == ret)
			GTP_ERROR("main clock data in %s is wrong, recalculate main clock", GTP_MAIN_CLK_PATH);
		else {
			GTP_INFO("main clock data in %s used, main clock freq: %d", GTP_MAIN_CLK_PATH, p_main_clk[0]);
			filp_close(clk_filp, NULL);
			goto update_main_clk;
		}
	}

#if GTP_ESD_PROTECT
	gtp_esd_switch(ts->client, SWITCH_OFF);
#endif
	ret = gup_clk_calibration();
	gtp_esd_recovery(ts->client);

#if GTP_ESD_PROTECT
	gtp_esd_switch(ts->client, SWITCH_ON);
#endif

	GTP_INFO("calibrate main clock: %d", ret);
	if (ret < 50 || ret > 120) {
		GTP_ERROR("wrong main clock: %d", ret);
		goto exit_main_clk;
	}

	for (i = 0; i < 5; ++i) {
		p_main_clk[i] = ret;
		clk_chksum += p_main_clk[i];
	}
	p_main_clk[5] = 0 - clk_chksum;

	if (!IS_ERR(clk_filp)) {
		GTP_DEBUG("write main clock data into %s", GTP_MAIN_CLK_PATH);
		clk_filp->f_op->llseek(clk_filp, 0, SEEK_SET);
		clk_filp->f_op->write(clk_filp, (char *)p_main_clk, 6, &clk_filp->f_pos);
		filp_close(clk_filp, NULL);
	}

update_main_clk:
	ret = i2c_write_bytes(ts->client, GTP_REG_MAIN_CLK, p_main_clk, 6);
	if (FAIL == ret) {
		GTP_ERROR("update main clock failed!");
		return FAIL;
	}
	return SUCCESS_OK;

exit_main_clk:
	if (!IS_ERR(clk_filp))
		filp_close(clk_filp, NULL);
	return FAIL;
}

s32 gtp_gt9xxf_init(struct i2c_client *client)
{
	s32 ret = 0;

	ret = gup_fw_download_proc(NULL, GTP_FL_FW_BURN);
	if (FAIL == ret)
		return FAIL;

	ret = gtp_fw_startup(client);
	if (FAIL == ret)
		return FAIL;

	return SUCCESS_OK;
}

void gtp_get_chip_type(struct goodix_ts_data *ts)
{
	u8 opr_buf[10] = {0x00};
	s32 ret = 0;

	msleep(10);

	ret = gtp_i2c_read_dbl_check(ts->client, GTP_REG_CHIP_TYPE, opr_buf, 10);

	if (FAIL == ret) {
		GTP_ERROR("Failed to get chip-type, set chip type default: GOODIX_GT9");
		ts->chip_type = CHIP_TYPE_GT9;
		return;
	}

	if (!memcmp(opr_buf, "GOODIX_GT9", 10))
		ts->chip_type = CHIP_TYPE_GT9;
	else
		ts->chip_type = CHIP_TYPE_GT9F;
	GTP_INFO("Chip Type: %s", (ts->chip_type == CHIP_TYPE_GT9) ? "GOODIX_GT9" : "GOODIX_GT9F");
}

#endif
/************* For GT9XXF End ************/
extern char *gtp_file_name;

static	void get_gtp_mounle_fw_name(struct goodix_ts_data *ts)
{
	gtp_file_name = gtp_fwfile_table[ts->sensor_id];
}
char gtp_sdcard_path[256];
char gtp_etc_path[256];
static	void get_gtp_mounle_fw_path(void)
{
	char gtp_sdcard_path_temp[256] = {"/sdcard/"};
	char gtp_etc_path_temp[256] = {"/etc/firmware/"};

	strlcat(gtp_sdcard_path_temp, gtp_file_name, sizeof(gtp_sdcard_path_temp));
	strlcat(gtp_etc_path_temp, gtp_file_name, sizeof(gtp_etc_path_temp));
	strlcpy(gtp_sdcard_path, gtp_sdcard_path_temp, sizeof(gtp_sdcard_path));
	strlcpy(gtp_etc_path, gtp_etc_path_temp, sizeof(gtp_etc_path));

	printk("etc path:%s\n", gtp_etc_path);

}
typedef struct {
	u8	hw_info[4];
	u8	pid[8];
	u16 vid;
} st_fw_head;
#define FW_HEAD_LENGTH 14
#define _CLOSE_FILE(p_file) if (p_file && !IS_ERR(p_file)) \
							{ \
								filp_close(p_file, NULL); \
							}


static int get_gtp_mounle_fw_info(st_fw_head *fw_head)
{
	struct file *file;
	int ret;
	char buf[FW_HEAD_LENGTH];
	mm_segment_t old_fs;

	if (strcmp(gtp_etc_path, "/etc/firmware/") == 0) {
		printk("there is no fw file in /etc/firmware \n");
		return -EINVAL;
	}

	file = filp_open(gtp_etc_path, O_RDONLY, 0);
	if (IS_ERR(file)) {
		GTP_ERROR("Open update file(%s) error!", gtp_etc_path);
		return -EINVAL;
	}
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	file->f_op->llseek(file, 0, SEEK_SET);

	ret = file->f_op->read(file, buf, FW_HEAD_LENGTH, &file->f_pos);
	if (ret < 0) {
		GTP_ERROR("Read firmware head in update file error.");
		goto load_failed;
	}
	memcpy(fw_head, buf, FW_HEAD_LENGTH);

	set_fs(old_fs);
	_CLOSE_FILE(file);

	return 0;

load_failed:
	set_fs(old_fs);
	_CLOSE_FILE(file);

	return -EINVAL;


}
int goodix_update_flag = 0;

static ssize_t
proc_read_val(struct file *file, char __user *page, size_t size, loff_t *ppos)

{
	int len = 0;
	int ret = 0;
	u8 ic_version[8];
	st_fw_head	fw_head;

	printk("%s:---enter---\n", __func__);
	if (*ppos) {
		printk("[PARAM]size: %d, *ppos: %d", (int)size, (int)*ppos);
		printk("[TOOL_READ]ADB call again, return it.");
		return 0;
	}

	ret = gtp_get_ic_version(gtp_ts->client, ic_version, sizeof(ic_version));
	len += snprintf(page + len, size - len, "Manufacturer : %s\n", "gtp");
#if GTP_COMPATIBLE_MODE
	len += snprintf(page + len, size - len, "chip type : %s\n",
		(gtp_ts->chip_type == CHIP_TYPE_GT9) ? "GOODIX_GT9" : "GOODIX_GT9F");
#else
	if (ret >= 0) {
		if (ic_version[5] == 0x00) {
			len += snprintf(page + len, size - len, "chip type : %c%c%c\n",
				ic_version[2], ic_version[3], ic_version[4]);
		} else {
			if (ic_version[5] == 'S' || ic_version[5] == 's') {
				len += snprintf(page + len, size - len, "chip type : %c%c%c%c\n",
					ic_version[2], ic_version[3], ic_version[4], ic_version[5]);
			}
		}
	}
#endif
	len += snprintf(page + len, size - len, "sensor id : %d\n", gtp_ts->sensor_id);
	len += snprintf(page + len, size - len, "fw version : %04x\n", (ic_version[7] << 8) | ic_version[6]);
	len += snprintf(page + len, size - len, "update flag : 0x%x\n", goodix_update_flag);

	if (strcmp(gtp_file_name, "")) {
	len += snprintf(page + len, size - len, "fw file name : %s\n", gtp_file_name);

	printk("etc path:%s\n", gtp_etc_path);
		if (get_gtp_mounle_fw_info(&fw_head) >= 0) {
			len += snprintf(page + len, size - len, "fw file info  : %d%d%d%d\n",
				fw_head.hw_info[0], fw_head.hw_info[1], fw_head.hw_info[2], fw_head.hw_info[3]);
			len += snprintf(page + len, size - len, "fw file pid : %s\n", fw_head.pid);
			len += snprintf(page + len, size - len, "fw file vid : %4x\n", ((fw_head.vid&0xff)<<8)|((fw_head.vid&0xff00)>>8));
			if ((((fw_head.vid&0xff)<<8)|((fw_head.vid&0xff00)>>8)) > ((ic_version[7] << 8) | ic_version[6]))
				len += snprintf(page + len, size - len, "you should update the ts fw\n");

		} else
			len += snprintf(page + len, size - len, "can not update fw now\n");
	} else
		len += snprintf(page + len, size - len, "there is no match fw to update\n");

	*ppos += len;
	return len;
}

static ssize_t proc_write_val(struct file *file, const char  __user *buffer,
		   size_t count, loff_t *off)

{
	unsigned long val, ret;
	printk("%s:---enter---\n", __func__);
	ret = copy_from_user(&val, buffer, 1);
	goodix_update_flag = 0;
	if (strcmp(gtp_file_name, "") == 0) {
		goodix_update_flag = 1;
		return -EINVAL;
	}
	gup_update_proc(gtp_etc_path);
	goodix_update_flag = 1;

	return -EINVAL;
}
static ssize_t
ts_update_read_val(struct file *file, char __user *page,
	size_t size, loff_t *ppos)
{
	int len = 1;

	printk("%s:---enter---\n", __func__);
	if (*ppos) {
		printk("[PARAM]size: %d, *ppos: %d", (int)size, (int)*ppos);
		printk("[TOOL_READ]ADB call again, return it.");
		return 0;
	}
	*ppos += len;
	goodix_update_flag = 0;
	if (strcmp(gtp_file_name, "") == 0) {
		goodix_update_flag = 1;
		return 1;
	}
	gup_update_proc(gtp_etc_path);
	goodix_update_flag = 1;

	return len;
}

static const struct file_operations proc_ops = {
	.owner = THIS_MODULE,
	.read = proc_read_val,
	.write = proc_write_val,
};
static const struct file_operations update_proc_ops = {
	.owner = THIS_MODULE,
	.read = ts_update_read_val,
};

#if 0


static int syna_parse_dt(struct device *dev, struct synaptics_dsx_platform_data *pdata)
{
	int rc;
	pdata->irq_gpio			= GPIO_TS_IRQ;
	pdata->reset_delay_ms	= 100;
	pdata->reset_gpio		= GPIO_TS_RST;
	pdata->vdd				= VREG_VDD;
	pdata->vbus				= VREG_VBUS;
	pdata->irq_flags		= IRQF_TRIGGER_FALLING;
	pdata->x_flip			= 0;
	pdata->y_flip			= 0;

	rc = of_property_read_u32(dev->of_node, "synaptics,max_y", &pdata->maxy_offset);
	if (rc) {
		dev_err(dev, "Failed to read display maxy_offset !!!\n");
		return -EINVAL;
	}
	printk("%s: maxy_offset:%d\n", __func__, pdata->maxy_offset);

	return 0;
}
#endif
static int goodix_parse_dt(struct device *dev, struct goodix_ts_data *pdata)
{

	pdata->irq_gpio			= of_get_named_gpio(pdata->client->dev.of_node,
		"goodix,irq-gpio", 0);
	pdata->reset_gpio		= of_get_named_gpio(pdata->client->dev.of_node,
		"goodix,reset-gpio", 0);
	printk("goodix get reset gpio:%d, irq gpio:%d", pdata->reset_gpio, pdata->irq_gpio);
	return 0;
}
 static int gt_pinctrl_init(struct goodix_ts_data *rmi4_data)
{
	int retval;

	rmi4_data->ts_pinctrl = devm_pinctrl_get(&(rmi4_data->client->dev));
	if (IS_ERR_OR_NULL(rmi4_data->ts_pinctrl)) {
		dev_dbg(&rmi4_data->client->dev,
			"Target does not use pinctrl\n");
		retval = PTR_ERR(rmi4_data->ts_pinctrl);
		rmi4_data->ts_pinctrl = NULL;
		return retval;
	}

	rmi4_data->gpio_state_active
		= pinctrl_lookup_state(rmi4_data->ts_pinctrl, "pmx_ts_active");
	if (IS_ERR_OR_NULL(rmi4_data->gpio_state_active)) {
		dev_dbg(&rmi4_data->client->dev,
			"Can not get ts default pinstate\n");
		retval = PTR_ERR(rmi4_data->gpio_state_active);
		rmi4_data->ts_pinctrl = NULL;
		return retval;
	}

	rmi4_data->gpio_state_suspend
		= pinctrl_lookup_state(rmi4_data->ts_pinctrl, "pmx_ts_suspend");
	if (IS_ERR_OR_NULL(rmi4_data->gpio_state_suspend)) {
		dev_dbg(&rmi4_data->client->dev,
			"Can not get ts sleep pinstate\n");
		retval = PTR_ERR(rmi4_data->gpio_state_suspend);
		rmi4_data->ts_pinctrl = NULL;
		return retval;
	}

	return 0;
}

static int gt_pinctrl_select(struct goodix_ts_data *rmi4_data,
						bool on)
{
	struct pinctrl_state *pins_state;
	int ret;

	pins_state = on ? rmi4_data->gpio_state_active
		: rmi4_data->gpio_state_suspend;
	if (!IS_ERR_OR_NULL(pins_state)) {
		ret = pinctrl_select_state(rmi4_data->ts_pinctrl, pins_state);
		if (ret) {
			dev_err(&rmi4_data->client->dev,
				"can not set %s pins\n",
				on ? "pmx_ts_active" : "pmx_ts_suspend");
			return ret;
		}
	} else
		dev_err(&rmi4_data->client->dev,
			"not a valid '%s' pinstate\n",
				on ? "pmx_ts_active" : "pmx_ts_suspend");

	return 0;
}


/*******************************************************
Function:
	I2c probe.
Input:
	client: i2c device struct.
	id: device id.
Output:
	Executive outcomes.
	0: succeed.
*******************************************************/
 static int goodix_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	s32 ret = -1;
	struct goodix_ts_data *ts;
	u16 version_info;
	struct proc_dir_entry *dir, *refresh;
	char *vdd		= VREG_VDD;
	char *vbus		= VREG_VBUS;
	GTP_DEBUG_FUNC();

	printk("zte_tzb goodix probe start\n");

	GTP_INFO("GTP Driver Version: %s", GTP_DRIVER_VERSION);
	GTP_INFO("GTP Driver Built@%s, %s", __TIME__, __DATE__);
	GTP_INFO("GTP I2C Address: 0x%02x", client->addr);

	i2c_connect_client = client;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		GTP_ERROR("I2C check functionality failed.");
		return -ENODEV;
	}
	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (ts == NULL) {
		GTP_ERROR("Alloc GFP_KERNEL memory failed.");
		return -ENOMEM;
	}

	memset(ts, 0, sizeof(*ts));
	INIT_WORK(&ts->work, goodix_ts_work_func);
	ts->client = client;

#if GTP_ESD_PROTECT
	ts->clk_tick_cnt = 2 * HZ;
	GTP_DEBUG("Clock ticks for an esd cycle: %d", ts->clk_tick_cnt);
	spin_lock_init(&ts->esd_lock);

#endif
	i2c_set_clientdata(client, ts);

	ts->gtp_rawdiff_mode = 0;
	goodix_parse_dt(&client->dev, ts);
	ret = touchscreen_power_init(&client->dev, 1, vdd, vbus);
	if (ret < 0) {
		pr_err("%s, gpio init failed! %d\n", __func__, ret);
		goto power_init_fail;
	}
	touchscreen_power(1);
	gtp_ts = ts;

	ret = gtp_request_io_port(ts);


	if (ret < 0) {
		GTP_ERROR("GTP request IO port failed.");
		goto gpio_request_fail;
	}
	ret = gt_pinctrl_init(ts);
	if (!ret && ts->ts_pinctrl) {
		ret = gt_pinctrl_select(ts, true);
		if (ret < 0)
			goto pinctrl_fail;
	}


	ret = gtp_i2c_test(client);
	if (ret < 0) {
		GTP_ERROR("I2C communication ERROR!");
		goto i2c_test_fail;
	}
	spin_lock_init(&ts->irq_lock);

#if GTP_COMPATIBLE_MODE
		gtp_get_chip_type(ts);

		if (CHIP_TYPE_GT9F == ts->chip_type) {
			ret = gtp_gt9xxf_init(ts->client);
			if (FAIL == ret)
				GTP_INFO("Failed to init GT9XXF.");
		}
#endif

	ret = gtp_read_version(client, &version_info);
	if (ret < 0)
		GTP_ERROR("Read version failed.");
	else
		ts->version = version_info;

	ret = gtp_init_panel(ts);
	if (ret < 0) {
		GTP_ERROR("GTP init panel failed.");
		ts->abs_x_max = GTP_MAX_WIDTH;
		ts->abs_y_max = GTP_MAX_HEIGHT;
		ts->int_trigger_type = GTP_INT_TRIGGER;
	}
	get_gtp_mounle_fw_name(ts);
	get_gtp_mounle_fw_path();


#if GTP_AUTO_UPDATE
	ret = gup_init_update_proc(ts);
	if (ret < 0)
		GTP_ERROR("Create update thread error.");
#endif

	ret = gtp_request_input_dev(ts);
	if (ret < 0)
		GTP_ERROR("GTP request input dev failed");

	ret = gtp_request_irq(ts);
	if (ret < 0)
		GTP_INFO("GTP works in polling mode.");
	else
		GTP_INFO("GTP works in interrupt mode.");

	if (ts->use_irq)
		gtp_irq_enable(ts);

#if GTP_CREATE_WR_NODE
	init_wr_node(client);
#endif

#if GTP_ESD_PROTECT
	gtp_esd_switch(client, SWITCH_ON);
#endif
#if defined(CONFIG_FB)
		ts->fb_notif.notifier_call = fb_notifier_callback;
		ret = fb_register_client(&ts->fb_notif);
		if (ret)
			dev_err(&ts->client->dev,
				"Unable to register fb_notifier: %d\n",
				ret);
#endif
	dir = proc_mkdir("touchscreen", NULL);
	refresh = proc_create("ts_information", 0664, dir, &proc_ops);
	if (refresh == NULL)
		printk("proc_create ts_information failed!\n");
	refresh = proc_create("ts_update", 0444, dir, &update_proc_ops);
	if (refresh == NULL)
		printk("proc_create ts_information failed!\n");

	printk("zte_tzb goodix probe end\n");



	return 0;
i2c_test_fail:
	if (ts->ts_pinctrl)
		gt_pinctrl_select(ts, false);
pinctrl_fail:

	gtp_free_io_port(ts);
gpio_request_fail:
	touchscreen_power(0);
	touchscreen_power_init(&client->dev, 0, vdd, vbus);
power_init_fail:
	kfree(ts);
return ret;
}


/*******************************************************
Function:
	Goodix touchscreen driver release function.
Input:
	client: i2c device struct.
Output:
	Executive outcomes. 0---succeed.
*******************************************************/
static int goodix_ts_remove(struct i2c_client *client)
{
	struct goodix_ts_data *ts = i2c_get_clientdata(client);

	GTP_DEBUG_FUNC();

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&ts->early_suspend);
#endif

#if GTP_CREATE_WR_NODE
	uninit_wr_node();
#endif

#if GTP_ESD_PROTECT
	destroy_workqueue(gtp_esd_check_workqueue);
#endif

	if (ts) {
		if (ts->use_irq) {
			GTP_GPIO_AS_INPUT(gtp_ts->irq_gpio);
			GTP_GPIO_FREE(gtp_ts->irq_gpio);
			free_irq(client->irq, ts);
		} else {
			hrtimer_cancel(&ts->timer);
		}
	}

	GTP_INFO("GTP driver removing...");
	i2c_set_clientdata(client, NULL);
	input_unregister_device(ts->input_dev);
	kfree(ts);

	return 0;
}

#ifdef CONFIG_FB
static int fb_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;
	struct goodix_ts_data *ts =
		container_of(self, struct goodix_ts_data, fb_notif);

	if (evdata && evdata->data && event == FB_EVENT_BLANK &&
			ts && ts->client) {
		blank = evdata->data;
		if (*blank == FB_BLANK_UNBLANK)
			goodix_ts_resume(&ts->client->dev);
		else if ((*blank == FB_BLANK_POWERDOWN) || (*blank == FB_BLANK_NORMAL))
			goodix_ts_suspend(&ts->client->dev);
	}

	return 0;
}

/*******************************************************
Function:
	Early suspend function.
Input:
	h: early_suspend struct.
Output:
	None.
*******************************************************/
static int goodix_ts_suspend(struct device *dev)
{
	struct goodix_ts_data *ts;
	s8 ret = -1;
	ts = gtp_ts;

	GTP_DEBUG_FUNC();
	printk("goodix_ts_suspend\n");
#if GTP_ESD_PROTECT
	gtp_esd_switch(ts->client, SWITCH_OFF);
#endif
	ts->gtp_is_suspend = 1;

#if GTP_GESTURE_WAKEUP
	ret = gtp_enter_doze(ts);
#else
	if (ts->use_irq)
		gtp_irq_disable(ts);
	else
		hrtimer_cancel(&ts->timer);
	ret = gtp_enter_sleep(ts);
#endif
	if (ret < 0)
		GTP_ERROR("GTP early suspend failed.");

	msleep(58);
	return 0;
}

/*******************************************************
Function:
	Late resume function.
Input:
	h: early_suspend struct.
Output:
	None.
*******************************************************/
static int goodix_ts_resume(struct device *dev)
{
	struct goodix_ts_data *ts;
	s8 ret = -1;
	ts = gtp_ts;

	GTP_DEBUG_FUNC();
	printk("goodix_ts_resume\n");

	if (ts->gtp_is_suspend == 0)
		return 0;

	ret = gtp_wakeup_sleep(ts);

#if GTP_GESTURE_WAKEUP
	doze_status = DOZE_DISABLED;
#endif

	if (ret < 0)
		GTP_ERROR("GTP later resume failed.");

#if (GTP_COMPATIBLE_MODE)
	else
#endif
	{
		gtp_send_cfg(ts->client);
	}

	if (ts->use_irq)
		gtp_irq_enable(ts);
	else
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);

	ts->gtp_is_suspend = 0;
#if GTP_ESD_PROTECT
	gtp_esd_switch(ts->client, SWITCH_ON);
#endif
	return 0;

}
#endif

#if GTP_ESD_PROTECT
s32 gtp_i2c_read_no_rst(struct i2c_client *client, u8 *buf, s32 len)
{
	struct i2c_msg msgs[2];
	s32 ret = -1;
	s32 retries = 0;

	GTP_DEBUG_FUNC();

	msgs[0].flags = !I2C_M_RD;
	msgs[0].addr  = client->addr;
	msgs[0].len   = GTP_ADDR_LENGTH;
	msgs[0].buf   = &buf[0];

	msgs[1].flags = I2C_M_RD;
	msgs[1].addr  = client->addr;
	msgs[1].len   = len - GTP_ADDR_LENGTH;
	msgs[1].buf   = &buf[GTP_ADDR_LENGTH];

	while (retries < 5) {
		ret = i2c_transfer(client->adapter, msgs, 2);
		if (ret == 2)
			break;
		retries++;
	}
	if ((retries >= 5))
		GTP_ERROR("I2C Read: 0x%04X, %d bytes failed, errcode: %d!",
			(((u16)(buf[0] << 8)) | buf[1]), len-2, ret);
	return ret;
}

s32 gtp_i2c_write_no_rst(struct i2c_client *client, u8 *buf, s32 len)
{
	struct i2c_msg msg;
	s32 ret = -1;
	s32 retries = 0;

	GTP_DEBUG_FUNC();

	msg.flags = !I2C_M_RD;
	msg.addr  = client->addr;
	msg.len   = len;
	msg.buf   = buf;

	while (retries < 5) {
		ret = i2c_transfer(client->adapter, &msg, 1);
		if (ret == 1)
			break;
		retries++;
	}
	if ((retries >= 5))
		GTP_ERROR("I2C Write: 0x%04X, %d bytes failed, errcode: %d!",
			(((u16)(buf[0] << 8)) | buf[1]), len-2, ret);

	return ret;
}
/*******************************************************
Function:
	switch on & off esd delayed work
Input:
	client:  i2c device
	on:		 SWITCH_ON / SWITCH_OFF
Output:
	void
*********************************************************/
void gtp_esd_switch(struct i2c_client *client, s32 on)
{
	struct goodix_ts_data *ts;

	ts = i2c_get_clientdata(client);
	spin_lock(&ts->esd_lock);

	if (SWITCH_ON == on) {
		if (!ts->esd_running) {
			ts->esd_running = 1;
			spin_unlock(&ts->esd_lock);
			GTP_INFO("Esd started");
			queue_delayed_work(gtp_esd_check_workqueue, &gtp_esd_check_work, ts->clk_tick_cnt);
		} else
			spin_unlock(&ts->esd_lock);
	} else {
		if (ts->esd_running) {
			ts->esd_running = 0;
			spin_unlock(&ts->esd_lock);
			GTP_INFO("Esd cancelled");
			cancel_delayed_work_sync(&gtp_esd_check_work);
		} else {
			spin_unlock(&ts->esd_lock);
		}
	}
}

/*******************************************************
Function:
	Initialize external watchdog for esd protect
Input:
	client:  i2c device.
Output:
	result of i2c write operation.
	1: succeed, otherwise: failed
*********************************************************/
static s32 gtp_init_ext_watchdog(struct i2c_client *client)
{
	u8 opr_buffer[3] = {0x80, 0x41, 0xAA};
	GTP_DEBUG("[Esd]Init external watchdog");
	return gtp_i2c_write_no_rst(client, opr_buffer, 3);
}

/*******************************************************
Function:
	Esd protect function.
	External watchdog added by meta, 2013/03/07
Input:
	work: delayed work
Output:
	None.
*******************************************************/
static void gtp_esd_check_func(struct work_struct *work)
{
	s32 i;
	s32 ret = -1;
	struct goodix_ts_data *ts = NULL;
	u8 esd_buf[5] = {0x80, 0x40};

	GTP_DEBUG_FUNC();

	ts = i2c_get_clientdata(i2c_connect_client);

	if (ts->gtp_is_suspend) {
		GTP_INFO("Esd suspended!");
		return;
	}
	if (ts->enter_update) {
		GTP_INFO("Esd enter update!");
		return;
	}
	for (i = 0; i < 3; i++) {
		ret = gtp_i2c_read_no_rst(ts->client, esd_buf, 4);

		GTP_DEBUG("[Esd]0x8040 = 0x%02X, 0x8041 = 0x%02X", esd_buf[2], esd_buf[3]);
		if ((ret < 0))
			continue;
		else {
			if ((esd_buf[2] == 0xAA) || (esd_buf[3] != 0xAA)) {
				u8 chk_buf[4] = {0x80, 0x40};

				gtp_i2c_read_no_rst(ts->client, chk_buf, 4);

				GTP_DEBUG("[Check]0x8040 = 0x%02X, 0x8041 = 0x%02X", chk_buf[2], chk_buf[3]);

				if ((chk_buf[2] == 0xAA) || (chk_buf[3] != 0xAA)) {
					i = 3;
					break;
				} else {
					continue;
				}
			} else {
				esd_buf[2] = 0xAA;
				gtp_i2c_write_no_rst(ts->client, esd_buf, 3);
				break;
			}
		}
	}
	if (i >= 3) {
	#if GTP_COMPATIBLE_MODE
		if (CHIP_TYPE_GT9F == ts->chip_type) {
			if (ts->rqst_processing)
				GTP_INFO("Request processing, no esd recovery");
			else {
				GTP_ERROR("IC working abnormally! Process esd recovery.");
				esd_buf[0] = 0x42;
				esd_buf[1] = 0x26;
				esd_buf[2] = 0x01;
				esd_buf[3] = 0x01;
				esd_buf[4] = 0x01;
				gtp_i2c_write_no_rst(ts->client, esd_buf, 5);
				msleep(50);
				gtp_esd_recovery(ts->client);
			}
		} else
	#endif
		{
			GTP_ERROR("IC working abnormally! Process reset guitar.");
			esd_buf[0] = 0x42;
			esd_buf[1] = 0x26;
			esd_buf[2] = 0x01;
			esd_buf[3] = 0x01;
			esd_buf[4] = 0x01;
			gtp_i2c_write_no_rst(ts->client, esd_buf, 5);
			msleep(50);
			gtp_reset_guitar(ts->client, 50);
			msleep(50);
			gtp_send_cfg(ts->client);
		}
	}

	if (!ts->gtp_is_suspend)
		queue_delayed_work(gtp_esd_check_workqueue, &gtp_esd_check_work, ts->clk_tick_cnt);
	else
		GTP_INFO("Esd suspended!");
	return;
}
#endif

static const struct i2c_device_id goodix_ts_id[] = {
	{ GTP_I2C_NAME, 0 },
	{ }
};
#ifdef CONFIG_OF
static struct of_device_id goodix_match_table[] = {
	{ .compatible = "goodix,gd-ts",},
	{ },
};
#endif

static struct i2c_driver goodix_ts_driver = {
	.probe		= goodix_ts_probe,
	.remove		= goodix_ts_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND

#endif
	.id_table	= goodix_ts_id,
	.driver = {
		.name	  = "Goodix-TS-tt",
		.owner	  = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = goodix_match_table,
#endif
	},
};

/*******************************************************
Function:
	Driver Install function.
Input:
	None.
Output:
	Executive Outcomes. 0---succeed.
********************************************************/
static int	goodix_ts_init(void)
{
	s32 ret;

	GTP_DEBUG_FUNC();
	GTP_INFO("GTP driver installing...");
	goodix_wq = create_singlethread_workqueue("goodix_wq");
	if (!goodix_wq) {
		GTP_ERROR("Creat workqueue failed.");
		return -ENOMEM;
	}
#if GTP_ESD_PROTECT
	INIT_DELAYED_WORK(&gtp_esd_check_work, gtp_esd_check_func);
	gtp_esd_check_workqueue = create_workqueue("gtp_esd_check");
#endif
	ret = i2c_add_driver(&goodix_ts_driver);
	return ret;
}

/*******************************************************
Function:
	Driver uninstall function.
Input:
	None.
Output:
	Executive Outcomes. 0---succeed.
********************************************************/
static void  goodix_ts_exit(void)
{
	GTP_DEBUG_FUNC();
	GTP_INFO("GTP driver exited.");
	i2c_del_driver(&goodix_ts_driver);
	if (goodix_wq)
		destroy_workqueue(goodix_wq);
}

module_init(goodix_ts_init);
module_exit(goodix_ts_exit);

MODULE_DESCRIPTION("GTP Series Driver");
MODULE_LICENSE("GPL");
