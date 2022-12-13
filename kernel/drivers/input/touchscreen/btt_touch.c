/***************************************************************
                    BTT TFT 触摸屏驱动程序
 ***************************************************************/
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/input-polldev.h>
#include <linux/input.h>
#include <linux/input/mt.h>

/* FT5422 寄存器相关宏定义 */
#define FT5422_DEVIDE_MODE_REG 0x00 // 模式寄存器
#define FT5422_CUR_POINT_REG 0x02 // 触摸点数量寄存器
#define FT5422_TOUCH_DATA_REG 0x03 // 触摸数据读取的起始寄存器
#define FT5422_ID_G_MODE_REG 0xA4 // 中断模式寄存器

#define MAX_SUPPORT_POINTS 5 // 最大支持5点触摸

#define TOUCH_EVENT_DOWN 0x00 // 按下
#define TOUCH_EVENT_UP 0x01 // 抬起
#define TOUCH_EVENT_ON 0x02 // 接触
#define TOUCH_EVENT_RESERVED 0x03 // 保留

#define RPI_TS_DEFAULT_WIDTH (800)
#define RPI_TS_DEFAULT_HEIGHT (480)

#define TS_POLL_INTERVAL 17 /* 60fps */
#define TS_POLL_INTERVAL_MAX 50

struct btt_read_regs {
	u8 device_mode;
	u8 gesture_id;
	u8 num_points;
	struct rpi_ts_touch {
		u8 xh;
		u8 xl;
		u8 yh;
		u8 yl;
		u8 pressure; /* Not supported */
		u8 area; /* Not supported */
	} point[MAX_SUPPORT_POINTS];
};

struct btt_touch_dev {
	struct i2c_client *client;
	struct input_polled_dev *idev;
};

#if 0 // 本次没有用到写操作;
static int btt_touch_write(struct btt_touch_dev *btt_ts,
            u8 addr, u8 *buf, u16 len)
{
    struct i2c_client *client = btt_ts->client;
    struct i2c_msg msg;
    u8 send_buf[6] = {0};
    int ret;

    send_buf[0] = addr;
    memcpy(&send_buf[1], buf, len);

    msg.flags = 0;                  //i2c写
    msg.addr = client->addr;
    msg.buf = send_buf;
    msg.len = len + 1;

    ret = i2c_transfer(client->adapter, &msg, 1);
    if (1 == ret)
        return 0;
    else {
        dev_err(&client->dev, "%s: write error, addr=0x%x len=%d.\n",
                    __func__, addr, len);
        return -1;
    }
}
#endif

static int btt_touch_read(struct btt_touch_dev *btt_ts, u8 addr, u8 *buf,
			  u16 len)
{
	struct i2c_client *client = btt_ts->client;
	struct i2c_msg msg[2];
	int ret;

	msg[0].flags = 0; // i2c写
	msg[0].addr = client->addr;
	msg[0].buf = &addr;
	msg[0].len = 1; // 1个字节

	msg[1].flags = I2C_M_RD; // i2c读
	msg[1].addr = client->addr;
	msg[1].buf = buf;
	msg[1].len = len;

	ret = i2c_transfer(client->adapter, msg, 2);
	if (2 == ret)
		return 0;
	else {
		dev_err(&client->dev,
			"%s: read error, ret=%d,addr=0x%x len=%d.\n", __func__,
			ret, addr, len);
		return -1;
	}
}

static void btt_touch_poll(struct input_polled_dev *dev)
{
	struct btt_read_regs regs;
	struct btt_touch_dev *btt_ts = dev->private;

	int event_type;
	int touchid;
	int x, y;
	int i;

	bool down;
	int ret;

	/* 读取 FT5422 触摸点个数 */
	ret = btt_touch_read(btt_ts, FT5422_CUR_POINT_REG, &regs.num_points, 1);
	if (ret) {
		printk("polling error\n");
		return;
	}

	for (i = 0; i < regs.num_points; i++) {
		btt_touch_read(btt_ts, FT5422_TOUCH_DATA_REG + (6 * i),
			       &regs.point[i].xh, 4);

		x = RPI_TS_DEFAULT_WIDTH -
		    ((((int)regs.point[i].xh & 0xf) << 8) + regs.point[i].xl);
		y = RPI_TS_DEFAULT_HEIGHT -
		    ((((int)regs.point[i].yh & 0xf) << 8) + regs.point[i].yl);
		touchid = (regs.point[i].yh >> 4) & 0xf;
		event_type = (regs.point[i].xh >> 6) & 0x03;

		if (event_type == TOUCH_EVENT_RESERVED)
			continue;

		down = event_type != TOUCH_EVENT_UP;

		if (event_type == TOUCH_EVENT_DOWN ||
		    event_type == TOUCH_EVENT_ON) {
			input_mt_slot(dev->input, touchid);
			input_mt_report_slot_state(dev->input, MT_TOOL_FINGER,
						   down);

			if (!down)
				continue;

			input_report_abs(dev->input, ABS_MT_POSITION_X, x);
			input_report_abs(dev->input, ABS_MT_POSITION_Y, y);

			// printk("x=%d ,y=%d\n",x,y);
		}
	}

	input_mt_slot(dev->input, i);
	input_mt_report_slot_inactive(dev->input);

	input_mt_report_pointer_emulation(dev->input, true);
	// input_mt_sync_frame(dev->input);
	input_sync(dev->input);
}

static int btt_touch_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct btt_touch_dev *btt_ts;
	struct input_polled_dev *input_poll_dev;
	int ret;

	/* Check basic functionality */
	ret = i2c_check_functionality(client->adapter,
				      I2C_FUNC_SMBUS_WRITE_BYTE_DATA |
					      I2C_FUNC_SMBUS_READ_BYTE_DATA |
					      I2C_FUNC_SMBUS_READ_I2C_BLOCK);
	if (!ret) {
		dev_err(&client->dev, "%s adapter not supported\n",
			dev_driver_string(&client->adapter->dev));
		return -ENODEV;
	}

	/* 实例化一个 struct btt_touch_dev 对象 */
	btt_ts = devm_kzalloc(&client->dev, sizeof(struct btt_touch_dev),
			      GFP_KERNEL);
	if (!btt_ts) {
		dev_err(&client->dev,
			"Failed to allocate btt touch driver data.\n");
		return -ENOMEM;
	}

	input_poll_dev = devm_input_allocate_polled_device(&client->dev);
	if (!input_poll_dev) {
		dev_err(&client->dev,
			"Failed to allocate btt touch polled driver data.\n");
		return -ENOMEM;
	}

	i2c_set_clientdata(client, btt_ts);

	btt_ts->client = client;
	btt_ts->idev = input_poll_dev;

	input_poll_dev->private = btt_ts;
	input_poll_dev->input->name = "BTT TFT TouchScreen";
	input_poll_dev->input->id.bustype = BUS_I2C;
	// input_poll_dev->input->id.vendor = 0xABCD;
	// input_poll_dev->input->id.product = 0x0001;
	// input_poll_dev->input->id.version = 0x0100;
	input_poll_dev->poll = btt_touch_poll;
	input_poll_dev->poll_interval = TS_POLL_INTERVAL;
	input_poll_dev->poll_interval_max = TS_POLL_INTERVAL_MAX;
	// input_poll_dev->open      = apple6500_open;
	// input_poll_dev->close     = apple6500_close;

	/* 设置input设备需要上报哪些事件*/
	__set_bit(EV_SYN, input_poll_dev->input->evbit);
	__set_bit(EV_KEY, input_poll_dev->input->evbit); /* 按键事件 */
	__set_bit(EV_ABS, input_poll_dev->input->evbit); /* 重复事件 */
	__set_bit(BTN_TOUCH, input_poll_dev->input->keybit);

	input_set_abs_params(input_poll_dev->input, ABS_MT_POSITION_X, 0,
			     RPI_TS_DEFAULT_WIDTH, 0, 0);
	input_set_abs_params(input_poll_dev->input, ABS_MT_POSITION_Y, 0,
			     RPI_TS_DEFAULT_HEIGHT, 0, 0);

	ret = input_mt_init_slots(input_poll_dev->input, MAX_SUPPORT_POINTS,
				  INPUT_MT_DIRECT);
	if (ret) {
		dev_err(&client->dev, "could not init mt slots, %d\n", ret);
		return ret;
	}

	ret = input_register_polled_device(input_poll_dev);
	if (ret) {
		input_unregister_polled_device(input_poll_dev);
		dev_err(&client->dev,
			"failed to register polled input device\n");
		return ret;
	}

	printk("BTT TouchScreen probe Success!\n");

	return 0;
}

static int btt_touch_remove(struct i2c_client *client)
{
	struct btt_touch_dev *btt_ts = i2c_get_clientdata(client);

	input_unregister_polled_device(btt_ts->idev);
	return 0;
}

static const struct of_device_id btt_touch_of_match[] = {
	{
		.compatible = "btt,BTT-touch",
	},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, btt_touch_of_match);

static struct i2c_driver btt_touch_driver = {
    .driver = {
        .owner     		= THIS_MODULE,
        .name          	= "BTT_Touch",
        .of_match_table	= of_match_ptr(btt_touch_of_match),
    },
    .probe    = btt_touch_probe,
    .remove   = btt_touch_remove,
};

module_i2c_driver(btt_touch_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("BTT-tech");
MODULE_INFO(intree, "Y");
