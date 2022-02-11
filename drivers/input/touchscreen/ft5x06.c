/***************************************************************
 Copyright © ALIENTEK Co., Ltd. 1998-2029. All rights reserved.
 文件名    : ft5426.c
 作者      : 正点原子Linux团队
 版本      : V1.0
 描述      : FocalTech FT5426触摸屏驱动程序
 其他      : 无
 论坛      : www.openedv.com
 日志      : 初版V1.0 2021/04/02 正点原子Linux团队创建
 ***************************************************************/
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/input/mt.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/interrupt.h>

/* FT5426寄存器相关宏定义 */
#define FT5426_DEVIDE_MODE_REG	0x00	// 模式寄存器
#define FT5426_TD_STATUS_REG    	0x02  	// 状态寄存器
#define FT5426_TOUCH_DATA_REG   	0x03   	// 触摸数据读取的起始寄存器
#define FT5426_ID_G_MODE_REG    	0xA4  	// 中断模式寄存器

#define MAX_SUPPORT_POINTS		5    	// ft5426最大支持5点触摸

#define TOUCH_EVENT_DOWN      	0x00    // 按下
#define TOUCH_EVENT_UP           	0x01    // 抬起
#define TOUCH_EVENT_ON         	0x02    // 接触
#define TOUCH_EVENT_RESERVED  	0x03    // 保留


struct edt_ft5426_dev {
    struct i2c_client *client;
    struct input_dev *input;
    int reset_gpio;
    int irq_gpio;
};

static int edt_ft5426_ts_write(struct edt_ft5426_dev *ft5426,
            u8 addr, u8 *buf, u16 len)
{
    struct i2c_client *client = ft5426->client;
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

static int edt_ft5426_ts_read(struct edt_ft5426_dev *ft5426,
            u8 addr, u8 *buf, u16 len)
{
    struct i2c_client *client = ft5426->client;
    struct i2c_msg msg[2];
    int ret;

    msg[0].flags = 0;             	// i2c写
    msg[0].addr = client->addr;
    msg[0].buf = &addr;
    msg[0].len = 1;              	// 1个字节

    msg[1].flags = I2C_M_RD;    	//i2c读
    msg[1].addr = client->addr;
    msg[1].buf = buf;
    msg[1].len = len;

    ret = i2c_transfer(client->adapter, msg, 2);
    if (2 == ret)
        return 0;
    else {
        dev_err(&client->dev, "%s: read error, addr=0x%x len=%d.\n",
                    __func__, addr, len);
        return -1;
    }
}

static int edt_ft5426_ts_reset(struct edt_ft5426_dev *ft5426)
{
    struct i2c_client *client = ft5426->client;
    int ret;

    /* 从设备树中获取复位管脚 */
    ft5426->reset_gpio = of_get_named_gpio(client->dev.of_node, "reset-gpios", 0);
    if (!gpio_is_valid(ft5426->reset_gpio)) {
        dev_err(&client->dev, "Failed to get ts reset gpio\n");
        return ft5426->reset_gpio;
    }

    /* 申请使用管脚 */
    ret = devm_gpio_request_one(&client->dev, ft5426->reset_gpio,
                GPIOF_OUT_INIT_HIGH, "ft5426 reset");
    if (ret < 0)
        return ret;

    msleep(20);
    gpio_set_value_cansleep(ft5426->reset_gpio, 0);    	// 拉低复位引脚
    msleep(5);
    gpio_set_value_cansleep(ft5426->reset_gpio, 1);   	// 拉高复位引脚，结束复位

    return 0;
}

static irqreturn_t edt_ft5426_ts_isr(int irq, void *dev_id)
{
    struct edt_ft5426_dev *ft5426 = dev_id;
    u8 rdbuf[30] = {0};
    int i, type, x, y, id;
    bool down;
    int ret;

    /* 读取FT5426触摸点坐标从0x02寄存器开始，连续读取29个寄存器 */
    ret = edt_ft5426_ts_read(ft5426, FT5426_TD_STATUS_REG, rdbuf, 29);
    if (ret)
        goto out;

    for (i = 0; i < MAX_SUPPORT_POINTS; i++) {

        u8 *buf = &rdbuf[i * 6 + 1];

        /* 以第一个触摸点为例，寄存器TOUCH1_XH(地址0x03)，各bit位描述如下：
         * bit7:6  Event flag  0:按下 1:释放 2:接触 3:没有事件
         * bit5:4  保留
         * bit3:0  X轴触摸点的11~8位
         */
        type = buf[0] >> 6;                     // 获取触摸点的Event Flag
        if (type == TOUCH_EVENT_RESERVED)
            continue;

        /* 我们所使用的触摸屏和FT5426是反过来的 */
        x = ((buf[2] << 8) | buf[3]) & 0x0fff;
        y = ((buf[0] << 8) | buf[1]) & 0x0fff;

        /* 以第一个触摸点为例，寄存器TOUCH1_YH(地址0x05)，各bit位描述如下：
         * bit7:4  Touch ID  触摸ID，表示是哪个触摸点
         * bit3:0  Y轴触摸点的11~8位。
         */
        id = (buf[2] >> 4) & 0x0f;
        down = type != TOUCH_EVENT_UP;

        input_mt_slot(ft5426->input, id);
        input_mt_report_slot_state(ft5426->input, MT_TOOL_FINGER, down);

        if (!down)
            continue;

        input_report_abs(ft5426->input, ABS_MT_POSITION_X, x);
        input_report_abs(ft5426->input, ABS_MT_POSITION_Y, y);
    }

    input_mt_report_pointer_emulation(ft5426->input, true);
    input_sync(ft5426->input);

out:
    return IRQ_HANDLED;
}

static int edt_ft5426_ts_irq(struct edt_ft5426_dev *ft5426)
{
    struct i2c_client *client = ft5426->client;
    int ret;

    /* 从设备树中获取中断管脚 */
    ft5426->irq_gpio = of_get_named_gpio(client->dev.of_node, "irq-gpios", 0);
    if (!gpio_is_valid(ft5426->irq_gpio)) {
        dev_err(&client->dev, "Failed to get ts interrupt gpio\n");
        return ft5426->irq_gpio;
    }

    /* 申请使用管脚 */
    ret = devm_gpio_request_one(&client->dev, ft5426->irq_gpio,
                GPIOF_IN, "ft5426 interrupt");
    if (ret < 0)
        return ret;

    /* 注册中断服务函数 */
    ret = devm_request_threaded_irq(&client->dev, gpio_to_irq(ft5426->irq_gpio),
                NULL, edt_ft5426_ts_isr, IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
                client->name, ft5426);
    if (ret) {
        dev_err(&client->dev, "Failed to request touchscreen IRQ.\n");
        return ret;
    }

    return 0;
}

static int edt_ft5426_ts_probe(struct i2c_client *client,
            const struct i2c_device_id *id)
{
    struct edt_ft5426_dev *ft5426;
    struct input_dev *input;
    u8 data;
    int ret;

    /* 实例化一个struct edt_ft5426_dev对象 */
    ft5426 = devm_kzalloc(&client->dev, sizeof(struct edt_ft5426_dev), GFP_KERNEL);
    if (!ft5426) {
        dev_err(&client->dev, "Failed to allocate ft5426 driver data.\n");
        return -ENOMEM;
    }

    ft5426->client = client;

    /* 复位FT5426触摸芯片 */
    ret = edt_ft5426_ts_reset(ft5426);
    if (ret)
        return ret;

    msleep(5);

    /* 初始化FT5426 */
    data = 0;
    edt_ft5426_ts_write(ft5426, FT5426_DEVIDE_MODE_REG, &data, 1);
    data = 1;
    edt_ft5426_ts_write(ft5426, FT5426_ID_G_MODE_REG, &data, 1);

    /* 申请、注册中断服务函数 */
    ret = edt_ft5426_ts_irq(ft5426);
    if (ret)
        return ret;

    /* 注册input设备 */
    input = devm_input_allocate_device(&client->dev);
    if (!input) {
        dev_err(&client->dev, "Failed to allocate input device.\n");
        return -ENOMEM;
    }

    ft5426->input = input;
    input->name = "FocalTech FT5426 TouchScreen";
    input->id.bustype = BUS_I2C;

    input_set_abs_params(input, ABS_MT_POSITION_X,
                0, 1024, 0, 0);
    input_set_abs_params(input, ABS_MT_POSITION_Y,
                0, 600, 0, 0);

    ret = input_mt_init_slots(input, MAX_SUPPORT_POINTS, INPUT_MT_DIRECT);
    if (ret) {
        dev_err(&client->dev, "Failed to init MT slots.\n");
        return ret;
    }

    ret = input_register_device(input);
    if (ret)
        return ret;

    i2c_set_clientdata(client, ft5426);
    return 0;
}

static int edt_ft5426_ts_remove(struct i2c_client *client)
{
    struct edt_ft5426_dev *ft5426 = i2c_get_clientdata(client);
    input_unregister_device(ft5426->input);
    return 0;
}

static const struct of_device_id edt_ft5426_of_match[] = {
    { .compatible = "edt,edt-ft5426", },
    { /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, edt_ft5426_of_match);

static struct i2c_driver edt_ft5426_ts_driver = {
    .driver = {
        .owner     		= THIS_MODULE,
        .name          	= "edt_ft5426",
        .of_match_table	= of_match_ptr(edt_ft5426_of_match),
    },
    .probe    = edt_ft5426_ts_probe,
    .remove   = edt_ft5426_ts_remove,
};

module_i2c_driver(edt_ft5426_ts_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("ALIENTEK");
MODULE_INFO(intree, "Y");
