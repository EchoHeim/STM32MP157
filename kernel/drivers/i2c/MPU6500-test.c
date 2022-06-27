/**********************************************************************************
Copyright (C), by AppleCai
Project                      : Study Kernel
Description                  : BSP level for MPU6500
CPU and Compiler             : AM335x,ARM-LINIX-GCC
|----------------------------------------------------------------------------------
|               R E V I S I O N   H I S T O R Y
|----------------------------------------------------------------------------------
| Date        Version  Author   Description
| --------    -------  ------   ---------------------------------------------------
| 2020-12-08  1.0.0    AppleCai MPU6500_001: Initial release version just add frame
| 2020-12-09  1.0.1    AppleCai MPU6500_002: add WMI reg and I2C DTB
| 2020-12-10  1.0.2    AppleCai MPU6500_003: add Initial reg and report functions
| 2020-12-11  1.0.3    AppleCai MPU6500_004: Optimize code structure
**********************************************************************************/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input-polldev.h>
#include <linux/of_device.h>

#define APPLE6500_DRV_NAME  "apple6500sensor"

#define POLL_INTERVAL           200
#define POLL_INTERVAL_MAX       500

/* bit op */
#define VAL_SIT_BIT(n,addr)        *addr |= (1<<n)
#define VAL_CLEAR_BIT(n,addr)      *addr &= ~(1<<n)
#define VAL_CHANGE_BIT(n,addr)     *addr ^= (1<<n)
#define VAL_TEST_BIT(n,addr)       *addr &= (1<<n)

struct apple6500_chip_reg {
    /* regs */
    char who_am_i;
    char pwr_mgmt_1;
    char pwr_mgmt_2;
    char smplrt_div;
    char sensor_config;
    char accel_config;
    char accel_config2;
    char accel_xout_h;
    char temp_out_h;
    char gyro_xout_h;
    char gyro_config;
    char signal_path_reset;
};
static const struct apple6500_chip_reg chipregs={
    /* reg addr */
    .who_am_i= 117,
    .pwr_mgmt_1= 107,
    .pwr_mgmt_2= 108,
    .smplrt_div= 25,
    .sensor_config= 26,
    .gyro_config= 27,
    .accel_config= 28,
    .accel_config2= 29,
    .accel_xout_h= 59,
    .temp_out_h= 65,
    .gyro_xout_h= 67,
    .signal_path_reset= 104,
};

struct apple6500_chip_config{
    char chip_id;
    char chip_enable_sensor;
    char chip_frequency_acc;
    char chip_frequency_gyro;
    char chip_frequency_DIV;
    char chip_reset;
    char chip_sensitivity_acc;
    char chip_sensitivity_gyro; 
    char chip_selectedclock;
};
struct apple6500;
struct apple6500_transfer_func {
    int (*read) (struct apple6500 *m, unsigned off);
    int (*write) (struct apple6500 *m, unsigned off, unsigned char v);
    int (*readblock)(struct apple6500 *m, unsigned off,unsigned char *buf, size_t size);
};

/* apple6500 status */
struct apple6500 {
    struct device   *dev;
    struct input_polled_dev *idev;
    const struct apple6500_chip_reg *regs;
    struct apple6500_chip_config cfgs;
    const struct apple6500_transfer_func *tf;
};

static int apple6500_read(struct apple6500 *m, unsigned off)
{
    //struct i2c_client *c = m->client;
    struct i2c_client *c = to_i2c_client(m->dev);
    int ret;

    ret = i2c_smbus_read_byte_data(c, off);
    if (ret < 0)
        dev_err(&c->dev,
            "failed to read register 0x%02x, error %d\n",
            off, ret);

    return ret;
}

static int apple6500_write(struct apple6500 *m, unsigned off, unsigned char v)
{
    //struct i2c_client *c = m->client;
    struct i2c_client *c = to_i2c_client(m->dev);
    int error;

    error = i2c_smbus_write_byte_data(c, off, v);
    if (error < 0) {
        dev_err(&c->dev,
            "failed to write to register 0x%02x, error %d\n",
            off, error);
        return error;
    }

    return 0;
}

static int apple6500_read_block(struct apple6500 *m, unsigned off,unsigned char *buf, size_t size)
{
    //struct i2c_client *c = m->client;
    struct i2c_client *c = to_i2c_client(m->dev);
    int err;

    err = i2c_smbus_read_i2c_block_data(c, off, size, buf);
    if (err < 0) {
        dev_err(&c->dev,
            "failed to read block data error %d\n", err);
        return err;
    }

    return 0;
}
static const struct apple6500_transfer_func apple6500_tf_i2c = {
    .write = apple6500_write,
    .read = apple6500_read,
    .readblock = apple6500_read_block
};
static void apple6500_poll(struct input_polled_dev *dev)
{
    struct apple6500 *m = dev->private;

    int ax, ay, az,gx,gy,gz,temp;
    int ret;
    u8 buf[14];

    ret = m->tf->readblock(m, m->regs->accel_xout_h, buf, sizeof(buf));
    if (ret < 0)
    {
        dev_err(m->dev, "polling error\n");
        return;
    }
    ax = ( buf[0] << 8 ) | buf[1];
    ay = ( buf[2] << 8 ) | buf[3];
    az = ( buf[4] << 8 ) | buf[5];
    temp = ( buf[6] << 8 ) | buf[7];
    gx = ( buf[8] << 8 ) | buf[9];
    gy = ( buf[10] << 8 ) | buf[11];
    gz = ( buf[12] << 8 ) | buf[13];

    input_report_abs(dev->input, ABS_X, ax);
    input_report_abs(dev->input, ABS_Y, ay);
    input_report_abs(dev->input, ABS_Z, az);
    input_report_abs(dev->input, ABS_RX, gx);
    input_report_abs(dev->input, ABS_RY, gy);
    input_report_abs(dev->input, ABS_RZ, gz);
    input_report_abs(dev->input, ABS_MISC, temp);//Temperature = 36.53 + regval/340
    input_sync(dev->input);
}
static void apple6500_Init_reg(struct apple6500_chip_config * cfgs)
{
    cfgs->chip_id = 0x70; // id for MPU6500
    cfgs->chip_enable_sensor = 0; // enable all
    cfgs->chip_frequency_acc = 4; // 50Hz/2=25Hz then select 20Hz
    cfgs->chip_frequency_DIV = 19; // 1000/(19+1) = 50Hz clock for acc
    cfgs->chip_frequency_gyro = 4; // 1K 20Hz,due to FCHOICE_B=0
    cfgs->chip_reset = 1<<7; // true 
    cfgs->chip_sensitivity_acc = 0; // 65536/4 = 16384 LBS/s
    cfgs->chip_sensitivity_gyro = 3<<3; // 65536/4000 = 16.384 LBS/s
    cfgs->chip_selectedclock = 1; // PLL with axis 
}
static int apple6500_CheckId(struct apple6500 *m)
{
    int IdNum = 0;
    IdNum = m->tf->read(m, m->regs->who_am_i);
    if(m->cfgs.chip_id != IdNum)
    {
        dev_err(m->dev, "Not found MPU6500\n");
        return -ENXIO;
    }   
    return 0;
}

static int apple6500_ResetChipAndSelectClock(struct apple6500 *m)
{
    int err = 0;
    int val = 0;
    val = m->cfgs.chip_reset+m->cfgs.chip_selectedclock;
    err = m->tf->write(m, m->regs->pwr_mgmt_1,val);
    if(err)
    {
        dev_err(m->dev, "reset command error\n");
    }   
    return err;
}

static int apple6500_EnableSensor(struct apple6500 *m)
{
    int err = 0;
    err = m->tf->write(m, m->regs->pwr_mgmt_2,m->cfgs.chip_enable_sensor);
    if(err)
    {
        dev_err(m->dev, "enable command error\n");
    }   
    return err;
}

static int apple6500_SetACCSamplerate(struct apple6500 *m)
{
    int err = 0;
    err = m->tf->write(m, m->regs->accel_config2,m->cfgs.chip_frequency_acc);
    if(err)
    {
        dev_err(m->dev, "set acc bandwidths error\n");
    }   
    err = m->tf->write(m, m->regs->smplrt_div,m->cfgs.chip_frequency_DIV);
    if(err)
    {
        dev_err(m->dev, "set acc clock division error\n");
    }   
    return err;
}

static int apple6500_SetGyroSamplerate(struct apple6500 *m)
{
    int err = 0;
    err = m->tf->write(m, m->regs->sensor_config,m->cfgs.chip_frequency_gyro);
    if(err)
    {
        dev_err(m->dev, "set gyro bandwidths error\n");
    }   
    return err;
}

static int apple6500_SetAccSensitivity(struct apple6500 *m)
{
    int err = 0;
    err = m->tf->write(m, m->regs->accel_config,m->cfgs.chip_sensitivity_acc);
    if(err)
    {
        dev_err(m->dev, "set acc sensitivity error\n");
    }   
    return err;
}

static int apple6500_SetGyroSensitivity(struct apple6500 *m)
{
    int err = 0;
    err = m->tf->write(m, m->regs->gyro_config,m->cfgs.chip_sensitivity_gyro);
    if(err)
    {
        dev_err(m->dev, "set gyro sensitivity error\n");
    }   
    return err;
}

static int apple6500_Init(struct apple6500 *m)
{
    
    int ret = 0;
    /* load default configuration value */
    apple6500_Init_reg(&m->cfgs);
    /* check id */
    ret = apple6500_CheckId(m);
    /* set reset chip and select clock */
    ret = apple6500_ResetChipAndSelectClock(m);
    /* enable acc and gyro */
    ret = apple6500_EnableSensor(m);
    /* set sample rate and bandwith for acc*/
    ret = apple6500_SetACCSamplerate(m);
    /* set sample rate and bandwith for gyro*/
    ret = apple6500_SetGyroSamplerate(m);   
    /* set sensitivity for acc*/
    ret = apple6500_SetAccSensitivity(m);   
    /* set sensitivity for gyro*/
    ret = apple6500_SetGyroSensitivity(m);
    if (ret < 0)
    {
        dev_err(m->dev,"init error\n");
        return -1;
    }
    return ret;
}

/* Initialize the APPLE6500 chip */
static void apple6500_open(struct input_polled_dev *dev)
{
    int ret = 0;
    struct apple6500 *m = dev->private;
    ret = apple6500_Init(m);
    if (ret < 0)
    {
        dev_err(m->dev, "apple6500 initialization failed\n");
        return;
    }
}

static void apple6500_close(struct input_polled_dev *dev)
{

    //struct apple6500 *m = dev->private;
    //int ret = 0;
    /* sleep */
    //ret = m->tf->write(m, APPLE6500_PWR_REG107,APPLE6500_PWR_SLEEP);
    //if (ret < 0)
    //  return;
}
static const struct of_device_id apple6500_dt_ids[] = {
    { .compatible = "applecai,apple6500", .data = &chipregs},
    { /* sentinel */ }
};
/*
 * I2C init/probing/exit functions
 */
static int apple6500_i2c_probe(struct i2c_client *c,
             const struct i2c_device_id *id)
{
    struct input_polled_dev *idev;
    struct apple6500 *m;
    int err;
    const struct of_device_id *match;
    /* Check basic functionality */
    err = i2c_check_functionality(c->adapter, I2C_FUNC_SMBUS_WRITE_BYTE_DATA|
                                  I2C_FUNC_SMBUS_READ_BYTE_DATA|
                                  I2C_FUNC_SMBUS_READ_I2C_BLOCK);
    if (!err) {
        dev_err(&c->dev, "%s adapter not supported\n",
            dev_driver_string(&c->adapter->dev));
        return -ENODEV;
    }
    
    match = of_match_node(apple6500_dt_ids,c->dev.of_node);
    if (!match)
    {
        dev_err(&c->dev, "No such device or address.\n");
        return -ENXIO;
    }
    m = devm_kzalloc(&c->dev, sizeof(*m), GFP_KERNEL);
    if (!m)
        return -ENOMEM;

    idev = devm_input_allocate_polled_device(&c->dev);
    if (!idev)
        return -ENOMEM;

    //m->client = c;
    m->dev = &c->dev;
    m->idev = idev;
    m->regs = match->data;
    m->tf = &apple6500_tf_i2c;
    i2c_set_clientdata(c, m);
    idev->private       = m;
    idev->input->name   = APPLE6500_DRV_NAME;
    idev->input->id.bustype = BUS_I2C;
    idev->input->id.vendor = 0xABCD;
    idev->input->id.product = 0x0001;
    idev->input->id.version = 0x0100;
    idev->poll      = apple6500_poll;
    idev->poll_interval = POLL_INTERVAL;
    idev->poll_interval_max = POLL_INTERVAL_MAX;
    idev->open      = apple6500_open;
    idev->close     = apple6500_close;

    __set_bit(EV_ABS, idev->input->evbit);
    input_set_abs_params(idev->input, ABS_X, -32768, 32767, 0, 0);
    input_set_abs_params(idev->input, ABS_Y, -32768, 32767, 0, 0);
    input_set_abs_params(idev->input, ABS_Z, -32768, 32767, 0, 0);
    input_set_abs_params(idev->input, ABS_RX, -32768, 32767, 0, 0);
    input_set_abs_params(idev->input, ABS_RY, -32768, 32767, 0, 0);
    input_set_abs_params(idev->input, ABS_RZ, -32768, 32767, 0, 0);
    input_set_abs_params(idev->input, ABS_MISC, -32768, 32767, 0, 0);

    err = input_register_polled_device(idev);
    if (err) {
        input_unregister_polled_device(idev);
        dev_err(&c->dev, "failed to register polled input device\n");
        goto out;
    }
    return 0;
out:
    input_unregister_polled_device(idev);
    return err;
}

static int apple6500_i2c_remove(struct i2c_client *c)
{
    struct apple6500 *m = i2c_get_clientdata(c);
    //disable sensor
    input_unregister_polled_device(m->idev);
    input_free_polled_device(m->idev);
    dev_info(m->dev, "%s: removed\n", APPLE6500_DRV_NAME);
    kfree(m);
    return 0;
}


MODULE_DEVICE_TABLE(of, apple6500_dt_ids);

static struct i2c_driver apple6500_driver = {
    .driver = {
        .name   = APPLE6500_DRV_NAME,
        .of_match_table = apple6500_dt_ids,
    },
    .probe      = apple6500_i2c_probe,
    .remove   = apple6500_i2c_remove,
};

module_i2c_driver(apple6500_driver);

MODULE_AUTHOR("AppleCai");
MODULE_DESCRIPTION("APPLE6500 Acceler and Gyro Driver");
MODULE_LICENSE("GPL");