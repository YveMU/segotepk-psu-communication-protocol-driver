// SPDX-License-Identifier: GPL-2.0
/*
 * Segotep PSU Hardware Monitoring Driver
 * 
 * Copyright (C) 2024
 * 
 * This driver interfaces with Segotep PSU via USB-Serial (CH340)
 * and exposes monitoring data through hwmon sysfs interface.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/serial.h>
#include <linux/jiffies.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/uaccess.h>

#define DRIVER_NAME "segotep_psu"
#define SERIAL_PORT "/dev/ttyUSB0"
#define UPDATE_INTERVAL_MS 1000

/* Protocol constants */
#define HEADER1 0x55
#define HEADER2 0x7E
#define FOOTER  0xAE

/* Packet types */
#define PKT_TYPE_ELECTRICAL 0x02
#define PKT_TYPE_STATUS     0x04
#define PKT_TYPE_MODEL      0x03
#define PKT_TYPE_SERIAL     0x05

struct segotep_psu_data {
    struct device *hwmon_dev;
    struct mutex update_lock;
    struct delayed_work update_work;
    struct file *tty_file;
    
    unsigned long last_updated;
    bool valid;
    
    /* Electrical data (packet 0x02) */
    u16 volt_3v3;    /* mV */
    u16 volt_5v;     /* mV */
    u16 volt_12v;    /* mV */
    u16 volt_5vsb;   /* mV */
    u16 curr_3v3;    /* mA raw */
    u16 curr_5v;     /* mA raw */
    u16 curr_12v;    /* mA raw */
    u16 ac_voltage;  /* 0.1V */
    u16 ac_freq;     /* 0.1Hz */
    u16 fan_rpm_raw; /* needs *30 */
    
    /* Status data (packet 0x04) */
    u8  fan_mode;
    u16 temp_main;   /* 0.1°C */
    u16 temp_secondary; /* 0.1°C */
    u16 temp_air;    /* 0.01°C */
    u16 ac_power;    /* 0.1W */
    
    /* Calculated values */
    u32 power_3v3;   /* mW */
    u32 power_5v;    /* mW */
    u32 power_12v;   /* mW */
    u32 power_total; /* mW */
    u32 efficiency;  /* 0.1% */
    
    char model[32];
    char serial[32];
};

static struct segotep_psu_data *g_psu_data;

/* Forward declarations for label functions */
static ssize_t show_label_in(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t show_label_curr(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t show_label_power(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t show_label_temp(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t show_label_fan(struct device *dev, struct device_attribute *attr, char *buf);

/* Read data from serial port */
static int segotep_read_serial(struct segotep_psu_data *data, u8 *buf, size_t len)
{
    loff_t pos = 0;
    ssize_t ret;
    
    if (!data->tty_file)
        return -ENODEV;
    
    ret = kernel_read(data->tty_file, buf, len, &pos);
    return ret;
}

/* Parse electrical packet (0x02) */
static void parse_electrical_packet(struct segotep_psu_data *data, u8 *payload, int len)
{
    u16 *values;
    u32 p_3v3, p_5v, p_12v;
    
    if (len < 27)
        return;
    
    /* Skip packet type byte, parse 13 uint16 values (little-endian) */
    values = (u16 *)(payload + 1);
    
    data->volt_3v3 = le16_to_cpu(values[0]);
    data->volt_5v = le16_to_cpu(values[1]);
    data->volt_12v = le16_to_cpu(values[2]);
    data->volt_5vsb = le16_to_cpu(values[3]);
    data->curr_3v3 = le16_to_cpu(values[4]);
    data->curr_5v = le16_to_cpu(values[5]);
    data->curr_12v = le16_to_cpu(values[6]);
    data->ac_freq = le16_to_cpu(values[7]);
    data->ac_voltage = le16_to_cpu(values[11]);
    data->fan_rpm_raw = le16_to_cpu(values[12]);
    
    /* Calculate power in mW */
    p_3v3 = (data->volt_3v3 * data->curr_3v3) / 4096;
    p_5v = (data->volt_5v * data->curr_5v) / 4096;
    p_12v = (data->volt_12v * data->curr_12v) / 256;
    
    data->power_3v3 = p_3v3;
    data->power_5v = p_5v;
    data->power_12v = p_12v;
    data->power_total = p_3v3 + p_5v + p_12v;
}

/* Parse status packet (0x04) */
static void parse_status_packet(struct segotep_psu_data *data, u8 *payload, int len)
{
    u16 *values;
    
    if (len < 28)
        return;
    
    data->fan_mode = payload[1];
    
    /* Parse uint16 values (big-endian!) */
    values = (u16 *)(payload + 2);
    
    data->temp_main = be16_to_cpu(values[0]);
    if (len >= 14)
        data->temp_secondary = be16_to_cpu(values[5]);
    if (len >= 18)
        data->ac_power = be16_to_cpu(values[6]);
    if (len >= 26)
        data->temp_air = be16_to_cpu(values[11]);
    
    /* Calculate efficiency */
    if (data->ac_power > 0 && data->power_total > 0) {
        data->efficiency = (data->power_total * 1000) / (data->ac_power * 100);
    }
}

/* Process incoming serial data */
static void process_serial_data(struct segotep_psu_data *data)
{
    u8 buffer[256];
    int ret, i;
    u8 pkt_len, pkt_type;
    u8 *payload;
    
    ret = segotep_read_serial(data, buffer, sizeof(buffer));
    if (ret <= 0)
        return;
    
    /* Find packet headers */
    for (i = 0; i < ret - 3; i++) {
        if (buffer[i] == HEADER1 && buffer[i+1] == HEADER2) {
            pkt_len = buffer[i+2];
            
            if (i + 3 + pkt_len <= ret) {
                payload = &buffer[i+3];
                pkt_type = payload[0];
                
                mutex_lock(&data->update_lock);
                
                switch (pkt_type) {
                case PKT_TYPE_ELECTRICAL:
                    parse_electrical_packet(data, payload, pkt_len);
                    data->valid = true;
                    data->last_updated = jiffies;
                    break;
                case PKT_TYPE_STATUS:
                    parse_status_packet(data, payload, pkt_len);
                    break;
                case PKT_TYPE_MODEL:
                    if (pkt_len > 2 && pkt_len < 30) {
                        memcpy(data->model, payload + 1, pkt_len - 2);
                        data->model[pkt_len - 2] = '\0';
                    }
                    break;
                case PKT_TYPE_SERIAL:
                    if (pkt_len > 2 && pkt_len < 30) {
                        memcpy(data->serial, payload + 1, pkt_len - 2);
                        data->serial[pkt_len - 2] = '\0';
                    }
                    break;
                }
                
                mutex_unlock(&data->update_lock);
            }
        }
    }
}

/* Work function for periodic updates */
static void segotep_update_work(struct work_struct *work)
{
    struct segotep_psu_data *data = container_of(work, 
                                                  struct segotep_psu_data, 
                                                  update_work.work);
    
    process_serial_data(data);
    
    /* Reschedule */
    schedule_delayed_work(&data->update_work, msecs_to_jiffies(UPDATE_INTERVAL_MS));
}

/* hwmon voltage read functions */
static ssize_t show_in(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct sensor_device_attribute *sattr = to_sensor_dev_attr(attr);
    struct segotep_psu_data *data = dev_get_drvdata(dev);
    u16 val = 0;
    
    mutex_lock(&data->update_lock);
    
    switch (sattr->index) {
    case 0: val = data->volt_3v3; break;
    case 1: val = data->volt_5v; break;
    case 2: val = data->volt_12v; break;
    case 3: val = data->volt_5vsb; break;
    case 4: val = data->ac_voltage * 100; break; /* Convert to mV */
    }
    
    mutex_unlock(&data->update_lock);
    
    return sprintf(buf, "%u\n", val);
}

/* hwmon current read functions */
static ssize_t show_curr(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct sensor_device_attribute *sattr = to_sensor_dev_attr(attr);
    struct segotep_psu_data *data = dev_get_drvdata(dev);
    u32 val = 0;
    
    mutex_lock(&data->update_lock);
    
    switch (sattr->index) {
    case 0: val = (data->curr_3v3 * 1000) / 4096; break; /* Convert to mA */
    case 1: val = (data->curr_5v * 1000) / 4096; break;
    case 2: val = (data->curr_12v * 1000) / 256; break;
    }
    
    mutex_unlock(&data->update_lock);
    
    return sprintf(buf, "%u\n", val);
}

/* hwmon power read functions */
static ssize_t show_power(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct sensor_device_attribute *sattr = to_sensor_dev_attr(attr);
    struct segotep_psu_data *data = dev_get_drvdata(dev);
    u32 val = 0;
    
    mutex_lock(&data->update_lock);
    
    switch (sattr->index) {
    case 0: val = data->power_3v3 * 1000; break; /* Convert to uW */
    case 1: val = data->power_5v * 1000; break;
    case 2: val = data->power_12v * 1000; break;
    case 3: val = data->power_total * 1000; break;
    case 4: val = data->ac_power * 100000; break; /* 0.1W to uW */
    }
    
    mutex_unlock(&data->update_lock);
    
    return sprintf(buf, "%u\n", val);
}

/* hwmon temperature read functions */
static ssize_t show_temp(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct sensor_device_attribute *sattr = to_sensor_dev_attr(attr);
    struct segotep_psu_data *data = dev_get_drvdata(dev);
    int val = 0;
    
    mutex_lock(&data->update_lock);
    
    switch (sattr->index) {
    case 0: val = data->temp_main * 100; break; /* 0.1C to mC */
    case 1: val = data->temp_secondary * 100; break;
    case 2: val = data->temp_air * 10; break; /* 0.01C to mC */
    }
    
    mutex_unlock(&data->update_lock);
    
    return sprintf(buf, "%d\n", val);
}

/* hwmon fan read function */
static ssize_t show_fan(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct segotep_psu_data *data = dev_get_drvdata(dev);
    u32 rpm;
    
    mutex_lock(&data->update_lock);
    rpm = data->fan_rpm_raw * 30;
    mutex_unlock(&data->update_lock);
    
    return sprintf(buf, "%u\n", rpm);
}

/* Label functions */
static ssize_t show_label_in(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct sensor_device_attribute *sattr = to_sensor_dev_attr(attr);
    const char *labels[] = { "+3.3V", "+5V", "+12V", "+5VSB", "AC Input" };
    
    return sprintf(buf, "%s\n", labels[sattr->index]);
}

static ssize_t show_label_curr(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct sensor_device_attribute *sattr = to_sensor_dev_attr(attr);
    const char *labels[] = { "+3.3V", "+5V", "+12V" };
    
    return sprintf(buf, "%s\n", labels[sattr->index]);
}

static ssize_t show_label_power(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct sensor_device_attribute *sattr = to_sensor_dev_attr(attr);
    const char *labels[] = { "+3.3V", "+5V", "+12V", "DC Total", "AC Input" };
    
    return sprintf(buf, "%s\n", labels[sattr->index]);
}

static ssize_t show_label_temp(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct sensor_device_attribute *sattr = to_sensor_dev_attr(attr);
    const char *labels[] = { "Main", "Secondary", "Air" };
    
    return sprintf(buf, "%s\n", labels[sattr->index]);
}

static ssize_t show_label_fan(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "PSU Fan\n");
}

/* Efficiency attribute */
static ssize_t show_efficiency(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct segotep_psu_data *data = dev_get_drvdata(dev);
    u32 eff;
    
    mutex_lock(&data->update_lock);
    eff = data->efficiency;
    mutex_unlock(&data->update_lock);
    
    return sprintf(buf, "%u.%u\n", eff / 10, eff % 10);
}

static DEVICE_ATTR(efficiency, S_IRUGO, show_efficiency, NULL);

/* Model and serial attributes */
static ssize_t show_model(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct segotep_psu_data *data = dev_get_drvdata(dev);
    return sprintf(buf, "%s\n", data->model);
}

static ssize_t show_serial(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct segotep_psu_data *data = dev_get_drvdata(dev);
    return sprintf(buf, "%s\n", data->serial);
}

static DEVICE_ATTR(model, S_IRUGO, show_model, NULL);
static DEVICE_ATTR(serial_number, S_IRUGO, show_serial, NULL);

/* Sensor attributes */
static SENSOR_DEVICE_ATTR(in0_input, S_IRUGO, show_in, NULL, 0);
static SENSOR_DEVICE_ATTR(in1_input, S_IRUGO, show_in, NULL, 1);
static SENSOR_DEVICE_ATTR(in2_input, S_IRUGO, show_in, NULL, 2);
static SENSOR_DEVICE_ATTR(in3_input, S_IRUGO, show_in, NULL, 3);
static SENSOR_DEVICE_ATTR(in4_input, S_IRUGO, show_in, NULL, 4);

static SENSOR_DEVICE_ATTR(in0_label, S_IRUGO, show_label_in, NULL, 0);
static SENSOR_DEVICE_ATTR(in1_label, S_IRUGO, show_label_in, NULL, 1);
static SENSOR_DEVICE_ATTR(in2_label, S_IRUGO, show_label_in, NULL, 2);
static SENSOR_DEVICE_ATTR(in3_label, S_IRUGO, show_label_in, NULL, 3);
static SENSOR_DEVICE_ATTR(in4_label, S_IRUGO, show_label_in, NULL, 4);

static SENSOR_DEVICE_ATTR(curr1_input, S_IRUGO, show_curr, NULL, 0);
static SENSOR_DEVICE_ATTR(curr2_input, S_IRUGO, show_curr, NULL, 1);
static SENSOR_DEVICE_ATTR(curr3_input, S_IRUGO, show_curr, NULL, 2);

static SENSOR_DEVICE_ATTR(curr1_label, S_IRUGO, show_label_curr, NULL, 0);
static SENSOR_DEVICE_ATTR(curr2_label, S_IRUGO, show_label_curr, NULL, 1);
static SENSOR_DEVICE_ATTR(curr3_label, S_IRUGO, show_label_curr, NULL, 2);

static SENSOR_DEVICE_ATTR(power1_input, S_IRUGO, show_power, NULL, 0);
static SENSOR_DEVICE_ATTR(power2_input, S_IRUGO, show_power, NULL, 1);
static SENSOR_DEVICE_ATTR(power3_input, S_IRUGO, show_power, NULL, 2);
static SENSOR_DEVICE_ATTR(power4_input, S_IRUGO, show_power, NULL, 3);
static SENSOR_DEVICE_ATTR(power5_input, S_IRUGO, show_power, NULL, 4);

static SENSOR_DEVICE_ATTR(power1_label, S_IRUGO, show_label_power, NULL, 0);
static SENSOR_DEVICE_ATTR(power2_label, S_IRUGO, show_label_power, NULL, 1);
static SENSOR_DEVICE_ATTR(power3_label, S_IRUGO, show_label_power, NULL, 2);
static SENSOR_DEVICE_ATTR(power4_label, S_IRUGO, show_label_power, NULL, 3);
static SENSOR_DEVICE_ATTR(power5_label, S_IRUGO, show_label_power, NULL, 4);

static SENSOR_DEVICE_ATTR(temp1_input, S_IRUGO, show_temp, NULL, 0);
static SENSOR_DEVICE_ATTR(temp2_input, S_IRUGO, show_temp, NULL, 1);
static SENSOR_DEVICE_ATTR(temp3_input, S_IRUGO, show_temp, NULL, 2);

static SENSOR_DEVICE_ATTR(temp1_label, S_IRUGO, show_label_temp, NULL, 0);
static SENSOR_DEVICE_ATTR(temp2_label, S_IRUGO, show_label_temp, NULL, 1);
static SENSOR_DEVICE_ATTR(temp3_label, S_IRUGO, show_label_temp, NULL, 2);

static SENSOR_DEVICE_ATTR(fan1_input, S_IRUGO, show_fan, NULL, 0);
static SENSOR_DEVICE_ATTR(fan1_label, S_IRUGO, show_label_fan, NULL, 0);

static struct attribute *segotep_attrs[] = {
    &sensor_dev_attr_in0_input.dev_attr.attr,
    &sensor_dev_attr_in1_input.dev_attr.attr,
    &sensor_dev_attr_in2_input.dev_attr.attr,
    &sensor_dev_attr_in3_input.dev_attr.attr,
    &sensor_dev_attr_in4_input.dev_attr.attr,
    
    &sensor_dev_attr_in0_label.dev_attr.attr,
    &sensor_dev_attr_in1_label.dev_attr.attr,
    &sensor_dev_attr_in2_label.dev_attr.attr,
    &sensor_dev_attr_in3_label.dev_attr.attr,
    &sensor_dev_attr_in4_label.dev_attr.attr,
    
    &sensor_dev_attr_curr1_input.dev_attr.attr,
    &sensor_dev_attr_curr2_input.dev_attr.attr,
    &sensor_dev_attr_curr3_input.dev_attr.attr,
    
    &sensor_dev_attr_curr1_label.dev_attr.attr,
    &sensor_dev_attr_curr2_label.dev_attr.attr,
    &sensor_dev_attr_curr3_label.dev_attr.attr,
    
    &sensor_dev_attr_power1_input.dev_attr.attr,
    &sensor_dev_attr_power2_input.dev_attr.attr,
    &sensor_dev_attr_power3_input.dev_attr.attr,
    &sensor_dev_attr_power4_input.dev_attr.attr,
    &sensor_dev_attr_power5_input.dev_attr.attr,
    
    &sensor_dev_attr_power1_label.dev_attr.attr,
    &sensor_dev_attr_power2_label.dev_attr.attr,
    &sensor_dev_attr_power3_label.dev_attr.attr,
    &sensor_dev_attr_power4_label.dev_attr.attr,
    &sensor_dev_attr_power5_label.dev_attr.attr,
    
    &sensor_dev_attr_temp1_input.dev_attr.attr,
    &sensor_dev_attr_temp2_input.dev_attr.attr,
    &sensor_dev_attr_temp3_input.dev_attr.attr,
    
    &sensor_dev_attr_temp1_label.dev_attr.attr,
    &sensor_dev_attr_temp2_label.dev_attr.attr,
    &sensor_dev_attr_temp3_label.dev_attr.attr,
    
    &sensor_dev_attr_fan1_input.dev_attr.attr,
    &sensor_dev_attr_fan1_label.dev_attr.attr,
    
    &dev_attr_efficiency.attr,
    &dev_attr_model.attr,
    &dev_attr_serial_number.attr,
    
    NULL
};

ATTRIBUTE_GROUPS(segotep);

/* Configure serial port without using deprecated set_fs/get_fs */
static int configure_serial_port(struct file *file)
{
    struct ktermios settings;
    struct tty_struct *tty;
    
    /* Get tty structure */
    tty = ((struct tty_file_private *)file->private_data)->tty;
    if (!tty || !tty->ops->set_termios)
        return -EINVAL;
    
    /* Get current settings */
    down_read(&tty->termios_rwsem);
    settings = tty->termios;
    up_read(&tty->termios_rwsem);
    
    /* Configure for 115200 8N1 */
    settings.c_cflag &= ~CBAUD;
    settings.c_cflag |= B115200;
    settings.c_cflag &= ~CSIZE;
    settings.c_cflag |= CS8;
    settings.c_cflag &= ~PARENB;
    settings.c_cflag &= ~CSTOPB;
    settings.c_cflag |= CLOCAL | CREAD;
    
    settings.c_iflag = IGNPAR;
    settings.c_oflag = 0;
    settings.c_lflag = 0;
    
    settings.c_cc[VMIN] = 0;
    settings.c_cc[VTIME] = 0;
    
    /* Apply new settings */
    tty->ops->set_termios(tty, &settings);
    
    return 0;
}

static int segotep_psu_probe(struct platform_device *pdev)
{
    struct segotep_psu_data *data;
    int err;
    
    data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
    if (!data)
        return -ENOMEM;
    
    mutex_init(&data->update_lock);
    platform_set_drvdata(pdev, data);
    g_psu_data = data;
    
    /* Open serial port */
    data->tty_file = filp_open(SERIAL_PORT, O_RDWR | O_NOCTTY | O_NONBLOCK, 0);
    if (IS_ERR(data->tty_file)) {
        dev_err(&pdev->dev, "Failed to open serial port %s: %ld\n", 
                SERIAL_PORT, PTR_ERR(data->tty_file));
        dev_info(&pdev->dev, "Make sure the CH340 driver is loaded and device is connected\n");
        return PTR_ERR(data->tty_file);
    }
    
    /* Configure serial port */
    err = configure_serial_port(data->tty_file);
    if (err) {
        dev_warn(&pdev->dev, "Failed to configure serial port, using defaults\n");
    }
    
    /* Register with hwmon */
    data->hwmon_dev = devm_hwmon_device_register_with_groups(&pdev->dev,
                                                               "segotep_psu",
                                                               data,
                                                               segotep_groups);
    
    if (IS_ERR(data->hwmon_dev)) {
        err = PTR_ERR(data->hwmon_dev);
        dev_err(&pdev->dev, "Failed to register hwmon device: %d\n", err);
        filp_close(data->tty_file, NULL);
        return err;
    }
    
    /* Start periodic update work */
    INIT_DELAYED_WORK(&data->update_work, segotep_update_work);
    schedule_delayed_work(&data->update_work, msecs_to_jiffies(UPDATE_INTERVAL_MS));
    
    dev_info(&pdev->dev, "Segotep PSU monitor initialized on %s\n", SERIAL_PORT);
    return 0;
}

static int segotep_psu_remove(struct platform_device *pdev)
{
    struct segotep_psu_data *data = platform_get_drvdata(pdev);
    
    cancel_delayed_work_sync(&data->update_work);
    
    if (data->tty_file)
        filp_close(data->tty_file, NULL);
    
    g_psu_data = NULL;
    
    return 0;
}

static struct platform_driver segotep_psu_driver = {
    .driver = {
        .name = DRIVER_NAME,
        .owner = THIS_MODULE,
    },
    .probe = segotep_psu_probe,
    .remove = segotep_psu_remove,
};

static struct platform_device *segotep_psu_device;

static int __init segotep_psu_init(void)
{
    int err;
    
    err = platform_driver_register(&segotep_psu_driver);
    if (err)
        return err;
    
    segotep_psu_device = platform_device_register_simple(DRIVER_NAME, -1, NULL, 0);
    if (IS_ERR(segotep_psu_device)) {
        platform_driver_unregister(&segotep_psu_driver);
        return PTR_ERR(segotep_psu_device);
    }
    
    return 0;
}

static void __exit segotep_psu_exit(void)
{
    platform_device_unregister(segotep_psu_device);
    platform_driver_unregister(&segotep_psu_driver);
}

module_init(segotep_psu_init);
module_exit(segotep_psu_exit);


MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Your Name");
MODULE_DESCRIPTION("Segotep PSU Hardware Monitoring Driver");
MODULE_ALIAS("platform:" DRIVER_NAME);
