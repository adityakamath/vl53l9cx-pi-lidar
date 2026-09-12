// SPDX-License-Identifier: GPL-2.0-only

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <media/v4l2-subdev.h>
#include <media/media-entity.h>

/*
 * VL53L9CX XSHUT
 *
 * STEVAL-VL53L9:
 *   MIPI0 connector pin 17 / RP1 GPIO34 (CD0_IO0)
 *       -> VL53L9CX XSHUT
 *
 * Sensor power is always present. GPIO34 controls ONLY XSHUT.
 *
 * Startup policy:
 *
 *   driver probe
 *       -> XSHUT LOW
 *       -> stay LOW
 *
 *   userspace:
 *       echo 1 > /sys/devices/platform/vl53l9/xshut_cycle
 *
 *       -> XSHUT LOW
 *       -> wait 10 ms
 *       -> XSHUT HIGH
 *       -> wait 200 ms for ROM boot
 *       -> return
 *
 *   userspace:
 *       echo 1 > /sys/devices/platform/vl53l9/xshut_state
 *           -> XSHUT HIGH (sensor released)
 *
 *       echo 0 > /sys/devices/platform/vl53l9/xshut_state
 *           -> XSHUT LOW (sensor shutdown)
 *
 * No automatic XSHUT release occurs during probe.
 */

#define VL53L9_XSHUT_HOLD_MS    10
#define VL53L9_BOOT_SETTLE_MS   200

struct dummy_cam {
    struct v4l2_subdev sd;
    struct media_pad pad;
    struct v4l2_mbus_framefmt fmt;

    /* GPIO34 / CD0_IO0 -> VL53L9CX XSHUT */
    struct gpio_desc *xshut;

    /*
     * Serialize XSHUT operations with future sensor I2C/CSI operations.
     */
    struct mutex power_lock;
};


/* ------------------------------------------------------------------------- */
/* V4L2                                                                      */
/* ------------------------------------------------------------------------- */

static int dummy_get_fmt(struct v4l2_subdev *sd,
                         struct v4l2_subdev_state *state,
                         struct v4l2_subdev_format *format)
{
    struct dummy_cam *dev = container_of(sd, struct dummy_cam, sd);

    format->format = dev->fmt;

    return 0;
}

static int dummy_set_fmt(struct v4l2_subdev *sd,
                         struct v4l2_subdev_state *state,
                         struct v4l2_subdev_format *format)
{
    struct dummy_cam *dev = container_of(sd, struct dummy_cam, sd);

    dev->fmt = format->format;

    return 0;
}

static int dummy_s_stream(struct v4l2_subdev *sd, int enable)
{
    struct dummy_cam *dev = container_of(sd, struct dummy_cam, sd);

    /*
     * CSI stream operations must not overlap XSHUT cycling.
     *
     * Future in-kernel VL53L9 I2C operations should use this same lock.
     */
    mutex_lock(&dev->power_lock);

    /* Future: sensor MIPI/I2C start/stop */

    mutex_unlock(&dev->power_lock);

    return 0;
}

static const struct v4l2_subdev_pad_ops dummy_pad_ops = {
    .get_fmt = dummy_get_fmt,
    .set_fmt = dummy_set_fmt,
};

static const struct v4l2_subdev_video_ops dummy_video_ops = {
    .s_stream = dummy_s_stream,
};

static const struct v4l2_subdev_ops dummy_ops = {
    .pad = &dummy_pad_ops,
    .video = &dummy_video_ops,
};


/* ------------------------------------------------------------------------- */
/* XSHUT sysfs control                                                       */
/* ------------------------------------------------------------------------- */

/*
 * Read current XSHUT state.
 *
 *   0 = LOW / sensor shutdown
 *   1 = HIGH / sensor released
 */
static ssize_t xshut_cycle_show(struct device *dev,
                                struct device_attribute *attr,
                                char *buf)
{
    struct dummy_cam *dc = dev_get_drvdata(dev);
    int value;

    if (!dc || !dc->xshut)
        return sysfs_emit(buf, "unavailable\n");

    value = gpiod_get_value_cansleep(dc->xshut);

    return sysfs_emit(buf, "%d\n", value);
}


/*
 * Release/reset the sensor.
 *
 * Userspace:
 *
 *   echo 1 | sudo tee /sys/devices/platform/vl53l9/xshut_cycle
 *
 * Sequence:
 *
 *   XSHUT LOW
 *       |
 *       | 10 ms
 *       v
 *   XSHUT HIGH
 *       |
 *       | 200 ms
 *       v
 *   sensor ROM boot
 *
 * After this returns, userspace can probe I2C address 0x29.
 */
static ssize_t xshut_cycle_store(struct device *dev,
                                 struct device_attribute *attr,
                                 const char *buf,
                                 size_t count)
{
    struct dummy_cam *dc = dev_get_drvdata(dev);
    int val;

    if (!dc || !dc->xshut)
        return -ENODEV;

    if (kstrtoint(buf, 10, &val) || val != 1) {
        dev_err(dev, "xshut_cycle: expected 1\n");
        return -EINVAL;
    }

    mutex_lock(&dc->power_lock);

    /*
     * Force a clean shutdown edge first.
     */
    gpiod_set_value_cansleep(dc->xshut, 0);

    msleep(VL53L9_XSHUT_HOLD_MS);

    /*
     * Release XSHUT.
     * AP_CLK should already be running before this point.
     */
    gpiod_set_value_cansleep(dc->xshut, 1);

    /*
     * Give the sensor ROM time to boot.
     */
    msleep(VL53L9_BOOT_SETTLE_MS);

    dev_info(dev,
             "xshut_cycle: XSHUT HIGH, waited %d ms for boot\n",
             VL53L9_BOOT_SETTLE_MS);

    mutex_unlock(&dc->power_lock);

    return count;
}

static DEVICE_ATTR_RW(xshut_cycle);


/*
 * Read current XSHUT state.
 *
 *   0 = LOW / sensor shutdown
 *   1 = HIGH / sensor released
 */
static ssize_t xshut_state_show(struct device *dev,
                                struct device_attribute *attr,
                                char *buf)
{
    return xshut_cycle_show(dev, attr, buf);
}


/*
 * Drive XSHUT directly to the requested level.
 *
 * Userspace:
 *
 *   echo 1 > /sys/devices/platform/vl53l9/xshut_state
 *       -> XSHUT HIGH (sensor released)
 *
 *   echo 0 > /sys/devices/platform/vl53l9/xshut_state
 *       -> XSHUT LOW (sensor shutdown)
 */
static ssize_t xshut_state_store(struct device *dev,
                                 struct device_attribute *attr,
                                 const char *buf,
                                 size_t count)
{
    struct dummy_cam *dc = dev_get_drvdata(dev);
    int val;

    if (!dc || !dc->xshut)
        return -ENODEV;

    if (kstrtoint(buf, 10, &val)) {
        dev_err(dev, "xshut_state: invalid value\n");
        return -EINVAL;
    }

    val = !!val;

    mutex_lock(&dc->power_lock);

    gpiod_set_value_cansleep(dc->xshut, val);

    dev_info(dev,
             "xshut_state: XSHUT %s\n",
             val ? "HIGH" : "LOW");

    mutex_unlock(&dc->power_lock);

    return count;
}

static DEVICE_ATTR_RW(xshut_state);


/* ------------------------------------------------------------------------- */
/* Probe                                                                     */
/* ------------------------------------------------------------------------- */

static int dummy_probe(struct platform_device *pdev)
{
    struct dummy_cam *dev;
    int ret;

    dev = devm_kzalloc(&pdev->dev, sizeof(*dev), GFP_KERNEL);
    if (!dev)
        return -ENOMEM;

    /*
     * Initialize the lock BEFORE any operation that may use it.
     */
    mutex_init(&dev->power_lock);

    /*
     * GPIO34 / CD0_IO0 -> VL53L9CX XSHUT.
     *
     * GPIOD_OUT_LOW guarantees that the sensor starts in shutdown.
     *
     * IMPORTANT:
     * We deliberately DO NOT raise XSHUT here.
     *
     * It remains LOW until userspace explicitly performs:
     *
     *     echo 1 > xshut_cycle
     */
    dev->xshut = devm_gpiod_get_optional(&pdev->dev,
                                         "xshut",
                                         GPIOD_OUT_LOW);

    if (IS_ERR(dev->xshut)) {
        ret = PTR_ERR(dev->xshut);

        dev_err(&pdev->dev,
                "failed to get xshut-gpios: %d\n",
                ret);

        return ret;
    }

    if (dev->xshut) {
        dev_info(&pdev->dev,
                 "XSHUT claimed and held LOW; "
                 "sensor startup requires xshut_cycle=1\n");
    } else {
        dev_info(&pdev->dev,
                 "no xshut-gpios in DT; XSHUT control unavailable\n");
    }

    /*
     * Store driver data before creating the sysfs attribute.
     */
    platform_set_drvdata(pdev, dev);

    /* ------------------------------------------------------------------ */
    /* V4L2 subdevice                                                     */
    /* ------------------------------------------------------------------ */

    v4l2_subdev_init(&dev->sd, &dummy_ops);

    dev->sd.dev = &pdev->dev;
    dev->sd.owner = THIS_MODULE;

    dev->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;

    snprintf(dev->sd.name,
             sizeof(dev->sd.name),
             "dummy-csi2");

    /*
     * VL53L9CX RAW8 / CSI-2 datatype 0x2A.
     */
    dev->fmt.code = MEDIA_BUS_FMT_Y8_1X8;
    dev->fmt.width = 100;
    dev->fmt.height = 38;
    dev->fmt.field = V4L2_FIELD_NONE;
    dev->fmt.colorspace = V4L2_COLORSPACE_RAW;

    dev->pad.flags = MEDIA_PAD_FL_SOURCE;
    dev->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;

    ret = media_entity_pads_init(&dev->sd.entity,
                                 1,
                                 &dev->pad);
    if (ret)
        return ret;

    ret = v4l2_async_register_subdev(&dev->sd);
    if (ret) {
        media_entity_cleanup(&dev->sd.entity);
        return ret;
    }

    /* ------------------------------------------------------------------ */
    /* Sysfs                                                               */
    /* ------------------------------------------------------------------ */

    ret = device_create_file(&pdev->dev,
                             &dev_attr_xshut_cycle);

    if (ret < 0) {
        dev_err(&pdev->dev,
                "failed to create xshut_cycle sysfs attr: %d\n",
                ret);

        v4l2_async_unregister_subdev(&dev->sd);
        media_entity_cleanup(&dev->sd.entity);

        return ret;
    }

    ret = device_create_file(&pdev->dev,
                             &dev_attr_xshut_state);

    if (ret < 0) {
        dev_err(&pdev->dev,
                "failed to create xshut_state sysfs attr: %d\n",
                ret);

        device_remove_file(&pdev->dev,
                           &dev_attr_xshut_cycle);

        v4l2_async_unregister_subdev(&dev->sd);
        media_entity_cleanup(&dev->sd.entity);

        return ret;
    }

    dev_info(&pdev->dev,
             "VL53L9 dummy CSI sensor registered; "
             "XSHUT initially LOW\n");

    return 0;
}


/* ------------------------------------------------------------------------- */
/* Remove                                                                    */
/* ------------------------------------------------------------------------- */

static void dummy_remove(struct platform_device *pdev)
{
    struct dummy_cam *dev = platform_get_drvdata(pdev);

    if (!dev)
        return;

    /*
     * Prevent userspace from starting another cycle while removing.
     */
    device_remove_file(&pdev->dev,
                       &dev_attr_xshut_state);

    device_remove_file(&pdev->dev,
                       &dev_attr_xshut_cycle);

    v4l2_async_unregister_subdev(&dev->sd);

    media_entity_cleanup(&dev->sd.entity);

    /*
     * Leave the sensor in shutdown when the driver is removed.
     *
     * Do NOT leave XSHUT HIGH.
     */
    if (dev->xshut) {
        mutex_lock(&dev->power_lock);

        gpiod_set_value_cansleep(dev->xshut, 0);

        mutex_unlock(&dev->power_lock);
    }

    dev_info(&pdev->dev,
             "VL53L9 removed; XSHUT held LOW\n");
}


/* ------------------------------------------------------------------------- */
/* Platform driver                                                           */
/* ------------------------------------------------------------------------- */

static const struct of_device_id dummy_dt_ids[] = {
    {
        .compatible = "raspberrypi,dummy-csi2-sensor",
    },
    { }
};

MODULE_DEVICE_TABLE(of, dummy_dt_ids);

static struct platform_driver dummy_driver = {
    .driver = {
        .name = "dummy_cam",
        .of_match_table = dummy_dt_ids,
    },

    .probe = dummy_probe,
    .remove = dummy_remove,
};

module_platform_driver(dummy_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION(
    "VL53L9 MIPI CSI-2 dummy sensor with GPIO XSHUT control");