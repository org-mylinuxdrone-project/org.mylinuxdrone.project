/*
 * mld_pru_cntrl_channel.c
 *
 *  Created on: 07 mag 2019
 *      Author: andrea
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/rpmsg.h>
#include "mylinuxdrone.h"
#include "pru_mylinuxdrone.h"

MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR("Andrea Lambruschini <andrea.lambruschini@gmail.com>");

#define MLD_PRU_CNTRL_CHANNEL_NAME "mld-control"
static int mld_pru_cntrl_channel_probe(struct mylinuxdrone_device *mlddev) {
    printk(KERN_DEBUG "mld_pru_cntrl_channel_probe started...\n");
    // TODO: Aggiornare status: 'Waiting for connection ...'
    return 0;
}
static void mld_pru_cntrl_channel_remove(struct mylinuxdrone_device *mlddev) {
    struct rpmsg_device* rpdev;
    printk(KERN_DEBUG "mld_pru_cntrl_channel_remove: dev:[%s] ...\n", mlddev->id.name);
    rpdev = dev_get_drvdata(&mlddev->dev);
    if(rpdev != NULL) {
        dev_set_drvdata(&rpdev->dev, NULL);
        // this rpmsg channel must remain active; we does not unregister it
        // TODO: Aggiornare status: 'Connected without control'
    }
    printk(KERN_DEBUG "mld_pru_cntrl_channel_remove: device:[%s] removed\n", mlddev->id.name);
}
static void mld_pru_cntrl_channel_dev_release(struct device* dev) {
    struct mylinuxdrone_device *mlddev = to_mylinuxdrone_device(dev);
    printk(KERN_DEBUG "mld_pru_cntrl_channel_dev_release dev:[%s] ...\n", mlddev->id.name);
    put_device(dev);
}
EXPORT_SYMBOL(mld_pru_cntrl_channel_dev_release);

static const struct of_device_id arm_mylinuxdrone_pru_control_matches[] = {
        { .compatible = "arm,mylinuxdrone_pru", .name = MLD_PRU_CNTRL_CHANNEL_NAME},
        {},
};
MODULE_DEVICE_TABLE(mylinuxdrone, arm_mylinuxdrone_pru_control_matches);

static const struct mylinuxdrone_device_id arm_mylinuxdrone_pru_control_id[] = {
        { .name = MLD_PRU_CNTRL_CHANNEL_NAME },
        { },
};
MODULE_DEVICE_TABLE(mylinuxdrone, arm_mylinuxdrone_pru_control_id);

/********************************************************************
 *********************** RPMSG DRIVER SECTION ***********************
 ********************************************************************/
struct pru_control_driver {
    struct mylinuxdrone_driver mlddrv;
    struct rpmsg_driver rpdrv;
};

/* pru_control_id - Structure that holds the channel name for which this driver
   should be probed
 */
static const struct rpmsg_device_id rpmsg_pru_control_id[] = {
        { .name = "pru-control" },
        { },
};
MODULE_DEVICE_TABLE(rpmsg, rpmsg_pru_control_id);

static const struct device_type pru_channel_type = {
        .name           = "pru_channel",
};

/***********************************************************
 * IMU ENABLE/DISABLE ATTRIBUTES
 ***********************************************************/
static int enable_imu(struct mylinuxdrone_device *cntrl) {
    struct rpmsg_device* rpdev;
    unsigned char startMessage[sizeof(PrbMessageType)];
    int ret;
    printk(KERN_DEBUG "enable_imu\n");
    rpdev = dev_get_drvdata(&cntrl->dev);

    ((PrbMessageType*)startMessage)->message_type = MPU_CREATE_CHANNEL_MSG_TYPE;

    ret = rpmsg_send(rpdev->ept, (void *)startMessage, sizeof(PrbMessageType));
    if (ret) {
        dev_err(&cntrl->dev, "Failed sending enable imu message to PRUs\n");
        // TODO: Inviare un allarme su iio status
     }
    printk(KERN_DEBUG "enable_imu: creation of pru_imu device requested.\n");
    // TODO: Aggiornare status: 'Waiting for IMU connection'
    return 0;
}
static int disable_imu(struct mylinuxdrone_device *cntrl) {
    struct rpmsg_device* rpdev;
    unsigned char startMessage[sizeof(PrbMessageType)];
    int ret;
    printk(KERN_DEBUG "disable_imu\n");

    rpdev = dev_get_drvdata(&cntrl->dev);

    ((PrbMessageType*)startMessage)->message_type = MPU_DESTROY_CHANNEL_MSG_TYPE;

    ret = rpmsg_send(rpdev->ept, (void *)startMessage, sizeof(PrbMessageType));
    if (ret) {
        dev_err(&cntrl->dev, "Failed sending disable imu message to PRUs\n");
        // TODO: Inviare un allarme su iio buffer
     }
    printk(KERN_DEBUG "disable_imu: remove of pru_imu device requested.\n");
    // TODO: Aggiornare status: 'Waiting for IMU disconnection'

    return 0;
}

static ssize_t imu_enable_store(struct device *dev,
                            struct device_attribute *attr,
                            const char *buf, size_t len)
{
        struct mylinuxdrone_device *ch = to_mylinuxdrone_device(dev);
        unsigned int enable;
        int ret;

        ret = kstrtouint(buf, 0, &enable);
        if (ret < 0)
                return ret;
        if (enable > 1)
                return -EINVAL;

        if(enable == 1) {
            ret = enable_imu(ch);
        } else {
            ret = disable_imu(ch);
        }
        return ret ? : len;
}
static DEVICE_ATTR_WO(imu_enable);

/***********************************************************
 * RC ENABLE/DISABLE ATTRIBUTES
 ***********************************************************/
static int enable_rc(struct mylinuxdrone_device *cntrl) {
    struct rpmsg_device* rpdev;
    unsigned char startMessage[sizeof(PrbMessageType)];
    int ret;
    printk(KERN_DEBUG "enable_rc\n");
    rpdev = dev_get_drvdata(&cntrl->dev);

    ((PrbMessageType*)startMessage)->message_type = RC_CREATE_CHANNEL_MSG_TYPE;

    ret = rpmsg_send(rpdev->ept, (void *)startMessage, sizeof(PrbMessageType));
    if (ret) {
        dev_err(&cntrl->dev, "Failed sending enable rc message to PRUs\n");
        // TODO: Inviare un allarme su iio status
     }
    printk(KERN_DEBUG "enable_rc: creation of pru_rc device requested.\n");
    // TODO: Aggiornare status: 'Waiting for RC connection'
    return 0;
}
static int disable_rc(struct mylinuxdrone_device *cntrl) {
    struct rpmsg_device* rpdev;
    unsigned char startMessage[sizeof(PrbMessageType)];
    int ret;
    printk(KERN_DEBUG "disable_rc\n");

    rpdev = dev_get_drvdata(&cntrl->dev);

    ((PrbMessageType*)startMessage)->message_type = RC_DESTROY_CHANNEL_MSG_TYPE;

    ret = rpmsg_send(rpdev->ept, (void *)startMessage, sizeof(PrbMessageType));
    if (ret) {
        dev_err(&cntrl->dev, "Failed sending disable rc message to PRUs\n");
        // TODO: Inviare un allarme su iio buffer
     }
    printk(KERN_DEBUG "disable_rc: remove of pru_rc device requested.\n");
    // TODO: Aggiornare status: 'Waiting for RC disconnection'

    return 0;
}

static ssize_t rc_enable_store(struct device *dev,
                            struct device_attribute *attr,
                            const char *buf, size_t len)
{
        struct mylinuxdrone_device *ch = to_mylinuxdrone_device(dev);
        unsigned int enable;
        int ret;

        ret = kstrtouint(buf, 0, &enable);
        if (ret < 0)
                return ret;
        if (enable > 1)
                return -EINVAL;

        if(enable == 1) {
            ret = enable_rc(ch);
        } else {
            ret = disable_rc(ch);
        }
        return ret ? : len;
}
static DEVICE_ATTR_WO(rc_enable);

static struct attribute *pru_control_attrs[] = {
        &dev_attr_imu_enable.attr,
        &dev_attr_rc_enable.attr,
        NULL,
};
ATTRIBUTE_GROUPS(pru_control);

/**
 * pru_control_driver_cb() - function gets invoked each time the pru sends some
 * data.
 */
static int pru_control_driver_cb(struct rpmsg_device *rpdev, void *data,
                  int len, void *priv, u32 src)
{
    printk(KERN_DEBUG "pru_control_driver_cb [%s].\n", rpdev->id.name);
    // TODO: da definire ... allarmi, segnali, etc
    // Forse serve un iio buffer per comunicare l'allarme
    return 0;
}

/**
 * pru_control_driver_probe() - function gets invoked when the rpmsg channel
 * as mentioned in the pru_control_id table
 */
static int pru_control_driver_probe(struct rpmsg_device *rpdev)
{
    int ret;
    struct mylinuxdrone_device *st;
    st = alloc_mylinuxdrone_device(MLD_PRU_CNTRL_CHANNEL_NAME, 0);
    // Aggiornare lo status: 'Connecting ...'

    printk(KERN_DEBUG "pru_control_driver_probe [%s].\n", rpdev->id.name);
    // FIXME: verificare se device già creato.
    printk(KERN_DEBUG "pru_control_driver_probe device created.\n");

    printk(KERN_DEBUG "pru_control_driver_probe allocated memory.\n");
    st->dev.type = &pru_channel_type;
    st->dev.devt = MKDEV(0, 0);
    st->dev.groups = pru_control_groups;
    st->dev.release = mld_pru_cntrl_channel_dev_release;
    printk(KERN_DEBUG "pru_control_driver_probe pru_control device prepared.\n");

    /* devices's circular reference
       must be set before register_mylinuxdrone_device to avoid
       a loop that create a new rpmsg_channel when register mylinuxdrone device
     */
    dev_set_drvdata(&rpdev->dev, st);
    dev_set_drvdata(&st->dev, rpdev);

    // registra il nuovo device
    printk(KERN_DEBUG "pru_control_driver_probe registering pru_control device... \n");
    ret = register_mylinuxdrone_device(st);
    if (ret) {
       printk(KERN_ERR "pru_control_driver_probe pru_control device registration failed.\n");
       // TODO: Inviare un allarme su iio buffer
       kfree(st);
       return ret;
    }
    printk(KERN_DEBUG "pru_control_driver_probe pru_control device registered \n");

    // Aggiornare lo status: 'Connected'

    return 0;
}

/**
 * pru_control_driver_remove() - function gets invoked when the rpmsg device is
 * removed
 */
static void pru_control_driver_remove(struct rpmsg_device *rpdev)
{
    struct mylinuxdrone_device *cntr;
    printk(KERN_DEBUG "pru_control_driver_remove.\n");
    cntr = dev_get_drvdata(&rpdev->dev);
    if(cntr != NULL) {
        cntr->rpdev = NULL; // remove reference to rpdev
        dev_set_drvdata(&cntr->dev, NULL);
        unregister_mylinuxdrone_device(cntr);
    }
    // TODO: Aggiornare lo status: 'Disconnected'
}

static struct pru_control_driver mld_pru_cntrl_channel_driver = {
        .mlddrv = {
                       .probe = mld_pru_cntrl_channel_probe,
                       .remove = mld_pru_cntrl_channel_remove,
                       .id_table = arm_mylinuxdrone_pru_control_id,
                       .driver = {
                               .name = MLD_PRU_CNTRL_CHANNEL_NAME,
                               .of_match_table = arm_mylinuxdrone_pru_control_matches,
                               .owner = THIS_MODULE,
                       },
        },
        .rpdrv = {
                  .drv.name   = KBUILD_MODNAME,
                  .drv.owner  = THIS_MODULE,
                  .id_table   = rpmsg_pru_control_id,
                  .probe      = pru_control_driver_probe,
                  .callback   = pru_control_driver_cb,
                  .remove     = pru_control_driver_remove,

        }
};

static int __init mld_pru_cntrl_channel_init(void)
{
    int ret;
    printk(KERN_DEBUG "mld_pru_cntrl_channel_init...\n");

    printk(KERN_DEBUG "mld_pru_cntrl_channel_init: registering mld_pru_cntrl_channel_driver ...\n");
    ret = register_mylinuxdrone_driver(&mld_pru_cntrl_channel_driver.mlddrv);
    if(ret) {
        pr_err("failed to register mld_pru_cntrl_channel_driver: %d\n", ret);
        return ret;
    }
    printk(KERN_DEBUG "mld_pru_cntrl_channel_init: mld_pru_cntrl_channel_driver registered ...\n");

    printk(KERN_DEBUG "mld_pru_cntrl_channel_init: registering rpmsg driver ...\n");
    ret = register_rpmsg_driver(&mld_pru_cntrl_channel_driver.rpdrv);
    if(ret) {
        pr_err("failed to register rpmsg driver: %d\n", ret);
        return ret;
    }
    printk(KERN_DEBUG "mld_pru_cntrl_channel_init: rpmsg driver registered ...\n");
    printk(KERN_DEBUG "mld_pru_cntrl_channel_init completed\n");
    return 0;
}

static void __exit mld_pru_cntrl_channel_fini(void)
{
    printk(KERN_DEBUG "mld_pru_cntrl_channel_fini started ...\n");
    unregister_mylinuxdrone_driver(&mld_pru_cntrl_channel_driver.mlddrv);
    unregister_rpmsg_driver (&mld_pru_cntrl_channel_driver.rpdrv);

    printk(KERN_DEBUG "mld_pru_cntrl_channel_fini ...\n");
}

postcore_initcall(mld_pru_cntrl_channel_init);
module_exit(mld_pru_cntrl_channel_fini);
