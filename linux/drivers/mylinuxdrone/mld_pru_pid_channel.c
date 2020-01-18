/*
 * mld_pru_pid_channel.c
 *  Created on: 18 gen 2020
 *      Author: Andrea Lambruschini <andrea.lambruschini@gmail.com>
 *
 * Copyright 2020 Andrea Lambruschini
 *
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/rpmsg.h>
#include <linux/device.h>
#include "mld_pru_pid_channel.h"
#include "mld_messages.h"

MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR("Andrea Lambruschini <andrea.lambruschini@gmail.com>");

#define MLD_PRU_PID_CHANNEL_NAME "mld-pid"

/********************************************************************
 **************** MLD_PRU_PID_CHANNEL DRIVER SECTION *****************
 ********************************************************************/
static int mld_pru_pid_channel_probe(struct mylinuxdrone_device *mlddev) {
    printk(KERN_DEBUG "mld_pru_pid_channel_probe started...\n");
    return 0;
}
static void mld_pru_pid_channel_remove(struct mylinuxdrone_device *mlddev) {
    struct rpmsg_device* rpdev;
    printk(KERN_DEBUG "mld_pru_pid_channel_remove: dev:[%s] ...\n", mlddev->id.name);
    rpdev = dev_get_drvdata(&mlddev->dev);
    if(rpdev != NULL) {
        dev_set_drvdata(&rpdev->dev, NULL);
    }
    printk(KERN_DEBUG "mld_pru_pid_channel_remove: device:[%s] removed\n", mlddev->id.name);
}

static void mld_pru_pid_channel_dev_release(struct device* dev) {
    struct mylinuxdrone_device *mlddev = to_mylinuxdrone_device(dev);

    printk(KERN_DEBUG "mld_pru_pid_channel_dev_release dev:[%s] ...\n", mlddev->id.name);
    // TODO: kfree?
}
EXPORT_SYMBOL(mld_pru_pid_channel_dev_release);

static const struct of_device_id arm_mylinuxdrone_pru_pid_matches[] = {
        { .compatible = "arm,mylinuxdrone_pru", .name = MLD_PRU_PID_CHANNEL_NAME},
        {},
};
MODULE_DEVICE_TABLE(mylinuxdrone, arm_mylinuxdrone_pru_pid_matches);

static const struct mylinuxdrone_device_id arm_mylinuxdrone_pru_pid_id[] = {
        { .name = MLD_PRU_PID_CHANNEL_NAME },
        { },
};
MODULE_DEVICE_TABLE(mylinuxdrone, arm_mylinuxdrone_pru_pid_id);

static int pid_start(struct mylinuxdrone_device *cntrl) {
    struct mld_pid_device* pidd = to_mld_pid_device(cntrl);
    struct rpmsg_device* rpdev;
    unsigned char startMessage[sizeof(PrbMessageType)];
    int ret;
    printk(KERN_DEBUG "pid_start\n");
    rpdev = dev_get_drvdata(&cntrl->dev);

    ((PrbMessageType*)startMessage)->message_type = PID_ENABLE_MSG_TYPE;

    ret = rpmsg_send(rpdev->ept, (void *)startMessage, sizeof(PrbMessageType));
    if (ret) {
        dev_err(&cntrl->dev, "Failed sending start pid message to PRUs\n");
        // TODO: Inviare un allarme su iio status
     }
    printk(KERN_DEBUG "pid_start: creation of pru_pid device requested.\n");

    // Status: 'Started'
    pidd->status.status = MLD_PID_DEVICE_STATUS_STARTED;
    return 0;
}
static int pid_stop(struct mylinuxdrone_device *cntrl) {
    struct mld_pid_device* pidd = to_mld_pid_device(cntrl);
    struct rpmsg_device* rpdev;
    unsigned char startMessage[sizeof(PrbMessageType)];
    int ret;
    printk(KERN_DEBUG "pid_stop\n");

    rpdev = dev_get_drvdata(&cntrl->dev);

    ((PrbMessageType*)startMessage)->message_type = PID_DISABLE_MSG_TYPE;

    ret = rpmsg_send(rpdev->ept, (void *)startMessage, sizeof(PrbMessageType));
    if (ret) {
        dev_err(&cntrl->dev, "Failed sending stop pid message to PRUs\n");
        // TODO: Inviare un allarme su iio buffer
     }
    printk(KERN_DEBUG "pid_stop: stop of pru_pid device requested.\n");

    // Status: 'Stopped'
    pidd->status.status = MLD_PID_DEVICE_STATUS_STOPPED;
    return 0;
}

static ssize_t pid_start_store(struct device *dev,
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
            pid_start(ch);
            printk(KERN_INFO "pru_pid_store: started.\n");
        } else {
            pid_stop(ch);
            printk(KERN_INFO "pru_pid_store: stopped.\n");
        }
        return ret ? : len;
}
static DEVICE_ATTR_WO(pid_start);

static int pid_start_calibration(struct mylinuxdrone_device *cntrl) {
    struct mld_pid_device* pidd = to_mld_pid_device(cntrl);
    struct rpmsg_device* rpdev;
    unsigned char startMessage[sizeof(PrbMessageType)];
    int ret;
    printk(KERN_DEBUG "pid_start_calibration\n");
    rpdev = dev_get_drvdata(&cntrl->dev);

    ((PrbMessageType*)startMessage)->message_type = PID_CALIBRATION_ENABLE_MSG_TYPE;

    ret = rpmsg_send(rpdev->ept, (void *)startMessage, sizeof(PrbMessageType));
    if (ret) {
        dev_err(&cntrl->dev, "Failed sending start pid message to PRUs\n");
        // TODO: Inviare un allarme su iio buffer
     }
    printk(KERN_DEBUG "pid_start_calibration: start calibration requested.\n");

    pidd->status.status = MLD_PID_DEVICE_STATUS_CALIBRATION_STARTING;
    return 0;
}
static int pid_stop_calibration(struct mylinuxdrone_device *cntrl) {
    struct mld_pid_device* pidd = to_mld_pid_device(cntrl);
    struct rpmsg_device* rpdev;
    unsigned char startMessage[sizeof(PrbMessageType)];
    int ret;
    printk(KERN_DEBUG "pid_stop_calibration\n");

    rpdev = dev_get_drvdata(&cntrl->dev);

    ((PrbMessageType*)startMessage)->message_type = PID_CALIBRATION_DISABLE_MSG_TYPE;

    ret = rpmsg_send(rpdev->ept, (void *)startMessage, sizeof(PrbMessageType));
    if (ret) {
        dev_err(&cntrl->dev, "Failed sending stop pid message to PRUs\n");
        // TODO: Inviare un allarme su iio buffer
     }
    printk(KERN_DEBUG "pid_stop_calibration: stop calibration requested.\n");

    pidd->status.status = MLD_PID_DEVICE_STATUS_CALIBRATION_STOPPING;
    return 0;
}

static ssize_t pid_start_calibration_store(struct device *dev,
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
            pid_start_calibration(ch);
            printk(KERN_INFO "pid_start_calibration_store: started.\n");
        } else {
            pid_stop_calibration(ch);
            printk(KERN_INFO "pid_stop_calibration_store: stopped.\n");
        }
        return ret ? : len;
}
static DEVICE_ATTR_WO(pid_start_calibration);

static struct attribute *pru_pid_attrs[] = {
        &dev_attr_pid_start.attr,
        &dev_attr_pid_start_calibration.attr,
        NULL,
};
ATTRIBUTE_GROUPS(pru_pid);

/********************************************************************
 *********************** RPMSG DRIVER SECTION ***********************
 ********************************************************************/
struct pru_pid_driver {
    struct mylinuxdrone_driver mlddrv;
    struct rpmsg_driver rpdrv;
};

/* pru_pid_id - Structure that holds the channel name for which this driver
   should be probed
 */
static const struct rpmsg_device_id rpmsg_pru_pid_id[] = {
        { .name = "pru-pid" },
        { },
};
MODULE_DEVICE_TABLE(rpmsg, rpmsg_pru_pid_id);

static const struct device_type pru_channel_type = {
        .name           = "pru_pid_channel",
};

/**
 * pru_pid_driver_cb() - function gets invoked each time the pru sends some
 * data.
 */
static int pru_pid_driver_cb(struct rpmsg_device *rpdev, void *data,
                  int len, void *priv, u32 src)
{
    struct mylinuxdrone_device* mlddev = dev_get_drvdata(&rpdev->dev);
    struct mld_pid_device* piddev = to_mld_pid_device(mlddev);
    int i = 0;

    PrbMessageType* dataStruct = (PrbMessageType*)data;
    printk(KERN_INFO "PID message type: [%d]\n", dataStruct->message_type);

    switch (dataStruct->message_type){
      case PID_CALIBRATION_ENABLED_MSG_TYPE:
      {
        piddev->status.status = MLD_PID_DEVICE_STATUS_CALIBRATING;
        printk(KERN_DEBUG "PID status: CALIBRATING\n");
        break;
      }
      case PID_CALIBRATION_DISABLED_MSG_TYPE:
      {
          unsigned char startMessage[sizeof(PrbMessageType)];
          int ret;
          piddev->status.status = MLD_PID_DEVICE_STATUS_STARTED;
          printk(KERN_DEBUG "PID status: STARTED\n");

          // send get pid configuration request to pru
          ((PrbMessageType*)startMessage)->message_type = PID_GET_CONFIG_MSG_TYPE;
          ret = rpmsg_send(rpdev->ept, (void *)startMessage, sizeof(PrbMessageType));
          if (ret) {
              printk(KERN_ERR "Sorry!! cannot get pid configuration from pru [err=%d]\n", ret);
           }
          break;
      }
      case PID_CONFIG_DATA_MSG_TYPE: {
          PrbConfigMessageType* pidConfig = (PrbConfigMessageType*)dataStruct;
              printk(KERN_DEBUG "PID_CNF: [%d, %d, %d, %d, %d, %d, %d].\n",
                     pidConfig->pid_config.ke,
                     pidConfig->pid_config.ki,
                     pidConfig->pid_config.kd,
                     pidConfig->pid_config.yke,
                     pidConfig->pid_config.ykd,
                     pidConfig->pid_config.yki,
                     pidConfig->pid_config.kgyro,
                     pidConfig->pid_config.mas
                     );
          break;
      }
      case PID_DATA_MSG_TYPE: {
          printk(KERN_DEBUG "PID_F: [%d, %d, %d, %d] - PID_M: [%d, %d, %d, %d].\n",
                 dataStruct->pid.F[0],
                 dataStruct->pid.F[1],
                 dataStruct->pid.F[2],
                 dataStruct->pid.F[3],
                 dataStruct->pid.M[0],
                 dataStruct->pid.M[1],
                 dataStruct->pid.M[2],
                 dataStruct->pid.M[3]
                 );
//          printk(KERN_DEBUG "ERR/I/D: [%d, %d, %d, %d] - [%d, %d, %d, %d] - [%d, %d, %d, %d].\n",
//                 dataStruct->pid.MErr[0],
//                 dataStruct->pid.MErr[1],
//                 dataStruct->pid.MErr[2],
//                 dataStruct->pid.MErr[3],
//                 dataStruct->pid.MIErr[0],
//                 dataStruct->pid.MIErr[1],
//                 dataStruct->pid.MIErr[2],
//                 dataStruct->pid.MIErr[3],
//                 dataStruct->pid.MDErr[0],
//                 dataStruct->pid.MDErr[1],
//                 dataStruct->pid.MDErr[2],
//                 dataStruct->pid.MDErr[3]
//                 );
          break;
      }
      default: {
          printk(KERN_DEBUG "pru_pid_driver_cb unknown message [%d] from [%s].\n", dataStruct->message_type, rpdev->id.name);
      }
    }
    return 0;
}

void prepare_mld_pid_device(struct mld_pid_device *piddev, const char* name, int id)
{
    int8_t i = 0;
    prepare_mylinuxdrone_device(&piddev->dev, name, id);
    for(i = 0; i <4; i++) {
        piddev->status.data.F[i] = 0;
        piddev->status.data.M[i] = 0;
        piddev->status.data.MErr[i] = 0;
        piddev->status.data.MIErr[i] = 0;
        piddev->status.data.MDErr[i] = 0;
    }
}
EXPORT_SYMBOL(prepare_mld_pid_device);

struct mld_pid_device *alloc_mld_pid_device(const char* name, int id)
{
    struct mld_pid_device *piddev;
    piddev = kzalloc(sizeof(*piddev), GFP_KERNEL);
    if (!piddev) {
        dev_err(&piddev->dev.dev, "Failed mld_pid device creation [%s]\n",
                         name);
        kfree(piddev);
        return NULL;
    }
    prepare_mld_pid_device(piddev, name, id);
    dev_dbg(&piddev->dev.dev, "device [%s] allocated \n",
            piddev->dev.id.name);
    return piddev;
}
EXPORT_SYMBOL(alloc_mld_pid_device);
/**
 * pru_pid_driver_probe() - function gets invoked when the rpmsg channel
 * as mentioned in the pru_pid_id table
 */
static int pru_pid_driver_probe(struct rpmsg_device *rpdev)
{
    int ret;
    struct mld_pid_device *st;
    st = alloc_mld_pid_device(MLD_PRU_PID_CHANNEL_NAME, 0);

    printk(KERN_DEBUG "pru_pid_driver_probe [%s].\n", rpdev->id.name);
    // FIXME: verificare se device già creato.
    printk(KERN_DEBUG "pru_pid_driver_probe device created.\n");

    printk(KERN_DEBUG "pru_pid_driver_probe allocated memory.\n");
    st->dev.dev.type = &pru_channel_type;
    st->dev.dev.devt = MKDEV(0, 0);
    st->dev.dev.groups = pru_pid_groups;
    st->dev.dev.release = mld_pru_pid_channel_dev_release;
    printk(KERN_DEBUG "pru_pid_driver_probe pru_pid device prepared.\n");

    /* devices's cipidular reference
       must be set before register_mylinuxdrone_device to avoid
       a loop that create a new rpmsg_channel when register mylinuxdrone device
     */
    dev_set_drvdata(&rpdev->dev, &st->dev);
    dev_set_drvdata(&st->dev.dev, rpdev);

    // registra il nuovo device
    printk(KERN_DEBUG "pru_pid_driver_probe registering pru_pid device... \n");
    ret = register_mylinuxdrone_device(&st->dev);
    if (ret) {
       printk(KERN_ERR "pru_pid_driver_probe pru_pid device registration failed.\n");
       kfree(st);
       return ret;
    }
    printk(KERN_DEBUG "pru_pid_driver_probe pru_pid device registered \n");

    st->status.status = MLD_PID_DEVICE_STATUS_ENABLED;
    return 0;
}

/**
 * pru_pid_driver_remove() - function gets invoked when the rpmsg device is
 * removed
 */
static void pru_pid_driver_remove(struct rpmsg_device *rpdev)
{
    struct mylinuxdrone_device *cntr;
    printk(KERN_DEBUG "pru_pid_driver_remove.\n");
    cntr = dev_get_drvdata(&rpdev->dev);
    if(cntr != NULL) {
        cntr->rpdev = NULL; // remove reference to rpdev
        dev_set_drvdata(&cntr->dev, NULL);
        unregister_mylinuxdrone_device(cntr);
    }
}

static struct pru_pid_driver mld_pru_pid_channel_driver = {
        .mlddrv = {
                       .probe = mld_pru_pid_channel_probe,
                       .remove = mld_pru_pid_channel_remove,
                       .id_table = arm_mylinuxdrone_pru_pid_id,
                       .driver = {
                               .name = MLD_PRU_PID_CHANNEL_NAME,
                               .of_match_table = arm_mylinuxdrone_pru_pid_matches,
                               .owner = THIS_MODULE,
                       },
        },
        .rpdrv = {
                  .drv.name   = KBUILD_MODNAME,
                  .drv.owner  = THIS_MODULE,
                  .id_table   = rpmsg_pru_pid_id,
                  .probe      = pru_pid_driver_probe,
                  .callback   = pru_pid_driver_cb,
                  .remove     = pru_pid_driver_remove,

        }
};


static int __init mld_pru_pid_channel_init(void)
{
    int ret;
    printk(KERN_DEBUG "mld_pru_pid_channel_init...\n");

    printk(KERN_DEBUG "mld_pru_pid_channel_init: registering mld_pru_pid_channel_driver ...\n");
    ret = register_mylinuxdrone_driver(&mld_pru_pid_channel_driver.mlddrv);
    if(ret) {
        pr_err("failed to register mld_pru_pid_channel_driver: %d\n", ret);
        return ret;
    }
    printk(KERN_DEBUG "mld_pru_pid_channel_init: mld_pru_pid_channel_driver registered ...\n");

    printk(KERN_DEBUG "mld_pru_pid_channel_init: registering rpmsg driver ...\n");
    ret = register_rpmsg_driver(&mld_pru_pid_channel_driver.rpdrv);
    if(ret) {
        pr_err("failed to register rpmsg driver: %d\n", ret);
        return ret;
    }
    printk(KERN_DEBUG "mld_pru_pid_channel_init: rpmsg driver registered ...\n");
    printk(KERN_DEBUG "mld_pru_pid_channel_init completed\n");
    return 0;
}

static void __exit mld_pru_pid_channel_fini(void)
{
    printk(KERN_DEBUG "mld_pru_pid_channel_fini started ...\n");
    unregister_mylinuxdrone_driver(&mld_pru_pid_channel_driver.mlddrv);
    unregister_rpmsg_driver (&mld_pru_pid_channel_driver.rpdrv);
    printk(KERN_DEBUG "mld_pru_pid_channel_fini ...\n");
}

postcore_initcall(mld_pru_pid_channel_init);
module_exit(mld_pru_pid_channel_fini);

