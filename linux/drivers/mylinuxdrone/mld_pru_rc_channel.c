/*
 * mld_pru_rc_channel.c
 *  Created on: 12 mag 2019
 *      Author: Andrea Lambruschini <andrea.lambruschini@gmail.com>
 *
 * Copyright 2019 Andrea Lambruschini
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/rpmsg.h>
#include <linux/device.h>
#include "mld_pru_rc_channel.h"
#include "mld_messages.h"


MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR("Andrea Lambruschini <andrea.lambruschini@gmail.com>");

#define MLD_PRU_RC_CHANNEL_NAME "mld-rc"

/********************************************************************
 **************** MLD_PRU_RC_CHANNEL DRIVER SECTION *****************
 ********************************************************************/
static int mld_pru_rc_channel_probe(struct mylinuxdrone_device *mlddev) {
    printk(KERN_DEBUG "mld_pru_rc_channel_probe started...\n");
    return 0;
}
static void mld_pru_rc_channel_remove(struct mylinuxdrone_device *mlddev) {
    struct rpmsg_device* rpdev;
    printk(KERN_DEBUG "mld_pru_rc_channel_remove: dev:[%s] ...\n", mlddev->id.name);
    rpdev = dev_get_drvdata(&mlddev->dev);
    if(rpdev != NULL) {
        dev_set_drvdata(&rpdev->dev, NULL);
    }
    printk(KERN_DEBUG "mld_pru_rc_channel_remove: device:[%s] removed\n", mlddev->id.name);
}

static void mld_pru_rc_channel_dev_release(struct device* dev) {
    struct mylinuxdrone_device *mlddev = to_mylinuxdrone_device(dev);

    printk(KERN_DEBUG "mld_pru_rc_channel_dev_release dev:[%s] ...\n", mlddev->id.name);
    // TODO: kfree?
}
EXPORT_SYMBOL(mld_pru_rc_channel_dev_release);

static const struct of_device_id arm_mylinuxdrone_pru_rc_matches[] = {
        { .compatible = "arm,mylinuxdrone_pru", .name = MLD_PRU_RC_CHANNEL_NAME},
        {},
};
MODULE_DEVICE_TABLE(mylinuxdrone, arm_mylinuxdrone_pru_rc_matches);

static const struct mylinuxdrone_device_id arm_mylinuxdrone_pru_rc_id[] = {
        { .name = MLD_PRU_RC_CHANNEL_NAME },
        { },
};
MODULE_DEVICE_TABLE(mylinuxdrone, arm_mylinuxdrone_pru_rc_id);


static int rc_start(struct mylinuxdrone_device *cntrl) {
    struct mld_rc_device* rcd = to_mld_rc_device(cntrl);
    struct rpmsg_device* rpdev;
    unsigned char startMessage[sizeof(PrbMessageType)];
    int ret;
    printk(KERN_DEBUG "rc_start\n");
    rpdev = dev_get_drvdata(&cntrl->dev);

    ((PrbMessageType*)startMessage)->message_type = RC_ENABLE_MSG_TYPE;

    ret = rpmsg_send(rpdev->ept, (void *)startMessage, sizeof(PrbMessageType));
    if (ret) {
        dev_err(&cntrl->dev, "Failed sending start rc message to PRUs\n");
        // TODO: Inviare un allarme su iio status
     }
    printk(KERN_DEBUG "rc_start: creation of pru_rc device requested.\n");

    // Status: 'Started'
    rcd->status.status = MLD_RC_DEVICE_STATUS_STARTED;
    return 0;
}
static int rc_stop(struct mylinuxdrone_device *cntrl) {
    struct mld_rc_device* rcd = to_mld_rc_device(cntrl);
    struct rpmsg_device* rpdev;
    unsigned char startMessage[sizeof(PrbMessageType)];
    int ret;
    printk(KERN_DEBUG "rc_stop\n");

    rpdev = dev_get_drvdata(&cntrl->dev);

    ((PrbMessageType*)startMessage)->message_type = RC_DISABLE_MSG_TYPE;

    ret = rpmsg_send(rpdev->ept, (void *)startMessage, sizeof(PrbMessageType));
    if (ret) {
        dev_err(&cntrl->dev, "Failed sending stop rc message to PRUs\n");
        // TODO: Inviare un allarme su iio buffer
     }
    printk(KERN_DEBUG "rc_stop: stop of pru_rc device requested.\n");

    // Status: 'Stopped'
    rcd->status.status = MLD_RC_DEVICE_STATUS_STOPPED;
    return 0;
}

static ssize_t rc_start_store(struct device *dev,
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
            rc_start(ch);
            printk(KERN_INFO "pru_rc_store: started.\n");
        } else {
            rc_stop(ch);
            printk(KERN_INFO "pru_rc_store: stopped.\n");
        }
        return ret ? : len;
}
static DEVICE_ATTR_WO(rc_start);


static int rc_start_calibration(struct mylinuxdrone_device *cntrl) {
    struct mld_rc_device* rcd = to_mld_rc_device(cntrl);
    struct rpmsg_device* rpdev;
    unsigned char startMessage[sizeof(PrbMessageType)];
    int ret;
    printk(KERN_DEBUG "rc_start_calibration\n");
    rpdev = dev_get_drvdata(&cntrl->dev);

    ((PrbMessageType*)startMessage)->message_type = RC_CALIBRATION_ENABLE_MSG_TYPE;

    ret = rpmsg_send(rpdev->ept, (void *)startMessage, sizeof(PrbMessageType));
    if (ret) {
        dev_err(&cntrl->dev, "Failed sending start rc message to PRUs\n");
        // TODO: Inviare un allarme su iio buffer
     }
    printk(KERN_DEBUG "rc_start_calibration: start calibration requested.\n");

    rcd->status.status = MLD_RC_DEVICE_STATUS_CALIBRATION_STARTING;
    return 0;
}
static int rc_stop_calibration(struct mylinuxdrone_device *cntrl) {
    struct mld_rc_device* rcd = to_mld_rc_device(cntrl);
    struct rpmsg_device* rpdev;
    unsigned char startMessage[sizeof(PrbMessageType)];
    int ret;
    printk(KERN_DEBUG "rc_stop_calibration\n");

    rpdev = dev_get_drvdata(&cntrl->dev);

    ((PrbMessageType*)startMessage)->message_type = RC_CALIBRATION_DISABLE_MSG_TYPE;

    ret = rpmsg_send(rpdev->ept, (void *)startMessage, sizeof(PrbMessageType));
    if (ret) {
        dev_err(&cntrl->dev, "Failed sending stop rc message to PRUs\n");
        // TODO: Inviare un allarme su iio buffer
     }
    printk(KERN_DEBUG "rc_stop_calibration: stop calibration requested.\n");

    rcd->status.status = MLD_RC_DEVICE_STATUS_CALIBRATION_STOPPING;
    return 0;
}

static ssize_t rc_start_calibration_store(struct device *dev,
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
            rc_start_calibration(ch);
            printk(KERN_INFO "rc_start_calibration_store: started.\n");
        } else {
            rc_stop_calibration(ch);
            printk(KERN_INFO "rc_stop_calibration_store: stopped.\n");
        }
        return ret ? : len;
}
static DEVICE_ATTR_WO(rc_start_calibration);

static struct attribute *pru_rc_attrs[] = {
        &dev_attr_rc_start.attr,
        &dev_attr_rc_start_calibration.attr,
        NULL,
};
ATTRIBUTE_GROUPS(pru_rc);

/********************************************************************
 *********************** RPMSG DRIVER SECTION ***********************
 ********************************************************************/
struct pru_rc_driver {
    struct mylinuxdrone_driver mlddrv;
    struct rpmsg_driver rpdrv;
};

/* pru_rc_id - Structure that holds the channel name for which this driver
   should be probed
 */
static const struct rpmsg_device_id rpmsg_pru_rc_id[] = {
        { .name = "pru-rc" },
        { },
};
MODULE_DEVICE_TABLE(rpmsg, rpmsg_pru_rc_id);

static const struct device_type pru_channel_type = {
        .name           = "pru_rc_channel",
};

/**
 * pru_rc_driver_cb() - function gets invoked each time the pru sends some
 * data.
 */
static int pru_rc_driver_cb(struct rpmsg_device *rpdev, void *data,
                  int len, void *priv, u32 src)
{
    struct mylinuxdrone_device* mlddev = dev_get_drvdata(&rpdev->dev);
    struct mld_rc_device* rcdev = to_mld_rc_device(mlddev);
    int i = 0;

    PrbMessageType* dataStruct = (PrbMessageType*)data;
    printk(KERN_INFO "RC message type: [%d]\n", dataStruct->message_type);

    switch (dataStruct->message_type){
      case RC_CALIBRATION_ENABLED_MSG_TYPE:
      {
        rcdev->status.status = MLD_RC_DEVICE_STATUS_CALIBRATING;
        printk(KERN_DEBUG "RC status: CALIBRATING\n");
        break;
      }
      case RC_CALIBRATION_DISABLED_MSG_TYPE:
      {
          unsigned char startMessage[sizeof(PrbMessageType)];
          int ret;
          rcdev->status.status = MLD_RC_DEVICE_STATUS_STARTED;
          printk(KERN_DEBUG "RC status: STARTED\n");

          // send get rc configuration request to pru
          ((PrbMessageType*)startMessage)->message_type = RC_GET_CONFIG_MSG_TYPE;
          ret = rpmsg_send(rpdev->ept, (void *)startMessage, sizeof(PrbMessageType));
          if (ret) {
              printk(KERN_ERR "Sorry!! cannot get rc configuration from pru [err=%d]\n", ret);
           }
          break;
      }
      case RC_CONFIG_DATA_MSG_TYPE: {
          PrbConfigMessageType* rcConfig = (PrbConfigMessageType*)dataStruct;
          for(i = 0; i < 8; i++) {
              printk(KERN_DEBUG "RC_CNF[%d]: [%d, %d, %d, %d, %d, %d, %d].\n",
                     i,
                     rcConfig->rc_config_chan[i].rawMin,
                     rcConfig->rc_config_chan[i].rawCenter,
                     rcConfig->rc_config_chan[i].rawMax,
                     rcConfig->rc_config_chan[i].min,
                     rcConfig->rc_config_chan[i].max,
                     rcConfig->rc_config_chan[i].radius,
                     rcConfig->rc_config_chan[i].factor
                     );
          }
          break;
      }
      case RC_DATA_MSG_TYPE: {
          printk(KERN_DEBUG "RC_TYPR12: [%d, %d, %d, %d, %d, %d].\n", dataStruct->rc.throttle, dataStruct->rc.yaw, dataStruct->rc.pitch, dataStruct->rc.roll, dataStruct->rc.aux1, dataStruct->rc.aux2);
          break;
      }
      default: {
          printk(KERN_DEBUG "pru_rc_driver_cb unknown message [%d] from [%s].\n", dataStruct->message_type, rpdev->id.name);
      }
    }
    return 0;
}

void prepare_mld_rc_device(struct mld_rc_device *rcdev, const char* name, int id)
{
    int8_t i = 0;
    prepare_mylinuxdrone_device(&rcdev->dev, name, id);
    rcdev->status.data.throttle = 0;
    rcdev->status.data.yaw = 0;
    rcdev->status.data.pitch = 0;
    rcdev->status.data.roll = 0;
    rcdev->status.data.aux1 = 0;
    rcdev->status.data.aux2 = 0;
    rcdev->status.data.aux3 = 0;
    rcdev->status.data.aux4 = 0;

    for(i = 0; i < 8; i++) {
        rcdev->status.config[i].rawMin = 0;
        rcdev->status.config[i].rawCenter = 0;
        rcdev->status.config[i].rawMax = 0;
        rcdev->status.config[i].min = 0;
        rcdev->status.config[i].max = 0;
        rcdev->status.config[i].radius = 0;
        rcdev->status.config[i].factor = 0;
    }
}
EXPORT_SYMBOL(prepare_mld_rc_device);

struct mld_rc_device *alloc_mld_rc_device(const char* name, int id)
{
    struct mld_rc_device *rcdev;
    rcdev = kzalloc(sizeof(*rcdev), GFP_KERNEL);
    if (!rcdev) {
        dev_err(&rcdev->dev.dev, "Failed mld_rc device creation [%s]\n",
                         name);
        kfree(rcdev);
        return NULL;
    }
    prepare_mld_rc_device(rcdev, name, id);
    dev_dbg(&rcdev->dev.dev, "device [%s] allocated \n",
            rcdev->dev.id.name);
    return rcdev;
}
EXPORT_SYMBOL(alloc_mld_rc_device);
/**
 * pru_rc_driver_probe() - function gets invoked when the rpmsg channel
 * as mentioned in the pru_rc_id table
 */
static int pru_rc_driver_probe(struct rpmsg_device *rpdev)
{
    int ret;
    struct mld_rc_device *st;
    st = alloc_mld_rc_device(MLD_PRU_RC_CHANNEL_NAME, 0);

    printk(KERN_DEBUG "pru_rc_driver_probe [%s].\n", rpdev->id.name);
    // FIXME: verificare se device già creato.
    printk(KERN_DEBUG "pru_rc_driver_probe device created.\n");

    printk(KERN_DEBUG "pru_rc_driver_probe allocated memory.\n");
    st->dev.dev.type = &pru_channel_type;
    st->dev.dev.devt = MKDEV(0, 0);
    st->dev.dev.groups = pru_rc_groups;
    st->dev.dev.release = mld_pru_rc_channel_dev_release;
    printk(KERN_DEBUG "pru_rc_driver_probe pru_rc device prepared.\n");

    /* devices's circular reference
       must be set before register_mylinuxdrone_device to avoid
       a loop that create a new rpmsg_channel when register mylinuxdrone device
     */
    dev_set_drvdata(&rpdev->dev, &st->dev);
    dev_set_drvdata(&st->dev.dev, rpdev);

    // registra il nuovo device
    printk(KERN_DEBUG "pru_rc_driver_probe registering pru_rc device... \n");
    ret = register_mylinuxdrone_device(&st->dev);
    if (ret) {
       printk(KERN_ERR "pru_rc_driver_probe pru_rc device registration failed.\n");
       kfree(st);
       return ret;
    }
    printk(KERN_DEBUG "pru_rc_driver_probe pru_rc device registered \n");

    st->status.status = MLD_RC_DEVICE_STATUS_ENABLED;
    return 0;
}

/**
 * pru_rc_driver_remove() - function gets invoked when the rpmsg device is
 * removed
 */
static void pru_rc_driver_remove(struct rpmsg_device *rpdev)
{
    struct mylinuxdrone_device *cntr;
    printk(KERN_DEBUG "pru_rc_driver_remove.\n");
    cntr = dev_get_drvdata(&rpdev->dev);
    if(cntr != NULL) {
        cntr->rpdev = NULL; // remove reference to rpdev
        dev_set_drvdata(&cntr->dev, NULL);
        unregister_mylinuxdrone_device(cntr);
    }
}

static struct pru_rc_driver mld_pru_rc_channel_driver = {
        .mlddrv = {
                       .probe = mld_pru_rc_channel_probe,
                       .remove = mld_pru_rc_channel_remove,
                       .id_table = arm_mylinuxdrone_pru_rc_id,
                       .driver = {
                               .name = MLD_PRU_RC_CHANNEL_NAME,
                               .of_match_table = arm_mylinuxdrone_pru_rc_matches,
                               .owner = THIS_MODULE,
                       },
        },
        .rpdrv = {
                  .drv.name   = KBUILD_MODNAME,
                  .drv.owner  = THIS_MODULE,
                  .id_table   = rpmsg_pru_rc_id,
                  .probe      = pru_rc_driver_probe,
                  .callback   = pru_rc_driver_cb,
                  .remove     = pru_rc_driver_remove,

        }
};


static int __init mld_pru_rc_channel_init(void)
{
    int ret;
    printk(KERN_DEBUG "mld_pru_rc_channel_init...\n");

    printk(KERN_DEBUG "mld_pru_rc_channel_init: registering mld_pru_rc_channel_driver ...\n");
    ret = register_mylinuxdrone_driver(&mld_pru_rc_channel_driver.mlddrv);
    if(ret) {
        pr_err("failed to register mld_pru_rc_channel_driver: %d\n", ret);
        return ret;
    }
    printk(KERN_DEBUG "mld_pru_rc_channel_init: mld_pru_rc_channel_driver registered ...\n");

    printk(KERN_DEBUG "mld_pru_rc_channel_init: registering rpmsg driver ...\n");
    ret = register_rpmsg_driver(&mld_pru_rc_channel_driver.rpdrv);
    if(ret) {
        pr_err("failed to register rpmsg driver: %d\n", ret);
        return ret;
    }
    printk(KERN_DEBUG "mld_pru_rc_channel_init: rpmsg driver registered ...\n");
    printk(KERN_DEBUG "mld_pru_rc_channel_init completed\n");
    return 0;
}

static void __exit mld_pru_rc_channel_fini(void)
{
    printk(KERN_DEBUG "mld_pru_rc_channel_fini started ...\n");
    unregister_mylinuxdrone_driver(&mld_pru_rc_channel_driver.mlddrv);
    unregister_rpmsg_driver (&mld_pru_rc_channel_driver.rpdrv);
    printk(KERN_DEBUG "mld_pru_rc_channel_fini ...\n");
}

postcore_initcall(mld_pru_rc_channel_init);
module_exit(mld_pru_rc_channel_fini);

