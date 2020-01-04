/*
 * mylinuxdrone.h
 *
 *  Created on: 03 mag 2019
 *      Author: andrea
 */

#ifndef MYLINUXDRONE_H_
#define MYLINUXDRONE_H_

#include <linux/rpmsg.h>
#include <linux/iio/iio.h>
#include <linux/device.h>

#define MYLINUXDRONE_BUS_NAME "mylinuxdrone"
#define MYLINUXDRONE_NAME_SIZE 32

/******************************************
 ******* Strutture Driver e Device  *******
 ******************************************/
struct mylinuxdrone_device_id {
    char name[RPMSG_NAME_SIZE];
};

struct mylinuxdrone_device {
    struct device dev;
    struct mylinuxdrone_device_id id;
    struct rpmsg_device* rpdev;
    struct iio_dev* iiodev;
    char type[RPMSG_NAME_SIZE]; // imu, rc, gps, etc. (hardcoded)
};

struct mylinuxdrone_driver {
    struct device_driver driver;
    const struct mylinuxdrone_device_id *id_table;
    int (*probe)(struct mylinuxdrone_device *mlddev);
    void (*remove)(struct mylinuxdrone_device *mlddev);
};

#define to_mylinuxdrone_driver(d) container_of(d, struct mylinuxdrone_driver, driver)
#define to_mylinuxdrone_device(d) container_of(d, struct mylinuxdrone_device, dev)

struct mylinuxdrone_device *alloc_mylinuxdrone_device(const char* name, int id);
int register_mylinuxdrone_device(struct mylinuxdrone_device *mlddev);
void unregister_mylinuxdrone_device(struct mylinuxdrone_device *mlddev);
int __register_mylinuxdrone_driver(struct mylinuxdrone_driver *mlddrv, struct module *owner);
void unregister_mylinuxdrone_driver(struct mylinuxdrone_driver *mlddrv);
#define register_mylinuxdrone_driver(mlddrv) \
    __register_mylinuxdrone_driver(mlddrv, THIS_MODULE)


#endif /* MYLINUXDRONE_H_ */
