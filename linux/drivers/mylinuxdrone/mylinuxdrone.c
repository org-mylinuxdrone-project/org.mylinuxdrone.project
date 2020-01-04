/*
 * provaKModule.c
 *
 *  Created on: 29 apr 2019
 *      Author: andrea
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include "mylinuxdrone.h"

MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR("Andrea Lambruschini <andrea.lambruschini@gmail.com>");

static int mylinuxdrone_dev_match(struct device *dev, struct device_driver *drv);
static int mylinuxdrone_dev_probe(struct device *dev);
static int mylinuxdrone_dev_remove(struct device *dev);
static int mylinuxdrone_dev_uevent(struct device *dev,
                                   struct kobj_uevent_env *env);
static struct bus_type mylinuxdrone_bus_type = {
        .name = MYLINUXDRONE_BUS_NAME, .match = mylinuxdrone_dev_match,
        .uevent = mylinuxdrone_dev_uevent, .probe = mylinuxdrone_dev_probe,
        .remove = mylinuxdrone_dev_remove, };

/*****************************************************************************
 * Device and Driver Registration methods
 *****************************************************************************/
int register_mylinuxdrone_device(struct mylinuxdrone_device *mlddev)
{
    int ret;
    printk(KERN_INFO "register_mylinuxdrone_device started with dev:[%s] ... \n",
           mlddev->id.name);

    mlddev->dev.bus = &mylinuxdrone_bus_type;
    mlddev->dev.parent = mylinuxdrone_bus_type.dev_root;
    ret = device_register(&mlddev->dev);
    if (ret)
    {
        dev_err(&mlddev->dev, "register_mylinuxdrone_device failed: %d\n", ret);
        put_device(&mlddev->dev);
        return ret;
    }
    printk(KERN_INFO "register_mylinuxdrone_device dev:[%s] registered ... \n",
           mlddev->id.name);
    return 0;
}
EXPORT_SYMBOL(register_mylinuxdrone_device);

void unregister_mylinuxdrone_device(struct mylinuxdrone_device *mlddev)
{
    printk(KERN_INFO "unregister_mylinuxdrone_device started with dev:[%s] ... \n",
           mlddev->id.name);
    device_unregister(&mlddev->dev);
    put_device(&mlddev->dev);
    printk(KERN_INFO "unregister_mylinuxdrone_device unregistered ... \n");
}
EXPORT_SYMBOL(unregister_mylinuxdrone_device);

int __register_mylinuxdrone_driver(struct mylinuxdrone_driver *mlddrv,
                                   struct module *owner)
{
    int ret;
    printk(KERN_INFO "__register_mylinuxdrone_driver started with drv:[%s] ... \n",
           mlddrv->driver.name);
    mlddrv->driver.bus = &mylinuxdrone_bus_type;
    mlddrv->driver.owner = owner;
    ret = driver_register(&mlddrv->driver);
    if (ret)
    {
        printk(KERN_ERR "__register_mylinuxdrone_driver failed for drv:[%s] ... \n",
               mlddrv->driver.name);
        return ret;
    }
    printk(KERN_INFO "__register_mylinuxdrone_driver drv:[%s] registered ... \n",
           mlddrv->driver.name);
    return 0;
}
EXPORT_SYMBOL(__register_mylinuxdrone_driver);

static int __mylinuxdrone_devices_unregister(struct device *dev, void *data)
{
    struct mylinuxdrone_driver *mlddrv = (struct mylinuxdrone_driver *)data;
        if(mylinuxdrone_dev_match(dev, &mlddrv->driver)) {
            device_unregister(dev);
        }
        return 0;
}

void unregister_mylinuxdrone_driver(struct mylinuxdrone_driver *mlddrv)
{
    printk(KERN_INFO "unregister_mylinuxdrone_driver started with drv:[%s] ... \n",
           mlddrv->driver.name);

    bus_for_each_dev(&mylinuxdrone_bus_type, NULL, mlddrv, __mylinuxdrone_devices_unregister);
    driver_unregister(&mlddrv->driver);
    printk(KERN_INFO "unregister_mylinuxdrone_driver drv:[%s] unregistered ... \n",
           mlddrv->driver.name);
}
EXPORT_SYMBOL(unregister_mylinuxdrone_driver);

/*****************************************************************************
 * Device's Life Cycle
 *****************************************************************************/
struct mylinuxdrone_device *alloc_mylinuxdrone_device(const char* name, int id)
{
    struct mylinuxdrone_device *mlddev;
    mlddev = kzalloc(sizeof(*mlddev), GFP_KERNEL);
    if (!mlddev) {
        dev_err(&mlddev->dev, "Failed mylinuxdrone device creation [%s]\n",
                         name);
        kfree(mlddev);
        return NULL;
    }

    strcpy(mlddev->id.name, name);
    dev_set_name(&mlddev->dev, name);
    mlddev->dev.id = id;
    dev_dbg(&mlddev->dev, "device [%s] allocated \n",
            mlddev->id.name);
    return mlddev;
}
EXPORT_SYMBOL(alloc_mylinuxdrone_device);

static inline int mylinuxdrone_id_match(struct mylinuxdrone_device* mldev,
                                        const char* id)
{
    return (strcmp(mldev->id.name, id) == 0);
}

static int mylinuxdrone_dev_match(struct device *dev, struct device_driver *drv)
{
    unsigned int i;
    struct mylinuxdrone_device* mlddev = to_mylinuxdrone_device(dev);
    struct mylinuxdrone_driver* mlddrv = to_mylinuxdrone_driver(drv);
    printk(KERN_INFO "mylinuxdrone_dev_match started with dev:[%s], drv:[%s] ... \n",
           mlddev->id.name, drv->name);

    for (i = 0; *((mlddrv->id_table + i)->name); i++)
    {
        if (mylinuxdrone_id_match(mlddev, mlddrv->id_table[i].name))
        {
            printk(KERN_INFO "mylinuxdrone_dev_match matched dev:[%s] ... \n",
                   mlddev->id.name);
            return 1;
        }
    }
    printk(KERN_INFO "mylinuxdrone_dev_match does not match dev:[%s] ... \n",
           mlddev->id.name);
    return 0;
}
static int mylinuxdrone_dev_probe(struct device *dev)
{
    int err;
    struct mylinuxdrone_device *mlddev = to_mylinuxdrone_device(dev);
    struct mylinuxdrone_driver *mlddrv = to_mylinuxdrone_driver(
            mlddev->dev.driver);
    printk(KERN_INFO "mylinuxdrone__dev_probe partially implemented dev:[%s]\n",
           mlddev->id.name);

    /* TODO:
     * 1) riconoscere il device (imu, rc, etc)
     * 2) assegnare funzione publish sul driver.
     *    La funzione publish punta su un altro driver .. es. il PID
     *
     *    Più generica:
     *    1) ogni driver dichiara a quali dati è interessato
     *    2) ogni driver dichiara quali dati può pubblicare
     *    3) ogni driver permette di aggiungere/togliere un observer
     *    4) il controller connette gli observer su chi pubblica i dati (evitando cicli?).
     *    5) quando un driver viene rimosso, deve essere eliminata la sua callback da tutti i driver su cui è registrata
     *       (vedi mylinuxdrone_dev_remove)
     *
     *    Più ristretta:
     *    1) ogni device dichiara il tipo (imu, rc, etc)
     *    2) ogni device permette di aggiungere / rimuovere observers (add_observer, remove_observer)
     *    3) il controller sa come connetterli in base al type espresso dal device (imu, rc, etc)
     *    Già modificato le strutture mylinuxdrone_driver e mylinuxdrone_device per questo
     */

    err = mlddrv->probe(mlddev);
    if (err)
    {
        dev_err(dev, "probe: %s: failed: %d\n", mlddev->id.name, err);
        return err;
    }
    return 0;
}
static int mylinuxdrone_dev_uevent(struct device *dev,
                                   struct kobj_uevent_env *env)
{
    struct mylinuxdrone_device *mlddev = to_mylinuxdrone_device(dev);
    printk(KERN_INFO "mylinuxdrone_uevent (dev:[%s]) not implemented yet\n",
           mlddev->id.name);
    return 0;
}
static int mylinuxdrone_dev_remove(struct device *dev)
{
    struct mylinuxdrone_device *mlddev = to_mylinuxdrone_device(dev);
    struct mylinuxdrone_driver *mlddrv = to_mylinuxdrone_driver(
            mlddev->dev.driver);
    printk(KERN_INFO "mylinuxdrone_dev_remove dev:[%s]\n", mlddev->id.name);
    mlddrv->remove(mlddev);
    return 0;
}

/*****************************************************************************
 * Bus device, the master.
 *****************************************************************************/
static const struct of_device_id arm_mylinuxdrone_matches[] = { { .compatible =
        "arm,mylinuxdrone", },
                                                                { }, };

static int mylinuxdrone_bus_cntr_probe(struct platform_device *pdev)
{
    printk(KERN_INFO "mylinuxdrone_bus_cntr_probe implemented only for testing [%s][%d] \n",
           pdev->name, pdev->dev.id);
    if ((!pdev->dev.of_node) || !of_device_is_available(pdev->dev.of_node))
    {
        return -ENODEV;
    }
    printk(KERN_INFO "mylinuxdrone_bus_cntr_probe device is available \n");
    mylinuxdrone_bus_type.dev_root = &pdev->dev;
    if(!mylinuxdrone_bus_type.dev_root) {
        printk(KERN_ERR "mylinuxdrone_bus_cntr_probe device root is null! \n");
    } else {
        printk(KERN_INFO "mylinuxdrone_bus_cntr_probe device root is NOT null \n");
    }
    return 0;
}

static int mylinuxdrone_bus_cntr_remove(struct platform_device *pdev)
{
    printk(KERN_INFO "mylinuxdrone_bus_cntr_probe not implemented yet \n");
    return 0;
}

static struct platform_driver mylinuxdrone_bus_cntr_driver = {
        .probe = mylinuxdrone_bus_cntr_probe, .remove =
                mylinuxdrone_bus_cntr_remove,
        .driver = { .name = MYLINUXDRONE_BUS_NAME, .of_match_table =
                            arm_mylinuxdrone_matches,
                    .owner = THIS_MODULE, }, };
/*
 * Verifica se il device ha status ok o no.
 */
static int __init mylinuxdrone_init(void)
{
    int ret;
    printk(KERN_ALERT "mylinuxdrone_init...\n");

    ret = bus_register(&mylinuxdrone_bus_type);
    if (ret)
    {
        pr_err("failed to register mylinuxdrone bus: %d\n", ret);
        return ret;
    }

    printk(KERN_ALERT "mylinuxdrone bus registered ...\n");
    ret = platform_driver_register(&mylinuxdrone_bus_cntr_driver);
    if (ret)
    {
        pr_err("failed to register driver for mylinuxdrone bus controller: %d\n",
               ret);
        return ret;
    }
    printk(KERN_ALERT "mylinuxdrone bus driver registered ...\n");

    printk(KERN_ALERT "mylinuxdrone_init completed\n");
    return 0;
}

static void __exit mylinuxdrone_fini(void)
{
    printk(KERN_WARNING "mylinuxdrone_fini ...\n");
    platform_driver_unregister(&mylinuxdrone_bus_cntr_driver);
    printk(KERN_WARNING "mylinuxdrone controller unregistered ...\n");
    bus_unregister(&mylinuxdrone_bus_type);
    printk(KERN_WARNING "mylinuxdrone_completed ...\n");
}

postcore_initcall(mylinuxdrone_init);
module_exit(mylinuxdrone_fini);

