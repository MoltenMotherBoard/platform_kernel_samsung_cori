/*
 * d2041-i2c.c: I2C (Serial Communication) driver for D2041
 *   
 * Copyright(c) 2011 Dialog Semiconductor Ltd.
 *  
 * Author: Dialog Semiconductor Ltd. D. Chen, D. Patel
 *  
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/d2041/core.h>
#include <linux/d2041/d2041_reg.h> 
#include <linux/slab.h>

#define CONFIG_D2041_USE_SMBUS_API

/*
 *
 */
static int d2041_i2c_read_device(struct d2041 * const d2041, char const reg,
				  int const bytes, void * const dest)
{
	int ret;

// DLG eric. 03/Nov/2011.
#ifdef CONFIG_D2041_USE_SMBUS_API
	if (bytes > 1)
		ret = i2c_smbus_read_i2c_block_data(d2041->i2c_client, reg, bytes, dest);
	else {
		ret = i2c_smbus_read_byte_data(d2041->i2c_client, reg);
		if (ret < 0)
			return ret;
		*(unsigned char *)dest = (unsigned char)ret;
	}
	return 0;
#else
	ret = i2c_master_send(d2041->i2c_client, &reg, 1);
	if (ret < 0) {
        dlg_err("Err in i2c_master_send(0x%x)\n", reg);
		return ret;
    }
	ret = i2c_master_recv(d2041->i2c_client, dest, bytes);
	if (ret < 0) {
        dlg_err("Err in i2c_master_recv(0x%x)\n", ret);
		return ret;
    }
	if (ret != bytes)
		return -EIO;
#endif /* CONFIG_D2041_USE_SMBUS_API */
	return 0;
}


/*
 *
 */
static int d2041_i2c_write_device(struct d2041 * const d2041, char const reg,
				   int const bytes, const u8 *src /*void * const src*/)
{
    int ret = 0;

    // Redundant. It already checked in d2041_reg_read & write function
    //if ((reg + bytes) > D2041_MAX_REGISTER_CNT) {
    //    printk(KERN_ERR "Bad input to d2041_i2c_write_device(0x%x, %d)\n", reg, bytes);
    //    return -EINVAL;
    //}

#ifdef CONFIG_D2041_USE_SMBUS_API
    if(bytes > 1)
        ret = i2c_smbus_write_i2c_block_data(d2041->i2c_client, reg, bytes, src);
    else
        ret = i2c_smbus_write_byte_data(d2041->i2c_client, reg, *src);

    return ret;
#else
    u8 msg[bytes + 1];
    
    msg[0] = reg;
    memcpy(&msg[1], src, bytes);

    ret = i2c_master_send(d2041->i2c_client, msg, bytes + 1);
    if (ret < 0)
        return ret;
    if (ret != bytes + 1)
        return -EIO;
    
    return 0;
#endif /* CONFIG_D2041_USE_SMBUS_API */
}


static int d2041_i2c_probe(struct i2c_client *i2c,
			    const struct i2c_device_id *id)
{
	struct d2041 *d2041;
	int ret = 0;

	d2041 = kzalloc(sizeof(struct d2041), GFP_KERNEL);
	if (d2041 == NULL) {
		kfree(i2c);
		return -ENOMEM;
	}

	i2c_set_clientdata(i2c, d2041);
	d2041->dev = &i2c->dev;
	d2041->i2c_client = i2c;
	d2041->read_dev = d2041_i2c_read_device;
	d2041->write_dev = d2041_i2c_write_device;

	ret = d2041_device_init(d2041, i2c->irq, i2c->dev.platform_data);
	if (ret < 0)
		goto err;


	return ret;

err:
	kfree(d2041);
	return ret;
}

static int d2041_i2c_remove(struct i2c_client *i2c)
{
	struct d2041 *d2041 = i2c_get_clientdata(i2c);

	d2041_device_exit(d2041);
	kfree(d2041);
	return 0;
}


static const struct i2c_device_id d2041_i2c_id[] = {
    { D2041_I2C, 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, d2041_i2c_id);


static struct i2c_driver d2041_i2c_driver = {
	.driver = {
		   .name = D2041_I2C,
		   .owner = THIS_MODULE,
	},
	.probe = d2041_i2c_probe,
	.remove = d2041_i2c_remove,
	.id_table = d2041_i2c_id,
};

static int __init d2041_i2c_init(void)
{
	return i2c_add_driver(&d2041_i2c_driver);
}

/* Initialised very early during bootup (in parallel with Subsystem init) */
subsys_initcall(d2041_i2c_init);
//module_init(d2041_i2c_init);

static void __exit d2041_i2c_exit(void)
{
	i2c_del_driver(&d2041_i2c_driver);
}
module_exit(d2041_i2c_exit);

MODULE_AUTHOR("Dialog Semiconductor Ltd <divyang.patel@diasemi.com>");
MODULE_DESCRIPTION("I2C MFD driver for Dialog D2041 PMIC plus Audio");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" D2041_I2C);
