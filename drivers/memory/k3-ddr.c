// SPDX-License-Identifier: GPL-2.0-only
/*
 * TI DDR Controller Driver for K3 family of SoCs
 *
 * Copyright (C) 2024 Texas Instruments Incorporated. http://www.ti.com/
 *
 */

#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/hwmon.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/reboot.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/spinlock.h>

#define MRR_TEMPCHK_NORM_THRESHOLD_F2_OFFSET		0x24C
#define MRR_TEMPCHK_NORM_THRESHOLD_F2_DEFAULT_VAL	0x0
#define MRR_TEMPCHK_NORM_THRESHOLD_F2_SET_VAL		0xF

#define AUTO_TEMPCHK_VAL_0_OFFSET			0x2F8
#define AUTO_TEMPCHK_VAL_0_START_BIT			0x18
#define AUTO_TEMPCHK_VAL_0_MR4_DATA_MASK		0x7

#define INT_STATUS_MASTER				0x538
#define INT_STATUS_MASTER_MISC				BIT(7)

#define INT_MASK_MASTER				0x53C
#define INT_MASK_MASTER_TIMEOUT			BIT(0)
#define INT_MASK_MASTER_ECC				BIT(1)
#define INT_MASK_MASTER_LOWPOWER			BIT(2)
#define INT_MASK_MASTER_PORT_TIMEOUT			BIT(3)
#define INT_MASK_MASTER_RFIFO_TIMEOUT			BIT(4)
#define INT_MASK_MASTER_TRAINING			BIT(5)
#define INT_MASK_MASTER_USERIF				BIT(6)
#define INT_MASK_MASTER_MISC				BIT(7)
#define INT_MASK_MASTER_BIST				BIT(8)
#define INT_MASK_MASTER_CRC				BIT(9)
#define INT_MASK_MASTER_DFI				BIT(10)
#define INT_MASK_MASTER_DIMM				BIT(11)
#define INT_MASK_MASTER_FREQ				BIT(12)
#define INT_MASK_MASTER_INIT				BIT(13)
#define INT_MASK_MASTER_MODE				BIT(14)
#define INT_MASK_MASTER_PARITY				BIT(15)

#define MASK_ALL_INT_GROUPS				(INT_MASK_MASTER_TIMEOUT |		\
							 INT_MASK_MASTER_ECC |			\
							 INT_MASK_MASTER_LOWPOWER |		\
							 INT_MASK_MASTER_PORT_TIMEOUT |		\
							 INT_MASK_MASTER_RFIFO_TIMEOUT |	\
							 INT_MASK_MASTER_TRAINING |		\
							 INT_MASK_MASTER_USERIF |		\
							 INT_MASK_MASTER_MISC |			\
							 INT_MASK_MASTER_BIST |			\
							 INT_MASK_MASTER_CRC |			\
							 INT_MASK_MASTER_DFI |			\
							 INT_MASK_MASTER_DIMM |			\
							 INT_MASK_MASTER_FREQ |			\
							 INT_MASK_MASTER_INIT |			\
							 INT_MASK_MASTER_MODE |			\
							 INT_MASK_MASTER_PARITY)
#define UNMASK_MISC_INT_GROUP				(MASK_ALL_INT_GROUPS &			\
							 ~(INT_MASK_MASTER_MISC))

#define INT_STATUS_MISC				0x554
#define INT_STATUS_MISC_TUF_BIT_SET			BIT(5)
#define INT_STATUS_MISC_TEMP_ALERT			BIT(6)

#define INT_ACK_MISC					0x574
#define INT_ACK_MISC_TUF_BIT_SET			BIT(5)
#define INT_ACK_MISC_TEMP_ALERT			BIT(6)

struct k3_ddr_dev {
	struct device *dev;
	struct device *hwmon_dev;
	struct work_struct intr_work;
	void __iomem *base;
	int irq;
	u32 auto_tempchk_val;
};

static const char *temp_status[] = { "low temperature\n",
				     "4x refresh interval\n",
				     "2x refresh interval\n",
				     "1x refresh interval\n",
				     "0.5x refresh interval\n",
				     "0.25x refresh interval\n",
				     "0.25x refresh interval with derating\n",
				     "high temperature\n" };

static void k3_ddr_temp_int_enable(struct k3_ddr_dev *ddr)
{
	writel_relaxed(UNMASK_MISC_INT_GROUP, ddr->base + INT_MASK_MASTER);
	writel_relaxed(MRR_TEMPCHK_NORM_THRESHOLD_F2_SET_VAL,
		       ddr->base + MRR_TEMPCHK_NORM_THRESHOLD_F2_OFFSET);
}

static void k3_ddr_temp_int_disable(struct k3_ddr_dev *ddr)
{
	writel_relaxed(MASK_ALL_INT_GROUPS, ddr->base + INT_MASK_MASTER);
	writel_relaxed(MRR_TEMPCHK_NORM_THRESHOLD_F2_DEFAULT_VAL,
		       ddr->base + MRR_TEMPCHK_NORM_THRESHOLD_F2_OFFSET);
}

static void k3_ddr_interrupt_status_notify(struct work_struct *ws)
{
	struct k3_ddr_dev *ddr = container_of(ws, struct k3_ddr_dev, intr_work);

	sysfs_notify(&ddr->hwmon_dev->kobj, NULL, "k3_ddr_interrupt_status");
	kobject_uevent(&ddr->hwmon_dev->kobj, KOBJ_CHANGE);
}

static ssize_t k3_ddr_interrupt_status_show(struct device *dev,
					    struct device_attribute *attr,
					    char *buf)
{
	struct k3_ddr_dev *ddr = dev_get_drvdata(dev);

	return sprintf(buf, "%s", temp_status[ddr->auto_tempchk_val]);
}

static irqreturn_t k3_ddr_irq_handler(int irq, void *dev_id)
{
	struct k3_ddr_dev *ddr = dev_id;

	if (!((readl(ddr->base + INT_STATUS_MASTER)) & INT_STATUS_MASTER_MISC))
		return IRQ_NONE;

	if ((readl(ddr->base + INT_STATUS_MISC)) &
	    INT_STATUS_MISC_TUF_BIT_SET) {
		ddr->auto_tempchk_val =
			(readl(ddr->base + AUTO_TEMPCHK_VAL_0_OFFSET) >>
			 AUTO_TEMPCHK_VAL_0_START_BIT) &
			AUTO_TEMPCHK_VAL_0_MR4_DATA_MASK;

		schedule_work(&ddr->intr_work);

		writel_relaxed(INT_ACK_MISC_TUF_BIT_SET,
			       ddr->base + INT_ACK_MISC);
	}
	if ((readl(ddr->base + INT_STATUS_MISC)) &
	    INT_STATUS_MISC_TEMP_ALERT) {
		ddr->auto_tempchk_val =
			(readl(ddr->base + AUTO_TEMPCHK_VAL_0_OFFSET) >>
			 AUTO_TEMPCHK_VAL_0_START_BIT) &
			AUTO_TEMPCHK_VAL_0_MR4_DATA_MASK;

		schedule_work(&ddr->intr_work);

		writel_relaxed(INT_ACK_MISC_TEMP_ALERT,
			       ddr->base + INT_ACK_MISC);
	}

	return IRQ_HANDLED;
}

static DEVICE_ATTR_RO(k3_ddr_interrupt_status);

static umode_t k3_ddr_is_visible(struct kobject *kobj, struct attribute *attr,
				 int index)
{
	return attr->mode;
}

static struct attribute *k3_ddr_attributes[] = {
	&dev_attr_k3_ddr_interrupt_status.attr, NULL
};

static const struct attribute_group k3_ddr_group = {
	.attrs = k3_ddr_attributes,
	.is_visible = k3_ddr_is_visible,
};

static const struct attribute_group *k3_ddr_groups[] = { &k3_ddr_group, NULL };

static const struct of_device_id k3_ddr_of_match[] = {
	{
		.compatible = "ti,am64-ddrss",
	},
	{},
};

static int k3_ddr_probe(struct platform_device *pdev)
{
	struct k3_ddr_dev *ddr;
	struct device *dev = &pdev->dev;
	int irq, r;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	ddr = devm_kzalloc(&pdev->dev, sizeof(struct k3_ddr_dev), GFP_KERNEL);
	if (!ddr)
		return -ENOMEM;

	ddr->base = devm_platform_ioremap_resource(pdev, 1);
	if (IS_ERR(ddr->base))
		return PTR_ERR(ddr->base);

	ddr->dev = &pdev->dev;
	ddr->irq = irq;

	k3_ddr_temp_int_enable(ddr);

	platform_set_drvdata(pdev, ddr);

	ddr->hwmon_dev = devm_hwmon_device_register_with_groups(
		dev, "k3_ddr", ddr, k3_ddr_groups);

	if (IS_ERR(ddr->hwmon_dev))
		return PTR_ERR(ddr->hwmon_dev);

	r = devm_request_irq(&pdev->dev, ddr->irq, k3_ddr_irq_handler,
			     IRQF_NO_SUSPEND, pdev->name, ddr);

	INIT_WORK(&ddr->intr_work, k3_ddr_interrupt_status_notify);

	return r;
}

static void k3_ddr_remove(struct platform_device *pdev)
{
	struct k3_ddr_dev *ddr = platform_get_drvdata(pdev);

	k3_ddr_temp_int_disable(ddr);
}

static struct platform_driver k3_ddr_driver = {
	.probe			= k3_ddr_probe,
	.remove_new		= k3_ddr_remove,
	.driver			= {
		.name		= "k3_ddr",
		.of_match_table = k3_ddr_of_match,
	},
};
module_platform_driver(k3_ddr_driver);

MODULE_DESCRIPTION("TI DDR Controller Driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Texas Instruments Inc");
