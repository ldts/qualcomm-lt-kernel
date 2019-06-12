 /*
 * Copyright (c) 2019, The Linux Foundation. All rights reserved.
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/cdev.h>
#include <linux/dma-mapping.h>
#include <linux/irqdomain.h>
#include <linux/interrupt.h>
#include <linux/irqreturn.h>
#include <linux/of.h>
#include <linux/mm.h>
#include <linux/irq.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/of_device.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/qcom_scm.h>

static int major;

struct qcom_tz_iface {
	struct device dev;
	struct cdev cdev;

	/* single lock for all processes attempting to send commands to TZ */
	struct mutex lock;
	bool panic_on_el3_irq;
	int irq;
};

struct qcom_tz_file {
	struct qcom_tz_iface *iface;
	struct tz_buffer {
	/*
	 * During the scm call memory protection will be enabled.
	 * Make sure it's physically contiguous, 4K aligned and
	 * non-cachable to avoid XPU violations.
	 */
		dma_addr_t phys;
		void *virt;
		size_t size;
	} buffer;
};

static ssize_t tz_usr_iface_read(struct file *file, char __user *user_buf,
				 size_t size, loff_t *pos)
{
	struct qcom_tz_iface *tz_if;
	struct qcom_tz_file *node;
	int ret;

	node = (struct qcom_tz_file *) file->private_data;
	tz_if = node->iface;

	if (size > node->buffer.size)
		return -EINVAL;

	mutex_lock(&tz_if->lock);
	memset(node->buffer.virt, 0x0, node->buffer.size);

	ret = qcom_scm_tz_read_req(node->buffer.phys, node->buffer.size);
	if (ret) {
		dev_err(&tz_if->dev, "tz read smc failed\n");
		mutex_unlock(&tz_if->lock);
		return -EIO;
	}

	ret = simple_read_from_buffer(user_buf, size, pos, node->buffer.virt,
				       node->buffer.size);
	if (ret < 0)
		dev_err(&tz_if->dev, "failed to read from buffer\n");

	mutex_unlock(&tz_if->lock);

	return ret;
}

static ssize_t tz_usr_iface_write(struct file *file, const char __user *user_buf,
				  size_t size, loff_t *pos)
{
	struct qcom_tz_iface *tz_if;
	struct qcom_tz_file *node;
	int ret;

	node = (struct qcom_tz_file *) file->private_data;
	tz_if = node->iface;

	if (size > node->buffer.size)
		return -EINVAL;

	mutex_lock(&tz_if->lock);
	memset(node->buffer.virt, 0x0, node->buffer.size);

	ret = simple_write_to_buffer(node->buffer.virt, node->buffer.size,
				     pos, user_buf, size);
	if (ret < 0) {
		dev_err(&tz_if->dev, "failed to write to buffer\n");
		mutex_unlock(&tz_if->lock);
		return -EIO;
	}

	ret = qcom_scm_tz_write_req(node->buffer.phys, node->buffer.size);
	if (!ret) {
		mutex_unlock(&tz_if->lock);
		return size;
	}

	dev_err(&tz_if->dev, "tz write smc failed\n");
	mutex_unlock(&tz_if->lock);

	return -EIO;
}

static int tz_usr_iface_open(struct inode *inode, struct file *file)
{
	struct qcom_tz_file *node;
	struct qcom_tz_iface *tz_if;

	tz_if = container_of(inode->i_cdev,struct qcom_tz_iface, cdev);
	get_device(&tz_if->dev);

	node = kzalloc(sizeof(struct qcom_tz_file), GFP_KERNEL);
	if (!node)
		return -ENOMEM;

	node->iface = tz_if;
	node->buffer.size = PAGE_SIZE;
	node->buffer.virt = dma_alloc_coherent(node->iface->dev.parent,
					       node->buffer.size,
					       &node->buffer.phys, GFP_KERNEL);
	if (!node->buffer.virt)
		return -ENOMEM;

	file->private_data = node;

	dev_dbg(&tz_if->dev,
		 "allocated virt = %p, bus = 0x%llx\n",
		 node->buffer.virt, node->buffer.phys);

	return 0;
}

static int tz_usr_iface_release(struct inode *inode, struct file *file)
{
	struct qcom_tz_file *node;

	node = (struct qcom_tz_file *) file->private_data;

	dma_free_coherent(node->iface->dev.parent, node->buffer.size,
			  node->buffer.virt, node->buffer.phys);

	dev_dbg(&node->iface->dev,
		 "released virt = %p, bus = 0x%llx\n",
		 node->buffer.virt, node->buffer.phys);

	kfree(node);

	return 0;
}

static const struct file_operations tz_usr_iface_fops = {
	.release = tz_usr_iface_release,
	.write = tz_usr_iface_write,
	.read = tz_usr_iface_read,
	.open = tz_usr_iface_open,
	.llseek = default_llseek,
};

static irqreturn_t el3_irq(int irq, void *data)
{
	struct qcom_tz_iface *tz_if = (struct qcom_tz_iface *) data;

	if (!tz_if)
		return IRQ_NONE;

	if (tz_if->panic_on_el3_irq)
		panic("EL3 IRQ: XPU Violation\n");
	else
		pr_emerg("EL3 IRQ: XPU Violation\n");

	return IRQ_HANDLED;
}

static const struct of_device_id qcom_tz_usr_iface_of_match[] = {
	{ .compatible = "qcom,tz-usr-iface" },
	{}
};

MODULE_DEVICE_TABLE(of, qcom_tz_usr_iface_of_match);

static void qcom_tz_user_iface_release_device(struct device *dev)
{
	struct qcom_tz_iface *priv = container_of(dev,
						  struct qcom_tz_iface, dev);
	kfree(priv);
}

static int qcom_tz_usr_iface_probe(struct platform_device *pdev)
{
	struct qcom_tz_iface *priv;
	struct device_node *np;
	struct irq_desc *desc;
	int ret;

	np = of_node_get(pdev->dev.of_node);
	if (!np) {
		dev_err(&pdev->dev, "OF node not found\n");
		return -ENOENT;
	}

	priv = kzalloc(sizeof(struct qcom_tz_iface), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	mutex_init(&priv->lock);

	device_initialize(&priv->dev);
	priv->dev.parent = &pdev->dev;
	priv->dev.id = 0;
	priv->dev.devt = MKDEV(MAJOR(major), priv->dev.id);
	priv->dev.release = qcom_tz_user_iface_release_device;
	dev_set_name(&priv->dev, "qcom_tz_mem");

	cdev_init(&priv->cdev, &tz_usr_iface_fops);
	priv->cdev.owner = THIS_MODULE;

	ret = cdev_device_add(&priv->cdev, &priv->dev);
	if (ret) {
		dev_err(&pdev->dev, "Failed to add cdev (%d)\n", ret);
		goto put_device;
	}

	priv->irq = of_irq_get(np, 0);
	if (priv->irq <= 0) {
		if (priv->irq != -EPROBE_DEFER)
			dev_err(&pdev->dev, "Unable to get the IRQ\n");
		ret = priv->irq;
		goto remove_device;
	}

	desc = irq_to_desc(priv->irq);
	if (!desc) {
		dev_err(&pdev->dev, "Error invalid irq descriptor\n");
		ret = -EINVAL;
		goto remove_device;
	}

	ret = qcm_scm_tz_register_irq(desc->irq_data.hwirq);
	if (ret) {
		dev_err(&pdev->dev, "Error registering hwirq %ld with TZ (%d)\n",
			desc->irq_data.hwirq, ret);
		goto remove_device;
	}

	ret = devm_request_irq(&pdev->dev, priv->irq, el3_irq,
			       IRQF_SHARED, "XPU Violation", priv);
	if (ret) {
		dev_err(&pdev->dev, "Error requesting irq %d (ret = %d)\n",
			priv->irq, ret);
		goto remove_device;
	}

	ret = of_property_read_bool(np, "qcom,panic-on-xpu-violation");
	if (ret)
		priv->panic_on_el3_irq = true;

	dev_info(&pdev->dev, "%s on XPU violations enabled\n",
		 priv->panic_on_el3_irq ? "panic" : "notify");

	dev_set_drvdata(&pdev->dev, priv);

	return 0;

remove_device:
	cdev_device_del(&priv->cdev, &priv->dev);
put_device:
	put_device(&priv->dev);

	return ret;
}

static int qcom_tz_usr_iface_remove(struct platform_device *pdev)
{
	struct qcom_tz_iface *priv = platform_get_drvdata(pdev);

	cdev_device_del(&priv->cdev, &priv->dev);
	put_device(&priv->dev);

	return 0;
}

static struct platform_driver qcom_tz_usr_iface_driver = {
	.remove = qcom_tz_usr_iface_remove,
	.probe = qcom_tz_usr_iface_probe,
	.driver  = {
		.of_match_table = qcom_tz_usr_iface_of_match,
		.name  = "qcom_tz_usr_iface",
	},
};

static int qcom_tz_user_iface_init(void)
{
	int ret;

	ret = alloc_chrdev_region(&major, 0, 1, "qcom-tz-usr-iface");
	if (ret < 0) {
		pr_err("failed to allocate char dev region\n");
		return ret;
	}

	ret = platform_driver_register(&qcom_tz_usr_iface_driver);
	if (ret < 0) {
		pr_err("failed to register tz driver\n");
		unregister_chrdev_region(major, 1);
	}

	return ret;
}
fs_initcall(qcom_tz_user_iface_init);

static void qcom_tz_user_iface_exit(void)
{
	platform_driver_unregister(&qcom_tz_usr_iface_driver);
	unregister_chrdev_region(major, 1);
}
module_exit(qcom_tz_user_iface_exit);

MODULE_AUTHOR("Linaro Ltd");
MODULE_DESCRIPTION("Qualcomm Trust Zone User Interface driver");
MODULE_LICENSE("GPL v2");
