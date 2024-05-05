// SPDX-License-Identifier: GPL-2.0-only

#include <linux/types.h>
#include <linux/io.h>
#include <linux/bits.h>
#include <linux/interrupt.h>
#include <linux/gpio/driver.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/of_irq.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/platform_device.h>
#include <linux/property.h>

#define AIROHA_GPIO_MAX		32

/* TODO: FIXME: Missing definitions from gpio/driver.h */
#define GPIO_LINE_DIRECTION_IN	1
#define GPIO_LINE_DIRECTION_OUT	0

/**
 * airoha_gpio_ctrl - Airoha GPIO driver data
 * @gc: Associated gpio_chip instance.
 * @data: The data register.
 * @dir0: The direction register for the lower 16 pins.
 * @dir1: The direction register for the higher 16 pins.
 * @output: The output enable register.
 * @irqstatus: The IRQ status register.
 * @level0: The level trigger register for the lower 16 pins.
 * @level1: The level trigger register for the higher 16 pins.
 * @edge0: The edge trigger register for the lower 16 pins.
 * @edge1: The edge trigger register for the higher 16 pins.
 * @gpio_irq: Shared interrupt number.
 * @lock: spinlock for interrupt handling.
 * @rising: Rising edge interrupt enabled.
 * @falling: Falling edge interrupt enabled.
 * @level_high: Level high interrupt enabled.
 * @level_low: Level low interrupt enabled.
 */
struct airoha_gpio_ctrl {
	/*
	 * Do not insert/reoder any fields starting from here.
	 * These fields are used by binary only module ecnt_legacy_api.ko
	 */
	struct gpio_chip gc;
	void __iomem *data;
	void __iomem *dir[2];
	void __iomem *output;

	/* fields maybe inserted/reodered starting from here */
	void __iomem *irqstatus;
	void __iomem *level[2];
	void __iomem *edge[2];

	int gpio_irq;
	spinlock_t lock;
	u32 rising;
	u32 falling;
	u32 level_high;
	u32 level_low;
};

/*
   This is for arht legacy gpio function support
 */
void __iomem *arht_gpio_ctrl = NULL;
void __iomem *arht_gpio_ctrl1 = NULL;
void __iomem *arht_gpio_data = NULL;
void __iomem *arht_gpio_oe_ctrl = NULL;

EXPORT_SYMBOL(arht_gpio_ctrl);
EXPORT_SYMBOL(arht_gpio_data);
EXPORT_SYMBOL(arht_gpio_oe_ctrl);
EXPORT_SYMBOL(arht_gpio_ctrl1);

static int arht_gpio_legacy_probe(struct airoha_gpio_ctrl *ctrl)
{
	arht_gpio_ctrl = ctrl->dir[0];
	arht_gpio_ctrl1 = ctrl->dir[1];
	arht_gpio_data = ctrl->data;
	arht_gpio_oe_ctrl = ctrl->output;

	return 0;
}

static struct airoha_gpio_ctrl *gc_to_ctrl(struct gpio_chip *gc)
{
	return container_of(gc, struct airoha_gpio_ctrl, gc);
}

static int airoha_dir_set(struct gpio_chip *gc, unsigned int gpio,
			  int val, int out)
{
	struct airoha_gpio_ctrl *ctrl = gc_to_ctrl(gc);
	unsigned long flags;
	u32 output;
	u32 dir;
	u32 shift = (gpio % 16) * 2;

	spin_lock_irqsave(&ctrl->lock, flags);

	output = ioread32(ctrl->output);
	dir = ioread32(ctrl->dir[gpio / 16]);

	dir &= ~(0x3 << shift);
	output &= ~BIT(gpio);
	if (out) {
		dir |= (0x1 << shift);
		output |= BIT(gpio);
	}

	iowrite32(dir, ctrl->dir[gpio / 16]);
	if (out)
		gc->set(gc, gpio, val);
	iowrite32(output, ctrl->output);

	spin_unlock_irqrestore(&ctrl->lock, flags);

	return 0;
}

static int airoha_dir_out(struct gpio_chip *gc, unsigned int gpio,
			  int val)
{
	return airoha_dir_set(gc, gpio, val, 1);
}

static int airoha_dir_in(struct gpio_chip *gc, unsigned int gpio)
{
	return airoha_dir_set(gc, gpio, 0, 0);
}

static int airoha_get_dir(struct gpio_chip *gc, unsigned int gpio)
{
	struct airoha_gpio_ctrl *ctrl = gc_to_ctrl(gc);
	u32 dir = ioread32(ctrl->dir[gpio / 16]);
	u32 shift = (gpio % 16) * 2;

	switch ((dir >> shift) & 0x03) {
	case 0:
		return GPIO_LINE_DIRECTION_IN;
	case 1:
		return GPIO_LINE_DIRECTION_OUT;
	default:
		break;
	}

	return -EINVAL;
}

static irqreturn_t airoha_gpio_irq_handler(int irq, void *data)
{
	struct gpio_chip *gc = data;
	struct airoha_gpio_ctrl *ctrl = gc_to_ctrl(gc);
	unsigned long pending;
	int bit;

	pending = ioread32(ctrl->irqstatus);
	if(!pending)
		return IRQ_NONE;

	for_each_set_bit(bit, &pending, AIROHA_GPIO_MAX) {
		u32 map = irq_find_mapping(gc->irq.domain, bit);

		generic_handle_irq(map);
		iowrite32(BIT(bit), ctrl->irqstatus);
	}

	return IRQ_HANDLED;
}

static void airoha_gpio_irq_unmask(struct irq_data *data)
{
	struct airoha_gpio_ctrl *ctrl = irq_data_get_irq_chip_data(data);
	int pin = data->hwirq;
	unsigned long flags;
	u32 tmp;
	u32 mask = BIT((pin % 16) * 2) | (BIT((pin % 16) * 2) << 1);

	spin_lock_irqsave(&ctrl->lock, flags);

	tmp = ioread32(ctrl->edge[(pin / 16) % 2]);
	tmp &= ~mask;
	if (BIT(pin) & ctrl->rising)
		tmp |= BIT((pin % 16) * 2);
	if (BIT(pin) & ctrl->falling)
		tmp |= (BIT((pin % 16) * 2)) << 1;
	iowrite32(tmp, ctrl->edge[(pin / 16) % 2]);

	tmp = ioread32(ctrl->level[(pin / 16) % 2]);
	tmp &= ~mask;
	if (BIT(pin) & ctrl->level_high)
		tmp |= BIT((pin % 16) * 2);
	if (BIT(pin) & ctrl->level_low)
		tmp |= (BIT((pin % 16) * 2)) << 1;
	iowrite32(tmp, ctrl->level[(pin / 16) % 2]);

	spin_unlock_irqrestore(&ctrl->lock, flags);
}

static void airoha_gpio_irq_mask(struct irq_data *data)
{
	struct airoha_gpio_ctrl *ctrl = irq_data_get_irq_chip_data(data);
	int pin = data->hwirq;
	unsigned long flags;
	u32 tmp;
	u32 mask = BIT((pin % 16) * 2) | (BIT((pin % 16) * 2) << 1);

	spin_lock_irqsave(&ctrl->lock, flags);

	tmp = ioread32(ctrl->edge[(pin / 16) % 2]);
	tmp &= ~mask;
	iowrite32(tmp, ctrl->edge[(pin / 16) % 2]);

	tmp = ioread32(ctrl->level[(pin / 16) % 2]);
	tmp &= ~mask;
	iowrite32(tmp, ctrl->level[(pin / 16) % 2]);

	spin_unlock_irqrestore(&ctrl->lock, flags);
}

static int airoha_gpio_irq_type(struct irq_data *data, unsigned int type)
{
	struct airoha_gpio_ctrl *ctrl = irq_data_get_irq_chip_data(data);
	int pin = data->hwirq;
	unsigned long flags;
	u32 mask = BIT(pin);

	spin_lock_irqsave(&ctrl->lock, flags);

	if (type == IRQ_TYPE_PROBE) {
		if ((ctrl->rising | ctrl->falling | ctrl->level_high | ctrl->level_low) & mask)
			goto out;

		type = IRQ_TYPE_EDGE_RISING | IRQ_TYPE_EDGE_FALLING;
	}

	ctrl->rising &= ~mask;
	ctrl->falling &= ~mask;
	ctrl->level_high &= ~mask;
	ctrl->level_low &= ~mask;

	switch (type & IRQ_TYPE_SENSE_MASK) {
	case IRQ_TYPE_EDGE_BOTH:
		ctrl->rising |= mask;
		ctrl->falling |= mask;
		break;
	case IRQ_TYPE_EDGE_RISING:
		ctrl->rising |= mask;
		break;
	case IRQ_TYPE_EDGE_FALLING:
		ctrl->falling |= mask;
		break;
	case IRQ_TYPE_LEVEL_HIGH:
		ctrl->level_high |= mask;
		break;
	case IRQ_TYPE_LEVEL_LOW:
		ctrl->level_low |= mask;
		break;
	}

out:
	spin_unlock_irqrestore(&ctrl->lock, flags);

	return 0;
}

static struct irq_chip en7523_irq_chip = {
	.name		= "en7523-gpio",
	.irq_unmask	= airoha_gpio_irq_unmask,
	.irq_mask	= airoha_gpio_irq_mask,
	.irq_mask_ack	= airoha_gpio_irq_mask,
	.irq_set_type	= airoha_gpio_irq_type,
	.flags		= IRQCHIP_SET_TYPE_MASKED,
};

static int airoha_gpio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct airoha_gpio_ctrl *ctrl;
	int err;
	static int gpio_base; /* Start indexing at 0 */

	ctrl = devm_kzalloc(dev, sizeof(*ctrl), GFP_KERNEL);
	if (!ctrl)
		return -ENOMEM;

	ctrl->data = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(ctrl->data))
		return PTR_ERR(ctrl->data);

	ctrl->dir[0] = devm_platform_ioremap_resource(pdev, 1);
	if (IS_ERR(ctrl->dir[0]))
		return PTR_ERR(ctrl->dir[0]);

	ctrl->dir[1] = devm_platform_ioremap_resource(pdev, 2);
	if (IS_ERR(ctrl->dir[1]))
		return PTR_ERR(ctrl->dir[1]);

	ctrl->output = devm_platform_ioremap_resource(pdev, 3);
	if (IS_ERR(ctrl->output))
		return PTR_ERR(ctrl->output);

	err = bgpio_init(&ctrl->gc, dev, 4, ctrl->data, NULL,
			 NULL, NULL, NULL, 0);
	if (err) {
		dev_err(dev, "unable to init generic GPIO");
		return err;
	}

	if (gpio_base == 0)
		arht_gpio_legacy_probe(ctrl);

	ctrl->gc.ngpio = AIROHA_GPIO_MAX;
	ctrl->gc.owner = THIS_MODULE;
	ctrl->gc.base = ctrl->gc.ngpio * gpio_base++;
	ctrl->gc.direction_output = airoha_dir_out;
	ctrl->gc.direction_input = airoha_dir_in;
	ctrl->gc.get_direction = airoha_get_dir;

	spin_lock_init(&ctrl->lock);

	if (of_find_property(np, "interrupt-controller", NULL)) {
		struct gpio_irq_chip *girq = NULL;
		int ret = platform_get_irq(pdev, 0);

		if (ret < 0) {
			dev_err(dev, "Error requesting IRQ %d\n", ret);
			return ret;
		}
		ctrl->gpio_irq = ret;

		ctrl->irqstatus = devm_platform_ioremap_resource(pdev, 4);
		if (IS_ERR(ctrl->irqstatus))
			return PTR_ERR(ctrl->irqstatus);

		ctrl->level[0] = devm_platform_ioremap_resource(pdev, 5);
		if (IS_ERR(ctrl->level[0]))
			return PTR_ERR(ctrl->level[0]);

		ctrl->level[1] = devm_platform_ioremap_resource(pdev, 6);
		if (IS_ERR(ctrl->level[1]))
			return PTR_ERR(ctrl->level[1]);

		ctrl->edge[0] = devm_platform_ioremap_resource(pdev, 7);
		if (IS_ERR(ctrl->edge[0]))
			return PTR_ERR(ctrl->edge[0]);

		ctrl->edge[1] = devm_platform_ioremap_resource(pdev, 8);
		if (IS_ERR(ctrl->edge[1]))
			return PTR_ERR(ctrl->edge[1]);

		/*
		 * Directly request the irq here instead of passing
		 * a flow-handler because the irq is shared.
		 */
		ret = devm_request_irq(dev, ctrl->gpio_irq, airoha_gpio_irq_handler, IRQF_SHARED, dev_name(dev), &ctrl->gc);
		if (ret) {
			dev_err(dev, "Error requesting IRQ %d: %d\n", ctrl->gpio_irq, ret);
			return ret;
		}

		girq = &ctrl->gc.irq;

		girq->chip = devm_kzalloc(dev, sizeof(en7523_irq_chip), GFP_KERNEL);
		if (!girq->chip)
			return -ENOMEM;
		memcpy(girq->chip, &en7523_irq_chip, sizeof(en7523_irq_chip));

		/* This will let us handle the parent IRQ in the driver */
		girq->parent_handler = NULL;
		girq->num_parents = 0;
		girq->parents = NULL;
		girq->default_type = IRQ_TYPE_NONE;
		girq->handler = handle_simple_irq;
	}

	return devm_gpiochip_add_data(dev, &ctrl->gc, ctrl);
}

static const struct of_device_id airoha_gpio_of_match[] = {
	{ .compatible = "airoha,en7523-gpio" },
	{ .compatible = "airoha,airoha-gpio" },
	{ }
};
MODULE_DEVICE_TABLE(of, airoha_gpio_of_match);

static struct platform_driver airoha_gpio_driver = {
	.driver = {
		.name = "airoha-gpio",
		.of_match_table	= airoha_gpio_of_match,
	},
	.probe = airoha_gpio_probe,
};
module_platform_driver(airoha_gpio_driver);

MODULE_DESCRIPTION("Airoha GPIO support");
MODULE_AUTHOR("John Crispin <john@phrozen.org>");
MODULE_LICENSE("GPL v2");
