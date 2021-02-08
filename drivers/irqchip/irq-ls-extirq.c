// SPDX-License-Identifier: GPL-2.0

#define pr_fmt(fmt) "irq-ls-extirq: " fmt

#include <linux/irq.h>
#include <linux/irqchip.h>
#include <linux/irqdomain.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <dt-bindings/interrupt-controller/arm-gic.h>

#define MAXIRQ 12

struct extirq_chip_data {
	struct regmap *syscon;
	u32           intpcr;
	bool          bit_reverse;
	u32           nirq;
	u32           parent_irq[MAXIRQ];
};

static int
ls_extirq_set_type(struct irq_data *data, unsigned int type)
{
	struct extirq_chip_data *chip_data = data->chip_data;
	irq_hw_number_t hwirq = data->hwirq;
	u32 value, mask;

	if (chip_data->bit_reverse)
		mask = 1U << (31 - hwirq);
	else
		mask = 1U << hwirq;

	switch (type) {
	case IRQ_TYPE_LEVEL_LOW:
		type = IRQ_TYPE_LEVEL_HIGH;
		value = mask;
		break;
	case IRQ_TYPE_EDGE_FALLING:
		type = IRQ_TYPE_EDGE_RISING;
		value = mask;
		break;
	case IRQ_TYPE_LEVEL_HIGH:
	case IRQ_TYPE_EDGE_RISING:
		value = 0;
		break;
	default:
		return -EINVAL;
	}

	regmap_update_bits(chip_data->syscon, chip_data->intpcr, mask, value);

	data = data->parent_data;
	return data->chip->irq_set_type(data, type);
}

static struct irq_chip extirq_chip = {
	.name			= "extirq",
	.irq_mask		= irq_chip_mask_parent,
	.irq_unmask		= irq_chip_unmask_parent,
	.irq_eoi		= irq_chip_eoi_parent,
	.irq_set_type		= ls_extirq_set_type,
	.irq_retrigger		= irq_chip_retrigger_hierarchy,
	.irq_set_affinity	= irq_chip_set_affinity_parent,
	.flags                  = IRQCHIP_SET_TYPE_MASKED,
};

static int
ls_extirq_domain_translate(struct irq_domain *d, struct irq_fwspec *fwspec,
			   unsigned long *hwirq, unsigned int *type)
{
	if (!is_of_node(fwspec->fwnode))
		return -EINVAL;

	if (fwspec->param_count != 2)
		return -EINVAL;

	*hwirq = fwspec->param[0];
	*type = fwspec->param[1] & IRQ_TYPE_SENSE_MASK;
	return 0;
}

static int
ls_extirq_domain_alloc(struct irq_domain *domain, unsigned int virq,
		       unsigned int nr_irqs, void *arg)
{
	irq_hw_number_t hwirq;
	struct irq_fwspec *fwspec = arg;
	struct irq_fwspec gic_fwspec;
	struct extirq_chip_data *chip_data = domain->host_data;

	if (fwspec->param_count != 2)
		return -EINVAL;

	hwirq = fwspec->param[0];
	if (hwirq >= chip_data->nirq)
		return -EINVAL;

	irq_domain_set_hwirq_and_chip(domain, virq, hwirq, &extirq_chip,
				      chip_data);

	gic_fwspec.fwnode = domain->parent->fwnode;
	gic_fwspec.param_count = 3;
	gic_fwspec.param[0] = GIC_SPI;
	gic_fwspec.param[1] = chip_data->parent_irq[hwirq];
	gic_fwspec.param[2] = fwspec->param[1];

	return irq_domain_alloc_irqs_parent(domain, virq, 1, &gic_fwspec);
}

static const struct irq_domain_ops extirq_domain_ops = {
	.translate	= ls_extirq_domain_translate,
	.alloc		= ls_extirq_domain_alloc,
	.free		= irq_domain_free_irqs_common,
};

static int __init
ls_extirq_of_init(struct device_node *node, struct device_node *parent)
{

	struct irq_domain *domain, *domain_parent;
	struct extirq_chip_data *chip;
	const __be32 *intpcr;
	int ret;

	domain_parent = irq_find_host(parent);
	if (!domain_parent) {
		pr_err("interrupt-parent not found\n");
		return -EINVAL;
	}

	chip = kzalloc(sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->syscon = syscon_node_to_regmap(node->parent);
	if (IS_ERR(chip->syscon)) {
		ret = PTR_ERR(chip->syscon);
		pr_err("Failed to lookup parent regmap\n");
		goto err;
	}
	intpcr = of_get_address(node, 0, NULL, NULL);
	if (!intpcr) {
		ret = -ENOENT;
		pr_err("Missing INTPCR offset value\n");
		goto err;
	}
	chip->intpcr = __be32_to_cpu(*intpcr);

	ret = of_property_read_variable_u32_array(node, "fsl,extirq-map",
						  chip->parent_irq,
						  1, ARRAY_SIZE(chip->parent_irq));
	if (ret < 0)
		goto err;
	chip->nirq = ret;
	chip->bit_reverse = of_property_read_bool(node, "fsl,bit-reverse");

	domain = irq_domain_add_hierarchy(domain_parent, 0, chip->nirq, node,
					  &extirq_domain_ops, chip);
	if (!domain) {
		ret = -ENOMEM;
		goto err;
	}

	return 0;

err:
	kfree(chip);
	return ret;
}

IRQCHIP_DECLARE(ls1021a_extirq, "fsl,ls1021a-extirq", ls_extirq_of_init);
