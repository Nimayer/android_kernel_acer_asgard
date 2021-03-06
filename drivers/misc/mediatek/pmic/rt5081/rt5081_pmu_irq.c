/*
 *  Copyright (C) 2016 Richtek Technology Corp.
 *  cy_huang <cy_huang@richtek.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/interrupt.h>

#include "inc/rt5081_pmu.h"

#define RT5081_PMU_IRQ_EVT_MAX (128)

struct irq_indicator_bit {
	uint8_t start;
	uint8_t size;
};

struct irq_mapping_tbl {
	const char *name;
	const int id;
};

/* indicator 8 bits */
static const struct irq_indicator_bit irq_ind_bits[8] = {
	{ 0, 0},
	{ 15, 1},
	{ 14, 1},
	{ 13, 1},
	{ 12, 1},
	{ 11, 1},
	{ 9, 2},
	{ 0, 9},
};

#define RT5081_PMU_IRQ_MAPPING(_name, _id) { .name = #_name, .id = _id}
static const struct irq_mapping_tbl rt5081_pmu_irq_mapping_tbl[] = {
	RT5081_PMU_IRQ_MAPPING(chg_treg, 4),
	RT5081_PMU_IRQ_MAPPING(chg_aicr, 5),
	RT5081_PMU_IRQ_MAPPING(chg_mivr, 6),
	RT5081_PMU_IRQ_MAPPING(pwr_rdy, 7),
	RT5081_PMU_IRQ_MAPPING(chg_vinovp, 11),
	RT5081_PMU_IRQ_MAPPING(chg_vsysuv, 12),
	RT5081_PMU_IRQ_MAPPING(chg_vsysov, 13),
	RT5081_PMU_IRQ_MAPPING(chg_vbatov, 14),
	RT5081_PMU_IRQ_MAPPING(chg_vbusov, 15),
	RT5081_PMU_IRQ_MAPPING(ts_bat_cold, 20),
	RT5081_PMU_IRQ_MAPPING(ts_bat_cool, 21),
	RT5081_PMU_IRQ_MAPPING(ts_bat_warm, 22),
	RT5081_PMU_IRQ_MAPPING(ts_bat_hot, 23),
	RT5081_PMU_IRQ_MAPPING(chg_tmri, 27),
	RT5081_PMU_IRQ_MAPPING(chg_batabsi, 28),
	RT5081_PMU_IRQ_MAPPING(chg_adpbadi, 29),
	RT5081_PMU_IRQ_MAPPING(chg_rvpi, 30),
	RT5081_PMU_IRQ_MAPPING(otpi, 31),
	RT5081_PMU_IRQ_MAPPING(chg_aiclmeasi, 32),
	RT5081_PMU_IRQ_MAPPING(chg_ichgmeasi, 33),
	RT5081_PMU_IRQ_MAPPING(chgdet_donei, 34),
	RT5081_PMU_IRQ_MAPPING(chg_wdtmri, 35),
	RT5081_PMU_IRQ_MAPPING(ssfinishi, 36),
	RT5081_PMU_IRQ_MAPPING(chg_rechgi, 37),
	RT5081_PMU_IRQ_MAPPING(chg_termi, 38),
	RT5081_PMU_IRQ_MAPPING(chg_ieoci, 39),
	RT5081_PMU_IRQ_MAPPING(adc_donei, 40),
	RT5081_PMU_IRQ_MAPPING(pumpx_donei, 41),
	RT5081_PMU_IRQ_MAPPING(bst_batuvi, 45),
	RT5081_PMU_IRQ_MAPPING(bst_vbusovi, 46),
	RT5081_PMU_IRQ_MAPPING(bst_olpi, 47),
	RT5081_PMU_IRQ_MAPPING(attachi, 48),
	RT5081_PMU_IRQ_MAPPING(detachi, 49),
	RT5081_PMU_IRQ_MAPPING(qc30stpdone, 51),
	RT5081_PMU_IRQ_MAPPING(qc_vbusdet_done, 52),
	RT5081_PMU_IRQ_MAPPING(hvdcp_det, 53),
	RT5081_PMU_IRQ_MAPPING(chgdeti, 54),
	RT5081_PMU_IRQ_MAPPING(dcdti, 55),
	RT5081_PMU_IRQ_MAPPING(dirchg_vgoki, 59),
	RT5081_PMU_IRQ_MAPPING(dirchg_wdtmri, 60),
	RT5081_PMU_IRQ_MAPPING(dirchg_uci, 61),
	RT5081_PMU_IRQ_MAPPING(dirchg_oci, 62),
	RT5081_PMU_IRQ_MAPPING(dirchg_ovi, 63),
	RT5081_PMU_IRQ_MAPPING(ovpctrl_swon_evt, 67),
	RT5081_PMU_IRQ_MAPPING(ovpctrl_uvp_d_evt, 68),
	RT5081_PMU_IRQ_MAPPING(ovpctrl_uvp_evt, 69),
	RT5081_PMU_IRQ_MAPPING(ovpctrl_ovp_d_evt, 70),
	RT5081_PMU_IRQ_MAPPING(ovpctrl_ovp_evt, 71),
	RT5081_PMU_IRQ_MAPPING(fled_strbpin, 72),
	RT5081_PMU_IRQ_MAPPING(fled_torpin, 73),
	RT5081_PMU_IRQ_MAPPING(fled_tx, 74),
	RT5081_PMU_IRQ_MAPPING(fled_lvf, 75),
	RT5081_PMU_IRQ_MAPPING(fled2_short, 78),
	RT5081_PMU_IRQ_MAPPING(fled1_short, 79),
	RT5081_PMU_IRQ_MAPPING(fled2_strb, 80),
	RT5081_PMU_IRQ_MAPPING(fled1_strb, 81),
	RT5081_PMU_IRQ_MAPPING(fled2_strb_to, 82),
	RT5081_PMU_IRQ_MAPPING(fled1_strb_to, 83),
	RT5081_PMU_IRQ_MAPPING(fled2_tor, 84),
	RT5081_PMU_IRQ_MAPPING(fled1_tor, 85),
	RT5081_PMU_IRQ_MAPPING(otp, 93),
	RT5081_PMU_IRQ_MAPPING(vdda_ovp, 94),
	RT5081_PMU_IRQ_MAPPING(vdda_uv, 95),
	RT5081_PMU_IRQ_MAPPING(ldo_oc, 103),
	RT5081_PMU_IRQ_MAPPING(isink4_short, 104),
	RT5081_PMU_IRQ_MAPPING(isink3_short, 105),
	RT5081_PMU_IRQ_MAPPING(isink2_short, 106),
	RT5081_PMU_IRQ_MAPPING(isink1_short, 107),
	RT5081_PMU_IRQ_MAPPING(isink4_open, 108),
	RT5081_PMU_IRQ_MAPPING(isink3_open, 109),
	RT5081_PMU_IRQ_MAPPING(isink2_open, 110),
	RT5081_PMU_IRQ_MAPPING(isink1_open, 111),
	RT5081_PMU_IRQ_MAPPING(bled_ocp, 118),
	RT5081_PMU_IRQ_MAPPING(bled_ovp, 119),
	RT5081_PMU_IRQ_MAPPING(dsv_vneg_ocp, 123),
	RT5081_PMU_IRQ_MAPPING(dsv_vpos_ocp, 124),
	RT5081_PMU_IRQ_MAPPING(dsv_bst_ocp, 125),
	RT5081_PMU_IRQ_MAPPING(dsv_vneg_scp, 126),
	RT5081_PMU_IRQ_MAPPING(dsv_vpos_scp, 127),
};

static uint8_t rt5081_pmu_curr_irqmask[16];

static void rt5081_pmu_irq_bus_lock(struct irq_data *data)
{
	struct rt5081_pmu_chip *chip = data->chip_data;
	int ret = 0;

	ret = rt5081_pmu_reg_block_read(chip,
					RT5081_PMU_CHGMASK1, 16,
					rt5081_pmu_curr_irqmask);
	if (ret < 0)
		dev_err(chip->dev, "%s: read irq mask fail\n", __func__);
}

static void rt5081_pmu_irq_bus_sync_unlock(struct irq_data *data)
{
	struct rt5081_pmu_chip *chip = data->chip_data;
	int ret = 0;

	ret = rt5081_pmu_reg_block_write(chip,
					 RT5081_PMU_CHGMASK1, 16,
					 rt5081_pmu_curr_irqmask);
	if (ret < 0)
		dev_err(chip->dev, "%s: write irq mask fail\n", __func__);
}

static void rt5081_pmu_irq_mask(struct irq_data *data)
{
	struct rt5081_pmu_chip *chip = data->chip_data;

	dev_dbg(chip->dev, "%s: hwirq = %d\n", __func__, (int)data->hwirq);
	rt5081_pmu_curr_irqmask[data->hwirq / 8] |= (1 << (data->hwirq % 8));
}

static void rt5081_pmu_irq_unmask(struct irq_data *data)
{
	struct rt5081_pmu_chip *chip = data->chip_data;

	dev_dbg(chip->dev, "%s: hwirq = %d\n", __func__, (int)data->hwirq);
	rt5081_pmu_curr_irqmask[data->hwirq / 8] &= ~(1 << (data->hwirq % 8));
}

static struct irq_chip rt5081_pmu_irq_chip = {
	.name = "rt5081_pmu_irq",
	.irq_bus_lock = rt5081_pmu_irq_bus_lock,
	.irq_bus_sync_unlock = rt5081_pmu_irq_bus_sync_unlock,
	.irq_mask = rt5081_pmu_irq_mask,
	.irq_unmask = rt5081_pmu_irq_unmask,
};

static int rt5081_pmu_irq_domain_map(struct irq_domain *d, unsigned int virq,
				     irq_hw_number_t hw)
{
	struct rt5081_pmu_chip *chip = d->host_data;

	if (hw >= RT5081_PMU_IRQ_EVT_MAX)
		return -EINVAL;
	irq_set_chip_data(virq, chip);
	irq_set_chip_and_handler(virq, &rt5081_pmu_irq_chip, handle_simple_irq);
	irq_set_nested_thread(virq, true);
#if 1 /*(LINUX_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)) */
	irq_set_parent(virq, chip->irq);
#endif
#ifdef CONFIG_ARM
	set_irq_flags(virq, IRQF_VALID);
#else
	irq_set_noprobe(virq);
#endif /* #ifdef CONFIG_ARM */
	return 0;
}

static void rt5081_pmu_irq_domain_unmap(struct irq_domain *d, unsigned int virq)
{
#ifdef CONFIG_ARM
	set_irq_flags(virq, 0);
#endif /* #ifdef CONFIG_ARM */
	irq_set_chip_and_handler(virq, NULL, NULL);
	irq_set_chip_data(virq, NULL);
}

static const struct irq_domain_ops rt5081_pmu_irq_domain_ops = {
	.map = rt5081_pmu_irq_domain_map,
	.unmap = rt5081_pmu_irq_domain_unmap,
	.xlate = irq_domain_xlate_onetwocell,
};

static irqreturn_t rt5081_pmu_irq_handler(int irq, void *priv)
{
	struct rt5081_pmu_chip *chip = (struct rt5081_pmu_chip *)priv;
	u8 irq_ind = 0, data[16] = {0}, mask[16] = {0};
	int i = 0, j = 0, ret = 0;

	dev_dbg(chip->dev, "%s\n", __func__);
	pm_runtime_get_sync(chip->dev);
	ret = rt5081_pmu_reg_read(chip, RT5081_PMU_REG_IRQIND);
	if (ret < 0) {
		dev_err(chip->dev, "read irq indicator fail\n");
		goto out_irq_handler;
	}
	irq_ind = ret;
	ret = rt5081_pmu_reg_block_read(chip,
					RT5081_PMU_REG_CHGIRQ1, 16, data);
	if (ret < 0) {
		dev_err(chip->dev, "read irq event fail\n");
		goto out_irq_handler;
	}
	ret = rt5081_pmu_reg_block_read(chip,
					RT5081_PMU_CHGMASK1, 16, mask);
	if (ret < 0) {
		dev_err(chip->dev, "read irq mask fail\n");
		goto out_irq_handler;
	}
	for (i = 0; i < 8; i++) {
		if (irq_ind_bits[i].size <= 0)
			continue;
		if (!(irq_ind & (1 << i)))
			memset(data + irq_ind_bits[i].start,
			       0, irq_ind_bits[i].size);
	}
	for (i = 0; i < 16; i++) {
		data[i] &= ~mask[i];
		if (!data[i])
			continue;
		for (j = 0; j < 8; j++) {
			if (!(data[i] & (1 << j)))
				continue;
			ret = irq_find_mapping(chip->irq_domain, j);
			if (ret)
				handle_nested_irq(ret);
			else
				dev_err(chip->dev, "unmapped %d %d\n", i, j);
		}
	}
out_irq_handler:
	pm_runtime_put(chip->dev);
	return IRQ_HANDLED;
}

static int rt5081_pmu_irq_init(struct rt5081_pmu_chip *chip)
{
	u8 data[16] = {0};
	int ret = 0;

	/* mask all */
	ret = rt5081_pmu_reg_write(chip, RT5081_PMU_REG_IRQMASK, 0xfe);
	if (ret < 0)
		return ret;
	memset(data, 0xff, 16);
	ret = rt5081_pmu_reg_block_write(chip, RT5081_PMU_CHGMASK1, 16, data);
	if (ret < 0)
		return ret;
	ret = rt5081_pmu_reg_block_read(chip,
					RT5081_PMU_REG_CHGIRQ1, 16, data);
	if (ret < 0)
		return ret;
	/* re-enable all, but the all part irq event masks are enabled */
	ret = rt5081_pmu_reg_write(chip, RT5081_PMU_REG_IRQMASK, 0x00);
	if (ret < 0)
		return ret;
	return 0;
}

void rt5081_pmu_irq_suspend(struct rt5081_pmu_chip *chip)
{
	if (device_may_wakeup(chip->dev))
		enable_irq_wake(chip->irq);
}
EXPORT_SYMBOL(rt5081_pmu_irq_suspend);

void rt5081_pmu_irq_resume(struct rt5081_pmu_chip *chip)
{
	if (device_may_wakeup(chip->dev))
		disable_irq_wake(chip->irq);
}
EXPORT_SYMBOL(rt5081_pmu_irq_resume);

int rt5081_pmu_get_virq_number(struct rt5081_pmu_chip *chip, const char *name)
{
	int i;

	if (!name) {
		dev_err(chip->dev, "%s: null name\n", __func__);
		return -EINVAL;
	}
	for (i = 0; i < ARRAY_SIZE(rt5081_pmu_irq_mapping_tbl); i++) {
		if (!strcmp(rt5081_pmu_irq_mapping_tbl[i].name, name)) {
			return irq_create_mapping(chip->irq_domain,
					rt5081_pmu_irq_mapping_tbl[i].id);
		}
	}
	return -EINVAL;
}
EXPORT_SYMBOL(rt5081_pmu_get_virq_number);

int rt5081_pmu_irq_register(struct rt5081_pmu_chip *chip)
{
	struct rt5081_pmu_platform_data *pdata = dev_get_platdata(chip->dev);
	int ret = 0;

	ret = rt5081_pmu_irq_init(chip);
	if (ret < 0)
		return ret;
	ret = gpio_request_one(pdata->intr_gpio, GPIOF_IN,
			       "rt5081_pmu_irq_gpio");
	if (ret < 0) {
		dev_err(chip->dev, "%s: gpio request fail\n", __func__);
		return ret;
	}
	ret = gpio_to_irq(pdata->intr_gpio);
	if (ret < 0) {
		dev_err(chip->dev, "%s: irq mapping fail\n", __func__);
		goto out_pmu_irq;
	}
	chip->irq_domain = irq_domain_add_linear(chip->dev->of_node,
						 RT5081_PMU_IRQ_EVT_MAX,
						 &rt5081_pmu_irq_domain_ops,
						 chip);
	if (!chip->irq_domain) {
		dev_err(chip->dev, "irq domain register fail\n");
		ret = -EINVAL;
		goto out_pmu_irq;
	}
	chip->irq = ret;
	ret = devm_request_threaded_irq(chip->dev, chip->irq, NULL,
		rt5081_pmu_irq_handler, IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
		"rt5081_pmu_irq", chip);
	if (ret < 0)
		goto out_pmu_irq;
	device_init_wakeup(chip->dev, true);
	return 0;
out_pmu_irq:
	gpio_free(pdata->intr_gpio);
	return ret;
}
EXPORT_SYMBOL(rt5081_pmu_irq_register);

void rt5081_pmu_irq_unregister(struct rt5081_pmu_chip *chip)
{
	struct rt5081_pmu_platform_data *pdata = dev_get_platdata(chip->dev);

	device_init_wakeup(chip->dev, false);
	devm_free_irq(chip->dev, chip->irq, chip);
#if 1 /*(LINUX_VERSION_CODE >= KERNEL_VERSION(3, 5, 0)) */
	irq_domain_remove(chip->irq_domain);
#endif
	gpio_free(pdata->intr_gpio);
}
EXPORT_SYMBOL(rt5081_pmu_irq_unregister);
