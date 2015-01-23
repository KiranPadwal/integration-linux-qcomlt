/*
 * Copyright (c) 2011-2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Qualcomm's PM8921/PM8018 ADC Arbiter driver
 */
#define pr_fmt(fmt) "%s: " fmt, __func__

#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/hwmon.h>
#include <linux/module.h>
#include <linux/debugfs.h>
#include <linux/interrupt.h>
#include <linux/completion.h>
#include <linux/hwmon-sysfs.h>
#include <linux/platform_device.h>
#include <linux/mfd/pm8xxx-adc.h>
#include <linux/regmap.h>
#include <linux/of.h>

/* User Bank register set */
#define PM8XXX_ADC_ARB_USRP_CNTRL1			0x197
#define PM8XXX_ADC_ARB_USRP_CNTRL1_EN_ARB		BIT(0)
#define PM8XXX_ADC_ARB_USRP_CNTRL1_RSV1			BIT(1)
#define PM8XXX_ADC_ARB_USRP_CNTRL1_RSV2			BIT(2)
#define PM8XXX_ADC_ARB_USRP_CNTRL1_RSV3			BIT(3)
#define PM8XXX_ADC_ARB_USRP_CNTRL1_RSV4			BIT(4)
#define PM8XXX_ADC_ARB_USRP_CNTRL1_RSV5			BIT(5)
#define PM8XXX_ADC_ARB_USRP_CNTRL1_EOC			BIT(6)
#define PM8XXX_ADC_ARB_USRP_CNTRL1_REQ			BIT(7)

#define PM8XXX_ADC_ARB_USRP_AMUX_CNTRL			0x198
#define PM8XXX_ADC_ARB_USRP_AMUX_CNTRL_RSV0		BIT(0)
#define PM8XXX_ADC_ARB_USRP_AMUX_CNTRL_RSV1		BIT(1)
#define PM8XXX_ADC_ARB_USRP_AMUX_CNTRL_PREMUX0		BIT(2)
#define PM8XXX_ADC_ARB_USRP_AMUX_CNTRL_PREMUX1		BIT(3)
#define PM8XXX_ADC_ARB_USRP_AMUX_CNTRL_SEL0		BIT(4)
#define PM8XXX_ADC_ARB_USRP_AMUX_CNTRL_SEL1		BIT(5)
#define PM8XXX_ADC_ARB_USRP_AMUX_CNTRL_SEL2		BIT(6)
#define PM8XXX_ADC_ARB_USRP_AMUX_CNTRL_SEL3		BIT(7)

#define PM8XXX_ADC_ARB_USRP_ANA_PARAM			0x199
#define PM8XXX_ADC_ARB_USRP_DIG_PARAM			0x19A
#define PM8XXX_ADC_ARB_USRP_DIG_PARAM_SEL_SHIFT0	BIT(0)
#define PM8XXX_ADC_ARB_USRP_DIG_PARAM_SEL_SHIFT1	BIT(1)
#define PM8XXX_ADC_ARB_USRP_DIG_PARAM_CLK_RATE0		BIT(2)
#define PM8XXX_ADC_ARB_USRP_DIG_PARAM_CLK_RATE1		BIT(3)
#define PM8XXX_ADC_ARB_USRP_DIG_PARAM_EOC		BIT(4)
#define PM8XXX_ADC_ARB_USRP_DIG_PARAM_DEC_RATE0		BIT(5)
#define PM8XXX_ADC_ARB_USRP_DIG_PARAM_DEC_RATE1		BIT(6)
#define PM8XXX_ADC_ARB_USRP_DIG_PARAM_EN		BIT(7)

#define PM8XXX_ADC_ARB_USRP_RSV				0x19B
#define PM8XXX_ADC_ARB_USRP_RSV_RST			BIT(0)
#define PM8XXX_ADC_ARB_USRP_RSV_DTEST0			BIT(1)
#define PM8XXX_ADC_ARB_USRP_RSV_DTEST1			BIT(2)
#define PM8XXX_ADC_ARB_USRP_RSV_OP			BIT(3)
#define PM8XXX_ADC_ARB_USRP_RSV_IP_SEL0			BIT(4)
#define PM8XXX_ADC_ARB_USRP_RSV_IP_SEL1			BIT(5)
#define PM8XXX_ADC_ARB_USRP_RSV_IP_SEL2			BIT(6)
#define PM8XXX_ADC_ARB_USRP_RSV_TRM			BIT(7)

#define PM8XXX_ADC_ARB_USRP_DATA0			0x19D
#define PM8XXX_ADC_ARB_USRP_DATA1			0x19C

#define PM8XXX_ADC_ARB_BTM_CNTRL1			0x17e
#define PM8XXX_ADC_ARB_BTM_CNTRL1_EN_BTM		BIT(0)
#define PM8XXX_ADC_ARB_BTM_CNTRL1_SEL_OP_MODE		BIT(1)
#define PM8XXX_ADC_ARB_BTM_CNTRL1_MEAS_INTERVAL1	BIT(2)
#define PM8XXX_ADC_ARB_BTM_CNTRL1_MEAS_INTERVAL2	BIT(3)
#define PM8XXX_ADC_ARB_BTM_CNTRL1_MEAS_INTERVAL3	BIT(4)
#define PM8XXX_ADC_ARB_BTM_CNTRL1_MEAS_INTERVAL4	BIT(5)
#define PM8XXX_ADC_ARB_BTM_CNTRL1_EOC			BIT(6)
#define PM8XXX_ADC_ARB_BTM_CNTRL1_REQ			BIT(7)

#define PM8XXX_ADC_ARB_BTM_CNTRL2			0x18c
#define PM8XXX_ADC_ARB_BTM_AMUX_CNTRL			0x17f
#define PM8XXX_ADC_ARB_BTM_ANA_PARAM			0x180
#define PM8XXX_ADC_ARB_BTM_DIG_PARAM			0x181
#define PM8XXX_ADC_ARB_BTM_RSV				0x182
#define PM8XXX_ADC_ARB_BTM_DATA1			0x183
#define PM8XXX_ADC_ARB_BTM_DATA0			0x184
#define PM8XXX_ADC_ARB_BTM_BAT_COOL_THR1		0x185
#define PM8XXX_ADC_ARB_BTM_BAT_COOL_THR0		0x186
#define PM8XXX_ADC_ARB_BTM_BAT_WARM_THR1		0x187
#define PM8XXX_ADC_ARB_BTM_BAT_WARM_THR0		0x188

#define PM8XXX_ADC_ARB_ANA_DIG				0xa0
#define PM8XXX_ADC_BTM_RSV				0x10
#define PM8XXX_ADC_AMUX_MPP_SEL				2
#define PM8XXX_ADC_AMUX_SEL				4
#define PM8XXX_ADC_RSV_IP_SEL				4
#define PM8XXX_ADC_BTM_CHANNEL_SEL			4
#define PM8XXX_MAX_CHANNEL_PROPERTIES			2
#define PM8XXX_ADC_IRQ_0				0
#define PM8XXX_ADC_IRQ_1				1
#define PM8XXX_ADC_IRQ_2				2
#define PM8XXX_ADC_BTM_INTERVAL_SEL_MASK		0xF
#define PM8XXX_ADC_BTM_INTERVAL_SEL_SHIFT		2
#define PM8XXX_ADC_BTM_DECIMATION_SEL			5
#define PM8XXX_ADC_MUL					10
#define PM8XXX_ADC_CONV_TIME_MIN			2000
#define PM8XXX_ADC_CONV_TIME_MAX			2100
#define PM8XXX_ADC_MPP_SETTLE_TIME_MIN			200
#define PM8XXX_ADC_MPP_SETTLE_TIME_MAX			200
#define PM8XXX_ADC_PA_THERM_VREG_UV_MIN			1800000
#define PM8XXX_ADC_PA_THERM_VREG_UV_MAX			1800000
#define PM8XXX_ADC_PA_THERM_VREG_UA_LOAD		100000
#define PM8XXX_ADC_HWMON_NAME_LENGTH			32
#define PM8XXX_ADC_BTM_INTERVAL_MAX			0x14
#define PM8XXX_ADC_COMPLETION_TIMEOUT			(2 * HZ)

struct pm8xxx_adc {
	struct device				*dev;
	struct pm8xxx_adc_properties		*adc_prop;
	int					adc_irq;
	struct mutex				adc_lock;
	struct regmap				*regmap;
	uint32_t				adc_num_board_channel;
	struct completion			adc_rslt_completion;
	struct pm8xxx_adc_amux			*adc_channel;
	struct dentry				*dent;
	uint32_t				mpp_base;
	struct device				*hwmon;
	//struct msm_xo_voter			*adc_voter;
	int					msm_suspend_check;
	struct pm8xxx_adc_amux_properties	*conv;
	struct sensor_device_attribute		sens_attr[0];
};

struct pm8xxx_adc_amux_properties {
	uint32_t				amux_channel;
	uint32_t				decimation;
	uint32_t				amux_ip_rsv;
	uint32_t				amux_mpp_channel;
	struct pm8xxx_adc_chan_properties	chan_prop[0];
};


static const struct pm8xxx_adc_scaling_ratio pm8xxx_amux_scaling_ratio[] = {
	{1, 1},
	{1, 3},
	{1, 4},
	{1, 6}
};

static struct pm8xxx_adc *pmic_adc;

static struct pm8xxx_adc_scale_fn adc_scale_fn[] = {
	[ADC_SCALE_DEFAULT] = {pm8xxx_adc_scale_default},
	[ADC_SCALE_PMIC_THERM] = {pm8xxx_adc_scale_pmic_therm},
	[ADC_SCALE_XOTHERM] = {pm8xxx_adc_tdkntcg_therm},
};

static bool pm8xxx_adc_calib_first_adc;
static bool pm8xxx_adc_initialized, pm8xxx_adc_calib_device_init;

static int32_t pm8xxx_adc_check_channel_valid(uint32_t channel)
{
	if (channel < CHANNEL_VCOIN ||
	(channel > CHANNEL_MUXOFF && channel < ADC_MPP_1_ATEST_8) ||
	(channel > ADC_MPP_1_ATEST_7 && channel < ADC_MPP_2_ATEST_8)
	|| (channel >= ADC_CHANNEL_MAX_NUM))
		return -EBADF;
	else
		return 0;
}

static int32_t pm8xxx_adc_arb_cntrl(uint32_t arb_cntrl,
					uint32_t channel)
{
	struct pm8xxx_adc *adc_pmic = pmic_adc;
	int i, rc;
	u8 data_arb_cntrl = 0;

	if (arb_cntrl) {
		if (adc_pmic->msm_suspend_check)
			pr_err("PM8xxx ADC request made after suspend_noirq "
					"with channel: %d\n", channel);
		data_arb_cntrl |= PM8XXX_ADC_ARB_USRP_CNTRL1_EN_ARB;
	}

	/* Write twice to the CNTRL register for the arbiter settings
	   to take into effect */
	for (i = 0; i < 2; i++) {
		rc = regmap_write(adc_pmic->regmap,
				PM8XXX_ADC_ARB_USRP_CNTRL1, data_arb_cntrl);
		if (rc < 0) {
			pr_err("PM8xxx arb cntrl write failed with %d\n", rc);
			return rc;
		}
	}

	if (arb_cntrl) {
		data_arb_cntrl |= PM8XXX_ADC_ARB_USRP_CNTRL1_REQ;
		reinit_completion(&adc_pmic->adc_rslt_completion);
		rc = regmap_write(adc_pmic->regmap,
			PM8XXX_ADC_ARB_USRP_CNTRL1, data_arb_cntrl);
	}

	return 0;
}

static int32_t pm8xxx_adc_channel_power_enable(uint32_t channel,
							bool power_cntrl)
{
	int rc = 0;

	switch (channel) {
	case ADC_MPP_1_AMUX8:
		//rc = pm8xxx_adc_patherm_power(power_cntrl);
		break;
	case CHANNEL_DIE_TEMP:
	case CHANNEL_MUXOFF:
		//rc = pm8xxx_adc_xo_vote(power_cntrl);
		break;
	default:
		break;
	}

	return rc;
}

static uint32_t pm8xxx_adc_read_reg(uint32_t reg, uint32_t *data)
{
	struct pm8xxx_adc *adc_pmic = pmic_adc;
	int rc;

	rc = regmap_read(adc_pmic->regmap, reg, data);
	if (rc < 0) {
		pr_err("PM8xxx adc read reg %d failed with %d\n", reg, rc);
	return rc;
	}

	return 0;
}

static uint32_t pm8xxx_adc_write_reg(uint32_t reg, uint32_t data)
{
	struct pm8xxx_adc *adc_pmic = pmic_adc;
	int rc;

	rc = regmap_write(adc_pmic->regmap, reg, data);
	if (rc < 0) {
		pr_err("PM8xxx adc write reg %d failed with %d\n", reg, rc);
		return rc;
	}

	return 0;
}

static int32_t pm8xxx_adc_configure(
				struct pm8xxx_adc_amux_properties *chan_prop)
{
	uint32_t data_amux_chan = 0, data_arb_rsv = 0, data_dig_param = 0;
	int rc;

	data_amux_chan |= chan_prop->amux_channel << PM8XXX_ADC_AMUX_SEL;

	if (chan_prop->amux_mpp_channel)
		data_amux_chan |= chan_prop->amux_mpp_channel <<
					PM8XXX_ADC_AMUX_MPP_SEL;

	rc = pm8xxx_adc_write_reg(PM8XXX_ADC_ARB_USRP_AMUX_CNTRL,
							data_amux_chan);
	if (rc < 0)
		return rc;

	rc = pm8xxx_adc_read_reg(PM8XXX_ADC_ARB_USRP_RSV, &data_arb_rsv);
	if (rc < 0)
		return rc;

	data_arb_rsv &= (PM8XXX_ADC_ARB_USRP_RSV_RST |
		PM8XXX_ADC_ARB_USRP_RSV_DTEST0 |
		PM8XXX_ADC_ARB_USRP_RSV_DTEST1 |
		PM8XXX_ADC_ARB_USRP_RSV_OP);
	data_arb_rsv |= (chan_prop->amux_ip_rsv << PM8XXX_ADC_RSV_IP_SEL |
				PM8XXX_ADC_ARB_USRP_RSV_TRM);

	rc = pm8xxx_adc_write_reg(PM8XXX_ADC_ARB_USRP_RSV, data_arb_rsv);
	if (rc < 0)
		return rc;

	rc = pm8xxx_adc_read_reg(PM8XXX_ADC_ARB_USRP_DIG_PARAM,
							&data_dig_param);
	if (rc < 0)
		return rc;

	/* Default 2.4Mhz clock rate */
	/* Client chooses the decimation */
	switch (chan_prop->decimation) {
	case ADC_DECIMATION_TYPE1:
		data_dig_param |= PM8XXX_ADC_ARB_USRP_DIG_PARAM_DEC_RATE0;
		break;
	case ADC_DECIMATION_TYPE2:
		data_dig_param |= (PM8XXX_ADC_ARB_USRP_DIG_PARAM_DEC_RATE0
				| PM8XXX_ADC_ARB_USRP_DIG_PARAM_DEC_RATE1);
		break;
	default:
		data_dig_param |= PM8XXX_ADC_ARB_USRP_DIG_PARAM_DEC_RATE0;
		break;
	}
	rc = pm8xxx_adc_write_reg(PM8XXX_ADC_ARB_USRP_DIG_PARAM,
						PM8XXX_ADC_ARB_ANA_DIG);
	if (rc < 0)
		return rc;

	rc = pm8xxx_adc_write_reg(PM8XXX_ADC_ARB_USRP_ANA_PARAM,
						PM8XXX_ADC_ARB_ANA_DIG);
	if (rc < 0)
		return rc;

	rc = pm8xxx_adc_arb_cntrl(1, data_amux_chan);
	if (rc < 0) {
		pr_err("Configuring ADC Arbiter"
				"enable failed with %d\n", rc);
		return rc;
	}

	return 0;
}

static uint32_t pm8xxx_adc_read_adc_code(int32_t *data)
{
	struct pm8xxx_adc *adc_pmic = pmic_adc;
	uint32_t rslt_lsb, rslt_msb;
	int32_t rc, max_ideal_adc_code = 1 << adc_pmic->adc_prop->bitresolution;

	rc = regmap_read(adc_pmic->regmap,
				PM8XXX_ADC_ARB_USRP_DATA0, &rslt_lsb);
	if (rc < 0) {
		pr_err("PM8xxx adc result read failed with %d\n", rc);
		return rc;
	}

	rc = regmap_read(adc_pmic->regmap,
				PM8XXX_ADC_ARB_USRP_DATA1, &rslt_msb);
	if (rc < 0) {
		pr_err("PM8xxx adc result read failed with %d\n", rc);
		return rc;
	}

	*data = (rslt_msb << 8) | rslt_lsb;

	/* Use the midpoint to determine underflow or overflow */
	if (*data > max_ideal_adc_code + (max_ideal_adc_code >> 1))
		*data |= ((1 << (8 * sizeof(*data) -
			adc_pmic->adc_prop->bitresolution)) - 1) <<
			adc_pmic->adc_prop->bitresolution;

	/* Default value for switching off the arbiter after reading
	   the ADC value. Bit 0 set to 0. */
	rc = pm8xxx_adc_arb_cntrl(0, CHANNEL_NONE);
	if (rc < 0) {
		pr_err("%s: Configuring ADC Arbiter disable"
					"failed\n", __func__);
		return rc;
	}

	return 0;
}

void trigger_completion(struct work_struct *work)
{
	struct pm8xxx_adc *adc_8xxx = pmic_adc;

	complete(&adc_8xxx->adc_rslt_completion);
}
DECLARE_WORK(trigger_completion_work, trigger_completion);

static irqreturn_t pm8xxx_adc_isr(int irq, void *dev_id)
{
	if (pm8xxx_adc_calib_first_adc)
		return IRQ_HANDLED;

	schedule_work(&trigger_completion_work);

	return IRQ_HANDLED;
}

static uint32_t pm8xxx_adc_calib_device(void)
{
	struct pm8xxx_adc *adc_pmic = pmic_adc;
	struct pm8xxx_adc_amux_properties conv;
	int rc, calib_read_1, calib_read_2;
	uint32_t data_arb_usrp_cntrl1 = 0;

	conv.amux_channel = CHANNEL_125V;
	conv.decimation = ADC_DECIMATION_TYPE2;
	conv.amux_ip_rsv = AMUX_RSV1;
	conv.amux_mpp_channel = PREMUX_MPP_SCALE_0;
	pm8xxx_adc_calib_first_adc = true;
	rc = pm8xxx_adc_configure(&conv);
	if (rc) {
		pr_err("pm8xxx_adc configure failed with %d\n", rc);
		goto calib_fail;
	}

	while (data_arb_usrp_cntrl1 != (PM8XXX_ADC_ARB_USRP_CNTRL1_EOC |
					PM8XXX_ADC_ARB_USRP_CNTRL1_EN_ARB)) {
		rc = pm8xxx_adc_read_reg(PM8XXX_ADC_ARB_USRP_CNTRL1,
					&data_arb_usrp_cntrl1);
		if (rc < 0)
			return rc;
		usleep_range(PM8XXX_ADC_CONV_TIME_MIN,
					PM8XXX_ADC_CONV_TIME_MAX);
	}
	data_arb_usrp_cntrl1 = 0;

	rc = pm8xxx_adc_read_adc_code(&calib_read_1);
	if (rc) {
		pr_err("pm8xxx_adc read adc failed with %d\n", rc);
		pm8xxx_adc_calib_first_adc = false;
		goto calib_fail;
	}
	pm8xxx_adc_calib_first_adc = false;

	conv.amux_channel = CHANNEL_625MV;
	conv.decimation = ADC_DECIMATION_TYPE2;
	conv.amux_ip_rsv = AMUX_RSV1;
	conv.amux_mpp_channel = PREMUX_MPP_SCALE_0;
	pm8xxx_adc_calib_first_adc = true;
	rc = pm8xxx_adc_configure(&conv);
	if (rc) {
		pr_err("pm8xxx_adc configure failed with %d\n", rc);
		goto calib_fail;
	}

	while (data_arb_usrp_cntrl1 != (PM8XXX_ADC_ARB_USRP_CNTRL1_EOC |
					PM8XXX_ADC_ARB_USRP_CNTRL1_EN_ARB)) {
		rc = pm8xxx_adc_read_reg(PM8XXX_ADC_ARB_USRP_CNTRL1,
					&data_arb_usrp_cntrl1);
		if (rc < 0)
			return rc;
		usleep_range(PM8XXX_ADC_CONV_TIME_MIN,
					PM8XXX_ADC_CONV_TIME_MAX);
	}
	data_arb_usrp_cntrl1 = 0;

	rc = pm8xxx_adc_read_adc_code(&calib_read_2);
	if (rc) {
		pr_err("pm8xxx_adc read adc failed with %d\n", rc);
		pm8xxx_adc_calib_first_adc = false;
		goto calib_fail;
	}
	pm8xxx_adc_calib_first_adc = false;

	adc_pmic->conv->chan_prop->adc_graph[ADC_CALIB_ABSOLUTE].dy =
					(calib_read_1 - calib_read_2);
	adc_pmic->conv->chan_prop->adc_graph[ADC_CALIB_ABSOLUTE].dx
						= PM8XXX_CHANNEL_ADC_625_UV;
	adc_pmic->conv->chan_prop->adc_graph[ADC_CALIB_ABSOLUTE].adc_vref =
					calib_read_1;
	adc_pmic->conv->chan_prop->adc_graph[ADC_CALIB_ABSOLUTE].adc_gnd =
					calib_read_2;
	rc = pm8xxx_adc_arb_cntrl(0, CHANNEL_NONE);
	if (rc < 0) {
		pr_err("%s: Configuring ADC Arbiter disable"
					"failed\n", __func__);
		return rc;
	}
	/* Ratiometric Calibration */
	conv.amux_channel = CHANNEL_MUXOFF;
	conv.decimation = ADC_DECIMATION_TYPE2;
	conv.amux_ip_rsv = AMUX_RSV5;
	conv.amux_mpp_channel = PREMUX_MPP_SCALE_0;
	pm8xxx_adc_calib_first_adc = true;
	rc = pm8xxx_adc_configure(&conv);
	if (rc) {
		pr_err("pm8xxx_adc configure failed with %d\n", rc);
		goto calib_fail;
	}

	while (data_arb_usrp_cntrl1 != (PM8XXX_ADC_ARB_USRP_CNTRL1_EOC |
					PM8XXX_ADC_ARB_USRP_CNTRL1_EN_ARB)) {
		rc = pm8xxx_adc_read_reg(PM8XXX_ADC_ARB_USRP_CNTRL1,
					&data_arb_usrp_cntrl1);
		if (rc < 0)
			return rc;
		usleep_range(PM8XXX_ADC_CONV_TIME_MIN,
					PM8XXX_ADC_CONV_TIME_MAX);
	}
	data_arb_usrp_cntrl1 = 0;

	rc = pm8xxx_adc_read_adc_code(&calib_read_1);
	if (rc) {
		pr_err("pm8xxx_adc read adc failed with %d\n", rc);
		pm8xxx_adc_calib_first_adc = false;
		goto calib_fail;
	}
	pm8xxx_adc_calib_first_adc = false;

	conv.amux_channel = CHANNEL_MUXOFF;
	conv.decimation = ADC_DECIMATION_TYPE2;
	conv.amux_ip_rsv = AMUX_RSV4;
	conv.amux_mpp_channel = PREMUX_MPP_SCALE_0;
	pm8xxx_adc_calib_first_adc = true;
	rc = pm8xxx_adc_configure(&conv);
	if (rc) {
		pr_err("pm8xxx_adc configure failed with %d\n", rc);
		goto calib_fail;
	}

	while (data_arb_usrp_cntrl1 != (PM8XXX_ADC_ARB_USRP_CNTRL1_EOC |
					PM8XXX_ADC_ARB_USRP_CNTRL1_EN_ARB)) {
		rc = pm8xxx_adc_read_reg(PM8XXX_ADC_ARB_USRP_CNTRL1,
					&data_arb_usrp_cntrl1);
		if (rc < 0)
			return rc;
		usleep_range(PM8XXX_ADC_CONV_TIME_MIN,
					PM8XXX_ADC_CONV_TIME_MAX);
	}
	data_arb_usrp_cntrl1 = 0;

	rc = pm8xxx_adc_read_adc_code(&calib_read_2);
	if (rc) {
		pr_err("pm8xxx_adc read adc failed with %d\n", rc);
		pm8xxx_adc_calib_first_adc = false;
		goto calib_fail;
	}
	pm8xxx_adc_calib_first_adc = false;

	adc_pmic->conv->chan_prop->adc_graph[ADC_CALIB_RATIOMETRIC].dy =
					(calib_read_1 - calib_read_2);
	adc_pmic->conv->chan_prop->adc_graph[ADC_CALIB_RATIOMETRIC].dx =
					adc_pmic->adc_prop->adc_vdd_reference;
	adc_pmic->conv->chan_prop->adc_graph[ADC_CALIB_RATIOMETRIC].adc_vref =
					calib_read_1;
	adc_pmic->conv->chan_prop->adc_graph[ADC_CALIB_RATIOMETRIC].adc_gnd =
					calib_read_2;
calib_fail:
	rc = pm8xxx_adc_arb_cntrl(0, CHANNEL_NONE);
	if (rc < 0) {
		pr_err("%s: Configuring ADC Arbiter disable"
					"failed\n", __func__);
	}

	return rc;
}

uint32_t pm8xxx_adc_read(enum pm8xxx_adc_channels channel,
				struct pm8xxx_adc_chan_result *result)
{
	struct pm8xxx_adc *adc_pmic = pmic_adc;
	int i = 0, rc = 0, rc_fail, amux_prescaling, scale_type;
	enum pm8xxx_adc_premux_mpp_scale_type mpp_scale;

	if (!pm8xxx_adc_initialized)
		return -ENODEV;

	if (!pm8xxx_adc_calib_device_init) {
		if (pm8xxx_adc_calib_device() == 0)
			pm8xxx_adc_calib_device_init = true;
	}

	mutex_lock(&adc_pmic->adc_lock);

	for (i = 0; i < adc_pmic->adc_num_board_channel; i++) {
		if (channel == adc_pmic->adc_channel[i].channel_name)
			break;
	}

	if (i == adc_pmic->adc_num_board_channel ||
		(pm8xxx_adc_check_channel_valid(channel) != 0)) {
		rc = -EBADF;
		goto fail_unlock;
	}

	if (channel < PM8XXX_CHANNEL_MPP_SCALE1_IDX) {
		mpp_scale = PREMUX_MPP_SCALE_0;
		adc_pmic->conv->amux_channel = channel;
	} else if (channel >= PM8XXX_CHANNEL_MPP_SCALE1_IDX &&
			channel < PM8XXX_CHANNEL_MPP_SCALE3_IDX) {
		mpp_scale = PREMUX_MPP_SCALE_1;
		adc_pmic->conv->amux_channel = channel %
				PM8XXX_CHANNEL_MPP_SCALE1_IDX;
	} else {
		mpp_scale = PREMUX_MPP_SCALE_1_DIV3;
		adc_pmic->conv->amux_channel = channel %
				PM8XXX_CHANNEL_MPP_SCALE3_IDX;
	}

	adc_pmic->conv->amux_mpp_channel = mpp_scale;
	adc_pmic->conv->amux_ip_rsv = adc_pmic->adc_channel[i].adc_rsv;
	adc_pmic->conv->decimation = adc_pmic->adc_channel[i].adc_decimation;
	amux_prescaling = adc_pmic->adc_channel[i].chan_path_prescaling;

	adc_pmic->conv->chan_prop->offset_gain_numerator =
		pm8xxx_amux_scaling_ratio[amux_prescaling].num;
	adc_pmic->conv->chan_prop->offset_gain_denominator =
		 pm8xxx_amux_scaling_ratio[amux_prescaling].den;

	rc = pm8xxx_adc_channel_power_enable(channel, true);
	if (rc) {
		rc = -EINVAL;
		goto fail_unlock;
	}

	rc = pm8xxx_adc_configure(adc_pmic->conv);
	if (rc) {
		rc = -EINVAL;
		goto fail;
	}

	rc = wait_for_completion_timeout(&adc_pmic->adc_rslt_completion,
						PM8XXX_ADC_COMPLETION_TIMEOUT);
	if (!rc) {
		uint32_t data_arb_usrp_cntrl1 = 0;

		rc = pm8xxx_adc_read_reg(PM8XXX_ADC_ARB_USRP_CNTRL1,
					&data_arb_usrp_cntrl1);
		if (rc < 0)
			goto fail;
		if (data_arb_usrp_cntrl1 == (PM8XXX_ADC_ARB_USRP_CNTRL1_EOC |
					PM8XXX_ADC_ARB_USRP_CNTRL1_EN_ARB))
			pr_debug("End of conversion status set\n");
		else {
			pr_err("EOC interrupt not received\n");
			rc = -EINVAL;
			goto fail;
		}
	}

	rc = pm8xxx_adc_read_adc_code(&result->adc_code);
	if (rc) {
		rc = -EINVAL;
		goto fail;
	}

	scale_type = adc_pmic->adc_channel[i].adc_scale_fn;
	if (scale_type >= ADC_SCALE_NONE) {
		rc = -EBADF;
		goto fail;
	}

	adc_scale_fn[scale_type].chan(result->adc_code,
			adc_pmic->adc_prop, adc_pmic->conv->chan_prop, result);

	rc = pm8xxx_adc_channel_power_enable(channel, false);
	if (rc) {
		rc = -EINVAL;
		goto fail_unlock;
	}

	mutex_unlock(&adc_pmic->adc_lock);

	return 0;
fail:
	rc_fail = pm8xxx_adc_channel_power_enable(channel, false);
	if (rc_fail)
		pr_err("pm8xxx adc power disable failed\n");
fail_unlock:
	mutex_unlock(&adc_pmic->adc_lock);
	pr_err("pm8xxx adc error with %d\n", rc);
	return rc;
}
EXPORT_SYMBOL_GPL(pm8xxx_adc_read);

static ssize_t pm8xxx_adc_show(struct device *dev,
			struct device_attribute *devattr, char *buf)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(devattr);
	struct pm8xxx_adc_chan_result result;
	int rc = -1;

	rc = pm8xxx_adc_read(attr->index, &result);

	if (rc)
		return 0;

	return snprintf(buf, PM8XXX_ADC_HWMON_NAME_LENGTH,
		"Result:%lld Raw:%d\n", result.physical, result.adc_code);
}


static int get_adc(void *data, u64 *val)
{
	struct pm8xxx_adc_chan_result result;
	int i = (int)data;
	int rc;

	rc = pm8xxx_adc_read(i, &result);
	if (!rc)
		pr_info("ADC value raw:%x physical:%lld\n",
			result.adc_code, result.physical);
	*val = result.physical;

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(reg_fops, get_adc, NULL, "%llu\n");


#ifdef CONFIG_DEBUG_FS
static void create_debugfs_entries(void)
{
	int i = 0;

	pmic_adc->dent = debugfs_create_dir("pm8xxx_adc", NULL);

	if (IS_ERR(pmic_adc->dent)) {
		pr_err("pmic adc debugfs dir not created\n");
		return;
	}

	for (i = 0; i < pmic_adc->adc_num_board_channel; i++)
		debugfs_create_file(pmic_adc->adc_channel[i].name,
			0644, pmic_adc->dent,
			(void *)pmic_adc->adc_channel[i].channel_name,
			&reg_fops);
}
#else
static inline void create_debugfs_entries(void)
{
}
#endif
static struct sensor_device_attribute pm8xxx_adc_attr =
	SENSOR_ATTR(NULL, S_IRUGO, pm8xxx_adc_show, NULL, 0);

static int32_t pm8xxx_adc_init_hwmon(struct platform_device *pdev)
{
	struct pm8xxx_adc *adc_pmic = pmic_adc;
	int rc = 0, i, channel;

	for (i = 0; i < pmic_adc->adc_num_board_channel; i++) {
		channel = adc_pmic->adc_channel[i].channel_name;
		if (pm8xxx_adc_check_channel_valid(channel)) {
			pr_err("Invalid ADC init HWMON channel: %d\n", channel);
			continue;
		}
		pm8xxx_adc_attr.index = adc_pmic->adc_channel[i].channel_name;
		pm8xxx_adc_attr.dev_attr.attr.name =
						adc_pmic->adc_channel[i].name;
		memcpy(&adc_pmic->sens_attr[i], &pm8xxx_adc_attr,
						sizeof(pm8xxx_adc_attr));
		sysfs_attr_init(&adc_pmic->sens_attr[i].dev_attr.attr);
		rc = device_create_file(&pdev->dev,
				&adc_pmic->sens_attr[i].dev_attr);
		if (rc) {
			dev_err(&pdev->dev, "device_create_file failed for "
					    "dev %s\n",
					    adc_pmic->adc_channel[i].name);
			goto hwmon_err_sens;
		}
	}

	return 0;

hwmon_err_sens:
	pr_info("Init HWMON failed for pm8xxx_adc with %d\n", rc);
	return rc;
}

static struct pm8xxx_adc_amux apq8064_pm8921_adc_channels_data[] = {
	{"vcoin", CHANNEL_VCOIN, CHAN_PATH_SCALING2, AMUX_RSV1,
		ADC_DECIMATION_TYPE2, ADC_SCALE_DEFAULT},
	{"vbat", CHANNEL_VBAT, CHAN_PATH_SCALING2, AMUX_RSV1,
		ADC_DECIMATION_TYPE2, ADC_SCALE_DEFAULT},
	{"dcin", CHANNEL_DCIN, CHAN_PATH_SCALING4, AMUX_RSV1,
		ADC_DECIMATION_TYPE2, ADC_SCALE_DEFAULT},
	{"ichg", CHANNEL_ICHG, CHAN_PATH_SCALING1, AMUX_RSV1,
		ADC_DECIMATION_TYPE2, ADC_SCALE_DEFAULT},
	{"vph_pwr", CHANNEL_VPH_PWR, CHAN_PATH_SCALING2, AMUX_RSV1,
		ADC_DECIMATION_TYPE2, ADC_SCALE_DEFAULT},
	{"ibat", CHANNEL_IBAT, CHAN_PATH_SCALING1, AMUX_RSV1,
		ADC_DECIMATION_TYPE2, ADC_SCALE_DEFAULT},
	{"batt_therm", CHANNEL_BATT_THERM, CHAN_PATH_SCALING1, AMUX_RSV2,
		ADC_DECIMATION_TYPE2, ADC_SCALE_BATT_THERM},
	{"batt_id", CHANNEL_BATT_ID, CHAN_PATH_SCALING1, AMUX_RSV1,
		ADC_DECIMATION_TYPE2, ADC_SCALE_DEFAULT},
	{"usbin", CHANNEL_USBIN, CHAN_PATH_SCALING3, AMUX_RSV1,
		ADC_DECIMATION_TYPE2, ADC_SCALE_DEFAULT},
	{"pmic_therm", CHANNEL_DIE_TEMP, CHAN_PATH_SCALING1, AMUX_RSV1,
		ADC_DECIMATION_TYPE2, ADC_SCALE_PMIC_THERM},
	{"625mv", CHANNEL_625MV, CHAN_PATH_SCALING1, AMUX_RSV1,
		ADC_DECIMATION_TYPE2, ADC_SCALE_DEFAULT},
	{"125v", CHANNEL_125V, CHAN_PATH_SCALING1, AMUX_RSV1,
		ADC_DECIMATION_TYPE2, ADC_SCALE_DEFAULT},
	{"chg_temp", CHANNEL_CHG_TEMP, CHAN_PATH_SCALING1, AMUX_RSV1,
		ADC_DECIMATION_TYPE2, ADC_SCALE_DEFAULT},
	{"xo_therm", CHANNEL_MUXOFF, CHAN_PATH_SCALING1, AMUX_RSV0,
		ADC_DECIMATION_TYPE2, ADC_SCALE_XOTHERM},
};
static struct pm8xxx_adc_properties apq8064_pm8921_adc_data = {
	.adc_vdd_reference      = 1800, /* milli-voltage for this adc */
	.bitresolution          = 15,
	.bipolar                = 0,
};

static struct pm8xxx_adc_platform_data apq8064_pm8921_adc_pdata = {
	.adc_channel            = apq8064_pm8921_adc_channels_data,
	.adc_num_board_channel  = ARRAY_SIZE(apq8064_pm8921_adc_channels_data),
	.adc_prop               = &apq8064_pm8921_adc_data,
	.adc_mpp_base           = 196, //PM8921_MPP_PM_TO_SYS(1),
};


static const struct of_device_id pm8921_adc_of_match[] = {
	{ .compatible = "qcom,pm8921-adc", .data = &apq8064_pm8921_adc_pdata },
};

MODULE_DEVICE_TABLE(of, pm8921_adc_of_match);

static int pm8xxx_adc_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct device *dev = &pdev->dev;
	const struct pm8xxx_adc_platform_data *pdata;
	struct pm8xxx_adc *adc_pmic;
	struct pm8xxx_adc_amux_properties *adc_amux_prop;
	int rc = 0;

	if (!np) {
		dev_err(dev, "Non DT not supported\n");
		return -EINVAL;
	}

	pdata = of_match_node(pm8921_adc_of_match, np)->data;

	adc_pmic = devm_kzalloc(&pdev->dev, sizeof(struct pm8xxx_adc) +
			(sizeof(struct sensor_device_attribute) *
			pdata->adc_num_board_channel), GFP_KERNEL);
	if (!adc_pmic)
		return -ENOMEM;

	adc_amux_prop = devm_kzalloc(&pdev->dev,
				sizeof(struct pm8xxx_adc_amux_properties) +
				sizeof(struct pm8xxx_adc_chan_properties)
				, GFP_KERNEL);
	if (!adc_amux_prop)
		return -ENOMEM;

	adc_pmic->dev = &pdev->dev;
	adc_pmic->adc_prop = pdata->adc_prop;
	adc_pmic->conv = adc_amux_prop;
	init_completion(&adc_pmic->adc_rslt_completion);
	adc_pmic->adc_channel = pdata->adc_channel;
	adc_pmic->adc_num_board_channel = pdata->adc_num_board_channel;
	adc_pmic->mpp_base = pdata->adc_mpp_base;

	mutex_init(&adc_pmic->adc_lock);

	adc_pmic->adc_irq = platform_get_irq_byname(pdev, "pm8921_adc_eoc_usr_irq");

	if (adc_pmic->adc_irq > 0) {
		rc = devm_request_irq(&pdev->dev, adc_pmic->adc_irq,
				pm8xxx_adc_isr,
			IRQF_TRIGGER_RISING, "pm8xxx_adc_interrupt", adc_pmic);
		if (rc < 0) {
			dev_err(&pdev->dev, "failed to request adc irq "
				"with error %d\n", rc);
		} else {
			enable_irq_wake(adc_pmic->adc_irq);
		}
	}

	adc_pmic->regmap = dev_get_regmap(pdev->dev.parent, NULL);
	if (!adc_pmic->regmap) {
		dev_err(&pdev->dev, "Parent regmap unavailable\n");
		return -ENXIO;
	}

	platform_set_drvdata(pdev, adc_pmic);
	adc_pmic->msm_suspend_check = 0;
	pmic_adc = adc_pmic;

	create_debugfs_entries();
	pm8xxx_adc_calib_first_adc = false;
	pm8xxx_adc_calib_device_init = false;
	pm8xxx_adc_initialized = true;

	rc = pm8xxx_adc_init_hwmon(pdev);
	if (rc)
		dev_err(&pdev->dev, "failed to initialize pm8xxx hwmon adc\n");

	adc_pmic->hwmon = hwmon_device_register(adc_pmic->dev);
	if (IS_ERR(adc_pmic->hwmon)) {
		rc = PTR_ERR(adc_pmic->hwmon);
		return rc;
	}

	return 0;
}

static int pm8xxx_adc_teardown(struct platform_device *pdev)
{
	struct pm8xxx_adc *adc_pmic = pmic_adc;
	int i;

	pr_crit("test1\n");
	hwmon_device_unregister(adc_pmic->hwmon);
	for (i = 0; i < adc_pmic->adc_num_board_channel; i++)
		device_remove_file(adc_pmic->dev,
				&adc_pmic->sens_attr[i].dev_attr);
	pm8xxx_adc_initialized = false;

	return 0;
}

static struct platform_driver pm8xxx_adc_driver = {
	.probe	= pm8xxx_adc_probe,
	.remove	= pm8xxx_adc_teardown,
	.driver	= {
		.name	= "pm8xxx-adc",
		.of_match_table = pm8921_adc_of_match,
	},
};

module_platform_driver(pm8xxx_adc_driver);

MODULE_ALIAS("platform: pm8xxx-adc");
MODULE_DESCRIPTION("PMIC8921 ADC driver");
MODULE_LICENSE("GPL v2");
