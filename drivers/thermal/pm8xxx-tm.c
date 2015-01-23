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
 */

/*
 * Qualcomm PMIC PM8xxx Thermal Manager driver
 */

#define pr_fmt(fmt) "%s: " fmt, __func__

#include <linux/module.h>
#include <linux/err.h>
#include <linux/string.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/thermal.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/mfd/tm.h>
#include <linux/completion.h>
#include <linux/mfd/pm8xxx-adc.h>
#include <linux/of.h>
#include <linux/init.h>
#include <linux/regmap.h>

/* ADC reg */
#define PM8XXX_ADC_ARB_USRP_RSV                         0x19B

#define PM8XXX_ADC_ARB_BTM_CNTRL2                       0x18c
#define PM8XXX_ADC_ARB_BTM_AMUX_CNTRL                   0x17f
#define PM8XXX_ADC_ARB_BTM_ANA_PARAM                    0x180
#define PM8XXX_ADC_ARB_BTM_DIG_PARAM                    0x181
#define PM8XXX_ADC_ARB_BTM_RSV                          0x182
#define PM8XXX_ADC_ARB_BTM_DATA1                        0x183
#define PM8XXX_ADC_ARB_BTM_DATA0                        0x184
#define PM8XXX_ADC_ARB_BTM_BAT_COOL_THR1                0x185
#define PM8XXX_ADC_ARB_BTM_BAT_COOL_THR0                0x186
#define PM8XXX_ADC_ARB_BTM_BAT_WARM_THR1                0x187
#define PM8XXX_ADC_ARB_BTM_BAT_WARM_THR0                0x188
#define PM8XXX_ADC_ARB_USRP_ANA_PARAM                   0x199
#define PM8XXX_ADC_ARB_USRP_DIG_PARAM                   0x19A
#define PM8XXX_ADC_ARB_USRP_AMUX_CNTRL                  0x198

/* Register TEMP_ALARM_CTRL bits */
#define TEMP_ALARM_CTRL_ST3_SD		0x80
#define TEMP_ALARM_CTRL_ST2_SD		0x40
#define TEMP_ALARM_CTRL_STATUS_MASK	0x30
#define TEMP_ALARM_CTRL_STATUS_SHIFT	4
#define TEMP_ALARM_CTRL_THRESH_MASK	0x0C
#define TEMP_ALARM_CTRL_THRESH_SHIFT	2
#define TEMP_ALARM_CTRL_OVRD_ST3	0x02
#define TEMP_ALARM_CTRL_OVRD_ST2	0x01
#define TEMP_ALARM_CTRL_OVRD_MASK	0x03

#define TEMP_STAGE_STEP			20000	/* Stage step: 20.000 C */
#define TEMP_STAGE_HYSTERESIS		2000

#define TEMP_THRESH_MIN			105000	/* Threshold Min: 105 C */
#define TEMP_THRESH_STEP		5000	/* Threshold step: 5 C */

/* Register TEMP_ALARM_PWM bits */
#define TEMP_ALARM_PWM_EN_MASK		0xC0
#define TEMP_ALARM_PWM_EN_NEVER		0x00
#define TEMP_ALARM_PWM_EN_SLEEP_B	0x40
#define TEMP_ALARM_PWM_EN_PWM		0x80
#define TEMP_ALARM_PWM_EN_ALWAYS	0xC0
#define TEMP_ALARM_PWM_PER_PRE_MASK	0x38
#define TEMP_ALARM_PWM_PER_PRE_SHIFT	3
#define TEMP_ALARM_PWM_PER_DIV_MASK	0x07
#define TEMP_ALARM_PWM_PER_DIV_SHIFT	0

/* Trips: from critical to less critical */
#define TRIP_STAGE3			0
#define TRIP_STAGE2			1
#define TRIP_STAGE1			2
#define TRIP_NUM			3

struct pm8xxx_tm_chip {
	struct pm8xxx_tm_core_data	cdata;
	struct delayed_work		irq_work;
	struct device			*dev;
	struct thermal_zone_device	*tz_dev;
	struct regmap			*regmap;
	unsigned long			temp;
	unsigned int			prev_stage;
	enum thermal_device_mode	mode;
	unsigned int			thresh;
	unsigned int			stage;
	unsigned int			tempstat_irq;
	unsigned int			overtemp_irq;
	void				*adc_handle;
};

enum pmic_thermal_override_mode {
	SOFTWARE_OVERRIDE_DISABLED = 0,
	SOFTWARE_OVERRIDE_ENABLED,
};


/* Delay between TEMP_STAT IRQ going high and status value changing in ms. */
#define STATUS_REGISTER_DELAY_MS	40

static inline int pm8xxx_tm_read_ctrl(struct pm8xxx_tm_chip *chip, unsigned int *reg)
{
	int rc;

	rc = regmap_read(chip->regmap, chip->cdata.reg_addr_temp_alarm_ctrl, reg);
	if (rc)
		pr_err("%s: pm8xxx_readb(0x%03X) failed, rc=%d\n",
			chip->cdata.tm_name,
			chip->cdata.reg_addr_temp_alarm_ctrl, rc);

	return rc;
}

static inline int pm8xxx_tm_write_ctrl(struct pm8xxx_tm_chip *chip, unsigned int reg)
{
	int rc;

	rc = regmap_write(chip->regmap, chip->cdata.reg_addr_temp_alarm_ctrl, reg);
	if (rc)
		pr_err("%s: pm8xxx_writeb(0x%03X)=0x%02X failed, rc=%d\n",
		       chip->cdata.tm_name,
		       chip->cdata.reg_addr_temp_alarm_ctrl, reg, rc);

	return rc;
}

static inline int pm8xxx_tm_write_pwm(struct pm8xxx_tm_chip *chip, unsigned int reg)
{
	int rc;

	rc = regmap_write(chip->regmap, chip->cdata.reg_addr_temp_alarm_pwm, reg);
	if (rc)
		pr_err("%s: pm8xxx_writeb(0x%03X)=0x%02X failed, rc=%d\n",
			chip->cdata.tm_name,
			chip->cdata.reg_addr_temp_alarm_pwm, reg, rc);

	return rc;
}


static inline int
pm8xxx_tm_shutdown_override(struct pm8xxx_tm_chip *chip,
			    enum pmic_thermal_override_mode mode)
{
	int rc;
	unsigned int reg;

	rc = pm8xxx_tm_read_ctrl(chip, &reg);
	if (rc < 0)
		return rc;

	reg &= ~(TEMP_ALARM_CTRL_OVRD_MASK | TEMP_ALARM_CTRL_STATUS_MASK);
	if (mode == SOFTWARE_OVERRIDE_ENABLED)
		reg |= (TEMP_ALARM_CTRL_OVRD_ST3 | TEMP_ALARM_CTRL_OVRD_ST2) &
			TEMP_ALARM_CTRL_OVRD_MASK;

	rc = pm8xxx_tm_write_ctrl(chip, reg);

	return rc;
}

/*
 * This function initializes the internal temperature value based on only the
 * current thermal stage and threshold.
 */
static int pm8xxx_tm_init_temp_no_adc(struct pm8xxx_tm_chip *chip)
{
	int rc;
	unsigned int reg;

	rc = pm8xxx_tm_read_ctrl(chip, &reg);
	if (rc < 0)
		return rc;

	chip->stage = (reg & TEMP_ALARM_CTRL_STATUS_MASK)
			>> TEMP_ALARM_CTRL_STATUS_SHIFT;
	chip->thresh = (reg & TEMP_ALARM_CTRL_THRESH_MASK)
			>> TEMP_ALARM_CTRL_THRESH_SHIFT;

	if (chip->stage)
		chip->temp = chip->thresh * TEMP_THRESH_MIN +
			   (chip->stage - 1) * TEMP_STAGE_STEP +
			   TEMP_THRESH_MIN;
	else
		chip->temp = chip->cdata.default_no_adc_temp;

	return 0;
}

/*
 * This function updates the internal temperature value based on the
 * current thermal stage and threshold as well as the previous stage
 */
static int pm8xxx_tm_update_temp_no_adc(struct pm8xxx_tm_chip *chip)
{
	unsigned int stage;
	int rc;
	unsigned int reg;

	rc = pm8xxx_tm_read_ctrl(chip, &reg);
	if (rc < 0)
		return rc;

	stage = (reg & TEMP_ALARM_CTRL_STATUS_MASK)
		>> TEMP_ALARM_CTRL_STATUS_SHIFT;
	chip->thresh = (reg & TEMP_ALARM_CTRL_THRESH_MASK)
			>> TEMP_ALARM_CTRL_THRESH_SHIFT;

	if (stage > chip->stage) {
		/* increasing stage, use lower bound */
		chip->temp = (stage - 1) * TEMP_STAGE_STEP
				+ chip->thresh * TEMP_THRESH_STEP
				+ TEMP_STAGE_HYSTERESIS + TEMP_THRESH_MIN;
	} else if (stage < chip->stage) {
		/* decreasing stage, use upper bound */
		chip->temp = stage * TEMP_STAGE_STEP
				+ chip->thresh * TEMP_THRESH_STEP
				- TEMP_STAGE_HYSTERESIS + TEMP_THRESH_MIN;
	}

	chip->stage = stage;

	return 0;
}

static int pm8xxx_tz_get_temp_no_adc(struct thermal_zone_device *thermal,
				     unsigned long *temp)
{
	struct pm8xxx_tm_chip *chip = thermal->devdata;
	int rc;

	if (!chip || !temp)
		return -EINVAL;

	rc = pm8xxx_tm_update_temp_no_adc(chip);
	if (rc < 0)
		return rc;

	*temp = chip->temp;

	return 0;
}

static int pm8xxx_tz_get_temp_pm8xxx_adc(struct thermal_zone_device *thermal,
				      unsigned long *temp)
{
	struct pm8xxx_tm_chip *chip = thermal->devdata;
	struct pm8xxx_adc_chan_result result = {
		.physical = 0lu,
	};
	int rc;

	if (!chip || !temp)
		return -EINVAL;

	*temp = chip->temp;

	rc = pm8xxx_adc_read(chip->cdata.adc_channel, &result);
	if (rc < 0) {
		pr_err("%s: adc_channel_read_result() failed, rc = %d\n",
			chip->cdata.tm_name, rc);
		return rc;
	}

	*temp = result.physical;
	chip->temp = result.physical;

	return 0;
}

static int pm8xxx_tz_get_mode(struct thermal_zone_device *thermal,
			      enum thermal_device_mode *mode)
{

	struct pm8xxx_tm_chip *chip = thermal->devdata;

	if (!chip || !mode)
		return -EINVAL;

	*mode = chip->mode;
	return 0;
}

static int pm8xxx_tz_set_mode(struct thermal_zone_device *thermal,
			      enum thermal_device_mode mode)
{
	struct pm8xxx_tm_chip *chip = thermal->devdata;

	if (!chip)
		return -EINVAL;

	/* Mask software override requests if they are not allowed. */
	if (!chip->cdata.allow_software_override)
		mode = THERMAL_DEVICE_DISABLED;

	if (mode != chip->mode) {
		if (mode == THERMAL_DEVICE_ENABLED)
			pm8xxx_tm_shutdown_override(chip,
						    SOFTWARE_OVERRIDE_ENABLED);
		else
			pm8xxx_tm_shutdown_override(chip,
						    SOFTWARE_OVERRIDE_DISABLED);
	}
	chip->mode = mode;
	return 0;
}

static int pm8xxx_tz_get_trip_type(struct thermal_zone_device *thermal,
				   int trip, enum thermal_trip_type *type)
{
	if (trip < 0 || !type)
		return -EINVAL;

	switch (trip) {
	case TRIP_STAGE3:
		*type = THERMAL_TRIP_CRITICAL;
		break;
	case TRIP_STAGE2:
		*type = THERMAL_TRIP_HOT;
		break;
	case TRIP_STAGE1:
		*type = THERMAL_TRIP_HOT;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int pm8xxx_tz_get_trip_temp(struct thermal_zone_device *thermal,
				   int trip, unsigned long *temp)
{
	struct pm8xxx_tm_chip *chip = thermal->devdata;
	int thresh_temp;

	if (!chip || trip < 0 || !temp)
		return -EINVAL;

	thresh_temp = chip->thresh * TEMP_THRESH_STEP +
			TEMP_THRESH_MIN;

	switch (trip) {
	case TRIP_STAGE3:
		thresh_temp += 2 * TEMP_STAGE_STEP;
		break;
	case TRIP_STAGE2:
		thresh_temp += TEMP_STAGE_STEP;
		break;
	case TRIP_STAGE1:
		break;
	default:
		return -EINVAL;
	}

	*temp = thresh_temp;

	return 0;
}

static int pm8xxx_tz_get_crit_temp(struct thermal_zone_device *thermal,
				   unsigned long *temp)
{
	struct pm8xxx_tm_chip *chip = thermal->devdata;

	if (!chip || !temp)
		return -EINVAL;

	*temp = chip->thresh * TEMP_THRESH_STEP + TEMP_THRESH_MIN +
		2 * TEMP_STAGE_STEP;

	return 0;
}

static struct thermal_zone_device_ops pm8xxx_thermal_zone_ops_no_adc = {
	.get_temp = pm8xxx_tz_get_temp_no_adc,
	.get_mode = pm8xxx_tz_get_mode,
	.set_mode = pm8xxx_tz_set_mode,
	.get_trip_type = pm8xxx_tz_get_trip_type,
	.get_trip_temp = pm8xxx_tz_get_trip_temp,
	.get_crit_temp = pm8xxx_tz_get_crit_temp,
};

static struct thermal_zone_device_ops pm8xxx_thermal_zone_ops_pm8xxx_adc = {
	.get_temp = pm8xxx_tz_get_temp_pm8xxx_adc,
	.get_mode = pm8xxx_tz_get_mode,
	.set_mode = pm8xxx_tz_set_mode,
	.get_trip_type = pm8xxx_tz_get_trip_type,
	.get_trip_temp = pm8xxx_tz_get_trip_temp,
	.get_crit_temp = pm8xxx_tz_get_crit_temp,
};

static void pm8xxx_tm_work(struct work_struct *work)
{
	struct delayed_work *dwork
		= container_of(work, struct delayed_work, work);
	struct pm8xxx_tm_chip *chip
		= container_of(dwork, struct pm8xxx_tm_chip, irq_work);
	unsigned long temp = 0;
	int rc, stage, thresh;
	unsigned int reg;

	rc = pm8xxx_tm_read_ctrl(chip, &reg);
	if (rc < 0)
		goto bail;

	/* Clear status bits. */
	if (reg & (TEMP_ALARM_CTRL_ST2_SD | TEMP_ALARM_CTRL_ST3_SD)) {
		reg &= ~(TEMP_ALARM_CTRL_ST2_SD | TEMP_ALARM_CTRL_ST3_SD
			 | TEMP_ALARM_CTRL_STATUS_MASK);

		pm8xxx_tm_write_ctrl(chip, reg);
	}

	stage = (reg & TEMP_ALARM_CTRL_STATUS_MASK)
		>> TEMP_ALARM_CTRL_STATUS_SHIFT;
	thresh = (reg & TEMP_ALARM_CTRL_THRESH_MASK)
		>> TEMP_ALARM_CTRL_THRESH_SHIFT;

	thermal_zone_device_update(chip->tz_dev);

	if (stage != chip->prev_stage) {
		chip->prev_stage = stage;

		if (chip->cdata.adc_type == PM8XXX_TM_ADC_PM8XXX_ADC)
			rc = pm8xxx_tz_get_temp_pm8xxx_adc(chip->tz_dev, &temp);

		if (rc < 0)
			goto bail;

		pr_crit("%s: PMIC Temp Alarm - stage=%u, threshold=%u, temp=%lu mC\n",
			chip->cdata.tm_name, stage, thresh, temp);

		/* Notify user space */
		sysfs_notify(&chip->tz_dev->device.kobj, NULL, "type");
	}

bail:
	return;
}

static irqreturn_t pm8xxx_tm_isr(int irq, void *data)
{
	struct pm8xxx_tm_chip *chip = data;

	schedule_delayed_work(&chip->irq_work,
		msecs_to_jiffies(STATUS_REGISTER_DELAY_MS) + 1);

	return IRQ_HANDLED;
}

static int pm8xxx_tm_init_reg(struct pm8xxx_tm_chip *chip)
{
	int rc;
	unsigned int reg;

	rc = pm8xxx_tm_read_ctrl(chip, &reg);
	if (rc < 0)
		return rc;

	chip->stage = (reg & TEMP_ALARM_CTRL_STATUS_MASK)
			>> TEMP_ALARM_CTRL_STATUS_SHIFT;
	chip->temp = 0;

	/* Use temperature threshold set 0: (105, 125, 145) */
	chip->thresh = 0;
	reg = (chip->thresh << TEMP_ALARM_CTRL_THRESH_SHIFT)
		& TEMP_ALARM_CTRL_THRESH_MASK;
	rc = pm8xxx_tm_write_ctrl(chip, reg);
	if (rc < 0)
		return rc;

	/*
	 * Set the PMIC temperature alarm module to be always on.  This ensures
	 * that die temperature monitoring is active even if CXO is disabled
	 * (i.e. when sleep_b is low).  This is necessary since CXO can be
	 * disabled while the system is still heavily loaded.  Also, using
	 * the alway-on instead of PWM-enabled configurations ensures that the
	 * die temperature can be measured by the PMIC ADC without reconfiguring
	 * the temperature alarm module first.
	 */
	rc = pm8xxx_tm_write_pwm(chip, TEMP_ALARM_PWM_EN_ALWAYS);

	return rc;
}

#define REG_TEMP_ALARM_CTRL     0x1B
#define REG_TEMP_ALARM_PWM      0x9B

struct pm8xxx_tm_core_data thermal_alarm_cdata = {
	.adc_channel =                  CHANNEL_DIE_TEMP, //it will be CHANNEL_MUXOFF
	.adc_type =                     PM8XXX_TM_ADC_PM8XXX_ADC,
	.reg_addr_temp_alarm_ctrl =     REG_TEMP_ALARM_CTRL,
	.reg_addr_temp_alarm_pwm =      REG_TEMP_ALARM_PWM,
	.tm_name =                      "pm8921_tz",
	.irq_name_temp_stat =           "pm8921_tempstat_irq",
	.irq_name_over_temp =           "pm8921_overtemp_irq",
};

static struct of_device_id pm8xxx_tm_of_match[] = {
	{ .compatible = "qcom,pm8921-tm", .data = &thermal_alarm_cdata },
	{}
};
MODULE_DEVICE_TABLE(of, pm8xxx_tm_of_match);

static int pm8xxx_tm_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct device *dev = &pdev->dev;
	struct pm8xxx_tm_core_data *cdata;
	struct thermal_zone_device_ops *tz_ops;
	struct pm8xxx_tm_chip *chip;
	int rc = 0;

	if (!np) {
		dev_err(dev, "Non DT not supported\n");
		return -EINVAL;
	}

	cdata = of_match_node(pm8xxx_tm_of_match, np)->data;

	chip = devm_kzalloc(dev, sizeof(struct pm8xxx_tm_chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->dev = &pdev->dev;
	memcpy(&(chip->cdata), cdata, sizeof(struct pm8xxx_tm_core_data));

	chip->regmap = dev_get_regmap(pdev->dev.parent, NULL);
	if (!chip->regmap) {
		dev_err(&pdev->dev, "Parent regmap unavailable.\n");
		return -ENXIO;
	}

	/* Select proper thermal zone ops functions based on ADC type. */
	if (chip->cdata.adc_type == PM8XXX_TM_ADC_PM8XXX_ADC)
		tz_ops = &pm8xxx_thermal_zone_ops_pm8xxx_adc;
	else
		tz_ops = &pm8xxx_thermal_zone_ops_no_adc;

	chip->mode = THERMAL_DEVICE_ENABLED;

	chip->tz_dev = thermal_zone_device_register(chip->cdata.tm_name,
			TRIP_NUM, 0, chip, tz_ops, NULL, 0, 0);
	if (IS_ERR(chip->tz_dev)) {
		dev_err(&pdev->dev, "thermal_zone_device_register() failed.\n");
		return -ENODEV;
	}

	rc = pm8xxx_tm_init_reg(chip);
	if (rc < 0)
		goto err_free_tz;

	rc = pm8xxx_tm_shutdown_override(chip, SOFTWARE_OVERRIDE_DISABLED);
	if (rc < 0)
		goto err_free_tz;

	if (chip->cdata.adc_type == PM8XXX_TM_ADC_NONE) {
		rc = pm8xxx_tm_init_temp_no_adc(chip);
		if (rc < 0)
			goto err_free_tz;
	}

	/* Start in HW control; switch to SW control when user changes mode. */
	//chip->mode = THERMAL_DEVICE_DISABLED;
	thermal_zone_device_update(chip->tz_dev);

	INIT_DELAYED_WORK(&chip->irq_work, pm8xxx_tm_work);

	chip->tempstat_irq = platform_get_irq_byname(pdev, "pm8921_tempstat_irq");

	if (chip->tempstat_irq > 0) {
		rc = devm_request_irq(&pdev->dev, chip->tempstat_irq,
				pm8xxx_tm_isr,
			IRQF_TRIGGER_RISING, "pm8921_tempstat_irq", chip);
		if (rc < 0) {
			dev_err(&pdev->dev, "failed to request tempstat irq "
				"with error %d\n", rc);
			goto err_cancel_work;
		}
	}

	chip->overtemp_irq = platform_get_irq_byname(pdev, "pm8921_overtemp_irq");
	if (chip->overtemp_irq > 0) {
		rc = devm_request_irq(&pdev->dev, chip->overtemp_irq,
					pm8xxx_tm_isr,
				IRQF_TRIGGER_RISING, "pm8921_overtemp_irq", chip);
		if (rc < 0) {
			dev_err(&pdev->dev, "failed to request overtempstat irq "
				"with error %d\n", rc);
			goto err_cancel_work;
		}
	}

	platform_set_drvdata(pdev, chip);

	return 0;

err_cancel_work:
	cancel_delayed_work_sync(&chip->irq_work);
err_free_tz:
	thermal_zone_device_unregister(chip->tz_dev);

	return rc;
}

static int pm8xxx_tm_remove(struct platform_device *pdev)
{
	struct pm8xxx_tm_chip *chip = platform_get_drvdata(pdev);

	cancel_delayed_work_sync(&chip->irq_work);
	pm8xxx_tm_shutdown_override(chip, SOFTWARE_OVERRIDE_DISABLED);
	thermal_zone_device_unregister(chip->tz_dev);
	return 0;
}

static struct platform_driver pm8xxx_tm_driver = {
	.probe	= pm8xxx_tm_probe,
	.remove	= pm8xxx_tm_remove,

	.driver	= {
		.name = "pm8xxx-tm",
		.of_match_table = pm8xxx_tm_of_match,
	},
};

module_platform_driver(pm8xxx_tm_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("PM8921 Thermal Manager driver");
MODULE_VERSION("1.0");
MODULE_ALIAS("platform:pm8xxx-tm");
