/* Copyright (c) 2011-2012, Code Aurora Forum. All rights reserved.
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
 */
#define pr_fmt(fmt)	"%s: " fmt, __func__

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/errno.h>
#include <linux/mfd/pm8xxx/pm8921-charger.h>
#include <linux/mfd/pm8xxx/pm8921-bms.h>
#include <linux/mfd/pm8xxx/pm8xxx-adc.h>
#include <linux/mfd/pm8xxx/ccadc.h>
#include <linux/mfd/pm8xxx/core.h>
#include <linux/mfd/pm8xxx/misc.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/bitops.h>
#include <linux/workqueue.h>
#include <linux/debugfs.h>
#include <linux/slab.h>

#include <mach/msm_xo.h>
#include <mach/msm_hsusb.h>

#ifdef CONFIG_LGE_PM
#include <mach/board_lge.h>
#endif

#ifdef CONFIG_LGE_PM_REMOVE_BATT
#include <linux/reboot.h>
#endif

#define ChgLog(x, ...) printk(x, ##__VA_ARGS__)

#ifdef CONFIG_LGE_PM_ADP_CHG
/* Adapive USB draw current limit */
enum {
	IUSB_MAX_NONE,
	IUSB_MAX_INCREASE,
	IUSB_MAX_DECREASE,
};

#if defined(CONFIG_MACH_APQ8064_J1KD) || defined(CONFIG_MACH_APQ8064_J1D)

	#define ADAPTIVE_USB_CURRENT_CHECK_PERIOD_MS    (5000)
	#define ADAPTIVE_USB_CURRENT_DROP_CHECK_MS      (500)
	#define CHARGING_COLLAPSE_VOLTAGE               (4500)

	#define ADAPTIVE_NUM_OF_TABLE                   (2)
	static int ADAPTIVE_MA_TABLE[ADAPTIVE_NUM_OF_TABLE] = { 900, 1500 };

#elif defined(CONFIG_MACH_APQ8064_GKSK) || defined(CONFIG_MACH_APQ8064_GKKT) || defined(CONFIG_MACH_APQ8064_GKU)

	#define ADAPTIVE_USB_CURRENT_CHECK_PERIOD_MS 	(1000)
	#define ADAPTIVE_USB_CURRENT_DROP_CHECK_MS		(100)
	#define CHARGING_COLLAPSE_VOLTAGE				(the_chip->max_voltage_mv)

	#define ADAPTIVE_NUM_OF_TABLE                   (5)
	static int ADAPTIVE_MA_TABLE[ADAPTIVE_NUM_OF_TABLE] = { 500, 700, 850, 900, 1500 };

#else

	#define ADAPTIVE_USB_CURRENT_CHECK_PERIOD_MS    (1000)
	#define ADAPTIVE_USB_CURRENT_DROP_CHECK_MS      (100)
	#define CHARGING_COLLAPSE_VOLTAGE               (the_chip->max_voltage_mv)

	#define ADAPTIVE_NUM_OF_TABLE                   (4)
	static int ADAPTIVE_MA_TABLE[ADAPTIVE_NUM_OF_TABLE] = { 500, 700, 850, 900 };

#endif

static struct delayed_work      adaptive_usb_current_work;
static struct workqueue_struct *adaptive_usb_current_queue;
static int search_iusb_max_status;
#endif /* End of CONFIG_LGE_PM_ADP_CHG */

#define CHG_BUCK_CLOCK_CTRL	0x14

#define PBL_ACCESS1		0x04
#define PBL_ACCESS2		0x05
#define SYS_CONFIG_1		0x06
#define SYS_CONFIG_2		0x07
#define CHG_CNTRL		0x204
#define CHG_IBAT_MAX		0x205
#define CHG_TEST		0x206
#define CHG_BUCK_CTRL_TEST1	0x207
#define CHG_BUCK_CTRL_TEST2	0x208
#define CHG_BUCK_CTRL_TEST3	0x209
#define COMPARATOR_OVERRIDE	0x20A
#define PSI_TXRX_SAMPLE_DATA_0	0x20B
#define PSI_TXRX_SAMPLE_DATA_1	0x20C
#define PSI_TXRX_SAMPLE_DATA_2	0x20D
#define PSI_TXRX_SAMPLE_DATA_3	0x20E
#define PSI_CONFIG_STATUS	0x20F
#define CHG_IBAT_SAFE		0x210
#define CHG_ITRICKLE		0x211
#define CHG_CNTRL_2		0x212
#define CHG_VBAT_DET		0x213
#define CHG_VTRICKLE		0x214
#define CHG_ITERM		0x215
#define CHG_CNTRL_3		0x216
#define CHG_VIN_MIN		0x217
#define CHG_TWDOG		0x218
#define CHG_TTRKL_MAX		0x219
#define CHG_TEMP_THRESH		0x21A
#define CHG_TCHG_MAX		0x21B
#define USB_OVP_CONTROL		0x21C
#define DC_OVP_CONTROL		0x21D
#define USB_OVP_TEST		0x21E
#define DC_OVP_TEST		0x21F
#define CHG_VDD_MAX		0x220
#define CHG_VDD_SAFE		0x221
#define CHG_VBAT_BOOT_THRESH	0x222
#define USB_OVP_TRIM		0x355
#define BUCK_CONTROL_TRIM1	0x356
#define BUCK_CONTROL_TRIM2	0x357
#define BUCK_CONTROL_TRIM3	0x358
#define BUCK_CONTROL_TRIM4	0x359
#define CHG_DEFAULTS_TRIM	0x35A
#define CHG_ITRIM		0x35B
#define CHG_TTRIM		0x35C
#define CHG_COMP_OVR		0x20A
#define IUSB_FINE_RES		0x2B6

/* check EOC every 10 seconds */
#define EOC_CHECK_PERIOD_MS	10000
/* check for USB unplug every 200 msecs */
#define UNPLUG_CHECK_WAIT_PERIOD_MS 200

enum chg_fsm_state {
	FSM_STATE_OFF_0 = 0,
	FSM_STATE_BATFETDET_START_12 = 12,
	FSM_STATE_BATFETDET_END_16 = 16,
	FSM_STATE_ON_CHG_HIGHI_1 = 1,
	FSM_STATE_ATC_2A = 2,
	FSM_STATE_ATC_2B = 18,
	FSM_STATE_ON_BAT_3 = 3,
	FSM_STATE_ATC_FAIL_4 = 4 ,
	FSM_STATE_DELAY_5 = 5,
	FSM_STATE_ON_CHG_AND_BAT_6 = 6,
	FSM_STATE_FAST_CHG_7 = 7,
	FSM_STATE_TRKL_CHG_8 = 8,
	FSM_STATE_CHG_FAIL_9 = 9,
	FSM_STATE_EOC_10 = 10,
	FSM_STATE_ON_CHG_VREGOK_11 = 11,
	FSM_STATE_ATC_PAUSE_13 = 13,
	FSM_STATE_FAST_CHG_PAUSE_14 = 14,
	FSM_STATE_TRKL_CHG_PAUSE_15 = 15,
	FSM_STATE_START_BOOT = 20,
	FSM_STATE_FLCB_VREGOK = 21,
	FSM_STATE_FLCB = 22,
};

struct fsm_state_to_batt_status {
	enum chg_fsm_state	fsm_state;
	int			batt_state;
};

static struct fsm_state_to_batt_status map[] = {
	{FSM_STATE_OFF_0, POWER_SUPPLY_STATUS_UNKNOWN},
	{FSM_STATE_BATFETDET_START_12, POWER_SUPPLY_STATUS_UNKNOWN},
	{FSM_STATE_BATFETDET_END_16, POWER_SUPPLY_STATUS_UNKNOWN},
	/*
	 * for CHG_HIGHI_1 report NOT_CHARGING if battery missing,
	 * too hot/cold, charger too hot
	 */
	{FSM_STATE_ON_CHG_HIGHI_1, POWER_SUPPLY_STATUS_FULL},
	{FSM_STATE_ATC_2A, POWER_SUPPLY_STATUS_CHARGING},
	{FSM_STATE_ATC_2B, POWER_SUPPLY_STATUS_CHARGING},
	{FSM_STATE_ON_BAT_3, POWER_SUPPLY_STATUS_DISCHARGING},
	{FSM_STATE_ATC_FAIL_4, POWER_SUPPLY_STATUS_DISCHARGING},
	{FSM_STATE_DELAY_5, POWER_SUPPLY_STATUS_UNKNOWN },
	{FSM_STATE_ON_CHG_AND_BAT_6, POWER_SUPPLY_STATUS_CHARGING},
	{FSM_STATE_FAST_CHG_7, POWER_SUPPLY_STATUS_CHARGING},
	{FSM_STATE_TRKL_CHG_8, POWER_SUPPLY_STATUS_CHARGING},
	{FSM_STATE_CHG_FAIL_9, POWER_SUPPLY_STATUS_DISCHARGING},
	{FSM_STATE_EOC_10, POWER_SUPPLY_STATUS_FULL},
	{FSM_STATE_ON_CHG_VREGOK_11, POWER_SUPPLY_STATUS_NOT_CHARGING},
	{FSM_STATE_ATC_PAUSE_13, POWER_SUPPLY_STATUS_NOT_CHARGING},
	{FSM_STATE_FAST_CHG_PAUSE_14, POWER_SUPPLY_STATUS_NOT_CHARGING},
	{FSM_STATE_TRKL_CHG_PAUSE_15, POWER_SUPPLY_STATUS_NOT_CHARGING},
	{FSM_STATE_START_BOOT, POWER_SUPPLY_STATUS_NOT_CHARGING},
	{FSM_STATE_FLCB_VREGOK, POWER_SUPPLY_STATUS_NOT_CHARGING},
	{FSM_STATE_FLCB, POWER_SUPPLY_STATUS_NOT_CHARGING},
};

enum chg_regulation_loop {
	VDD_LOOP = BIT(3),
	BAT_CURRENT_LOOP = BIT(2),
	INPUT_CURRENT_LOOP = BIT(1),
	INPUT_VOLTAGE_LOOP = BIT(0),
	CHG_ALL_LOOPS = VDD_LOOP | BAT_CURRENT_LOOP
			| INPUT_CURRENT_LOOP | INPUT_VOLTAGE_LOOP,
};

enum pmic_chg_interrupts {
	USBIN_VALID_IRQ = 0,
	USBIN_OV_IRQ,
	BATT_INSERTED_IRQ,
	VBATDET_LOW_IRQ,
	USBIN_UV_IRQ,
	VBAT_OV_IRQ,
	CHGWDOG_IRQ,
	VCP_IRQ,
	ATCDONE_IRQ,
	ATCFAIL_IRQ,
	CHGDONE_IRQ,
	CHGFAIL_IRQ,
	CHGSTATE_IRQ,
	LOOP_CHANGE_IRQ,
	FASTCHG_IRQ,
	TRKLCHG_IRQ,
	BATT_REMOVED_IRQ,
	BATTTEMP_HOT_IRQ,
	CHGHOT_IRQ,
	BATTTEMP_COLD_IRQ,
	CHG_GONE_IRQ,
	BAT_TEMP_OK_IRQ,
	COARSE_DET_LOW_IRQ,
	VDD_LOOP_IRQ,
	VREG_OV_IRQ,
	VBATDET_IRQ,
	BATFET_IRQ,
	PSI_IRQ,
	DCIN_VALID_IRQ,
	DCIN_OV_IRQ,
	DCIN_UV_IRQ,
	PM_CHG_MAX_INTS,
};

struct bms_notify {
	int			is_battery_full;
	int			is_charging;
#ifdef CONFIG_LGE_PM
	int			is_charge_done_hit;
#endif
	struct	work_struct	work;
};

/**
 * struct pm8921_chg_chip -device information
 * @dev:			device pointer to access the parent
 * @usb_present:		present status of usb
 * @dc_present:			present status of dc
 * @usb_charger_current:	usb current to charge the battery with used when
 *				the usb path is enabled or charging is resumed
 * @safety_time:		max time for which charging will happen
 * @update_time:		how frequently the userland needs to be updated
 * @max_voltage_mv:		the max volts the batt should be charged up to
 * @min_voltage_mv:		the min battery voltage before turning the FETon
 * @cool_temp_dc:		the cool temp threshold in deciCelcius
 * @warm_temp_dc:		the warm temp threshold in deciCelcius
 * @resume_voltage_delta:	the voltage delta from vdd max at which the
 *				battery should resume charging
 * @term_current:		The charging based term current
 *
 */
struct pm8921_chg_chip {
	struct device			*dev;
	unsigned int			usb_present;
	unsigned int			dc_present;
	unsigned int			usb_charger_current;
	unsigned int			max_bat_chg_current;
	unsigned int			pmic_chg_irq[PM_CHG_MAX_INTS];
	unsigned int			safety_time;
	unsigned int			ttrkl_time;
	unsigned int			update_time;
	unsigned int			max_voltage_mv;
	unsigned int			min_voltage_mv;
	int				cool_temp_dc;
	int				warm_temp_dc;
	unsigned int			temp_check_period;
	unsigned int			cool_bat_chg_current;
	unsigned int			warm_bat_chg_current;
	unsigned int			cool_bat_voltage;
	unsigned int			warm_bat_voltage;
	unsigned int			is_bat_cool;
	unsigned int			is_bat_warm;
	unsigned int			resume_voltage_delta;
	unsigned int			term_current;
	unsigned int			vbat_channel;
	unsigned int			batt_temp_channel;
	unsigned int			batt_id_channel;
	struct power_supply		usb_psy;
	struct power_supply		dc_psy;
	struct power_supply		*ext_psy;
	struct power_supply		batt_psy;
	struct dentry			*dent;
	struct bms_notify		bms_notify;
	bool				keep_btm_on_suspend;
	bool				ext_charging;
	bool				ext_charge_done;
	bool				iusb_fine_res;
	DECLARE_BITMAP(enabled_irqs, PM_CHG_MAX_INTS);
	struct work_struct		battery_id_valid_work;
	int64_t				batt_id_min;
	int64_t				batt_id_max;
	int				trkl_voltage;
	int				weak_voltage;
	int				trkl_current;
	int				weak_current;
	int				vin_min;
	unsigned int			*thermal_mitigation;
	int				thermal_levels;
	struct delayed_work		update_heartbeat_work;
	struct delayed_work		eoc_work;
	struct delayed_work		unplug_check_work;
	struct delayed_work		vin_collapse_check_work;
	struct wake_lock		eoc_wake_lock;
	enum pm8921_chg_cold_thr	cold_thr;
	enum pm8921_chg_hot_thr		hot_thr;
	int				rconn_mohm;
	enum pm8921_chg_led_src_config	led_src_config;
	int				recent_reported_soc;
#ifdef CONFIG_LGE_PM_ADP_CHG
	struct delayed_work		unplug_wrkarnd_restore_work;
	struct wake_lock		unplug_wrkarnd_restore_wake_lock;
#endif
#ifdef CONFIG_LGE_CHARGER_TEMP_SCENARIO
	/* Create work for temp scerario kwangjae1.lee@lge.com */
	struct delayed_work		monitor_batt_temp_work;
	struct wake_lock        monitor_batt_temp_wake_lock;
	int				temp_level_1;
#if defined(CONFIG_MACH_APQ8064_J1SP)
	/* Add temp for charing scenario on SPRINT */
	int				temp_level_1_1;
#endif
	int				temp_level_2;
	int				temp_level_3;
	int				temp_level_4;
	int				temp_level_5;
#endif
#ifdef CONFIG_LGE_PM_REMOVE_BATT
	/* Restart the machine when Battery Remove/Insert - seonghun1.kim */
	struct work_struct		kernel_restart_work;
#endif
#ifdef CONFIG_LGE_PM
	/* Wake lock for deliver Uevent before suspend */
	struct wake_lock		deliver_uevent_wake_lock;
#endif
};

#ifdef CONFIG_LGE_CHARGER_TEMP_SCENARIO
#define BATT_TEMP_LEVEL_POWER_OFF	(60)	/* UI code force power off if above this level. */
#define BATT_TEMP_LEVEL_OVERHEAT	(55)	/* chargerlogo consider OVERHEAT if above this level. */
#define BATT_TEMP_LEVEL_COLD		(-10)	/* chargerlogo consider COLD if below this level. */
enum {
	THERM_M10,
	THERM_M5,
	THERM_42,
	THERM_45,
	THERM_55,
	THERM_57,
	THERM_60,
	THERM_65,
	THERM_LAST,
};

enum {
	DISCHG_BATT_TEMP_OVER_60,
	DISCHG_BATT_TEMP_57_60,
	DISCHG_BATT_TEMP_UNDER_57,
	CHG_BATT_TEMP_LEVEL_1, // OVER_55
	CHG_BATT_TEMP_LEVEL_2, // 46_55
	CHG_BATT_TEMP_LEVEL_3, // 42_45
	CHG_BATT_TEMP_LEVEL_4, // M4_41
	CHG_BATT_TEMP_LEVEL_5, // M10_M5
	CHG_BATT_TEMP_LEVEL_6, // UNDER_M10
#if defined(CONFIG_MACH_APQ8064_J1SP)
	/* Add temp for charing scenario on SPRINT */
	CHG_BATT_TEMP_LEVEL_1_1,
#endif
};

enum {
	DISCHG_BATT_NORMAL_STATE,
	DISCHG_BATT_WARNING_STATE,
	DISCHG_BATT_POWEROFF_STATE,
	CHG_BATT_NORMAL_STATE,
	CHG_BATT_DC_CURRENT_STATE,
	CHG_BATT_WARNING_STATE,
	CHG_BATT_STOP_CHARGING_STATE,
};
#endif

/* user space parameter to limit usb current */
static unsigned int usb_max_current;
/*
 * usb_target_ma is used for wall charger
 * adaptive input current limiting only. Use
 * pm_iusbmax_get() to get current maximum usb current setting.
 */
static int usb_target_ma;
static int charging_disabled;
static int thermal_mitigation;

static struct pm8921_chg_chip *the_chip;

static struct pm8xxx_adc_arb_btm_param btm_config;

#ifdef CONFIG_LGE_CHARGER_TEMP_SCENARIO
static unsigned int last_stop_charging;
static unsigned int chg_batt_temp_state;
/*
 * kiwone.seo@lge.com 2011-0609
 * for show charging ani.although charging is stopped : charging scenario
 */
static int pseudo_ui_charging;
static int last_usb_chg_current;
/* 120712 cs.kim@lge.com Implements Thermal Mitigation iUSB setting */
static int pre_mitigation;
static int soc_limit;
#endif

#ifdef CONFIG_LGE_PM
/* add for thermister test kwangjae1.lee@lge.com */
struct pseudo_batt_info_type pseudo_batt_info = {
        .mode = 0,
};
/* kwangjae1.lee@lge.com 2012-06-11 Add bms debugger */
struct bms_batt_info_type bms_batt_info = {
        .mode = 0,
};
/* 1 : charing, 0 : block charging */
static int block_charging_state = 1;
#endif

#ifdef CONFIG_LGE_PM_LOW_BATT_CHG
/* this is indicator that chargerlogo app is running. */
extern int chargerlogo_state;
#endif

#if defined(CONFIG_MACH_APQ8064_J1KD) || defined(CONFIG_MACH_APQ8064_J1D)
int coincell_charging = 1;
#endif

static int pm_chg_masked_write(struct pm8921_chg_chip *chip, u16 addr,
							u8 mask, u8 val)
{
	int rc;
	u8 reg;

	rc = pm8xxx_readb(chip->dev->parent, addr, &reg);
	if (rc) {
		pr_err("pm8xxx_readb failed: addr=%03X, rc=%d\n", addr, rc);
		return rc;
	}
	reg &= ~mask;
	reg |= val & mask;
	rc = pm8xxx_writeb(chip->dev->parent, addr, reg);
	if (rc) {
		pr_err("pm8xxx_writeb failed: addr=%03X, rc=%d\n", addr, rc);
		return rc;
	}
	return 0;
}

#ifdef CONFIG_LGE_PM
static int get_prop_batt_temp(struct pm8921_chg_chip *chip);
#endif
static int pm_chg_get_rt_status(struct pm8921_chg_chip *chip, int irq_id)
{
#ifdef CONFIG_LGE_PM
	/* fake battery */
	if ((irq_id == BATTTEMP_COLD_IRQ) && (get_prop_batt_temp(chip) == -30))
		return 0;
#endif

	return pm8xxx_read_irq_stat(chip->dev->parent,
					chip->pmic_chg_irq[irq_id]);
}

/* Treat OverVoltage/UnderVoltage as source missing */
static int is_usb_chg_plugged_in(struct pm8921_chg_chip *chip)
{
	return pm_chg_get_rt_status(chip, USBIN_VALID_IRQ);
}

/* Treat OverVoltage/UnderVoltage as source missing */
static int is_dc_chg_plugged_in(struct pm8921_chg_chip *chip)
{
	return pm_chg_get_rt_status(chip, DCIN_VALID_IRQ);
}

static int is_batfet_closed(struct pm8921_chg_chip *chip)
{
	return pm_chg_get_rt_status(chip, BATFET_IRQ);
}

#ifdef CONFIG_LGE_PM
static bool is_factory_cable(void)
{
	acc_cable_type cable_type = lge_pm_get_cable_type();

	if (cable_type == CABLE_56K ||
		cable_type == CABLE_130K ||
		cable_type == CABLE_910K)
		return 1;
	else
		return 0;
}

/* jungwoo.yun@lge.com 2012-07-21 asked by mass production SW*/
static bool is_real_battery_or_factory_cable(struct pm8921_chg_chip *chip)
{
	if (pm_chg_get_rt_status(chip, BATT_INSERTED_IRQ))
		return 0;

	if (is_factory_cable())
		return 1;
	else
		return 0;
}
#endif

#define CAPTURE_FSM_STATE_CMD	0xC2
#define READ_BANK_7		0x70
#define READ_BANK_4		0x40
static int pm_chg_get_fsm_state(struct pm8921_chg_chip *chip)
{
	u8 temp;
	int err, ret = 0;

	temp = CAPTURE_FSM_STATE_CMD;
	err = pm8xxx_writeb(chip->dev->parent, CHG_TEST, temp);
	if (err) {
		pr_err("Error %d writing %d to addr %d\n", err, temp, CHG_TEST);
		return err;
	}

	temp = READ_BANK_7;
	err = pm8xxx_writeb(chip->dev->parent, CHG_TEST, temp);
	if (err) {
		pr_err("Error %d writing %d to addr %d\n", err, temp, CHG_TEST);
		return err;
	}

	err = pm8xxx_readb(chip->dev->parent, CHG_TEST, &temp);
	if (err) {
		pr_err("pm8xxx_readb fail: addr=%03X, rc=%d\n", CHG_TEST, err);
		return err;
	}
	/* get the lower 4 bits */
	ret = temp & 0xF;

	temp = READ_BANK_4;
	err = pm8xxx_writeb(chip->dev->parent, CHG_TEST, temp);
	if (err) {
		pr_err("Error %d writing %d to addr %d\n", err, temp, CHG_TEST);
		return err;
	}

	err = pm8xxx_readb(chip->dev->parent, CHG_TEST, &temp);
	if (err) {
		pr_err("pm8xxx_readb fail: addr=%03X, rc=%d\n", CHG_TEST, err);
		return err;
	}
	/* get the upper 1 bit */
	ret |= (temp & 0x1) << 4;
	return  ret;
}

#define READ_BANK_6		0x60
static int pm_chg_get_regulation_loop(struct pm8921_chg_chip *chip)
{
	u8 temp;
	int err;

	temp = READ_BANK_6;
	err = pm8xxx_writeb(chip->dev->parent, CHG_TEST, temp);
	if (err) {
		pr_err("Error %d writing %d to addr %d\n", err, temp, CHG_TEST);
		return err;
	}

	err = pm8xxx_readb(chip->dev->parent, CHG_TEST, &temp);
	if (err) {
		pr_err("pm8xxx_readb fail: addr=%03X, rc=%d\n", CHG_TEST, err);
		return err;
	}

	/* return the lower 4 bits */
	return temp & CHG_ALL_LOOPS;
}

#define CHG_USB_SUSPEND_BIT  BIT(2)
static int pm_chg_usb_suspend_enable(struct pm8921_chg_chip *chip, int enable)
{
	return pm_chg_masked_write(chip, CHG_CNTRL_3, CHG_USB_SUSPEND_BIT,
			enable ? CHG_USB_SUSPEND_BIT : 0);
}

#define CHG_EN_BIT	BIT(7)
static int pm_chg_auto_enable(struct pm8921_chg_chip *chip, int enable)
{
	ChgLog("ChgLog> %s : %d\n",__func__,enable);
	return pm_chg_masked_write(chip, CHG_CNTRL_3, CHG_EN_BIT,
				enable ? CHG_EN_BIT : 0);
}

#define CHG_FAILED_CLEAR	BIT(0)
#define ATC_FAILED_CLEAR	BIT(1)
static int pm_chg_failed_clear(struct pm8921_chg_chip *chip, int clear)
{
	int rc;

	rc = pm_chg_masked_write(chip, CHG_CNTRL_3, ATC_FAILED_CLEAR,
				clear ? ATC_FAILED_CLEAR : 0);
	rc |= pm_chg_masked_write(chip, CHG_CNTRL_3, CHG_FAILED_CLEAR,
				clear ? CHG_FAILED_CLEAR : 0);
	return rc;
}

#define CHG_CHARGE_DIS_BIT	BIT(1)
static int pm_chg_charge_dis(struct pm8921_chg_chip *chip, int disable)
{
	ChgLog("ChgLog> %s : %d\n",__func__,disable);
	return pm_chg_masked_write(chip, CHG_CNTRL, CHG_CHARGE_DIS_BIT,
				disable ? CHG_CHARGE_DIS_BIT : 0);
}

static int pm_is_chg_charge_dis(struct pm8921_chg_chip *chip)
{
	u8 temp;

	pm8xxx_readb(chip->dev->parent, CHG_CNTRL, &temp);
	return  temp & CHG_CHARGE_DIS_BIT;
}
#define PM8921_CHG_V_MIN_MV	3240
#define PM8921_CHG_V_STEP_MV	20
#define PM8921_CHG_V_STEP_10MV_OFFSET_BIT	BIT(7)
#define PM8921_CHG_VDDMAX_MAX	4500
#define PM8921_CHG_VDDMAX_MIN	3400
#define PM8921_CHG_V_MASK	0x7F
static int __pm_chg_vddmax_set(struct pm8921_chg_chip *chip, int voltage)
{
	int remainder;
	u8 temp = 0;

	if (voltage < PM8921_CHG_VDDMAX_MIN
			|| voltage > PM8921_CHG_VDDMAX_MAX) {
		pr_err("bad mV=%d asked to set\n", voltage);
		return -EINVAL;
	}

	temp = (voltage - PM8921_CHG_V_MIN_MV) / PM8921_CHG_V_STEP_MV;

	remainder = voltage % 20;
	if (remainder >= 10) {
		temp |= PM8921_CHG_V_STEP_10MV_OFFSET_BIT;
	}

	pr_debug("voltage=%d setting %02x\n", voltage, temp);
	return pm8xxx_writeb(chip->dev->parent, CHG_VDD_MAX, temp);
}

static int pm_chg_vddmax_get(struct pm8921_chg_chip *chip, int *voltage)
{
	u8 temp;
	int rc;

	rc = pm8xxx_readb(chip->dev->parent, CHG_VDD_MAX, &temp);
	if (rc) {
		pr_err("rc = %d while reading vdd max\n", rc);
		*voltage = 0;
		return rc;
	}
	*voltage = (int)(temp & PM8921_CHG_V_MASK) * PM8921_CHG_V_STEP_MV
							+ PM8921_CHG_V_MIN_MV;
	if (temp & PM8921_CHG_V_STEP_10MV_OFFSET_BIT)
		*voltage =  *voltage + 10;
	return 0;
}

static int pm_chg_vddmax_set(struct pm8921_chg_chip *chip, int voltage)
{
	int current_mv, ret, steps, i;
	bool increase;

	ret = 0;

	ChgLog("ChgLog> %s : %d\n",__func__,voltage);
	if (voltage < PM8921_CHG_VDDMAX_MIN
		|| voltage > PM8921_CHG_VDDMAX_MAX) {
		pr_err("bad mV=%d asked to set\n", voltage);
		return -EINVAL;
	}

	ret = pm_chg_vddmax_get(chip, &current_mv);
	if (ret) {
		pr_err("Failed to read vddmax rc=%d\n", ret);
		return -EINVAL;
	}
	if (current_mv == voltage)
		return 0;

	/* Only change in increments when USB is present */
	if (is_usb_chg_plugged_in(chip)) {
		if (current_mv < voltage) {
			steps = (voltage - current_mv) / PM8921_CHG_V_STEP_MV;
			increase = true;
		} else {
			steps = (current_mv - voltage) / PM8921_CHG_V_STEP_MV;
			increase = false;
		}
		for (i = 0; i < steps; i++) {
			if (increase)
				current_mv += PM8921_CHG_V_STEP_MV;
			else
				current_mv -= PM8921_CHG_V_STEP_MV;
			ret |= __pm_chg_vddmax_set(chip, current_mv);
		}
	}
	ret |= __pm_chg_vddmax_set(chip, voltage);
	return ret;
}

#define PM8921_CHG_VDDSAFE_MIN	3400
#define PM8921_CHG_VDDSAFE_MAX	4500
static int pm_chg_vddsafe_set(struct pm8921_chg_chip *chip, int voltage)
{
	u8 temp;

	if (voltage < PM8921_CHG_VDDSAFE_MIN
			|| voltage > PM8921_CHG_VDDSAFE_MAX) {
		pr_err("bad mV=%d asked to set\n", voltage);
		return -EINVAL;
	}
	temp = (voltage - PM8921_CHG_V_MIN_MV) / PM8921_CHG_V_STEP_MV;
	pr_debug("voltage=%d setting %02x\n", voltage, temp);
	return pm_chg_masked_write(chip, CHG_VDD_SAFE, PM8921_CHG_V_MASK, temp);
}

#define PM8921_CHG_VBATDET_MIN	3240
#define PM8921_CHG_VBATDET_MAX	5780
static int pm_chg_vbatdet_set(struct pm8921_chg_chip *chip, int voltage)
{
	u8 temp;

	if (voltage < PM8921_CHG_VBATDET_MIN
			|| voltage > PM8921_CHG_VBATDET_MAX) {
		pr_err("bad mV=%d asked to set\n", voltage);
		return -EINVAL;
	}
	temp = (voltage - PM8921_CHG_V_MIN_MV) / PM8921_CHG_V_STEP_MV;
	pr_debug("voltage=%d setting %02x\n", voltage, temp);
	return pm_chg_masked_write(chip, CHG_VBAT_DET, PM8921_CHG_V_MASK, temp);
}

#define PM8921_CHG_VINMIN_MIN_MV	3800
#define PM8921_CHG_VINMIN_STEP_MV	100
#define PM8921_CHG_VINMIN_USABLE_MAX	6500
#define PM8921_CHG_VINMIN_USABLE_MIN	4300
#define PM8921_CHG_VINMIN_MASK		0x1F
static int pm_chg_vinmin_set(struct pm8921_chg_chip *chip, int voltage)
{
	u8 temp;

	if (voltage < PM8921_CHG_VINMIN_USABLE_MIN
			|| voltage > PM8921_CHG_VINMIN_USABLE_MAX) {
		pr_err("bad mV=%d asked to set\n", voltage);
		return -EINVAL;
	}
	temp = (voltage - PM8921_CHG_VINMIN_MIN_MV) / PM8921_CHG_VINMIN_STEP_MV;
	pr_debug("voltage=%d setting %02x\n", voltage, temp);
	return pm_chg_masked_write(chip, CHG_VIN_MIN, PM8921_CHG_VINMIN_MASK,
									temp);
}

static int pm_chg_vinmin_get(struct pm8921_chg_chip *chip)
{
	u8 temp;
	int rc, voltage_mv;

	rc = pm8xxx_readb(chip->dev->parent, CHG_VIN_MIN, &temp);
	temp &= PM8921_CHG_VINMIN_MASK;

	voltage_mv = PM8921_CHG_VINMIN_MIN_MV +
			(int)temp * PM8921_CHG_VINMIN_STEP_MV;

	return voltage_mv;
}

#define PM8921_CHG_IBATMAX_MIN	325
#define PM8921_CHG_IBATMAX_MAX	2000
#define PM8921_CHG_I_MIN_MA	225
#define PM8921_CHG_I_STEP_MA	50
#define PM8921_CHG_I_MASK	0x3F

static int pm_chg_ibatmax_set(struct pm8921_chg_chip *chip, int chg_current)
{
	u8 temp;

	ChgLog("ChgLog> %s : %d\n",__func__,chg_current);
	if (chg_current < PM8921_CHG_IBATMAX_MIN
			|| chg_current > PM8921_CHG_IBATMAX_MAX) {
		pr_err("bad mA=%d asked to set\n", chg_current);
		return -EINVAL;
	}
	temp = (chg_current - PM8921_CHG_I_MIN_MA) / PM8921_CHG_I_STEP_MA;
	return pm_chg_masked_write(chip, CHG_IBAT_MAX, PM8921_CHG_I_MASK, temp);
}

#define PM8921_CHG_IBATSAFE_MIN	225
#define PM8921_CHG_IBATSAFE_MAX	3375
static int pm_chg_ibatsafe_set(struct pm8921_chg_chip *chip, int chg_current)
{
	u8 temp;

	ChgLog("ChgLog> %s : %d\n",__func__,chg_current);

	if (chg_current < PM8921_CHG_IBATSAFE_MIN
			|| chg_current > PM8921_CHG_IBATSAFE_MAX) {
		pr_err("bad mA=%d asked to set\n", chg_current);
		return -EINVAL;
	}
	temp = (chg_current - PM8921_CHG_I_MIN_MA) / PM8921_CHG_I_STEP_MA;
	return pm_chg_masked_write(chip, CHG_IBAT_SAFE,
						PM8921_CHG_I_MASK, temp);
}

#define PM8921_CHG_ITERM_MIN_MA		50
#define PM8921_CHG_ITERM_MAX_MA		200
#define PM8921_CHG_ITERM_STEP_MA	10
#define PM8921_CHG_ITERM_MASK		0xF
static int pm_chg_iterm_set(struct pm8921_chg_chip *chip, int chg_current)
{
	u8 temp;

	ChgLog("ChgLog> %s : %d\n",__func__,chg_current);

	if (chg_current < PM8921_CHG_ITERM_MIN_MA
			|| chg_current > PM8921_CHG_ITERM_MAX_MA) {
		pr_err("bad mA=%d asked to set\n", chg_current);
		return -EINVAL;
	}

	temp = (chg_current - PM8921_CHG_ITERM_MIN_MA)
				/ PM8921_CHG_ITERM_STEP_MA;
	return pm_chg_masked_write(chip, CHG_ITERM, PM8921_CHG_ITERM_MASK,
					 temp);
}

static int pm_chg_iterm_get(struct pm8921_chg_chip *chip, int *chg_current)
{
	u8 temp;
	int rc;

	rc = pm8xxx_readb(chip->dev->parent, CHG_ITERM, &temp);
	if (rc) {
		pr_err("err=%d reading CHG_ITEM\n", rc);
		*chg_current = 0;
		return rc;
	}
	temp &= PM8921_CHG_ITERM_MASK;
	*chg_current = (int)temp * PM8921_CHG_ITERM_STEP_MA
					+ PM8921_CHG_ITERM_MIN_MA;
	return 0;
}

struct usb_ma_limit_entry {
	int	usb_ma;
	u8	value;
};

static struct usb_ma_limit_entry usb_ma_table[] = {
	{100, 0x0},
	{200, 0x1},
	{500, 0x2},
	{600, 0x3},
	{700, 0x4},
	{800, 0x5},
	{850, 0x6},
	{900, 0x8},
	{950, 0x7},
	{1000, 0x9},
	{1100, 0xA},
	{1200, 0xB},
	{1300, 0xC},
	{1400, 0xD},
	{1500, 0xE},
	{1600, 0xF},
};

#define PM8921_CHG_IUSB_MASK 0x1C
#define PM8921_CHG_IUSB_SHIFT 2
#define PM8921_CHG_IUSB_MAX  7
#define PM8921_CHG_IUSB_MIN  0
#define PM8917_IUSB_FINE_RES BIT(0)

#ifdef CONFIG_LGE_PM
#define PM8921_CHG_IBATT_FACTORY_CABLE 600
#endif

static int pm_chg_iusbmax_set(struct pm8921_chg_chip *chip, int reg_val)
{
	u8 temp, fineres;
	int rc;

	ChgLog("ChgLog> %s : %d, %d mA\n",__func__,reg_val,usb_ma_table[reg_val].usb_ma);

	fineres = PM8917_IUSB_FINE_RES & usb_ma_table[reg_val].value;
	reg_val = usb_ma_table[reg_val].value >> 1;

	if (reg_val < PM8921_CHG_IUSB_MIN || reg_val > PM8921_CHG_IUSB_MAX) {
		pr_err("bad mA=%d asked to set\n", reg_val);
		return -EINVAL;
	}
	temp = reg_val << PM8921_CHG_IUSB_SHIFT;

	/* IUSB_FINE_RES */
	if (chip->iusb_fine_res) {
		/* Clear IUSB_FINE_RES bit to avoid overshoot */
		rc = pm_chg_masked_write(chip, IUSB_FINE_RES,
			PM8917_IUSB_FINE_RES, 0);

		rc |= pm_chg_masked_write(chip, PBL_ACCESS2,
			PM8921_CHG_IUSB_MASK, temp);

		if (rc) {
			pr_err("Failed to write PBL_ACCESS2 rc=%d\n", rc);
			return rc;
		}

		if (fineres) {
			rc = pm_chg_masked_write(chip, IUSB_FINE_RES,
				PM8917_IUSB_FINE_RES, fineres);
			if (rc)
				pr_err("Failed to write ISUB_FINE_RES rc=%d\n",
					rc);
		}
	} else {
		rc = pm_chg_masked_write(chip, PBL_ACCESS2,
			PM8921_CHG_IUSB_MASK, temp);
		if (rc)
			pr_err("Failed to write PBL_ACCESS2 rc=%d\n", rc);
	}

	return rc;
}

static int pm_chg_iusbmax_get(struct pm8921_chg_chip *chip, int *mA)
{
	u8 temp, fineres;
	int rc, i;

	fineres = 0;
	*mA = 0;
	rc = pm8xxx_readb(chip->dev->parent, PBL_ACCESS2, &temp);
	if (rc) {
		pr_err("err=%d reading PBL_ACCESS2\n", rc);
		return rc;
	}

	if (chip->iusb_fine_res) {
		rc = pm8xxx_readb(chip->dev->parent, IUSB_FINE_RES, &fineres);
		if (rc) {
			pr_err("err=%d reading IUSB_FINE_RES\n", rc);
			return rc;
		}
	}
	temp &= PM8921_CHG_IUSB_MASK;
	temp = temp >> PM8921_CHG_IUSB_SHIFT;

	temp = (temp << 1) | (fineres & PM8917_IUSB_FINE_RES);
	for (i = ARRAY_SIZE(usb_ma_table) - 1; i >= 0; i--) {
		if (usb_ma_table[i].value == temp)
			break;
	}

	*mA = usb_ma_table[i].usb_ma;

	return rc;
}

#if 0	//def CONFIG_LGE_PM
static int pm_chg_pbl_iusbmax_get(struct pm8921_chg_chip *chip)
{
	u8 temp;
	int rc;

	rc = pm8xxx_readb(chip->dev->parent, PBL_ACCESS2, &temp);
	if (rc) {
		pr_err("err=%d reading PBL_ACCESS2\n", rc);
		return rc;
	}
	temp &= PM8921_CHG_IUSB_MASK;
	temp = temp >> 2;

	return temp;
}
#endif

#define PM8921_CHG_WD_MASK 0x1F
static int pm_chg_disable_wd(struct pm8921_chg_chip *chip)
{
	/* writing 0 to the wd timer disables it */
	return pm_chg_masked_write(chip, CHG_TWDOG, PM8921_CHG_WD_MASK, 0);
}

#define PM8921_CHG_TCHG_MASK	0x7F
#define PM8921_CHG_TCHG_MIN	4
#define PM8921_CHG_TCHG_MAX	512
#define PM8921_CHG_TCHG_STEP	4
static int pm_chg_tchg_max_set(struct pm8921_chg_chip *chip, int minutes)
{
	u8 temp;

	ChgLog("ChgLog> %s : %d\n",__func__,minutes);

	if (minutes < PM8921_CHG_TCHG_MIN || minutes > PM8921_CHG_TCHG_MAX) {
		pr_err("bad max minutes =%d asked to set\n", minutes);
		return -EINVAL;
	}

	temp = (minutes - 1)/PM8921_CHG_TCHG_STEP;
	return pm_chg_masked_write(chip, CHG_TCHG_MAX, PM8921_CHG_TCHG_MASK,
					 temp);
}

#define PM8921_CHG_TTRKL_MASK	0x1F
#define PM8921_CHG_TTRKL_MIN	1
#define PM8921_CHG_TTRKL_MAX	64
static int pm_chg_ttrkl_max_set(struct pm8921_chg_chip *chip, int minutes)
{
	u8 temp;

	ChgLog("ChgLog> %s : %d\n",__func__,minutes);

	if (minutes < PM8921_CHG_TTRKL_MIN || minutes > PM8921_CHG_TTRKL_MAX) {
		pr_err("bad max minutes =%d asked to set\n", minutes);
		return -EINVAL;
	}

	temp = minutes - 1;
	return pm_chg_masked_write(chip, CHG_TTRKL_MAX, PM8921_CHG_TTRKL_MASK,
					 temp);
}

#define PM8921_CHG_VTRKL_MIN_MV		2050
#define PM8921_CHG_VTRKL_MAX_MV		2800
#define PM8921_CHG_VTRKL_STEP_MV	50
#define PM8921_CHG_VTRKL_SHIFT		4
#define PM8921_CHG_VTRKL_MASK		0xF0
static int pm_chg_vtrkl_low_set(struct pm8921_chg_chip *chip, int millivolts)
{
	u8 temp;

	ChgLog("ChgLog> %s : %d\n",__func__,millivolts);

	if (millivolts < PM8921_CHG_VTRKL_MIN_MV
			|| millivolts > PM8921_CHG_VTRKL_MAX_MV) {
		pr_err("bad voltage = %dmV asked to set\n", millivolts);
		return -EINVAL;
	}

	temp = (millivolts - PM8921_CHG_VTRKL_MIN_MV)/PM8921_CHG_VTRKL_STEP_MV;
	temp = temp << PM8921_CHG_VTRKL_SHIFT;
	return pm_chg_masked_write(chip, CHG_VTRICKLE, PM8921_CHG_VTRKL_MASK,
					 temp);
}

#define PM8921_CHG_VWEAK_MIN_MV		2100
#define PM8921_CHG_VWEAK_MAX_MV		3600
#define PM8921_CHG_VWEAK_STEP_MV	100
#define PM8921_CHG_VWEAK_MASK		0x0F
static int pm_chg_vweak_set(struct pm8921_chg_chip *chip, int millivolts)
{
	u8 temp;

	ChgLog("ChgLog> %s : %d\n",__func__,millivolts);

	if (millivolts < PM8921_CHG_VWEAK_MIN_MV
			|| millivolts > PM8921_CHG_VWEAK_MAX_MV) {
		pr_err("bad voltage = %dmV asked to set\n", millivolts);
		return -EINVAL;
	}

	temp = (millivolts - PM8921_CHG_VWEAK_MIN_MV)/PM8921_CHG_VWEAK_STEP_MV;
	return pm_chg_masked_write(chip, CHG_VTRICKLE, PM8921_CHG_VWEAK_MASK,
					 temp);
}

#define PM8921_CHG_ITRKL_MIN_MA		50
#define PM8921_CHG_ITRKL_MAX_MA		200
#define PM8921_CHG_ITRKL_MASK		0x0F
#define PM8921_CHG_ITRKL_STEP_MA	10
static int pm_chg_itrkl_set(struct pm8921_chg_chip *chip, int milliamps)
{
	u8 temp;

	ChgLog("ChgLog> %s : %d\n",__func__,milliamps);

	if (milliamps < PM8921_CHG_ITRKL_MIN_MA
		|| milliamps > PM8921_CHG_ITRKL_MAX_MA) {
		pr_err("bad current = %dmA asked to set\n", milliamps);
		return -EINVAL;
	}

	temp = (milliamps - PM8921_CHG_ITRKL_MIN_MA)/PM8921_CHG_ITRKL_STEP_MA;

	return pm_chg_masked_write(chip, CHG_ITRICKLE, PM8921_CHG_ITRKL_MASK,
					 temp);
}

#define PM8921_CHG_IWEAK_MIN_MA		325
#define PM8921_CHG_IWEAK_MAX_MA		525
#define PM8921_CHG_IWEAK_SHIFT		7
#define PM8921_CHG_IWEAK_MASK		0x80
static int pm_chg_iweak_set(struct pm8921_chg_chip *chip, int milliamps)
{
	u8 temp;

	ChgLog("ChgLog> %s : %d\n",__func__,milliamps);

	if (milliamps < PM8921_CHG_IWEAK_MIN_MA
		|| milliamps > PM8921_CHG_IWEAK_MAX_MA) {
		pr_err("bad current = %dmA asked to set\n", milliamps);
		return -EINVAL;
	}

	if (milliamps < PM8921_CHG_IWEAK_MAX_MA)
		temp = 0;
	else
		temp = 1;

	temp = temp << PM8921_CHG_IWEAK_SHIFT;
	return pm_chg_masked_write(chip, CHG_ITRICKLE, PM8921_CHG_IWEAK_MASK,
					 temp);
}

#define PM8921_CHG_BATT_TEMP_THR_COLD	BIT(1)
#define PM8921_CHG_BATT_TEMP_THR_COLD_SHIFT	1
static int pm_chg_batt_cold_temp_config(struct pm8921_chg_chip *chip,
					enum pm8921_chg_cold_thr cold_thr)
{
	u8 temp;

	temp = cold_thr << PM8921_CHG_BATT_TEMP_THR_COLD_SHIFT;
	temp = temp & PM8921_CHG_BATT_TEMP_THR_COLD;
	return pm_chg_masked_write(chip, CHG_CNTRL_2,
					PM8921_CHG_BATT_TEMP_THR_COLD,
					 temp);
}

#define PM8921_CHG_BATT_TEMP_THR_HOT		BIT(0)
#define PM8921_CHG_BATT_TEMP_THR_HOT_SHIFT	0
static int pm_chg_batt_hot_temp_config(struct pm8921_chg_chip *chip,
					enum pm8921_chg_hot_thr hot_thr)
{
	u8 temp;

	temp = hot_thr << PM8921_CHG_BATT_TEMP_THR_HOT_SHIFT;
	temp = temp & PM8921_CHG_BATT_TEMP_THR_HOT;
	return pm_chg_masked_write(chip, CHG_CNTRL_2,
					PM8921_CHG_BATT_TEMP_THR_HOT,
					 temp);
}

#define PM8921_CHG_LED_SRC_CONFIG_SHIFT	4
#define PM8921_CHG_LED_SRC_CONFIG_MASK	0x30
static int pm_chg_led_src_config(struct pm8921_chg_chip *chip,
				enum pm8921_chg_led_src_config led_src_config)
{
	u8 temp;

	if (led_src_config < LED_SRC_GND ||
			led_src_config > LED_SRC_BYPASS)
		return -EINVAL;

	if (led_src_config == LED_SRC_BYPASS)
		return 0;

	temp = led_src_config << PM8921_CHG_LED_SRC_CONFIG_SHIFT;

	return pm_chg_masked_write(chip, CHG_CNTRL_3,
					PM8921_CHG_LED_SRC_CONFIG_MASK, temp);
}

#ifdef CONFIG_LGE_PM
#define CHG_BATFET_ON  BIT(3)
/* TRUE : forcing CLOSE, FALSE : depend on PMIC FSM */
static int pm_chg_batfet_set(struct pm8921_chg_chip *chip, int on)
{
	int rc;

	pr_info("set CHG_BATFET_ON=%d\n", on);
	rc = pm_chg_masked_write(chip, CHG_CNTRL, CHG_BATFET_ON, on?CHG_BATFET_ON:0);
	return rc;
}
static int pm_chg_batfet_get(struct pm8921_chg_chip *chip)
{
	int rc;
	u8 temp;

	rc = pm8xxx_readb(chip->dev->parent, CHG_CNTRL, &temp);
	if (rc) {
		pr_err("pm8xxx_readb to %x value =%d errored = %d\n", CHG_CNTRL, temp, rc);
		return -EAGAIN;
	}
	pr_info("get CHG_BATFET_ON=%d\n", !!(temp & CHG_BATFET_ON));
	return !!(temp & CHG_BATFET_ON);
}
int pm8921_chg_batfet_set_ext(int on)
{
	if(the_chip==NULL) return -EAGAIN;
	return pm_chg_batfet_set(the_chip, on);
}
int pm8921_chg_batfet_get_ext(void)
{
	if(the_chip==NULL) return -EAGAIN;
	return pm_chg_batfet_get(the_chip);
}
#endif

#ifndef CONFIG_LGE_PM_ADP_CHG
/* Revert USB unplug detection to 1043 - seonghun1.kim */
static void disable_input_voltage_regulation(struct pm8921_chg_chip *chip)
{
	u8 temp;
	int rc;

	rc = pm8xxx_writeb(chip->dev->parent, CHG_BUCK_CTRL_TEST3, 0x70);
	if (rc) {
		pr_err("Failed to write 0x70 to CTRL_TEST3 rc = %d\n", rc);
		return;
	}
	rc = pm8xxx_readb(chip->dev->parent, CHG_BUCK_CTRL_TEST3, &temp);
	if (rc) {
		pr_err("Failed to read CTRL_TEST3 rc = %d\n", rc);
		return;
	}
	/* set the input voltage disable bit and the write bit */
	temp |= 0x81;
	rc = pm8xxx_writeb(chip->dev->parent, CHG_BUCK_CTRL_TEST3, temp);
	if (rc) {
		pr_err("Failed to write 0x%x to CTRL_TEST3 rc=%d\n", temp, rc);
		return;
	}
}

static void enable_input_voltage_regulation(struct pm8921_chg_chip *chip)
{
	u8 temp;
	int rc;

	rc = pm8xxx_writeb(chip->dev->parent, CHG_BUCK_CTRL_TEST3, 0x70);
	if (rc) {
		pr_err("Failed to write 0x70 to CTRL_TEST3 rc = %d\n", rc);
		return;
	}
	rc = pm8xxx_readb(chip->dev->parent, CHG_BUCK_CTRL_TEST3, &temp);
	if (rc) {
		pr_err("Failed to read CTRL_TEST3 rc = %d\n", rc);
		return;
	}
	/* unset the input voltage disable bit */
	temp &= 0xFE;
	/* set the write bit */
	temp |= 0x80;
	rc = pm8xxx_writeb(chip->dev->parent, CHG_BUCK_CTRL_TEST3, temp);
	if (rc) {
		pr_err("Failed to write 0x%x to CTRL_TEST3 rc=%d\n", temp, rc);
		return;
	}
}
#endif

#ifndef CONFIG_LGE_PM_BATTERY_ID_CHECKER
static int64_t read_battery_id(struct pm8921_chg_chip *chip)
{
	int rc;
	struct pm8xxx_adc_chan_result result;

	rc = pm8xxx_adc_read(chip->batt_id_channel, &result);
	if (rc) {
		pr_err("error reading batt id channel = %d, rc = %d\n",
					chip->vbat_channel, rc);
		return rc;
	}
	pr_debug("batt_id phy = %lld meas = 0x%llx\n", result.physical,
						result.measurement);
	return result.physical;
}
#endif

static int is_battery_valid(struct pm8921_chg_chip *chip)
{
#ifndef CONFIG_LGE_PM_BATTERY_ID_CHECKER
	int64_t rc;
#endif

#ifdef CONFIG_LGE_PM
	if(pseudo_batt_info.mode == 1)
		return 1;
	else if (is_factory_cable())
		return 1;
#endif

#ifdef CONFIG_LGE_PM_BATTERY_ID_CHECKER
	if (lge_battery_info == BATT_DS2704_N || lge_battery_info == BATT_DS2704_L ||
		lge_battery_info == BATT_ISL6296_N || lge_battery_info == BATT_ISL6296_L)
		return 1;
	else
		return 0;
#else
	if (chip->batt_id_min == 0 && chip->batt_id_max == 0)
		return 1;

	rc = read_battery_id(chip);
	if (rc < 0) {
		pr_err("error reading batt id channel = %d, rc = %lld\n",
					chip->vbat_channel, rc);
		/* assume battery id is valid when adc error happens */
		return 1;
	}

	if (rc < chip->batt_id_min || rc > chip->batt_id_max) {
		pr_err("batt_id phy =%lld is not valid\n", rc);
		return 0;
	}
	return 1;
#endif
}

static void check_battery_valid(struct pm8921_chg_chip *chip)
{
	if (is_battery_valid(chip) == 0) {
		pr_err("batt_id not valid, disbling charging\n");
		pm_chg_auto_enable(chip, 0);
	} else {
		pm_chg_auto_enable(chip, !charging_disabled);
	}
}

static void battery_id_valid(struct work_struct *work)
{
	struct pm8921_chg_chip *chip = container_of(work,
				struct pm8921_chg_chip, battery_id_valid_work);

	check_battery_valid(chip);
}

static void pm8921_chg_enable_irq(struct pm8921_chg_chip *chip, int interrupt)
{
	if (!__test_and_set_bit(interrupt, chip->enabled_irqs)) {
		dev_dbg(chip->dev, "%d\n", chip->pmic_chg_irq[interrupt]);
		enable_irq(chip->pmic_chg_irq[interrupt]);
	}
}

static void pm8921_chg_disable_irq(struct pm8921_chg_chip *chip, int interrupt)
{
	if (__test_and_clear_bit(interrupt, chip->enabled_irqs)) {
		dev_dbg(chip->dev, "%d\n", chip->pmic_chg_irq[interrupt]);
		disable_irq_nosync(chip->pmic_chg_irq[interrupt]);
	}
}

static int pm8921_chg_is_enabled(struct pm8921_chg_chip *chip, int interrupt)
{
	return test_bit(interrupt, chip->enabled_irqs);
}

static bool is_ext_charging(struct pm8921_chg_chip *chip)
{
	union power_supply_propval ret = {0,};

	if (!chip->ext_psy)
		return false;
	if (chip->ext_psy->get_property(chip->ext_psy,
					POWER_SUPPLY_PROP_CHARGE_TYPE, &ret))
		return false;
	if (ret.intval > POWER_SUPPLY_CHARGE_TYPE_NONE)
		return ret.intval;

	return false;
}

static bool is_ext_trickle_charging(struct pm8921_chg_chip *chip)
{
	union power_supply_propval ret = {0,};

	if (!chip->ext_psy)
		return false;
	if (chip->ext_psy->get_property(chip->ext_psy,
					POWER_SUPPLY_PROP_CHARGE_TYPE, &ret))
		return false;
	if (ret.intval == POWER_SUPPLY_CHARGE_TYPE_TRICKLE)
		return true;

	return false;
}

static int is_battery_charging(int fsm_state)
{
	if (is_ext_charging(the_chip))
		return 1;

	switch (fsm_state) {
	case FSM_STATE_ATC_2A:
	case FSM_STATE_ATC_2B:
	case FSM_STATE_ON_CHG_AND_BAT_6:
	case FSM_STATE_FAST_CHG_7:
	case FSM_STATE_TRKL_CHG_8:
		return 1;
	}
	return 0;
}

static void bms_notify(struct work_struct *work)
{
	struct bms_notify *n = container_of(work, struct bms_notify, work);

	if (n->is_charging) {
		pm8921_bms_charging_began();
	} else {
		pm8921_bms_charging_end(n->is_battery_full);
		n->is_battery_full = 0;
	}
}

static void bms_notify_check(struct pm8921_chg_chip *chip)
{
	int fsm_state, new_is_charging;

	fsm_state = pm_chg_get_fsm_state(chip);
	new_is_charging = is_battery_charging(fsm_state);

	if (chip->bms_notify.is_charging ^ new_is_charging) {
		chip->bms_notify.is_charging = new_is_charging;
		schedule_work(&(chip->bms_notify.work));
	}
}

#ifdef CONFIG_LGE_PM
/* submit ATnT temp scenario kwangjae1.lee */
static int get_over_temp(struct pm8921_chg_chip *chip)
{
	union power_supply_propval ret = {0,};

	chip->batt_psy.get_property(&(chip->batt_psy),
					POWER_SUPPLY_PROP_TEMP, &ret);

	return ret.intval;
}

/* Adjust SoC upper 95 percent after EOC */
int is_adjust_soc_condition(void)
{
	if (the_chip == NULL)
		return 0;
	return the_chip->bms_notify.is_charge_done_hit;
}
#endif

static enum power_supply_property pm_power_props_usb[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CURRENT_MAX,
};

static enum power_supply_property pm_power_props_mains[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
};

static char *pm_power_supplied_to[] = {
	"battery",
};
#ifdef CONFIG_LGE_PM
#define USB_WALL_THRESHOLD_MA	1500
#else
#define USB_WALL_THRESHOLD_MA	500
#endif
static int pm_power_get_property_mains(struct power_supply *psy,
				  enum power_supply_property psp,
				  union power_supply_propval *val)
{
	/* Check if called before init */
	if (!the_chip)
		return -EINVAL;

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
#ifdef CONFIG_LGE_PM
		/* make separated case, 2012.7.24, hayoung.lee@ */
		val->intval = 0;
		if ( the_chip->usb_psy.type == POWER_SUPPLY_TYPE_MAINS ||
			 the_chip->usb_psy.type == POWER_SUPPLY_TYPE_USB_DCP )
			val->intval = is_usb_chg_plugged_in(the_chip);
		return 0;
#endif

	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = 0;
		if (charging_disabled)
			return 0;

		/* check external charger first before the dc path */
		if (is_ext_charging(the_chip)) {
			val->intval = 1;
			return 0;
		}
#ifdef CONFIG_LGE_PM
		/* submit ATnT temp scenario kwangjae1.lee */
		if (pseudo_ui_charging) {
			if((get_over_temp(the_chip) > (BATT_TEMP_LEVEL_POWER_OFF * 10))
				&& (the_chip->usb_psy.type != POWER_SUPPLY_TYPE_BATTERY))
				val->intval = 1;
			else
				val->intval = 0;
			return 0;
		}
#endif
		if (pm_is_chg_charge_dis(the_chip)) {
			val->intval = 0;
			return 0;
		}

		if (the_chip->dc_present) {
			val->intval = 1;
			return 0;
		}
#ifdef CONFIG_LGE_PM
		/* Decision AC(TA) charger */
		if ( the_chip->usb_psy.type == POWER_SUPPLY_TYPE_MAINS ||
		     the_chip->usb_psy.type == POWER_SUPPLY_TYPE_USB_DCP )
#else
		/* USB with max current greater than 500 mA connected */
		if (usb_target_ma > USB_WALL_THRESHOLD_MA)
#endif
			val->intval = is_usb_chg_plugged_in(the_chip);
			return 0;

		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int pm_power_get_property_usb(struct power_supply *psy,
				  enum power_supply_property psp,
				  union power_supply_propval *val)
{
	int current_max;

	/* Check if called before init */
	if (!the_chip)
		return -EINVAL;

	switch (psp) {
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		if (pm_is_chg_charge_dis(the_chip)) {
			val->intval = 0;
		} else {
			pm_chg_iusbmax_get(the_chip, &current_max);
			val->intval = current_max;
		}
		break;
	case POWER_SUPPLY_PROP_PRESENT:
#ifdef CONFIG_LGE_PM
		/* make separated case, 2012.7.24, hayoung.lee@ */
		val->intval = 0;
		if ( the_chip->usb_psy.type == POWER_SUPPLY_TYPE_USB ||
			 the_chip->usb_psy.type == POWER_SUPPLY_TYPE_USB_CDP ||
			 the_chip->usb_psy.type == POWER_SUPPLY_TYPE_USB_ACA )
			val->intval = is_usb_chg_plugged_in(the_chip);
		return 0;
#endif
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = 0;
		if (charging_disabled)
			return 0;
#ifdef CONFIG_LGE_PM
		/* submit ATnT temp scenario kwangjae1.lee */
		if (pseudo_ui_charging) {
			if((get_over_temp(the_chip) > (BATT_TEMP_LEVEL_POWER_OFF * 10))
				&& (the_chip->usb_psy.type != POWER_SUPPLY_TYPE_BATTERY))
				val->intval = 1;
			else
				val->intval = 0;
			return 0;
		}
#endif
		/*
		 * if drawing any current from usb is disabled behave
		 * as if no usb cable is connected
		 */
		if (pm_is_chg_charge_dis(the_chip))
			return 0;

#ifdef CONFIG_LGE_PM
		/* Decision USB charger */
		if ( the_chip->usb_psy.type == POWER_SUPPLY_TYPE_USB ||
		     the_chip->usb_psy.type == POWER_SUPPLY_TYPE_USB_CDP ||
		     the_chip->usb_psy.type == POWER_SUPPLY_TYPE_USB_ACA )
#else
		/* USB charging */
		if (usb_target_ma < USB_WALL_THRESHOLD_MA)
#endif
			val->intval = is_usb_chg_plugged_in(the_chip);
		else
		    return 0;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static enum power_supply_property msm_batt_power_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_ENERGY_FULL,
#ifdef CONFIG_LGE_PM
	POWER_SUPPLY_PROP_PSEUDO_BATT,
	POWER_SUPPLY_PROP_BLOCK_CHARGING,
	POWER_SUPPLY_PROP_EXT_PWR_CHECK,
	/* kwangjae1.lee@lge.com 2012-06-11 Add bms debugger */
	POWER_SUPPLY_PROP_BMS_BATT,
	/* Add battery present check in the testmode */
	POWER_SUPPLY_PROP_REAL_BATT_PRESENT,
#endif
#ifdef CONFIG_LGE_PM_BATTERY_ID_CHECKER
	POWER_SUPPLY_PROP_BATTERY_ID_CHECK,
#endif
};

static int get_prop_battery_uvolts(struct pm8921_chg_chip *chip)
{
	int rc;
	struct pm8xxx_adc_chan_result result;

	rc = pm8xxx_adc_read(chip->vbat_channel, &result);
	if (rc) {
		pr_err("error reading adc channel = %d, rc = %d\n",
					chip->vbat_channel, rc);
		return rc;
	}

#ifdef CONFIG_LGE_PM
	if (pseudo_batt_info.mode == 1) {
		pr_debug("pseudo_batt_mode=enable fake_volt= 3990 real mvolts phy = %lld real meas = 0x%llx\n",
			result.physical, result.measurement);
		return 3990*1000;
	}
	else if (is_real_battery_or_factory_cable(chip)) {
		pr_debug("is_real_battery_or_factory_cable=enable fake_volt=3990 real mvolts phy = %lld real meas = 0x%llx\n", 
			 result.physical, result.measurement);
		return 3990*1000;
	}
	else {
		pr_debug("mvolts phy = %lld meas = 0x%llx\n", result.physical,
						result.measurement);
		return (int)result.physical;
	}
#else
	pr_debug("mvolts phy = %lld meas = 0x%llx\n", result.physical,
					result.measurement);
	return (int)result.physical;
#endif
}

static unsigned int voltage_based_capacity(struct pm8921_chg_chip *chip)
{
	unsigned int current_voltage_uv = get_prop_battery_uvolts(chip);
	unsigned int current_voltage_mv = current_voltage_uv / 1000;
	unsigned int low_voltage = chip->min_voltage_mv;
	unsigned int high_voltage = chip->max_voltage_mv;

	if (current_voltage_mv <= low_voltage)
		return 0;
	else if (current_voltage_mv >= high_voltage)
		return 100;
	else
		return (current_voltage_mv - low_voltage) * 100
		    / (high_voltage - low_voltage);
}

static int get_prop_batt_present(struct pm8921_chg_chip *chip)
{
#ifdef CONFIG_LGE_PM
	if(pseudo_batt_info.mode == 1)
		return 1;
	else if (is_factory_cable())
		return 1;
#endif

#ifdef CONFIG_LGE_PM_BATTERY_ID_CHECKER
	if ((lge_battery_info == BATT_DS2704_N || lge_battery_info == BATT_DS2704_L ||
			lge_battery_info == BATT_ISL6296_N || lge_battery_info == BATT_ISL6296_L) &&
				pm_chg_get_rt_status(chip, BATT_INSERTED_IRQ))
		return 1;
	else
		return 0;
#else
        return pm_chg_get_rt_status(chip, BATT_INSERTED_IRQ);
#endif
}

static int get_prop_batt_capacity(struct pm8921_chg_chip *chip)
{
	int percent_soc;

	if (!get_prop_batt_present(chip))
		percent_soc = voltage_based_capacity(chip);
	else
		percent_soc = pm8921_bms_get_percent_charge();

	if (percent_soc == -ENXIO)
		percent_soc = voltage_based_capacity(chip);

	if (percent_soc <= 10)
		pr_warn("low battery charge = %d%%\n", percent_soc);

#ifdef CONFIG_LGE_PM
	if (pseudo_batt_info.mode == 1) {
		pr_debug("pseudo_batt_mode=enable percent_soc = 75, real_percent_soc = %d\n",percent_soc);
		if( percent_soc > 1 )/* Order from MST test */
			percent_soc = 75;
	}
	else if (is_real_battery_or_factory_cable(chip)) {
		pr_debug("is_real_battery_or_factory_cable=enable percent_soc = 75, real_percent_soc = %d\n",percent_soc);
		percent_soc = 75;
	}
#endif

	chip->recent_reported_soc = percent_soc;
	return percent_soc;
}

static int get_prop_batt_current(struct pm8921_chg_chip *chip)
{
	int result_ua, rc;

	rc = pm8921_bms_get_battery_current(&result_ua);
	if (rc == -ENXIO) {
		rc = pm8xxx_ccadc_get_battery_current(&result_ua);
	}

	if (rc) {
		pr_err("unable to get batt current rc = %d\n", rc);
		return rc;
	} else {
		return result_ua;
	}
}

static int get_prop_batt_fcc(struct pm8921_chg_chip *chip)
{
	int rc;

	rc = pm8921_bms_get_fcc();
	if (rc < 0)
		pr_err("unable to get batt fcc rc = %d\n", rc);
	return rc;
}

static int get_prop_batt_health(struct pm8921_chg_chip *chip)
{
	int temp;

#ifdef CONFIG_LGE_PM
	/* LGE does not PMIC temp IRQ, so directly read batt-ADC. */
	temp = get_prop_batt_temp(chip);
	if (temp > BATT_TEMP_LEVEL_OVERHEAT*10)
		return POWER_SUPPLY_HEALTH_OVERHEAT;
	else if (temp < BATT_TEMP_LEVEL_COLD*10)
		return POWER_SUPPLY_HEALTH_COLD;
#else
	temp = pm_chg_get_rt_status(chip, BATTTEMP_HOT_IRQ);
	if (temp)
		return POWER_SUPPLY_HEALTH_OVERHEAT;

	temp = pm_chg_get_rt_status(chip, BATTTEMP_COLD_IRQ);
	if (temp)
		return POWER_SUPPLY_HEALTH_COLD;
#endif

	return POWER_SUPPLY_HEALTH_GOOD;
}

static int get_prop_charge_type(struct pm8921_chg_chip *chip)
{
	int temp;

	if (!get_prop_batt_present(chip))
		return POWER_SUPPLY_CHARGE_TYPE_NONE;

	if (is_ext_trickle_charging(chip))
		return POWER_SUPPLY_CHARGE_TYPE_TRICKLE;

	if (is_ext_charging(chip))
		return POWER_SUPPLY_CHARGE_TYPE_FAST;

	temp = pm_chg_get_rt_status(chip, TRKLCHG_IRQ);
	if (temp)
		return POWER_SUPPLY_CHARGE_TYPE_TRICKLE;

	temp = pm_chg_get_rt_status(chip, FASTCHG_IRQ);
	if (temp)
		return POWER_SUPPLY_CHARGE_TYPE_FAST;

	return POWER_SUPPLY_CHARGE_TYPE_NONE;
}

static int get_prop_batt_status(struct pm8921_chg_chip *chip)
{
	int batt_state = POWER_SUPPLY_STATUS_DISCHARGING;
	int fsm_state = pm_chg_get_fsm_state(chip);
	int i;
#ifdef CONFIG_LGE_PM
	/* jooyeong.lee@lge.com 2012-01-19 Change battery status sequence when start charging */
	int batt_capacity;
#endif

	if (chip->ext_psy) {
		if (chip->ext_charge_done)
			return POWER_SUPPLY_STATUS_FULL;
		if (chip->ext_charging)
			return POWER_SUPPLY_STATUS_CHARGING;
	}

	for (i = 0; i < ARRAY_SIZE(map); i++)
		if (map[i].fsm_state == fsm_state)
			batt_state = map[i].batt_state;

#ifdef CONFIG_LGE_PM
	/* jooyeong.lee@lge.com 2012-01-19 Change battery status sequence when start charging */
	if (batt_state == POWER_SUPPLY_STATUS_FULL) {
		batt_capacity = get_prop_batt_capacity(chip);

		if (pm_chg_get_rt_status(chip, BATT_INSERTED_IRQ)
			&& is_battery_valid(chip)
			&& pm_chg_get_rt_status(chip, BAT_TEMP_OK_IRQ)
			&& batt_capacity >= 100 ) {
			return batt_state;
		}

		if (pm_chg_get_rt_status(chip, BATT_INSERTED_IRQ)
			&& is_battery_valid(chip)
			&& pm_chg_get_rt_status(chip, BAT_TEMP_OK_IRQ)
			&& batt_capacity >= 0 && batt_capacity < 100) {
			batt_state = POWER_SUPPLY_STATUS_CHARGING;
			return batt_state;
		}
	}
#endif

	if (fsm_state == FSM_STATE_ON_CHG_HIGHI_1) {
#ifdef CONFIG_LGE_CHARGER_TEMP_SCENARIO
		if (!pm_chg_get_rt_status(chip, BATT_INSERTED_IRQ)
			|| !pm_chg_get_rt_status(chip, BAT_TEMP_OK_IRQ)
			|| !pm_chg_get_rt_status(chip, CHGHOT_IRQ)
			|| ((pseudo_ui_charging == 0)
			    && (chg_batt_temp_state == CHG_BATT_STOP_CHARGING_STATE)))

			batt_state = POWER_SUPPLY_STATUS_NOT_CHARGING;
#else
		if (!pm_chg_get_rt_status(chip, BATT_INSERTED_IRQ)
			|| !pm_chg_get_rt_status(chip, BAT_TEMP_OK_IRQ)
			|| pm_chg_get_rt_status(chip, CHGHOT_IRQ)
			|| pm_chg_get_rt_status(chip, VBATDET_LOW_IRQ))

			batt_state = POWER_SUPPLY_STATUS_NOT_CHARGING;
#endif
	}
	return batt_state;
}

#define MAX_TOLERABLE_BATT_TEMP_DDC	680
static int get_prop_batt_temp(struct pm8921_chg_chip *chip)
{
	int rc;
	struct pm8xxx_adc_chan_result result;

	rc = pm8xxx_adc_read(chip->batt_temp_channel, &result);
	if (rc) {
		pr_err("error reading adc channel = %d, rc = %d\n",
					chip->vbat_channel, rc);
		return rc;
	}
	pr_debug("batt_temp phy = %lld meas = 0x%llx\n", result.physical,
						result.measurement);
#ifdef CONFIG_LGE_PM
	/* add for thermister test kwangjae1.lee@lge.com */
	if (pseudo_batt_info.mode)
		rc = (pseudo_batt_info.temp * 10);
	else
		rc = (int)result.physical - LGE_temp_OFFSET;

	pr_debug("batt_temp phy = %lld - offset(%d), result = %d\n",
						result.physical, LGE_temp_OFFSET ,rc);

	return rc;
#else
	if (result.physical > MAX_TOLERABLE_BATT_TEMP_DDC)
		pr_err("BATT_TEMP= %d > 68degC, device will be shutdown\n",
							(int) result.physical);

	return (int)result.physical;
#endif
}


/* [LGE_UPDATE_S for high temperature scaling] */
static int old_batt_temperature = -9999;
static int scaling_count = 0;
/* [LGE_UPDATE_E for high temperature scaling] */

static int pm_batt_power_get_property(struct power_supply *psy,
				       enum power_supply_property psp,
				       union power_supply_propval *val)
{
    int now_batt_temperature = 0;  /* [LGE_UPDATE for high temperature scaling] */

	struct pm8921_chg_chip *chip = container_of(psy, struct pm8921_chg_chip,
								batt_psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
#ifdef CONFIG_LGE_PM
		if(block_charging_state == 0)
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		else
			val->intval = get_prop_batt_status(chip);
#else
		val->intval = get_prop_batt_status(chip);
#endif
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		val->intval = get_prop_charge_type(chip);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = get_prop_batt_health(chip);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = get_prop_batt_present(chip);
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = chip->max_voltage_mv * 1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		val->intval = chip->min_voltage_mv * 1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:

		val->intval = get_prop_battery_uvolts(chip) / 1000;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = get_prop_batt_capacity(chip);
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = get_prop_batt_current(chip);
		break;
	case POWER_SUPPLY_PROP_TEMP:
#ifdef CONFIG_LGE_PM
		if (is_real_battery_or_factory_cable(chip))
			val->intval = 25 * 10;
		else
		{
/* [LGE_UPDATE_S for high temperature scaling] */
#if 1
          now_batt_temperature = get_prop_batt_temp(chip);

          if(old_batt_temperature == -9999)
          {
			val->intval = now_batt_temperature;
			scaling_count = 0;
          }
		  else
		  {
		    if(now_batt_temperature < 550)
		    {
		  	  val->intval = now_batt_temperature;
			  scaling_count = 0;
		    }
		    else
		    {
		      if(scaling_count == 0)
		      {
		        val->intval = old_batt_temperature;
			    scaling_count++;
		      }
			  else
			  {
			    val->intval = now_batt_temperature;
			    scaling_count = 0;
			  }
		    }
		  }
		  old_batt_temperature = now_batt_temperature;
		}
#else
		val->intval = get_prop_batt_temp(chip);
#endif
/* [LGE_UPDATE_E for high temperature scaling] */
#else
		val->intval = get_prop_batt_temp(chip);
#endif
		break;
	case POWER_SUPPLY_PROP_ENERGY_FULL:
		val->intval = get_prop_batt_fcc(chip) * 1000;
		break;
#ifdef CONFIG_LGE_PM
	case POWER_SUPPLY_PROP_PSEUDO_BATT:
		val->intval = pseudo_batt_info.mode;
		break;
	case POWER_SUPPLY_PROP_BLOCK_CHARGING:
		val->intval = block_charging_state;
		break;
	case POWER_SUPPLY_PROP_EXT_PWR_CHECK:
		val->intval = lge_pm_get_cable_type();
		break;
	/* kwangjae1.lee@lge.com 2012-06-11 Add bms debugger */
	case POWER_SUPPLY_PROP_BMS_BATT:
		val->intval = bms_batt_info.mode;
		break;
	/* 2012-07-11 Add battery present check in the testmode */
	case POWER_SUPPLY_PROP_REAL_BATT_PRESENT:
		val->intval = pm_chg_get_rt_status(chip, BATT_INSERTED_IRQ);
		break;
#endif
#ifdef CONFIG_LGE_PM_BATTERY_ID_CHECKER
	case POWER_SUPPLY_PROP_BATTERY_ID_CHECK:
		val->intval = is_battery_valid(chip);
		break;
#endif
	default:
		return -EINVAL;
	}

	return 0;
}

/* [START] sungsookim */
#ifdef CONFIG_LGE_PM
/* LGE_S kwangjae1.lee@lge.com 2012-06-11 Add bms debugger */
int bms_batt_set(struct bms_batt_info_type* bms_info)
{
	bms_batt_info.mode = bms_info->mode;
	pr_debug("%s value =%d\n",__func__,bms_batt_info.mode);
	return 0;
}
EXPORT_SYMBOL(bms_batt_set);
/* LGE_E kwangjae1.lee@lge.com 2012-06-11 Add bms debugger */
#define PSEUDO_BATT_MAX 800
int pseudo_batt_set(struct pseudo_batt_info_type* info)
{
	struct pm8921_chg_chip *chip = the_chip;

	pseudo_batt_info.mode = info->mode;
	pseudo_batt_info.id = info->id;
	pseudo_batt_info.therm = info->therm;
	pseudo_batt_info.temp = info->temp;
	pseudo_batt_info.volt = info->volt;
	pseudo_batt_info.capacity = info->capacity;
	pseudo_batt_info.charging = info->charging;

	power_supply_changed(&chip->batt_psy);

	/* If pseudo batt enabled, Set iusb max to PSEUDO_BATT_MAX(800mA) - seonghun1.kim */
	if ( pseudo_batt_info.mode == 1 ) {
		if ( lge_pm_get_usb_current() <= PSEUDO_BATT_MAX )
			pm8921_charger_vbus_draw(PSEUDO_BATT_MAX);
	}
	else {
		if ( lge_pm_get_usb_current() <= PSEUDO_BATT_MAX )
			pm8921_charger_vbus_draw(lge_pm_get_usb_current());
	}

	return 0;
}
EXPORT_SYMBOL(pseudo_batt_set);

void block_charging_set(int block)
{
	if(block)
	{
		//pm_chg_auto_disable(0);
		//pm_chg_charge_dis(the_chip, 0);
		pm8921_charger_enable(true);
	}
	else
	{
		//pm_chg_auto_disable(1);
		//pm_chg_auto_enable(the_chip, 1);
		pm8921_charger_enable(false);
	}
}
void batt_block_charging_set(int block)
{
	struct pm8921_chg_chip *chip = the_chip;

	block_charging_state = block;
	block_charging_set(block);

	power_supply_changed(&chip->batt_psy);
}
EXPORT_SYMBOL(batt_block_charging_set);
#endif
/* [END] */

static void (*notify_vbus_state_func_ptr)(int);
static int usb_chg_current;
static DEFINE_SPINLOCK(vbus_lock);

int pm8921_charger_register_vbus_sn(void (*callback)(int))
{
	pr_debug("%p\n", callback);
	notify_vbus_state_func_ptr = callback;
	return 0;
}
EXPORT_SYMBOL_GPL(pm8921_charger_register_vbus_sn);

/* this is passed to the hsusb via platform_data msm_otg_pdata */
void pm8921_charger_unregister_vbus_sn(void (*callback)(int))
{
	pr_debug("%p\n", callback);
	notify_vbus_state_func_ptr = NULL;
}
EXPORT_SYMBOL_GPL(pm8921_charger_unregister_vbus_sn);

static void notify_usb_of_the_plugin_event(int plugin)
{
	plugin = !!plugin;
	if (notify_vbus_state_func_ptr) {
		pr_debug("notifying plugin\n");
		(*notify_vbus_state_func_ptr) (plugin);
	} else {
		pr_debug("unable to notify plugin\n");
	}
}

/* assumes vbus_lock is held */
static void __pm8921_charger_vbus_draw(unsigned int mA)
{
	int i, rc;
	if (!the_chip) {
		pr_err("called before init\n");
		return;
	}

#ifdef CONFIG_LGE_PM
	/* If pseudo batt enabled, Set iusb max to PSEUDO_BATT_MAX(800mA) - seonghun1.kim */
	if ((pseudo_batt_info.mode == 1) && (mA <= PSEUDO_BATT_MAX))
		mA = PSEUDO_BATT_MAX;
#endif

	if (mA >= 0 && mA <= 2) {
		usb_chg_current = 0;
		rc = pm_chg_iusbmax_set(the_chip, 0);
		if (rc) {
			pr_err("unable to set iusb to %d rc = %d\n", 0, rc);
		}
		rc = pm_chg_usb_suspend_enable(the_chip, 1);
		if (rc)
			pr_err("fail to set suspend bit rc=%d\n", rc);
	} else {
		rc = pm_chg_usb_suspend_enable(the_chip, 0);
		if (rc)
			pr_err("fail to reset suspend bit rc=%d\n", rc);
		for (i = ARRAY_SIZE(usb_ma_table) - 1; i >= 0; i--) {
			if (usb_ma_table[i].usb_ma <= mA)
				break;
		}

		/* Check if IUSB_FINE_RES is available */
		if ((usb_ma_table[i].value & PM8917_IUSB_FINE_RES)
				&& !the_chip->iusb_fine_res)
			i--;
		if (i < 0)
			i = 0;
		rc = pm_chg_iusbmax_set(the_chip, i);
		if (rc) {
			pr_err("unable to set iusb to %d rc = %d\n", i, rc);
		}
	}
}

/* USB calls these to tell us how much max usb current the system can draw */
void pm8921_charger_vbus_draw(unsigned int mA)
{
	unsigned long flags;

	pr_debug("Enter charge=%d\n", mA);

	if (!the_chip) {
		pr_err("chip not yet initalized\n");
		return;
	}

	/*
	 * Reject VBUS requests if USB connection is the only available
	 * power source. This makes sure that if booting without
	 * battery the iusb_max value is not decreased avoiding potential
	 * brown_outs.
	 *
	 * This would also apply when the battery has been
	 * removed from the running system.
	 */
	if (!get_prop_batt_present(the_chip)
		&& !is_dc_chg_plugged_in(the_chip)) {
		pr_err("rejected: no other power source connected\n");
		return;
	}

	if (usb_max_current && mA > usb_max_current) {
		pr_warn("restricting usb current to %d instead of %d\n",
					usb_max_current, mA);
		mA = usb_max_current;
	}
	if (usb_target_ma == 0 && mA > USB_WALL_THRESHOLD_MA)
		usb_target_ma = mA;

	spin_lock_irqsave(&vbus_lock, flags);
	if (the_chip) {
		if (mA > USB_WALL_THRESHOLD_MA)
			__pm8921_charger_vbus_draw(USB_WALL_THRESHOLD_MA);
		else
			__pm8921_charger_vbus_draw(mA);
	} else {
		/*
		 * called before pmic initialized,
		 * save this value and use it at probe
		 */
		if (mA > USB_WALL_THRESHOLD_MA)
			usb_chg_current = USB_WALL_THRESHOLD_MA;
		else
			usb_chg_current = mA;
	}
#ifdef CONFIG_LGE_CHARGER_TEMP_SCENARIO
	last_usb_chg_current = mA;
#endif
	spin_unlock_irqrestore(&vbus_lock, flags);
}
EXPORT_SYMBOL_GPL(pm8921_charger_vbus_draw);

int pm8921_charger_enable(bool enable)
{
	int rc;

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	enable = !!enable;
	rc = pm_chg_auto_enable(the_chip, enable);
	if (rc)
		pr_err("Failed rc=%d\n", rc);
	return rc;
}
EXPORT_SYMBOL(pm8921_charger_enable);

int pm8921_is_usb_chg_plugged_in(void)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	return is_usb_chg_plugged_in(the_chip);
}
EXPORT_SYMBOL(pm8921_is_usb_chg_plugged_in);

int pm8921_is_dc_chg_plugged_in(void)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	return is_dc_chg_plugged_in(the_chip);
}
EXPORT_SYMBOL(pm8921_is_dc_chg_plugged_in);

int pm8921_is_battery_present(void)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	return get_prop_batt_present(the_chip);
}
EXPORT_SYMBOL(pm8921_is_battery_present);

int pm8921_is_batfet_closed(void)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	return is_batfet_closed(the_chip);
}
EXPORT_SYMBOL(pm8921_is_batfet_closed);

#ifdef CONFIG_LGE_PM
/* jungwoo.yun@lge.com 2012-08-07 check battery preset regardless of factory cable*/
int pm8921_is_real_battery_present(void)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	return pm_chg_get_rt_status(the_chip, BATT_INSERTED_IRQ);
}
EXPORT_SYMBOL(pm8921_is_real_battery_present);
#endif

/*
 * Disabling the charge current limit causes current
 * current limits to have no monitoring. An adequate charger
 * capable of supplying high current while sustaining VIN_MIN
 * is required if the limiting is disabled.
 */
int pm8921_disable_input_current_limit(bool disable)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	if (disable) {
		pr_warn("Disabling input current limit!\n");

		return pm8xxx_writeb(the_chip->dev->parent,
			 CHG_BUCK_CTRL_TEST3, 0xF2);
	}
	return 0;
}
EXPORT_SYMBOL(pm8921_disable_input_current_limit);

int pm8921_set_max_battery_charge_current(int ma)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	return pm_chg_ibatmax_set(the_chip, ma);
}
EXPORT_SYMBOL(pm8921_set_max_battery_charge_current);

int pm8921_disable_source_current(bool disable)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	if (disable)
		pr_warn("current drawn from chg=0, battery provides current\n");
	return pm_chg_charge_dis(the_chip, disable);
}
EXPORT_SYMBOL(pm8921_disable_source_current);

int pm8921_regulate_input_voltage(int voltage)
{
	int rc;

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	rc = pm_chg_vinmin_set(the_chip, voltage);

	if (rc == 0)
		the_chip->vin_min = voltage;

	return rc;
}

#define USB_OV_THRESHOLD_MASK  0x60
#define USB_OV_THRESHOLD_SHIFT  5
int pm8921_usb_ovp_set_threshold(enum pm8921_usb_ov_threshold ov)
{
	u8 temp;

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	if (ov > PM_USB_OV_7V) {
		pr_err("limiting to over voltage threshold to 7volts\n");
		ov = PM_USB_OV_7V;
	}

	temp = USB_OV_THRESHOLD_MASK & (ov << USB_OV_THRESHOLD_SHIFT);

	return pm_chg_masked_write(the_chip, USB_OVP_CONTROL,
				USB_OV_THRESHOLD_MASK, temp);
}
EXPORT_SYMBOL(pm8921_usb_ovp_set_threshold);

#define USB_DEBOUNCE_TIME_MASK	0x06
#define USB_DEBOUNCE_TIME_SHIFT 1
int pm8921_usb_ovp_set_hystersis(enum pm8921_usb_debounce_time ms)
{
	u8 temp;

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	if (ms > PM_USB_DEBOUNCE_80P5MS) {
		pr_err("limiting debounce to 80.5ms\n");
		ms = PM_USB_DEBOUNCE_80P5MS;
	}

	temp = USB_DEBOUNCE_TIME_MASK & (ms << USB_DEBOUNCE_TIME_SHIFT);

	return pm_chg_masked_write(the_chip, USB_OVP_CONTROL,
				USB_DEBOUNCE_TIME_MASK, temp);
}
EXPORT_SYMBOL(pm8921_usb_ovp_set_hystersis);

#define USB_OVP_DISABLE_MASK	0x80
int pm8921_usb_ovp_disable(int disable)
{
	u8 temp = 0;

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	if (disable)
		temp = USB_OVP_DISABLE_MASK;

	return pm_chg_masked_write(the_chip, USB_OVP_CONTROL,
				USB_OVP_DISABLE_MASK, temp);
}

bool pm8921_is_battery_charging(int *source)
{
	int fsm_state, is_charging, dc_present, usb_present;

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	fsm_state = pm_chg_get_fsm_state(the_chip);
	is_charging = is_battery_charging(fsm_state);
	if (is_charging == 0) {
		*source = PM8921_CHG_SRC_NONE;
		return is_charging;
	}

	if (source == NULL)
		return is_charging;

	/* the battery is charging, the source is requested, find it */
	dc_present = is_dc_chg_plugged_in(the_chip);
	usb_present = is_usb_chg_plugged_in(the_chip);

	if (dc_present && !usb_present)
		*source = PM8921_CHG_SRC_DC;

	if (usb_present && !dc_present)
		*source = PM8921_CHG_SRC_USB;

	if (usb_present && dc_present)
		/*
		 * The system always chooses dc for charging since it has
		 * higher priority.
		 */
		*source = PM8921_CHG_SRC_DC;

	return is_charging;
}
EXPORT_SYMBOL(pm8921_is_battery_charging);

int pm8921_set_usb_power_supply_type(enum power_supply_type type)
{
	ChgLog("ChgLog> %s : %d\n", __func__,type);
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

#ifdef CONFIG_LGE_PM
	/* Wake lock for delever Uevnet before suspend */
	wake_lock_timeout(&the_chip->deliver_uevent_wake_lock, round_jiffies_relative(msecs_to_jiffies(1000)));

	power_supply_changed(&the_chip->batt_psy);
#endif
	if (type < POWER_SUPPLY_TYPE_USB)
		return 0; // -EINVAL;		 //Prevent otg error log

	the_chip->usb_psy.type = type;
	power_supply_changed(&the_chip->usb_psy);
	power_supply_changed(&the_chip->dc_psy);
#ifdef CONFIG_LGE_PM_ADP_CHG
	/* Adapive USB draw current limit */
	if (type == POWER_SUPPLY_TYPE_USB_DCP)
		search_iusb_max_status = IUSB_MAX_INCREASE;
	else
		search_iusb_max_status = IUSB_MAX_NONE;
#endif
	return 0;
}
EXPORT_SYMBOL_GPL(pm8921_set_usb_power_supply_type);

int pm8921_batt_temperature(void)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	return get_prop_batt_temp(the_chip);
}

static void handle_usb_insertion_removal(struct pm8921_chg_chip *chip)
{
	int usb_present;

	pm_chg_failed_clear(chip, 1);
	usb_present = is_usb_chg_plugged_in(chip);
	if (chip->usb_present ^ usb_present) {
		notify_usb_of_the_plugin_event(usb_present);
		chip->usb_present = usb_present;
#ifdef CONFIG_LGE_PM
		/*
		 * Do not report power event here.
		 * Power event should reported after USB enumeration. - In pm8921_set_usb_power_supply_type()
		 * Because we don't know USB or TA before finished enumeration.
		 */
		the_chip->usb_psy.type = 0;
#else
		power_supply_changed(&chip->usb_psy);
		power_supply_changed(&chip->batt_psy);
#endif
		pm8921_bms_calibrate_hkadc();
	}
#ifdef CONFIG_LGE_CHARGER_TEMP_SCENARIO
	if (usb_present) {
		pseudo_ui_charging = 0;
		last_usb_chg_current = 0;
		/* 120712 cs.kim@lge.com Implements Thermal Mitigation iUSB setting */
		pre_mitigation = 0;
	}
#endif

#ifdef CONFIG_LGE_PM_ADP_CHG
	if (usb_present) {
		schedule_delayed_work(&chip->unplug_check_work,
			round_jiffies_relative(msecs_to_jiffies
				(UNPLUG_CHECK_WAIT_PERIOD_MS)));
	}
#else
	/* Revert USB unplug detection to 1043 - seonghun1.kim */
	if (usb_present) {
		schedule_delayed_work(&chip->unplug_check_work,
			round_jiffies_relative(msecs_to_jiffies
				(UNPLUG_CHECK_WAIT_PERIOD_MS)));
		pm8921_chg_enable_irq(chip, CHG_GONE_IRQ);
	} else {
		/* USB unplugged reset target current */
		usb_target_ma = 0;
		pm8921_chg_disable_irq(chip, CHG_GONE_IRQ);
	}
	enable_input_voltage_regulation(chip);
#endif

#ifdef CONFIG_LGE_PM
	chip->bms_notify.is_charge_done_hit = 0;
#endif
	bms_notify_check(chip);
}

static void handle_stop_ext_chg(struct pm8921_chg_chip *chip)
{
	if (!chip->ext_psy) {
		pr_debug("external charger not registered.\n");
		return;
	}

	if (!chip->ext_charging) {
		pr_debug("already not charging.\n");
		return;
	}

	power_supply_set_charge_type(chip->ext_psy,
					POWER_SUPPLY_CHARGE_TYPE_NONE);
	pm8921_disable_source_current(false); /* release BATFET */
	power_supply_changed(&chip->dc_psy);
	chip->ext_charging = false;
	chip->ext_charge_done = false;
	bms_notify_check(chip);
	/* Update battery charging LEDs and user space battery info */
	power_supply_changed(&chip->batt_psy);
}

static void handle_start_ext_chg(struct pm8921_chg_chip *chip)
{
	int dc_present;
	int batt_present;
	int batt_temp_ok;
	int vbat_ov;
	unsigned long delay =
		round_jiffies_relative(msecs_to_jiffies(EOC_CHECK_PERIOD_MS));

	if (!chip->ext_psy) {
		pr_debug("external charger not registered.\n");
		return;
	}

	if (chip->ext_charging) {
		pr_debug("already charging.\n");
		return;
	}

	dc_present = is_dc_chg_plugged_in(the_chip);
	batt_present = pm_chg_get_rt_status(chip, BATT_INSERTED_IRQ);
	batt_temp_ok = pm_chg_get_rt_status(chip, BAT_TEMP_OK_IRQ);

	if (!dc_present) {
		pr_warn("%s. dc not present.\n", __func__);
		return;
	}
	if (!batt_present) {
		pr_warn("%s. battery not present.\n", __func__);
		return;
	}
	if (!batt_temp_ok) {
		pr_warn("%s. battery temperature not ok.\n", __func__);
		return;
	}
	pm8921_disable_source_current(true); /* Force BATFET=ON */
	vbat_ov = pm_chg_get_rt_status(chip, VBAT_OV_IRQ);
	if (vbat_ov) {
		pr_warn("%s. battery over voltage.\n", __func__);
		return;
	}

	power_supply_set_online(chip->ext_psy, dc_present);
	power_supply_set_charge_type(chip->ext_psy,
					POWER_SUPPLY_CHARGE_TYPE_FAST);
	power_supply_changed(&chip->dc_psy);
	chip->ext_charging = true;
	chip->ext_charge_done = false;
	bms_notify_check(chip);
	/* Start BMS */
	schedule_delayed_work(&chip->eoc_work, delay);
	wake_lock(&chip->eoc_wake_lock);
	/* Update battery charging LEDs and user space battery info */
	power_supply_changed(&chip->batt_psy);
}

static void turn_off_usb_ovp_fet(struct pm8921_chg_chip *chip)
{
	u8 temp;
	int rc;

	rc = pm8xxx_writeb(chip->dev->parent, USB_OVP_TEST, 0x30);
	if (rc) {
		pr_err("Failed to write 0x30 to USB_OVP_TEST rc = %d\n", rc);
		return;
	}
	rc = pm8xxx_readb(chip->dev->parent, USB_OVP_TEST, &temp);
	if (rc) {
		pr_err("Failed to read from USB_OVP_TEST rc = %d\n", rc);
		return;
	}
	/* set ovp fet disable bit and the write bit */
	temp |= 0x81;
	rc = pm8xxx_writeb(chip->dev->parent, USB_OVP_TEST, temp);
	if (rc) {
		pr_err("Failed to write 0x%x USB_OVP_TEST rc=%d\n", temp, rc);
		return;
	}
}

static void turn_on_usb_ovp_fet(struct pm8921_chg_chip *chip)
{
	u8 temp;
	int rc;

	rc = pm8xxx_writeb(chip->dev->parent, USB_OVP_TEST, 0x30);
	if (rc) {
		pr_err("Failed to write 0x30 to USB_OVP_TEST rc = %d\n", rc);
		return;
	}
	rc = pm8xxx_readb(chip->dev->parent, USB_OVP_TEST, &temp);
	if (rc) {
		pr_err("Failed to read from USB_OVP_TEST rc = %d\n", rc);
		return;
	}
	/* unset ovp fet disable bit and set the write bit */
	temp &= 0xFE;
	temp |= 0x80;
	rc = pm8xxx_writeb(chip->dev->parent, USB_OVP_TEST, temp);
	if (rc) {
		pr_err("Failed to write 0x%x to USB_OVP_TEST rc = %d\n",
								temp, rc);
		return;
	}
}

static int param_open_ovp_counter = 10;
module_param(param_open_ovp_counter, int, 0644);

#define WRITE_BANK_4		0xC0
#ifdef CONFIG_LGE_PM_ADP_CHG
static void unplug_wrkarnd_restore_worker(struct work_struct *work)
{
	u8 temp;
	int rc;
	struct delayed_work *dwork = to_delayed_work(work);
	struct pm8921_chg_chip *chip = container_of(dwork,
				struct pm8921_chg_chip,
				unplug_wrkarnd_restore_work);

	pr_debug("restoring vin_min to %d mV\n", chip->vin_min);
	rc = pm_chg_vinmin_set(the_chip, chip->vin_min);

	temp = WRITE_BANK_4 | 0xA;
	rc = pm8xxx_writeb(chip->dev->parent, CHG_BUCK_CTRL_TEST3, temp);
	if (rc) {
		pr_err("Error %d writing %d to addr %d\n", rc,
					temp, CHG_BUCK_CTRL_TEST3);
	}
	wake_unlock(&chip->unplug_wrkarnd_restore_wake_lock);
}
#endif

#define USB_OVP_DEBOUNCE_TIME 0x06
static void unplug_ovp_fet_open(struct pm8921_chg_chip *chip)
{
	int chg_gone, usb_chg_plugged_in;
	int count = 0;

	while (count++ < param_open_ovp_counter) {
		pm_chg_masked_write(chip, USB_OVP_CONTROL,
						USB_OVP_DEBOUNCE_TIME, 0x0);
		usleep(10);
		usb_chg_plugged_in = is_usb_chg_plugged_in(chip);
		chg_gone = pm_chg_get_rt_status(chip, CHG_GONE_IRQ);
		pr_info("OVP FET count = %d chg_gone=%d, usb_valid = %d\n",
					count, chg_gone, usb_chg_plugged_in);

		/* note usb_chg_plugged_in=0 => chg_gone=1 */
		if (chg_gone == 1 && usb_chg_plugged_in == 1) {
			pr_debug("since chg_gone = 1 dis ovp_fet for 20msec\n");
			turn_off_usb_ovp_fet(chip);

			msleep(20);

			turn_on_usb_ovp_fet(chip);
		} else {
			break;
		}
	}
	pm_chg_masked_write(chip, USB_OVP_CONTROL,
		USB_OVP_DEBOUNCE_TIME, 0x2);
	pr_info("Exit count=%d chg_gone=%d, usb_valid=%d\n",
		count, chg_gone, usb_chg_plugged_in);
	return;
}

#ifdef CONFIG_LGE_PM_ADP_CHG
/* Adapive USB draw current limit */
int lge_get_next_iusb_max(int direction)
{
	int i, usb_mA;

	pm_chg_iusbmax_get(the_chip,&usb_mA);

	/* Increase */
	if ( direction ) {
		for ( i = 0 ; i < ADAPTIVE_NUM_OF_TABLE ; i ++ )
			if ( ADAPTIVE_MA_TABLE[i] > usb_mA ) break;
	}
	/* Decrease */
	else {
		for ( i = ADAPTIVE_NUM_OF_TABLE-1 ; i ; i -- )
			if ( ADAPTIVE_MA_TABLE[i] < usb_mA ) break;
	}

	if ( i < ADAPTIVE_NUM_OF_TABLE )
		return ADAPTIVE_MA_TABLE[i];
	else
		return ADAPTIVE_MA_TABLE[ADAPTIVE_NUM_OF_TABLE-1];
}

static void adaptive_usb_current_worker(struct work_struct *work)
{
	static int next_ma = 0, prev_ma = 0;
	struct pm8xxx_adc_chan_result Vchg;

	/* Do nothing if not in charging mode */
	if ( !the_chip->bms_notify.is_charging ) {
		prev_ma = 0;
		return;
	}

	/* Increase USB draw current
		Conditiosn : TA && open type cable */
	if ( lge_pm_get_cable_type() == CABLE_NONE &&
		the_chip->usb_psy.type == POWER_SUPPLY_TYPE_USB_DCP &&
		search_iusb_max_status == IUSB_MAX_INCREASE &&
		!thermal_mitigation ) /* 120712 cs.kim@lge.com Implements Thermal Mitigation iUSB setting */
	{
		next_ma = lge_get_next_iusb_max(1);
		if (next_ma != prev_ma) {
			printk("ChgLog> IUSB_MAX_INCREASE  %d mA\n", next_ma);
			pm8921_charger_vbus_draw(next_ma);
			prev_ma = next_ma;
			search_iusb_max_status = IUSB_MAX_INCREASE;

			/* Wait for a while to monitor Vusb-in collapse */
			msleep(ADAPTIVE_USB_CURRENT_DROP_CHECK_MS);
		}
		else
			search_iusb_max_status = IUSB_MAX_DECREASE;
	}

	/* Decrease USB draw current
		Condition : Vin < Vbat_max(4.36V) */
	pm8xxx_adc_read(CHANNEL_USBIN, &Vchg);
	if ( Vchg.physical < CHARGING_COLLAPSE_VOLTAGE*1000 &&
		 search_iusb_max_status != IUSB_MAX_NONE )
	{
		next_ma = lge_get_next_iusb_max(0);
		if ( prev_ma != next_ma ) {
			printk("ChgLog> IUSB_MAX_DECREASE Vchg %d mA	%lld uV\n", next_ma, Vchg.physical);
			pm8921_charger_vbus_draw(next_ma);
			prev_ma = next_ma;
		}
		search_iusb_max_status = IUSB_MAX_DECREASE;
	}

	/* Arm next */
	if ( search_iusb_max_status != IUSB_MAX_NONE )
		queue_delayed_work(adaptive_usb_current_queue, &adaptive_usb_current_work,
				  round_jiffies_relative(msecs_to_jiffies(ADAPTIVE_USB_CURRENT_CHECK_PERIOD_MS)));
}
#else
/* Remove QCT's Adaptive current limiting */
static int find_usb_ma_value(int value)
{
	int i;

	for (i = ARRAY_SIZE(usb_ma_table) - 1; i >= 0; i--) {
		if (value >= usb_ma_table[i].usb_ma)
			break;
	}

	return i;
}

static void decrease_usb_ma_value(int *value)
{
	int i;

	if (value) {
		i = find_usb_ma_value(*value);
		if (i > 0)
			i--;
		*value = usb_ma_table[i].usb_ma;
	}
}

static void increase_usb_ma_value(int *value)
{
	int i;

	if (value) {
		i = find_usb_ma_value(*value);

		if (i < (ARRAY_SIZE(usb_ma_table) - 1))
			i++;
		/* Get next correct entry if IUSB_FINE_RES is not available */
		while (!the_chip->iusb_fine_res
			&& (usb_ma_table[i].value & PM8917_IUSB_FINE_RES)
			&& i < (ARRAY_SIZE(usb_ma_table) - 1))
			i++;

		*value = usb_ma_table[i].usb_ma;
	}
}

static void vin_collapse_check_worker(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct pm8921_chg_chip *chip = container_of(dwork,
			struct pm8921_chg_chip, vin_collapse_check_work);

	/* AICL only for wall-chargers */
	if (is_usb_chg_plugged_in(chip) &&
		usb_target_ma > USB_WALL_THRESHOLD_MA) {
		/* decrease usb_target_ma */
		decrease_usb_ma_value(&usb_target_ma);
		/* reset here, increase in unplug_check_worker */
		__pm8921_charger_vbus_draw(USB_WALL_THRESHOLD_MA);
		pr_debug("usb_now=%d, usb_target = %d\n",
				USB_WALL_THRESHOLD_MA, usb_target_ma);
	} else {
		handle_usb_insertion_removal(chip);
	}
}
#endif /* End of CONFIG_LGE_PM_ADP_CHG */

#ifdef CONFIG_LGE_PM_REMOVE_BATT
/* Restart the machine when Battery Remove/Insert - seonghun1.kim */
static void kernel_restart_worker(struct work_struct *work)
{
	kernel_restart("battery");
}
#endif

#define VIN_MIN_COLLAPSE_CHECK_MS	50
static irqreturn_t usbin_valid_irq_handler(int irq, void *data)
{
	struct pm8921_chg_chip *chip = data;
	int high_transition;

	high_transition = pm_chg_get_rt_status(chip, USBIN_VALID_IRQ);
	ChgLog("ChgLog> %s : %d\n", __func__,high_transition);

#ifdef CONFIG_LGE_PM_ADP_CHG
	handle_usb_insertion_removal(data);
#else
	/* Remove QCT's Adaptive current limiting */
	if (usb_target_ma)
		schedule_delayed_work(&the_chip->vin_collapse_check_work,
				      round_jiffies_relative(msecs_to_jiffies
						(VIN_MIN_COLLAPSE_CHECK_MS)));
	else
		handle_usb_insertion_removal(data);
#endif

	return IRQ_HANDLED;
}

static irqreturn_t usbin_ov_irq_handler(int irq, void *data)
{
	pr_err("USB OverVoltage\n");
	return IRQ_HANDLED;
}

static irqreturn_t batt_inserted_irq_handler(int irq, void *data)
{
	struct pm8921_chg_chip *chip = data;
	int status;

	ChgLog("ChgLog> %s\n", __func__);
#ifdef CONFIG_LGE_PM_REMOVE_BATT
	/* Restart the machine when Battery Remove/Insert - seonghun1.kim */
	if(lge_pm_get_cable_type() > NO_INIT_CABLE && lge_pm_get_cable_type() <= CABLE_NONE
		&& !is_factory_cable() && !pseudo_batt_info.mode) {
		pr_info("%s: Battery inserted but cable exists. Now reset as scenario!\n", __func__);
		schedule_work(&chip->kernel_restart_work);
	}
#endif

	status = pm_chg_get_rt_status(chip, BATT_INSERTED_IRQ);

#ifdef CONFIG_LGE_PM
	/* return control to PMIC FSM */
	if (status > 0 && pm_chg_batfet_get(chip) > 0) {
		pm_chg_batfet_set(chip, 0);
	}
#endif

	schedule_work(&chip->battery_id_valid_work);
	handle_start_ext_chg(chip);
	pr_debug("battery present=%d", status);
	power_supply_changed(&chip->batt_psy);
	return IRQ_HANDLED;
}

/*
 * this interrupt used to restart charging a battery.
 *
 * Note: When DC-inserted the VBAT can't go low.
 * VPH_PWR is provided by the ext-charger.
 * After End-Of-Charging from DC, charging can be resumed only
 * if DC is removed and then inserted after the battery was in use.
 * Therefore the handle_start_ext_chg() is not called.
 */
static irqreturn_t vbatdet_low_irq_handler(int irq, void *data)
{
	struct pm8921_chg_chip *chip = data;
	int high_transition;

	high_transition = pm_chg_get_rt_status(chip, VBATDET_LOW_IRQ);
	ChgLog("ChgLog> %s : %d temp_state : %d\n", __func__,high_transition,chg_batt_temp_state);

#ifdef CONFIG_LGE_PM
	if (chg_batt_temp_state == CHG_BATT_STOP_CHARGING_STATE)
		return IRQ_HANDLED;//Batt is too cold or hot to restart charging.
#endif

	if (high_transition) {
		/* enable auto charging */
		pm_chg_auto_enable(chip, !charging_disabled);
		pr_info("batt fell below resume voltage %s\n",
			charging_disabled ? "" : "charger enabled");
	}
	pr_debug("fsm_state=%d\n", pm_chg_get_fsm_state(data));

	power_supply_changed(&chip->batt_psy);
	power_supply_changed(&chip->usb_psy);
	power_supply_changed(&chip->dc_psy);

	return IRQ_HANDLED;
}

static irqreturn_t usbin_uv_irq_handler(int irq, void *data)
{
	pr_err("USB UnderVoltage\n");

#ifdef CONFIG_LGE_PM_ADP_CHG
	/* Adapive USB draw current limit */
	if ( search_iusb_max_status != IUSB_MAX_NONE ) {
		int next_ma;
		next_ma = lge_get_next_iusb_max(0);
		printk("ChgLog> IUSB_MAX_DECREASE uv_irq %d mA\n", next_ma);
		pm8921_charger_vbus_draw(next_ma);
		search_iusb_max_status = IUSB_MAX_DECREASE;
	}
#endif

	return IRQ_HANDLED;
}

static irqreturn_t vbat_ov_irq_handler(int irq, void *data)
{
	ChgLog("ChgLog> %s\n", __func__);
	pr_debug("fsm_state=%d\n", pm_chg_get_fsm_state(data));
	return IRQ_HANDLED;
}

static irqreturn_t chgwdog_irq_handler(int irq, void *data)
{
	ChgLog("ChgLog> %s\n", __func__);
	pr_debug("fsm_state=%d\n", pm_chg_get_fsm_state(data));
	return IRQ_HANDLED;
}

static irqreturn_t vcp_irq_handler(int irq, void *data)
{
	ChgLog("ChgLog> %s\n", __func__);
	pr_debug("fsm_state=%d\n", pm_chg_get_fsm_state(data));
	return IRQ_HANDLED;
}

static irqreturn_t atcdone_irq_handler(int irq, void *data)
{
	ChgLog("ChgLog> %s\n", __func__);
	pr_debug("fsm_state=%d\n", pm_chg_get_fsm_state(data));
	return IRQ_HANDLED;
}

static irqreturn_t atcfail_irq_handler(int irq, void *data)
{
	ChgLog("ChgLog> %s\n", __func__);
	pr_debug("fsm_state=%d\n", pm_chg_get_fsm_state(data));
	return IRQ_HANDLED;
}

static irqreturn_t chgdone_irq_handler(int irq, void *data)
{
	struct pm8921_chg_chip *chip = data;

	ChgLog("ChgLog> %s\n", __func__);
	pr_debug("state_changed_to=%d\n", pm_chg_get_fsm_state(data));

	handle_stop_ext_chg(chip);

	power_supply_changed(&chip->batt_psy);
	power_supply_changed(&chip->usb_psy);
	power_supply_changed(&chip->dc_psy);

#ifdef CONFIG_LGE_PM
	chip->bms_notify.is_charge_done_hit = 1;
#endif
	bms_notify_check(chip);

	return IRQ_HANDLED;
}

static irqreturn_t chgfail_irq_handler(int irq, void *data)
{
	struct pm8921_chg_chip *chip = data;
	int ret;

	ChgLog("ChgLog> %s\n", __func__);
	ret = pm_chg_failed_clear(chip, 1);
	if (ret)
		pr_err("Failed to write CHG_FAILED_CLEAR bit\n");

	pr_err("batt_present = %d, batt_temp_ok = %d, state_changed_to=%d\n",
			get_prop_batt_present(chip),
			pm_chg_get_rt_status(chip, BAT_TEMP_OK_IRQ),
			pm_chg_get_fsm_state(data));

	power_supply_changed(&chip->batt_psy);
	power_supply_changed(&chip->usb_psy);
	power_supply_changed(&chip->dc_psy);
	return IRQ_HANDLED;
}

static irqreturn_t chgstate_irq_handler(int irq, void *data)
{
	struct pm8921_chg_chip *chip = data;

	ChgLog("ChgLog> %s\n", __func__);
	pr_debug("state_changed_to=%d\n", pm_chg_get_fsm_state(data));
	power_supply_changed(&chip->batt_psy);
	power_supply_changed(&chip->usb_psy);
	power_supply_changed(&chip->dc_psy);

	bms_notify_check(chip);

	return IRQ_HANDLED;
}

static int param_vin_disable_counter = 5;
module_param(param_vin_disable_counter, int, 0644);

#ifndef CONFIG_LGE_PM_ADP_CHG
/* Revert USB unplug detection to 1043 - seonghun1.kim */
static void attempt_reverse_boost_fix(struct pm8921_chg_chip *chip,
							int count, int usb_ma)
{
	__pm8921_charger_vbus_draw(500);
	pr_debug("count = %d iusb=500mA\n", count);
	disable_input_voltage_regulation(chip);
	pr_debug("count = %d disable_input_regulation\n", count);

	msleep(20);

	pr_debug("count = %d end sleep 20ms chg_gone=%d, usb_valid = %d\n",
				count,
				pm_chg_get_rt_status(chip, CHG_GONE_IRQ),
				is_usb_chg_plugged_in(chip));
	pr_debug("count = %d restoring input regulation and usb_ma = %d\n",
		 count, usb_ma);
	enable_input_voltage_regulation(chip);
	__pm8921_charger_vbus_draw(usb_ma);
}
#endif

#define VIN_ACTIVE_BIT BIT(0)
#define UNPLUG_WRKARND_RESTORE_WAIT_PERIOD_US 200
#define VIN_MIN_INCREASE_MV 100
#ifdef CONFIG_LGE_PM_ADP_CHG
static void unplug_check_worker(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct pm8921_chg_chip *chip = container_of(dwork,
				struct pm8921_chg_chip, unplug_check_work);
	u8 reg_loop;
	int ibat, usb_chg_plugged_in;

	usb_chg_plugged_in = is_usb_chg_plugged_in(chip);
	if (!usb_chg_plugged_in) {
		pr_debug("Stopping Unplug Check Worker since USB is removed"
			"reg_loop = %d, fsm = %d ibat = %d\n",
			pm_chg_get_regulation_loop(chip),
			pm_chg_get_fsm_state(chip),
			get_prop_batt_current(chip)
			);
		return;
	}
	reg_loop = pm_chg_get_regulation_loop(chip);
	pr_debug("reg_loop=0x%x\n", reg_loop);

	if (reg_loop & VIN_ACTIVE_BIT) {
		ibat = get_prop_batt_current(chip);

		pr_debug("ibat = %d fsm = %d reg_loop = 0x%x\n",
				ibat, pm_chg_get_fsm_state(chip), reg_loop);
		if (ibat > 0) {
			int err;
			u8 temp;

			temp = WRITE_BANK_4 | 0xE;
			err = pm8xxx_writeb(chip->dev->parent,
						CHG_BUCK_CTRL_TEST3, temp);
			if (err) {
				pr_err("Error %d writing %d to addr %d\n", err,
						temp, CHG_BUCK_CTRL_TEST3);
			}

			pm_chg_vinmin_set(chip,
					chip->vin_min + VIN_MIN_INCREASE_MV);

			wake_lock(&chip->unplug_wrkarnd_restore_wake_lock);
			schedule_delayed_work(
				&chip->unplug_wrkarnd_restore_work,
				round_jiffies_relative(usecs_to_jiffies
				(UNPLUG_WRKARND_RESTORE_WAIT_PERIOD_US)));
		}
	}

	// USB Unplug detect using OVP_FET on/off
	if ( pm_chg_get_rt_status(chip, CHG_GONE_IRQ) &&
		is_battery_charging(pm_chg_get_fsm_state(chip)) )
	{
		int USB_Vin;
		struct pm8xxx_adc_chan_result Vchg;

		pm8xxx_adc_read(CHANNEL_USBIN, &Vchg);
		USB_Vin = Vchg.physical;
		pr_info(" USB Vin : %d\n",USB_Vin);

		/*  Turn off / on the OVP_FET for detect USB unplug if the USB Vin under Vbat_max */
		if ( USB_Vin/1000 <= chip->max_voltage_mv)
		{
			unplug_ovp_fet_open(chip);
		}
	}

	schedule_delayed_work(&chip->unplug_check_work,
		      round_jiffies_relative(msecs_to_jiffies
				(UNPLUG_CHECK_WAIT_PERIOD_MS)));
}
#else
/* Revert USB unplug detection to 1043 - seonghun1.kim */
static void unplug_check_worker(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct pm8921_chg_chip *chip = container_of(dwork,
				struct pm8921_chg_chip, unplug_check_work);
	u8 reg_loop;
	int ibat, usb_chg_plugged_in, usb_ma;
	int chg_gone = 0;

	reg_loop = 0;
	usb_chg_plugged_in = is_usb_chg_plugged_in(chip);
	if (!usb_chg_plugged_in) {
		pr_debug("Stopping Unplug Check Worker since USB is removed"
			"reg_loop = %d, fsm = %d ibat = %d\n",
			pm_chg_get_regulation_loop(chip),
			pm_chg_get_fsm_state(chip),
			get_prop_batt_current(chip)
			);
		return;
	}

	pm_chg_iusbmax_get(chip, &usb_ma);
	if (usb_ma == 500 && !usb_target_ma) {
		pr_debug("Stopping Unplug Check Worker since USB == 500mA\n");
		disable_input_voltage_regulation(chip);
		return;
	}

	if (usb_ma <= 100) {
		pr_debug(
			"Unenumerated yet or suspended usb_ma = %d skipping\n",
			usb_ma);
		goto check_again_later;
	}
	if (pm8921_chg_is_enabled(chip, CHG_GONE_IRQ))
		pr_debug("chg gone irq is enabled\n");

	reg_loop = pm_chg_get_regulation_loop(chip);
	pr_debug("reg_loop=0x%x usb_ma = %d\n", reg_loop, usb_ma);

	if ((reg_loop & VIN_ACTIVE_BIT) && (usb_ma > USB_WALL_THRESHOLD_MA)) {
		decrease_usb_ma_value(&usb_ma);
		usb_target_ma = usb_ma;
		/* end AICL here */
		__pm8921_charger_vbus_draw(usb_ma);
		pr_debug("usb_now=%d, usb_target = %d\n",
			usb_ma, usb_target_ma);
	}

	reg_loop = pm_chg_get_regulation_loop(chip);
	pr_debug("reg_loop=0x%x usb_ma = %d\n", reg_loop, usb_ma);

	if (reg_loop & VIN_ACTIVE_BIT) {
		ibat = get_prop_batt_current(chip);

		pr_debug("ibat = %d fsm = %d reg_loop = 0x%x\n",
				ibat, pm_chg_get_fsm_state(chip), reg_loop);
		if (ibat > 0) {
			int count = 0;

			while (count++ < param_vin_disable_counter
					&& usb_chg_plugged_in == 1) {
				attempt_reverse_boost_fix(chip, count, usb_ma);
				usb_chg_plugged_in
					= is_usb_chg_plugged_in(chip);
			}
		}
	}

	usb_chg_plugged_in = is_usb_chg_plugged_in(chip);
	chg_gone = pm_chg_get_rt_status(chip, CHG_GONE_IRQ);

	if (chg_gone == 1  && usb_chg_plugged_in == 1) {
		/* run the worker directly */
		pr_debug(" ver5 step: chg_gone=%d, usb_valid = %d\n",
						chg_gone, usb_chg_plugged_in);
		unplug_ovp_fet_open(chip);
	}

	if (!(reg_loop & VIN_ACTIVE_BIT)) {
		/* only increase iusb_max if vin loop not active */
		if (usb_ma < usb_target_ma) {
			increase_usb_ma_value(&usb_ma);
			__pm8921_charger_vbus_draw(usb_ma);
			pr_debug("usb_now=%d, usb_target = %d\n",
					usb_ma, usb_target_ma);
		} else {
			usb_target_ma = usb_ma;
		}
	}
check_again_later:
	/* schedule to check again later */
	schedule_delayed_work(&chip->unplug_check_work,
		      round_jiffies_relative(msecs_to_jiffies
				(UNPLUG_CHECK_WAIT_PERIOD_MS)));
}
#endif

static irqreturn_t loop_change_irq_handler(int irq, void *data)
{
	struct pm8921_chg_chip *chip = data;

	ChgLog("ChgLog> %s\n", __func__);
	pr_debug("fsm_state=%d reg_loop=0x%x\n",
		pm_chg_get_fsm_state(data),
		pm_chg_get_regulation_loop(data));
	schedule_work(&chip->unplug_check_work.work);
	return IRQ_HANDLED;
}

static irqreturn_t fastchg_irq_handler(int irq, void *data)
{
	struct pm8921_chg_chip *chip = data;
	int high_transition;

	high_transition = pm_chg_get_rt_status(chip, FASTCHG_IRQ);
	ChgLog("ChgLog> %s : %d ", __func__,high_transition);
	if (high_transition && !delayed_work_pending(&chip->eoc_work)) {
		wake_lock(&chip->eoc_wake_lock);
		ChgLog(" EOC Start");
		schedule_delayed_work(&chip->eoc_work,
				      round_jiffies_relative(msecs_to_jiffies
						     (EOC_CHECK_PERIOD_MS)));
#ifdef CONFIG_LGE_PM
		/* Reset the resume charging voltage */
		pm_chg_vbatdet_set(chip, chip->max_voltage_mv - chip->resume_voltage_delta);
#endif
	}
	ChgLog(" \n");
	power_supply_changed(&chip->batt_psy);
	bms_notify_check(chip);

#ifdef CONFIG_LGE_PM_ADP_CHG
	/* Adapive USB draw current limit */
	if ( high_transition )
		queue_delayed_work(adaptive_usb_current_queue, &adaptive_usb_current_work,
			  round_jiffies_relative(msecs_to_jiffies(ADAPTIVE_USB_CURRENT_CHECK_PERIOD_MS)));
#endif

	return IRQ_HANDLED;
}

static irqreturn_t trklchg_irq_handler(int irq, void *data)
{
	struct pm8921_chg_chip *chip = data;

	ChgLog("ChgLog> %s\n", __func__);
	power_supply_changed(&chip->batt_psy);
	return IRQ_HANDLED;
}

static irqreturn_t batt_removed_irq_handler(int irq, void *data)
{
	struct pm8921_chg_chip *chip = data;
	int status;

	ChgLog("ChgLog> %s\n", __func__);
#ifdef CONFIG_LGE_PM_REMOVE_BATT
	/* Restart the machine when Battery Remove/Insert - seonghun1.kim */
	if(lge_pm_get_cable_type() > NO_INIT_CABLE && lge_pm_get_cable_type() <= CABLE_NONE
		&& !is_factory_cable() && !pseudo_batt_info.mode) {
		pr_info("%s: Battery removed but cable exists. Now reset as scenario!\n", __func__);
		schedule_work(&chip->kernel_restart_work);
	}
#endif

	status = pm_chg_get_rt_status(chip, BATT_REMOVED_IRQ);
	pr_debug("battery present=%d state=%d", !status,
					 pm_chg_get_fsm_state(data));
	handle_stop_ext_chg(chip);
	power_supply_changed(&chip->batt_psy);
	return IRQ_HANDLED;
}

static irqreturn_t batttemp_hot_irq_handler(int irq, void *data)
{
	struct pm8921_chg_chip *chip = data;

	ChgLog("ChgLog> %s\n", __func__);
	handle_stop_ext_chg(chip);
	power_supply_changed(&chip->batt_psy);
	return IRQ_HANDLED;
}

static irqreturn_t chghot_irq_handler(int irq, void *data)
{
	struct pm8921_chg_chip *chip = data;

	ChgLog("ChgLog> %s\n", __func__);
	pr_debug("Chg hot fsm_state=%d\n", pm_chg_get_fsm_state(data));
	power_supply_changed(&chip->batt_psy);
	power_supply_changed(&chip->usb_psy);
	handle_stop_ext_chg(chip);
	return IRQ_HANDLED;
}

static irqreturn_t batttemp_cold_irq_handler(int irq, void *data)
{
	struct pm8921_chg_chip *chip = data;

	ChgLog("ChgLog> %s\n", __func__);
	pr_debug("Batt cold fsm_state=%d\n", pm_chg_get_fsm_state(data));
	handle_stop_ext_chg(chip);

	power_supply_changed(&chip->batt_psy);
	power_supply_changed(&chip->usb_psy);
	return IRQ_HANDLED;
}

static irqreturn_t chg_gone_irq_handler(int irq, void *data)
{
#ifndef CONFIG_LGE_PM_ADP_CHG
	/* Revert USB unplug detection to 1043 - seonghun1.kim */
	struct pm8921_chg_chip *chip = data;
	int chg_gone, usb_chg_plugged_in;

	usb_chg_plugged_in = is_usb_chg_plugged_in(chip);
	chg_gone = pm_chg_get_rt_status(chip, CHG_GONE_IRQ);

	pr_debug("chg_gone=%d, usb_valid = %d\n", chg_gone, usb_chg_plugged_in);
	pr_debug("Chg gone fsm_state=%d\n", pm_chg_get_fsm_state(data));

	power_supply_changed(&chip->batt_psy);
	power_supply_changed(&chip->usb_psy);
#endif
	ChgLog("ChgLog> %s\n", __func__);
	return IRQ_HANDLED;
}
/*
 *
 * bat_temp_ok_irq_handler - is edge triggered, hence it will
 * fire for two cases:
 *
 * If the interrupt line switches to high temperature is okay
 * and thus charging begins.
 * If bat_temp_ok is low this means the temperature is now
 * too hot or cold, so charging is stopped.
 *
 */
static irqreturn_t bat_temp_ok_irq_handler(int irq, void *data)
{
	int bat_temp_ok;
	struct pm8921_chg_chip *chip = data;

	ChgLog("ChgLog> %s\n", __func__);
	bat_temp_ok = pm_chg_get_rt_status(chip, BAT_TEMP_OK_IRQ);

	pr_debug("batt_temp_ok = %d fsm_state%d\n",
			 bat_temp_ok, pm_chg_get_fsm_state(data));

	if (bat_temp_ok)
		handle_start_ext_chg(chip);
	else
		handle_stop_ext_chg(chip);

	power_supply_changed(&chip->batt_psy);
	power_supply_changed(&chip->usb_psy);
	bms_notify_check(chip);
	return IRQ_HANDLED;
}

static irqreturn_t coarse_det_low_irq_handler(int irq, void *data)
{
	ChgLog("ChgLog> %s fsm:%d\n", __func__,pm_chg_get_fsm_state(data));
	pr_debug("fsm_state=%d\n", pm_chg_get_fsm_state(data));
	return IRQ_HANDLED;
}

static irqreturn_t vdd_loop_irq_handler(int irq, void *data)
{
	ChgLog("ChgLog> %s fsm:%d\n", __func__,pm_chg_get_fsm_state(data));
	pr_debug("fsm_state=%d\n", pm_chg_get_fsm_state(data));
	return IRQ_HANDLED;
}

static irqreturn_t vreg_ov_irq_handler(int irq, void *data)
{
	ChgLog("ChgLog> %s fsm:%d\n", __func__,pm_chg_get_fsm_state(data));
	pr_debug("fsm_state=%d\n", pm_chg_get_fsm_state(data));
	return IRQ_HANDLED;
}

static irqreturn_t vbatdet_irq_handler(int irq, void *data)
{
	ChgLog("ChgLog> %s fsm:%d\n", __func__,pm_chg_get_fsm_state(data));
	pr_debug("fsm_state=%d\n", pm_chg_get_fsm_state(data));
	return IRQ_HANDLED;
}

static irqreturn_t batfet_irq_handler(int irq, void *data)
{
	struct pm8921_chg_chip *chip = data;

	ChgLog("ChgLog> %s\n", __func__);
	pr_debug("vreg ov\n");
	power_supply_changed(&chip->batt_psy);
	return IRQ_HANDLED;
}

static irqreturn_t dcin_valid_irq_handler(int irq, void *data)
{
	struct pm8921_chg_chip *chip = data;
	int dc_present;

	ChgLog("ChgLog> %s\n", __func__);
	dc_present = pm_chg_get_rt_status(chip, DCIN_VALID_IRQ);
	if (chip->ext_psy)
		power_supply_set_online(chip->ext_psy, dc_present);
	chip->dc_present = dc_present;
	if (dc_present)
		handle_start_ext_chg(chip);
	else
		handle_stop_ext_chg(chip);
#ifdef CONFIG_LGE_CHARGER_TEMP_SCENARIO
	if (dc_present) {
		pseudo_ui_charging = 0;
		last_usb_chg_current = 0;
		/* 120712 cs.kim@lge.com Implements Thermal Mitigation iUSB setting */
		pre_mitigation = 0;
	}
#endif
	return IRQ_HANDLED;
}

static irqreturn_t dcin_ov_irq_handler(int irq, void *data)
{
	struct pm8921_chg_chip *chip = data;

	ChgLog("ChgLog> %s\n", __func__);
	handle_stop_ext_chg(chip);
	return IRQ_HANDLED;
}

static irqreturn_t dcin_uv_irq_handler(int irq, void *data)
{
	struct pm8921_chg_chip *chip = data;

	ChgLog("ChgLog> %s\n", __func__);
	handle_stop_ext_chg(chip);

	return IRQ_HANDLED;
}

#ifdef CONFIG_LGE_CHARGER_TEMP_SCENARIO
static int chg_is_battery_too_hot_or_too_cold(void *data,int batt_temp, int batt_level)
{
	int batt_temp_level;
	int rtnValue = 0;
	struct pm8921_chg_chip *chip = data;

	pseudo_ui_charging = 0;

	if (batt_temp > chip->temp_level_1)
		batt_temp_level = CHG_BATT_TEMP_LEVEL_1;/*  Over 55  */
	else if (batt_temp >= chip->temp_level_2)
		batt_temp_level = CHG_BATT_TEMP_LEVEL_2;/*  46~55   */
	else if (batt_temp >= chip->temp_level_3)
		batt_temp_level = CHG_BATT_TEMP_LEVEL_3;/*  42~45   */
	else if (batt_temp > chip->temp_level_4)
		batt_temp_level = CHG_BATT_TEMP_LEVEL_4;/*  M4~41   */
	else if (batt_temp >= chip->temp_level_5)
		batt_temp_level = CHG_BATT_TEMP_LEVEL_5;/*  M10~M5   */
	else
		batt_temp_level = CHG_BATT_TEMP_LEVEL_6;/*  Under M10   */

#if defined(CONFIG_MACH_APQ8064_J1SP)
/* Add temp for charing scenario on SPRINT */
	if( chg_batt_temp_state == CHG_BATT_STOP_CHARGING_STATE)
	{
		if (batt_temp <= chip->temp_level_1_1 && batt_temp >= chip->temp_level_4)
			batt_temp_level = CHG_BATT_TEMP_LEVEL_1_1; /*	M5 ~ 50	*/
	}
#endif

	pr_debug("%s state = %d bat_level=%d \n",
		__func__ ,chg_batt_temp_state, batt_temp_level);
	switch(chg_batt_temp_state)
	{
	case CHG_BATT_NORMAL_STATE:
		if (batt_temp_level == CHG_BATT_TEMP_LEVEL_1 ||
			batt_temp_level == CHG_BATT_TEMP_LEVEL_6 ||
			(batt_temp_level == CHG_BATT_TEMP_LEVEL_2 && batt_level > 4000)) {

			chg_batt_temp_state = CHG_BATT_STOP_CHARGING_STATE;
			/*
			 * kiwone.seo@lge.com 2011-0609
			 * for show charging ani.although charging is stopped : charging scenario
			 */
/* CONFIG_PM_S submit ATnT temp scenario kwangjae1.lee */
#if defined(CONFIG_MACH_APQ8064_J1D)
			if (batt_temp_level == CHG_BATT_TEMP_LEVEL_6 ||
				batt_temp_level == CHG_BATT_TEMP_LEVEL_1)
#else
			/* Change termal scenario to LGE ver 1.6 - charge Icon under -10 */
			if (batt_temp_level == CHG_BATT_TEMP_LEVEL_1)
#endif
			{
				pseudo_ui_charging = 1;
			} else if(batt_temp_level == CHG_BATT_TEMP_LEVEL_2 && batt_level > 4000) {
				/* we must show charging image although charging is stopped. */
				pseudo_ui_charging = 0;
			}
/* CONFIG_PM_E submit ATnT temp scenario kwangjae1.lee */
			pr_debug("%s: BATT TEMP OUT OF SPEC (STATE: %d) (thm: %d) (volt: %d)!.\n",
				__func__, CHG_BATT_NORMAL_STATE, batt_temp, batt_level);

			rtnValue = 1;
		} else if (batt_temp_level == CHG_BATT_TEMP_LEVEL_2 && batt_level <= 4000) {
			chg_batt_temp_state = CHG_BATT_DC_CURRENT_STATE;
#if defined(CONFIG_MACH_APQ8064_J1SP)
/* changed charing of therm for SPRINT  kwangjae1.lee@lge.com*/
			pr_debug("%s: BATT TEMP 46 ~ 53 (STATE: %d) (thm: %d) (volt: %d)!.\n",
				__func__, CHG_BATT_NORMAL_STATE, batt_temp, batt_level);
#else
			pr_debug("%s: BATT TEMP 46 ~ 55 (STATE: %d) (thm: %d) (volt: %d)!.\n",
				__func__, CHG_BATT_NORMAL_STATE, batt_temp, batt_level);
#endif
			rtnValue = 0;

			if (the_chip)
#ifdef CONFIG_LGE_PM
/* changed set current on Battery input kwangjae1.lee@lge.com */
				pm_chg_ibatmax_set(chip, PM8921_CHG_IBATMAX_MIN);
#else
				pm8921_charger_vbus_draw_internal(500);
#endif
		} else {
			chg_batt_temp_state = CHG_BATT_NORMAL_STATE;

			pr_debug("%s: BATT TEMP NORMAL (STATE: %d) (thm: %d) (volt: %d)!.\n",
				__func__, CHG_BATT_NORMAL_STATE, batt_temp, batt_level);

			rtnValue = 0;
		}
		break;

	case CHG_BATT_DC_CURRENT_STATE:
		if (batt_temp_level == CHG_BATT_TEMP_LEVEL_1 ||
			batt_temp_level == CHG_BATT_TEMP_LEVEL_6 ||
			(batt_temp_level == CHG_BATT_TEMP_LEVEL_2 && batt_level > 4000)) {

			chg_batt_temp_state = CHG_BATT_STOP_CHARGING_STATE;
			/*
			 * kiwone.seo@lge.com 2011-0609
			 * for show charging ani.although charging is stopped : charging scenario
			 */
/* CONFIG_PM_S submit ATnT temp scenario kwangjae1.lee */
#if defined(CONFIG_MACH_APQ8064_J1D)
			if (batt_temp_level == CHG_BATT_TEMP_LEVEL_6 ||
				batt_temp_level == CHG_BATT_TEMP_LEVEL_1)
#else
			/* Change termal scenario to LGE ver 1.6 - charge Icon under -10 */
			if (batt_temp_level == CHG_BATT_TEMP_LEVEL_1)
#endif
			{
				pseudo_ui_charging = 1;
			} else if(batt_temp_level == CHG_BATT_TEMP_LEVEL_2 && batt_level > 4000) {
				/* we must show charging image although charging is stopped. */
				pseudo_ui_charging = 0;
			}
/* CONFIG_PM_E submit ATnT temp scenario kwangjae1.lee */
			pr_debug("%s: BATT TEMP OUT OF SPEC (STATE: %d) (thm: %d) (volt: %d)!.\n",
				__func__, CHG_BATT_DC_CURRENT_STATE, batt_temp, batt_level);

			rtnValue = 1;

		} else if ((batt_temp_level == CHG_BATT_TEMP_LEVEL_2 ||
					batt_temp_level == CHG_BATT_TEMP_LEVEL_3) && batt_level <= 4000) {

			chg_batt_temp_state = CHG_BATT_DC_CURRENT_STATE;
#if defined(CONFIG_MACH_APQ8064_J1SP)
/* changed charing of therm for SPRINT  kwangjae1.lee@lge.com*/
			pr_debug("%s: BATT TEMP 46 ~ 53 (STATE: %d) (thm: %d) (volt: %d)!.\n",
				__func__, CHG_BATT_DC_CURRENT_STATE, batt_temp, batt_level);
#else
			pr_debug("%s: BATT TEMP 46 ~ 55 (STATE: %d) (thm: %d) (volt: %d)!.\n",
				__func__, CHG_BATT_DC_CURRENT_STATE, batt_temp, batt_level);
#endif
			rtnValue = 0;

		} else if (batt_temp_level == CHG_BATT_TEMP_LEVEL_4) {

			chg_batt_temp_state = CHG_BATT_NORMAL_STATE;

			pr_debug("%s: BATT TEMP NORMAL (STATE: %d) (thm: %d) (volt: %d)!.\n",
				__func__, CHG_BATT_DC_CURRENT_STATE, batt_temp, batt_level);

			rtnValue = 0;

			if (the_chip){
#ifdef CONFIG_LGE_PM
/* changed set current on Battery input kwangjae1.lee@lge.com */
				pm_chg_ibatmax_set(chip, chip->max_bat_chg_current);
#else
				pm8921_charger_vbus_draw(last_usb_chg_current);
#endif
				pr_err("%s: BATT TEMP NORMAL last_usb_chg_current : %d)!.\n",
					__func__, last_usb_chg_current);
			}
		} else {
			chg_batt_temp_state = CHG_BATT_DC_CURRENT_STATE;

			pr_debug("%s: BATT TEMP UNREAL (STATE: %d) (thm: %d) (volt: %d)!.\n",
				__func__, CHG_BATT_DC_CURRENT_STATE, batt_temp, batt_level);

			rtnValue = 0;
		}
		break;

	case CHG_BATT_STOP_CHARGING_STATE:
#if defined(CONFIG_MACH_APQ8064_J1SP)
/* Add temp for charing scenario on SPRINT */
		if (batt_temp_level == CHG_BATT_TEMP_LEVEL_1_1)
#else
		if (batt_temp_level == CHG_BATT_TEMP_LEVEL_4)
#endif
		{
			chg_batt_temp_state = CHG_BATT_NORMAL_STATE;

			pr_err("%s: BATT TEMP NORMAL (STATE: %d) (thm: %d) (volt: %d)!.\n",
				__func__, CHG_BATT_STOP_CHARGING_STATE, batt_temp, batt_level);

			rtnValue = 0;

			if (the_chip){
#ifdef CONFIG_LGE_PM
/* changed set current on Battery input kwangjae1.lee@lge.com */
				pm_chg_ibatmax_set(chip, chip->max_bat_chg_current);
#else
				pm8921_charger_vbus_draw(last_usb_chg_current);
#endif
				pr_err("%s: BATT TEMP NORMAL last_usb_chg_current : %d)!.\n",
					__func__, last_usb_chg_current);
			}
		}
		/*
		 * kiwone.seo@lge.com 2011-05-12, charging scenario. do not anything.
		 */
#if 0
		else if ((batt_temp_level == CHG_BATT_TEMP_46_55 && batt_level <= 4000) ||
					batt_temp_level == CHG_BATT_TEMP_42_45) {
			chg_batt_temp_state = CHG_BATT_DC_CURRENT_STATE;

			pr_debug("%s: BATT TEMP NORMAL (STATE: %d) (thm: %d) (volt: %d)!.",
				__func__,CHG_BATT_STOP_CHARGING_STATE, batt_temp,batt_level);

			rtnValue = 0;

			if (the_chip)
				pm8921_charger_vbus_draw_internal(500);
		}
#endif
		else {
			chg_batt_temp_state = CHG_BATT_STOP_CHARGING_STATE;
			/*
			 * kiwone.seo@lge.com 2011-0621
			 * 1. stop charging
			 * 2. remove charger
			 * 3 insert charger while stop charging */
/* CONFIG_PM_S submit ATnT temp scenario kwangjae1.lee */
#if defined(CONFIG_MACH_APQ8064_J1D)
			if (batt_temp_level == CHG_BATT_TEMP_LEVEL_6 ||
				batt_temp_level == CHG_BATT_TEMP_LEVEL_1)
#else
			/* Change termal scenario to LGE ver 1.6 - charge Icon under -10 */
			if (batt_temp_level == CHG_BATT_TEMP_LEVEL_1)
#endif
			{
				pseudo_ui_charging = 1;
			} else if(batt_temp_level == CHG_BATT_TEMP_LEVEL_2 && batt_level > 4000) {
				 /* we must show charging image although charging is stopped. */
				pseudo_ui_charging = 0;
			}
/* CONFIG_PM_E submit ATnT temp scenario kwangjae1.lee */
			pr_debug("%s: BATT TEMP OUT OF SPEC (STATE: %d) (thm: %d) (volt: %d)!.\n",
				__func__,CHG_BATT_STOP_CHARGING_STATE, batt_temp,batt_level);

			rtnValue = 1;
		}
		break;
	}

	return rtnValue;
}
#endif

static int __pm_batt_external_power_changed_work(struct device *dev, void *data)
{
	struct power_supply *psy = &the_chip->batt_psy;
	struct power_supply *epsy = dev_get_drvdata(dev);
	int i, dcin_irq;

	/* Only search for external supply if none is registered */
	if (!the_chip->ext_psy) {
		dcin_irq = the_chip->pmic_chg_irq[DCIN_VALID_IRQ];
		for (i = 0; i < epsy->num_supplicants; i++) {
			if (!strncmp(epsy->supplied_to[i], psy->name, 7)) {
				if (!strncmp(epsy->name, "dc", 2)) {
					the_chip->ext_psy = epsy;
					dcin_valid_irq_handler(dcin_irq,
							the_chip);
				}
			}
		}
	}
	return 0;
}

static void pm_batt_external_power_changed(struct power_supply *psy)
{
	/* Only look for an external supply if it hasn't been registered */
	if (!the_chip->ext_psy)
		class_for_each_device(power_supply_class, NULL, psy,
					 __pm_batt_external_power_changed_work);
}

/**
 * update_heartbeat - internal function to update userspace
 *		per update_time minutes
 *
 */
#ifdef CONFIG_LGE_CHARGER_TEMP_SCENARIO
/* battery of therm H/W register level reading kwangjae1.lee@lge.com */
static int get_rt_get_temp(struct pm8921_chg_chip *chip)
{
	int rc;
	struct pm8xxx_adc_chan_result result;

	rc = pm8xxx_adc_read(chip->batt_temp_channel, &result);
	if (rc) {
		pr_err("error reading adc channel = %d, rc = %d\n",
					chip->vbat_channel, rc);
		return rc;
	}
	pr_debug("batt_temp phy = %lld - offset(%d) meas = 0x%llx\n", result.physical,
		LGE_temp_OFFSET, result.measurement);

	return (int)result.adc_value;
}
#endif

#define LOW_SOC_HEARTBEAT_MS	20000
static void update_heartbeat(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct pm8921_chg_chip *chip = container_of(dwork,
				struct pm8921_chg_chip, update_heartbeat_work);

	pm_chg_failed_clear(chip, 1);
	power_supply_changed(&chip->batt_psy);
	if (chip->recent_reported_soc <= 20)
		schedule_delayed_work(&chip->update_heartbeat_work,
			      round_jiffies_relative(msecs_to_jiffies
						     (LOW_SOC_HEARTBEAT_MS)));
	else
		schedule_delayed_work(&chip->update_heartbeat_work,
			      round_jiffies_relative(msecs_to_jiffies
						     (chip->update_time)));
}
#define VDD_LOOP_ACTIVE_BIT	BIT(3)
#define VDD_MAX_INCREASE_MV	400
static int vdd_max_increase_mv = VDD_MAX_INCREASE_MV;
module_param(vdd_max_increase_mv, int, 0644);

static int ichg_threshold_ua = -400000;
module_param(ichg_threshold_ua, int, 0644);
static void adjust_vdd_max_for_fastchg(struct pm8921_chg_chip *chip)
{
	int ichg_meas_ua, vbat_uv;
	int ichg_meas_ma;
	int adj_vdd_max_mv, programmed_vdd_max;
	int vbat_batt_terminal_uv;
	int vbat_batt_terminal_mv;
	int reg_loop;
	int delta_mv = 0;

	if (chip->rconn_mohm == 0) {
		pr_debug("Exiting as rconn_mohm is 0\n");
		return;
	}
	/* adjust vdd_max only in normal temperature zone */
	if (chip->is_bat_cool || chip->is_bat_warm) {
		pr_debug("Exiting is_bat_cool = %d is_batt_warm = %d\n",
				chip->is_bat_cool, chip->is_bat_warm);
		return;
	}

	reg_loop = pm_chg_get_regulation_loop(chip);
	if (!(reg_loop & VDD_LOOP_ACTIVE_BIT)) {
		pr_debug("Exiting Vdd loop is not active reg loop=0x%x\n",
			reg_loop);
		return;
	}

	pm8921_bms_get_simultaneous_battery_voltage_and_current(&ichg_meas_ua,
								&vbat_uv);
	if (ichg_meas_ua >= 0) {
		pr_debug("Exiting ichg_meas_ua = %d > 0\n", ichg_meas_ua);
		return;
	}
	if (ichg_meas_ua <= ichg_threshold_ua) {
		pr_debug("Exiting ichg_meas_ua = %d < ichg_threshold_ua = %d\n",
					ichg_meas_ua, ichg_threshold_ua);
		return;
	}
	ichg_meas_ma = ichg_meas_ua / 1000;

	/* rconn_mohm is in milliOhms */
	vbat_batt_terminal_uv = vbat_uv + ichg_meas_ma * the_chip->rconn_mohm;
	vbat_batt_terminal_mv = vbat_batt_terminal_uv/1000;
	pm_chg_vddmax_get(the_chip, &programmed_vdd_max);

	delta_mv =  chip->max_voltage_mv - vbat_batt_terminal_mv;

	adj_vdd_max_mv = programmed_vdd_max + delta_mv;
	pr_debug("vdd_max needs to be changed by %d mv from %d to %d\n",
			delta_mv,
			programmed_vdd_max,
			adj_vdd_max_mv);

	if (adj_vdd_max_mv < chip->max_voltage_mv) {
		pr_debug("adj vdd_max lower than default max voltage\n");
		return;
	}

	if (adj_vdd_max_mv > (chip->max_voltage_mv + vdd_max_increase_mv))
		adj_vdd_max_mv = chip->max_voltage_mv + vdd_max_increase_mv;

	pr_debug("adjusting vdd_max_mv to %d to make "
		"vbat_batt_termial_uv = %d to %d\n",
		adj_vdd_max_mv, vbat_batt_terminal_uv, chip->max_voltage_mv);
	pm_chg_vddmax_set(chip, adj_vdd_max_mv);
}

enum {
	CHG_IN_PROGRESS,
	CHG_NOT_IN_PROGRESS,
	CHG_FINISHED,
};

#define VBAT_TOLERANCE_MV	70
#define CHG_DISABLE_MSLEEP	100
static int is_charging_finished(struct pm8921_chg_chip *chip)
{
	int vbat_meas_uv, vbat_meas_mv, vbat_programmed, vbatdet_low;
	int ichg_meas_ma, iterm_programmed;
	int regulation_loop, fast_chg, vcp;
	int rc;
	static int last_vbat_programmed = -EINVAL;

	if (!is_ext_charging(chip)) {
		/* return if the battery is not being fastcharged */
		fast_chg = pm_chg_get_rt_status(chip, FASTCHG_IRQ);
		pr_debug("fast_chg = %d\n", fast_chg);
		if (fast_chg == 0)
			return CHG_NOT_IN_PROGRESS;

		vcp = pm_chg_get_rt_status(chip, VCP_IRQ);
		pr_debug("vcp = %d\n", vcp);
		if (vcp == 1)
			return CHG_IN_PROGRESS;

		vbatdet_low = pm_chg_get_rt_status(chip, VBATDET_LOW_IRQ);
		pr_debug("vbatdet_low = %d\n", vbatdet_low);
		if (vbatdet_low == 1)
			return CHG_IN_PROGRESS;

		/* reset count if battery is hot/cold */
		rc = pm_chg_get_rt_status(chip, BAT_TEMP_OK_IRQ);
		pr_debug("batt_temp_ok = %d\n", rc);
		if (rc == 0)
			return CHG_IN_PROGRESS;

		/* reset count if battery voltage is less than vddmax */
		vbat_meas_uv = get_prop_battery_uvolts(chip);
		if (vbat_meas_uv < 0)
			return CHG_IN_PROGRESS;
		vbat_meas_mv = vbat_meas_uv / 1000;

		rc = pm_chg_vddmax_get(chip, &vbat_programmed);
		if (rc) {
			pr_err("couldnt read vddmax rc = %d\n", rc);
			return CHG_IN_PROGRESS;
		}
		pr_debug("vddmax = %d vbat_meas_mv=%d\n",
			 vbat_programmed, vbat_meas_mv);

		if (last_vbat_programmed == -EINVAL)
			last_vbat_programmed = vbat_programmed;
		if (last_vbat_programmed !=  vbat_programmed) {
			/* vddmax changed, reset and check again */
			pr_debug("vddmax = %d last_vdd_max=%d\n",
				 vbat_programmed, last_vbat_programmed);
			last_vbat_programmed = vbat_programmed;
			return CHG_IN_PROGRESS;
		}

		regulation_loop = pm_chg_get_regulation_loop(chip);
		if (regulation_loop < 0) {
			pr_err("couldnt read the regulation loop err=%d\n",
				regulation_loop);
			return CHG_IN_PROGRESS;
		}
		pr_debug("regulation_loop=%d\n", regulation_loop);

		if (regulation_loop != 0 && regulation_loop != VDD_LOOP)
			return CHG_IN_PROGRESS;
	} /* !is_ext_charging */

	/* reset count if battery chg current is more than iterm */
	rc = pm_chg_iterm_get(chip, &iterm_programmed);
	if (rc) {
		pr_err("couldnt read iterm rc = %d\n", rc);
		return CHG_IN_PROGRESS;
	}

	ichg_meas_ma = (get_prop_batt_current(chip)) / 1000;
	pr_debug("iterm_programmed = %d ichg_meas_ma=%d\n",
				iterm_programmed, ichg_meas_ma);
	/*
	 * ichg_meas_ma < 0 means battery is drawing current
	 * ichg_meas_ma > 0 means battery is providing current
	 */
	if (ichg_meas_ma > 0)
		return CHG_IN_PROGRESS;

	if (ichg_meas_ma * -1 > iterm_programmed)
		return CHG_IN_PROGRESS;

	return CHG_FINISHED;
}

/**
 * eoc_worker - internal function to check if battery EOC
 *		has happened
 *
 * If all conditions favouring, if the charge current is
 * less than the term current for three consecutive times
 * an EOC has happened.
 * The wakelock is released if there is no need to reshedule
 * - this happens when the battery is removed or EOC has
 * happened
 */
#define CONSECUTIVE_COUNT	3
static void eoc_worker(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct pm8921_chg_chip *chip = container_of(dwork,
				struct pm8921_chg_chip, eoc_work);
	static int count;
	int end;

	pm_chg_failed_clear(chip, 1);
	end = is_charging_finished(chip);

	if (end == CHG_NOT_IN_PROGRESS) {
		count = 0;
		wake_unlock(&chip->eoc_wake_lock);
		return;
	}

	if (end == CHG_FINISHED) {
		count++;
	} else {
		count = 0;
	}

	if (count == CONSECUTIVE_COUNT) {
		count = 0;
		pr_info("End of Charging\n");

		pm_chg_auto_enable(chip, 0);

		if (is_ext_charging(chip))
			chip->ext_charge_done = true;

		if (chip->is_bat_warm || chip->is_bat_cool)
			chip->bms_notify.is_battery_full = 0;
		else
			chip->bms_notify.is_battery_full = 1;
		/* declare end of charging by invoking chgdone interrupt */
		chgdone_irq_handler(chip->pmic_chg_irq[CHGDONE_IRQ], chip);
#ifdef CONFIG_LGE_PM
		/* Run EOC Worker once more befor enter sleep mode to give a chance handling the uevent */
		schedule_delayed_work(&chip->eoc_work,
				  round_jiffies_relative(msecs_to_jiffies
							 (EOC_CHECK_PERIOD_MS)));
#else
		wake_unlock(&chip->eoc_wake_lock);
#endif
	} else {
		adjust_vdd_max_for_fastchg(chip);
		pr_info("EOC count = %d\n", count);
		schedule_delayed_work(&chip->eoc_work,
			      round_jiffies_relative(msecs_to_jiffies
						     (EOC_CHECK_PERIOD_MS)));
	}
}

static void btm_configure_work(struct work_struct *work)
{
	int rc;

	rc = pm8xxx_adc_btm_configure(&btm_config);
	if (rc)
		pr_err("failed to configure btm rc=%d", rc);
}

DECLARE_WORK(btm_config_work, btm_configure_work);

static void set_appropriate_battery_current(struct pm8921_chg_chip *chip)
{
	unsigned int chg_current = chip->max_bat_chg_current;

	if (chip->is_bat_cool)
		chg_current = min(chg_current, chip->cool_bat_chg_current);

	if (chip->is_bat_warm)
		chg_current = min(chg_current, chip->warm_bat_chg_current);

#ifndef CONFIG_LGE_PM
	/*
	 * do nothing, we just wanna get "thermal_mitigation" values.
	 * we will use this value to set charging current setting.
	 * and it should not operate origin code.
	 */
	if (thermal_mitigation != 0 && chip->thermal_mitigation)
		chg_current = min(chg_current,
				chip->thermal_mitigation[thermal_mitigation]);
#endif
	pm_chg_ibatmax_set(the_chip, chg_current);
}

#define TEMP_HYSTERISIS_DEGC 2
static void battery_cool(bool enter)
{
	pr_debug("enter = %d\n", enter);
	ChgLog("ChgLog> %s : %d\n", __func__,enter);
	if (enter == the_chip->is_bat_cool)
		return;
	the_chip->is_bat_cool = enter;
	if (enter) {
		btm_config.low_thr_temp =
			the_chip->cool_temp_dc + TEMP_HYSTERISIS_DEGC;
		set_appropriate_battery_current(the_chip);
		pm_chg_vddmax_set(the_chip, the_chip->cool_bat_voltage);
		pm_chg_vbatdet_set(the_chip,
			the_chip->cool_bat_voltage
			- the_chip->resume_voltage_delta);
	} else {
		btm_config.low_thr_temp = the_chip->cool_temp_dc;
		set_appropriate_battery_current(the_chip);
		pm_chg_vddmax_set(the_chip, the_chip->max_voltage_mv);
		pm_chg_vbatdet_set(the_chip,
			the_chip->max_voltage_mv
			- the_chip->resume_voltage_delta);
	}
	schedule_work(&btm_config_work);
}

static void battery_warm(bool enter)
{
	pr_debug("enter = %d\n", enter);
	ChgLog("ChgLog> %s : %d\n", __func__,enter);
	if (enter == the_chip->is_bat_warm)
		return;
	the_chip->is_bat_warm = enter;
	if (enter) {
		btm_config.high_thr_temp =
			the_chip->warm_temp_dc - TEMP_HYSTERISIS_DEGC;
		set_appropriate_battery_current(the_chip);
		pm_chg_vddmax_set(the_chip, the_chip->warm_bat_voltage);
		pm_chg_vbatdet_set(the_chip,
			the_chip->warm_bat_voltage
			- the_chip->resume_voltage_delta);
	} else {
		btm_config.high_thr_temp = the_chip->warm_temp_dc;
		set_appropriate_battery_current(the_chip);
		pm_chg_vddmax_set(the_chip, the_chip->max_voltage_mv);
		pm_chg_vbatdet_set(the_chip,
			the_chip->max_voltage_mv
			- the_chip->resume_voltage_delta);
	}
	schedule_work(&btm_config_work);
}

static int configure_btm(struct pm8921_chg_chip *chip)
{
	int rc;

	if (chip->warm_temp_dc != INT_MIN)
		btm_config.btm_warm_fn = battery_warm;
	else
		btm_config.btm_warm_fn = NULL;

	if (chip->cool_temp_dc != INT_MIN)
		btm_config.btm_cool_fn = battery_cool;
	else
		btm_config.btm_cool_fn = NULL;

	btm_config.low_thr_temp = chip->cool_temp_dc;
	btm_config.high_thr_temp = chip->warm_temp_dc;
	btm_config.interval = chip->temp_check_period;
	rc = pm8xxx_adc_btm_configure(&btm_config);
	if (rc)
		pr_err("failed to configure btm rc = %d\n", rc);
	rc = pm8xxx_adc_btm_start();
	if (rc)
		pr_err("failed to start btm rc = %d\n", rc);

	return rc;
}

/**
 * set_disable_status_param -
 *
 * Internal function to disable battery charging and also disable drawing
 * any current from the source. The device is forced to run on a battery
 * after this.
 */
static int set_disable_status_param(const char *val, struct kernel_param *kp)
{
	int ret;
	struct pm8921_chg_chip *chip = the_chip;

	ret = param_set_int(val, kp);
	if (ret) {
		pr_err("error setting value %d\n", ret);
		return ret;
	}
	pr_info("factory set disable param to %d\n", charging_disabled);
	if (chip) {
		pm_chg_auto_enable(chip, !charging_disabled);
		pm_chg_charge_dis(chip, charging_disabled);
	}
	return 0;
}
module_param_call(disabled, set_disable_status_param, param_get_uint,
					&charging_disabled, 0644);

static int rconn_mohm;
static int set_rconn_mohm(const char *val, struct kernel_param *kp)
{
	int ret;
	struct pm8921_chg_chip *chip = the_chip;

	ret = param_set_int(val, kp);
	if (ret) {
		pr_err("error setting value %d\n", ret);
		return ret;
	}
	if (chip)
		chip->rconn_mohm = rconn_mohm;
	return 0;
}
module_param_call(rconn_mohm, set_rconn_mohm, param_get_uint,
					&rconn_mohm, 0644);
/**
 * set_thermal_mitigation_level -
 *
 * Internal function to control battery charging current to reduce
 * temperature
 */
static int set_therm_mitigation_level(const char *val, struct kernel_param *kp)
{
	int ret;
	struct pm8921_chg_chip *chip = the_chip;

	ret = param_set_int(val, kp);
	if (ret) {
		pr_err("error setting value %d\n", ret);
		return ret;
	}

	if (!chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	if (!chip->thermal_mitigation) {
		pr_err("no thermal mitigation\n");
		return -EINVAL;
	}

#ifndef CONFIG_LGE_PM
	/*
	 * do nothing, we just wanna get "thermal_mitigation" values.
	 * we will use this value to set charging current setting.
	 * and it should not operate origin code.
	 */
	if (thermal_mitigation < 0
		|| thermal_mitigation >= chip->thermal_levels) {
		pr_err("out of bound level selected\n");
		return -EINVAL;
	}
	set_appropriate_battery_current(chip);
#endif

	return ret;
}
module_param_call(thermal_mitigation, set_therm_mitigation_level,
					param_get_uint,
					&thermal_mitigation, 0644);

static int set_usb_max_current(const char *val, struct kernel_param *kp)
{
	int ret, mA;
	struct pm8921_chg_chip *chip = the_chip;

	ret = param_set_int(val, kp);
	if (ret) {
		pr_err("error setting value %d\n", ret);
		return ret;
	}
	if (chip) {
		pr_warn("setting current max to %d\n", usb_max_current);
		pm_chg_iusbmax_get(chip, &mA);
		if (mA > usb_max_current)
			pm8921_charger_vbus_draw(usb_max_current);
		return 0;
	}
	return -EINVAL;
}
module_param_call(usb_max_current, set_usb_max_current,
	param_get_uint, &usb_max_current, 0644);

#ifdef CONFIG_LGE_PM_LOW_BATT_CHG
/* this is indicator that chargerlogo app is running. */
/* sysfs : /sys/module/pm8921_charger/parameters/chargerlogo_state */
static int param_set_chargerlogo_state(const char *val, struct kernel_param *kp)
{
	int ret;

	ret = param_set_int(val, kp);
	if (ret) {
		pr_err("error setting value %d\n", ret);
		return ret;
	}

	pr_info("set chargerlogo_state to %d\n", chargerlogo_state);
	return 0;
}
module_param_call(chargerlogo_state, param_set_chargerlogo_state,
	param_get_uint, &chargerlogo_state, 0644);
#endif

#if defined(CONFIG_MACH_APQ8064_J1KD) || defined(CONFIG_MACH_APQ8064_J1D)
/* this is indicator that coincell is charging. */
/* sysfs : /sys/module/pm8921_charger/parameters/coincell_charging */
static int param_set_coincell_charging(const char *val, struct kernel_param *kp)
{
	int ret;
	struct pm8xxx_coincell_chg chg_config;

	ret = param_set_int(val, kp);
	if (ret) {
		pr_err("error setting value %d\n", ret);
		return ret;
	}

	chg_config.state = coincell_charging;
	chg_config.voltage = PM8XXX_COINCELL_VOLTAGE_3p2V;
	chg_config.resistor = PM8XXX_COINCELL_RESISTOR_800_OHMS;
	ret = pm8xxx_coincell_chg_config(&chg_config);
	if (ret) {
		pr_err("error pm8xxx_coincell_chg_config %d\n", ret);
		return ret;
	}
	pr_info("set coincell_charging to %d\n", coincell_charging);
	return 0;
}
module_param_call(coincell_charging, param_set_coincell_charging,
	param_get_uint, &coincell_charging, 0644);
#endif

static void free_irqs(struct pm8921_chg_chip *chip)
{
	int i;

	for (i = 0; i < PM_CHG_MAX_INTS; i++)
		if (chip->pmic_chg_irq[i]) {
			free_irq(chip->pmic_chg_irq[i], chip);
			chip->pmic_chg_irq[i] = 0;
		}
}

/* determines the initial present states */
static void __devinit determine_initial_state(struct pm8921_chg_chip *chip)
{
	unsigned long flags;
	int fsm_state;

	chip->dc_present = !!is_dc_chg_plugged_in(chip);
	chip->usb_present = !!is_usb_chg_plugged_in(chip);

	notify_usb_of_the_plugin_event(chip->usb_present);
	if (chip->usb_present) {
		schedule_delayed_work(&chip->unplug_check_work,
			round_jiffies_relative(msecs_to_jiffies
				(UNPLUG_CHECK_WAIT_PERIOD_MS)));
#ifndef CONFIG_LGE_PM
		pm8921_chg_enable_irq(chip, CHG_GONE_IRQ); //Removed - seonghun1.kim
#endif
	}

	pm8921_chg_enable_irq(chip, DCIN_VALID_IRQ);
	pm8921_chg_enable_irq(chip, USBIN_VALID_IRQ);
	pm8921_chg_enable_irq(chip, BATT_REMOVED_IRQ);
	pm8921_chg_enable_irq(chip, BATT_INSERTED_IRQ);
#ifndef CONFIG_LGE_PM
	pm8921_chg_enable_irq(chip, USBIN_OV_IRQ); //Removed - seonghun1.kim
#endif
	pm8921_chg_enable_irq(chip, USBIN_UV_IRQ);
	pm8921_chg_enable_irq(chip, DCIN_OV_IRQ);
	pm8921_chg_enable_irq(chip, DCIN_UV_IRQ);
	pm8921_chg_enable_irq(chip, CHGFAIL_IRQ);
	pm8921_chg_enable_irq(chip, FASTCHG_IRQ);
	pm8921_chg_enable_irq(chip, VBATDET_LOW_IRQ);
	pm8921_chg_enable_irq(chip, BAT_TEMP_OK_IRQ);

#ifdef CONFIG_LGE_PM
	/* return control to PMIC FSM */
	pr_info("boot battery present=%d\n", pm_chg_get_rt_status(chip, BATT_INSERTED_IRQ));
	if( pm_chg_batfet_get(chip) > 0 && pm_chg_get_rt_status(chip, BATT_INSERTED_IRQ) > 0 )
	{
		pm_chg_batfet_set(chip, 0);
	}
#endif

	spin_lock_irqsave(&vbus_lock, flags);
#ifdef CONFIG_LGE_PM
	/* Fix for EOC check at Power on - for Charger Logo */
	if (usb_chg_current || pm_chg_get_rt_status(chip, USBIN_VALID_IRQ)) {
		if (usb_chg_current)
			__pm8921_charger_vbus_draw(usb_chg_current);
		fastchg_irq_handler(chip->pmic_chg_irq[FASTCHG_IRQ], chip);
	}
#else
	if (usb_chg_current) {
		/* reissue a vbus draw call */
		__pm8921_charger_vbus_draw(usb_chg_current);
		fastchg_irq_handler(chip->pmic_chg_irq[FASTCHG_IRQ], chip);
	}
#endif
	spin_unlock_irqrestore(&vbus_lock, flags);

	fsm_state = pm_chg_get_fsm_state(chip);
	if (is_battery_charging(fsm_state)) {
		chip->bms_notify.is_charging = 1;
		pm8921_bms_charging_began();
	}

	check_battery_valid(chip);

	pr_debug("usb = %d, dc = %d  batt = %d state=%d\n",
			chip->usb_present,
			chip->dc_present,
			get_prop_batt_present(chip),
			fsm_state);
}

struct pm_chg_irq_init_data {
	unsigned int	irq_id;
	char		*name;
	unsigned long	flags;
	irqreturn_t	(*handler)(int, void *);
};

#define CHG_IRQ(_id, _flags, _handler) \
{ \
	.irq_id		= _id, \
	.name		= #_id, \
	.flags		= _flags, \
	.handler	= _handler, \
}
struct pm_chg_irq_init_data chg_irq_data[] = {
	CHG_IRQ(USBIN_VALID_IRQ, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
						usbin_valid_irq_handler),
	CHG_IRQ(USBIN_OV_IRQ, IRQF_TRIGGER_RISING, usbin_ov_irq_handler),
	CHG_IRQ(BATT_INSERTED_IRQ, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
						batt_inserted_irq_handler),
	CHG_IRQ(VBATDET_LOW_IRQ, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
						vbatdet_low_irq_handler),
	CHG_IRQ(USBIN_UV_IRQ, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
							usbin_uv_irq_handler),
	CHG_IRQ(VBAT_OV_IRQ, IRQF_TRIGGER_RISING, vbat_ov_irq_handler),
	CHG_IRQ(CHGWDOG_IRQ, IRQF_TRIGGER_RISING, chgwdog_irq_handler),
	CHG_IRQ(VCP_IRQ, IRQF_TRIGGER_RISING, vcp_irq_handler),
	CHG_IRQ(ATCDONE_IRQ, IRQF_TRIGGER_RISING, atcdone_irq_handler),
	CHG_IRQ(ATCFAIL_IRQ, IRQF_TRIGGER_RISING, atcfail_irq_handler),
	CHG_IRQ(CHGDONE_IRQ, IRQF_TRIGGER_RISING, chgdone_irq_handler),
	CHG_IRQ(CHGFAIL_IRQ, IRQF_TRIGGER_RISING, chgfail_irq_handler),
	CHG_IRQ(CHGSTATE_IRQ, IRQF_TRIGGER_RISING, chgstate_irq_handler),
	CHG_IRQ(LOOP_CHANGE_IRQ, IRQF_TRIGGER_RISING, loop_change_irq_handler),
	CHG_IRQ(FASTCHG_IRQ, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
						fastchg_irq_handler),
	CHG_IRQ(TRKLCHG_IRQ, IRQF_TRIGGER_RISING, trklchg_irq_handler),
	CHG_IRQ(BATT_REMOVED_IRQ, IRQF_TRIGGER_RISING,
						batt_removed_irq_handler),
	CHG_IRQ(BATTTEMP_HOT_IRQ, IRQF_TRIGGER_RISING,
						batttemp_hot_irq_handler),
	CHG_IRQ(CHGHOT_IRQ, IRQF_TRIGGER_RISING, chghot_irq_handler),
	CHG_IRQ(BATTTEMP_COLD_IRQ, IRQF_TRIGGER_RISING,
						batttemp_cold_irq_handler),
	CHG_IRQ(CHG_GONE_IRQ, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
						chg_gone_irq_handler),
	CHG_IRQ(BAT_TEMP_OK_IRQ, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
						bat_temp_ok_irq_handler),
	CHG_IRQ(COARSE_DET_LOW_IRQ, IRQF_TRIGGER_RISING,
						coarse_det_low_irq_handler),
	CHG_IRQ(VDD_LOOP_IRQ, IRQF_TRIGGER_RISING, vdd_loop_irq_handler),
	CHG_IRQ(VREG_OV_IRQ, IRQF_TRIGGER_RISING, vreg_ov_irq_handler),
	CHG_IRQ(VBATDET_IRQ, IRQF_TRIGGER_RISING, vbatdet_irq_handler),
	CHG_IRQ(BATFET_IRQ, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
						batfet_irq_handler),
	CHG_IRQ(DCIN_VALID_IRQ, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
						dcin_valid_irq_handler),
	CHG_IRQ(DCIN_OV_IRQ, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
						dcin_ov_irq_handler),
	CHG_IRQ(DCIN_UV_IRQ, IRQF_TRIGGER_RISING, dcin_uv_irq_handler),
};

static int __devinit request_irqs(struct pm8921_chg_chip *chip,
					struct platform_device *pdev)
{
	struct resource *res;
	int ret, i;

	ret = 0;
	bitmap_fill(chip->enabled_irqs, PM_CHG_MAX_INTS);

	for (i = 0; i < ARRAY_SIZE(chg_irq_data); i++) {
		res = platform_get_resource_byname(pdev, IORESOURCE_IRQ,
				chg_irq_data[i].name);
		if (res == NULL) {
			pr_err("couldn't find %s\n", chg_irq_data[i].name);
			goto err_out;
		}
		chip->pmic_chg_irq[chg_irq_data[i].irq_id] = res->start;
		ret = request_irq(res->start, chg_irq_data[i].handler,
			chg_irq_data[i].flags,
			chg_irq_data[i].name, chip);
		if (ret < 0) {
			pr_err("couldn't request %d (%s) %d\n", res->start,
					chg_irq_data[i].name, ret);
			chip->pmic_chg_irq[chg_irq_data[i].irq_id] = 0;
			goto err_out;
		}
		pm8921_chg_disable_irq(chip, chg_irq_data[i].irq_id);
	}
	return 0;

err_out:
	free_irqs(chip);
	return -EINVAL;
}

static void pm8921_chg_force_19p2mhz_clk(struct pm8921_chg_chip *chip)
{
	int err;
	u8 temp;

	temp  = 0xD1;
	err = pm8xxx_writeb(chip->dev->parent, CHG_TEST, temp);
	if (err) {
		pr_err("Error %d writing %d to addr %d\n", err, temp, CHG_TEST);
		return;
	}

	temp  = 0xD3;
	err = pm8xxx_writeb(chip->dev->parent, CHG_TEST, temp);
	if (err) {
		pr_err("Error %d writing %d to addr %d\n", err, temp, CHG_TEST);
		return;
	}

	temp  = 0xD1;
	err = pm8xxx_writeb(chip->dev->parent, CHG_TEST, temp);
	if (err) {
		pr_err("Error %d writing %d to addr %d\n", err, temp, CHG_TEST);
		return;
	}

	temp  = 0xD5;
	err = pm8xxx_writeb(chip->dev->parent, CHG_TEST, temp);
	if (err) {
		pr_err("Error %d writing %d to addr %d\n", err, temp, CHG_TEST);
		return;
	}

	udelay(183);

	temp  = 0xD1;
	err = pm8xxx_writeb(chip->dev->parent, CHG_TEST, temp);
	if (err) {
		pr_err("Error %d writing %d to addr %d\n", err, temp, CHG_TEST);
		return;
	}

	temp  = 0xD0;
	err = pm8xxx_writeb(chip->dev->parent, CHG_TEST, temp);
	if (err) {
		pr_err("Error %d writing %d to addr %d\n", err, temp, CHG_TEST);
		return;
	}
	udelay(32);

	temp  = 0xD1;
	err = pm8xxx_writeb(chip->dev->parent, CHG_TEST, temp);
	if (err) {
		pr_err("Error %d writing %d to addr %d\n", err, temp, CHG_TEST);
		return;
	}

	temp  = 0xD3;
	err = pm8xxx_writeb(chip->dev->parent, CHG_TEST, temp);
	if (err) {
		pr_err("Error %d writing %d to addr %d\n", err, temp, CHG_TEST);
		return;
	}
}

static void pm8921_chg_set_hw_clk_switching(struct pm8921_chg_chip *chip)
{
	int err;
	u8 temp;

	temp  = 0xD1;
	err = pm8xxx_writeb(chip->dev->parent, CHG_TEST, temp);
	if (err) {
		pr_err("Error %d writing %d to addr %d\n", err, temp, CHG_TEST);
		return;
	}

	temp  = 0xD0;
	err = pm8xxx_writeb(chip->dev->parent, CHG_TEST, temp);
	if (err) {
		pr_err("Error %d writing %d to addr %d\n", err, temp, CHG_TEST);
		return;
	}
}

#define VREF_BATT_THERM_FORCE_ON	BIT(7)
static void detect_battery_removal(struct pm8921_chg_chip *chip)
{
	u8 temp;

	pm8xxx_readb(chip->dev->parent, CHG_CNTRL, &temp);
	pr_debug("upon restart CHG_CNTRL = 0x%x\n",  temp);

	if (!(temp & VREF_BATT_THERM_FORCE_ON))
		/*
		 * batt therm force on bit is battery backed and is default 0
		 * The charger sets this bit at init time. If this bit is found
		 * 0 that means the battery was removed. Tell the bms about it
		 */
		pm8921_bms_invalidate_shutdown_soc();
}

#ifndef QCT_CLK_KICK_START
static void pm8921_turn_on_19p2mhz_clk(void)
{
	int err;
	u8 temp;
	struct pm8921_chg_chip *chip = the_chip;

	if (!the_chip) {
		pr_err("%s called before init\n", __func__);
		return;
	}

	pr_info("Kick Start\n");

	temp  = 0xD1;
	err = pm8xxx_writeb(chip->dev->parent, CHG_TEST, temp);
	if (err) {
		pr_err("Error %d writing %d to addr %d\n", err, temp, CHG_TEST);
		return;
	}

	temp  = 0xD3;
	err = pm8xxx_writeb(chip->dev->parent, CHG_TEST, temp);
	if (err) {
		pr_err("Error %d writing %d to addr %d\n", err, temp, CHG_TEST);
		return;
	}
}

void pm8921_turn_on_19p2mhz_clk_ext(void)
{
	pm8921_turn_on_19p2mhz_clk();
}
#endif

#define ENUM_TIMER_STOP_BIT	BIT(1)
#define BOOT_DONE_BIT		BIT(6)
#define CHG_BATFET_ON_BIT	BIT(3)
#define CHG_VCP_EN		BIT(0)
#define CHG_BAT_TEMP_DIS_BIT	BIT(2)
#define SAFE_CURRENT_MA		1500
static int __devinit pm8921_chg_hw_init(struct pm8921_chg_chip *chip)
{
	int rc;
	int vdd_safe;
#ifdef CONFIG_LGE_PM
	int m;
	int mask;
#endif

	/* forcing 19p2mhz before accessing any charger registers */
	pm8921_chg_force_19p2mhz_clk(chip);

	detect_battery_removal(chip);

	rc = pm_chg_masked_write(chip, SYS_CONFIG_2,
					BOOT_DONE_BIT, BOOT_DONE_BIT);
	if (rc) {
		pr_err("Failed to set BOOT_DONE_BIT rc=%d\n", rc);
		return rc;
	}

	vdd_safe = chip->max_voltage_mv + VDD_MAX_INCREASE_MV;

	if (vdd_safe > PM8921_CHG_VDDSAFE_MAX)
		vdd_safe = PM8921_CHG_VDDSAFE_MAX;

	rc = pm_chg_vddsafe_set(chip, vdd_safe);

	if (rc) {
		pr_err("Failed to set safe voltage to %d rc=%d\n",
						chip->max_voltage_mv, rc);
		return rc;
	}

#ifdef CONFIG_LGE_PM
	/* Force enable charger at boot time for High Voltage battery */
	rc = pm_chg_vbatdet_set(chip, chip->max_voltage_mv);
#else
	rc = pm_chg_vbatdet_set(chip,
				chip->max_voltage_mv
				- chip->resume_voltage_delta);
#endif
	if (rc) {
		pr_err("Failed to set vbatdet comprator voltage to %d rc=%d\n",
			chip->max_voltage_mv - chip->resume_voltage_delta, rc);
		return rc;
	}

	rc = pm_chg_vddmax_set(chip, chip->max_voltage_mv);
	if (rc) {
		pr_err("Failed to set max voltage to %d rc=%d\n",
						chip->max_voltage_mv, rc);
		return rc;
	}
	rc = pm_chg_ibatsafe_set(chip, SAFE_CURRENT_MA);
	if (rc) {
		pr_err("Failed to set max voltage to %d rc=%d\n",
						SAFE_CURRENT_MA, rc);
		return rc;
	}
#ifdef CONFIG_LGE_PM
	if(lge_get_factory_boot())
	{
		rc = pm_chg_ibatmax_set(chip, PM8921_CHG_IBATT_FACTORY_CABLE);
		pr_info("factory setting ibatmax=%d mA\n",PM8921_CHG_IBATT_FACTORY_CABLE);
	}
	else
	{
		rc = pm_chg_ibatmax_set(chip, chip->max_bat_chg_current);
	}
#else
	rc = pm_chg_ibatmax_set(chip, chip->max_bat_chg_current);
#endif
	if (rc) {
		pr_err("Failed to set max current to 400 rc=%d\n", rc);
		return rc;
	}

	rc = pm_chg_iterm_set(chip, chip->term_current);
	if (rc) {
		pr_err("Failed to set term current to %d rc=%d\n",
						chip->term_current, rc);
		return rc;
	}

	/* Disable the ENUM TIMER */
	rc = pm_chg_masked_write(chip, PBL_ACCESS2, ENUM_TIMER_STOP_BIT,
			ENUM_TIMER_STOP_BIT);
	if (rc) {
		pr_err("Failed to set enum timer stop rc=%d\n", rc);
		return rc;
	}

#ifdef CONFIG_LGE_PM
	/* Set initial charging current based on USB max power */
	pm_chg_iusbmax_get(chip, &mask);

	for (m = 0; m < ARRAY_SIZE(usb_ma_table); m++) {
		if (usb_ma_table[m].usb_ma == usb_chg_current)
			break;
	}

	if (m >= ARRAY_SIZE(usb_ma_table)) {
		m = 2;	// default 500mA
	}

	/* Set limit current masking when factory boot mode */
	if (lge_get_factory_boot())
	{
		/*LGE_S jungwoo.yun@lge.com 2012-08-06 iusbmax set to 1100mA in 56K/910K cable*/
		if(lge_get_boot_cable_type() == LGE_BOOT_LT_CABLE_56K || lge_get_boot_cable_type() == LGE_BOOT_LT_CABLE_910K)
		{
			m = 0x0A;
		}
		else
		/*LGE_E jungwoo.yun@lge.com 2012-08-06 iusbmax set to 1100mA in 910K cable*/
		{
		       m = ARRAY_SIZE(usb_ma_table) - 1;
	        }
	}

	pr_info("sbl=%d mA, kernel=%d mA\n",
		mask, usb_ma_table[m].usb_ma);

	rc = pm_chg_iusbmax_set(chip, m);
	if (rc) {
		pr_err("Failed to set usb max to %d rc=%d\n", 0, rc);
		return rc;
	}
#endif

	if (chip->safety_time != 0) {
		rc = pm_chg_tchg_max_set(chip, chip->safety_time);
		if (rc) {
			pr_err("Failed to set max time to %d minutes rc=%d\n",
							chip->safety_time, rc);
			return rc;
		}
	}

	if (chip->ttrkl_time != 0) {
		rc = pm_chg_ttrkl_max_set(chip, chip->ttrkl_time);
		if (rc) {
			pr_err("Failed to set trkl time to %d minutes rc=%d\n",
							chip->safety_time, rc);
			return rc;
		}
	}

	if (chip->vin_min != 0) {
		rc = pm_chg_vinmin_set(chip, chip->vin_min);
		if (rc) {
			pr_err("Failed to set vin min to %d mV rc=%d\n",
							chip->vin_min, rc);
			return rc;
		}
	} else {
		chip->vin_min = pm_chg_vinmin_get(chip);
	}

	rc = pm_chg_disable_wd(chip);
	if (rc) {
		pr_err("Failed to disable wd rc=%d\n", rc);
		return rc;
	}

#ifdef CONFIG_LGE_PM
	/* Disable Battery temperature Check Function */
	rc = pm_chg_masked_write(chip, CHG_CNTRL_2, CHG_BAT_TEMP_DIS_BIT, CHG_BAT_TEMP_DIS_BIT);
#else
	rc = pm_chg_masked_write(chip, CHG_CNTRL_2,
				CHG_BAT_TEMP_DIS_BIT, 0);
#endif
	if (rc) {
		pr_err("Failed to enable temp control chg rc=%d\n", rc);
		return rc;
	}
	/* switch to a 3.2Mhz for the buck */
	rc = pm8xxx_writeb(chip->dev->parent, CHG_BUCK_CLOCK_CTRL, 0x15);
	if (rc) {
		pr_err("Failed to switch buck clk rc=%d\n", rc);
		return rc;
	}

	if (chip->trkl_voltage != 0) {
		rc = pm_chg_vtrkl_low_set(chip, chip->trkl_voltage);
		if (rc) {
			pr_err("Failed to set trkl voltage to %dmv  rc=%d\n",
							chip->trkl_voltage, rc);
			return rc;
		}
	}

	if (chip->weak_voltage != 0) {
		rc = pm_chg_vweak_set(chip, chip->weak_voltage);
		if (rc) {
			pr_err("Failed to set weak voltage to %dmv  rc=%d\n",
							chip->weak_voltage, rc);
			return rc;
		}
	}

	if (chip->trkl_current != 0) {
		rc = pm_chg_itrkl_set(chip, chip->trkl_current);
		if (rc) {
			pr_err("Failed to set trkl current to %dmA  rc=%d\n",
							chip->trkl_voltage, rc);
			return rc;
		}
	}

	if (chip->weak_current != 0) {
		rc = pm_chg_iweak_set(chip, chip->weak_current);
		if (rc) {
			pr_err("Failed to set weak current to %dmA  rc=%d\n",
							chip->weak_current, rc);
			return rc;
		}
	}

	rc = pm_chg_batt_cold_temp_config(chip, chip->cold_thr);
	if (rc) {
		pr_err("Failed to set cold config %d  rc=%d\n",
						chip->cold_thr, rc);
	}

	rc = pm_chg_batt_hot_temp_config(chip, chip->hot_thr);
	if (rc) {
		pr_err("Failed to set hot config %d  rc=%d\n",
						chip->hot_thr, rc);
	}

	rc = pm_chg_led_src_config(chip, chip->led_src_config);
	if (rc) {
		pr_err("Failed to set charger LED src config %d  rc=%d\n",
						chip->led_src_config, rc);
	}

	/* Workarounds for die 1.1 and 1.0 */
	if (pm8xxx_get_revision(chip->dev->parent) < PM8XXX_REVISION_8921_2p0) {
		pm8xxx_writeb(chip->dev->parent, CHG_BUCK_CTRL_TEST2, 0xF1);
		pm8xxx_writeb(chip->dev->parent, CHG_BUCK_CTRL_TEST3, 0xCE);
		pm8xxx_writeb(chip->dev->parent, CHG_BUCK_CTRL_TEST3, 0xD8);

		/* software workaround for correct battery_id detection */
		pm8xxx_writeb(chip->dev->parent, PSI_TXRX_SAMPLE_DATA_0, 0xFF);
		pm8xxx_writeb(chip->dev->parent, PSI_TXRX_SAMPLE_DATA_1, 0xFF);
		pm8xxx_writeb(chip->dev->parent, PSI_TXRX_SAMPLE_DATA_2, 0xFF);
		pm8xxx_writeb(chip->dev->parent, PSI_TXRX_SAMPLE_DATA_3, 0xFF);
		pm8xxx_writeb(chip->dev->parent, PSI_CONFIG_STATUS, 0x0D);
		udelay(100);
		pm8xxx_writeb(chip->dev->parent, PSI_CONFIG_STATUS, 0x0C);
	}

	/* Workarounds for die 3.0 */
	if (pm8xxx_get_revision(chip->dev->parent) == PM8XXX_REVISION_8921_3p0)
		pm8xxx_writeb(chip->dev->parent, CHG_BUCK_CTRL_TEST3, 0xAC);

	/* Enable isub_fine resolution AICL for PM8917 */
	if (pm8xxx_get_version(chip->dev->parent) == PM8XXX_VERSION_8917)
		chip->iusb_fine_res = true;

	pm8xxx_writeb(chip->dev->parent, CHG_BUCK_CTRL_TEST3, 0xD9);

	/* Disable EOC FSM processing */
	pm8xxx_writeb(chip->dev->parent, CHG_BUCK_CTRL_TEST3, 0x91);

	rc = pm_chg_masked_write(chip, CHG_CNTRL, VREF_BATT_THERM_FORCE_ON,
						VREF_BATT_THERM_FORCE_ON);
	if (rc)
		pr_err("Failed to Force Vref therm rc=%d\n", rc);

	rc = pm_chg_charge_dis(chip, charging_disabled);
	if (rc) {
		pr_err("Failed to disable CHG_CHARGE_DIS bit rc=%d\n", rc);
		return rc;
	}

	rc = pm_chg_auto_enable(chip, !charging_disabled);
	if (rc) {
		pr_err("Failed to enable charging rc=%d\n", rc);
		return rc;
	}

	return 0;
}

static int get_rt_status(void *data, u64 * val)
{
	int i = (int)data;
	int ret;

	/* global irq number is passed in via data */
	ret = pm_chg_get_rt_status(the_chip, i);
	*val = ret;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(rt_fops, get_rt_status, NULL, "%llu\n");

static int get_fsm_status(void *data, u64 * val)
{
	u8 temp;

	temp = pm_chg_get_fsm_state(the_chip);
	*val = temp;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(fsm_fops, get_fsm_status, NULL, "%llu\n");

static int get_reg_loop(void *data, u64 * val)
{
	u8 temp;

	if (!the_chip) {
		pr_err("%s called before init\n", __func__);
		return -EINVAL;
	}
	temp = pm_chg_get_regulation_loop(the_chip);
	*val = temp;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(reg_loop_fops, get_reg_loop, NULL, "0x%02llx\n");

static int get_reg(void *data, u64 * val)
{
	int addr = (int)data;
	int ret;
	u8 temp;

	ret = pm8xxx_readb(the_chip->dev->parent, addr, &temp);
	if (ret) {
		pr_err("pm8xxx_readb to %x value =%d errored = %d\n",
			addr, temp, ret);
		return -EAGAIN;
	}
	*val = temp;
	return 0;
}

static int set_reg(void *data, u64 val)
{
	int addr = (int)data;
	int ret;
	u8 temp;

	temp = (u8) val;
	ret = pm8xxx_writeb(the_chip->dev->parent, addr, temp);
	if (ret) {
		pr_err("pm8xxx_writeb to %x value =%d errored = %d\n",
			addr, temp, ret);
		return -EAGAIN;
	}
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(reg_fops, get_reg, set_reg, "0x%02llx\n");

enum {
	BAT_WARM_ZONE,
	BAT_COOL_ZONE,
};
static int get_warm_cool(void *data, u64 * val)
{
	if (!the_chip) {
		pr_err("%s called before init\n", __func__);
		return -EINVAL;
	}
	if ((int)data == BAT_WARM_ZONE)
		*val = the_chip->is_bat_warm;
	if ((int)data == BAT_COOL_ZONE)
		*val = the_chip->is_bat_cool;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(warm_cool_fops, get_warm_cool, NULL, "0x%lld\n");

static void create_debugfs_entries(struct pm8921_chg_chip *chip)
{
	int i;

	chip->dent = debugfs_create_dir("pm8921_chg", NULL);

	if (IS_ERR(chip->dent)) {
		pr_err("pmic charger couldnt create debugfs dir\n");
		return;
	}

	debugfs_create_file("CHG_CNTRL", 0644, chip->dent,
			    (void *)CHG_CNTRL, &reg_fops);
	debugfs_create_file("CHG_CNTRL_2", 0644, chip->dent,
			    (void *)CHG_CNTRL_2, &reg_fops);
	debugfs_create_file("CHG_CNTRL_3", 0644, chip->dent,
			    (void *)CHG_CNTRL_3, &reg_fops);
	debugfs_create_file("PBL_ACCESS1", 0644, chip->dent,
			    (void *)PBL_ACCESS1, &reg_fops);
	debugfs_create_file("PBL_ACCESS2", 0644, chip->dent,
			    (void *)PBL_ACCESS2, &reg_fops);
	debugfs_create_file("SYS_CONFIG_1", 0644, chip->dent,
			    (void *)SYS_CONFIG_1, &reg_fops);
	debugfs_create_file("SYS_CONFIG_2", 0644, chip->dent,
			    (void *)SYS_CONFIG_2, &reg_fops);
	debugfs_create_file("CHG_VDD_MAX", 0644, chip->dent,
			    (void *)CHG_VDD_MAX, &reg_fops);
	debugfs_create_file("CHG_VDD_SAFE", 0644, chip->dent,
			    (void *)CHG_VDD_SAFE, &reg_fops);
	debugfs_create_file("CHG_VBAT_DET", 0644, chip->dent,
			    (void *)CHG_VBAT_DET, &reg_fops);
	debugfs_create_file("CHG_IBAT_MAX", 0644, chip->dent,
			    (void *)CHG_IBAT_MAX, &reg_fops);
	debugfs_create_file("CHG_IBAT_SAFE", 0644, chip->dent,
			    (void *)CHG_IBAT_SAFE, &reg_fops);
	debugfs_create_file("CHG_VIN_MIN", 0644, chip->dent,
			    (void *)CHG_VIN_MIN, &reg_fops);
	debugfs_create_file("CHG_VTRICKLE", 0644, chip->dent,
			    (void *)CHG_VTRICKLE, &reg_fops);
	debugfs_create_file("CHG_ITRICKLE", 0644, chip->dent,
			    (void *)CHG_ITRICKLE, &reg_fops);
	debugfs_create_file("CHG_ITERM", 0644, chip->dent,
			    (void *)CHG_ITERM, &reg_fops);
	debugfs_create_file("CHG_TCHG_MAX", 0644, chip->dent,
			    (void *)CHG_TCHG_MAX, &reg_fops);
	debugfs_create_file("CHG_TWDOG", 0644, chip->dent,
			    (void *)CHG_TWDOG, &reg_fops);
	debugfs_create_file("CHG_TEMP_THRESH", 0644, chip->dent,
			    (void *)CHG_TEMP_THRESH, &reg_fops);
	debugfs_create_file("CHG_COMP_OVR", 0644, chip->dent,
			    (void *)CHG_COMP_OVR, &reg_fops);
	debugfs_create_file("CHG_BUCK_CTRL_TEST1", 0644, chip->dent,
			    (void *)CHG_BUCK_CTRL_TEST1, &reg_fops);
	debugfs_create_file("CHG_BUCK_CTRL_TEST2", 0644, chip->dent,
			    (void *)CHG_BUCK_CTRL_TEST2, &reg_fops);
	debugfs_create_file("CHG_BUCK_CTRL_TEST3", 0644, chip->dent,
			    (void *)CHG_BUCK_CTRL_TEST3, &reg_fops);
	debugfs_create_file("CHG_TEST", 0644, chip->dent,
			    (void *)CHG_TEST, &reg_fops);

	debugfs_create_file("FSM_STATE", 0644, chip->dent, NULL,
			    &fsm_fops);

	debugfs_create_file("REGULATION_LOOP_CONTROL", 0644, chip->dent, NULL,
			    &reg_loop_fops);

	debugfs_create_file("BAT_WARM_ZONE", 0644, chip->dent,
				(void *)BAT_WARM_ZONE, &warm_cool_fops);
	debugfs_create_file("BAT_COOL_ZONE", 0644, chip->dent,
				(void *)BAT_COOL_ZONE, &warm_cool_fops);

	for (i = 0; i < ARRAY_SIZE(chg_irq_data); i++) {
		if (chip->pmic_chg_irq[chg_irq_data[i].irq_id])
			debugfs_create_file(chg_irq_data[i].name, 0444,
				chip->dent,
				(void *)chg_irq_data[i].irq_id,
				&rt_fops);
	}
}

/* LGE_CHANGE
* for ATCMD
* 2011-11-04, janghyun.baek@lge.com
*/

#ifdef CONFIG_MACH_LGE
extern void machine_restart(char *cmd);

int pm8921_stop_charging_for_ATCMD(void)
{

	struct pm8921_chg_chip *chip = the_chip;

	pm8921_chg_disable_irq(chip, ATCFAIL_IRQ);
	pm8921_chg_disable_irq(chip, CHGHOT_IRQ);
	pm8921_chg_disable_irq(chip, ATCDONE_IRQ);
	pm8921_chg_disable_irq(chip, FASTCHG_IRQ);
	pm8921_chg_disable_irq(chip, CHGDONE_IRQ);
	pm8921_chg_disable_irq(chip, VBATDET_IRQ);
	pm8921_chg_disable_irq(chip, VBATDET_LOW_IRQ);

	return 1;
}
int pm8921_start_charging_for_ATCMD(void)
{

	struct pm8921_chg_chip *chip = the_chip;

	pm8921_chg_enable_irq(chip, ATCFAIL_IRQ);
	pm8921_chg_enable_irq(chip, CHGHOT_IRQ);
	pm8921_chg_enable_irq(chip, ATCDONE_IRQ);
	pm8921_chg_enable_irq(chip, FASTCHG_IRQ);
	pm8921_chg_enable_irq(chip, CHGDONE_IRQ);
	pm8921_chg_enable_irq(chip, VBATDET_IRQ);
	pm8921_chg_enable_irq(chip, VBATDET_LOW_IRQ);

	return 1;
}


static ssize_t at_chg_status_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int fsm_state, is_charging, r;
	bool b_chg_ok = false;

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	fsm_state = pm_chg_get_fsm_state(the_chip);
	is_charging = is_battery_charging(fsm_state);

	if (is_charging) {
		b_chg_ok = true;
		r = sprintf(buf, "%d\n", b_chg_ok);
		printk(KERN_INFO "at_chg_status_show , true ! buf = %s, is_charging = %d\n",
							buf, is_charging);
	} else {
		b_chg_ok = false;
		r = sprintf(buf, "%d\n", b_chg_ok);
		printk(KERN_INFO "at_chg_status_show , false ! buf = %s, is_charging = %d\n",
							buf, is_charging);
	}

	return r;
}

static ssize_t at_chg_status_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int ret = 0, batt_status = 0;
	struct pm8921_chg_chip *chip = the_chip;

	if (!count)
		return -EINVAL;

	batt_status = get_prop_batt_status(chip);

	if (strncmp(buf, "0", 1) == 0) {
		/* stop charging */
		printk(KERN_INFO "at_chg_status_store : stop charging start\n");
		if (batt_status == POWER_SUPPLY_STATUS_CHARGING) {
			ret = pm8921_stop_charging_for_ATCMD();
			pm_chg_auto_enable(chip, 0);
			pm_chg_charge_dis(chip,1);
			printk(KERN_INFO "at_chg_status_store : stop charging end\n");
		}
	} else if (strncmp(buf, "1", 1) == 0) {
		/* start charging */
		printk(KERN_INFO "at_chg_status_store : start charging start\n");
		if (batt_status != POWER_SUPPLY_STATUS_CHARGING) {
			ret = pm8921_start_charging_for_ATCMD();
			pm_chg_auto_enable(chip, 1);
			pm_chg_charge_dis(chip,0);
			printk(KERN_INFO "at_chg_status_store : start charging end\n");
		}
	}

	if(ret == 0)
		return -EINVAL;

	return ret;
}

static ssize_t at_chg_complete_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int guage_level = 0, r = 0;
        /*[start] jungwoo.yun@leg.com*/
	//guage_level = 90; //temp
        guage_level=get_prop_batt_capacity(the_chip);
        /*[end] jungwoo.yun@leg.com*/
	if (guage_level == 100) {
		r = sprintf(buf, "%d\n", 0);
	} else {
		r = sprintf(buf, "%d\n", 1);
	}
	return r;

}

static ssize_t at_chg_complete_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int ret = 0, batt_status = 0;
	struct pm8921_chg_chip *chip = the_chip;

	if (!count)
		return -EINVAL;

	batt_status = get_prop_batt_status(chip);

	if (strncmp(buf, "0", 1) == 0) {
		/* charging not complete */
		printk(KERN_INFO "at_chg_complete_store : charging not complete start\n");
		if (batt_status != POWER_SUPPLY_STATUS_CHARGING) {
			pm8921_chg_enable_irq(chip, FASTCHG_IRQ);
			pm_chg_auto_enable(chip, 1);
			printk(KERN_INFO "at_chg_complete_store : charging not complete end\n");
		}
	} else if (strncmp(buf, "1", 1) == 0) {
		/* charging complete */
		printk(KERN_INFO "at_chg_complete_store : charging complete start\n");
		if (batt_status != POWER_SUPPLY_STATUS_FULL) {
			ret = pm8921_stop_charging_for_ATCMD();
			pm_chg_auto_enable(chip, 0);
			printk(KERN_INFO "at_chg_complete_store : charging complete end\n");
		}
	}

	return ret;
}

static ssize_t at_pmic_reset_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int r = 0;
	bool pm_reset = true;

	msleep(3000); /* for waiting return values of testmode */

	machine_restart(NULL);

	r = sprintf(buf, "%d\n", pm_reset);

	return r;
}

/* test code by sungsookim */
static ssize_t at_current_limit_show(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	int r = 0;
	bool current_limit = true;

	printk(KERN_INFO "Current limit is successfully called\n");

	__pm8921_charger_vbus_draw(1500);
	r = sprintf(buf, "%d\n", current_limit);

	return r;
}

static ssize_t at_vcoin_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int r;
	struct pm8xxx_adc_chan_result result = { 0 };

	r = pm8xxx_adc_read(CHANNEL_VCOIN, &result);

	r = sprintf(buf, "%lld\n", result.physical);

	return r;
}

DEVICE_ATTR(at_current, 0644, at_current_limit_show, NULL);
DEVICE_ATTR(at_charge, 0664, at_chg_status_show, at_chg_status_store);
DEVICE_ATTR(at_chcomp, 0664, at_chg_complete_show, at_chg_complete_store);
DEVICE_ATTR(at_pmrst, 0644, at_pmic_reset_show, NULL);
DEVICE_ATTR(at_vcoin, 0644, at_vcoin_show, NULL);
#endif

#ifdef CONFIG_LGE_CHARGER_TEMP_SCENARIO
static ssize_t pm8921_batt_therm_adc_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int rc;
	struct pm8xxx_adc_chan_result result;

	rc = pm8xxx_adc_read(CHANNEL_BATT_THERM, &result);
	if (rc) {
		pr_err("error reading adc channel = %d, rc = %d\n",
					CHANNEL_BATT_THERM, rc);
		return rc;
	}
	pr_debug("batt_temp phy = %lld - offset(%d) meas = 0x%llx adc_code = %d\n",
				result.physical, LGE_temp_OFFSET, result.measurement, result.adc_code);
	rc = sprintf(buf, "%d\n", result.adc_code);
	return rc;
}
DEVICE_ATTR(batt_temp_adc, 0644, pm8921_batt_therm_adc_show, NULL);
#endif

static int pm8921_charger_suspend_noirq(struct device *dev)
{
	int rc;
	struct pm8921_chg_chip *chip = dev_get_drvdata(dev);

	rc = pm_chg_masked_write(chip, CHG_CNTRL, VREF_BATT_THERM_FORCE_ON, 0);
	if (rc)
		pr_err("Failed to Force Vref therm off rc=%d\n", rc);
	pm8921_chg_set_hw_clk_switching(chip);
	return 0;
}

static int pm8921_charger_resume_noirq(struct device *dev)
{
	int rc;
	struct pm8921_chg_chip *chip = dev_get_drvdata(dev);

	pm8921_chg_force_19p2mhz_clk(chip);

	rc = pm_chg_masked_write(chip, CHG_CNTRL, VREF_BATT_THERM_FORCE_ON,
						VREF_BATT_THERM_FORCE_ON);
	if (rc)
		pr_err("Failed to Force Vref therm on rc=%d\n", rc);
	return 0;
}

static int pm8921_charger_resume(struct device *dev)
{
	int rc;
	struct pm8921_chg_chip *chip = dev_get_drvdata(dev);

#ifdef CONFIG_LGE_CHARGER_TEMP_SCENARIO
/* Create work for temp scerario kwangjae1.lee@lge.com */
	if (chip) {
		rc = schedule_delayed_work(&chip->monitor_batt_temp_work,
				  round_jiffies_relative(msecs_to_jiffies
							 (60000)));
	}
#endif
#ifdef CONFIG_LGE_PM
/* Qulcomm charging scenario off kwangjae1.lee@lge.com */
		if (!(chip->cool_temp_dc == 0 && chip->warm_temp_dc == 0)
#else /* QCT_ORG */
	if (!(chip->cool_temp_dc == INT_MIN && chip->warm_temp_dc == INT_MIN)
#endif
		&& !(chip->keep_btm_on_suspend)) {
		rc = pm8xxx_adc_btm_configure(&btm_config);
		if (rc)
			pr_err("couldn't reconfigure btm rc=%d\n", rc);

		rc = pm8xxx_adc_btm_start();
		if (rc)
			pr_err("couldn't restart btm rc=%d\n", rc);
	}
	if (pm8921_chg_is_enabled(chip, LOOP_CHANGE_IRQ)) {
		disable_irq_wake(chip->pmic_chg_irq[LOOP_CHANGE_IRQ]);
		pm8921_chg_disable_irq(chip, LOOP_CHANGE_IRQ);
	}
	return 0;
}

static int pm8921_charger_suspend(struct device *dev)
{
	int rc;
	struct pm8921_chg_chip *chip = dev_get_drvdata(dev);

#ifdef CONFIG_LGE_CHARGER_TEMP_SCENARIO
/* Create work for temp scerario kwangjae1.lee@lge.com */
	if (chip) {
		rc = cancel_delayed_work_sync(&chip->monitor_batt_temp_work);
	}
#endif
#ifdef CONFIG_LGE_PM
/* Qulcomm charging scenario off kwangjae1.lee@lge.com */
	if (!(chip->cool_temp_dc == 0 && chip->warm_temp_dc == 0)
#else /* QCT_ORG */
	if (!(chip->cool_temp_dc == INT_MIN && chip->warm_temp_dc == INT_MIN)
#endif
		&& !(chip->keep_btm_on_suspend)) {
		rc = pm8xxx_adc_btm_end();
		if (rc)
			pr_err("Failed to disable BTM on suspend rc=%d\n", rc);
	}

	if (is_usb_chg_plugged_in(chip)) {
		pm8921_chg_enable_irq(chip, LOOP_CHANGE_IRQ);
		enable_irq_wake(chip->pmic_chg_irq[LOOP_CHANGE_IRQ]);
	}

	return 0;
}
#ifdef CONFIG_LGE_CHARGER_TEMP_SCENARIO
extern void kernel_power_off(void); //low battery protect code kwangjae1.lee
/* Create work for temp scerario kwangjae1.lee@lge.com */
static void monitor_batt_temp(struct work_struct *work)
{

	struct delayed_work *dwork = to_delayed_work(work);
	struct pm8921_chg_chip *chip = container_of(dwork,
				struct pm8921_chg_chip, monitor_batt_temp_work);
	int batt_temp;
	int batt_volt;
	int stop_charging = 0;
	int ret = 0;
	int bat_voltage;
	int iusb_pre = 0;
	static unsigned int prev_chg_batt_temp_state = 0;

	if (is_factory_cable())
		batt_temp = 25;
	else
		batt_temp = get_prop_batt_temp(chip);
	batt_volt = get_prop_battery_uvolts(chip) / 1000;

	if(pseudo_batt_info.mode == 1)
		batt_volt = (pseudo_batt_info.volt == 0) ? batt_volt : pseudo_batt_info.volt;

	bat_voltage = get_rt_get_temp(chip);
/* LGE_S kwangjae1.lee low battery protect code */
	if( batt_volt < 3400 && ((pseudo_batt_info.mode == 1) || (batt_temp/10) < -10)){
		mdelay(2000);
		if((get_prop_battery_uvolts(chip) / 1000) < 3400){
			pr_info(" power off, couse voltage low (%dmV) !!!! \n",batt_volt);
			kernel_power_off();
		}
	}
/* LGE_E kwangjae1.lee low battery protect code */
	pr_debug(" psudo=%d, factory=%d temp=%d, volt=%d, temp_ADC=%d\n",
		pseudo_batt_info.mode,is_factory_cable(), batt_temp, batt_volt, bat_voltage);

	stop_charging = chg_is_battery_too_hot_or_too_cold(chip, batt_temp, batt_volt);

	if(wake_lock_active(&chip->monitor_batt_temp_wake_lock) && !is_usb_chg_plugged_in(chip)
		&& !is_dc_chg_plugged_in(chip)) {
		printk(KERN_INFO "[PM] monitor_batt_temp: Release wake lock , no charger\n");
		wake_unlock(&chip->monitor_batt_temp_wake_lock);
	}
	if (stop_charging == 1 && last_stop_charging == 0) {
		if(wake_lock_active(&chip->eoc_wake_lock)) {
			printk(KERN_INFO "[PM] monitor_batt_temp: Charging stop & wake lock by Temperature Scenario\n");
			wake_lock(&chip->monitor_batt_temp_wake_lock);
		}
		/* When stop charging, next step do not resume charing kwangjae1.lee@lge.com */
		pm_chg_auto_enable(chip, 0);
		last_stop_charging = stop_charging;
	}
	else if (stop_charging == 0 && last_stop_charging == 1) {
		/* When stop charging, next step do not resume charing kwangjae1.lee@lge.com*/
		pm_chg_auto_enable(chip, 1);
		last_stop_charging = stop_charging;
		pseudo_ui_charging = 0;
		if(wake_lock_active(&chip->monitor_batt_temp_wake_lock)) {
			printk(KERN_INFO "[PM] monitor_batt_temp: Release wake lock by Temperature Scenario\n");
			wake_unlock(&chip->monitor_batt_temp_wake_lock);
		}
	}
	pm_chg_failed_clear(chip, 1);

	/*
	 * chager can get thermal_mitigation parameter from thermal daemon.
	 * so, we decide to use this parameter for charging current mitigation.
	 * if thermal daemon triggered by special threshold and do "battery" action,
	 * "battery actioninfo" value is come to charger.
	 */
#if defined(CONFIG_MACH_APQ8064_J1D)
	/* do nothing */
#else
	/* below 10% soc, thermal mitigation can't decrease IUSB*/
	if (pm8921_bms_get_percent_charge() <= 10) {
		thermal_mitigation = 0;
		soc_limit = 1;
	}
	/* above 30%, soc limit is cleared */
	else if ((soc_limit == 1) && (pm8921_bms_get_percent_charge() >= 30)) {
		soc_limit = 0;
		pre_mitigation = 0;
	}
#endif

	if ((!last_stop_charging) && (chg_batt_temp_state != CHG_BATT_DC_CURRENT_STATE)
			&& (the_chip->usb_psy.type == POWER_SUPPLY_TYPE_USB_DCP)) {
#if defined(CONFIG_MACH_APQ8064_J1D)
		if ((thermal_mitigation == 1)&&(thermal_mitigation != pre_mitigation)&&(soc_limit == 0)) {
			pm_chg_iusbmax_get(chip, &iusb_pre);
			__pm8921_charger_vbus_draw(900); /* set 900mA*/
			ChgLog("ChgLog> resetting current iusb_pre = %d : mitigation current = %d\n",iusb_pre, 900);
			pre_mitigation = thermal_mitigation;
		}
#else
		if ((thermal_mitigation == 1)&&(thermal_mitigation != pre_mitigation)&&(soc_limit == 0)) {
			pm_chg_iusbmax_get(chip, &iusb_pre);
			__pm8921_charger_vbus_draw(700); /* set 700mA*/
			ChgLog("ChgLog> resetting current iusb_pre = %d : mitigation current = %d\n",iusb_pre, 700);
			pre_mitigation = thermal_mitigation;
		}
#endif
		else if ((thermal_mitigation == 2)&&(thermal_mitigation != pre_mitigation)&&(soc_limit == 0)) {
			pm_chg_iusbmax_get(chip, &iusb_pre);
			__pm8921_charger_vbus_draw(500); /* set 500mA*/
			ChgLog("ChgLog> resetting current iusb_pre = %d : mitigation current = %d\n",iusb_pre, 500);
			pre_mitigation = thermal_mitigation;
		}
#if defined(CONFIG_MACH_APQ8064_J1D)
		else if ((thermal_mitigation == 0)&&(thermal_mitigation != pre_mitigation))	{
			pm_chg_iusbmax_get(chip, &iusb_pre);
			__pm8921_charger_vbus_draw(USB_WALL_THRESHOLD_MA); /* set max iusb current*/
			ChgLog("ChgLog> resetting current iusb_pre = %d : retore current = %d\n", iusb_pre, USB_WALL_THRESHOLD_MA);
			pre_mitigation = thermal_mitigation;
		}
#else
		else if ((thermal_mitigation == 0)&&(thermal_mitigation != pre_mitigation)) {
			pm_chg_iusbmax_get(chip, &iusb_pre);
			__pm8921_charger_vbus_draw(900); /* set 900mA*/
			ChgLog("ChgLog> resetting current iusb_pre = %d : retore current = %d\n", iusb_pre, 900);
			pre_mitigation = thermal_mitigation;
		}
#endif
	}

	if (prev_chg_batt_temp_state != chg_batt_temp_state) {
		prev_chg_batt_temp_state = chg_batt_temp_state;
		power_supply_changed(&chip->batt_psy);
	}

	ret = schedule_delayed_work(&chip->monitor_batt_temp_work,
					  round_jiffies_relative(msecs_to_jiffies
								 (10000)));
/*Shawn-Dbg*/{
	#include <linux/msm_tsens.h>
	int Vbat, Ibat;
	struct pm8xxx_adc_chan_result Vchg;
	struct tsens_device tsens_dev;
	unsigned long temp = 0;

	tsens_dev.sensor_num = 0;
	tsens_get_temp(&tsens_dev, &temp);

	pm8xxx_adc_read(CHANNEL_USBIN, &Vchg);
	pm8921_bms_get_simultaneous_battery_voltage_and_current(&Ibat, &Vbat);
	pm_chg_iusbmax_get(chip, &iusb_pre);
	ChgLog("ChgLog>, %d, %d, %d, %lld, %d-%d, %ld, %d, %d\n",
		pm8921_bms_get_percent_charge(), Vbat, 0-Ibat, Vchg.physical,
		is_battery_charging(pm_chg_get_fsm_state(chip)), pm_chg_get_fsm_state(chip), temp, batt_temp,iusb_pre);
/*Shawn-Dbg*/}
}
#endif
static int __devinit pm8921_charger_probe(struct platform_device *pdev)
{
	int rc = 0;
	struct pm8921_chg_chip *chip;
	const struct pm8921_charger_platform_data *pdata
				= pdev->dev.platform_data;

	if (!pdata) {
		pr_err("missing platform data\n");
		return -EINVAL;
	}

	chip = kzalloc(sizeof(struct pm8921_chg_chip),
					GFP_KERNEL);
	if (!chip) {
		pr_err("Cannot allocate pm_chg_chip\n");
		return -ENOMEM;
	}

	chip->dev = &pdev->dev;
	chip->safety_time = pdata->safety_time;
	chip->ttrkl_time = pdata->ttrkl_time;
	chip->update_time = pdata->update_time;
	chip->max_voltage_mv = pdata->max_voltage;
	chip->min_voltage_mv = pdata->min_voltage;
	chip->resume_voltage_delta = pdata->resume_voltage_delta;
	chip->term_current = pdata->term_current;
	chip->vbat_channel = pdata->charger_cdata.vbat_channel;
	chip->batt_temp_channel = pdata->charger_cdata.batt_temp_channel;
	chip->batt_id_channel = pdata->charger_cdata.batt_id_channel;
	chip->batt_id_min = pdata->batt_id_min;
	chip->batt_id_max = pdata->batt_id_max;
	if (pdata->cool_temp != INT_MIN)
		chip->cool_temp_dc = pdata->cool_temp * 10;
	else
		chip->cool_temp_dc = INT_MIN;

	if (pdata->warm_temp != INT_MIN)
		chip->warm_temp_dc = pdata->warm_temp * 10;
	else
		chip->warm_temp_dc = INT_MIN;

	chip->temp_check_period = pdata->temp_check_period;
	chip->max_bat_chg_current = pdata->max_bat_chg_current;
	chip->cool_bat_chg_current = pdata->cool_bat_chg_current;
	chip->warm_bat_chg_current = pdata->warm_bat_chg_current;
	chip->cool_bat_voltage = pdata->cool_bat_voltage;
	chip->warm_bat_voltage = pdata->warm_bat_voltage;
	chip->keep_btm_on_suspend = pdata->keep_btm_on_suspend;
	chip->trkl_voltage = pdata->trkl_voltage;
	chip->weak_voltage = pdata->weak_voltage;
	chip->trkl_current = pdata->trkl_current;
	chip->weak_current = pdata->weak_current;
	chip->vin_min = pdata->vin_min;
	chip->thermal_mitigation = pdata->thermal_mitigation;
	chip->thermal_levels = pdata->thermal_levels;

	chip->cold_thr = pdata->cold_thr;
	chip->hot_thr = pdata->hot_thr;
	chip->rconn_mohm = pdata->rconn_mohm;
	chip->led_src_config = pdata->led_src_config;

#ifdef CONFIG_LGE_CHARGER_TEMP_SCENARIO
	chip->temp_level_1 = pdata->temp_level_1;
#if defined(CONFIG_MACH_APQ8064_J1SP)
/* Add temp for charing scenario on SPRINT */
	chip->temp_level_1_1 = pdata->temp_level_1_1;
#endif
	chip->temp_level_2 = pdata->temp_level_2;
	chip->temp_level_3 = pdata->temp_level_3;
	chip->temp_level_4 = pdata->temp_level_4;
	chip->temp_level_5 = pdata->temp_level_5;
	last_stop_charging = 0;
	chg_batt_temp_state = CHG_BATT_NORMAL_STATE;
	pseudo_ui_charging = 0;
	last_usb_chg_current = 0;
	/* 120712 cs.kim@lge.com Implements Thermal Mitigation iUSB setting */
	pre_mitigation = 0;
	soc_limit = 0;
#endif

	rc = pm8921_chg_hw_init(chip);
	if (rc) {
		pr_err("couldn't init hardware rc=%d\n", rc);
		goto free_chip;
	}

	chip->usb_psy.name = "usb",
	chip->usb_psy.type = POWER_SUPPLY_TYPE_USB,
	chip->usb_psy.supplied_to = pm_power_supplied_to,
	chip->usb_psy.num_supplicants = ARRAY_SIZE(pm_power_supplied_to),
	chip->usb_psy.properties = pm_power_props_usb,
	chip->usb_psy.num_properties = ARRAY_SIZE(pm_power_props_usb),
	chip->usb_psy.get_property = pm_power_get_property_usb,

	chip->dc_psy.name = "pm8921-dc",
	chip->dc_psy.type = POWER_SUPPLY_TYPE_MAINS,
	chip->dc_psy.supplied_to = pm_power_supplied_to,
	chip->dc_psy.num_supplicants = ARRAY_SIZE(pm_power_supplied_to),
	chip->dc_psy.properties = pm_power_props_mains,
	chip->dc_psy.num_properties = ARRAY_SIZE(pm_power_props_mains),
	chip->dc_psy.get_property = pm_power_get_property_mains,

	chip->batt_psy.name = "battery",
	chip->batt_psy.type = POWER_SUPPLY_TYPE_BATTERY,
	chip->batt_psy.properties = msm_batt_power_props,
	chip->batt_psy.num_properties = ARRAY_SIZE(msm_batt_power_props),
	chip->batt_psy.get_property = pm_batt_power_get_property,
	chip->batt_psy.external_power_changed = pm_batt_external_power_changed,
	rc = power_supply_register(chip->dev, &chip->usb_psy);
	if (rc < 0) {
		pr_err("power_supply_register usb failed rc = %d\n", rc);
		goto free_chip;
	}

	rc = power_supply_register(chip->dev, &chip->dc_psy);
	if (rc < 0) {
		pr_err("power_supply_register usb failed rc = %d\n", rc);
		goto unregister_usb;
	}

	rc = power_supply_register(chip->dev, &chip->batt_psy);
	if (rc < 0) {
		pr_err("power_supply_register batt failed rc = %d\n", rc);
		goto unregister_dc;
	}

	platform_set_drvdata(pdev, chip);
	the_chip = chip;

	wake_lock_init(&chip->eoc_wake_lock, WAKE_LOCK_SUSPEND, "pm8921_eoc");
	INIT_DELAYED_WORK(&chip->eoc_work, eoc_worker);
#ifdef CONFIG_LGE_PM_ADP_CHG
	wake_lock_init(&chip->unplug_wrkarnd_restore_wake_lock,
			WAKE_LOCK_SUSPEND, "pm8921_unplug_wrkarnd");
	INIT_DELAYED_WORK(&chip->unplug_wrkarnd_restore_work,
					unplug_wrkarnd_restore_worker);
#else
	/* Revert USB unplug detection to 1043 - seonghun1.kim */
	INIT_DELAYED_WORK(&chip->vin_collapse_check_work,
						vin_collapse_check_worker);
#endif
	INIT_DELAYED_WORK(&chip->unplug_check_work, unplug_check_worker);

#ifdef CONFIG_LGE_PM_REMOVE_BATT
	/* Restart the machine when Battery Remove/Insert - seonghun1.kim */
	INIT_WORK(&chip->kernel_restart_work, kernel_restart_worker);
#endif
	rc = request_irqs(chip, pdev);
	if (rc) {
		pr_err("couldn't register interrupts rc=%d\n", rc);
		goto unregister_batt;
	}

	enable_irq_wake(chip->pmic_chg_irq[USBIN_VALID_IRQ]);
	enable_irq_wake(chip->pmic_chg_irq[USBIN_OV_IRQ]);
	enable_irq_wake(chip->pmic_chg_irq[USBIN_UV_IRQ]);
	enable_irq_wake(chip->pmic_chg_irq[BAT_TEMP_OK_IRQ]);
	enable_irq_wake(chip->pmic_chg_irq[VBATDET_LOW_IRQ]);
	enable_irq_wake(chip->pmic_chg_irq[FASTCHG_IRQ]);
	/*
	 * if both the cool_temp_dc and warm_temp_dc are invalid device doesnt
	 * care for jeita compliance
	 */
#ifdef CONFIG_LGE_PM
/* Qulcomm charging scenario off kwangjae1.lee@lge.com */
	if (!(chip->cool_temp_dc == 0 && chip->warm_temp_dc == 0))
#else  /* QCT_ORG */
	if (!(chip->cool_temp_dc == INT_MIN && chip->warm_temp_dc == INT_MIN))
#endif
	{
		rc = configure_btm(chip);
		if (rc) {
			pr_err("couldn't register with btm rc=%d\n", rc);
			goto free_irq;
		}
	}

	create_debugfs_entries(chip);

	INIT_WORK(&chip->bms_notify.work, bms_notify);
	INIT_WORK(&chip->battery_id_valid_work, battery_id_valid);

#ifdef CONFIG_LGE_PM_ADP_CHG
	/* Adapive USB draw current limit */
	INIT_DELAYED_WORK(&adaptive_usb_current_work, adaptive_usb_current_worker);
	adaptive_usb_current_queue = create_singlethread_workqueue("adaptive_usb_current_queue");

	if (!adaptive_usb_current_queue) {
		pr_err("adaptive_usb_current_queue: could not create workqueue.\n");
		destroy_workqueue(adaptive_usb_current_queue);
	}
#endif

	/* determine what state the charger is in */
	determine_initial_state(chip);

#ifdef CONFIG_MACH_LGE
	rc = device_create_file(&pdev->dev, &dev_attr_at_current);
	rc = device_create_file(&pdev->dev, &dev_attr_at_charge);
	rc = device_create_file(&pdev->dev, &dev_attr_at_chcomp);
	rc = device_create_file(&pdev->dev, &dev_attr_at_pmrst);
	rc = device_create_file(&pdev->dev, &dev_attr_at_vcoin);
#endif
#ifdef CONFIG_LGE_CHARGER_TEMP_SCENARIO
	rc = device_create_file(&pdev->dev, &dev_attr_batt_temp_adc);
#endif
	if (chip->update_time) {
		INIT_DELAYED_WORK(&chip->update_heartbeat_work,
							update_heartbeat);
		schedule_delayed_work(&chip->update_heartbeat_work,
				      round_jiffies_relative(msecs_to_jiffies
							(chip->update_time)));
	}

#ifdef CONFIG_LGE_PM
	/* Initialize wake lock for deliver Uevent before suspend */
	wake_lock_init(&chip->deliver_uevent_wake_lock,WAKE_LOCK_SUSPEND, "deliver_uevent");
#endif

#ifdef CONFIG_LGE_CHARGER_TEMP_SCENARIO
	/* Create work for temp scerario kwangjae1.lee@lge.com */
	wake_lock_init(&chip->monitor_batt_temp_wake_lock,
					WAKE_LOCK_SUSPEND,"pm8921_monitor_batt_temp");

	INIT_DELAYED_WORK(&chip->monitor_batt_temp_work,
					monitor_batt_temp);

	rc = schedule_delayed_work(&chip->monitor_batt_temp_work,
				  round_jiffies_relative(msecs_to_jiffies
						(10000)));
	pr_debug("%s rc = %d\n",__func__, rc);
#endif

	return 0;

free_irq:
	free_irqs(chip);
unregister_batt:
	power_supply_unregister(&chip->batt_psy);
unregister_dc:
	power_supply_unregister(&chip->dc_psy);
unregister_usb:
	power_supply_unregister(&chip->usb_psy);
free_chip:
	kfree(chip);
	return rc;
}

static int __devexit pm8921_charger_remove(struct platform_device *pdev)
{
	struct pm8921_chg_chip *chip = platform_get_drvdata(pdev);

#ifdef CONFIG_MACH_LGE
	device_remove_file(&pdev->dev, &dev_attr_at_current); /* sungsookim */
	device_remove_file(&pdev->dev, &dev_attr_at_charge);
	device_remove_file(&pdev->dev, &dev_attr_at_chcomp);
	device_remove_file(&pdev->dev, &dev_attr_at_pmrst);
	device_remove_file(&pdev->dev, &dev_attr_at_vcoin);
#endif
#ifdef CONFIG_LGE_CHARGER_TEMP_SCENARIO
	device_remove_file(&pdev->dev, &dev_attr_batt_temp_adc);
#endif
	free_irqs(chip);
	platform_set_drvdata(pdev, NULL);
	the_chip = NULL;
	kfree(chip);
	return 0;
}
static const struct dev_pm_ops pm8921_pm_ops = {
	.suspend	= pm8921_charger_suspend,
	.suspend_noirq  = pm8921_charger_suspend_noirq,
	.resume_noirq   = pm8921_charger_resume_noirq,
	.resume		= pm8921_charger_resume,
};
static struct platform_driver pm8921_charger_driver = {
	.probe		= pm8921_charger_probe,
	.remove		= __devexit_p(pm8921_charger_remove),
	.driver		= {
			.name	= PM8921_CHARGER_DEV_NAME,
			.owner	= THIS_MODULE,
			.pm	= &pm8921_pm_ops,
	},
};

static int __init pm8921_charger_init(void)
{
	return platform_driver_register(&pm8921_charger_driver);
}

static void __exit pm8921_charger_exit(void)
{
	platform_driver_unregister(&pm8921_charger_driver);
}

late_initcall(pm8921_charger_init);
module_exit(pm8921_charger_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("PMIC8921 charger/battery driver");
MODULE_VERSION("1.0");
MODULE_ALIAS("platform:" PM8921_CHARGER_DEV_NAME);
