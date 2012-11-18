/* ehci-msm-hsic.c - HSUSB Host Controller Driver Implementation
 *
 * Copyright (c) 2011-2012, Code Aurora Forum. All rights reserved.
 *
 * Partly derived from ehci-fsl.c and ehci-hcd.c
 * Copyright (c) 2000-2004 by David Brownell
 * Copyright (c) 2005 MontaVista Software
 *
 * All source code in this file is licensed under the following license except
 * where indicated.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you can find it at http://www.fsf.org
 */

#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/wakelock.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>

#include <linux/usb/msm_hsusb_hw.h>
#include <linux/usb/msm_hsusb.h>
#include <linux/gpio.h>
#include <linux/spinlock.h>
#include <linux/wait.h>


#include <mach/msm_bus.h>
#include <mach/clk.h>
#include <mach/msm_iomap.h>
#include <mach/msm_xo.h>
#include <linux/spinlock.h>
#include <linux/cpu.h>
#include <mach/rpm-regulator.h>

#define MSM_USB_BASE (hcd->regs)
#define USB_REG_START_OFFSET 0x90
#define USB_REG_END_OFFSET 0x250

/* #define HSIC_DISCONNECT_DEBUG */

static struct workqueue_struct  *ehci_wq;

struct ehci_timer {
#define GPT_LD(p)	((p) & 0x00FFFFFF)
	u32	gptimer0_ld;
#define GPT_RUN		BIT(31)
#define GPT_RESET	BIT(30)
#define GPT_MODE	BIT(24)
#define GPT_CNT(p)	((p) & 0x00FFFFFF)
	u32	gptimer0_ctrl;

	u32	gptimer1_ld;
	u32	gptimer1_ctrl;
};

struct msm_hsic_hcd {
	struct ehci_hcd		ehci;
	struct device		*dev;
	struct clk		*ahb_clk;
	struct clk		*core_clk;
	struct clk		*alt_core_clk;
	struct clk		*phy_clk;
	struct clk		*cal_clk;
	struct regulator	*hsic_vddcx;
	bool			async_int;
	atomic_t                in_lpm;
	struct wake_lock	wlock;
	int			peripheral_status_irq;
	int			wakeup_irq;
	int			wakeup_gpio;
	int			err_fatal;
	int			ready_gpio;
	int			pbl_gpio;
	bool			wakeup_irq_enabled;
	atomic_t		pm_usage_cnt;
	uint32_t		bus_perf_client;
	uint32_t		wakeup_int_cnt;
	enum usb_vdd_type	vdd_type;

	/* gp timer */
	struct ehci_timer __iomem *timer;

	struct work_struct	resume_w;
	wait_queue_head_t	resume_wait;
	int			resume_status;
};

extern int subsystem_restart(const char *name);
struct msm_hsic_hcd *__mehci;

static bool debug_bus_voting_enabled = true;

static unsigned int enable_dbg_log = 1;
module_param(enable_dbg_log, uint, S_IRUGO | S_IWUSR);
/*by default log ep0 and efs sync ep*/
static unsigned int ep_addr_rxdbg_mask = 9;
module_param(ep_addr_rxdbg_mask, uint, S_IRUGO | S_IWUSR);
static unsigned int ep_addr_txdbg_mask = 9;
module_param(ep_addr_txdbg_mask, uint, S_IRUGO | S_IWUSR);

/* Maximum debug message length */
#define DBG_MSG_LEN   100UL

/* Maximum number of messages */
#define DBG_MAX_MSG   1024UL

#define TIME_BUF_LEN  20

enum event_type {
	EVENT_UNDEF = -1,
	URB_SUBMIT,
	URB_COMPLETE,
	EVENT_NONE,
};

#define EVENT_STR_LEN	5

static char *event_to_str(enum event_type e)
{
	switch (e) {
	case URB_SUBMIT:
		return "S";
	case URB_COMPLETE:
		return "C";
	case EVENT_NONE:
		return "NONE";
	default:
		return "UNDEF";
	}
}

static enum event_type str_to_event(const char *name)
{
	if (!strncasecmp("S", name, EVENT_STR_LEN))
		return URB_SUBMIT;
	if (!strncasecmp("C", name, EVENT_STR_LEN))
		return URB_COMPLETE;
	if (!strncasecmp("", name, EVENT_STR_LEN))
		return EVENT_NONE;

	return EVENT_UNDEF;
}

/*log ep0 activity*/
static struct {
	char     (buf[DBG_MAX_MSG])[DBG_MSG_LEN];   /* buffer */
	unsigned idx;   /* index */
	rwlock_t lck;   /* lock */
} dbg_hsic_ctrl = {
	.idx = 0,
	.lck = __RW_LOCK_UNLOCKED(lck)
};

static struct {
	char     (buf[DBG_MAX_MSG])[DBG_MSG_LEN];   /* buffer */
	unsigned idx;   /* index */
	rwlock_t lck;   /* lock */
} dbg_hsic_data = {
	.idx = 0,
	.lck = __RW_LOCK_UNLOCKED(lck)
};

/**
 * dbg_inc: increments debug event index
 * @idx: buffer index
 */
static void dbg_inc(unsigned *idx)
{
	*idx = (*idx + 1) & (DBG_MAX_MSG-1);
}

/*get_timestamp - returns time of day in us */
static char *get_timestamp(char *tbuf)
{
	unsigned long long t;
	unsigned long nanosec_rem;

	t = cpu_clock(smp_processor_id());
	nanosec_rem = do_div(t, 1000000000)/1000;
	scnprintf(tbuf, TIME_BUF_LEN, "[%5lu.%06lu] ", (unsigned long)t,
		nanosec_rem);
	return tbuf;
}

static int allow_dbg_log(int ep_addr)
{
	int dir, num;

	dir = ep_addr & USB_DIR_IN ? USB_DIR_IN : USB_DIR_OUT;
	num = ep_addr & ~USB_DIR_IN;
	num = 1 << num;

	if ((dir == USB_DIR_IN) && (num & ep_addr_rxdbg_mask))
		return 1;
	if ((dir == USB_DIR_OUT) && (num & ep_addr_txdbg_mask))
		return 1;

	return 0;
}

void dbg_log_event(struct urb *urb, char * event, unsigned extra, char * event1, int extra1, char * event2, int extra2)
{
	unsigned long flags;
	int ep_addr;
	char tbuf[TIME_BUF_LEN];

	if (!enable_dbg_log)
		return;

	if (!urb) {
		write_lock_irqsave(&dbg_hsic_ctrl.lck, flags);
		scnprintf(dbg_hsic_ctrl.buf[dbg_hsic_ctrl.idx], DBG_MSG_LEN,
			"%s: %s : %u %s : %u %s : %u\n", get_timestamp(tbuf), event, extra,
			event1, extra1, event2, extra2);
		dbg_inc(&dbg_hsic_ctrl.idx);
		write_unlock_irqrestore(&dbg_hsic_ctrl.lck, flags);
		return;
	}

	ep_addr = urb->ep->desc.bEndpointAddress;
	if (!allow_dbg_log(ep_addr))
		return;

	if ((ep_addr & 0x0f) == 0x0) {
		/*submit event*/
		if (!str_to_event(event)) {
			write_lock_irqsave(&dbg_hsic_ctrl.lck, flags);
			scnprintf(dbg_hsic_ctrl.buf[dbg_hsic_ctrl.idx],
				DBG_MSG_LEN, "%s: [%s : %p]:[%s] "
				  "%02x %02x %04x %04x %04x  %u %d\n",
				  get_timestamp(tbuf), event, urb,
				  (ep_addr & USB_DIR_IN) ? "in" : "out",
				  urb->setup_packet[0], urb->setup_packet[1],
				  (urb->setup_packet[3] << 8) |
				  urb->setup_packet[2],
				  (urb->setup_packet[5] << 8) |
				  urb->setup_packet[4],
				  (urb->setup_packet[7] << 8) |
				  urb->setup_packet[6],
				  urb->transfer_buffer_length, urb->status);

			dbg_inc(&dbg_hsic_ctrl.idx);
			write_unlock_irqrestore(&dbg_hsic_ctrl.lck, flags);
		} else {
			write_lock_irqsave(&dbg_hsic_ctrl.lck, flags);
			scnprintf(dbg_hsic_ctrl.buf[dbg_hsic_ctrl.idx],
				DBG_MSG_LEN, "%s: [%s : %p]:[%s] %u %d\n",
				  get_timestamp(tbuf), event, urb,
				  (ep_addr & USB_DIR_IN) ? "in" : "out",
				  urb->actual_length, extra);

			dbg_inc(&dbg_hsic_ctrl.idx);
			write_unlock_irqrestore(&dbg_hsic_ctrl.lck, flags);
		}
	} else {
		write_lock_irqsave(&dbg_hsic_data.lck, flags);
		scnprintf(dbg_hsic_data.buf[dbg_hsic_data.idx], DBG_MSG_LEN,
			  "%s: [%s : %p]:ep%d[%s]  %u %d\n",
			  get_timestamp(tbuf), event, urb, ep_addr & 0x0f,
			  (ep_addr & USB_DIR_IN) ? "in" : "out",
			  str_to_event(event) ? urb->actual_length :
			  urb->transfer_buffer_length,
			  str_to_event(event) ?  extra : urb->status);

		dbg_inc(&dbg_hsic_data.idx);
		write_unlock_irqrestore(&dbg_hsic_data.lck, flags);
	}
}

static inline struct msm_hsic_hcd *hcd_to_hsic(struct usb_hcd *hcd)
{
	return (struct msm_hsic_hcd *) (hcd->hcd_priv);
}

static inline struct usb_hcd *hsic_to_hcd(struct msm_hsic_hcd *mehci)
{
	return container_of((void *) mehci, struct usb_hcd, hcd_priv);
}

static void dump_hsic_regs(struct usb_hcd *hcd)
{
	int i;
	struct msm_hsic_hcd *mehci = hcd_to_hsic(hcd);

	if (atomic_read(&mehci->in_lpm))
		return;

	for (i = USB_REG_START_OFFSET; i <= USB_REG_END_OFFSET; i += 0x10)
		pr_info("%p: %08x\t%08x\t%08x\t%08x\n", hcd->regs + i,
				readl_relaxed(hcd->regs + i),
				readl_relaxed(hcd->regs + i + 4),
				readl_relaxed(hcd->regs + i + 8),
				readl_relaxed(hcd->regs + i + 0xc));
}

#define ULPI_IO_TIMEOUT_USEC	(10 * 1000)

#define USB_PHY_VDD_DIG_VOL_NONE	0 /*uV */
#define USB_PHY_VDD_DIG_VOL_MIN		1000000 /* uV */
#define USB_PHY_VDD_DIG_VOL_MAX		1320000 /* uV */

#define HSIC_DBG1_REG		0x38

static const int vdd_val[VDD_TYPE_MAX][VDD_VAL_MAX] = {
		{   /* VDD_CX CORNER Voting */
			[VDD_NONE]	= RPM_VREG_CORNER_NONE,
			[VDD_MIN]	= RPM_VREG_CORNER_NOMINAL,
			[VDD_MAX]	= RPM_VREG_CORNER_HIGH,
		},
		{   /* VDD_CX Voltage Voting */
			[VDD_NONE]	= USB_PHY_VDD_DIG_VOL_NONE,
			[VDD_MIN]	= USB_PHY_VDD_DIG_VOL_MIN,
			[VDD_MAX]	= USB_PHY_VDD_DIG_VOL_MAX,
		},
};

static int msm_hsic_init_vddcx(struct msm_hsic_hcd *mehci, int init)
{
	int ret = 0;
	int none_vol, min_vol, max_vol;

	if (!mehci->hsic_vddcx) {
		mehci->vdd_type = VDDCX_CORNER;
		mehci->hsic_vddcx = devm_regulator_get(mehci->dev,
			"hsic_vdd_dig");
		if (IS_ERR(mehci->hsic_vddcx)) {
			mehci->hsic_vddcx = devm_regulator_get(mehci->dev,
				"HSIC_VDDCX");
			if (IS_ERR(mehci->hsic_vddcx)) {
				dev_err(mehci->dev, "unable to get hsic vddcx\n");
				return PTR_ERR(mehci->hsic_vddcx);
			}
			mehci->vdd_type = VDDCX;
		}
	}

	none_vol = vdd_val[mehci->vdd_type][VDD_NONE];
	min_vol = vdd_val[mehci->vdd_type][VDD_MIN];
	max_vol = vdd_val[mehci->vdd_type][VDD_MAX];

	if (!init)
		goto disable_reg;

	ret = regulator_set_voltage(mehci->hsic_vddcx, min_vol, max_vol);
	if (ret) {
		dev_err(mehci->dev, "unable to set the voltage"
				"for hsic vddcx\n");
		return ret;
	}

	ret = regulator_enable(mehci->hsic_vddcx);
	if (ret) {
		dev_err(mehci->dev, "unable to enable hsic vddcx\n");
		goto reg_enable_err;
	}

	return 0;

disable_reg:
	regulator_disable(mehci->hsic_vddcx);
reg_enable_err:
	regulator_set_voltage(mehci->hsic_vddcx, none_vol, max_vol);

	return ret;

}

static int ulpi_read(struct msm_hsic_hcd *mehci, u32 reg)
{
	struct usb_hcd *hcd = hsic_to_hcd(mehci);
	unsigned long timeout;

	/* initiate read operation */
	writel_relaxed(ULPI_RUN | ULPI_READ | ULPI_ADDR(reg),
	       USB_ULPI_VIEWPORT);
#if 0
	/* wait for completion */
	timeout = jiffies + usecs_to_jiffies(ULPI_IO_TIMEOUT_USEC);
	while (readl_relaxed(USB_ULPI_VIEWPORT) & ULPI_RUN) {
		if (time_after(jiffies, timeout)) {
			dev_err(mehci->dev, "ulpi_read: timeout %08x\n",
				readl_relaxed(USB_ULPI_VIEWPORT));
			return -ETIMEDOUT;
		}
		udelay(1);
	}
#else
	timeout = 0 ;
	while( timeout++ < ULPI_IO_TIMEOUT_USEC ) {
	    if( !(readl_relaxed(USB_ULPI_VIEWPORT) & ULPI_RUN ) )
	        break ;
	    udelay(1) ;
	}

	if( timeout >= ULPI_IO_TIMEOUT_USEC ) {
	    dev_err(mehci->dev,"ulpi_read: timeout %08x\n",
			readl_relaxed(USB_ULPI_VIEWPORT));
	    if(system_state == SYSTEM_RUNNING)
		    subsystem_restart("external_modem");
	    return -ETIMEDOUT ;
	}
#endif
	return ULPI_DATA_READ(readl_relaxed(USB_ULPI_VIEWPORT));
}

static int ulpi_write(struct msm_hsic_hcd *mehci, u32 val, u32 reg)
{
	struct usb_hcd *hcd = hsic_to_hcd(mehci);
	int cnt = 0;

	/* initiate write operation */
	writel_relaxed(ULPI_RUN | ULPI_WRITE |
	       ULPI_ADDR(reg) | ULPI_DATA(val),
	       USB_ULPI_VIEWPORT);

	/* wait for completion */
	while (cnt < ULPI_IO_TIMEOUT_USEC) {
		if (!(readl_relaxed(USB_ULPI_VIEWPORT) & ULPI_RUN))
			break;
		udelay(1);
		cnt++;
	}

	if (cnt >= ULPI_IO_TIMEOUT_USEC) {
		dev_err(mehci->dev, "ulpi_write: timeout\n");
		return -ETIMEDOUT;
	}

	return 0;
}

#define HSIC_DBG1		0X38
#define ULPI_MANUAL_ENABLE	BIT(4)
#define ULPI_LINESTATE_DATA	BIT(5)
#define ULPI_LINESTATE_STROBE	BIT(6)
static void ehci_msm_enable_ulpi_control(struct usb_hcd *hcd, u32 linestate)
{
	struct msm_hsic_hcd *mehci = hcd_to_hsic(hcd);
	int val;

	switch (linestate) {
	case PORT_RESET:
		val = ulpi_read(mehci, HSIC_DBG1);
		val |= ULPI_MANUAL_ENABLE;
		val &= ~(ULPI_LINESTATE_DATA | ULPI_LINESTATE_STROBE);
		ulpi_write(mehci, val, HSIC_DBG1);
		break;
	default:
		pr_info("%s: Unknown linestate:%0x\n", __func__, linestate);
	}
}

static void ehci_msm_disable_ulpi_control(struct usb_hcd *hcd)
{
	struct msm_hsic_hcd *mehci = hcd_to_hsic(hcd);
	int val;

	val = ulpi_read(mehci, HSIC_DBG1);
	val &= ~ULPI_MANUAL_ENABLE;
	ulpi_write(mehci, val, HSIC_DBG1);
}

static int msm_hsic_config_gpios(struct msm_hsic_hcd *mehci, int gpio_en)
{
	int rc = 0;
	struct msm_hsic_host_platform_data *pdata;
	static int gpio_status;

	pdata = mehci->dev->platform_data;

	if (!pdata || !pdata->strobe || !pdata->data)
		return rc;

	if (gpio_status == gpio_en)
		return 0;

	gpio_status = gpio_en;

	if (!gpio_en)
		goto free_gpio;

	rc = gpio_request(pdata->strobe, "HSIC_STROBE_GPIO");
	if (rc < 0) {
		dev_err(mehci->dev, "gpio request failed for HSIC STROBE\n");
		return rc;
	}

	rc = gpio_request(pdata->data, "HSIC_DATA_GPIO");
	if (rc < 0) {
		dev_err(mehci->dev, "gpio request failed for HSIC DATA\n");
		goto free_strobe;
		}

	if (mehci->wakeup_gpio) {
		rc = gpio_request(mehci->wakeup_gpio, "HSIC_WAKEUP_GPIO");
		if (rc < 0) {
			dev_err(mehci->dev, "gpio request failed for HSIC WAKEUP\n");
			goto free_data;
		}
	}

	return 0;

free_gpio:
	if (mehci->wakeup_gpio)
		gpio_free(mehci->wakeup_gpio);
free_data:
	gpio_free(pdata->data);
free_strobe:
	gpio_free(pdata->strobe);

	return rc;
}
static int in_progress;

static void do_restart(struct work_struct *dummy)
{
	int normal_boot = 0;
        int on_pbl = 0;
	int err_fatal=0;

	normal_boot = gpio_get_value(__mehci->ready_gpio);
	on_pbl = gpio_get_value(__mehci->pbl_gpio);
	err_fatal = gpio_get_value(__mehci->err_fatal);
	if(normal_boot && !on_pbl && !in_progress && !err_fatal && system_state == SYSTEM_RUNNING){
        pr_err("%s: normal_boot(%d), on_pbl(%d), in_progress(%d), err_fatal(%d), system_state(%d)\n",
                __func__, normal_boot, on_pbl, in_progress, err_fatal, system_state);
		in_progress = 1;
		subsystem_restart("external_modem");
	}

}
DECLARE_DELAYED_WORK(restart_work,do_restart);
static int usbdev_notify(struct notifier_block *self,
			unsigned long action, void *priv)
{
	struct usb_device *udev = priv;
	struct usb_hcd *hcd;

	if(!__mehci)
		goto out;
	hcd = hsic_to_hcd(__mehci);
	if(&hcd->self != udev->bus)
		goto out;

	if (action == USB_BUS_ADD || action == USB_BUS_REMOVE || action == USB_DEVICE_CONFIG)
		goto out;

	if (!udev->parent || udev->parent->parent)
		goto out;

	switch (action) {
		case USB_DEVICE_ADD:
			in_progress = 0;
			break;
		/* fall through */
		case USB_DEVICE_REMOVE:
			schedule_delayed_work(&restart_work,0);
			break;
		default:
			break;
	}
out:
	return NOTIFY_OK;
}

static void msm_hsic_clk_reset(struct msm_hsic_hcd *mehci)
{
	int ret;

	ret = clk_reset(mehci->core_clk, CLK_RESET_ASSERT);
	if (ret) {
		dev_err(mehci->dev, "hsic clk assert failed:%d\n", ret);
		return;
	}
	clk_disable(mehci->core_clk);

	ret = clk_reset(mehci->core_clk, CLK_RESET_DEASSERT);
	if (ret)
		dev_err(mehci->dev, "hsic clk deassert failed:%d\n", ret);

	usleep_range(10000, 12000);

	clk_enable(mehci->core_clk);
}

#define HSIC_STROBE_GPIO_PAD_CTL	(MSM_TLMM_BASE+0x20C0)
#define HSIC_DATA_GPIO_PAD_CTL		(MSM_TLMM_BASE+0x20C4)
#define HSIC_CAL_PAD_CTL       (MSM_TLMM_BASE+0x20C8)
#define HSIC_LV_MODE		0x04
#define HSIC_PAD_CALIBRATION	0xA8
#define HSIC_GPIO_PAD_VAL	0x0A0AAA10
#define LINK_RESET_TIMEOUT_USEC		(250 * 1000)
static int msm_hsic_reset(struct msm_hsic_hcd *mehci)
{
	struct usb_hcd *hcd = hsic_to_hcd(mehci);
	int ret;
	struct msm_hsic_host_platform_data *pdata = mehci->dev->platform_data;

	msm_hsic_clk_reset(mehci);

	/* select ulpi phy */
	writel_relaxed(0x80000000, USB_PORTSC);

	mb();

	/* HSIC init sequence when HSIC signals (Strobe/Data) are
	routed via GPIOs */
	if (pdata && pdata->strobe && pdata->data) {

		/* Enable LV_MODE in HSIC_CAL_PAD_CTL register */
		writel_relaxed(HSIC_LV_MODE, HSIC_CAL_PAD_CTL);

		mb();

		/*set periodic calibration interval to ~2.048sec in
		  HSIC_IO_CAL_REG */
		ulpi_write(mehci, 0xFF, 0x33);

		/* Enable periodic IO calibration in HSIC_CFG register */
		ulpi_write(mehci, HSIC_PAD_CALIBRATION, 0x30);

		/* Configure GPIO pins for HSIC functionality mode */
		ret = msm_hsic_config_gpios(mehci, 1);
		if (ret) {
			dev_err(mehci->dev, " gpio configuarion failed\n");
			return ret;
		}
		/* Set LV_MODE=0x1 and DCC=0x2 in HSIC_GPIO PAD_CTL register */
		writel_relaxed(HSIC_GPIO_PAD_VAL, HSIC_STROBE_GPIO_PAD_CTL);
		writel_relaxed(HSIC_GPIO_PAD_VAL, HSIC_DATA_GPIO_PAD_CTL);

		mb();

		/* Enable HSIC mode in HSIC_CFG register */
		ulpi_write(mehci, 0x01, 0x31);
	} else {
		/* HSIC init sequence when HSIC signals (Strobe/Data) are routed
		via dedicated I/O */

		/* programmable length of connect signaling (33.2ns) */
		ret = ulpi_write(mehci, 3, HSIC_DBG1_REG);
		if (ret) {
			pr_err("%s: Unable to program length of connect "
			      "signaling\n", __func__);
		}

		/*set periodic calibration interval to ~2.048sec in
		  HSIC_IO_CAL_REG */
		ulpi_write(mehci, 0xFF, 0x33);

		/* Enable HSIC mode in HSIC_CFG register */
		ulpi_write(mehci, 0xA9, 0x30);
	}

	/*disable auto resume*/
	ulpi_write(mehci, ULPI_IFC_CTRL_AUTORESUME, ULPI_CLR(ULPI_IFC_CTRL));

	return 0;
}

#define PHY_SUSPEND_TIMEOUT_USEC	(500 * 1000)
#define PHY_RESUME_TIMEOUT_USEC		(100 * 1000)

#ifdef CONFIG_PM_SLEEP
static int msm_hsic_suspend(struct msm_hsic_hcd *mehci)
{
	struct usb_hcd *hcd = hsic_to_hcd(mehci);
	int cnt = 0, ret;
	u32 val;
	int none_vol, max_vol;

	if (atomic_read(&mehci->in_lpm)) {
		dev_dbg(mehci->dev, "%s called in lpm\n", __func__);
		return 0;
	}

	if (!(readl_relaxed(USB_PORTSC) & PORT_PE)) {
		dev_dbg(mehci->dev, "%s:port is not enabled skip suspend\n",
				__func__);
		return -EAGAIN;
	}

	disable_irq(hcd->irq);

	/* make sure we don't race against a remote wakeup */
	if (test_bit(HCD_FLAG_WAKEUP_PENDING, &hcd->flags) ||
	    readl_relaxed(USB_PORTSC) & PORT_RESUME) {
		dev_dbg(mehci->dev, "wakeup pending, aborting suspend\n");
		enable_irq(hcd->irq);
		return -EBUSY;
	}

	/*
	 * PHY may take some time or even fail to enter into low power
	 * mode (LPM). Hence poll for 500 msec and reset the PHY and link
	 * in failure case.
	 */
	val = readl_relaxed(USB_PORTSC);
	val &= ~PORT_RWC_BITS;
	val |= PORTSC_PHCD;
	writel_relaxed(val, USB_PORTSC);
	while (cnt < PHY_SUSPEND_TIMEOUT_USEC) {
		if (readl_relaxed(USB_PORTSC) & PORTSC_PHCD)
			break;
		udelay(1);
		cnt++;
	}

	if (cnt >= PHY_SUSPEND_TIMEOUT_USEC) {
		dev_err(mehci->dev, "Unable to suspend PHY\n");
		msm_hsic_config_gpios(mehci, 0);
		msm_hsic_reset(mehci);
	}

	/*
	 * PHY has capability to generate interrupt asynchronously in low
	 * power mode (LPM). This interrupt is level triggered. So USB IRQ
	 * line must be disabled till async interrupt enable bit is cleared
	 * in USBCMD register. Assert STP (ULPI interface STOP signal) to
	 * block data communication from PHY.
	 */
	writel_relaxed(readl_relaxed(USB_USBCMD) | ASYNC_INTR_CTRL |
				ULPI_STP_CTRL, USB_USBCMD);

	/*
	 * Ensure that hardware is put in low power mode before
	 * clocks are turned OFF and VDD is allowed to minimize.
	 */
	mb();

	clk_disable_unprepare(mehci->core_clk);
	clk_disable_unprepare(mehci->phy_clk);
	clk_disable_unprepare(mehci->cal_clk);
	clk_disable_unprepare(mehci->ahb_clk);

	none_vol = vdd_val[mehci->vdd_type][VDD_NONE];
	max_vol = vdd_val[mehci->vdd_type][VDD_MAX];

	ret = regulator_set_voltage(mehci->hsic_vddcx, none_vol, max_vol);
	if (ret < 0)
		dev_err(mehci->dev, "unable to set vddcx voltage for VDD MIN\n");

	if (mehci->bus_perf_client && debug_bus_voting_enabled) {
		ret = msm_bus_scale_client_update_request(
				mehci->bus_perf_client, 0);
		if (ret)
			dev_err(mehci->dev, "%s: Failed to dvote for "
				   "bus bandwidth %d\n", __func__, ret);
	}

	atomic_set(&mehci->in_lpm, 1);
	enable_irq(hcd->irq);

	mehci->wakeup_irq_enabled = 1;
	enable_irq_wake(mehci->wakeup_irq);
	enable_irq(mehci->wakeup_irq);

	wake_unlock(&mehci->wlock);

	dev_info(mehci->dev, "HSIC-USB in low power mode\n");

	return 0;
}

#ifdef HSIC_DISCONNECT_DEBUG
int jk = 0;
static void force_crash(void)
{
	struct msm_hsic_hcd *mehci = __mehci;
	struct usb_hcd *hcd = hsic_to_hcd(mehci);
	struct ehci_hcd *ehci = hcd_to_ehci(hcd);

	if (atomic_read(&mehci->in_lpm)) {
		printk( "%s called in !in_lpm\n", __func__);
	}
	disable_irq(hcd->irq);
	ehci_halt(ehci);
	pr_info("Force reset\n");
	msm_hsic_config_gpios(mehci, 0);
	msm_hsic_reset(mehci);
	enable_irq(hcd->irq);
	jk = 0;
}
#endif

static int msm_hsic_resume(struct msm_hsic_hcd *mehci)
{
	struct usb_hcd *hcd = hsic_to_hcd(mehci);
	int cnt = 0, ret;
	unsigned temp;
	int min_vol, max_vol;

	if (!atomic_read(&mehci->in_lpm)) {
		dev_dbg(mehci->dev, "%s called in !in_lpm\n", __func__);
		return 0;
	}

	if (mehci->wakeup_irq_enabled) {
		disable_irq_wake(mehci->wakeup_irq);
		disable_irq_nosync(mehci->wakeup_irq);
		mehci->wakeup_irq_enabled = 0;
	}

	wake_lock(&mehci->wlock);

	if (mehci->bus_perf_client && debug_bus_voting_enabled) {
		ret = msm_bus_scale_client_update_request(
				mehci->bus_perf_client, 1);
		if (ret)
			dev_err(mehci->dev, "%s: Failed to vote for "
				   "bus bandwidth %d\n", __func__, ret);
	}

	min_vol = vdd_val[mehci->vdd_type][VDD_MIN];
	max_vol = vdd_val[mehci->vdd_type][VDD_MAX];

	ret = regulator_set_voltage(mehci->hsic_vddcx, min_vol, max_vol);
	if (ret < 0)
		dev_err(mehci->dev, "unable to set nominal vddcx voltage (no VDD MIN)\n");

	clk_prepare_enable(mehci->core_clk);
	clk_prepare_enable(mehci->phy_clk);
	clk_prepare_enable(mehci->cal_clk);
	clk_prepare_enable(mehci->ahb_clk);

	temp = readl_relaxed(USB_USBCMD);
	temp &= ~ASYNC_INTR_CTRL;
	temp &= ~ULPI_STP_CTRL;
	writel_relaxed(temp, USB_USBCMD);

	dbg_log_event(NULL, "Set RS", readl_relaxed(USB_USBCMD),"", 0, "", 0);

	if (!(readl_relaxed(USB_PORTSC) & PORTSC_PHCD))
		goto skip_phy_resume;

	temp = readl_relaxed(USB_PORTSC);
	temp &= ~(PORT_RWC_BITS | PORTSC_PHCD);
	writel_relaxed(temp, USB_PORTSC);
	while (cnt < PHY_RESUME_TIMEOUT_USEC) {
		if (!(readl_relaxed(USB_PORTSC) & PORTSC_PHCD) &&
			(readl_relaxed(USB_ULPI_VIEWPORT) & ULPI_SYNC_STATE))
			break;
		udelay(1);
		cnt++;
	}

	if (cnt >= PHY_RESUME_TIMEOUT_USEC) {
		/*
		 * This is a fatal error. Reset the link and
		 * PHY to make hsic working.
		 */
		dev_err(mehci->dev, "Unable to resume USB. Reset the hsic\n");
		msm_hsic_config_gpios(mehci, 0);
		msm_hsic_reset(mehci);
	}

skip_phy_resume:

	usb_hcd_resume_root_hub(hcd);

	atomic_set(&mehci->in_lpm, 0);

	if (mehci->async_int) {
		mehci->async_int = false;
		pm_runtime_put_noidle(mehci->dev);
		enable_irq(hcd->irq);
	}

	if (atomic_read(&mehci->pm_usage_cnt)) {
		atomic_set(&mehci->pm_usage_cnt, 0);
		pm_runtime_put_noidle(mehci->dev);
	}

	dev_info(mehci->dev, "HSIC-USB exited from low power mode\n");

#ifdef HSIC_DISCONNECT_DEBUG
	jk++;
	if(jk == 10)
		force_crash();
#endif

	return 0;
}
#endif

static irqreturn_t msm_hsic_irq(struct usb_hcd *hcd)
{
	struct msm_hsic_hcd *mehci = hcd_to_hsic(hcd);

	if (atomic_read(&mehci->in_lpm)) {
		disable_irq_nosync(hcd->irq);
		dev_dbg(mehci->dev, "phy async intr\n");
		mehci->async_int = true;
		pm_runtime_get(mehci->dev);
		return IRQ_HANDLED;
	}

	return ehci_irq(hcd);
}

static int ehci_hsic_reset(struct usb_hcd *hcd)
{
	struct ehci_hcd *ehci = hcd_to_ehci(hcd);
	struct msm_hsic_hcd *mehci = hcd_to_hsic(hcd);
	int retval;

	mehci->timer = USB_HS_GPTIMER_BASE;
	ehci->caps = USB_CAPLENGTH;
	ehci->regs = USB_CAPLENGTH +
		HC_LENGTH(ehci, ehci_readl(ehci, &ehci->caps->hc_capbase));
	dbg_hcs_params(ehci, "reset");
	dbg_hcc_params(ehci, "reset");

	/* cache the data to minimize the chip reads*/
	ehci->hcs_params = ehci_readl(ehci, &ehci->caps->hcs_params);

	hcd->has_tt = 1;
	ehci->sbrn = HCD_USB2;

	retval = ehci_halt(ehci);
	if (retval)
		return retval;

	/* data structure init */
	retval = ehci_init(hcd);
	if (retval)
		return retval;

	retval = ehci_reset(ehci);
	if (retval)
		return retval;

	/* bursts of unspecified length. */
	writel_relaxed(0, USB_AHBBURST);
	/* Use the AHB transactor */
	writel_relaxed(0x08, USB_AHBMODE);
	/* Disable streaming mode and select host mode */
	writel_relaxed(0x13, USB_USBMODE);

	ehci_port_power(ehci, 1);
	return 0;
}

static int ehci_hsic_urb_enqueue(struct usb_hcd *hcd, struct urb *urb,
		gfp_t mem_flags)
{
	dbg_log_event(urb, event_to_str(URB_SUBMIT), 0,"", 0, "", 0);
	return ehci_urb_enqueue(hcd, urb, mem_flags);
}

static int ehci_hsic_bus_suspend(struct usb_hcd *hcd)
{
	dbg_log_event(NULL, "Suspend RH", 0,"", 0, "", 0);
	return ehci_bus_suspend(hcd);
}

#define RESUME_SIGNAL_TIME_MS	(22*999)
#define RESUME_RETRY_LIMIT 20
static void ehci_hsic_bus_resume_w(struct work_struct *w) 
{
	struct msm_hsic_hcd *mehci =
	container_of(w, struct msm_hsic_hcd, resume_w);
	struct usb_hcd *hcd = hsic_to_hcd(mehci);
	struct ehci_hcd		*ehci = hcd_to_ehci(hcd);
	u32			temp;
	unsigned long		resume_needed = 0;
	int			retry_cnt = 0;
		int 		tight_resume = 0;
	int 		resume_poll_duration;

	/* keep delay between bus states */
	if (time_before(jiffies, ehci->next_statechange))
		usleep_range(5000, 5000);
	
	spin_lock_irq(&ehci->lock);
	if (!HCD_HW_ACCESSIBLE(hcd)) {
		spin_unlock_irq(&ehci->lock);
		mehci->resume_status = -ESHUTDOWN;
		wake_up(&mehci->resume_wait);
		return;
	}
	
	if (unlikely(ehci->debug)) {
		if (!dbgp_reset_prep())
			ehci->debug = NULL;
		else
			dbgp_external_startup();
	}

	/* at least some APM implementations will try to deliver
	* IRQs right away, so delay them until we're ready.
	*/
	ehci_writel(ehci, 0, &ehci->regs->intr_enable);

	/* re-init operational registers */
	ehci_writel(ehci, 0, &ehci->regs->segment);
	ehci_writel(ehci, ehci->periodic_dma, &ehci->regs->frame_list);
	ehci_writel(ehci, (u32) ehci->async->qh_dma, &ehci->regs->async_next);

	/*CMD_RUN will be set after, PORT_RESUME gets cleared*/
	if (ehci->resume_sof_bug)
		ehci->command &= ~CMD_RUN;

	/* restore CMD_RUN, framelist size, and irq threshold */
	ehci_writel(ehci, ehci->command, &ehci->regs->command);

	/* manually resume the ports we suspended during bus_suspend() */
resume_again:
	if (retry_cnt >= RESUME_RETRY_LIMIT) {
		pr_info("retry count(%d) reached max try resume in tight loop\n", retry_cnt);
		tight_resume = 1;

	}


	temp = ehci_readl(ehci, &ehci->regs->port_status[0]);
	temp &= ~(PORT_RWC_BITS | PORT_WAKE_BITS);
	if (test_bit(0, &ehci->bus_suspended) && (temp & PORT_SUSPEND)) {
		temp |= PORT_RESUME;
		set_bit(0, &resume_needed);
	}

	dbg_log_event(NULL, "FPR: Set", temp,"",0,"", 0);
	ehci_writel(ehci, temp, &ehci->regs->port_status[0]);

	/* HSIC controller has a h/w bug due to which it can try to send SOFs
	 * (start of frames) during port resume resulting in phy lockup. HSIC hw
	 * controller in MSM clears FPR bit after driving the resume signal for
	 * 20ms. Workaround is to stop SOFs before driving resume and then start
	 * sending SOFs immediately. Need to send SOFs within 3ms of resume
	 * completion otherwise peripheral may enter undefined state. As
	 * usleep_range does not gurantee exact sleep time, GPTimer is used to
	 * to time the resume sequence. If driver exceeds allowable time SOFs,
	 * repeat the resume process.
	 */
	if (ehci->resume_sof_bug && resume_needed) {
		int timeleft;

		ehci_writel(ehci, GPT_LD(RESUME_SIGNAL_TIME_MS),
			&mehci->timer->gptimer0_ld);
		ehci_writel(ehci, GPT_RESET | GPT_RUN,
			&mehci->timer->gptimer0_ctrl);

	
		if (!tight_resume) {
			spin_unlock_irq(&ehci->lock);
			usleep_range(18000, 20000);
			spin_lock_irq(&ehci->lock);

			/* resume duration in ms, resume signalling can go
			 * upto 21ms, in worst case usleep might return right
			 * away after lower limit, 18ms. Cannot afford one
			 * more usleep, so Poll for FPR for rest of 3ms
			 */
			resume_poll_duration = 3 * 1000;
		} else {
			/* resume duration in ms, in tight loop poll for FPR
			 * for 21ms below
			 */
			resume_poll_duration = 21 * 1000;
		}
	

		timeleft = GPT_CNT(ehci_readl(ehci,
			&mehci->timer->gptimer0_ctrl));
		dbg_log_event(NULL, "FPR: timecheck", timeleft,"",0,"", 0);
		if (!timeleft) {
			int temp;
			dbg_log_event(NULL, "FPR: Re-Resume", retry_cnt,"",0,"", 0);
			pr_info("FPR: retry count: %d\n", retry_cnt);
			spin_unlock_irq(&ehci->lock);
			temp = ehci_readl(ehci, &ehci->regs->port_status[0]);
			temp &= ~PORT_RWC_BITS;
			temp |= PORT_SUSPEND;
			ehci_writel(ehci, temp, &ehci->regs->port_status[0]);
			/* Keep the bus idle for some time so that peripheral
			 * can detect and initiate suspend
			 */
			usleep_range(5000, 5000);
			
			dbg_log_event(NULL,
				"FPR: RResume",
				ehci_readl(ehci, &ehci->regs->port_status[0]),"",0,"", 0);
			spin_lock_irq(&ehci->lock);

			retry_cnt++;
			goto resume_again;
		}

		dbg_log_event(NULL, "FPR: B. Handshake",0,"",0,"", 0);
		handshake(ehci, &ehci->regs->port_status[0],
			PORT_RESUME, 0, resume_poll_duration);
		dbg_log_event(NULL, "FPR: A. Handshake",0,"",0,"", 0);
		ehci_writel(ehci, ehci_readl(ehci,
			&ehci->regs->command) | CMD_RUN,
			&ehci->regs->command);
		dbg_log_event(NULL, "FPR: SET RS BIT",0,"",0,"", 0);
	}

	mehci->resume_status = 1;
	wake_up(&mehci->resume_wait);
	spin_unlock_irq(&ehci->lock);
}

static int ehci_hsic_bus_resume(struct usb_hcd *hcd)
{
	struct msm_hsic_hcd *mehci = hcd_to_hsic(hcd);
	struct ehci_hcd		*ehci = hcd_to_ehci(hcd);
	u32			temp;
	int			ret;

	dbg_log_event(NULL, "Resume RH", 0,"", 0, "", 0);
	
	mehci->resume_status = 0;
	queue_work(ehci_wq, &mehci->resume_w);

	ret = wait_event_interruptible(mehci->resume_wait,
				mehci->resume_status);

	if (ret) {
		pr_err("%s: wait event interrupt failed:%d\n", __func__, ret);
		return ret;
	}

	if (mehci->resume_status < 0)
		return mehci->resume_status;

	dbg_log_event(NULL, "FPR: Wokeup",0,"",0,"", 0);
	spin_lock_irq(&ehci->lock);
	(void) ehci_readl(ehci, &ehci->regs->command);

	temp = 0;
	if (ehci->async->qh_next.qh)
		temp |= CMD_ASE;
	if (ehci->periodic_sched)
		temp |= CMD_PSE;
	if (temp) {
		ehci->command |= temp;
		ehci_writel(ehci, ehci->command, &ehci->regs->command);
	}

	ehci->next_statechange = jiffies + msecs_to_jiffies(5);
	hcd->state = HC_STATE_RUNNING;

	/* Now we can safely re-enable irqs */
	ehci_writel(ehci, INTR_MASK, &ehci->regs->intr_enable);
	spin_unlock_irq(&ehci->lock);
	dbg_log_event(NULL, "FPR: B.comapanion",0,"",0,"", 0);

	ehci_handover_companion_ports(ehci);
	dbg_log_event(NULL, "FPR: A.comapanion",0,"",0,"", 0);

	return 0; 
}

static struct notifier_block usbdev_nb = {
	.notifier_call =	usbdev_notify,
};

static struct hc_driver msm_hsic_driver = {
	.description		= hcd_name,
	.product_desc		= "Qualcomm EHCI Host Controller using HSIC",
	.hcd_priv_size		= sizeof(struct msm_hsic_hcd),

	/*
	 * generic hardware linkage
	 */
	.irq			= msm_hsic_irq,
	.flags			= HCD_USB2 | HCD_MEMORY | HCD_OLD_ENUM,

	.reset			= ehci_hsic_reset,
	.start			= ehci_run,

	.stop			= ehci_stop,
	.shutdown		= ehci_shutdown,

	/*
	 * managing i/o requests and associated device resources
	 */
	.urb_enqueue		= ehci_hsic_urb_enqueue,
	.urb_dequeue		= ehci_urb_dequeue,
	.endpoint_disable	= ehci_endpoint_disable,
	.endpoint_reset		= ehci_endpoint_reset,
	.clear_tt_buffer_complete	 = ehci_clear_tt_buffer_complete,

	/*
	 * scheduling support
	 */
	.get_frame_number	= ehci_get_frame,

	/*
	 * root hub support
	 */
	.hub_status_data	= ehci_hub_status_data,
	.hub_control		= ehci_hub_control,
	.relinquish_port	= ehci_relinquish_port,
	.port_handed_over	= ehci_port_handed_over,

	/*
	 * PM support
	 */
	.bus_suspend		= ehci_hsic_bus_suspend,
	.bus_resume		= ehci_hsic_bus_resume,

	.log_urb_complete	= dbg_log_event,
	.dump_regs		= dump_hsic_regs,

	.enable_ulpi_control	= ehci_msm_enable_ulpi_control,
	.disable_ulpi_control	= ehci_msm_disable_ulpi_control,
};

static int msm_hsic_init_clocks(struct msm_hsic_hcd *mehci, u32 init)
{
	int ret = 0;

	if (!init)
		goto put_clocks;

	/*core_clk is required for LINK protocol engine
	 *clock rate appropriately set by target specific clock driver */
	mehci->core_clk = clk_get(mehci->dev, "core_clk");
	if (IS_ERR(mehci->core_clk)) {
		dev_err(mehci->dev, "failed to get core_clk\n");
		ret = PTR_ERR(mehci->core_clk);
		return ret;
	}

	/* alt_core_clk is for LINK to be used during PHY RESET
	 * clock rate appropriately set by target specific clock driver */
	mehci->alt_core_clk = clk_get(mehci->dev, "alt_core_clk");
	if (IS_ERR(mehci->alt_core_clk)) {
		dev_err(mehci->dev, "failed to core_clk\n");
		ret = PTR_ERR(mehci->alt_core_clk);
		goto put_core_clk;
	}

	/* phy_clk is required for HSIC PHY operation
	 * clock rate appropriately set by target specific clock driver */
	mehci->phy_clk = clk_get(mehci->dev, "phy_clk");
	if (IS_ERR(mehci->phy_clk)) {
		dev_err(mehci->dev, "failed to get phy_clk\n");
		ret = PTR_ERR(mehci->phy_clk);
		goto put_alt_core_clk;
	}

	/* 10MHz cal_clk is required for calibration of I/O pads */
	mehci->cal_clk = clk_get(mehci->dev, "cal_clk");
	if (IS_ERR(mehci->cal_clk)) {
		dev_err(mehci->dev, "failed to get cal_clk\n");
		ret = PTR_ERR(mehci->cal_clk);
		goto put_phy_clk;
	}
	clk_set_rate(mehci->cal_clk, 10000000);

	/* ahb_clk is required for data transfers */
	mehci->ahb_clk = clk_get(mehci->dev, "iface_clk");
	if (IS_ERR(mehci->ahb_clk)) {
		dev_err(mehci->dev, "failed to get iface_clk\n");
		ret = PTR_ERR(mehci->ahb_clk);
		goto put_cal_clk;
	}

	clk_prepare_enable(mehci->core_clk);
	clk_prepare_enable(mehci->phy_clk);
	clk_prepare_enable(mehci->cal_clk);
	clk_prepare_enable(mehci->ahb_clk);

	return 0;

put_clocks:
	if (!atomic_read(&mehci->in_lpm)) {
		clk_disable_unprepare(mehci->core_clk);
		clk_disable_unprepare(mehci->phy_clk);
		clk_disable_unprepare(mehci->cal_clk);
		clk_disable_unprepare(mehci->ahb_clk);
	}
	clk_put(mehci->ahb_clk);
put_cal_clk:
	clk_put(mehci->cal_clk);
put_phy_clk:
	clk_put(mehci->phy_clk);
put_alt_core_clk:
	clk_put(mehci->alt_core_clk);
put_core_clk:
	clk_put(mehci->core_clk);

	return ret;
}
static irqreturn_t hsic_peripheral_status_change(int irq, void *dev_id)
{
	struct msm_hsic_hcd *mehci = dev_id;

	pr_debug("%s: mehci:%p dev_id:%p\n", __func__, mehci, dev_id);

	if (mehci)
		msm_hsic_config_gpios(mehci, 0);

	return IRQ_HANDLED;
}

static irqreturn_t msm_hsic_wakeup_irq(int irq, void *data)
{
	struct msm_hsic_hcd *mehci = data;

	mehci->wakeup_int_cnt++;
	dbg_log_event(NULL, "Remote Wakeup IRQ", mehci->wakeup_int_cnt,"", 0, "", 0);
	dev_dbg(mehci->dev, "%s: hsic remote wakeup interrupt cnt: %u\n",
			__func__, mehci->wakeup_int_cnt);

	wake_lock(&mehci->wlock);

	if (mehci->wakeup_irq_enabled) {
		mehci->wakeup_irq_enabled = 0;
		disable_irq_wake(irq);
		disable_irq_nosync(irq);
	}

	if (!atomic_read(&mehci->pm_usage_cnt)) {
		atomic_set(&mehci->pm_usage_cnt, 1);
		pm_runtime_get(mehci->dev);
	}

	return IRQ_HANDLED;
}

static int ehci_hsic_msm_bus_show(struct seq_file *s, void *unused)
{
	if (debug_bus_voting_enabled)
		seq_printf(s, "enabled\n");
	else
		seq_printf(s, "disabled\n");

	return 0;
}

static int ehci_hsic_msm_bus_open(struct inode *inode, struct file *file)
{
	return single_open(file, ehci_hsic_msm_bus_show, inode->i_private);
}

static ssize_t ehci_hsic_msm_bus_write(struct file *file,
	const char __user *ubuf, size_t count, loff_t *ppos)
{
	char buf[8];
	int ret;
	struct seq_file *s = file->private_data;
	struct msm_hsic_hcd *mehci = s->private;

	memset(buf, 0x00, sizeof(buf));

	if (copy_from_user(&buf, ubuf, min_t(size_t, sizeof(buf) - 1, count)))
		return -EFAULT;

	if (!strncmp(buf, "enable", 6)) {
		/* Do not vote here. Let hsic driver decide when to vote */
		debug_bus_voting_enabled = true;
	} else {
		debug_bus_voting_enabled = false;
		if (mehci->bus_perf_client) {
			ret = msm_bus_scale_client_update_request(
					mehci->bus_perf_client, 0);
			if (ret)
				dev_err(mehci->dev, "%s: Failed to devote "
					   "for bus bw %d\n", __func__, ret);
		}
	}

	return count;
}

const struct file_operations ehci_hsic_msm_bus_fops = {
	.open = ehci_hsic_msm_bus_open,
	.read = seq_read,
	.write = ehci_hsic_msm_bus_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static int ehci_hsic_msm_wakeup_cnt_show(struct seq_file *s, void *unused)
{
	struct msm_hsic_hcd *mehci = s->private;

	seq_printf(s, "%u\n", mehci->wakeup_int_cnt);

	return 0;
}

static int ehci_hsic_msm_wakeup_cnt_open(struct inode *inode, struct file *f)
{
	return single_open(f, ehci_hsic_msm_wakeup_cnt_show, inode->i_private);
}

const struct file_operations ehci_hsic_msm_wakeup_cnt_fops = {
	.open = ehci_hsic_msm_wakeup_cnt_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int ehci_hsic_msm_data_events_show(struct seq_file *s, void *unused)
{
	unsigned long	flags;
	unsigned	i;

	read_lock_irqsave(&dbg_hsic_data.lck, flags);

	i = dbg_hsic_data.idx;
	for (dbg_inc(&i); i != dbg_hsic_data.idx; dbg_inc(&i)) {
		if (!strnlen(dbg_hsic_data.buf[i], DBG_MSG_LEN))
			continue;
		seq_printf(s, "%s\n", dbg_hsic_data.buf[i]);
	}

	read_unlock_irqrestore(&dbg_hsic_data.lck, flags);

	return 0;
}

static int ehci_hsic_msm_data_events_open(struct inode *inode, struct file *f)
{
	return single_open(f, ehci_hsic_msm_data_events_show, inode->i_private);
}

const struct file_operations ehci_hsic_msm_dbg_data_fops = {
	.open = ehci_hsic_msm_data_events_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int ehci_hsic_msm_ctrl_events_show(struct seq_file *s, void *unused)
{
	unsigned long	flags;
	unsigned	i;

	read_lock_irqsave(&dbg_hsic_ctrl.lck, flags);

	i = dbg_hsic_ctrl.idx;
	for (dbg_inc(&i); i != dbg_hsic_ctrl.idx; dbg_inc(&i)) {
		if (!strnlen(dbg_hsic_ctrl.buf[i], DBG_MSG_LEN))
			continue;
		seq_printf(s, "%s\n", dbg_hsic_ctrl.buf[i]);
	}

	read_unlock_irqrestore(&dbg_hsic_ctrl.lck, flags);

	return 0;
}

static int ehci_hsic_msm_ctrl_events_open(struct inode *inode, struct file *f)
{
	return single_open(f, ehci_hsic_msm_ctrl_events_show, inode->i_private);
}

const struct file_operations ehci_hsic_msm_dbg_ctrl_fops = {
	.open = ehci_hsic_msm_ctrl_events_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static struct dentry *ehci_hsic_msm_dbg_root;
static int ehci_hsic_msm_debugfs_init(struct msm_hsic_hcd *mehci)
{
	struct dentry *ehci_hsic_msm_dentry;

	ehci_hsic_msm_dbg_root = debugfs_create_dir("ehci_hsic_msm_dbg", NULL);

	if (!ehci_hsic_msm_dbg_root || IS_ERR(ehci_hsic_msm_dbg_root))
		return -ENODEV;

	ehci_hsic_msm_dentry = debugfs_create_file("bus_voting",
		S_IRUGO | S_IWUSR,
		ehci_hsic_msm_dbg_root, mehci,
		&ehci_hsic_msm_bus_fops);

	if (!ehci_hsic_msm_dentry) {
		debugfs_remove_recursive(ehci_hsic_msm_dbg_root);
		return -ENODEV;
	}

	ehci_hsic_msm_dentry = debugfs_create_file("wakeup_cnt",
		S_IRUGO,
		ehci_hsic_msm_dbg_root, mehci,
		&ehci_hsic_msm_wakeup_cnt_fops);

	if (!ehci_hsic_msm_dentry) {
		debugfs_remove_recursive(ehci_hsic_msm_dbg_root);
		return -ENODEV;
	}

	ehci_hsic_msm_dentry = debugfs_create_file("show_ctrl_events",
		S_IRUGO,
		ehci_hsic_msm_dbg_root, mehci,
		&ehci_hsic_msm_dbg_ctrl_fops);

	if (!ehci_hsic_msm_dentry) {
		debugfs_remove_recursive(ehci_hsic_msm_dbg_root);
		return -ENODEV;
	}

	ehci_hsic_msm_dentry = debugfs_create_file("show_data_events",
		S_IRUGO,
		ehci_hsic_msm_dbg_root, mehci,
		&ehci_hsic_msm_dbg_data_fops);

	if (!ehci_hsic_msm_dentry) {
		debugfs_remove_recursive(ehci_hsic_msm_dbg_root);
		return -ENODEV;
	}

	return 0;
}

static void ehci_hsic_msm_debugfs_cleanup(void)
{
	debugfs_remove_recursive(ehci_hsic_msm_dbg_root);
}

static int __devinit ehci_hsic_msm_probe(struct platform_device *pdev)
{
	struct usb_hcd *hcd;
	struct resource *res;
	struct msm_hsic_hcd *mehci;
	struct msm_hsic_host_platform_data *pdata;
	int ret;

	dev_dbg(&pdev->dev, "ehci_msm-hsic probe\n");

	/* After parent device's probe is executed, it will be put in suspend
	 * mode. When child device's probe is called, driver core is not
	 * resuming parent device due to which parent will be in suspend even
	 * though child is active. Hence resume the parent device explicitly.
	 */
	if (pdev->dev.parent)
		pm_runtime_get_sync(pdev->dev.parent);

	hcd = usb_create_hcd(&msm_hsic_driver, &pdev->dev,
				dev_name(&pdev->dev));
	if (!hcd) {
		dev_err(&pdev->dev, "Unable to create HCD\n");
		return  -ENOMEM;
	}

	hcd->irq = platform_get_irq(pdev, 0);
	if (hcd->irq < 0) {
		dev_err(&pdev->dev, "Unable to get IRQ resource\n");
		ret = hcd->irq;
		goto put_hcd;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "Unable to get memory resource\n");
		ret = -ENODEV;
		goto put_hcd;
	}

	hcd->rsrc_start = res->start;
	hcd->rsrc_len = resource_size(res);
	hcd->regs = ioremap(hcd->rsrc_start, hcd->rsrc_len);
	if (!hcd->regs) {
		dev_err(&pdev->dev, "ioremap failed\n");
		ret = -ENOMEM;
		goto put_hcd;
	}

	mehci = hcd_to_hsic(hcd);
	mehci->dev = &pdev->dev;

	mehci->ehci.susp_sof_bug = 1;
	mehci->ehci.reset_sof_bug = 1;

	mehci->ehci.resume_sof_bug = 1;

	mehci->ehci.max_log2_irq_thresh = 4;

	res = platform_get_resource_byname(pdev,
			IORESOURCE_IRQ,
			"peripheral_status_irq");
	if (res)
		mehci->peripheral_status_irq = res->start;

	res = platform_get_resource_byname(pdev, IORESOURCE_IO, "wakeup");
	if (res) {
		mehci->wakeup_gpio = res->start;
		mehci->wakeup_irq = MSM_GPIO_TO_INT(res->start);
		dev_dbg(mehci->dev, "wakeup_irq: %d\n", mehci->wakeup_irq);
	}

	res = platform_get_resource_byname(pdev,
			IORESOURCE_IO, "MDM2AP_PBLRDY");
	if (res) {
	    dev_dbg(mehci->dev, "pblrdy: %d\n", res->start);
	    mehci->pbl_gpio = res->start;
	}

	res = platform_get_resource_byname(pdev,
			IORESOURCE_IO, "AP2MDM_HSICRDY");
	if (res) {
		dev_dbg(mehci->dev, "AP2MDM_HSICRDY: %d\n", res->start);
		mehci->ready_gpio = res->start;
	}
	res = platform_get_resource_byname(pdev,
			IORESOURCE_IO, "MDM2AP_ERRFATAL");
	if (res) {
		dev_dbg(mehci->dev, "MDM2AP_ERRFATAL: %d\n", res->start);
		mehci->err_fatal = res->start;
	}
	ret = msm_hsic_init_clocks(mehci, 1);
	if (ret) {
		dev_err(&pdev->dev, "unable to initialize clocks\n");
		ret = -ENODEV;
		goto unmap;
	}

	ret = msm_hsic_init_vddcx(mehci, 1);
	if (ret) {
		dev_err(&pdev->dev, "unable to initialize VDDCX\n");
		ret = -ENODEV;
		goto deinit_clocks;
	}

	ret = msm_hsic_reset(mehci);
	if (ret) {
		dev_err(&pdev->dev, "unable to initialize PHY\n");
		goto deinit_vddcx;
	}

	ehci_wq = create_singlethread_workqueue("ehci_wq");
	if (!ehci_wq) {
		dev_err(&pdev->dev, "unable to create workqueue\n");
		ret = -ENOMEM;
		goto deinit_vddcx;
	}

	INIT_WORK(&mehci->resume_w, ehci_hsic_bus_resume_w);
	init_waitqueue_head(&mehci->resume_wait);
	usb_register_notify(&usbdev_nb);
	ret = usb_add_hcd(hcd, hcd->irq, IRQF_SHARED);
	if (ret) {
		dev_err(&pdev->dev, "unable to register HCD\n");
		goto unconfig_gpio;
	}

	device_init_wakeup(&pdev->dev, 1);
	wake_lock_init(&mehci->wlock, WAKE_LOCK_SUSPEND, dev_name(&pdev->dev));
	wake_lock(&mehci->wlock);

	if (mehci->peripheral_status_irq) {
		ret = request_threaded_irq(mehci->peripheral_status_irq,
			NULL, hsic_peripheral_status_change,
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING
						| IRQF_SHARED,
			"hsic_peripheral_status", mehci);
		if (ret)
			dev_err(&pdev->dev, "%s:request_irq:%d failed:%d",
				__func__, mehci->peripheral_status_irq, ret);
	}

	/* configure wakeup irq */
	if (mehci->wakeup_irq) {
		ret = request_irq(mehci->wakeup_irq, msm_hsic_wakeup_irq,
				IRQF_TRIGGER_HIGH,
				"msm_hsic_wakeup", mehci);
		if (!ret) {
			disable_irq_nosync(mehci->wakeup_irq);
		} else {
			dev_err(&pdev->dev, "request_irq(%d) failed: %d\n",
					mehci->wakeup_irq, ret);
			mehci->wakeup_irq = 0;
		}
	}

	ret = ehci_hsic_msm_debugfs_init(mehci);
	if (ret)
		dev_dbg(&pdev->dev, "mode debugfs file is"
			"not available\n");

	pdata = mehci->dev->platform_data;
	if (pdata && pdata->bus_scale_table) {
		mehci->bus_perf_client =
		    msm_bus_scale_register_client(pdata->bus_scale_table);
		/* Configure BUS performance parameters for MAX bandwidth */
		if (mehci->bus_perf_client) {
			ret = msm_bus_scale_client_update_request(
					mehci->bus_perf_client, 1);
			if (ret)
				dev_err(&pdev->dev, "%s: Failed to vote for "
					   "bus bandwidth %d\n", __func__, ret);
		} else {
			dev_err(&pdev->dev, "%s: Failed to register BUS "
						"scaling client!!\n", __func__);
		}
	}

	__mehci = mehci;

	/*
	 * This pdev->dev is assigned parent of root-hub by USB core,
	 * hence, runtime framework automatically calls this driver's
	 * runtime APIs based on root-hub's state.
	 */
	pm_runtime_set_active(&pdev->dev);
	pm_runtime_enable(&pdev->dev);
	/* Decrement the parent device's counter after probe.
	 * As child is active, parent will not be put into
	 * suspend mode.
	 */
	if (pdev->dev.parent)
		pm_runtime_put_sync(pdev->dev.parent);

	return 0;

unconfig_gpio:
	destroy_workqueue(ehci_wq);
	msm_hsic_config_gpios(mehci, 0);
deinit_vddcx:
	msm_hsic_init_vddcx(mehci, 0);
deinit_clocks:
	msm_hsic_init_clocks(mehci, 0);
unmap:
	iounmap(hcd->regs);
put_hcd:
	usb_put_hcd(hcd);

	return ret;
}

static int __devexit ehci_hsic_msm_remove(struct platform_device *pdev)
{
	struct usb_hcd *hcd = platform_get_drvdata(pdev);
	struct msm_hsic_hcd *mehci = hcd_to_hsic(hcd);

	if (mehci->peripheral_status_irq)
		free_irq(mehci->peripheral_status_irq, mehci);

	if (mehci->wakeup_irq) {
		if (mehci->wakeup_irq_enabled)
			disable_irq_wake(mehci->wakeup_irq);
		free_irq(mehci->wakeup_irq, mehci);
	}

	if (mehci->bus_perf_client)
		msm_bus_scale_unregister_client(mehci->bus_perf_client);

	ehci_hsic_msm_debugfs_cleanup();
	device_init_wakeup(&pdev->dev, 0);
	pm_runtime_set_suspended(&pdev->dev);
	
	destroy_workqueue(ehci_wq);

	usb_remove_hcd(hcd);
	usb_unregister_notify(&usbdev_nb);
	msm_hsic_config_gpios(mehci, 0);
	msm_hsic_init_vddcx(mehci, 0);

	msm_hsic_init_clocks(mehci, 0);
	wake_lock_destroy(&mehci->wlock);
	iounmap(hcd->regs);
	usb_put_hcd(hcd);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int msm_hsic_pm_suspend(struct device *dev)
{
	int ret;
	struct usb_hcd *hcd = dev_get_drvdata(dev);
	struct msm_hsic_hcd *mehci = hcd_to_hsic(hcd);

	dev_dbg(dev, "ehci-msm-hsic PM suspend\n");

	dbg_log_event(NULL, "PM Suspend", 0,"", 0, "", 0);

	if (device_may_wakeup(dev))
		enable_irq_wake(hcd->irq);

	ret = msm_hsic_suspend(mehci);

	if (ret && device_may_wakeup(dev))
		disable_irq_wake(hcd->irq);

	return ret;
}

static int msm_hsic_pm_resume(struct device *dev)
{
	int ret;
	struct usb_hcd *hcd = dev_get_drvdata(dev);
	struct msm_hsic_hcd *mehci = hcd_to_hsic(hcd);

	dbg_log_event(NULL, "PM Resume", 0,"", 0, "", 0);

	if (device_may_wakeup(dev))
		disable_irq_wake(hcd->irq);

	ret = msm_hsic_resume(mehci);
	if (ret)
		return ret;

	/* Bring the device to full powered state upon system resume */
	pm_runtime_disable(dev);
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);

	return 0;
}
#endif

#ifdef CONFIG_PM_RUNTIME
static int msm_hsic_runtime_idle(struct device *dev)
{
	dev_dbg(dev, "EHCI runtime idle\n");
	return 0;
}

static int msm_hsic_runtime_suspend(struct device *dev)
{
	struct usb_hcd *hcd = dev_get_drvdata(dev);
	struct msm_hsic_hcd *mehci = hcd_to_hsic(hcd);

	dev_dbg(dev, "EHCI runtime suspend\n");

	dbg_log_event(NULL, "Run Time PM Suspend", 0,"", 0, "", 0);

	return msm_hsic_suspend(mehci);
}

static int msm_hsic_runtime_resume(struct device *dev)
{
	struct usb_hcd *hcd = dev_get_drvdata(dev);
	struct msm_hsic_hcd *mehci = hcd_to_hsic(hcd);

	dev_dbg(dev, "EHCI runtime resume\n");

	dbg_log_event(NULL, "Run Time PM Resume", 0,"", 0, "", 0);

	return msm_hsic_resume(mehci);
}
#endif

#ifdef CONFIG_PM
static const struct dev_pm_ops msm_hsic_dev_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(msm_hsic_pm_suspend, msm_hsic_pm_resume)
	SET_RUNTIME_PM_OPS(msm_hsic_runtime_suspend, msm_hsic_runtime_resume,
				msm_hsic_runtime_idle)
};
#endif

static struct platform_driver ehci_msm_hsic_driver = {
	.probe	= ehci_hsic_msm_probe,
	.remove	= __devexit_p(ehci_hsic_msm_remove),
	.driver = {
		.name = "msm_hsic_host",
#ifdef CONFIG_PM
		.pm = &msm_hsic_dev_pm_ops,
#endif
	},
};