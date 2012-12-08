/* Copyright (c) 2009-2012, Code Aurora Forum. All rights reserved.
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/time.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/hrtimer.h>
#include <linux/delay.h>
#include <mach/hardware.h>
#include <linux/io.h>

#include <asm/system.h>
#include <asm/mach-types.h>
#include <linux/semaphore.h>
#include <linux/spinlock.h>

#include <linux/fb.h>

#include <linux/ktime.h>
#include <linux/wakelock.h>
#include <linux/time.h>

#include "mdp.h"
#include "msm_fb.h"
#include "mdp4.h"

#ifdef CONFIG_FB_MSM_MDP40
#define LCDC_BASE	0xC0000
#else
#define LCDC_BASE	0xE0000
#endif

int first_pixel_start_x;
int first_pixel_start_y;
static int lcdc_enabled;

static struct mdp4_overlay_pipe *lcdc_pipe;
static struct completion lcdc_comp;

#define MAX_CONTROLLER	1

static struct vsycn_ctrl {
	struct device *dev;
	int inited;
	int update_ndx;
	int ov_koff;
	int ov_done;
	atomic_t suspend;
	atomic_t vsync_resume;
	int wait_vsync_cnt;
	int blt_change;
	int blt_free;
	int sysfs_created;
	struct mutex update_lock;
	struct completion ov_comp;
	struct completion dmap_comp;
	struct completion vsync_comp;
	spinlock_t spin_lock;
	struct msm_fb_data_type *mfd;
	struct mdp4_overlay_pipe *base_pipe;
	struct vsync_update vlist[2];
	int vsync_irq_enabled;
	ktime_t vsync_time;
} vsync_ctrl_db[MAX_CONTROLLER];


/*******************************************************
to do:
1) move vsync_irq_enable/vsync_irq_disable to mdp.c to be shared
*******************************************************/
static void vsync_irq_enable(int intr, int term)
{
	unsigned long flag;

	spin_lock_irqsave(&mdp_spin_lock, flag);
	outp32(MDP_INTR_CLEAR, intr);
	mdp_intr_mask |= intr;
	outp32(MDP_INTR_ENABLE, mdp_intr_mask);
	mdp_enable_irq(term);
	spin_unlock_irqrestore(&mdp_spin_lock, flag);
	pr_debug("%s: IRQ-en done, term=%x\n", __func__, term);
}


static void mdp4_overlay_lcdc_start(void)
{
	if (!lcdc_enabled) {
		/* enable DSI block */
		mdp4_iommu_attach();
		mdp_pipe_ctrl(MDP_OVERLAY0_BLOCK, MDP_BLOCK_POWER_ON, FALSE);
		MDP_OUTP(MDP_BASE + LCDC_BASE, 1);
		lcdc_enabled = 1;
	}
}
/*
 * mdp4_lcdc_pipe_queue:
 * called from thread context
 */
void mdp4_lcdc_pipe_queue(int cndx, struct mdp4_overlay_pipe *pipe)
{
	struct vsycn_ctrl *vctrl;
	struct vsync_update *vp;
	struct mdp4_overlay_pipe *pp;
	int undx;

	if (cndx >= MAX_CONTROLLER) {
		pr_err("%s: out or range: cndx=%d\n", __func__, cndx);
		return;
	}

	vctrl = &vsync_ctrl_db[cndx];

	if (atomic_read(&vctrl->suspend) > 0)
		return;

	mutex_lock(&vctrl->update_lock);
	undx =  vctrl->update_ndx;
	vp = &vctrl->vlist[undx];

	pp = &vp->plist[pipe->pipe_ndx - 1];	/* ndx start form 1 */

	pr_debug("%s: vndx=%d pipe_ndx=%d pid=%d\n", __func__,
			undx, pipe->pipe_ndx, current->pid);

	*pp = *pipe;	/* clone it */
	vp->update_cnt++;
	mutex_unlock(&vctrl->update_lock);
	mdp4_stat.overlay_play[pipe->mixer_num]++;
}

/*
 * MDP Timer/Functions
 * Set up an HR timer to wake up the CPU's just before the return of a VSync
 * This reduces any latencies that may arise if the CPU's were power collapsed
 * in between.
 */
#define VSYNC_INTERVAL	16
#define WAKE_DELAY	3 /* 3 ioctls * 600 us = 2ms + 1ms buffer */
#define MAX_VSYNC_GAP   4 /* Marker to detect whether to skip timer */

/* Move the globals into context data structure for 3.4 upgrade */
static int first_time = 1;
static ktime_t last_vsync_time_ns;
static struct hrtimer hr_mdp_timer_pc;
static void mdp4_lcdc_blt_ov_update(struct mdp4_overlay_pipe *pipe);
static void mdp4_lcdc_wait4dmap(int cndx);
static void mdp4_lcdc_wait4ov(int cndx);

static unsigned long compute_vsync_interval(void)
{
	ktime_t currtime_us;
	unsigned long diff_from_vsync, vsync_interval;
	/*
	 * Get interval beween last vsync and current time
	 * Current time = CPU programming MDP for next Vsync
	 */
	currtime_us = ktime_get();
	diff_from_vsync =
		(ktime_to_us(ktime_sub(currtime_us, last_vsync_time_ns)));
	diff_from_vsync /= USEC_PER_MSEC;
	/*
	 * If the last Vsync occurred more than 64 ms ago, skip programming
	 * the timer
	 */
	if (diff_from_vsync < (VSYNC_INTERVAL*MAX_VSYNC_GAP)) {
		vsync_interval =
			(VSYNC_INTERVAL-diff_from_vsync)%VSYNC_INTERVAL;
	} else
		vsync_interval = VSYNC_INTERVAL+1;

	return vsync_interval;
}

static void program_pc_timer(unsigned long diff_interval)
{
	ktime_t ktime_pc;
	unsigned long delay_in_ns = 0;

	/* Skip programming timer due to invalid delay */
	if (diff_interval > VSYNC_INTERVAL)
		return;

	if (diff_interval < WAKE_DELAY) {
		/*
		 * Difference from last vsync was a multiple of refresh rate
		 * (16*x)%16). Reset it to actual time to next vsync
		 */
		if (diff_interval == 0)
			delay_in_ns = VSYNC_INTERVAL-WAKE_DELAY;
		else
			return;	/* too close to vsync to fire timer - skip */
	} else if (diff_interval == WAKE_DELAY) {
		delay_in_ns = 1;  /* diff_interval/WAKE_DELAY */
	} else {
		delay_in_ns = diff_interval-WAKE_DELAY;
	}
	delay_in_ns *= NSEC_PER_MSEC;
	ktime_pc = ktime_set(0, delay_in_ns);
	hrtimer_start(&hr_mdp_timer_pc, ktime_pc, HRTIMER_MODE_REL);
}

int mdp4_lcdc_pipe_commit(int cndx, int wait)
{

	int  i, undx;
	int mixer = 0;
	struct vsycn_ctrl *vctrl;
	struct vsync_update *vp;
	struct mdp4_overlay_pipe *pipe;
	struct mdp4_overlay_pipe *real_pipe;
	unsigned long flags;
	int cnt = 0;

	vctrl = &vsync_ctrl_db[cndx];

	mutex_lock(&vctrl->update_lock);
	undx =  vctrl->update_ndx;
	vp = &vctrl->vlist[undx];
	pipe = vctrl->base_pipe;
	mixer = pipe->mixer_num;

	if (vp->update_cnt == 0) {
		mutex_unlock(&vctrl->update_lock);
		return 0;
	}

	vctrl->update_ndx++;
	vctrl->update_ndx &= 0x01;
	vp->update_cnt = 0;     /* reset */
	if (vctrl->blt_free) {
		vctrl->blt_free--;
		if (vctrl->blt_free == 0)
			mdp4_free_writeback_buf(vctrl->mfd, mixer);
	}
	mutex_unlock(&vctrl->update_lock);

	/* free previous committed iommu back to pool */
	mdp4_overlay_iommu_unmap_freelist(mixer);

	spin_lock_irqsave(&vctrl->spin_lock, flags);
	if (vctrl->ov_koff != vctrl->ov_done) {
		spin_unlock_irqrestore(&vctrl->spin_lock, flags);
		pr_err("%s: Error, frame dropped %d %d\n", __func__,
			vctrl->ov_koff, vctrl->ov_done);
		return 0;
	}
	spin_unlock_irqrestore(&vctrl->spin_lock, flags);

	mdp4_overlay_mdp_perf_upd(vctrl->mfd, 1);

	if (vctrl->blt_change) {
		pipe = vctrl->base_pipe;
		spin_lock_irqsave(&vctrl->spin_lock, flags);
		INIT_COMPLETION(vctrl->dmap_comp);
		INIT_COMPLETION(vctrl->ov_comp);
		vsync_irq_enable(INTR_DMA_P_DONE, MDP_DMAP_TERM);
		spin_unlock_irqrestore(&vctrl->spin_lock, flags);
		mdp4_lcdc_wait4dmap(0);
		if (pipe->ov_blt_addr)
			mdp4_lcdc_wait4ov(0);
	}

	pipe = vp->plist;
	for (i = 0; i < OVERLAY_PIPE_MAX; i++, pipe++) {
		if (pipe->pipe_used) {
			cnt++;
			real_pipe = mdp4_overlay_ndx2pipe(pipe->pipe_ndx);
			if (real_pipe && real_pipe->pipe_used) {
				/* pipe not unset */
				mdp4_overlay_vsync_commit(pipe);
			}
			/* free previous iommu to freelist
			 * which will be freed at next
			 * pipe_commit
			 */
			mdp4_overlay_iommu_pipe_free(pipe->pipe_ndx, 0);
			pipe->pipe_used = 0; /* clear */
		}
	}

	mdp4_mixer_stage_commit(mixer);

	/* start timing generator & mmu if they are not started yet */
	mdp4_overlay_lcdc_start();

	pipe = vctrl->base_pipe;
	spin_lock_irqsave(&vctrl->spin_lock, flags);
	if (pipe->ov_blt_addr) {
		mdp4_lcdc_blt_ov_update(pipe);
		pipe->ov_cnt++;
		INIT_COMPLETION(vctrl->ov_comp);
		vsync_irq_enable(INTR_OVERLAY0_DONE, MDP_OVERLAY0_TERM);
		mb();
		vctrl->ov_koff++;
		/* kickoff overlay engine */
		mdp4_stat.kickoff_ov0++;
		outpdw(MDP_BASE + 0x0004, 0);
	} else {
		/* schedule second phase update  at dmap */
		INIT_COMPLETION(vctrl->dmap_comp);
		vsync_irq_enable(INTR_DMA_P_DONE, MDP_DMAP_TERM);
	}
	spin_unlock_irqrestore(&vctrl->spin_lock, flags);

	mdp4_stat.overlay_commit[pipe->mixer_num]++;

	if (wait) {
		if (pipe->ov_blt_addr)
			mdp4_lcdc_wait4ov(0);
		else
			mdp4_lcdc_wait4dmap(0);
	}

	return cnt;
}

static enum hrtimer_restart mdp_pc_hrtimer_callback(struct hrtimer *timer)
{
	if (!wake_lock_active(&mdp_idle_wakelock)) {
		/* Hold Wakelock if no locks held */
		wake_lock(&mdp_idle_wakelock);
	}
	return HRTIMER_NORESTART;
}

static void init_pc_timer(void)
{
	/*
	 * Initialize hr timer which fires a few ms before Vsync - this
	 * gets rid of any latencies that may arise due to
	 * wake up from PC
	 */
	hrtimer_init(&hr_mdp_timer_pc, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	hr_mdp_timer_pc.function = &mdp_pc_hrtimer_callback;
}

static void mdp4_lcdc_wait4dmap(int cndx)
{
	struct vsycn_ctrl *vctrl;

	if (cndx >= MAX_CONTROLLER) {
		pr_err("%s: out or range: cndx=%d\n", __func__, cndx);
		return;
	}

	vctrl = &vsync_ctrl_db[cndx];

	if (atomic_read(&vctrl->suspend) > 0)
		return;

	wait_for_completion(&vctrl->dmap_comp);
}

static void mdp4_lcdc_wait4ov(int cndx)
{
	struct vsycn_ctrl *vctrl;

	if (cndx >= MAX_CONTROLLER) {
		pr_err("%s: out or range: cndx=%d\n", __func__, cndx);
		return;
	}

	vctrl = &vsync_ctrl_db[cndx];

	if (atomic_read(&vctrl->suspend) > 0)
		return;

	wait_for_completion(&vctrl->ov_comp);
}

static ssize_t vsync_show_event(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int cndx;
	struct vsycn_ctrl *vctrl;
	ssize_t ret = 0;
	unsigned long flags;

	cndx = 0;
	vctrl = &vsync_ctrl_db[0];

	if (atomic_read(&vctrl->suspend) > 0 ||
		atomic_read(&vctrl->vsync_resume) == 0)
		return 0;

	spin_lock_irqsave(&vctrl->spin_lock, flags);
	if (vctrl->wait_vsync_cnt == 0)
		INIT_COMPLETION(vctrl->vsync_comp);
	vctrl->wait_vsync_cnt++;
	spin_unlock_irqrestore(&vctrl->spin_lock, flags);
	wait_for_completion(&vctrl->vsync_comp);

	ret = snprintf(buf, PAGE_SIZE, "VSYNC=%llu",
			ktime_to_ns(vctrl->vsync_time));
	buf[strlen(buf) + 1] = '\0';
	return ret;
}

static DEVICE_ATTR(vsync_event, S_IRUGO, vsync_show_event, NULL);
static struct attribute *vsync_fs_attrs[] = {
	&dev_attr_vsync_event.attr,
	NULL,
};
static struct attribute_group vsync_fs_attr_group = {
	.attrs = vsync_fs_attrs,
};

void mdp4_lcdc_vsync_init(int cndx, struct msm_fb_data_type *mfd)
{
	struct vsycn_ctrl *vctrl;
	int ret;

	if (cndx >= MAX_CONTROLLER) {
		pr_err("%s: out or range: cndx=%d\n", __func__, cndx);
		return;
	}

	pr_info("%s: ndx=%d\n", __func__, cndx);

	vctrl = &vsync_ctrl_db[cndx];
	if (vctrl->inited)
		return;

	vctrl->mfd = mfd;
	vctrl->dev = mfd->fbi->dev;
	vctrl->inited = 1;
	vctrl->update_ndx = 0;
	mutex_init(&vctrl->update_lock);
	init_completion(&vctrl->vsync_comp);
	init_completion(&vctrl->dmap_comp);
	init_completion(&vctrl->ov_comp);
	atomic_set(&vctrl->suspend, 1);
	atomic_set(&vctrl->vsync_resume, 1);
	spin_lock_init(&vctrl->spin_lock);
	if (!vctrl->sysfs_created) {
		ret = sysfs_create_group(&vctrl->dev->kobj,
			&vsync_fs_attr_group);
		if (ret) {
			pr_err("%s: sysfs group creation failed, ret=%d\n",
				__func__, ret);
			return;
		}

		kobject_uevent(&vctrl->dev->kobj, KOBJ_ADD);
		pr_debug("%s: kobject_uevent(KOBJ_ADD)\n", __func__);
		vctrl->sysfs_created = 1;
	}
}

void mdp4_lcdc_base_swap(int cndx, struct mdp4_overlay_pipe *pipe)
{
	struct vsycn_ctrl *vctrl;

	if (cndx >= MAX_CONTROLLER) {
		pr_err("%s: out or range: cndx=%d\n", __func__, cndx);
		return;
	}

	vctrl = &vsync_ctrl_db[cndx];
	vctrl->base_pipe = pipe;
}


int mdp4_lcdc_on(struct platform_device *pdev)
{
	int lcdc_width;
	int lcdc_height;
	int lcdc_bpp;
	int lcdc_border_clr;
	int lcdc_underflow_clr;
	int lcdc_hsync_skew;

	int hsync_period;
	int hsync_ctrl;
	int vsync_period;
	int display_hctl;
	int display_v_start;
	int display_v_end;
	int active_hctl;
	int active_h_start;
	int active_h_end;
	int active_v_start;
	int active_v_end;
	int ctrl_polarity;
	int h_back_porch;
	int h_front_porch;
	int v_back_porch;
	int v_front_porch;
	int hsync_pulse_width;
	int vsync_pulse_width;
	int hsync_polarity;
	int vsync_polarity;
	int data_en_polarity;
	int hsync_start_x;
	int hsync_end_x;
	uint8 *buf;
	unsigned int buf_offset;
	int bpp, ptype;
	struct fb_info *fbi;
	struct fb_var_screeninfo *var;
	struct msm_fb_data_type *mfd;
	struct mdp4_overlay_pipe *pipe;
	int ret = 0;
	int cndx = 0;
	struct vsycn_ctrl *vctrl;

	vctrl = &vsync_ctrl_db[cndx];
	mfd = (struct msm_fb_data_type *)platform_get_drvdata(pdev);

	if (!mfd)
		return -ENODEV;

	if (mfd->key != MFD_KEY)
		return -EINVAL;

	vctrl->mfd = mfd;
	vctrl->dev = mfd->fbi->dev;

	/* mdp clock on */
	mdp_clk_ctrl(1);

	fbi = mfd->fbi;
	var = &fbi->var;

	/* MDP cmd block enable */
	mdp_pipe_ctrl(MDP_CMD_BLOCK, MDP_BLOCK_POWER_ON, FALSE);
	if (is_mdp4_hw_reset()) {
		mdp4_hw_init();
		outpdw(MDP_BASE + 0x0038, mdp4_display_intf);
	}

	bpp = fbi->var.bits_per_pixel / 8;
	buf = (uint8 *) fbi->fix.smem_start;
	buf_offset = calc_fb_offset(mfd, fbi, bpp);

	if (vctrl->base_pipe == NULL) {
		ptype = mdp4_overlay_format2type(mfd->fb_imgType);
		if (ptype < 0)
			printk(KERN_INFO "%s: format2type failed\n", __func__);
		pipe = mdp4_overlay_pipe_alloc(ptype, MDP4_MIXER0);
		if (pipe == NULL)
			printk(KERN_INFO "%s: pipe_alloc failed\n", __func__);
		pipe->pipe_used++;
		pipe->mixer_stage  = MDP4_MIXER_STAGE_BASE;
		pipe->mixer_num  = MDP4_MIXER0;
		pipe->src_format = mfd->fb_imgType;
		mdp4_overlay_panel_mode(pipe->mixer_num, MDP4_PANEL_LCDC);
		ret = mdp4_overlay_format2pipe(pipe);
		if (ret < 0)
			printk(KERN_INFO "%s: format2pipe failed\n", __func__);
		lcdc_pipe = pipe; /* keep it */
		init_completion(&lcdc_comp);

		mdp4_init_writeback_buf(mfd, MDP4_MIXER0);
		pipe->ov_blt_addr = 0;
		pipe->dma_blt_addr = 0;

		vctrl->base_pipe = pipe; /* keep it */
	} else {
		pipe = vctrl->base_pipe;
	}

	pipe->src_height = fbi->var.yres;
	pipe->src_width = fbi->var.xres;
	pipe->src_h = fbi->var.yres;
	pipe->src_w = fbi->var.xres;
	pipe->src_y = 0;
	pipe->src_x = 0;
	pipe->dst_h = fbi->var.yres;
	pipe->dst_w = fbi->var.xres;

	if (mfd->display_iova)
		pipe->srcp0_addr = mfd->display_iova + buf_offset;
	else
		pipe->srcp0_addr = (uint32)(buf + buf_offset);

	pipe->srcp0_ystride = fbi->fix.line_length;
	pipe->bpp = bpp;

	mdp4_overlay_mdp_pipe_req(pipe, mfd);

	atomic_set(&vctrl->suspend, 0);

	mdp4_overlay_dmap_xy(pipe);
	mdp4_overlay_dmap_cfg(mfd, 1);
	mdp4_overlay_rgb_setup(pipe);
	mdp4_overlayproc_cfg(pipe);

	mdp4_overlay_reg_flush(pipe, 1);
	mdp4_mixer_stage_up(pipe, 0);


	/*
	 * LCDC timing setting
	 */
	h_back_porch = var->left_margin;
	h_front_porch = var->right_margin;
	v_back_porch = var->upper_margin;
	v_front_porch = var->lower_margin;
	hsync_pulse_width = var->hsync_len;
	vsync_pulse_width = var->vsync_len;
	lcdc_border_clr = mfd->panel_info.lcdc.border_clr;
	lcdc_underflow_clr = mfd->panel_info.lcdc.underflow_clr;
	lcdc_hsync_skew = mfd->panel_info.lcdc.hsync_skew;

	lcdc_width = var->xres + mfd->panel_info.lcdc.xres_pad;
	lcdc_height = var->yres + mfd->panel_info.lcdc.yres_pad;
	lcdc_bpp = mfd->panel_info.bpp;

	hsync_period =
	    hsync_pulse_width + h_back_porch + h_front_porch;
	if ((mfd->panel_info.type == LVDS_PANEL) &&
		(mfd->panel_info.lvds.channel_mode == LVDS_DUAL_CHANNEL_MODE))
		hsync_period += lcdc_width / 2;
	else
		hsync_period += lcdc_width;
	hsync_ctrl = (hsync_period << 16) | hsync_pulse_width;
	hsync_start_x = hsync_pulse_width + h_back_porch;
	hsync_end_x = hsync_period - h_front_porch - 1;
	display_hctl = (hsync_end_x << 16) | hsync_start_x;

	vsync_period =
	    (vsync_pulse_width + v_back_porch + lcdc_height +
	     v_front_porch) * hsync_period;
	display_v_start =
	    (vsync_pulse_width + v_back_porch) * hsync_period + lcdc_hsync_skew;
	display_v_end =
	    vsync_period - (v_front_porch * hsync_period) + lcdc_hsync_skew - 1;

	if (lcdc_width != var->xres) {
		active_h_start = hsync_start_x + first_pixel_start_x;
		active_h_end = active_h_start + var->xres - 1;
		active_hctl =
		    ACTIVE_START_X_EN | (active_h_end << 16) | active_h_start;
	} else {
		active_hctl = 0;
	}

	if (lcdc_height != var->yres) {
		active_v_start =
		    display_v_start + first_pixel_start_y * hsync_period;
		active_v_end = active_v_start + (var->yres) * hsync_period - 1;
		active_v_start |= ACTIVE_START_Y_EN;
	} else {
		active_v_start = 0;
		active_v_end = 0;
	}


#ifdef CONFIG_FB_MSM_MDP40
	if (mfd->panel_info.lcdc.is_sync_active_high) {
		hsync_polarity = 0;
		vsync_polarity = 0;
	} else {
		hsync_polarity = 1;
		vsync_polarity = 1;
	}
	lcdc_underflow_clr |= 0x80000000;	/* enable recovery */
#else
	hsync_polarity = 0;
	vsync_polarity = 0;
#endif
	data_en_polarity = 0;

	ctrl_polarity =
	    (data_en_polarity << 2) | (vsync_polarity << 1) | (hsync_polarity);

	mdp_pipe_ctrl(MDP_CMD_BLOCK, MDP_BLOCK_POWER_ON, FALSE);
	MDP_OUTP(MDP_BASE + LCDC_BASE + 0x4, hsync_ctrl);
	MDP_OUTP(MDP_BASE + LCDC_BASE + 0x8, vsync_period);
	MDP_OUTP(MDP_BASE + LCDC_BASE + 0xc, vsync_pulse_width * hsync_period);
	MDP_OUTP(MDP_BASE + LCDC_BASE + 0x10, display_hctl);
	MDP_OUTP(MDP_BASE + LCDC_BASE + 0x14, display_v_start);
	MDP_OUTP(MDP_BASE + LCDC_BASE + 0x18, display_v_end);
	MDP_OUTP(MDP_BASE + LCDC_BASE + 0x28, lcdc_border_clr);
	MDP_OUTP(MDP_BASE + LCDC_BASE + 0x2c, lcdc_underflow_clr);
	MDP_OUTP(MDP_BASE + LCDC_BASE + 0x30, lcdc_hsync_skew);
	MDP_OUTP(MDP_BASE + LCDC_BASE + 0x38, ctrl_polarity);
	MDP_OUTP(MDP_BASE + LCDC_BASE + 0x1c, active_hctl);
	MDP_OUTP(MDP_BASE + LCDC_BASE + 0x20, active_v_start);
	MDP_OUTP(MDP_BASE + LCDC_BASE + 0x24, active_v_end);
	mdp_pipe_ctrl(MDP_CMD_BLOCK, MDP_BLOCK_POWER_OFF, FALSE);

	mdp4_overlay_reg_flush(pipe, 1);
	mdp4_mixer_stage_up(pipe,0);

#ifdef CONFIG_MSM_BUS_SCALING
	mdp_bus_scale_update_request(2);
#endif
	mdp_histogram_ctrl_all(TRUE);

	return ret;
}

int mdp_lcdc_off(struct platform_device *pdev)
{
	int ret = 0;
	struct msm_fb_data_type *mfd;

	mfd = (struct msm_fb_data_type *)platform_get_drvdata(pdev);

	mutex_lock(&mfd->dma->ov_mutex);

	/* MDP cmd block enable */
	mdp_pipe_ctrl(MDP_CMD_BLOCK, MDP_BLOCK_POWER_ON, FALSE);
	mdp4_mixer_pipe_cleanup(lcdc_pipe->mixer_num);
	MDP_OUTP(MDP_BASE + LCDC_BASE, 0);
	lcdc_enabled = 0;
	/* MDP cmd block disable */
	mdp_pipe_ctrl(MDP_CMD_BLOCK, MDP_BLOCK_POWER_OFF, FALSE);
	mdp_pipe_ctrl(MDP_OVERLAY0_BLOCK, MDP_BLOCK_POWER_OFF, FALSE);

	mdp_histogram_ctrl_all(FALSE);
	ret = panel_next_off(pdev);

	mutex_unlock(&mfd->dma->ov_mutex);

	/* delay to make sure the last frame finishes */
	msleep(20);

	/* dis-engage rgb0 from mixer0 */
	if (lcdc_pipe) {
		mdp4_mixer_stage_down(lcdc_pipe,1);
		mdp4_iommu_unmap(lcdc_pipe);
	}

#ifdef CONFIG_MSM_BUS_SCALING
	mdp_bus_scale_update_request(0);
#endif

	return ret;
}

static void mdp4_lcdc_blt_ov_update(struct mdp4_overlay_pipe *pipe)
{
	uint32 off, addr;
	int bpp;
	char *overlay_base;


	if (pipe->ov_blt_addr == 0)
		return;


#ifdef BLT_RGB565
	bpp = 2; /* overlay ouput is RGB565 */
#else
	bpp = 3; /* overlay ouput is RGB888 */
#endif
	off = 0;
	if (pipe->ov_cnt & 0x01)
		off = pipe->src_height * pipe->src_width * bpp;
	addr = pipe->ov_blt_addr + off;

	/* overlay 0 */
	overlay_base = MDP_BASE + MDP4_OVERLAYPROC0_BASE;/* 0x10000 */
	outpdw(overlay_base + 0x000c, addr);
	outpdw(overlay_base + 0x001c, addr);
}

static void mdp4_lcdc_blt_dmap_update(struct mdp4_overlay_pipe *pipe)
{
	uint32 off, addr;
	int bpp;

	if (pipe->ov_blt_addr == 0)
		return;


#ifdef BLT_RGB565
	bpp = 2; /* overlay ouput is RGB565 */
#else
	bpp = 3; /* overlay ouput is RGB888 */
#endif
	off = 0;
	if (pipe->dmap_cnt & 0x01)
		off = pipe->src_height * pipe->src_width * bpp;
	addr = pipe->dma_blt_addr + off;

	/* dmap */
	MDP_OUTP(MDP_BASE + 0x90008, addr);
}

/*
 * mdp4_overlay_lcdc_wait4event:
 * INTR_DMA_P_DONE and INTR_PRIMARY_VSYNC event only
 * no INTR_OVERLAY0_DONE event allowed.
 */
static void mdp4_overlay_lcdc_wait4event(struct msm_fb_data_type *mfd,
					int intr_done)
{
	unsigned long flag;
	unsigned int data;
    unsigned long vsync_interval;

	data = inpdw(MDP_BASE + LCDC_BASE);
	data &= 0x01;
	if (data == 0)	/* timing generator disabled */
		return;

	if ((intr_done == INTR_PRIMARY_VSYNC) ||
			(intr_done == INTR_DMA_P_DONE)) {
		if (first_time) {
			init_pc_timer();
			first_time = 0;
		}
		vsync_interval = compute_vsync_interval();
		program_pc_timer(vsync_interval);
	}

	spin_lock_irqsave(&mdp_spin_lock, flag);
	INIT_COMPLETION(lcdc_comp);
	mfd->dma->waiting = TRUE;
	outp32(MDP_INTR_CLEAR, intr_done);
	mdp_intr_mask |= intr_done;
	outp32(MDP_INTR_ENABLE, mdp_intr_mask);
	mdp_enable_irq(MDP_DMA2_TERM);  /* enable intr */
	spin_unlock_irqrestore(&mdp_spin_lock, flag);
	wait_for_completion(&lcdc_comp);
	mdp_disable_irq(MDP_DMA2_TERM);
}

static void mdp4_overlay_lcdc_dma_busy_wait(struct msm_fb_data_type *mfd)
{
	unsigned long flag;
	int need_wait = 0;

	pr_debug("%s: start pid=%d\n", __func__, current->pid);

	spin_lock_irqsave(&mdp_spin_lock, flag);
	if (mfd->dma->busy == TRUE) {
		INIT_COMPLETION(mfd->dma->comp);
		need_wait++;
	}
	spin_unlock_irqrestore(&mdp_spin_lock, flag);

	if (need_wait) {
		/* wait until DMA finishes the current job */
		pr_debug("%s: pending pid=%d\n", __func__, current->pid);
		wait_for_completion(&mfd->dma->comp);
	}
	pr_debug("%s: done pid=%d\n", __func__, current->pid);
}

void mdp4_overlay_lcdc_vsync_push(struct msm_fb_data_type *mfd,
			struct mdp4_overlay_pipe *pipe)
{
	unsigned long flag;

	if (pipe->flags & MDP_OV_PLAY_NOWAIT)
		return;

	if (lcdc_pipe->ov_blt_addr) {
		mdp4_overlay_lcdc_dma_busy_wait(mfd);

		mdp4_lcdc_blt_ov_update(lcdc_pipe);
		lcdc_pipe->ov_cnt++;

		spin_lock_irqsave(&mdp_spin_lock, flag);
		outp32(MDP_INTR_CLEAR, INTR_OVERLAY0_DONE);
		mdp_intr_mask |= INTR_OVERLAY0_DONE;
		outp32(MDP_INTR_ENABLE, mdp_intr_mask);
		mdp_enable_irq(MDP_OVERLAY0_TERM);
		mfd->dma->busy = TRUE;
		mb();	/* make sure all registers updated */
		spin_unlock_irqrestore(&mdp_spin_lock, flag);
		outpdw(MDP_BASE + 0x0004, 0); /* kickoff overlay engine */
		mdp4_stat.kickoff_ov0++;
		mb();
		mdp4_overlay_lcdc_wait4event(mfd, INTR_DMA_P_DONE);
	} else {
		mdp4_overlay_lcdc_wait4event(mfd, INTR_PRIMARY_VSYNC);
	}
	mdp4_set_perf_level();
}

/*
 * mdp4_primary_vsync_lcdc: called from isr
 */
void mdp4_primary_vsync_lcdc(void)
{
	complete_all(&lcdc_comp);
	last_vsync_time_ns = ktime_get();
	/* Release Wakelock */
	if (wake_lock_active(&mdp_idle_wakelock))
		wake_unlock(&mdp_idle_wakelock);
}

/*
 * mdp4_dma_p_done_lcdc: called from isr
 */
void mdp4_dma_p_done_lcdc(void)
{
	complete_all(&lcdc_comp);
	last_vsync_time_ns = ktime_get();
	/* Release Wakelock */
	if (wake_lock_active(&mdp_idle_wakelock))
		wake_unlock(&mdp_idle_wakelock);
}

/*
 * mdp4_overlay0_done_lcdc: called from isr
 */
void mdp4_overlay0_done_lcdc(struct mdp_dma_data *dma)
{
	spin_lock(&mdp_spin_lock);
	dma->busy = FALSE;
	if (lcdc_pipe->ov_blt_addr == 0) {
		spin_unlock(&mdp_spin_lock);
		return;
	}
	mdp4_lcdc_blt_dmap_update(lcdc_pipe);
	lcdc_pipe->dmap_cnt++;
	mdp_disable_irq_nosync(MDP_OVERLAY0_TERM);
	spin_unlock(&mdp_spin_lock);
	complete(&dma->comp);
}

static void mdp4_overlay_lcdc_prefill(struct msm_fb_data_type *mfd)
{
	unsigned long flag;

	if (lcdc_pipe->ov_blt_addr) {
		mdp4_overlay_lcdc_dma_busy_wait(mfd);

		mdp4_lcdc_blt_ov_update(lcdc_pipe);
		lcdc_pipe->ov_cnt++;

		spin_lock_irqsave(&mdp_spin_lock, flag);
		outp32(MDP_INTR_CLEAR, INTR_OVERLAY0_DONE);
		mdp_intr_mask |= INTR_OVERLAY0_DONE;
		outp32(MDP_INTR_ENABLE, mdp_intr_mask);
		mdp_enable_irq(MDP_OVERLAY0_TERM);
		mfd->dma->busy = TRUE;
		mb();	/* make sure all registers updated */
		spin_unlock_irqrestore(&mdp_spin_lock, flag);
		outpdw(MDP_BASE + 0x0004, 0); /* kickoff overlay engine */
		mdp4_stat.kickoff_ov0++;
		mb();
	}
}
/*
 * make sure the MIPI_DSI_WRITEBACK_SIZE defined at boardfile
 * has enough space h * w * 3 * 2
 */
static void mdp4_lcdc_do_blt(struct msm_fb_data_type *mfd, int enable)
{
	unsigned long flag;
	int change = 0;

	mdp4_allocate_writeback_buf(mfd, MDP4_MIXER0);

	if (!mfd->ov0_wb_buf->write_addr) {
		pr_debug("%s: no blt_base assigned\n", __func__);
		return;
	}

	spin_lock_irqsave(&mdp_spin_lock, flag);
	if (enable && lcdc_pipe->ov_blt_addr == 0) {
		lcdc_pipe->ov_blt_addr = mfd->ov0_wb_buf->write_addr;
		lcdc_pipe->dma_blt_addr = mfd->ov0_wb_buf->read_addr;
		change++;
		lcdc_pipe->blt_cnt = 0;
		lcdc_pipe->ov_cnt = 0;
		lcdc_pipe->dmap_cnt = 0;
		mdp4_stat.blt_lcdc++;
	} else if (enable == 0 && lcdc_pipe->ov_blt_addr) {
		lcdc_pipe->ov_blt_addr = 0;
		lcdc_pipe->dma_blt_addr = 0;
		change++;
	}
	pr_info("%s: ov_blt_addr=%x\n", __func__, (int)lcdc_pipe->ov_blt_addr);
	spin_unlock_irqrestore(&mdp_spin_lock, flag);

	if (!change)
		return;

	mdp4_overlay_lcdc_wait4event(mfd, INTR_DMA_P_DONE);
	MDP_OUTP(MDP_BASE + LCDC_BASE, 0);	/* stop lcdc */
	msleep(20);
	mdp4_overlay_dmap_xy(lcdc_pipe);
	mdp4_overlayproc_cfg(lcdc_pipe);
	if (lcdc_pipe->ov_blt_addr) {
		mdp4_overlay_lcdc_prefill(mfd);
		mdp4_overlay_lcdc_prefill(mfd);
	}
	MDP_OUTP(MDP_BASE + LCDC_BASE, 1);	/* start lcdc */
}

int mdp4_lcdc_overlay_blt_offset(struct msm_fb_data_type *mfd,
					struct msmfb_overlay_blt *req)
{
	req->offset = 0;
	req->width = lcdc_pipe->src_width;
	req->height = lcdc_pipe->src_height;
	req->bpp = lcdc_pipe->bpp;

	return sizeof(*req);
}

void mdp4_lcdc_overlay_blt(struct msm_fb_data_type *mfd,
					struct msmfb_overlay_blt *req)
{
	mdp4_lcdc_do_blt(mfd, req->enable);
}

void mdp4_lcdc_overlay_blt_start(struct msm_fb_data_type *mfd)
{
	mdp4_lcdc_do_blt(mfd, 1);
}

void mdp4_lcdc_overlay_blt_stop(struct msm_fb_data_type *mfd)
{
	mdp4_lcdc_do_blt(mfd, 0);
}

void mdp4_lcdc_overlay(struct msm_fb_data_type *mfd)
{
	struct fb_info *fbi = mfd->fbi;
	uint8 *buf;
	unsigned int buf_offset;
	int bpp;
	int cndx = 0;
	struct vsycn_ctrl *vctrl;
	struct mdp4_overlay_pipe *pipe;


	vctrl = &vsync_ctrl_db[cndx];
	pipe = vctrl->base_pipe;

	if (!pipe || !mfd->panel_power_on)
		return;

	pr_debug("%s: cpu=%d pid=%d\n", __func__,
			smp_processor_id(), current->pid);
	if (pipe->pipe_type == OVERLAY_TYPE_RGB) {
		bpp = fbi->var.bits_per_pixel / 8;
		buf = (uint8 *) fbi->fix.smem_start;
		buf_offset = calc_fb_offset(mfd, fbi, bpp);

		if (mfd->display_iova)
			pipe->srcp0_addr = mfd->display_iova + buf_offset;
		else
			pipe->srcp0_addr = (uint32)(buf + buf_offset);

		mdp4_lcdc_pipe_queue(0, pipe);
	}

	mdp4_overlay_mdp_perf_upd(mfd, 1);

	mutex_lock(&mfd->dma->ov_mutex);
	mdp4_lcdc_pipe_commit(0, 0);
	mutex_unlock(&mfd->dma->ov_mutex);

	if (pipe->ov_blt_addr)
		mdp4_lcdc_wait4ov(0);
	else
		mdp4_lcdc_wait4dmap(0);

	mdp4_overlay_mdp_perf_upd(mfd, 0);
}
