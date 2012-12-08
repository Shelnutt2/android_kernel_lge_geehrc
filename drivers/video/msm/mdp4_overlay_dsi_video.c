/* Copyright (c) 2010-2012, Code Aurora Forum. All rights reserved.
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
#include <linux/io.h>
#include <linux/semaphore.h>
#include <linux/spinlock.h>
#include <linux/fb.h>
#include <linux/msm_mdp.h>
#include <linux/ktime.h>
#include <linux/wakelock.h>
#include <linux/time.h>
#include <asm/system.h>
#include <asm/mach-types.h>
#include <mach/hardware.h>
#include "mdp.h"
#include "msm_fb.h"
#include "mdp4.h"
#include "mipi_dsi.h"
#include <mach/iommu_domains.h>

/* LGE_CHANGE_S
* for power sequence of lgit panel
* 2012-05-28 jungbeom.shim@lge.com
*/
#if defined(CONFIG_MACH_LGE)
#if defined(CONFIG_FB_MSM_MIPI_LGIT_VIDEO_WXGA_PT)
#include "mipi_lgit.h"
#elif defined(CONFIG_FB_MSM_MIPI_LGIT_VIDEO_FHD_PT) ||\
	defined(CONFIG_FB_MSM_MIPI_LGIT_VIDEO_FHD_INVERSE_PT)
#include "mipi_lgit_fhd.h"
#endif
#endif
/* LGE_CHANGE_E */

#define DSI_VIDEO_BASE	0xE0000

static int first_pixel_start_x;
static int first_pixel_start_y;
static int dsi_video_enabled;

#define MAX_CONTROLLER	1
static struct mdp4_overlay_pipe *dsi_pipe;
static struct completion dsi_video_comp;
static int blt_cfg_changed;

static cmd_fxn_t display_on;

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
	int blt_ctrl;
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
#ifdef CONFIG_HRZ_II
int hrz_res_conv = 0;
int hrz_mode_chg = 0;
#endif

static void vsync_irq_disable(int intr, int term)
{
	unsigned long flag;

	spin_lock_irqsave(&mdp_spin_lock, flag);
	outp32(MDP_INTR_CLEAR, intr);
	mdp_intr_mask &= ~intr;
	outp32(MDP_INTR_ENABLE, mdp_intr_mask);
	mdp_disable_irq_nosync(term);
	spin_unlock_irqrestore(&mdp_spin_lock, flag);
	pr_debug("%s: IRQ-dis done, term=%x\n", __func__, term);
}

static __u32 msm_fb_line_length(__u32 fb_index, __u32 xres, int bpp)
{
	/*
	 * The adreno GPU hardware requires that the pitch be aligned to
	 * 32 pixels for color buffers, so for the cases where the GPU
	 * is writing directly to fb0, the framebuffer pitch
	 * also needs to be 32 pixel aligned
	 */

	if (fb_index == 0)
		return ALIGN(xres, 32) * bpp;
		else
		return xres * bpp;
}

/*
 * mdp4_dsi_video_pipe_queue:
 * called from thread context
 */
void mdp4_dsi_video_pipe_queue(int cndx, struct mdp4_overlay_pipe *pipe)
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

	pr_debug("%s: vndx=%d pipe=%x ndx=%d num=%d pid=%d\n",
		 __func__, undx, (int)pipe, pipe->pipe_ndx, pipe->pipe_num,
		current->pid);

	*pp = *pipe;	/* clone it */
	vp->update_cnt++;
	mutex_unlock(&vctrl->update_lock);
	mdp4_stat.overlay_play[pipe->mixer_num]++;
}

static void mdp4_dsi_video_blt_ov_update(struct mdp4_overlay_pipe *pipe);
static void mdp4_dsi_video_wait4dmap(int cndx);
static void mdp4_dsi_video_wait4ov(int cndx);

int mdp4_dsi_video_pipe_commit(int cndx, int wait)
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
		return cnt;
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

	if (vctrl->blt_change) {
		pipe = vctrl->base_pipe;
		spin_lock_irqsave(&vctrl->spin_lock, flags);
		INIT_COMPLETION(vctrl->dmap_comp);
		INIT_COMPLETION(vctrl->ov_comp);
		vsync_irq_enable(INTR_DMA_P_DONE, MDP_DMAP_TERM);
		spin_unlock_irqrestore(&vctrl->spin_lock, flags);
		mdp4_dsi_video_wait4dmap(0);
		if (pipe->ov_blt_addr)
			mdp4_dsi_video_wait4ov(0);
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
	mdp4_overlay_dsi_video_start();

	pipe = vctrl->base_pipe;
	spin_lock_irqsave(&vctrl->spin_lock, flags);
	if (pipe->ov_blt_addr) {
		mdp4_dsi_video_blt_ov_update(pipe);
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
			mdp4_dsi_video_wait4ov(0);
		else
			mdp4_dsi_video_wait4dmap(0);
	}

	return cnt;
}

void mdp4_dsi_video_vsync_ctrl(struct fb_info *info, int enable)
{
	struct vsycn_ctrl *vctrl;
	int cndx = 0;

	vctrl = &vsync_ctrl_db[cndx];

	if (vctrl->vsync_irq_enabled == enable)
		return;

	pr_debug("%s: vsync enable=%d\n", __func__, enable);

	vctrl->vsync_irq_enabled = enable;

	if (enable)
		vsync_irq_enable(INTR_PRIMARY_VSYNC, MDP_PRIM_VSYNC_TERM);
	else
		vsync_irq_disable(INTR_PRIMARY_VSYNC, MDP_PRIM_VSYNC_TERM);

	if (vctrl->vsync_irq_enabled &&  atomic_read(&vctrl->suspend) == 0)
		atomic_set(&vctrl->vsync_resume, 1);
}

void mdp4_dsi_video_wait4vsync(int cndx, long long *vtime)
{
	struct vsycn_ctrl *vctrl;
	struct mdp4_overlay_pipe *pipe;
	unsigned long flags;

	if (cndx >= MAX_CONTROLLER) {
		pr_err("%s: out or range: cndx=%d\n", __func__, cndx);
		return;
	}

	vctrl = &vsync_ctrl_db[cndx];
	pipe = vctrl->base_pipe;

	if (atomic_read(&vctrl->suspend) > 0) {
		*vtime = -1;
		return;
	}

	/* start timing generator & mmu if they are not started yet */
	mdp4_overlay_dsi_video_start();

	spin_lock_irqsave(&vctrl->spin_lock, flags);
	if (vctrl->wait_vsync_cnt == 0)
		INIT_COMPLETION(vctrl->vsync_comp);

	vctrl->wait_vsync_cnt++;
	spin_unlock_irqrestore(&vctrl->spin_lock, flags);

	wait_for_completion(&vctrl->vsync_comp);
	mdp4_stat.wait4vsync0++;

	*vtime = ktime_to_ns(vctrl->vsync_time);
}

static void mdp4_dsi_video_wait4dmap(int cndx)
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


static void mdp4_dsi_video_wait4dmap_done(int cndx)
{
	unsigned long flags;
	struct vsycn_ctrl *vctrl;

	if (cndx >= MAX_CONTROLLER) {
		pr_err("%s: out or range: cndx=%d\n", __func__, cndx);
		return;
	}
	vctrl = &vsync_ctrl_db[cndx];

	spin_lock_irqsave(&vctrl->spin_lock, flags);
	INIT_COMPLETION(vctrl->dmap_comp);
	vsync_irq_enable(INTR_DMA_P_DONE, MDP_DMAP_TERM);
	spin_unlock_irqrestore(&vctrl->spin_lock, flags);
	mdp4_dsi_video_wait4dmap(cndx);
}


static void mdp4_dsi_video_wait4ov(int cndx)
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
	ret = wait_for_completion_interruptible(&vctrl->vsync_comp);
	if (ret < 0)
		return ret;

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

void mdp4_dsi_vsync_init(int cndx, struct msm_fb_data_type *mfd)
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
void mdp4_dsi_video_base_swap(int cndx, struct mdp4_overlay_pipe *pipe)
{
	struct vsycn_ctrl *vctrl;

	if (cndx >= MAX_CONTROLLER) {
		pr_err("%s: out or range: cndx=%d\n", __func__, cndx);
		return;
	}

	vctrl = &vsync_ctrl_db[cndx];
	vctrl->base_pipe = pipe;
}

void mdp4_dsi_video_fxn_register(cmd_fxn_t fxn)
{
	display_on = fxn;
}


int mdp4_dsi_video_on(struct platform_device *pdev)
{
	int dsi_width;
	int dsi_height;
	int dsi_bpp;
	int dsi_border_clr;
	int dsi_underflow_clr;
	int dsi_hsync_skew;

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
	struct msm_panel_info *pinfo;

	vctrl = &vsync_ctrl_db[cndx];
	mfd = (struct msm_fb_data_type *)platform_get_drvdata(pdev);
	pinfo = &mfd->panel_info;

	if (!mfd)
		return -ENODEV;

	if (mfd->key != MFD_KEY)
		return -EINVAL;

	vctrl->mfd = mfd;
	vctrl->dev = mfd->fbi->dev;
	vctrl->blt_ctrl = pinfo->lcd.blt_ctrl;

	/* mdp clock on */
	mdp_clk_ctrl(1);

	fbi = mfd->fbi;
	var = &fbi->var;

	bpp = fbi->var.bits_per_pixel / 8;
	buf = (uint8 *) fbi->fix.smem_start;
	buf_offset = calc_fb_offset(mfd, fbi, bpp);

	if (vctrl->base_pipe == NULL) {
		ptype = mdp4_overlay_format2type(mfd->fb_imgType);
		if (ptype < 0)
			printk(KERN_INFO "%s: format2type failed\n", __func__);
		pipe = mdp4_overlay_pipe_alloc(ptype, MDP4_MIXER0);
		if (pipe == NULL) {
			printk(KERN_INFO "%s: pipe_alloc failed\n", __func__);
			return -EBUSY;
		}
		pipe->pipe_used++;
		pipe->mixer_stage  = MDP4_MIXER_STAGE_BASE;
		pipe->mixer_num  = MDP4_MIXER0;
		pipe->src_format = mfd->fb_imgType;
		mdp4_overlay_panel_mode(pipe->mixer_num, MDP4_PANEL_DSI_VIDEO);
		ret = mdp4_overlay_format2pipe(pipe);
		if (ret < 0)
			printk(KERN_INFO "%s: format2type failed\n", __func__);

		pipe->ov_blt_addr = 0;
		pipe->dma_blt_addr = 0;
		vctrl->base_pipe = pipe; /* keep it */
		mdp4_init_writeback_buf(mfd, MDP4_MIXER0);

	} else {
		pipe = vctrl->base_pipe;
	}
#if defined(CONFIG_FB_MSM_MIPI_LGIT_VIDEO_FHD_INVERSE_PT)
	pipe->mfd = mfd;
#endif

	/* MDP cmd block enable */
	mdp_pipe_ctrl(MDP_CMD_BLOCK, MDP_BLOCK_POWER_ON, FALSE);

	if (!(mfd->cont_splash_done)) {
		mfd->cont_splash_done = 1;
		mdp4_dsi_video_wait4dmap_done(0);
		MDP_OUTP(MDP_BASE + DSI_VIDEO_BASE, 0);
		mipi_dsi_controller_cfg(0);
		/* Clks are enabled in probe.
		   Disabling clocks now */
		mdp_clk_ctrl(0);
	}

	pipe->src_height = fbi->var.yres;
	pipe->src_width = fbi->var.xres;
	pipe->src_h = fbi->var.yres;
	pipe->src_w = fbi->var.xres;
	pipe->src_y = 0;
	pipe->src_x = 0;
	pipe->dst_h = fbi->var.yres;
	pipe->dst_w = fbi->var.xres;
	pipe->srcp0_ystride = fbi->fix.line_length;
	pipe->bpp = bpp;

	if (mfd->display_iova)
		pipe->srcp0_addr = mfd->display_iova + buf_offset;
	else
		pipe->srcp0_addr = (uint32)(buf + buf_offset);

	pipe->dst_h = fbi->var.yres;
	pipe->dst_w = fbi->var.xres;

	mdp4_overlay_mdp_pipe_req(pipe, mfd);

	atomic_set(&vctrl->suspend, 0);

	mdp4_overlay_dmap_xy(pipe);	/* dma_p */
	mdp4_overlay_dmap_cfg(mfd, 1);

	mdp4_overlay_rgb_setup(pipe);

	mdp4_overlayproc_cfg(pipe);

	mdp4_overlay_reg_flush(pipe, 1);

	mdp4_mixer_stage_up(pipe, 0);
	mdp4_mixer_stage_commit(pipe->mixer_num);

	/*
	 * DSI timing setting
	 */
	h_back_porch = var->left_margin;
	h_front_porch = var->right_margin;
	v_back_porch = var->upper_margin;
	v_front_porch = var->lower_margin;
	hsync_pulse_width = var->hsync_len;
	vsync_pulse_width = var->vsync_len;
	dsi_border_clr = mfd->panel_info.lcdc.border_clr;
	dsi_underflow_clr = mfd->panel_info.lcdc.underflow_clr;
	dsi_hsync_skew = mfd->panel_info.lcdc.hsync_skew;
	dsi_width = mfd->panel_info.xres +
		mfd->panel_info.lcdc.xres_pad;
	dsi_height = mfd->panel_info.yres +
		mfd->panel_info.lcdc.yres_pad;
	dsi_bpp = mfd->panel_info.bpp;

	hsync_period = hsync_pulse_width + h_back_porch + dsi_width
				+ h_front_porch;
	hsync_ctrl = (hsync_period << 16) | hsync_pulse_width;
	hsync_start_x = h_back_porch + hsync_pulse_width;
	hsync_end_x = hsync_period - h_front_porch - 1;
	display_hctl = (hsync_end_x << 16) | hsync_start_x;

	vsync_period =
	    (vsync_pulse_width + v_back_porch + dsi_height + v_front_porch);
	display_v_start = ((vsync_pulse_width + v_back_porch) * hsync_period)
				+ dsi_hsync_skew;
	display_v_end =
	  ((vsync_period - v_front_porch) * hsync_period) + dsi_hsync_skew - 1;

	if (dsi_width != var->xres) {
		active_h_start = hsync_start_x + first_pixel_start_x;
		active_h_end = active_h_start + var->xres - 1;
		active_hctl =
		    ACTIVE_START_X_EN | (active_h_end << 16) | active_h_start;
	} else {
		active_hctl = 0;
	}

	if (dsi_height != var->yres) {
		active_v_start =
		    display_v_start + first_pixel_start_y * hsync_period;
		active_v_end = active_v_start + (var->yres) * hsync_period - 1;
		active_v_start |= ACTIVE_START_Y_EN;
	} else {
		active_v_start = 0;
		active_v_end = 0;
	}

	dsi_underflow_clr |= 0x80000000;	/* enable recovery */
	hsync_polarity = 0;
	vsync_polarity = 0;
	data_en_polarity = 0;

	ctrl_polarity =
	    (data_en_polarity << 2) | (vsync_polarity << 1) | (hsync_polarity);

	MDP_OUTP(MDP_BASE + DSI_VIDEO_BASE + 0x4, hsync_ctrl);
	MDP_OUTP(MDP_BASE + DSI_VIDEO_BASE + 0x8, vsync_period * hsync_period);
	MDP_OUTP(MDP_BASE + DSI_VIDEO_BASE + 0xc,
				vsync_pulse_width * hsync_period);
	MDP_OUTP(MDP_BASE + DSI_VIDEO_BASE + 0x10, display_hctl);
	MDP_OUTP(MDP_BASE + DSI_VIDEO_BASE + 0x14, display_v_start);
	MDP_OUTP(MDP_BASE + DSI_VIDEO_BASE + 0x18, display_v_end);
	MDP_OUTP(MDP_BASE + DSI_VIDEO_BASE + 0x1c, active_hctl);
	MDP_OUTP(MDP_BASE + DSI_VIDEO_BASE + 0x20, active_v_start);
	MDP_OUTP(MDP_BASE + DSI_VIDEO_BASE + 0x24, active_v_end);
	MDP_OUTP(MDP_BASE + DSI_VIDEO_BASE + 0x28, dsi_border_clr);
	MDP_OUTP(MDP_BASE + DSI_VIDEO_BASE + 0x2c, dsi_underflow_clr);
	MDP_OUTP(MDP_BASE + DSI_VIDEO_BASE + 0x30, dsi_hsync_skew);
	MDP_OUTP(MDP_BASE + DSI_VIDEO_BASE + 0x38, ctrl_polarity);
	mdp_pipe_ctrl(MDP_CMD_BLOCK, MDP_BLOCK_POWER_OFF, FALSE);

	mdp_histogram_ctrl_all(TRUE);
	mdp4_overlay_dsi_video_start();

	return ret;
}

int mdp4_dsi_video_off(struct platform_device *pdev)
{
	int ret = 0;

/* LGE_CHANGE_S
* for power sequence of lgit panel
* 2012-05-28 jungbeom.shim@lge.com
*/
#if defined(CONFIG_MACH_LGE)
	int retry_cnt = 0;

	#if defined(CONFIG_FB_MSM_MIPI_LGIT_VIDEO_WXGA_PT) || defined(CONFIG_FB_MSM_MIPI_LGIT_VIDEO_FHD_PT) ||\
		defined(CONFIG_FB_MSM_MIPI_LGIT_VIDEO_FHD_INVERSE_PT)
	do {
		ret = mipi_lgit_lcd_off(pdev);

		if (ret < 0) {
			panel_next_off(pdev);
			msleep(2);
			panel_next_on(pdev);
			msleep(5);
			retry_cnt++;
		}
		else
		{
			// if upper routine is successed, need to initialize ret variable.
			ret = 0;
			break;
		}
	} while(retry_cnt < 10);
	printk(KERN_INFO "%s : mipi_lgit_lcd_off retry_cnt = %d\n", __func__, retry_cnt);
	#endif
#endif
/* LGE_CHANGE_E */

	/* MDP cmd block enable */
	mdp_pipe_ctrl(MDP_CMD_BLOCK, MDP_BLOCK_POWER_ON, FALSE);
	mdp4_mixer_pipe_cleanup(dsi_pipe->mixer_num);
	MDP_OUTP(MDP_BASE + DSI_VIDEO_BASE, 0);
	dsi_video_enabled = 0;
	/* MDP cmd block disable */
	mdp_pipe_ctrl(MDP_CMD_BLOCK, MDP_BLOCK_POWER_OFF, FALSE);
	mdp_pipe_ctrl(MDP_OVERLAY0_BLOCK, MDP_BLOCK_POWER_OFF, FALSE);
	mdp_histogram_ctrl_all(FALSE);
#if defined(CONFIG_MACH_LGE)
	#if defined(CONFIG_FB_MSM_MIPI_LGIT_VIDEO_WXGA_PT) || defined(CONFIG_FB_MSM_MIPI_LGIT_VIDEO_FHD_PT) ||\
		defined(CONFIG_FB_MSM_MIPI_LGIT_VIDEO_FHD_INVERSE_PT)
		ret = panel_next_off(pdev);
	#elif defined(CONFIG_FB_MSM_MIPI_HITACHI_VIDEO_HD_PT)
		do {
			ret = panel_next_off(pdev);
			if (ret < 0) {
				panel_next_on(pdev);
				msleep(5);
				retry_cnt++;
			}
			else
			{
				// if upper routine is successed, need to initialize ret variable.
				ret = 0;
				break;
			}
		} while(retry_cnt < 10);
		printk(KERN_INFO "%s : panel_next_off retry_cnt = %d\n", __func__, retry_cnt);
	#endif
#else
	ret = panel_next_off(pdev);
#endif
	/* delay to make sure the last frame finishes */
	msleep(20);

	/* dis-engage rgb0 from mixer0 */
	if (dsi_pipe) {
		mdp4_mixer_stage_down(dsi_pipe,0);
		mdp4_iommu_unmap(dsi_pipe);
	}

	return ret;
}

#ifdef CONFIG_HRZ_II
int hrz_mode(struct platform_device *pdev, int hrz_mode)
{
	uint8 *buf;
	int bpp;
	unsigned int buf_offset;
	struct fb_info *fbi;
	struct msm_fb_data_type *mfd;
	struct mdp4_overlay_pipe *pipe;

	mfd = (struct msm_fb_data_type *)platform_get_drvdata(pdev);

	if (!mfd)
		return -ENODEV;

	if (mfd->key != MFD_KEY)
		return -EINVAL;

	if(dsi_pipe == NULL) return -EINVAL;

	fbi = mfd->fbi;
	pipe = dsi_pipe;

	bpp = fbi->var.bits_per_pixel / 8;
	buf = (uint8 *) fbi->fix.smem_start;
	buf_offset = calc_fb_offset(mfd, fbi, bpp);

	if(hrz_mode) {
		pipe->src_height = fbi->var.yres/2;
		pipe->src_width = fbi->var.xres;
		pipe->src_h = fbi->var.yres/2;
		pipe->src_w = fbi->var.xres;
	} else {
		pipe->src_height = fbi->var.yres;
		pipe->src_width = fbi->var.xres;
		pipe->src_h = fbi->var.yres;
		pipe->src_w = fbi->var.xres;
	}

	pipe->src_y = 0;
	pipe->src_x = 0;
	pipe->srcp0_ystride = fbi->fix.line_length;
	pipe->bpp = fbi->var.bits_per_pixel / 8;

	if (mfd->display_iova)
		pipe->srcp0_addr = mfd->display_iova + buf_offset;
	else
		pipe->srcp0_addr = (uint32)(buf + buf_offset);
#if 0
	if (mfd->map_buffer) {
		pipe->srcp0_addr = (unsigned int)mfd->map_buffer->iova[0] + \
			buf_offset;
		pr_debug("start 0x%lx srcp0_addr 0x%x\n", mfd->
			map_buffer->iova[0], pipe->srcp0_addr);
	} else {
		pipe->srcp0_addr = (uint32)(buf + buf_offset);
	}
#endif
	pipe->dst_h = fbi->var.yres;
	pipe->dst_w = fbi->var.xres;

	mdp4_overlay_rgb_setup(pipe);
	mdp4_overlayproc_cfg(pipe);

	mdp4_overlay_dmap_xy(pipe);	/* dma_p */
	mdp4_overlay_dmap_cfg(mfd, 1);

	mdp4_overlay_reg_flush(pipe, 1);
	mdp4_mixer_stage_up(pipe,1);

	mb();
	/* wait for vsync */
	mdp4_overlay_dsi_video_vsync_push(mfd,pipe);

	return 0;
}
#endif

/* 3D side by side */
void mdp4_dsi_video_3d_sbys(struct msm_fb_data_type *mfd,
				struct msmfb_overlay_3d *r3d)
{
	struct fb_info *fbi;
	struct mdp4_overlay_pipe *pipe;
	unsigned int buf_offset;
	int bpp;
	uint8 *buf = NULL;

	if (dsi_pipe == NULL)
		return;

	dsi_pipe->is_3d = r3d->is_3d;
	dsi_pipe->src_height_3d = r3d->height;
	dsi_pipe->src_width_3d = r3d->width;

	pipe = dsi_pipe;

	if (pipe->is_3d)
		mdp4_overlay_panel_3d(pipe->mixer_num, MDP4_3D_SIDE_BY_SIDE);
	else
		mdp4_overlay_panel_3d(pipe->mixer_num, MDP4_3D_NONE);

	fbi = mfd->fbi;

	bpp = fbi->var.bits_per_pixel / 8;
	buf = (uint8 *) fbi->fix.smem_start;
	buf_offset = calc_fb_offset(mfd, fbi, bpp);

	if (pipe->is_3d) {
		pipe->src_height = pipe->src_height_3d;
		pipe->src_width = pipe->src_width_3d;
		pipe->src_h = pipe->src_height_3d;
		pipe->src_w = pipe->src_width_3d;
		pipe->dst_h = pipe->src_height_3d;
		pipe->dst_w = pipe->src_width_3d;
		pipe->srcp0_ystride = msm_fb_line_length(0,
					pipe->src_width, bpp);
	} else {
		 /* 2D */
		pipe->src_height = fbi->var.yres;
		pipe->src_width = fbi->var.xres;
		pipe->src_h = fbi->var.yres;
		pipe->src_w = fbi->var.xres;
		pipe->dst_h = fbi->var.yres;
		pipe->dst_w = fbi->var.xres;
		pipe->srcp0_ystride = fbi->fix.line_length;
	}

	pipe->src_y = 0;
	pipe->src_x = 0;
	pipe->dst_y = 0;
	pipe->dst_x = 0;

	if (mfd->display_iova)
		pipe->srcp0_addr = mfd->display_iova + buf_offset;
	else
		pipe->srcp0_addr = (uint32)(buf + buf_offset);

	mdp4_overlay_rgb_setup(pipe);

	mdp4_overlayproc_cfg(pipe);

	mdp4_overlay_dmap_xy(pipe);

	mdp4_overlay_dmap_cfg(mfd, 1);

	mdp4_overlay_reg_flush(pipe, 1);

	mdp4_mixer_stage_up(pipe, 0);

	mb();

	/* wait for vsycn */
	mdp4_overlay_dsi_video_vsync_push(mfd, pipe);
}

static void mdp4_dsi_video_blt_ov_update(struct mdp4_overlay_pipe *pipe)
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

static void mdp4_dsi_video_blt_dmap_update(struct mdp4_overlay_pipe *pipe)
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

/*
 * mdp4_overlay_dsi_video_wait4event:
 * INTR_DMA_P_DONE and INTR_PRIMARY_VSYNC event only
 * no INTR_OVERLAY0_DONE event allowed.
 */
static void mdp4_overlay_dsi_video_wait4event(struct msm_fb_data_type *mfd,
						int intr_done)
{
	unsigned long flag;
	unsigned int data;
	unsigned long vsync_interval;

	data = inpdw(MDP_BASE + DSI_VIDEO_BASE);
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
	INIT_COMPLETION(dsi_video_comp);
	mfd->dma->waiting = TRUE;
	outp32(MDP_INTR_CLEAR, intr_done);
	mdp_intr_mask |= intr_done;
	outp32(MDP_INTR_ENABLE, mdp_intr_mask);
	mdp_enable_irq(MDP_DMA2_TERM);  /* enable intr */
	spin_unlock_irqrestore(&mdp_spin_lock, flag);
	wait_for_completion(&dsi_video_comp);
	mdp_disable_irq(MDP_DMA2_TERM);
}

static void mdp4_overlay_dsi_video_dma_busy_wait(struct msm_fb_data_type *mfd)
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

void mdp4_overlay_dsi_video_start(void)
{
	if (!dsi_video_enabled) {
		/* enable DSI block */
		mdp4_iommu_attach();
		mdp_pipe_ctrl(MDP_CMD_BLOCK, MDP_BLOCK_POWER_ON, FALSE);
		MDP_OUTP(MDP_BASE + DSI_VIDEO_BASE, 1);
		mdp_pipe_ctrl(MDP_CMD_BLOCK, MDP_BLOCK_POWER_OFF, FALSE);
		dsi_video_enabled = 1;
	}
}

void mdp4_overlay_dsi_video_vsync_push(struct msm_fb_data_type *mfd,
			struct mdp4_overlay_pipe *pipe)
{
	unsigned long flag;

	if (pipe->flags & MDP_OV_PLAY_NOWAIT)
		return;

	if (dsi_pipe->ov_blt_addr) {
		mdp4_overlay_dsi_video_dma_busy_wait(mfd);

		mdp4_dsi_video_blt_ov_update(dsi_pipe);
		dsi_pipe->ov_cnt++;

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
		mdp4_overlay_dsi_video_wait4event(mfd, INTR_DMA_P_DONE);
	} else {
		mdp4_overlay_dsi_video_wait4event(mfd, INTR_PRIMARY_VSYNC);
	}

	mdp4_set_perf_level();
}

/*
 * mdp4_primary_vsync_dsi_video: called from isr
 */
void mdp4_primary_vsync_dsi_video(void)
{
	complete_all(&dsi_video_comp);
	last_vsync_time_ns = ktime_get();
	/* Release Wakelock */
	if (wake_lock_active(&mdp_idle_wakelock))
		wake_unlock(&mdp_idle_wakelock);
}

 /*
 * mdp4_dma_p_done_dsi_video: called from isr
 */
void mdp4_dma_p_done_dsi_video(struct mdp_dma_data *dma)
{
	if (blt_cfg_changed) {
		mdp_is_in_isr = TRUE;
		if (dsi_pipe->ov_blt_addr) {
			mdp4_overlay_dmap_xy(dsi_pipe);
			mdp4_overlayproc_cfg(dsi_pipe);
		} else {
			mdp4_overlayproc_cfg(dsi_pipe);
			mdp4_overlay_dmap_xy(dsi_pipe);
		}
		mdp_is_in_isr = FALSE;
		if (dsi_pipe->ov_blt_addr) {
			mdp4_dsi_video_blt_ov_update(dsi_pipe);
			dsi_pipe->ov_cnt++;
			outp32(MDP_INTR_CLEAR, INTR_OVERLAY0_DONE);
			mdp_intr_mask |= INTR_OVERLAY0_DONE;
			outp32(MDP_INTR_ENABLE, mdp_intr_mask);
			dma->busy = TRUE;
			mdp_enable_irq(MDP_OVERLAY0_TERM);
			/* kickoff overlay engine */
			outpdw(MDP_BASE + 0x0004, 0);
		}
		blt_cfg_changed = 0;
	}
	complete_all(&dsi_video_comp);
	last_vsync_time_ns = ktime_get();
	/*  Release Wakelock */
	if (wake_lock_active(&mdp_idle_wakelock))
		wake_unlock(&mdp_idle_wakelock);
}

/*
 * mdp4_overlay1_done_dsi: called from isr
 */
void mdp4_overlay0_done_dsi_video(struct mdp_dma_data *dma)
{
	spin_lock(&mdp_spin_lock);
	dma->busy = FALSE;
	if (dsi_pipe->ov_blt_addr == 0) {
		spin_unlock(&mdp_spin_lock);
		return;
	}
	mdp4_dsi_video_blt_dmap_update(dsi_pipe);
	dsi_pipe->dmap_cnt++;
	mdp_disable_irq_nosync(MDP_OVERLAY0_TERM);
	spin_unlock(&mdp_spin_lock);
	complete(&dma->comp);
}
#if defined(CONFIG_FB_MSM_MIPI_LGIT_VIDEO_FHD_PT) ||\
	defined(CONFIG_FB_MSM_MIPI_LGIT_VIDEO_FHD_INVERSE_PT)
	// fixed the underrun for 1920 * 1080 video play at the full hd lcd.
static void mdp4_overlay_dsi_video_prefill(struct msm_fb_data_type *mfd) 
{ 
	unsigned long flag; 

	if (dsi_pipe->ov_blt_addr) {
		mdp4_overlay_dsi_video_dma_busy_wait(mfd); 
		mdp4_dsi_video_blt_ov_update(dsi_pipe);
		dsi_pipe->ov_cnt++; 
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
#endif
/*
 * make sure the MIPI_DSI_WRITEBACK_SIZE defined at boardfile
 * has enough space h * w * 3 * 2
 */
static void mdp4_dsi_video_do_blt(struct msm_fb_data_type *mfd, int enable)
{
	unsigned long flag;
	int data;
	int change = 0;

	mdp4_allocate_writeback_buf(mfd, MDP4_MIXER0);

	if (mfd->ov0_wb_buf->write_addr == 0) {
		pr_info("%s: no blt_base assigned\n", __func__);
		return;
	}

	spin_lock_irqsave(&mdp_spin_lock, flag);
	if (enable && dsi_pipe->ov_blt_addr == 0) {
		dsi_pipe->ov_blt_addr = mfd->ov0_wb_buf->write_addr;
		dsi_pipe->dma_blt_addr = mfd->ov0_wb_buf->read_addr;
		dsi_pipe->blt_cnt = 0;
		dsi_pipe->ov_cnt = 0;
		dsi_pipe->dmap_cnt = 0;
		mdp4_stat.blt_dsi_video++;
		change++;
	} else if (enable == 0 && dsi_pipe->ov_blt_addr) {
		dsi_pipe->ov_blt_addr = 0;
		dsi_pipe->dma_blt_addr = 0;
		change++;
	}
#if defined(CONFIG_FB_MSM_MIPI_LGIT_VIDEO_FHD_PT)||\
	defined(CONFIG_FB_MSM_MIPI_LGIT_VIDEO_FHD_INVERSE_PT)
	 // fixed the underrun for 1920 * 1080 video play at the full hd lcd.
    pr_debug("%s: enable=%d ov_blt_addr=%x\n", __func__, enable, (int)dsi_pipe->ov_blt_addr); 
	spin_unlock_irqrestore(&mdp_spin_lock, flag); 

	if (!change)
		return; 
/*
* may need mutex here to sync with whom dsiable
* timing generator
*/
	data = inpdw(MDP_BASE + DSI_VIDEO_BASE); 
	data &= 0x01; 
	if (data) {	/* timing generator enabled */ 
		mdp4_overlay_dsi_video_wait4event(mfd, INTR_DMA_P_DONE); 
		MDP_OUTP(MDP_BASE + DSI_VIDEO_BASE, 0); 
		msleep(20);	/* make sure last frame is finished */ 
		mipi_dsi_controller_cfg(0);
	}
	
	mdp4_overlayproc_cfg(dsi_pipe);
	mdp4_overlay_dmap_xy(dsi_pipe); 

	if (data) {	/* timing generator enabled */ 
		if (dsi_pipe->ov_blt_addr) {
			MDP_OUTP(MDP_BASE + DSI_VIDEO_BASE, 1); 
			mdp4_overlay_dsi_video_prefill(mfd);
			mdp4_overlay_dsi_video_prefill(mfd);
			MDP_OUTP(MDP_BASE + DSI_VIDEO_BASE, 0); 
			} 
	mipi_dsi_sw_reset();
	mipi_dsi_controller_cfg(1);
	MDP_OUTP(MDP_BASE + DSI_VIDEO_BASE, 1); 
	} 
#else
	if (!change) {
		spin_unlock_irqrestore(&mdp_spin_lock, flag);
		return;
	}

	pr_debug("%s: enable=%d ov_blt_addr=%x\n", __func__,
			enable, (int)dsi_pipe->ov_blt_addr);
	blt_cfg_changed = 1;

	spin_unlock_irqrestore(&mdp_spin_lock, flag);


	/*
	 * may need mutex here to sync with whom dsiable
	 * timing generator
	 */
	data = inpdw(MDP_BASE + DSI_VIDEO_BASE);
	data &= 0x01;
	if (data) {	/* timing generator enabled */
		mdp4_overlay_dsi_video_wait4event(mfd, INTR_DMA_P_DONE);
		msleep(20);
	}
#endif


}

int mdp4_dsi_video_overlay_blt_offset(struct msm_fb_data_type *mfd,
					struct msmfb_overlay_blt *req)
{
	req->offset = 0;
	req->width = dsi_pipe->src_width;
	req->height = dsi_pipe->src_height;
	req->bpp = dsi_pipe->bpp;

	return sizeof(*req);
}

void mdp4_dsi_video_overlay_blt(struct msm_fb_data_type *mfd,
					struct msmfb_overlay_blt *req)
{
	mdp4_dsi_video_do_blt(mfd, req->enable);
}

void mdp4_dsi_video_blt_start(struct msm_fb_data_type *mfd)
{
	mdp4_dsi_video_do_blt(mfd, 1);
}

void mdp4_dsi_video_blt_stop(struct msm_fb_data_type *mfd)
{
	mdp4_dsi_video_do_blt(mfd, 0);
}

void mdp4_dsi_video_overlay(struct msm_fb_data_type *mfd)
{
	struct fb_info *fbi = mfd->fbi;
	uint8 *buf;
	unsigned int buf_offset;
	int bpp;
	struct mdp4_overlay_pipe *pipe;

	if (!mfd->panel_power_on)
		return;

	/* no need to power on cmd block since it's dsi video mode */
	bpp = fbi->var.bits_per_pixel / 8;
	buf = (uint8 *) fbi->fix.smem_start;
	buf_offset = calc_fb_offset(mfd, fbi, bpp);

	mutex_lock(&mfd->dma->ov_mutex);

	pipe = dsi_pipe;

	if (mfd->display_iova)
		pipe->srcp0_addr = mfd->display_iova + buf_offset;
	else
		pipe->srcp0_addr = (uint32)(buf + buf_offset);

	mdp4_overlay_rgb_setup(pipe);
	mdp4_overlay_reg_flush(pipe, 0);
	mdp4_mixer_stage_up(pipe,0);
	mdp4_overlay_dsi_video_start();
	mdp4_overlay_dsi_video_vsync_push(mfd, pipe);
	mdp4_iommu_unmap(pipe);
	mutex_unlock(&mfd->dma->ov_mutex);
}
