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
#include <asm/system.h>
#include <asm/mach-types.h>
#include <mach/hardware.h>

#include "mdp.h"
#include "msm_fb.h"
#include "mdp4.h"
#include "mipi_dsi.h"

static struct mdp4_overlay_pipe *dsi_pipe;
static struct msm_fb_data_type *dsi_mfd;
static int busy_wait_cnt;
static int dsi_state;
static unsigned long  tout_expired;

#define TOUT_PERIOD	HZ	/* 1 second */
#define MS_100		(HZ/10)	/* 100 ms */

static int vsync_start_y_adjust = 4;

struct timer_list dsi_clock_timer;
#define MAX_CONTROLLER	1
#define VSYNC_EXPIRE_TICK 4

static struct vsycn_ctrl {
	struct device *dev;
	int inited;
	int update_ndx;
	int expire_tick;
	int blt_wait;
	u32 ov_koff;
	u32 ov_done;
	u32 dmap_koff;
	u32 dmap_done;
	uint32 rdptr_intr_tot;
	uint32 rdptr_sirq_tot;
	atomic_t suspend;
	atomic_t vsync_resume;
	int wait_vsync_cnt;
	int blt_change;
	int blt_free;
	int blt_end;
	int sysfs_created;
	struct mutex update_lock;
	struct completion ov_comp;
	struct completion dmap_comp;
	struct completion vsync_comp;
	spinlock_t spin_lock;
	struct msm_fb_data_type *mfd;
	struct mdp4_overlay_pipe *base_pipe;
	struct vsync_update vlist[2];
	int vsync_enabled;
	int clk_enabled;
	int clk_control;
	int new_update;
	ktime_t vsync_time;
	struct work_struct clk_work;
} vsync_ctrl_db[MAX_CONTROLLER];

static void vsync_irq_enable(int intr, int term)
{
	unsigned long flag;

	spin_lock_irqsave(&mdp_spin_lock, flag);
	/* no need to clrear other interrupts for comamnd mode */
	mdp_intr_mask |= intr;
	outp32(MDP_INTR_ENABLE, mdp_intr_mask);
	mdp_enable_irq(term);
	spin_unlock_irqrestore(&mdp_spin_lock, flag);
}

static void vsync_irq_disable(int intr, int term)
{
	unsigned long flag;

	spin_lock_irqsave(&mdp_spin_lock, flag);
	/* no need to clrear other interrupts for comamnd mode */
	mdp_intr_mask &= ~intr;
	outp32(MDP_INTR_ENABLE, mdp_intr_mask);
	mdp_disable_irq_nosync(term);
	spin_unlock_irqrestore(&mdp_spin_lock, flag);
}

static void mdp4_dsi_cmd_blt_ov_update(struct mdp4_overlay_pipe *pipe)
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

static void dsi_clock_tout(unsigned long data)
{
	if (mipi_dsi_clk_on) {
		if (dsi_state == ST_DSI_PLAYING) {
			mipi_dsi_turn_off_clks();
			mdp4_overlay_dsi_state_set(ST_DSI_CLK_OFF);
		}
	}
}

static void mdp4_dsi_cmd_blt_dmap_update(struct mdp4_overlay_pipe *pipe)
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

static void mdp4_dsi_cmd_wait4dmap(int cndx);
static void mdp4_dsi_cmd_wait4ov(int cndx);

static void mdp4_dsi_cmd_do_blt(struct msm_fb_data_type *mfd, int enable)
{
	unsigned long flags;
	int cndx = 0;
	struct vsycn_ctrl *vctrl;
	struct mdp4_overlay_pipe *pipe;
	int need_wait;

	vctrl = &vsync_ctrl_db[cndx];
	pipe = vctrl->base_pipe;

	mdp4_allocate_writeback_buf(mfd, MDP4_MIXER0);

	if (mfd->ov0_wb_buf->write_addr == 0) {
		pr_err("%s: no blt_base assigned\n", __func__);
		return;
	}

	spin_lock_irqsave(&vctrl->spin_lock, flags);
	if (enable && pipe->ov_blt_addr == 0) {
		vctrl->blt_change++;
		if (vctrl->dmap_koff != vctrl->dmap_done) {
			INIT_COMPLETION(vctrl->dmap_comp);
			need_wait = 1;
		}
	} else if (enable == 0 && pipe->ov_blt_addr) {
		vctrl->blt_change++;
		if (vctrl->ov_koff != vctrl->dmap_done) {
			INIT_COMPLETION(vctrl->dmap_comp);
			need_wait = 1;
		}
	}
	spin_unlock_irqrestore(&vctrl->spin_lock, flags);

	if (need_wait)
		mdp4_dsi_cmd_wait4dmap(0);

	spin_lock_irqsave(&vctrl->spin_lock, flags);
	if (enable && pipe->ov_blt_addr == 0) {
		pipe->ov_blt_addr = mfd->ov0_wb_buf->write_addr;
		pipe->dma_blt_addr = mfd->ov0_wb_buf->read_addr;
		pipe->ov_cnt = 0;
		pipe->dmap_cnt = 0;
		vctrl->ov_koff = vctrl->dmap_koff;
		vctrl->ov_done = vctrl->dmap_done;
		vctrl->blt_free = 0;
		vctrl->blt_wait = 0;
		vctrl->blt_end = 0;
		mdp4_stat.blt_dsi_video++;
	} else if (enable == 0 && pipe->ov_blt_addr) {
		pipe->ov_blt_addr = 0;
		pipe->dma_blt_addr =  0;
		vctrl->blt_end = 1;
		vctrl->blt_free = 4;	/* 4 commits to free wb buf */
	}

	pr_debug("%s: changed=%d enable=%d ov_blt_addr=%x\n", __func__,
		vctrl->blt_change, enable, (int)pipe->ov_blt_addr);

	spin_unlock_irqrestore(&vctrl->spin_lock, flags);
}

/*
 * mdp4_dsi_cmd_do_update:
 * called from thread context
 */
void mdp4_dsi_cmd_pipe_queue(int cndx, struct mdp4_overlay_pipe *pipe)
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

	pr_debug("%s: vndx=%d pipe_ndx=%d expire=%x pid=%d\n", __func__,
		undx, pipe->pipe_ndx, vctrl->expire_tick, current->pid);

	*pp = *pipe;	/* clone it */
	vp->update_cnt++;

	mutex_unlock(&vctrl->update_lock);
	mdp4_stat.overlay_play[pipe->mixer_num]++;
}

static void mdp4_dsi_cmd_blt_ov_update(struct mdp4_overlay_pipe *pipe);

int mdp4_dsi_cmd_pipe_commit(int cndx, int wait)
{
	int  i, undx;
	int mixer = 0;
	struct vsycn_ctrl *vctrl;
	struct vsync_update *vp;
	struct mdp4_overlay_pipe *pipe;
	struct mdp4_overlay_pipe *real_pipe;
	unsigned long flags;
	int need_dmap_wait = 0;
	int need_ov_wait = 0;
	int cnt = 0;

	vctrl = &vsync_ctrl_db[0];

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
	if (pipe->ov_blt_addr) {
		/* Blt */
		if (vctrl->blt_wait)
			need_dmap_wait = 1;
		if (vctrl->ov_koff != vctrl->ov_done) {
			INIT_COMPLETION(vctrl->ov_comp);
			need_ov_wait = 1;
		}
	} else {
		/* direct out */
		if (vctrl->dmap_koff != vctrl->dmap_done) {
			INIT_COMPLETION(vctrl->dmap_comp);
			pr_debug("%s: wait, ok=%d od=%d dk=%d dd=%d cpu=%d\n",
			 __func__, vctrl->ov_koff, vctrl->ov_done,
			vctrl->dmap_koff, vctrl->dmap_done, smp_processor_id());
			need_dmap_wait = 1;
		}
	}
	spin_unlock_irqrestore(&vctrl->spin_lock, flags);

	if (need_dmap_wait) {
		pr_debug("%s: wait4dmap\n", __func__);
		mdp4_dsi_cmd_wait4dmap(0);
	}

	if (need_ov_wait) {
		pr_debug("%s: wait4ov\n", __func__);
		mdp4_dsi_cmd_wait4ov(0);
	}

	if (pipe->ov_blt_addr) {
		if (vctrl->blt_end) {
			vctrl->blt_end = 0;
			pipe->ov_blt_addr = 0;
			pipe->dma_blt_addr =  0;
		}
	}

	if (vctrl->blt_change) {
		mdp4_overlayproc_cfg(pipe);
		mdp4_overlay_dmap_xy(pipe);
		vctrl->blt_change = 0;
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

	/* tx dcs command if had any */
	mipi_dsi_cmdlist_commit(1);

	mdp4_mixer_stage_commit(mixer);

	pipe = vctrl->base_pipe;
	spin_lock_irqsave(&vctrl->spin_lock, flags);
	if (pipe->ov_blt_addr) {
		mdp4_dsi_cmd_blt_ov_update(pipe);
		pipe->ov_cnt++;
		vctrl->ov_koff++;
		vsync_irq_enable(INTR_OVERLAY0_DONE, MDP_OVERLAY0_TERM);
	} else {
		vsync_irq_enable(INTR_DMA_P_DONE, MDP_DMAP_TERM);
		vctrl->dmap_koff++;
	}
	pr_debug("%s: kickoff\n", __func__);
	/* kickoff overlay engine */
	mdp4_stat.kickoff_ov0++;
	outpdw(MDP_BASE + 0x0004, 0);
	mb(); /* make sure kickoff ececuted */
	spin_unlock_irqrestore(&vctrl->spin_lock, flags);

	mdp4_stat.overlay_commit[pipe->mixer_num]++;

	if (wait) {
		long long tick;

		mdp4_dsi_cmd_wait4vsync(0, &tick);
	}

	return cnt;
}

static void mdp4_overlay_update_dsi_cmd(struct msm_fb_data_type *mfd);

void mdp4_dsi_cmd_vsync_ctrl(struct fb_info *info, int enable)
{
	struct vsycn_ctrl *vctrl;
	unsigned long flags;
	int clk_set_on = 0;
	int cndx = 0;

	vctrl = &vsync_ctrl_db[cndx];

	pr_debug("%s: clk_enabled=%d vsycn_enabeld=%d req=%d\n", __func__,
		vctrl->clk_enabled, vctrl->vsync_enabled, enable);

	mutex_lock(&vctrl->update_lock);

	if (vctrl->vsync_enabled == enable) {
		mutex_unlock(&vctrl->update_lock);
		return;
	}

	vctrl->vsync_enabled = enable;

	if (enable) {
		if (vctrl->clk_enabled == 0) {
			pr_debug("%s: SET_CLK_ON\n", __func__);
			mipi_dsi_clk_cfg(1);
			mdp_clk_ctrl(1);
			vctrl->clk_enabled = 1;
			clk_set_on = 1;
		}
		spin_lock_irqsave(&vctrl->spin_lock, flags);
		vctrl->clk_control = 0;
		vctrl->expire_tick = 0;
		vctrl->new_update = 1;
		if (clk_set_on) {
			vsync_irq_enable(INTR_PRIMARY_RDPTR,
						MDP_PRIM_RDPTR_TERM);
		}
		spin_unlock_irqrestore(&vctrl->spin_lock, flags);
	} else {
		spin_lock_irqsave(&vctrl->spin_lock, flags);
		vctrl->clk_control = 1;
		if (vctrl->clk_enabled)
			vctrl->expire_tick = VSYNC_EXPIRE_TICK;
		spin_unlock_irqrestore(&vctrl->spin_lock, flags);
	}
	mutex_unlock(&vctrl->update_lock);

	if (vctrl->vsync_enabled &&  atomic_read(&vctrl->suspend) == 0)
		atomic_set(&vctrl->vsync_resume, 1);
}

void mdp4_dsi_cmd_wait4vsync(int cndx, long long *vtime)
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

	spin_lock_irqsave(&vctrl->spin_lock, flags);
	if (vctrl->wait_vsync_cnt == 0)
		INIT_COMPLETION(vctrl->vsync_comp);
	vctrl->wait_vsync_cnt++;
	spin_unlock_irqrestore(&vctrl->spin_lock, flags);

	wait_for_completion(&vctrl->vsync_comp);
	mdp4_stat.wait4vsync0++;

	*vtime = ktime_to_ns(vctrl->vsync_time);
}

static void mdp4_dsi_cmd_wait4dmap(int cndx)
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

static void mdp4_dsi_cmd_wait4ov(int cndx)
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

/*
 * primary_rdptr_isr:
 * called from interrupt context
 */

static void primary_rdptr_isr(int cndx)
{
	struct vsycn_ctrl *vctrl;

	vctrl = &vsync_ctrl_db[cndx];
	pr_debug("%s: ISR, cpu=%d\n", __func__, smp_processor_id());
	vctrl->rdptr_intr_tot++;
	vctrl->vsync_time = ktime_get();

	spin_lock(&vctrl->spin_lock);

	complete_all(&vctrl->vsync_comp);
	vctrl->wait_vsync_cnt = 0;

	if (vctrl->expire_tick) {
		vctrl->expire_tick--;
		if (vctrl->expire_tick == 0)
			schedule_work(&vctrl->clk_work);
	}
	spin_unlock(&vctrl->spin_lock);
}

void mdp4_dmap_done_dsi_cmd(int cndx)
{
	struct vsycn_ctrl *vctrl;
	struct mdp4_overlay_pipe *pipe;
	int diff;

	vctrl = &vsync_ctrl_db[cndx];
	pipe = vctrl->base_pipe;

	 /* blt enabled */
	spin_lock(&vctrl->spin_lock);
	vsync_irq_disable(INTR_DMA_P_DONE, MDP_DMAP_TERM);
	vctrl->dmap_done++;
	diff = vctrl->ov_done - vctrl->dmap_done;
	pr_debug("%s: ov_koff=%d ov_done=%d dmap_koff=%d dmap_done=%d cpu=%d\n",
		__func__, vctrl->ov_koff, vctrl->ov_done, vctrl->dmap_koff,
		vctrl->dmap_done, smp_processor_id());
	complete_all(&vctrl->dmap_comp);
	if (diff <= 0) {
		if (vctrl->blt_wait)
			vctrl->blt_wait = 0;
		spin_unlock(&vctrl->spin_lock);
		return;
	}

	/* kick dmap */
	mdp4_dsi_cmd_blt_dmap_update(pipe);
	pipe->dmap_cnt++;
	mdp4_stat.kickoff_dmap++;
	vctrl->dmap_koff++;
	vsync_irq_enable(INTR_DMA_P_DONE, MDP_DMAP_TERM);
	outpdw(MDP_BASE + 0x000c, 0); /* kickoff dmap engine */
	mb(); /* make sure kickoff executed */
	spin_unlock(&vctrl->spin_lock);
}

/*
 * mdp4_overlay0_done_dsi_cmd: called from isr
 */
void mdp4_overlay0_done_dsi_cmd(int cndx)
{
	struct vsycn_ctrl *vctrl;
	struct mdp4_overlay_pipe *pipe;
	int diff;

	vctrl = &vsync_ctrl_db[cndx];
	pipe = vctrl->base_pipe;

	spin_lock(&vctrl->spin_lock);
	vsync_irq_disable(INTR_OVERLAY0_DONE, MDP_OVERLAY0_TERM);
	vctrl->ov_done++;
	complete_all(&vctrl->ov_comp);
	diff = vctrl->ov_done - vctrl->dmap_done;

	pr_debug("%s: ov_koff=%d ov_done=%d dmap_koff=%d dmap_done=%d cpu=%d\n",
		__func__, vctrl->ov_koff, vctrl->ov_done, vctrl->dmap_koff,
		vctrl->dmap_done, smp_processor_id());

	if (pipe->ov_blt_addr == 0) {
		/* blt disabled */
		spin_unlock(&vctrl->spin_lock);
		return;
	}

	if (diff > 1) {
		/*
		 * two overlay_done and none dmap_done yet
		 * let dmap_done kickoff dmap
		 * and put pipe_commit to wait
		 */
		vctrl->blt_wait = 1;
		pr_debug("%s: blt_wait set\n", __func__);
		spin_unlock(&vctrl->spin_lock);
		return;
	}
	mdp4_dsi_cmd_blt_dmap_update(pipe);
	pipe->dmap_cnt++;
	mdp4_stat.kickoff_dmap++;
	vctrl->dmap_koff++;
	vsync_irq_enable(INTR_DMA_P_DONE, MDP_DMAP_TERM);
	outpdw(MDP_BASE + 0x000c, 0); /* kickoff dmap engine */
	mb(); /* make sure kickoff executed */
	spin_unlock(&vctrl->spin_lock);
}

static void clk_ctrl_work(struct work_struct *work)
{
	struct vsycn_ctrl *vctrl =
		container_of(work, typeof(*vctrl), clk_work);
	unsigned long flags;

	mutex_lock(&vctrl->update_lock);
	if (vctrl->clk_control && vctrl->clk_enabled) {
		pr_debug("%s: SET_CLK_OFF\n", __func__);
		mdp_clk_ctrl(0);
		mipi_dsi_clk_cfg(0);
		spin_lock_irqsave(&vctrl->spin_lock, flags);
		vsync_irq_disable(INTR_PRIMARY_RDPTR, MDP_PRIM_RDPTR_TERM);
		vctrl->clk_enabled = 0;
		vctrl->clk_control = 0;
		spin_unlock_irqrestore(&vctrl->spin_lock, flags);
	}
	mutex_unlock(&vctrl->update_lock);
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

void mdp4_dsi_rdptr_init(int cndx, struct msm_fb_data_type *mfd)
{
	struct vsycn_ctrl *vctrl;
	int ret;

	if (cndx >= MAX_CONTROLLER) {
		pr_err("%s: out or range: cndx=%d\n", __func__, cndx);
		return;
	}

	vctrl = &vsync_ctrl_db[cndx];
	if (vctrl->inited)
		return;

	vctrl->mfd = mfd;
	vctrl->dev = mfd->fbi->dev;
	vctrl->inited = 1;
	vctrl->update_ndx = 0;
	mutex_init(&vctrl->update_lock);
	init_completion(&vctrl->ov_comp);
	init_completion(&vctrl->dmap_comp);
	init_completion(&vctrl->vsync_comp);
	spin_lock_init(&vctrl->spin_lock);
	atomic_set(&vctrl->suspend, 1);
	atomic_set(&vctrl->vsync_resume, 1);
	INIT_WORK(&vctrl->clk_work, clk_ctrl_work);
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

void mdp4_primary_rdptr(void)
{
	primary_rdptr_isr(0);
}

void mdp4_overlay_dsi_state_set(int state)
{
	unsigned long flag;

	spin_lock_irqsave(&mdp_spin_lock, flag);
	dsi_state = state;
	spin_unlock_irqrestore(&mdp_spin_lock, flag);
}

int mdp4_overlay_dsi_state_get(void)
{
	return dsi_state;
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

void mdp4_mipi_vsync_enable(struct msm_fb_data_type *mfd,
		struct mdp4_overlay_pipe *pipe, int which)
{
	uint32 start_y, data, tear_en;

	tear_en = (1 << which);

	if ((mfd->use_mdp_vsync) && (mfd->ibuf.vsync_enable) &&
		(mfd->panel_info.lcd.vsync_enable)) {

		if (vsync_start_y_adjust <= pipe->dst_y)
			start_y = pipe->dst_y - vsync_start_y_adjust;
		else
			start_y = (mfd->total_lcd_lines - 1) -
				(vsync_start_y_adjust - pipe->dst_y);
		if (which == 0)
			MDP_OUTP(MDP_BASE + 0x210, start_y);	/* primary */
		else
			MDP_OUTP(MDP_BASE + 0x214, start_y);	/* secondary */

		data = inpdw(MDP_BASE + 0x20c);
		data |= tear_en;
		MDP_OUTP(MDP_BASE + 0x20c, data);
	} else {
		data = inpdw(MDP_BASE + 0x20c);
		data &= ~tear_en;
		MDP_OUTP(MDP_BASE + 0x20c, data);
	}
}

void mdp4_overlay_update_dsi_cmd(struct msm_fb_data_type *mfd)
{
	MDPIBUF *iBuf = &mfd->ibuf;
	uint8 *src;
	int ptype;
	struct mdp4_overlay_pipe *pipe;
	int bpp;
	int ret;

	if (mfd->key != MFD_KEY)
		return;

	dsi_mfd = mfd;		/* keep it */

	/* MDP cmd block enable */
	mdp_pipe_ctrl(MDP_CMD_BLOCK, MDP_BLOCK_POWER_ON, FALSE);

	if (dsi_pipe == NULL) {
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
		mdp4_overlay_panel_mode(pipe->mixer_num, MDP4_PANEL_DSI_CMD);
		ret = mdp4_overlay_format2pipe(pipe);
		if (ret < 0)
			printk(KERN_INFO "%s: format2type failed\n", __func__);

		init_timer(&dsi_clock_timer);
		dsi_clock_timer.function = dsi_clock_tout;
		dsi_clock_timer.data = (unsigned long) mfd;;
		dsi_clock_timer.expires = 0xffffffff;
		add_timer(&dsi_clock_timer);
		tout_expired = jiffies;

		dsi_pipe = pipe; /* keep it */

		mdp4_init_writeback_buf(mfd, MDP4_MIXER0);
		pipe->ov_blt_addr = 0;
		pipe->dma_blt_addr = 0;

	} else {
		pipe = dsi_pipe;
	}
	/*
	 * configure dsi stream id
	 * dma_p = 0, dma_s = 1
	 */
	MDP_OUTP(MDP_BASE + 0x000a0, 0x10);
	/* disable dsi trigger */
	MDP_OUTP(MDP_BASE + 0x000a4, 0x00);
	/* whole screen for base layer */
	src = (uint8 *) iBuf->buf;


	{
		struct fb_info *fbi;

		fbi = mfd->fbi;
		if (pipe->is_3d) {
			bpp = fbi->var.bits_per_pixel / 8;
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
		pipe->srcp0_addr = (uint32)src;
	}


	mdp4_overlay_rgb_setup(pipe);

	mdp4_overlay_reg_flush(pipe, 1);

	mdp4_mixer_stage_up(pipe, 0);

	mdp4_overlayproc_cfg(pipe);

	mdp4_overlay_dmap_xy(pipe);

	mdp4_overlay_dmap_cfg(mfd, 0);

	mdp4_mipi_vsync_enable(mfd, pipe, 0);

	/* MDP cmd block disable */
	mdp_pipe_ctrl(MDP_CMD_BLOCK, MDP_BLOCK_POWER_OFF, FALSE);

	wmb();
}

/* 3D side by side */
void mdp4_dsi_cmd_3d_sbys(struct msm_fb_data_type *mfd,
				struct msmfb_overlay_3d *r3d)
{
	struct fb_info *fbi;
	struct mdp4_overlay_pipe *pipe;
	int bpp;
	uint8 *src = NULL;

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

	if (mfd->panel_power_on) {
		mdp4_dsi_cmd_dma_busy_wait(mfd);
		mdp4_dsi_blt_dmap_busy_wait(mfd);
	}

	fbi = mfd->fbi;
	if (pipe->is_3d) {
		bpp = fbi->var.bits_per_pixel / 8;
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
	pipe->srcp0_addr = (uint32)src;

	mdp_pipe_ctrl(MDP_CMD_BLOCK, MDP_BLOCK_POWER_ON, FALSE);

	mdp4_overlay_rgb_setup(pipe);

	mdp4_overlay_reg_flush(pipe, 1);

	mdp4_mixer_stage_up(pipe, 0);

	mdp4_overlayproc_cfg(pipe);

	mdp4_overlay_dmap_xy(pipe);

	mdp4_overlay_dmap_cfg(mfd, 0);

	/* MDP cmd block disable */
	mdp_pipe_ctrl(MDP_CMD_BLOCK, MDP_BLOCK_POWER_OFF, FALSE);
}


void mdp4_dsi_cmd_blt_start(struct msm_fb_data_type *mfd)
{
	mdp4_dsi_cmd_do_blt(mfd, 1);
}

void mdp4_dsi_cmd_blt_stop(struct msm_fb_data_type *mfd)
{
	mdp4_dsi_cmd_do_blt(mfd, 0);
}

void mdp4_dsi_cmd_overlay_blt(struct msm_fb_data_type *mfd,
					struct msmfb_overlay_blt *req)
{
	mdp4_dsi_cmd_do_blt(mfd, req->enable);
}


int mdp4_dsi_overlay_blt_start(struct msm_fb_data_type *mfd)
{
	unsigned long flag;

	pr_debug("%s: blt_end=%d ov_blt_addr=%x pid=%d\n",
	__func__, dsi_pipe->blt_end, (int)dsi_pipe->ov_blt_addr, current->pid);

	mdp4_allocate_writeback_buf(mfd, MDP4_MIXER0);

	if (mfd->ov0_wb_buf->write_addr == 0) {
		pr_info("%s: no blt_base assigned\n", __func__);
		return -EBUSY;
	}

	if (dsi_pipe->ov_blt_addr == 0) {
		mdp4_dsi_cmd_dma_busy_wait(mfd);
		spin_lock_irqsave(&mdp_spin_lock, flag);
		dsi_pipe->blt_end = 0;
		dsi_pipe->blt_cnt = 0;
		dsi_pipe->ov_cnt = 0;
		dsi_pipe->dmap_cnt = 0;
		dsi_pipe->ov_blt_addr = mfd->ov0_wb_buf->write_addr;
		dsi_pipe->dma_blt_addr = mfd->ov0_wb_buf->read_addr;
		mdp4_stat.blt_dsi_cmd++;
		spin_unlock_irqrestore(&mdp_spin_lock, flag);
		return 0;
	}

	return -EBUSY;
}

int mdp4_dsi_overlay_blt_stop(struct msm_fb_data_type *mfd)
{
	unsigned long flag;


	pr_debug("%s: blt_end=%d ov_blt_addr=%x\n",
		 __func__, dsi_pipe->blt_end, (int)dsi_pipe->ov_blt_addr);

	if ((dsi_pipe->blt_end == 0) && dsi_pipe->ov_blt_addr) {
		spin_lock_irqsave(&mdp_spin_lock, flag);
		dsi_pipe->blt_end = 1;	/* mark as end */
		spin_unlock_irqrestore(&mdp_spin_lock, flag);
		return 0;
	}

	return -EBUSY;
}

int mdp4_dsi_overlay_blt_offset(struct msm_fb_data_type *mfd,
					struct msmfb_overlay_blt *req)
{
	req->offset = 0;
	req->width = dsi_pipe->src_width;
	req->height = dsi_pipe->src_height;
	req->bpp = dsi_pipe->bpp;

	return sizeof(*req);
}


int mdp4_dsi_cmd_on(struct platform_device *pdev)
{
	int ret = 0;
	int cndx = 0;
	struct msm_fb_data_type *mfd;
	struct vsycn_ctrl *vctrl;

	pr_debug("%s+:\n", __func__);

	mfd = (struct msm_fb_data_type *)platform_get_drvdata(pdev);

	vctrl = &vsync_ctrl_db[cndx];
	vctrl->mfd = mfd;
	vctrl->dev = mfd->fbi->dev;

	mdp_clk_ctrl(1);
	mdp4_overlay_update_dsi_cmd(mfd);
	mdp_clk_ctrl(0);

	mdp4_iommu_attach();

	atomic_set(&vctrl->suspend, 0);
	pr_debug("%s-:\n", __func__);

	return ret;
}

void mdp4_blt_xy_update(struct mdp4_overlay_pipe *pipe)
{
	uint32 off, addr, addr2;
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
	if (pipe->dmap_cnt & 0x01)
		off = pipe->src_height * pipe->src_width * bpp;
	addr = pipe->dma_blt_addr + off;

	/* dmap */
	MDP_OUTP(MDP_BASE + 0x90008, addr);

	off = 0;
	if (pipe->ov_cnt & 0x01)
		off = pipe->src_height * pipe->src_width * bpp;
	addr2 = pipe->ov_blt_addr + off;
	/* overlay 0 */
	overlay_base = MDP_BASE + MDP4_OVERLAYPROC0_BASE;/* 0x10000 */
	outpdw(overlay_base + 0x000c, addr2);
	outpdw(overlay_base + 0x001c, addr2);
}


/*
 * mdp4_dmap_done_dsi: called from isr
 * DAM_P_DONE only used when blt enabled
 */
void mdp4_dma_p_done_dsi(struct mdp_dma_data *dma)
{
	int diff;

	dsi_pipe->dmap_cnt++;
	diff = dsi_pipe->ov_cnt - dsi_pipe->dmap_cnt;
	pr_debug("%s: ov_cnt=%d dmap_cnt=%d\n",
			__func__, dsi_pipe->ov_cnt, dsi_pipe->dmap_cnt);

	if (diff <= 0) {
		spin_lock(&mdp_spin_lock);
		dma->dmap_busy = FALSE;
		complete(&dma->dmap_comp);
		spin_unlock(&mdp_spin_lock);
		if (dsi_pipe->blt_end) {
			dsi_pipe->blt_end = 0;
			dsi_pipe->dma_blt_addr = 0;
			dsi_pipe->ov_blt_addr = 0;
			pr_debug("%s: END, ov_cnt=%d dmap_cnt=%d\n",
				__func__, dsi_pipe->ov_cnt, dsi_pipe->dmap_cnt);
			mdp_intr_mask &= ~INTR_DMA_P_DONE;
			outp32(MDP_INTR_ENABLE, mdp_intr_mask);
		}
		mdp_pipe_ctrl(MDP_OVERLAY0_BLOCK, MDP_BLOCK_POWER_OFF, TRUE);
		mdp_disable_irq_nosync(MDP_DMA2_TERM);  /* disable intr */
		return;
	}

	spin_lock(&mdp_spin_lock);
	dma->busy = FALSE;
	spin_unlock(&mdp_spin_lock);
	complete(&dma->comp);
	if (busy_wait_cnt)
		busy_wait_cnt--;

	pr_debug("%s: kickoff dmap\n", __func__);

	mdp4_blt_xy_update(dsi_pipe);
	/* kick off dmap */
	outpdw(MDP_BASE + 0x000c, 0x0);
	mdp4_stat.kickoff_dmap++;
	/* trigger dsi cmd engine */
	mipi_dsi_cmd_mdp_start();

	mdp_pipe_ctrl(MDP_OVERLAY0_BLOCK, MDP_BLOCK_POWER_OFF, TRUE);
}

void mdp4_dsi_cmd_overlay_restore(void)
{
	/* mutex holded by caller */
	if (dsi_mfd && dsi_pipe) {
		mdp4_dsi_cmd_dma_busy_wait(dsi_mfd);
		mipi_dsi_mdp_busy_wait(dsi_mfd);
		mdp4_overlay_update_dsi_cmd(dsi_mfd);

		if (dsi_pipe->ov_blt_addr)
			mdp4_dsi_blt_dmap_busy_wait(dsi_mfd);
		mdp4_dsi_cmd_overlay_kickoff(dsi_mfd, dsi_pipe);
	}
}

void mdp4_dsi_blt_dmap_busy_wait(struct msm_fb_data_type *mfd)
{
	unsigned long flag;
	int need_wait = 0;

	spin_lock_irqsave(&mdp_spin_lock, flag);
	if (mfd->dma->dmap_busy == TRUE) {
		INIT_COMPLETION(mfd->dma->dmap_comp);
		need_wait++;
	}
	spin_unlock_irqrestore(&mdp_spin_lock, flag);

	if (need_wait) {
		/* wait until DMA finishes the current job */
		wait_for_completion(&mfd->dma->dmap_comp);
	}
}

/*
 * mdp4_dsi_cmd_dma_busy_wait: check dsi link activity
 * dsi link is a shared resource and it can only be used
 * while it is in idle state.
 * ov_mutex need to be acquired before call this function.
 */
void mdp4_dsi_cmd_dma_busy_wait(struct msm_fb_data_type *mfd)
{
	unsigned long flag;
	int need_wait = 0;



	if (dsi_clock_timer.function) {
		if (time_after(jiffies, tout_expired)) {
			tout_expired = jiffies + TOUT_PERIOD;
			mod_timer(&dsi_clock_timer, tout_expired);
			tout_expired -= MS_100;
		}
	}

	pr_debug("%s: start pid=%d dsi_clk_on=%d\n",
			__func__, current->pid, mipi_dsi_clk_on);

	/* satrt dsi clock if necessary */
	if (mipi_dsi_clk_on == 0) {
		local_bh_disable();
		mipi_dsi_turn_on_clks();
		local_bh_enable();
	}

	spin_lock_irqsave(&mdp_spin_lock, flag);
	if (mfd->dma->busy == TRUE) {
		if (busy_wait_cnt == 0)
			INIT_COMPLETION(mfd->dma->comp);
		busy_wait_cnt++;
		need_wait++;
	}
	spin_unlock_irqrestore(&mdp_spin_lock, flag);

	if (need_wait) {
		/* wait until DMA finishes the current job */
		pr_debug("%s: pending pid=%d dsi_clk_on=%d\n",
				__func__, current->pid, mipi_dsi_clk_on);
		wait_for_completion(&mfd->dma->comp);
	}
	pr_debug("%s: done pid=%d dsi_clk_on=%d\n",
			 __func__, current->pid, mipi_dsi_clk_on);
}

void mdp4_dsi_cmd_kickoff_video(struct msm_fb_data_type *mfd,
				struct mdp4_overlay_pipe *pipe)
{
	/*
	 * a video kickoff may happen before UI kickoff after
	 * blt enabled. mdp4_overlay_update_dsi_cmd() need
	 * to be called before kickoff.
	 * vice versa for blt disabled.
	 */
	if (dsi_pipe->ov_blt_addr && dsi_pipe->blt_cnt == 0)
		mdp4_overlay_update_dsi_cmd(mfd); /* first time */
	else if (dsi_pipe->ov_blt_addr == 0  && dsi_pipe->blt_cnt) {
		mdp4_overlay_update_dsi_cmd(mfd); /* last time */
		dsi_pipe->blt_cnt = 0;
	}

	pr_debug("%s: ov_blt_addr=%d blt_cnt=%d\n",
		__func__, (int)dsi_pipe->ov_blt_addr, dsi_pipe->blt_cnt);

	if (dsi_pipe->ov_blt_addr)
		mdp4_dsi_blt_dmap_busy_wait(dsi_mfd);

	mdp4_dsi_cmd_overlay_kickoff(mfd, pipe);
}

void mdp4_dsi_cmd_kickoff_ui(struct msm_fb_data_type *mfd,
				struct mdp4_overlay_pipe *pipe)
{

	pr_debug("%s: pid=%d\n", __func__, current->pid);
	mdp4_dsi_cmd_overlay_kickoff(mfd, pipe);
}


void mdp4_dsi_cmd_overlay_kickoff(struct msm_fb_data_type *mfd,
				struct mdp4_overlay_pipe *pipe)
{
	unsigned long flag;

	mdp4_iommu_attach();
	/* change mdp clk */
	mdp4_set_perf_level();

	mipi_dsi_mdp_busy_wait(mfd);

	if (dsi_pipe->ov_blt_addr == 0)
		mipi_dsi_cmd_mdp_start();

	mdp4_overlay_dsi_state_set(ST_DSI_PLAYING);

	spin_lock_irqsave(&mdp_spin_lock, flag);
	mdp_enable_irq(MDP_OVERLAY0_TERM);
	mfd->dma->busy = TRUE;
	if (dsi_pipe->ov_blt_addr)
		mfd->dma->dmap_busy = TRUE;
	/* start OVERLAY pipe */
	spin_unlock_irqrestore(&mdp_spin_lock, flag);
	mdp_pipe_kickoff(MDP_OVERLAY0_TERM, mfd);
	mdp4_stat.kickoff_ov0++;
}

void mdp_dsi_cmd_overlay_suspend(struct msm_fb_data_type *mfd)
{
	int cndx = 0;
	struct vsycn_ctrl *vctrl;
	struct mdp4_overlay_pipe *pipe;

	vctrl = &vsync_ctrl_db[cndx];
	pipe = vctrl->base_pipe;
	/* dis-engage rgb0 from mixer0 */
	if (pipe) {
		if (mfd->ref_cnt == 0) {
			/* adb stop */
			if (pipe->pipe_type == OVERLAY_TYPE_BF)
				mdp4_overlay_borderfill_stage_down(pipe);

			/* pipe == rgb1 */
			mdp4_overlay_unset_mixer(pipe->mixer_num);
			vctrl->base_pipe = NULL;
		} else {
			mdp4_mixer_stage_down(pipe, 1);
			mdp4_overlay_iommu_pipe_free(pipe->pipe_ndx, 1);
		}
	}
}

void mdp4_dsi_cmd_overlay(struct msm_fb_data_type *mfd)
{
	mutex_lock(&mfd->dma->ov_mutex);

	if (mfd && mfd->panel_power_on) {
		mdp4_dsi_cmd_dma_busy_wait(mfd);

		if (dsi_pipe && dsi_pipe->ov_blt_addr)
			mdp4_dsi_blt_dmap_busy_wait(mfd);

		mdp4_overlay_update_dsi_cmd(mfd);

		mdp4_dsi_cmd_kickoff_ui(mfd, dsi_pipe);
		mdp4_iommu_unmap(dsi_pipe);
	/* signal if pan function is waiting for the update completion */
		if (mfd->pan_waiting) {
			mfd->pan_waiting = FALSE;
			complete(&mfd->pan_comp);
		}
	}
	mutex_unlock(&mfd->dma->ov_mutex);
}
