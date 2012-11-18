/*
 * android vibrator driver (msm7x27, Motor IC)
 *
 * Copyright (C) 2009 LGE, Inc.
 *
 * Author: Jinkyu Choi <jinkyu@lge.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 */

#include <linux/platform_device.h>
#include <mach/gpio.h>
#include <linux/delay.h>
#include <linux/timer.h>
#include <linux/err.h>
#include "../staging/android/timed_output.h"
#include <linux/android_vibrator.h>
#define CONFIG_LGE_LOCAL_WORK_VIB
//#undef CONFIG_PM

#ifdef CONFIG_LGE_LOCAL_WORK_VIB
static struct workqueue_struct *local_workqueue ;
#endif

struct timed_vibrator_data {
	struct timed_output_dev dev;
	struct hrtimer timer;
	spinlock_t lock;
	atomic_t vib_status;			/* on/off */

	int max_timeout;
	atomic_t vibe_gain;				/* default max gain */
	atomic_t vibe_pwm;
	atomic_t vib_timems;			/* vibrator duration */	
	struct android_vibrator_platform_data *vibe_data;
	struct work_struct work_vibrator_off;
	struct work_struct work_vibrator_on; 
};

static int android_vibrator_force_set(struct timed_vibrator_data *vib, int nVibIntensity, int n_value)
{
	/* Check the Force value with Max and Min force value */
	int vib_dutation_ms = 0;
	INFO_MSG("nVibIntensity : %d\n", nVibIntensity);

	if (nVibIntensity > 127)
		nVibIntensity = 127;
	if (nVibIntensity < -127)
		nVibIntensity = -127;

	/* TODO: control the gain of vibrator */

	if (nVibIntensity == 0) {

		vib->vibe_data->ic_enable_set(0);
		vib->vibe_data->pwm_set(0, 0, n_value);
		vib->vibe_data->power_set(0); /* should be checked for vibrator response time */

		atomic_set(&vib->vib_status, false);
	} else {

		cancel_work_sync(&vib->work_vibrator_off);
		hrtimer_cancel(&vib->timer);
		vib_dutation_ms = atomic_read(&vib->vib_timems);
		vib->vibe_data->power_set(1); /* should be checked for vibrator response time */
		vib->vibe_data->pwm_set(1, nVibIntensity, n_value);
		vib->vibe_data->ic_enable_set(1);

		atomic_set(&vib->vib_status, true);
		hrtimer_start(&vib->timer, ktime_set(vib_dutation_ms / 1000, (vib_dutation_ms % 1000) * 1000000), HRTIMER_MODE_REL);
	}
	return 0;
}

static void android_vibrator_on(struct work_struct *work)
{
	struct timed_vibrator_data *vib = container_of(work, struct timed_vibrator_data, work_vibrator_on);
	int gain = atomic_read(&vib->vibe_gain);
	int n_value = atomic_read(&vib->vibe_pwm);
	INFO_MSG("LGE:%s\n", __func__);
	// suspend /resume logging test	
	INFO_MSG("%s is stating ...gain = %d n_value = %d\n", __func__,gain, n_value);
	
	android_vibrator_force_set(vib, gain, n_value);

	// suspend /resume logging test	
	INFO_MSG("%s is exting ... \n", __func__);
}

static void android_vibrator_off(struct work_struct *work)
{
	struct timed_vibrator_data *vib = container_of(work, struct timed_vibrator_data, work_vibrator_off);

	// suspend /resume logging test	
	INFO_MSG("%s is stating ... \n", __func__);
	android_vibrator_force_set(vib, 0, vib->vibe_data->vibe_n_value);
	// suspend /resume logging test	
	INFO_MSG("%s is exting ... \n", __func__);
}

static enum hrtimer_restart vibrator_timer_func(struct hrtimer *timer)
{
	struct timed_vibrator_data *vib = container_of(timer, struct timed_vibrator_data, timer);
#ifdef CONFIG_LGE_LOCAL_WORK_VIB	
	queue_work(local_workqueue,&vib->work_vibrator_off);
#endif
	return HRTIMER_NORESTART;
}

static int vibrator_get_time(struct timed_output_dev *dev)
{
	struct timed_vibrator_data *vib = container_of(dev, struct timed_vibrator_data, dev);

	if (hrtimer_active(&vib->timer)) {
		ktime_t r = hrtimer_get_remaining(&vib->timer);
		return ktime_to_ms(r);
	} else
		return 0;
}

static void vibrator_enable(struct timed_output_dev *dev, int nDurationMs)
{
	struct timed_vibrator_data *vib = container_of(dev, struct timed_vibrator_data, dev);
	unsigned long	flags;
	// suspend /resume logging test	
	INFO_MSG("%s is stating ... \n", __func__);
	INFO_MSG("LGE:Android_Vibrator[%d] DurationMs = %d \n",__LINE__, nDurationMs);	
	spin_lock_irqsave(&vib->lock, flags);

	if (nDurationMs > 0) {

//		cancel_work_sync(&vib->work_vibrator_on);
		
		if (nDurationMs > vib->max_timeout)
			nDurationMs = vib->max_timeout;

		atomic_set(&vib->vib_timems, nDurationMs);

#ifdef CONFIG_LGE_LOCAL_WORK_VIB	
		queue_work(local_workqueue,&vib->work_vibrator_on);
#endif

	} else {
//		cancel_work_sync(&vib->work_vibrator_off);
#ifdef CONFIG_LGE_LOCAL_WORK_VIB	
		queue_work(local_workqueue,&vib->work_vibrator_off);
#endif

	}
	spin_unlock_irqrestore(&vib->lock, flags);


	// suspend /resume logging test	
	INFO_MSG("%s is exting ... \n", __func__);
}
#if 1
static ssize_t vibrator_amp_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct timed_output_dev *dev_ = (struct timed_output_dev *)dev_get_drvdata(dev);
	struct timed_vibrator_data *vib = container_of(dev_, struct timed_vibrator_data, dev);
	int gain = atomic_read(&(vib->vibe_gain));

	return sprintf(buf, "%d\n", gain);
}

static ssize_t vibrator_amp_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct timed_output_dev *dev_ = (struct timed_output_dev *)dev_get_drvdata(dev);
	struct timed_vibrator_data *vib = container_of(dev_, struct timed_vibrator_data, dev);

	int gain;
	sscanf(buf, "%d", &gain);
	atomic_set(&vib->vibe_gain, gain);

	return size;
}

static ssize_t vibrator_pwm_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct timed_output_dev *dev_ = (struct timed_output_dev *)dev_get_drvdata(dev);
	struct timed_vibrator_data *vib = container_of(dev_, struct timed_vibrator_data, dev);
	int gain = atomic_read(&(vib->vibe_pwm));

	gain = 4800/gain;

	return sprintf(buf, "%d\n", gain);
}

static ssize_t vibrator_pwm_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct timed_output_dev *dev_ = (struct timed_output_dev *)dev_get_drvdata(dev);
	struct timed_vibrator_data *vib = container_of(dev_, struct timed_vibrator_data, dev);

	int gain;
	sscanf(buf, "%d", &gain);

	gain = 4800/gain;

	atomic_set(&vib->vibe_pwm, gain);

	return size;
}

static struct device_attribute sm100_device_attrs[] = {
	__ATTR(amp, S_IRUGO | S_IWUSR, vibrator_amp_show, vibrator_amp_store),
	__ATTR(pwm, S_IRUGO | S_IWUSR, vibrator_pwm_show, vibrator_pwm_store),
};
#endif
struct timed_vibrator_data android_vibrator_data = {
	.dev.name = "vibrator",
	.dev.enable = vibrator_enable,
	.dev.get_time = vibrator_get_time,
	.max_timeout = 30000, /* max time for vibrator enable 30 sec. */
	.vibe_data = NULL,
};

static int android_vibrator_probe(struct platform_device *pdev)
{

	int i, ret = 0;
	struct timed_vibrator_data *vib;

	platform_set_drvdata(pdev, &android_vibrator_data);
	vib = (struct timed_vibrator_data *)platform_get_drvdata(pdev);

	vib->vibe_data = (struct android_vibrator_platform_data *)pdev->dev.platform_data;

	if (vib->vibe_data->vibrator_init() < 0) {
		ERR_MSG("Android Vreg, GPIO set failed\n");
		return -ENODEV;
	}

	atomic_set(&vib->vibe_gain, vib->vibe_data->amp); /* max value is 128 */
	atomic_set(&vib->vibe_pwm, vib->vibe_data->vibe_n_value);
	atomic_set(&vib->vib_status, false);
	INFO_MSG("LGE:Android_Vibrator[%d] %s  nVibIntensity = %d \n",__LINE__, __func__,vib->vibe_data->amp);	

	INIT_WORK(&vib->work_vibrator_off, android_vibrator_off);
	INIT_WORK(&vib->work_vibrator_on, android_vibrator_on);
	hrtimer_init(&vib->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	vib->timer.function = vibrator_timer_func;
	spin_lock_init(&vib->lock);

	ret = timed_output_dev_register(&vib->dev);
	if (ret < 0) {
		timed_output_dev_unregister(&vib->dev);
		return -ENODEV;
	}
	for (i = 0; i < ARRAY_SIZE(sm100_device_attrs); i++) {
		ret = device_create_file(vib->dev.dev, &sm100_device_attrs[i]);
		if (ret < 0) {
			timed_output_dev_unregister(&vib->dev);
			device_remove_file(vib->dev.dev, &sm100_device_attrs[i]);
			return -ENODEV;
		}
	}

	INFO_MSG("Android Vibrator Initialization was done\n");

	return 0;
}

static int android_vibrator_remove(struct platform_device *pdev)
{
	struct timed_vibrator_data *vib = (struct timed_vibrator_data *)platform_get_drvdata(pdev);

#ifdef CONFIG_LGE_LOCAL_WORK_VIB	
	queue_work(local_workqueue,&vib->work_vibrator_off);
#endif

	timed_output_dev_unregister(&vib->dev);

	return 0;
}

#ifdef CONFIG_PM
static int android_vibrator_suspend(struct platform_device *pdev, pm_message_t state)
{
#if 0 
	struct timed_vibrator_data *vib = (struct timed_vibrator_data *)platform_get_drvdata(pdev);

	INFO_MSG("Android Vibrator Driver Suspend\n");
	schedule_work(&vib->work_vibrator_off); 
#endif
	return 0;
}

static int android_vibrator_resume(struct platform_device *pdev)
{
#if 0
	struct timed_vibrator_data *vib = (struct timed_vibrator_data *)platform_get_drvdata(pdev);

	INFO_MSG("Android Vibrator Driver Resume\n");

	if (vib->vibe_data->vibrator_init() < 0) {
		printk(KERN_ERR "%s Android Vreg, GPIO set failed\n", __FUNCTION__);
		return -1;
	}

	schedule_work(&vib->work_vibrator_off); 
#endif
	return 0;
}
#endif

static void android_vibrator_shutdown(struct platform_device *pdev)
{
#if 0
	struct timed_vibrator_data *vib = (struct timed_vibrator_data *)platform_get_drvdata(pdev);
	
#ifdef CONFIG_LGE_LOCAL_WORK_VIB	
	queue_work(local_workqueue,&vib->work_vibrator_off);
#endif
#endif	

}

static struct platform_driver android_vibrator_driver = {
	.probe = android_vibrator_probe,
	.remove = android_vibrator_remove,
	.shutdown = android_vibrator_shutdown,
#ifdef CONFIG_PM
	.suspend = android_vibrator_suspend,
	.resume = android_vibrator_resume,
#else
	.suspend = NULL,
	.resume = NULL,
#endif
	.driver = {
		.name = "android-vibrator",
	},
};

static int __init android_vibrator_init(void)
{

#if defined(CONFIG_ANDROID_VIBRATOR)
#if defined(CONFIG_MACH_APQ8064_J1V) || defined(CONFIG_MACH_APQ8064_J1U) || defined(CONFIG_MACH_APQ8064_J1A) || defined(CONFIG_MACH_APQ8064_J1SP) || defined(CONFIG_MACH_APQ8064_J1D) || defined(CONFIG_MACH_APQ8064_J1SK) || defined(CONFIG_MACH_APQ8064_J1KT) || defined(CONFIG_MACH_APQ8064_J1KD) || defined(CONFIG_MACH_APQ8064_J1R) || defined(CONFIG_MACH_APQ8064_J1B) || defined(CONFIG_MACH_APQ8064_J1TL) || defined(CONFIG_MACH_APQ8064_J1TM) || defined(CONFIG_MACH_APQ8064_GKU) || defined(CONFIG_MACH_APQ8064_GKKT) || defined(CONFIG_MACH_APQ8064_GKSK)
	INFO_MSG("Android Vibrator Driver Init\n");
#ifdef CONFIG_LGE_LOCAL_WORK_VIB	
	local_workqueue = create_workqueue("lge_vib") ;
	
	if(!local_workqueue)
		return -ENOMEM;
#endif
	return platform_driver_register(&android_vibrator_driver);
#endif
#endif
	return 0;
}

static void __exit android_vibrator_exit(void)
{

#if defined(CONFIG_ANDROID_VIBRATOR)
#if defined(CONFIG_MACH_APQ8064_J1V) || defined(CONFIG_MACH_APQ8064_J1U) || defined(CONFIG_MACH_APQ8064_J1A) || defined(CONFIG_MACH_APQ8064_J1SP) || defined(CONFIG_MACH_APQ8064_J1D) || defined(CONFIG_MACH_APQ8064_J1SK) || defined(CONFIG_MACH_APQ8064_J1KT) || defined(CONFIG_MACH_APQ8064_J1KD) || defined(CONFIG_MACH_APQ8064_J1R) || defined(CONFIG_MACH_APQ8064_J1B) || defined(CONFIG_MACH_APQ8064_J1TL) || defined(CONFIG_MACH_APQ8064_J1TM)
	INFO_MSG("Android Vibrator Driver Exit\n");
#ifdef CONFIG_LGE_LOCAL_WORK_VIB	
	if(local_workqueue)
		destroy_workqueue(local_workqueue);

	local_workqueue = NULL;
#endif
	platform_driver_unregister(&android_vibrator_driver);
#endif
#endif
}

/* to let init lately */
/* module_init(android_vibrator_init); */
late_initcall_sync(android_vibrator_init);
module_exit(android_vibrator_exit);

MODULE_AUTHOR("LG Electronics Inc.");
MODULE_DESCRIPTION("Android Common Vibrator Driver");
MODULE_LICENSE("GPL");


