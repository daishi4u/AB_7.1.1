/* Copyright (c) 2015, Varun Chitre <varun.chitre15@gmail.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * A simple hotplugging driver.
 * Compatible from dual core CPUs to Octa Core CPUs
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/hrtimer.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/powersuspend.h>
#include "thunderplug.h"

#define DEBUG                        0

#define THUNDERPLUG                  "thunderplug"

#ifdef CONFIG_SCHED_HMP
#define DRIVER_VERSION                5
#else
#define DRIVER_VERSION                3
#define DRIVER_SUBVER                 0
#endif

#define DEFAULT_CPU_LOAD_THRESHOLD   (65)
#define MIN_CPU_LOAD_THRESHOLD       (10)

#define HOTPLUG_ENABLED              (0)
#define DEFAULT_HOTPLUG_STYLE         HOTPLUG_SCHED
#define DEFAULT_SCHED_MODE            BALANCED

#define DEF_SAMPLING_MS	             (500)
#define MIN_SAMLING_MS               (50)
#define MIN_CPU_UP_TIME              (750)
#define TOUCH_BOOST_ENABLED          (0)

static bool isSuspended = false;

static int suspend_cpu_num = 4, resume_cpu_num = (NR_CPUS -1);
static int endurance_level = 0;
static int core_limit = NR_CPUS;

static int now[8], last_time[8];

static int sampling_time = DEF_SAMPLING_MS;
static int load_threshold = DEFAULT_CPU_LOAD_THRESHOLD;
static int stop_boost = 0;

struct cpufreq_policy old_policy[NR_CPUS];

#ifdef CONFIG_SCHED_HMP
static int tplug_hp_style = DEFAULT_HOTPLUG_STYLE;
static int tplug_sched_mode = DEFAULT_SCHED_MODE;
#else
static int tplug_hp_enabled = HOTPLUG_ENABLED;
#endif
static int touch_boost_enabled = TOUCH_BOOST_ENABLED;

static struct workqueue_struct *tplug_wq;
static struct delayed_work tplug_work;

static struct workqueue_struct *tplug_boost_wq;
static struct delayed_work tplug_boost;

/* Two Endurance Levels for Octa Cores,
 * Two for Quad Cores and
 * One for Dual
 */
static void offline_cpus(void)
{
	unsigned int cpu;
	for(cpu = NR_CPUS - 1; cpu > (suspend_cpu_num - 1); cpu--) {
		if (cpu_online(cpu))
			cpu_down(cpu);
	}
	pr_info("%s: %d cpus were offlined\n", THUNDERPLUG, (NR_CPUS - suspend_cpu_num));
}

static void __cpuinit cpus_online_all(void)
{
	unsigned int cpu;

#if DEBUG
	pr_info("%s: resume_cpu_num = %d\n",THUNDERPLUG, resume_cpu_num);
#endif
	
	for (cpu = suspend_cpu_num; cpu <= resume_cpu_num; cpu++) {
		if (cpu_is_offline(cpu))
			cpu_up(cpu);
	}

	pr_info("%s: all cpus were onlined\n", THUNDERPLUG);
}

static void __ref tplug_boost_work_fn(struct work_struct *work)
{
	struct cpufreq_policy policy;
	int cpu, ret;

   for(cpu = suspend_cpu_num; cpu < NR_CPUS; cpu++) {
		if(cpu_is_offline(cpu))
			cpu_up(cpu);
		ret = cpufreq_get_policy(&policy, cpu);
		if (ret)
			continue;
		old_policy[cpu] = policy;
		policy.min = policy.max;
		cpufreq_update_policy(cpu);
	}
	if(stop_boost == 0)
		queue_delayed_work_on(0, tplug_boost_wq, &tplug_boost,
			msecs_to_jiffies(10));
}

static void tplug_input_event(struct input_handle *handle, unsigned int type,
		unsigned int code, int value)
{
#ifdef CONFIG_SCHED_HMP
	if(tplug_hp_style == 1) {
#else
	if(tplug_hp_enabled == 1) {
#endif
		if (type == EV_KEY && code == BTN_TOUCH) {
#if DEBUG
			pr_info("%s : type = %d, code = %d, value = %d\n", THUNDERPLUG, type, code, value);
#endif
			if(value == 0) {
				stop_boost = 1;
#if DEBUG
				pr_info("%s: stopping boost\n", THUNDERPLUG);
#endif			
			}
			else {
				stop_boost = 0;
#if DEBUG
				pr_info("%s: starting boost\n", THUNDERPLUG);
#endif				
			}
		}
#ifdef CONFIG_SCHED_HMP
		if ((type == EV_KEY) && (code == BTN_TOUCH) && (value == 1)
			&& touch_boost_enabled == 1)
#else
		if ((type == EV_KEY) && (code == BTN_TOUCH) && (value == 1)
			&& touch_boost_enabled == 1)
#endif
		{
#if DEBUG
			pr_info("%s : touch boost\n", THUNDERPLUG);
#endif
			queue_delayed_work_on(0, tplug_boost_wq, &tplug_boost,
				msecs_to_jiffies(0));
		}
	}
}

static int tplug_input_connect(struct input_handler *handler,
		struct input_dev *dev, const struct input_device_id *id)
{
	struct input_handle *handle;
	int error;

	handle = kzalloc(sizeof(struct input_handle), GFP_KERNEL);
	if (!handle)
		return -ENOMEM;

	handle->dev = dev;
	handle->handler = handler;
	handle->name = "cpufreq";

	error = input_register_handle(handle);
	if (error)
		goto err2;

	error = input_open_device(handle);
	if (error)
		goto err1;

	return 0;
err1:
	input_unregister_handle(handle);
err2:
	kfree(handle);
	return error;
}

static void tplug_input_disconnect(struct input_handle *handle)
{
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}

static const struct input_device_id tplug_ids[] = {
	{ .driver_info = 1 },
	{ },
};

static struct input_handler tplug_input_handler = {
	.event          = tplug_input_event,
	.connect        = tplug_input_connect,
	.disconnect     = tplug_input_disconnect,
	.name           = "tplug_handler",
	.id_table       = tplug_ids,
};

static ssize_t thunderplug_suspend_cpus_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    return sprintf(buf, "%d", suspend_cpu_num);
}

static ssize_t thunderplug_suspend_cpus_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int val;
	sscanf(buf, "%d", &val);
	if(val < 1 || val > NR_CPUS)
		pr_info("%s: suspend cpus off-limits\n", THUNDERPLUG);
	else
		suspend_cpu_num = val;

	return count;
}

static ssize_t thunderplug_endurance_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    return sprintf(buf, "%d", endurance_level);
}

static ssize_t __ref thunderplug_endurance_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int val;
	sscanf(buf, "%d", &val);
#ifdef CONFIG_SCHED_HMP
	if(tplug_hp_style==1) {
#else
	if(tplug_hp_enabled) {
#endif
		switch(val) {
		case 0:
		case 1:
		case 2:
			if(endurance_level!=val && !(endurance_level > 1 && NR_CPUS < 4)) {
				endurance_level = val;
				
				switch(endurance_level) {
					case 1:
						if(suspend_cpu_num > NR_CPUS / 2 )
							suspend_cpu_num = NR_CPUS / 2;
						if(resume_cpu_num > (NR_CPUS / 2) - 1 || resume_cpu_num == 1)
							resume_cpu_num = ((NR_CPUS / 2) - 1);
						core_limit = NR_CPUS / 2;
					break;
					case 2:
						if( NR_CPUS >=4 && suspend_cpu_num > NR_CPUS / 4)
							suspend_cpu_num = NR_CPUS / 4;
						if( NR_CPUS >= 4 && resume_cpu_num > ((NR_CPUS / 4) - 1))
							resume_cpu_num = ((NR_CPUS / 4) - 1);
						core_limit = NR_CPUS / 4;
					break;
					default:
						suspend_cpu_num = NR_CPUS / 4; //will only work with quad or Octa core cups, but a default should be reset here
						resume_cpu_num = (NR_CPUS - 1);
						core_limit = NR_CPUS;
					break;
				}
				
				offline_cpus();
				cpus_online_all();
			}
		break;
		default:
			pr_info("%s: invalid endurance level\n", THUNDERPLUG);
		break;
		}
	}
	else
	   pr_info("%s: per-core hotplug style is disabled, ignoring endurance mode values\n", THUNDERPLUG);

	return count;
}

static ssize_t thunderplug_sampling_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    return sprintf(buf, "%d", sampling_time);
}

static ssize_t __ref thunderplug_sampling_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int val;
	sscanf(buf, "%d", &val);
	if(val > MIN_SAMLING_MS)
		sampling_time = val;

	return count;
}

static ssize_t thunderplug_tb_enabled_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    return sprintf(buf, "%d", touch_boost_enabled);
}

static ssize_t __ref thunderplug_tb_enabled_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int val;
	sscanf(buf, "%d", &val);
	switch(val)
	{
		case 0:
		case 1:
			touch_boost_enabled = val;
		break;
		default:
			pr_info("%s : invalid choice\n", THUNDERPLUG);
		break;
	}

	return count;
}

static ssize_t thunderplug_load_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    return sprintf(buf, "%d", load_threshold);
}

static ssize_t __ref thunderplug_load_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int val;
	sscanf(buf, "%d", &val);
	if(val > 10)
		load_threshold = val;

	return count;
}


static void __cpuinit tplug_work_fn(struct work_struct *work)
{
	int total_avg, cpus_on;
	
	cpus_on = num_online_cpus();
	total_avg = cpufreq_quick_get_util(0);

	if(total_avg > load_threshold && cpus_on < core_limit)
	{
		last_time[cpus_on] = ktime_to_ms(ktime_get());
		cpu_up(cpus_on);
	}
	else if(total_avg < load_threshold && cpus_on > suspend_cpu_num)
	{
		now[cpus_on - 1] = ktime_to_ms(ktime_get());
		if((now[cpus_on - 1] - last_time[cpus_on - 1]) > MIN_CPU_UP_TIME)
			cpu_down(cpus_on - 1);
	}

	queue_delayed_work_on(0, tplug_wq, &tplug_work,	msecs_to_jiffies(sampling_time));
}

static void tplug_es_suspend_work(struct power_suspend *p) {

	isSuspended = true;
	suspend_cpu_num = 1;	// change to 1 cpu while suspended
	offline_cpus();		// we're only going to put this here because if the cpu gets overwhelmed during suspend and no more cores can come online then we get random reboots
	
	pr_info("thunderplug : suspend called\n");
}

static void __cpuinit tplug_es_resume_work(struct power_suspend *p) {
	isSuspended = false;
	suspend_cpu_num = 4;	// 4 cpus online while not suspened
	cpus_online_all();

	pr_info("%s: resume\n", THUNDERPLUG);
}

static struct power_suspend __refdata tplug_power_suspend_handler = 
	{
		.suspend = tplug_es_suspend_work,
		.resume = tplug_es_resume_work,
	};

/* Thunderplug load balancer */
#ifdef CONFIG_SCHED_HMP

static void set_sched_profile(int mode) {
    switch(mode) {
	   case 1:
	       /* Balanced */
	       sched_set_boost(DISABLED);
	   break;
	   case 2:
	       /* Turbo */
	       sched_set_boost(ENABLED);
	   break;
	   default:
	       pr_info("%s: Invalid mode\n", THUNDERPLUG);
	   break;
	}
}

static ssize_t thunderplug_sched_mode_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    return sprintf(buf, "%d", tplug_sched_mode);
}

static ssize_t __ref thunderplug_sched_mode_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int val;
	sscanf(buf, "%d", &val);
	set_sched_profile(val);
	tplug_sched_mode = val;
	return count;
}

static ssize_t thunderplug_hp_style_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    return sprintf(buf, "%d", tplug_hp_style);
}

static ssize_t __ref thunderplug_hp_style_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int val, last_val;
	sscanf(buf, "%d", &val);
	last_val = tplug_hp_style;
	switch(val)
	{
		case HOTPLUG_PERCORE:
		case HOTPLUG_SCHED:
			   tplug_hp_style = val;
		break;
		default:
			pr_info("%s : invalid choice\n", THUNDERPLUG);
		break;
	}

	if(tplug_hp_style == HOTPLUG_PERCORE && tplug_hp_style != last_val) {
	    pr_info("%s: Switching to Per-core hotplug model\n", THUNDERPLUG);
	    sched_set_boost(DISABLED);
		queue_delayed_work_on(0, tplug_wq, &tplug_work,
							msecs_to_jiffies(sampling_time));
	}
	else if(tplug_hp_style==2) {
	    pr_info("%s: Switching to sched based hotplug model\n", THUNDERPLUG);
	    set_sched_profile(tplug_sched_mode);
	}

	return count;
}

static struct kobj_attribute thunderplug_hp_style_attribute =
       __ATTR(hotplug_style,
               0666,
               thunderplug_hp_style_show, thunderplug_hp_style_store);

static struct kobj_attribute thunderplug_mode_attribute =
       __ATTR(sched_mode,
               0666,
               thunderplug_sched_mode_show, thunderplug_sched_mode_store);

#else
static ssize_t thunderplug_hp_enabled_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    return sprintf(buf, "%d", tplug_hp_enabled);
}

static ssize_t __ref thunderplug_hp_enabled_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int val;
	int last_val = tplug_hp_enabled;
	sscanf(buf, "%d", &val);
	switch(val)
	{
		case 0:
		case 1:
			tplug_hp_enabled = val;
		break;
		default:
			pr_info("%s : invalid choice\n", THUNDERPLUG);
		break;
	}

	if(tplug_hp_enabled != last_val)
	{
		if(tplug_hp_enabled == 1)
		{
			queue_delayed_work_on(0, tplug_wq, &tplug_work,
								msecs_to_jiffies(sampling_time));
			register_power_suspend(&tplug_power_suspend_handler);				
		}
		else if(tplug_hp_enabled == 0)
		{
			cancel_delayed_work_sync(&tplug_work);
			unregister_power_suspend(&tplug_power_suspend_handler);
		}
	}

	return count;
}


static struct kobj_attribute thunderplug_hp_enabled_attribute =
       __ATTR(hotplug_enabled,
               0666,
               thunderplug_hp_enabled_show, thunderplug_hp_enabled_store);

#endif  //SCHED_HMP

static ssize_t thunderplug_ver_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
       return sprintf(buf, "ThunderPlug %u.%u", DRIVER_VERSION, DRIVER_SUBVER);
}

static struct kobj_attribute thunderplug_ver_attribute =
       __ATTR(version,
               0444,
               thunderplug_ver_show, NULL);

static struct kobj_attribute thunderplug_suspend_cpus_attribute =
       __ATTR(suspend_cpus,
               0666,
               thunderplug_suspend_cpus_show, thunderplug_suspend_cpus_store);

static struct kobj_attribute thunderplug_endurance_attribute =
       __ATTR(endurance_level,
               0666,
               thunderplug_endurance_show, thunderplug_endurance_store);

static struct kobj_attribute thunderplug_sampling_attribute =
       __ATTR(sampling_rate,
               0666,
               thunderplug_sampling_show, thunderplug_sampling_store);

static struct kobj_attribute thunderplug_load_attribute =
       __ATTR(load_threshold,
               0666,
               thunderplug_load_show, thunderplug_load_store);

static struct kobj_attribute thunderplug_tb_enabled_attribute =
       __ATTR(touch_boost,
               0666,
               thunderplug_tb_enabled_show, thunderplug_tb_enabled_store);

static struct attribute *thunderplug_attrs[] =
    {
        &thunderplug_ver_attribute.attr,
        &thunderplug_suspend_cpus_attribute.attr,
        &thunderplug_endurance_attribute.attr,
        &thunderplug_sampling_attribute.attr,
        &thunderplug_load_attribute.attr,
#ifdef CONFIG_SCHED_HMP
        &thunderplug_mode_attribute.attr,
        &thunderplug_hp_style_attribute.attr,
#else
        &thunderplug_hp_enabled_attribute.attr,
#endif
        &thunderplug_tb_enabled_attribute.attr,
        NULL,
    };

static struct attribute_group thunderplug_attr_group =
    {
        .attrs = thunderplug_attrs,
    };

static struct kobject *thunderplug_kobj;

static int __init thunderplug_init(void)
{
        int ret = 0;
        int sysfs_result;
        printk(KERN_DEBUG "[%s]\n",__func__);

        thunderplug_kobj = kobject_create_and_add("thunderplug", kernel_kobj);

        if (!thunderplug_kobj) {
                pr_err("%s Interface create failed!\n",
                        __FUNCTION__);
                return -ENOMEM;
        }

        sysfs_result = sysfs_create_group(thunderplug_kobj, &thunderplug_attr_group);

        if (sysfs_result) {
                pr_info("%s sysfs create failed!\n", __FUNCTION__);
                kobject_put(thunderplug_kobj);
        }

		pr_info("%s : registering input boost", THUNDERPLUG);
		ret = input_register_handler(&tplug_input_handler);
		if (ret) {
		pr_err("%s: Failed to register input handler: %d\n",
		       THUNDERPLUG, ret);
		}

		tplug_wq = alloc_workqueue("tplug",
				WQ_HIGHPRI | WQ_UNBOUND, 1);

		tplug_boost_wq = alloc_workqueue("tplug_boost",
				WQ_HIGHPRI | WQ_UNBOUND, 1);

		INIT_DELAYED_WORK(&tplug_work, tplug_work_fn);
		INIT_DELAYED_WORK(&tplug_boost, tplug_boost_work_fn);
		
		if (tplug_hp_enabled)
		{
			queue_delayed_work_on(0, tplug_wq, &tplug_work,
		                      msecs_to_jiffies(10));
			register_power_suspend(&tplug_power_suspend_handler);
		}

        pr_info("%s: init\n", THUNDERPLUG);

        return ret;
}

MODULE_LICENSE("GPL and additional rights");
MODULE_AUTHOR("Varun Chitre <varun.chitre15@gmail.com>");
MODULE_DESCRIPTION("Hotplug driver for ARM SoCs");
late_initcall(thunderplug_init);