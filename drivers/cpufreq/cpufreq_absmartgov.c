/*
 * drivers/cpufreq/cpufreq_absmartgov.c
 *
 * Copyright (C) 2010 Google, Inc.
 * Copyright (C) 2014 Paul Reioux
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
 * Author: Mike Chan (mike@android.com)
 * Author: Paul Reioux (reioux@gmail.com) Modified for absmartgov
 */

#include "cpu_load_metric.h"
#include <linux/cpu.h>
#include <linux/cpumask.h>
#include <linux/cpufreq.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/rwsem.h>
#include <linux/sched.h>
#include <linux/sched/rt.h>
#include <linux/tick.h>
#include <linux/time.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/kthread.h>
#include <linux/slab.h>
#include <linux/kernel_stat.h>
#include <asm/cputime.h>

#if defined(CONFIG_POWERSUSPEND)
#include <linux/powersuspend.h>
#endif

#if defined(CONFIG_ARM_EXYNOS_MP_CPUFREQ) || defined(CONFIG_ARM_EXYNOS_SMP_CPUFREQ)
#include <mach/cpufreq.h>
#endif

#include <../drivers/gpu/arm/t7xx/r15p0/platform/exynos/mali_kbase_platform.h>

static int active_count;

struct cpufreq_absmartgov_cpuinfo {
	struct timer_list cpu_timer;
	struct timer_list cpu_slack_timer;
	struct cpufreq_policy *policy;
	struct cpufreq_frequency_table *freq_table;
	unsigned int target_freq;
	struct rw_semaphore enable_sem;
	int governor_enabled;
};

static DEFINE_PER_CPU(struct cpufreq_absmartgov_cpuinfo, cpuinfo);

/* realtime thread handles frequency scaling */
static struct task_struct *speedchange_task;
static cpumask_t speedchange_cpumask;
static spinlock_t speedchange_cpumask_lock;
static struct mutex gov_lock;

#define MAX_FREQ			1700000
static unsigned int max_freq = MAX_FREQ;

#define MIN_FREQ			300000
static unsigned int min_freq = MIN_FREQ;

#define TARGET_LOAD			80
static unsigned int target_load = TARGET_LOAD;
static unsigned int load_history1 = 100;
static unsigned int load_history2 = 100;

#define ONE_MHZ				100000

/*
 * The sample rate of the timer used to increase frequency
 */
#define DEFAULT_TIMER_RATE (20 * USEC_PER_MSEC)
static unsigned long timer_rate = DEFAULT_TIMER_RATE;

#if defined(CONFIG_POWERSUSPEND)
#define SCREEN_OFF_TIMER_RATE ((unsigned long)(60 * USEC_PER_MSEC))
#endif

/* Busy SDF parameters*/
#define MIN_BUSY_TIME (100 * USEC_PER_MSEC)

#if defined(CONFIG_POWERSUSPEND)
#define DEFAULT_SCREEN_OFF_MAX 1000000
static unsigned long screen_off_max = DEFAULT_SCREEN_OFF_MAX;
#endif

/*
 * Max additional time to wait in idle, beyond timer_rate, at speeds above
 * minimum before wakeup to reduce speed, or -1 if unnecessary.
 */
#define DEFAULT_TIMER_SLACK (4 * DEFAULT_TIMER_RATE)
static int timer_slack_val = DEFAULT_TIMER_SLACK;

static int cpufreq_governor_absmartgov(struct cpufreq_policy *policy,
		unsigned int event);

#ifndef CONFIG_CPU_FREQ_DEFAULT_GOV_ABSMARTGOV
static
#endif
struct cpufreq_governor cpufreq_gov_absmartgov = {
	.name = "absmartgov",
	.governor = cpufreq_governor_absmartgov,
	.max_transition_latency = 10000000,
	.owner = THIS_MODULE,
};

static void cpufreq_absmartgov_timer_resched(
	struct cpufreq_absmartgov_cpuinfo *pcpu)
{
	unsigned long expires;
	
	expires = jiffies + usecs_to_jiffies(timer_rate);
	mod_timer_pinned(&pcpu->cpu_timer, expires);

	if (timer_slack_val >= 0 && pcpu->target_freq > pcpu->policy->min) {
		expires += usecs_to_jiffies(timer_slack_val);
		mod_timer_pinned(&pcpu->cpu_slack_timer, expires);
	}
}

/* The caller shall take enable_sem write semaphore to avoid any timer race.
 * The cpu_timer and cpu_slack_timer must be deactivated when calling this
 * function.
 */
static void cpufreq_absmartgov_timer_start(int cpu)
{
	struct cpufreq_absmartgov_cpuinfo *pcpu = &per_cpu(cpuinfo, cpu);
	unsigned long expires = jiffies + usecs_to_jiffies(timer_rate);

	pcpu->cpu_timer.expires = expires;
	if (cpu_online(cpu)) {
		add_timer_on(&pcpu->cpu_timer, cpu);
		if (timer_slack_val >= 0 && pcpu->target_freq >
		     pcpu->policy->min) {
			expires += usecs_to_jiffies(timer_slack_val);
			pcpu->cpu_slack_timer.expires = expires;
			add_timer_on(&pcpu->cpu_slack_timer, cpu);
		}
	}
}

/*
 * If increasing frequencies never map to a lower target load then
 * choose_freq() will find the minimum frequency that does not exceed its
 * target load given the current load.
 */

static unsigned int choose_freq(int cpu)
{
	unsigned int freq, cur_load, cur_freq;

	cur_freq = cpufreq_quick_get(cpu);
	cur_load = cpu_get_load(cpu);
	
	freq = min_freq + ((cur_load / target_load) * (max_freq - min_freq));
	
	if (cur_load > load_history1 && load_history1 > load_history2) {
		freq = (freq * 125) / 100;
	} else if (!(cur_load < load_history1 && load_history1 < load_history2) && cur_freq > freq) {
		freq = cur_freq;
	}
	
	if ((freq % ONE_MHZ) >= 5 * (ONE_MHZ / 10)) {
		freq = (freq / ONE_MHZ) + 1;
		freq *= ONE_MHZ;
	}
	
	if (freq > max_freq)
		freq = max_freq;

	load_history2 = load_history1;
	load_history1 = cur_load;

	return freq;
}

static void cpufreq_absmartgov_timer(unsigned long data)
{
	u64 now;
	struct cpufreq_absmartgov_cpuinfo *pcpu =
		&per_cpu(cpuinfo, data);
	unsigned int new_freq;
	unsigned long flags;

	if (!down_read_trylock(&pcpu->enable_sem))
		return;
	if (!pcpu->governor_enabled)
		goto exit;

	if (cpu_is_offline(data))
		goto exit;

	now = ktime_to_us(ktime_get());

#if defined(CONFIG_POWERSUSPEND)	
	if (!power_suspend_active)
		timer_rate = DEFAULT_TIMER_RATE;
	else
		timer_rate = SCREEN_OFF_TIMER_RATE;
#endif

	new_freq = choose_freq(data);

	if (pcpu->target_freq == new_freq) {
		goto rearm_if_notmax;
	}

	pcpu->target_freq = new_freq;
	spin_lock_irqsave(&speedchange_cpumask_lock, flags);
	cpumask_set_cpu(data, &speedchange_cpumask);
	spin_unlock_irqrestore(&speedchange_cpumask_lock, flags);
	wake_up_process(speedchange_task);

rearm_if_notmax:
	/*
	 * Already set max speed and don't see a need to change that,
	 * wait until next idle to re-evaluate, don't need timer.
	 */
	if (pcpu->target_freq == pcpu->policy->max)
		goto exit;

	if (!timer_pending(&pcpu->cpu_timer))
		cpufreq_absmartgov_timer_resched(pcpu);

exit:
	up_read(&pcpu->enable_sem);
	return;
}

static void cpufreq_absmartgov_idle_start(void)
{
	int cpu = smp_processor_id();
	struct cpufreq_absmartgov_cpuinfo *pcpu =
		&per_cpu(cpuinfo, smp_processor_id());
	int pending;

	if (!down_read_trylock(&pcpu->enable_sem))
		return;
	if (!pcpu->governor_enabled)
		goto exit;

	/* Cancel the timer if cpu is offline */
	if (cpu_is_offline(cpu)) {
		del_timer(&pcpu->cpu_timer);
		del_timer(&pcpu->cpu_slack_timer);
		goto exit;
	}

	pending = timer_pending(&pcpu->cpu_timer);

	if (pcpu->target_freq != pcpu->policy->min) {
		/*
		 * Entering idle while not at lowest speed.  On some
		 * platforms this can hold the other CPU(s) at that speed
		 * even though the CPU is idle. Set a timer to re-evaluate
		 * speed so this idle CPU doesn't hold the other CPUs above
		 * min indefinitely.  This should probably be a quirk of
		 * the CPUFreq driver.
		 */
		if (!pending)
			cpufreq_absmartgov_timer_resched(pcpu);
	}
exit:
	up_read(&pcpu->enable_sem);
}

static void cpufreq_absmartgov_idle_end(void)
{
	struct cpufreq_absmartgov_cpuinfo *pcpu =
		&per_cpu(cpuinfo, smp_processor_id());

	if (!down_read_trylock(&pcpu->enable_sem))
		return;
	if (!pcpu->governor_enabled) {
		up_read(&pcpu->enable_sem);
		return;
	}

	/* Arm the timer for 1-2 ticks later if not already. */
	if (!timer_pending(&pcpu->cpu_timer)) {
		cpufreq_absmartgov_timer_resched(pcpu);
	} else if (time_after_eq(jiffies, pcpu->cpu_timer.expires)) {
		del_timer(&pcpu->cpu_timer);
		del_timer(&pcpu->cpu_slack_timer);
		cpufreq_absmartgov_timer(smp_processor_id());
	}

	up_read(&pcpu->enable_sem);
}

static int cpufreq_absmartgov_speedchange_task(void *data)
{
	unsigned int cpu;
	cpumask_t tmp_mask;
	unsigned long flags;
	struct cpufreq_absmartgov_cpuinfo *pcpu;

	while (1) {
		set_current_state(TASK_INTERRUPTIBLE);
		spin_lock_irqsave(&speedchange_cpumask_lock, flags);

		if (cpumask_empty(&speedchange_cpumask)) {
			spin_unlock_irqrestore(&speedchange_cpumask_lock,
					       flags);
			schedule();

			if (kthread_should_stop())
				break;

			spin_lock_irqsave(&speedchange_cpumask_lock, flags);
		}

		set_current_state(TASK_RUNNING);
		tmp_mask = speedchange_cpumask;
		cpumask_clear(&speedchange_cpumask);
		spin_unlock_irqrestore(&speedchange_cpumask_lock, flags);

		for_each_cpu(cpu, &tmp_mask) {
			pcpu = &per_cpu(cpuinfo, cpu);
			
			if (!down_read_trylock(&pcpu->enable_sem))
				continue;
			if (!pcpu->governor_enabled) {
				up_read(&pcpu->enable_sem);
				continue;
			}

#if defined(CONFIG_POWERSUSPEND)
			if (power_suspend_active && pcpu->target_freq > screen_off_max) 
					pcpu->target_freq = screen_off_max;
#endif

			if (pcpu->target_freq != pcpu->policy->cur)
				__cpufreq_driver_target(pcpu->policy,
							pcpu->target_freq,
							CPUFREQ_RELATION_H);

			up_read(&pcpu->enable_sem);
		}
	}

	return 0;
}

static int cpufreq_absmartgov_notifier(
	struct notifier_block *nb, unsigned long val, void *data)
{
	struct cpufreq_freqs *freq = data;
	struct cpufreq_absmartgov_cpuinfo *pcpu;
	int cpu;

	if (val == CPUFREQ_PRECHANGE) {
		pcpu = &per_cpu(cpuinfo, freq->cpu);
		if (!down_read_trylock(&pcpu->enable_sem))
			return 0;
		if (!pcpu->governor_enabled) {
			up_read(&pcpu->enable_sem);
			return 0;
		}

		for_each_cpu(cpu, pcpu->policy->cpus) {
			struct cpufreq_absmartgov_cpuinfo *pjcpu =
				&per_cpu(cpuinfo, cpu);
			if (cpu != freq->cpu) {
				if (!down_read_trylock(&pjcpu->enable_sem))
					continue;
				if (!pjcpu->governor_enabled) {
					up_read(&pjcpu->enable_sem);
					continue;
				}
			}
			if (cpu != freq->cpu)
				up_read(&pjcpu->enable_sem);
		}

		up_read(&pcpu->enable_sem);
	}
	return 0;
}

static struct notifier_block cpufreq_notifier_block = {
	.notifier_call = cpufreq_absmartgov_notifier,
};

static ssize_t show_min_freq(struct kobject *kobj,
				struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", min_freq);
}

static ssize_t store_min_freq(struct kobject *kobj,
			struct attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned int val;

	ret = strict_strtoul(buf, 0, &val);
	if (ret < 0)
		return ret;
	if (val < MIN_FREQ || val > MAX_FREQ || val > max_freq)
		return -EINVAL;
	min_freq = val;
	return count;
}

static struct global_attr min_freq_attr = __ATTR(min_freq, 0644,
		show_min_freq, store_min_freq);
		
static ssize_t show_max_freq(struct kobject *kobj,
				struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", max_freq);
}

static ssize_t store_max_freq(struct kobject *kobj,
			struct attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned int val;

	ret = strict_strtoul(buf, 0, &val);
	if (ret < 0)
		return ret;
	if (val < MIN_FREQ || val > MAX_FREQ || val < min_freq)
		return -EINVAL;
	max_freq = val;
	return count;
}

static struct global_attr max_freq_attr = __ATTR(max_freq, 0644,
		show_max_freq, store_max_freq);
		
static ssize_t show_target_load(struct kobject *kobj,
				struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", target_load);
}

static ssize_t store_target_load(struct kobject *kobj,
			struct attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned int val;

	ret = strict_strtoul(buf, 0, &val);
	if (ret < 0)
		return ret;
	if (val < 0 || val > 100)
		return -EINVAL;
	target_load = val;
	return count;
}

static struct global_attr target_load_attr = __ATTR(target_load, 0644,
		show_target_load, store_target_load);
		
static ssize_t show_timer_rate(struct kobject *kobj,
			struct attribute *attr, char *buf)
{
	return sprintf(buf, "%lu\n", timer_rate);
}

static ssize_t store_timer_rate(struct kobject *kobj,
			struct attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned long val;

	ret = strict_strtoul(buf, 0, &val);
	if (ret < 0)
		return ret;
	timer_rate = val;
	return count;
}

static struct global_attr timer_rate_attr = __ATTR(timer_rate, 0644,
		show_timer_rate, store_timer_rate);

static ssize_t show_timer_slack(
	struct kobject *kobj, struct attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", timer_slack_val);
}

static ssize_t store_timer_slack(
	struct kobject *kobj, struct attribute *attr, const char *buf,
	size_t count)
{
	int ret;
	unsigned long val;

	ret = kstrtol(buf, 10, &val);
	if (ret < 0)
		return ret;

	timer_slack_val = val;
	return count;
}

define_one_global_rw(timer_slack);

static struct attribute *absmartgov_attributes[] = {
	&min_freq_attr.attr,
	&max_freq_attr.attr,
	&target_load_attr.attr,
	&timer_rate_attr.attr,
	&timer_slack.attr,
	NULL,
};

static struct attribute_group absmartgov_attr_group = {
	.attrs = absmartgov_attributes,
	.name = "absmartgov",
};

static int cpufreq_absmartgov_idle_notifier(struct notifier_block *nb,
					     unsigned long val,
					     void *data)
{
	switch (val) {
	case IDLE_START:
		cpufreq_absmartgov_idle_start();
		break;
	case IDLE_END:
		cpufreq_absmartgov_idle_end();
		break;
	}

	return 0;
}

static struct notifier_block cpufreq_absmartgov_idle_nb = {
	.notifier_call = cpufreq_absmartgov_idle_notifier,
};

static int cpufreq_governor_absmartgov(struct cpufreq_policy *policy,
		unsigned int event)
{
	int rc;
	unsigned int j;
	struct cpufreq_absmartgov_cpuinfo *pcpu;
	struct cpufreq_frequency_table *freq_table;

	switch (event) {
	case CPUFREQ_GOV_START:
		if (!cpu_online(policy->cpu))
			return -EINVAL;

		mutex_lock(&gov_lock);

		freq_table =
			cpufreq_frequency_get_table(policy->cpu);

		for_each_cpu(j, policy->cpus) {
			pcpu = &per_cpu(cpuinfo, j);
			pcpu->policy = policy;
			pcpu->target_freq = policy->cur;
			pcpu->freq_table = freq_table;
			down_write(&pcpu->enable_sem);
			cpufreq_absmartgov_timer_start(j);
			pcpu->governor_enabled = 1;
			up_write(&pcpu->enable_sem);
		}

		/*
		 * Do not register the idle hook and create sysfs
		 * entries if we have already done so.
		 */
		if (++active_count > 1) {
			mutex_unlock(&gov_lock);
			return 0;
		}

		rc = sysfs_create_group(cpufreq_global_kobject,
				&absmartgov_attr_group);
		if (rc) {
			mutex_unlock(&gov_lock);
			return rc;
		}

		idle_notifier_register(&cpufreq_absmartgov_idle_nb);
		cpufreq_register_notifier(
			&cpufreq_notifier_block, CPUFREQ_TRANSITION_NOTIFIER);
		mutex_unlock(&gov_lock);
		break;

	case CPUFREQ_GOV_STOP:
		mutex_lock(&gov_lock);
		for_each_cpu(j, policy->cpus) {
			pcpu = &per_cpu(cpuinfo, j);
			down_write(&pcpu->enable_sem);
			pcpu->governor_enabled = 0;
			del_timer_sync(&pcpu->cpu_timer);
			del_timer_sync(&pcpu->cpu_slack_timer);
			up_write(&pcpu->enable_sem);
		}

		if (--active_count > 0) {
			mutex_unlock(&gov_lock);
			return 0;
		}

		cpufreq_unregister_notifier(
			&cpufreq_notifier_block, CPUFREQ_TRANSITION_NOTIFIER);
		idle_notifier_unregister(&cpufreq_absmartgov_idle_nb);
		sysfs_remove_group(cpufreq_global_kobject,
				&absmartgov_attr_group);
		mutex_unlock(&gov_lock);

		break;

	case CPUFREQ_GOV_LIMITS:
		if (policy->max < policy->cur)
			__cpufreq_driver_target(policy,
					policy->max, CPUFREQ_RELATION_H);
		else if (policy->min > policy->cur)
			__cpufreq_driver_target(policy,
					policy->min, CPUFREQ_RELATION_L);
		for_each_cpu(j, policy->cpus) {
			pcpu = &per_cpu(cpuinfo, j);

			/* hold write semaphore to avoid race */
			down_write(&pcpu->enable_sem);
			if (pcpu->governor_enabled == 0) {
				up_write(&pcpu->enable_sem);
				continue;
			}

			/* update target_freq firstly */
			if (policy->max < pcpu->target_freq)
				pcpu->target_freq = policy->max;
			else if (policy->min > pcpu->target_freq)
				pcpu->target_freq = policy->min;

			/* Reschedule timer.
			 * Delete the timers, else the timer callback may
			 * return without re-arm the timer when failed
			 * acquire the semaphore. This race may cause timer
			 * stopped unexpectedly.
			 */
			del_timer_sync(&pcpu->cpu_timer);
			del_timer_sync(&pcpu->cpu_slack_timer);
			cpufreq_absmartgov_timer_start(j);
			up_write(&pcpu->enable_sem);
		}
		break;
	}
	return 0;
}

static void cpufreq_absmartgov_nop_timer(unsigned long data)
{
}

static int __init cpufreq_absmartgov_init(void)
{
	unsigned int i;
	struct cpufreq_absmartgov_cpuinfo *pcpu;
	struct sched_param param = { .sched_priority = MAX_RT_PRIO-1 };

	/* Initalize per-cpu timers */
	for_each_possible_cpu(i) {
		pcpu = &per_cpu(cpuinfo, i);
		init_timer_deferrable(&pcpu->cpu_timer);
		pcpu->cpu_timer.function = cpufreq_absmartgov_timer;
		pcpu->cpu_timer.data = i;
		init_timer(&pcpu->cpu_slack_timer);
		pcpu->cpu_slack_timer.function = cpufreq_absmartgov_nop_timer;
		init_rwsem(&pcpu->enable_sem);
	}

	spin_lock_init(&speedchange_cpumask_lock);
	mutex_init(&gov_lock);
	speedchange_task =
		kthread_create(cpufreq_absmartgov_speedchange_task, NULL,
			       "cfabsmartgov");
	if (IS_ERR(speedchange_task))
		return PTR_ERR(speedchange_task);

	sched_setscheduler_nocheck(speedchange_task, SCHED_FIFO, &param);
	get_task_struct(speedchange_task);

	/* NB: wake up so the thread does not look hung to the freezer */
	wake_up_process(speedchange_task);

	return cpufreq_register_governor(&cpufreq_gov_absmartgov);
}

#ifdef CONFIG_CPU_FREQ_DEFAULT_GOV_ABSMARTGOV
fs_initcall(cpufreq_absmartgov_init);
#else
module_init(cpufreq_absmartgov_init);
#endif

static void __exit cpufreq_absmartgov_exit(void)
{
	cpufreq_unregister_governor(&cpufreq_gov_absmartgov);
	kthread_stop(speedchange_task);
	put_task_struct(speedchange_task);
}

module_exit(cpufreq_absmartgov_exit);

MODULE_AUTHOR("Mike Chan <mike@android.com>");
MODULE_AUTHOR("Paul Reioux <reioux@gmail.com>");
MODULE_DESCRIPTION("'cpufreq_absmartgov' - A cpufreq governor for "
	"Latency sensitive workloads based on Google's Interactive");
MODULE_LICENSE("GPL");
