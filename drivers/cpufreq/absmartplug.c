#include <linux/module.h>
#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/moduleparam.h>

#if defined(CONFIG_POWERSUSPEND)
#include <linux/powersuspend.h>
#endif

#include "cpu_load_metric.h"

static struct delayed_work exynos_hotplug;
static struct workqueue_struct *khotplug_wq;

struct exynos_hotplug_ctrl {
	unsigned int sampling_rate;
	unsigned int max_cpus;
	unsigned int min_cpus;
	unsigned int target_load;
	unsigned int load_history1;
	unsigned int load_history2;
};

#define SUSPENDED_CPUS	 		2
#define SCREEN_ON_MIN_CPUS	 	2
#define WAKE_UP_CPUS			NR_CPUS
#define SAMPLING_RATE 			100		// 100ms (Stock)
#define TARGET_LOAD				70

static int enabled __read_mostly = 1; // enabled by default

static struct exynos_hotplug_ctrl ctrl_hotplug = {
	.sampling_rate = SAMPLING_RATE,		/* ms */
	.min_cpus = SCREEN_ON_MIN_CPUS,
	.max_cpus = NR_CPUS,
	.target_load = TARGET_LOAD,
	.load_history1 = 100,
	.load_history2 = 100,
};

static DEFINE_MUTEX(hotplug_lock);

static void __ref hotplug_cpu(int cores)
{
	int i, num_online, least_busy_cpu;
	unsigned int least_busy_cpu_load;
	
	if (power_suspend_active && cores > 2)
		return;

	/* Check the Online CPU supposed to be online or offline */
	for (i = 0 ; i < NR_CPUS ; i++) {
		num_online = num_online_cpus();
		
		if(num_online == cores) 
		{
			break;
		}
		
		if (cores > num_online) {
			if (!cpu_online(i))
				cpu_up(i);
		} else {
			least_busy_cpu = get_least_busy_cpu(&least_busy_cpu_load);
		
			cpu_down(least_busy_cpu);
		}
	}
}

static int get_target_cores(void)
{
	int target_cores, num_online, min_cpus, max_cpus;
	unsigned int cpu_load;

	num_online = num_online_cpus();
	cpu_load = cpu_get_avg_load();
	target_cores = (cpu_load * num_online) / ctrl_hotplug.target_load;
	
	min_cpus = ctrl_hotplug.min_cpus;
	max_cpus = ctrl_hotplug.max_cpus;

	if ((cpu_load * num_online) % ctrl_hotplug.target_load)
		target_cores++;
	
	if (cpu_load > ctrl_hotplug.load_history1 && ctrl_hotplug.load_history1 > ctrl_hotplug.load_history2) {
			target_cores++;
	} else if (!(cpu_load < ctrl_hotplug.load_history1 && ctrl_hotplug.load_history1 < ctrl_hotplug.load_history2) && num_online > target_cores) {
		target_cores = num_online;
	}

	if (target_cores > max_cpus)
		target_cores = max_cpus;
	else if (target_cores < min_cpus)
		target_cores = min_cpus;
	
	ctrl_hotplug.load_history2 = ctrl_hotplug.load_history1;
	ctrl_hotplug.load_history1 = cpu_load;

	return target_cores;
}

static void update_load(void)
{
	int cpu;
	
	for_each_online_cpu(cpu) {
		update_cpu_load_metric(cpu);
	}
}

static void exynos_work(struct work_struct *dwork)
{
	int target_cpus_online;
	update_load();

	mutex_lock(&hotplug_lock);

	target_cpus_online = get_target_cores();
		
	if (num_online_cpus() != target_cpus_online)
		hotplug_cpu(target_cpus_online);

	queue_delayed_work_on(0, khotplug_wq, &exynos_hotplug, msecs_to_jiffies(ctrl_hotplug.sampling_rate));
	mutex_unlock(&hotplug_lock);
}

#define show_one(file_name, object)					\
static ssize_t show_##file_name						\
(struct kobject *kobj, struct attribute *attr, char *buf)		\
{									\
	return sprintf(buf, "%u\n", ctrl_hotplug.object);			\
}

show_one(sampling_rate, sampling_rate);
show_one(min_cpus, min_cpus);
show_one(max_cpus, max_cpus);
show_one(target_load, target_load);

#define store_one(file_name, object)					\
static ssize_t store_##file_name					\
(struct kobject *a, struct attribute *b, const char *buf, size_t count)	\
{									\
	unsigned int input;						\
	int ret;							\
	ret = sscanf(buf, "%u", &input);				\
	if (ret != 1)							\
		return -EINVAL;						\
	ctrl_hotplug.object = input;					\
	return count;							\
}

store_one(sampling_rate, sampling_rate);
store_one(target_load, target_load);

static ssize_t store_max_cpus(struct kobject *a,
	struct attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned int target_state;

	ret = sscanf(buf, "%u", &target_state);
	if (ret != 1)
		return -EINVAL;

	mutex_lock(&hotplug_lock);

	if ((target_state <= NR_CPUS) && (target_state >= SCREEN_ON_MIN_CPUS))
		ctrl_hotplug.max_cpus = target_state;

	mutex_unlock(&hotplug_lock);

	return count;
}

static ssize_t store_min_cpus(struct kobject *a,
	struct attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned int target_state;

	ret = sscanf(buf, "%u", &target_state);
	if (ret != 1)
		return -EINVAL;

	mutex_lock(&hotplug_lock);

	if ((target_state <= NR_CPUS) && (target_state >= SCREEN_ON_MIN_CPUS))
		ctrl_hotplug.min_cpus = target_state;

	mutex_unlock(&hotplug_lock);

	return count;
}

define_one_global_rw(sampling_rate);
define_one_global_rw(max_cpus);
define_one_global_rw(min_cpus);
define_one_global_rw(target_load);

#if defined(CONFIG_POWERSUSPEND)
static void __cpuinit powersave_resume(struct power_suspend *handler)
{
	hotplug_cpu(WAKE_UP_CPUS);
	
	queue_delayed_work_on(0, khotplug_wq, &exynos_hotplug,
			msecs_to_jiffies(ctrl_hotplug.sampling_rate));
}

static void __cpuinit powersave_suspend(struct power_suspend *handler)
{
	hotplug_cpu(SUSPENDED_CPUS);

	cancel_delayed_work_sync(&exynos_hotplug);
}

static struct power_suspend __refdata powersave_powersuspend = {
	.suspend = powersave_suspend,
	.resume = powersave_resume,
};
#endif /* (defined(CONFIG_POWERSUSPEND)... */

static int __cpuinit set_enabled(const char *val, const struct kernel_param *kp) {
	int ret, last_val = enabled;
	unsigned int cpu;

	cpu = 0;

	ret = param_set_bool(val, kp);
	
	if(enabled != last_val)
	{
		if (enabled) {
			queue_delayed_work_on(0, khotplug_wq, &exynos_hotplug,
				msecs_to_jiffies(ctrl_hotplug.sampling_rate));
			register_power_suspend(&powersave_powersuspend);
		} else {
			cancel_delayed_work_sync(&exynos_hotplug);
			unregister_power_suspend(&powersave_powersuspend);
			for_each_present_cpu(cpu) {
				if (num_online_cpus() >= nr_cpu_ids)
					break;
				if (!cpu_online(cpu))
					cpu_up(cpu);
			}
		}
	}
	return ret;
}

static struct kernel_param_ops module_ops = {
	.set = set_enabled,
	.get = param_get_bool,
};

module_param_cb(enabled, &module_ops, &enabled, 0644);
MODULE_PARM_DESC(enabled, "Afterburner Smartplug. Hotplug cores based on target load.");

static struct attribute *clusterhotplug_default_attrs[] = {
	&sampling_rate.attr,
	&min_cpus.attr,
	&max_cpus.attr,
	&target_load.attr,
	NULL
};

static struct attribute_group clusterhotplug_attr_group = {
	.attrs = clusterhotplug_default_attrs,
	.name = "absmartplugconf",
};

static int __init absmartplug_init(void)
{
	int ret;

	INIT_DEFERRABLE_WORK(&exynos_hotplug, exynos_work);

	khotplug_wq = alloc_workqueue("absmartplug_wq", WQ_FREEZABLE, 0);
	if (!khotplug_wq) {
		pr_err("Failed to create khotplug workqueue\n");
		ret = -EFAULT;
		goto err_wq;
	}

	ret = sysfs_create_group(kernel_kobj, &clusterhotplug_attr_group);
	if (ret) {
		pr_err("Failed to create sysfs for hotplug\n");
		goto err_sys;
	}
	
	if (enabled) {
#if defined(CONFIG_POWERSUSPEND)
		register_power_suspend(&powersave_powersuspend);
#endif

		queue_delayed_work_on(0, khotplug_wq, &exynos_hotplug, msecs_to_jiffies(ctrl_hotplug.sampling_rate) * 250);
	}

	return 0;

err_sys:
	destroy_workqueue(khotplug_wq);
err_wq:
	return ret;
}

MODULE_LICENSE("GPL and additional rights");
MODULE_AUTHOR("Brett Wagner <daishi4u@yahoo.com>");
MODULE_DESCRIPTION("Afterburner Hotplug driver for Exynos 7580");
late_initcall(absmartplug_init);
