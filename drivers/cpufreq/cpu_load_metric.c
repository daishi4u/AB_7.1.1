/*
 *  ARM Intelligent Power Allocation
 *
 *  Copyright (C) 2013 ARM Limited.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/cpufreq.h>
#include <linux/cpumask.h>
#include <linux/kernel.h>
#include <linux/percpu.h>
#include <linux/types.h>

#include <trace/events/power.h>

#include <linux/powersuspend.h>

#include "cpu_load_metric.h"

struct cpu_load
{
	unsigned int frequency;
	unsigned int load;
	u64 last_update;
};
static DEFINE_PER_CPU(struct cpu_load, cpuload);

void update_cpu_metric(int cpu, u64 now, u64 delta_idle, u64 delta_time,
		       struct cpufreq_policy *policy)
{
	struct cpu_load *pcpuload = &per_cpu(cpuload, cpu);
	unsigned int load;

	/*
	 * Calculate the active time in the previous time window
	 *
	 * load = active time / total_time * 100
	 */
	if (delta_time <= delta_idle)
		load = 0;
	else
		load = div64_u64((100 * (delta_time - delta_idle)), delta_time);

	pcpuload->load = load;
	pcpuload->frequency = policy->cur;
	pcpuload->last_update = now;
#ifdef CONFIG_CPU_THERMAL_IPA_DEBUG
	trace_printk("cpu_load: cpu: %d freq: %u load: %u\n", cpu, policy->cur, load);
#endif
}

void cpu_load_metric_get(int *load, int *freq)
{
	int _load = 0, _freq = 0;
	int cpu;

	for_each_online_cpu(cpu) {
		struct cpu_load *pcpuload = &per_cpu(cpuload, cpu);

		_load += pcpuload->load;
		_freq = pcpuload->frequency;
	}

	*load = _load;
	*freq = _freq;
}

unsigned int cpu_get_load(int cpu)
{
	struct cpu_load *pcpuload = &per_cpu(cpuload, cpu);

	return pcpuload->load;	
}

unsigned int cpu_get_avg_load(void)
{
	unsigned int avg_load = 0;
	int cpu, online_cpus = 0;
	
	for_each_online_cpu(cpu) {
		struct cpu_load *pcpuload = &per_cpu(cpuload, cpu);

		avg_load += pcpuload->load;
		online_cpus++;
	}
	
	return avg_load / online_cpus;
}

int get_least_busy_cpu(void)
{
	int least_busy_cpu = 1, cpu, min_cpu = 1;
	unsigned int least_busy_cpu_load = 100, curr_load;
	
	if(power_suspend_active)
		min_cpu = 0;
	
	for_each_online_cpu(cpu)
	{
		struct cpu_load *pcpuload = &per_cpu(cpuload, cpu);
		
		curr_load = pcpuload->load;
		
		if((cpu > min_cpu) && (curr_load <= least_busy_cpu_load))
		{
			least_busy_cpu_load = curr_load;
			least_busy_cpu = cpu;
		}
	}
	
	return least_busy_cpu;
}

static void get_cluster_stat(struct cluster_stats *cl)
{
	int util = 0, freq = 0;
	int cpu, i = 0;

	for_each_cpu(cpu, cl->mask) {
		struct cpu_load *pcpuload = &per_cpu(cpuload, cpu);
		int load = (cpu_online(cpu)) ?  pcpuload->load : 0;

		util += load;
		cl->utils[i++] = load;
		freq = pcpuload->frequency;
	}

	cl->util = util;
	cl->freq = freq;
}

void get_cluster_stats(struct cluster_stats *clstats)
{
	get_cluster_stat(&clstats[0]);
	get_cluster_stat(&clstats[1]);
}
