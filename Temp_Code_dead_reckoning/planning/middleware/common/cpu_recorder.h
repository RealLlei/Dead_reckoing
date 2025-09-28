/*
 * Copyright (c) TL Technologies Co., Ltd. 2019-2021. All rights reserved.
 * Description:  planning debug
 */

#include <string.h>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <thread>
#include <utility>

#include "sys/stat.h"
#include "sys/sysinfo.h"
#include "sys/time.h"

/**
 * @namespace TL::planning::CpuRecorder
 * @brief TL::planning::CpuRecorder
 */
namespace TL {
namespace planning {
namespace CpuRecorder {
// NOLINTBEGIN
#define VMRSS_LINE 22
#define PROCESS_ITEM 14
#define KB2MB 1024

typedef struct MEMPACKED {
  char name1[20];
  unsigned int MemTotal;
  char name2[20];
  unsigned int MemFree;
  char name3[20];
  unsigned int Buffers;
  char name4[20];
  unsigned int Cached;
  char name5[20];
  unsigned int SwapCached;
} MEM_OCCUPY;

typedef struct CPUPACKED {
  char name[20];
  unsigned int user;
  unsigned int nice;
  unsigned int system;
  unsigned int idle;
  unsigned int lowait;
  unsigned int irq;
  unsigned int softirq;
} CPU_OCCUPY;

/**
 * @brief GetCurrentPid
 *
 */
inline int GetCurrentPid() {
  return getpid();
}

/**
 * @brief // get specific process cpu occupation ratio by pid
// FIXME: can also get cpu and mem status from popen cmd
// the info line num in /proc/{pid}/status file
 * 
 * @param buffer 
 * @param item 
 * @return const char* 
 */
inline const char* GetItems(const char* buffer, unsigned int item) {
  // read from buffer by offset
  const char* p = buffer;

  size_t len = strlen(buffer);
  int count = 0;

  for (int i = 0; i < len; i++) {
    if (' ' == *p) {
      count++;
      if (count == item - 1) {
        p++;
        break;
      }
    }
    p++;
  }

  return p;
}

/**
 * @brief Get the Cpu Total Occupy object
 * 
 * @return uuint64 
 */
inline int64_t GetCpuTotalOccupy() {
  // get total cpu use time

  // different mode cpu occupy time
  int64_t user_time = 0;
  int64_t nice_time = 0;
  int64_t system_time = 0;
  int64_t idle_time = 0;

  FILE* fd;
  char buff[1024] = {0};

  fd = fopen("/proc/stat", "r");
  if (nullptr == fd) {
    return 0;
  }

  fgets(buff, sizeof(buff), fd);
  char name[64] = {0};
  sscanf(buff, "%s %ld %ld %ld %ld", name, &user_time, &nice_time, &system_time,
         &idle_time);
  fclose(fd);

  return (user_time + nice_time + system_time + idle_time);
}

/**
 * @brief Get the Cpu Proc Occupy object
 * 
 * @param pid 
 * @return uint64 
 */
inline int64_t GetCpuProcOccupy(int pid) {
  // get specific pid cpu use time
  int64_t tmp_pid = 0;
  int64_t utime = 0;   // user time
  int64_t stime = 0;   // kernel time
  int64_t cutime = 0;  // all user time
  int64_t cstime = 0;  // all dead time

  char file_name[64] = {0};

  char line_buff[1024] = {0};
  snprintf(file_name, sizeof(file_name), "/proc/%d/stat", pid);

  FILE* fd = fopen(file_name, "r");
  if (nullptr == fd)
    return 0;

  fgets(line_buff, sizeof(line_buff), fd);

  sscanf(line_buff, "%ld", &tmp_pid);
  const char* q = GetItems(line_buff, PROCESS_ITEM);
  sscanf(q, "%ld %ld %ld %ld", &utime, &stime, &cutime, &cstime);
  fclose(fd);

  return (utime + stime + cutime + cstime);
}

/**
 * @brief Get the Cpu Usage Ratio object
 * 
 * @return float 
 */
inline float GetCpuUsageRatio() {
  static int64_t last_total_cpu_time = 0;
  static int64_t last_proc_cpu_time = 0;
  int64_t curr_total_cpu_time = GetCpuTotalOccupy();
  int64_t curr_proc_cpu_time = GetCpuProcOccupy(GetCurrentPid());

  float pcpu = 0.0;
  if (0 != curr_total_cpu_time - last_total_cpu_time &&
      last_total_cpu_time != 0 && last_proc_cpu_time != 0) {
    pcpu = static_cast<float>((curr_proc_cpu_time - last_proc_cpu_time)) /
           static_cast<float>(curr_total_cpu_time - last_total_cpu_time);
  }
  int cpu_num = get_nprocs();
  pcpu *= cpu_num;
  last_total_cpu_time = curr_total_cpu_time;
  last_proc_cpu_time = curr_proc_cpu_time;
  return pcpu;
}

/**
 * @brief get specific process physical memeory occupation size by pid (MB)
 * 
 * @return float 
 */
inline float GetMemoryUsage() {
  char file_name[64] = {0};
  FILE* fd;
  char line_buff[512] = {0};
  snprintf(file_name, sizeof(file_name), "/proc/%d/status", GetCurrentPid());
  fd = fopen(file_name, "r");
  if (nullptr == fd)
    return 0;
  char name[64];
  int vmrss = 0;
  for (int i = 0; i < VMRSS_LINE - 1; i++)
    fgets(line_buff, sizeof(line_buff), fd);
  fgets(line_buff, sizeof(line_buff), fd);
  sscanf(line_buff, "%s %d", name, &vmrss);
  fclose(fd);
  // cnvert VmRSS from KB to MB
  return static_cast<float>(vmrss / KB2MB);
}

/**
 * @brief Get the Mem Occupy object
 * 
 * @param mem 
 */
inline void GetMemOccupy(MEM_OCCUPY* const mem) {
  char buff[256];
  FILE* fd = fopen("/proc/meminfo", "r");
  if (fd == nullptr) {
    return;
  }
  fgets(buff, sizeof(buff), fd);
  sscanf(buff, "%s %u ", mem->name1, &mem->MemTotal);
  fgets(buff, sizeof(buff), fd);
  sscanf(buff, "%s %u ", mem->name2, &mem->MemFree);
  fgets(buff, sizeof(buff), fd);
  // sscanf(buff, "%s %lu ", mem->name3, &mem->Buffers);
  // fgets(buff, sizeof(buff), fd);
  // sscanf(buff, "%s %lu ", mem->name4, &mem->Cached);
  // fgets(buff, sizeof(buff), fd);
  // sscanf(buff, "%s %lu", mem->name5, &mem->SwapCached);
  fclose(fd);
}

/**
 * @brief Get the Cpu Occupy object
 * 
 * @param cpust 
 * @return int 
 */
inline int GetCpuOccupy(CPU_OCCUPY* const cpust) {
  FILE* fd;
  char buff[256];
  fd = fopen("/proc/stat", "r");
  if (fd == nullptr) {
    return 0;
  }
  fgets(buff, sizeof(buff), fd);

  sscanf(buff, "%s %u %u %u %u %u %u %u", cpust->name, &cpust->user,
         &cpust->nice, &cpust->system, &cpust->idle, &cpust->lowait,
         &cpust->irq, &cpust->softirq);

  fclose(fd);

  return 0;
}

/**
 * @brief 
 * 
 * @param o 
 * @param n 
 * @return double 
 */
inline double CalCpuOccupy(CPU_OCCUPY* o, CPU_OCCUPY* n) {
  int od, nd;
  double cpu_use = 0;

  od = static_cast<int>(
      o->user + o->nice + o->system + o->idle + o->lowait + o->irq +
      o->softirq);  // 第一次(用户+优先级+系统+空闲)的时间再赋给od
  nd = static_cast<int>(
      n->user + n->nice + n->system + n->idle + n->lowait + n->irq +
      n->softirq);  // 第二次(用户+优先级+系统+空闲)的时间再赋给od
  double sum = nd - od;
  double idle = n->idle - o->idle;
  idle = n->user + n->system + n->nice - o->user - o->system - o->nice;
  cpu_use = idle / sum;
  return cpu_use;
}

/**
 * @brief Get the All Cpu Data object
 * 
 * @return double 
 */
inline double GetAllCpuData() {
  static CPU_OCCUPY last_cpu_stat;
  static bool first = true;
  CPU_OCCUPY curr_cpu_stat;
  double all_cpu_use = 0.0;
  // 第一次获取cpu使用情况
  GetCpuOccupy(&curr_cpu_stat);
  if (first) {
    all_cpu_use = 0.0;
    first = false;
  } else {
    // 计算cpu使用率
    all_cpu_use = CalCpuOccupy(&last_cpu_stat, &curr_cpu_stat);
  }
  last_cpu_stat = curr_cpu_stat;
  return all_cpu_use;
}

/**
 * @brief Get the Sys Tem Mem Data object
 * 
 * @return std::pair<double, double> 
 */
inline std::pair<double, double> GetSystemMemData() {
  MEM_OCCUPY mem_stat;
  GetMemOccupy(&mem_stat);
  return std::make_pair(mem_stat.MemFree * 1.0 / KB2MB,
                        mem_stat.MemTotal * 1.0 / KB2MB);
}

// NOLINTEND
}  // namespace CpuRecorder
}  // namespace planning
}  // namespace TL
