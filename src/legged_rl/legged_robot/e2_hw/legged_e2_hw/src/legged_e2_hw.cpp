/********************************************************************************
Modified Copyright (c) 2023-2024, Noetix Robotics.Co.Ltd. All rights reserved.

For further information, contact: huaxinghuang@noetixrobotics.com or visit our website
at www.noetixrobotics.com.
********************************************************************************/

#include <legged_hw/LeggedHWLoop.h>
#include "E2HW.h"

#include <sys/mman.h>
#include <sched.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>

int main(int argc, char** argv)
{
  // 初始化ROS节点
  ros::init(argc, argv, "legged_e2_hw");
  // 创建ROS节点句柄
  ros::NodeHandle nh;
  // 创建用于访问私有命名空间参数的节点句柄
  ros::NodeHandle robot_hw_nh("~");
  bool requireRealtime = false;
  robot_hw_nh.param("require_realtime", requireRealtime, false);

  // Lock all current and future memory into RAM so the 500 Hz RT loop does not
  // hit page faults (a major source of ms-level latency spikes). Requires
  // CAP_IPC_LOCK / a sufficient memlock rlimit. In require_realtime mode a
  // failure is fatal; development mode keeps the legacy warning-only behavior.
  if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0)
  {
    if (requireRealtime)
    {
      ROS_FATAL("require_realtime: mlockall failed: %s", std::strerror(errno));
      return 1;
    }
    ROS_WARN("mlockall failed: %s (need CAP_IPC_LOCK or raised memlock limit). RT loop may page-fault.",
             std::strerror(errno));
  }
  else
  {
    ROS_INFO("mlockall succeeded: process memory locked for real-time execution.");
  }
  // Keep all non-RT threads (the AsyncSpinner callback threads and this main
  // thread) off the isolated RT core. Threads inherit the creating thread's CPU
  // affinity mask, so setting it here before spinner.start() propagates to the
  // spinner threads. The RT loop thread later overrides its own affinity
  // (pthread_setaffinity_np) to land on the isolated core.
  {
    int rtCore = -1;
    robot_hw_nh.param("cpu_affinity", rtCore, -1);
    const long nCpus = sysconf(_SC_NPROCESSORS_ONLN);
    if (rtCore >= 0 && nCpus > 1 && rtCore < static_cast<long>(nCpus))
    {
      cpu_set_t cpuSet;
      CPU_ZERO(&cpuSet);
      for (long c = 0; c < nCpus; ++c)
      {
        if (c != rtCore)
        {
          CPU_SET(c, &cpuSet);
        }
      }
      if (sched_setaffinity(0, sizeof(cpu_set_t), &cpuSet) != 0)
      {
        if (requireRealtime)
        {
          ROS_FATAL("require_realtime: failed to keep non-RT threads off core %d: %s",
                    rtCore, std::strerror(errno));
          return 1;
        }
        ROS_WARN("Failed to keep non-RT threads off core %d: %s", rtCore, std::strerror(errno));
      }
      else
      {
        ROS_INFO("Non-RT threads restricted to cores other than %d (isolated RT core).", rtCore);
      }
    }
    else if (requireRealtime)
    {
      ROS_FATAL("require_realtime: cpu_affinity must identify a valid control CPU on a multi-core system");
      return 1;
    }
  }

  // 创建并启动一个异步旋转器，允许多线程处理回调
  ros::AsyncSpinner spinner(3); // 使用3个线程
  spinner.start();

  try
  {
    // 创建硬件接口的共享指针实例
    std::shared_ptr<legged::E2HW> legged_e2_hw = std::make_shared<legged::E2HW>();

    // 初始化硬件接口
    if (!legged_e2_hw->init(nh, robot_hw_nh))
    {
      ROS_FATAL("Failed to initialize E2 hardware interface; control loop will not start.");
      return 1;
    }
    std::cout << "1:"<<"legged_e2_hw line"<<__LINE__<<std::endl;
    // 通过调用LeggedHWLoop类的构造函数，我们创建了一个名为control_loop的LeggedHWLoop对象，这个对象会启动硬件控制循环
    legged::LeggedHWLoop control_loop(nh, legged_e2_hw);std::cout<< "2:"<<"legged_e2_hw line"<<__LINE__<<std::endl;
    std::cout<< "3:"<<"legged_e2_hw line"<<__LINE__<<std::endl;
    // 等待ROS节点关闭
    ros::waitForShutdown();
  }
  catch (const ros::Exception& e)
  {
    // 如果发生异常，打印错误信息并退出
    ROS_FATAL_STREAM("Error in the hardware interface:\n"
                     << "\t" << e.what());
    return 1; // 返回1表示异常退出
  }
  catch (const std::exception& e)
  {
    ROS_FATAL_STREAM("Hardware interface initialization failed: " << e.what());
    return 1;
  }

  return 0; // 正常退出
}

