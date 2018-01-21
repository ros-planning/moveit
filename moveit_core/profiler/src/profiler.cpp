/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Ioan Sucan */

#include "moveit/profiler/profiler.h"

moveit::tools::Profiler& moveit::tools::Profiler::Instance()
{
  static Profiler p(false, false);
  return p;
}

#if MOVEIT_ENABLE_PROFILING

#include <console_bridge/console.h>
#include <vector>
#include <algorithm>
#include <sstream>

void moveit::tools::Profiler::start()
{
  lock_.lock();
  if (!running_)
  {
    tinfo_.set();
    running_ = true;
  }
  lock_.unlock();
}

void moveit::tools::Profiler::stop()
{
  lock_.lock();
  if (running_)
  {
    tinfo_.update();
    running_ = false;
  }
  lock_.unlock();
}

void moveit::tools::Profiler::clear()
{
  lock_.lock();
  data_.clear();
  tinfo_ = TimeInfo();
  if (running_)
    tinfo_.set();
  lock_.unlock();
}

void moveit::tools::Profiler::event(const std::string& name, const unsigned int times)
{
  lock_.lock();
  data_[boost::this_thread::get_id()].events[name] += times;
  lock_.unlock();
}

void moveit::tools::Profiler::average(const std::string& name, const double value)
{
  lock_.lock();
  AvgInfo& a = data_[boost::this_thread::get_id()].avg[name];
  a.total += value;
  a.totalSqr += value * value;
  a.parts++;
  lock_.unlock();
}

void moveit::tools::Profiler::begin(const std::string& name)
{
  lock_.lock();
  data_[boost::this_thread::get_id()].time[name].set();
  lock_.unlock();
}

void moveit::tools::Profiler::end(const std::string& name)
{
  lock_.lock();
  data_[boost::this_thread::get_id()].time[name].update();
  lock_.unlock();
}

namespace
{
inline double to_seconds(const boost::posix_time::time_duration& d)
{
  return (double)d.total_microseconds() / 1000000.0;
}
}

void moveit::tools::Profiler::status(std::ostream& out, bool merge)
{
  stop();
  lock_.lock();
  printOnDestroy_ = false;

  out << std::endl;
  out << " *** Profiling statistics. Total counted time : " << to_seconds(tinfo_.total) << " seconds" << std::endl;

  if (merge)
  {
    PerThread combined;
    for (std::map<boost::thread::id, PerThread>::const_iterator it = data_.begin(); it != data_.end(); ++it)
    {
      for (const auto& event : it->second.events)
        combined.events[event.first] += event.second;
      for (const auto& iavg : it->second.avg)
      {
        combined.avg[iavg.first].total += iavg.second.total;
        combined.avg[iavg.first].totalSqr += iavg.second.totalSqr;
        combined.avg[iavg.first].parts += iavg.second.parts;
      }
      for (const auto& itm : it->second.time)
      {
        TimeInfo& tc = combined.time[itm.first];
        tc.total = tc.total + itm.second.total;
        tc.parts = tc.parts + itm.second.parts;
        if (tc.shortest > itm.second.shortest)
          tc.shortest = itm.second.shortest;
        if (tc.longest < itm.second.longest)
          tc.longest = itm.second.longest;
      }
    }
    printThreadInfo(out, combined);
  }
  else
    for (std::map<boost::thread::id, PerThread>::const_iterator it = data_.begin(); it != data_.end(); ++it)
    {
      out << "Thread " << it->first << ":" << std::endl;
      printThreadInfo(out, it->second);
    }
  lock_.unlock();
}

void moveit::tools::Profiler::console()
{
  std::stringstream ss;
  ss << std::endl;
  status(ss, true);
  logInform(ss.str().c_str());
}

/// @cond IGNORE
namespace
{
struct dataIntVal
{
  std::string name;
  unsigned long int value;
};

struct SortIntByValue
{
  bool operator()(const dataIntVal& a, const dataIntVal& b) const
  {
    return a.value > b.value;
  }
};

struct dataDoubleVal
{
  std::string name;
  double value;
};

struct SortDoubleByValue
{
  bool operator()(const dataDoubleVal& a, const dataDoubleVal& b) const
  {
    return a.value > b.value;
  }
};
}
/// @endcond

void moveit::tools::Profiler::printThreadInfo(std::ostream& out, const PerThread& data)
{
  double total = to_seconds(tinfo_.total);

  std::vector<dataIntVal> events;
  for (const auto& event : data.events)
  {
    dataIntVal next = { event.first, event.second };
    events.push_back(next);
  }
  std::sort(events.begin(), events.end(), SortIntByValue());
  if (!events.empty())
    out << "Events:" << std::endl;
  for (auto& event : events)
    out << event.name << ": " << event.value << std::endl;

  std::vector<dataDoubleVal> avg;
  for (const auto& ia : data.avg)
  {
    dataDoubleVal next = { ia.first, ia.second.total / (double)ia.second.parts };
    avg.push_back(next);
  }
  std::sort(avg.begin(), avg.end(), SortDoubleByValue());
  if (!avg.empty())
    out << "Averages:" << std::endl;
  for (auto& i : avg)
  {
    const AvgInfo& a = data.avg.find(i.name)->second;
    out << i.name << ": " << i.value
        << " (stddev = " << sqrt(fabs(a.totalSqr - (double)a.parts * i.value * i.value) / ((double)a.parts - 1.)) << ")"
        << std::endl;
  }

  std::vector<dataDoubleVal> time;

  for (const auto& itm : data.time)
  {
    dataDoubleVal next = { itm.first, to_seconds(itm.second.total) };
    time.push_back(next);
  }

  std::sort(time.begin(), time.end(), SortDoubleByValue());
  if (!time.empty())
    out << "Blocks of time:" << std::endl;

  double unaccounted = total;
  for (auto& time_data : time)
  {
    const TimeInfo& d = data.time.find(time_data.name)->second;

    double tS = to_seconds(d.shortest);
    double tL = to_seconds(d.longest);
    out << time_data.name << ": " << time_data.value << "s (" << (100.0 * time_data.value / total) << "%), [" << tS
        << "s --> " << tL << " s], " << d.parts << " parts";
    if (d.parts > 0)
    {
      double pavg = to_seconds(d.total) / (double)d.parts;
      out << ", " << pavg << " s on average";
      if (pavg < 1.0)
        out << " (" << 1.0 / pavg << " /s)";
    }
    out << std::endl;
    unaccounted -= time_data.value;
  }
  // if we do not appear to have counted time multiple times, print the unaccounted time too
  if (unaccounted >= 0.0)
  {
    out << "Unaccounted time : " << unaccounted;
    if (total > 0.0)
      out << " (" << (100.0 * unaccounted / total) << " %)";
    out << std::endl;
  }

  out << std::endl;
}

#endif
