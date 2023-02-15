/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, Bielefeld University
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
 *   * Neither the name of Bielefeld University nor the names of its
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

#include <memory>
#include <mutex>

#include <moveit/py_bindings_tools/roscpp_initializer.h>
#include <ros/init.h>
#include <ros/spinner.h>

static const char* LOGNAME = "py_tools";

namespace moveit
{
namespace py_bindings_tools
{
namespace
{
/// singleton class to initialize ROS C++ (once) from Python
struct InitProxy
{
  static void init(const std::string& node_name = "moveit_python_wrappers",
                   const std::map<std::string, std::string>& remappings = std::map<std::string, std::string>(),
                   uint32_t options = 0);
  static void reset(bool shutdown);

  InitProxy(const std::string& node_name, const std::map<std::string, std::string>& remappings, uint32_t options)
  {
    usage = 0;
    if (!ros::isInitialized())
      ros::init(remappings, node_name, options | ros::init_options::NoSigintHandler);
    spinner = std::make_unique<ros::AsyncSpinner>(1);
    spinner->start();
  }
  ~InitProxy()
  {
    spinner->stop();
    spinner.reset();
    if (ros::isInitialized() && !ros::isShuttingDown())
      ros::shutdown();
  }

  static std::mutex mutex_;
  static std::unique_ptr<InitProxy> singleton_instance_;
  unsigned int usage;
  std::unique_ptr<ros::AsyncSpinner> spinner;
};
std::mutex InitProxy::mutex_;
std::unique_ptr<InitProxy> InitProxy::singleton_instance_;
}  // namespace

void InitProxy::init(const std::string& node_name, const std::map<std::string, std::string>& remappings,
                     uint32_t options)
{
  std::unique_lock<std::mutex> lock(mutex_);
  if (!singleton_instance_)
    singleton_instance_ = std::make_unique<InitProxy>(node_name, remappings, options);
  ++singleton_instance_->usage;
}

void InitProxy::reset(bool shutdown)
{
  std::unique_lock<std::mutex> lock(mutex_);
  if (!singleton_instance_ || singleton_instance_->usage == 0)
    ROS_ERROR_NAMED(LOGNAME, "Calling roscpp_shutdown() without a matching call to roscpp_init()!");
  else if (--singleton_instance_->usage == 0 && shutdown)
  {
    ROS_WARN_NAMED(LOGNAME, "It's not recommended to call roscpp_shutdown().");
    singleton_instance_.reset();
  }
}

ROScppInitializer::ROScppInitializer()
{
  InitProxy::init();
}
ROScppInitializer::~ROScppInitializer()
{
  InitProxy::reset(false);
}

void roscpp_init(const std::string& node_name, const std::map<std::string, std::string>& remappings, uint32_t options)
{
  InitProxy::init(node_name, remappings, options);
}

void roscpp_shutdown()
{
  // This might shutdown ROS C++, which will stop ROS logging!
  InitProxy::reset(true);
}

}  // namespace py_bindings_tools
}  // namespace moveit
