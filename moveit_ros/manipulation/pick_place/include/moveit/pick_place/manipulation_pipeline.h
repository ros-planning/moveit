/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, Willow Garage, Inc.
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

#pragma once

#include <moveit/pick_place/manipulation_stage.h>
#include <boost/thread.hpp>
#include <boost/function.hpp>
#include <vector>
#include <deque>

namespace pick_place
{
/** \brief Represent the sequence of steps that are executed for a manipulation plan */
class ManipulationPipeline
{
public:
  ManipulationPipeline(const std::string& name, unsigned int nthreads);
  virtual ~ManipulationPipeline();

  const std::string& getName() const
  {
    return name_;
  }

  void setSolutionCallback(const boost::function<void()>& callback)
  {
    solution_callback_ = callback;
  }

  void setEmptyQueueCallback(const boost::function<void()>& callback)
  {
    empty_queue_callback_ = callback;
  }

  ManipulationPipeline& addStage(const ManipulationStagePtr& next);
  const ManipulationStagePtr& getFirstStage() const;
  const ManipulationStagePtr& getLastStage() const;
  void reset();

  void setVerbose(bool flag);

  void signalStop();
  void start();
  void stop();

  void push(const ManipulationPlanPtr& grasp);
  void clear();

  const std::vector<ManipulationPlanPtr>& getSuccessfulManipulationPlans() const
  {
    return success_;
  }

  const std::vector<ManipulationPlanPtr>& getFailedManipulationPlans() const
  {
    return failed_;
  }

  void reprocessLastFailure();

protected:
  void processingThread(unsigned int index);

  std::string name_;
  unsigned int nthreads_;
  bool verbose_;
  std::vector<ManipulationStagePtr> stages_;

  std::deque<ManipulationPlanPtr> queue_;
  std::vector<ManipulationPlanPtr> success_;
  std::vector<ManipulationPlanPtr> failed_;

  std::vector<boost::thread*> processing_threads_;
  boost::condition_variable queue_access_cond_;
  boost::mutex queue_access_lock_;
  boost::mutex result_lock_;

  boost::function<void()> solution_callback_;
  boost::function<void()> empty_queue_callback_;
  unsigned int empty_queue_threads_;

  bool stop_processing_;
};
}

