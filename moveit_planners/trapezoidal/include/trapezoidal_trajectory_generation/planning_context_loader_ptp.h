/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018 Pilz GmbH & Co. KG
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
 *   * Neither the name of Pilz GmbH & Co. KG nor the names of its
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

#pragma once

#include "trapezoidal_trajectory_generation/planning_context_loader.h"

#include <moveit/planning_interface/planning_interface.h>

namespace trapezoidal
{
/**
 * @brief Plugin that can generate instances of PlanningContextPTP.
 * Generates instances of PlanningContextPTP.
 */
class PlanningContextLoaderPTP : public PlanningContextLoader
{
public:
  PlanningContextLoaderPTP();
  virtual ~PlanningContextLoaderPTP();

  /**
   * @brief return a instance of trapezoidal::PlanningContextPTP
   * @param planning_context returned context
   * @param name
   * @param group
   * @return true on success, false otherwise
   */
  virtual bool loadContext(planning_interface::PlanningContextPtr& planning_context, const std::string& name,
                           const std::string& group) const override;
};

typedef boost::shared_ptr<PlanningContextLoaderPTP> PlanningContextLoaderPTPPtr;
typedef boost::shared_ptr<const PlanningContextLoaderPTP> PlanningContextLoaderPTPConstPtr;

}  // namespace trapezoidal
