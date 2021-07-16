/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Willow Garage, Inc.
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

/* Author: Acorn Pooley, Ioan Sucan */

#include <moveit/collision_detection/world.h>
#include <geometric_shapes/check_isometry.h>
#include <boost/algorithm/string/predicate.hpp>
#include <ros/console.h>

namespace collision_detection
{
World::World()
{
}

World::World(const World& other)
{
  objects_ = other.objects_;
}

World::~World()
{
  while (!observers_.empty())
    removeObserver(observers_.front());
}

inline void World::addToObjectInternal(const ObjectPtr& obj, const shapes::ShapeConstPtr& shape,
                                       const Eigen::Isometry3d& pose)
{
  obj->shapes_.push_back(shape);
  ASSERT_ISOMETRY(pose)  // unsanitized input, could contain a non-isometry
  obj->shape_poses_.push_back(pose);
}

void World::addToObject(const std::string& id, const std::vector<shapes::ShapeConstPtr>& shapes,
                        const EigenSTL::vector_Isometry3d& poses)
{
  if (shapes.size() != poses.size())
  {
    ROS_ERROR_NAMED("collision_detection", "Number of shapes and number of poses do not match. "
                                           "Not adding this object to collision world.");
    return;
  }

  if (shapes.empty())
    return;

  int action = ADD_SHAPE;

  ObjectPtr& obj = objects_[id];
  if (!obj)
  {
    obj = std::make_shared<Object>(id);
    action |= CREATE;
  }

  ensureUnique(obj);

  for (std::size_t i = 0; i < shapes.size(); ++i)
    addToObjectInternal(obj, shapes[i], poses[i]);

  notify(obj, Action(action));
}

void World::addToObject(const std::string& id, const shapes::ShapeConstPtr& shape, const Eigen::Isometry3d& pose)
{
  int action = ADD_SHAPE;

  ObjectPtr& obj = objects_[id];
  if (!obj)
  {
    obj = std::make_shared<Object>(id);
    action |= CREATE;
  }

  ensureUnique(obj);
  addToObjectInternal(obj, shape, pose);

  notify(obj, Action(action));
}

std::vector<std::string> World::getObjectIds() const
{
  std::vector<std::string> id;
  for (const auto& object : objects_)
    id.push_back(object.first);
  return id;
}

World::ObjectConstPtr World::getObject(const std::string& object_id) const
{
  auto it = objects_.find(object_id);
  if (it == objects_.end())
    return ObjectConstPtr();
  else
    return it->second;
}

void World::ensureUnique(ObjectPtr& obj)
{
  if (obj && !obj.unique())
    obj = std::make_shared<Object>(*obj);
}

bool World::hasObject(const std::string& object_id) const
{
  return objects_.find(object_id) != objects_.end();
}

bool World::knowsTransform(const std::string& name) const
{
  // Check object names first
  std::map<std::string, ObjectPtr>::const_iterator it = objects_.find(name);
  if (it != objects_.end())
    // only accept object name as frame if it is associated to a unique shape
    return !it->second->shape_poses_.empty();
  else  // Then objects' subframes
  {
    for (const std::pair<const std::string, ObjectPtr>& object : objects_)
    {
      // if "object name/" matches start of object_id, we found the matching object
      if (boost::starts_with(name, object.first) && name[object.first.length()] == '/')
        return object.second->subframe_poses_.find(name.substr(object.first.length() + 1)) !=
               object.second->subframe_poses_.end();
    }
  }
  return false;
}

const Eigen::Isometry3d& World::getTransform(const std::string& name) const
{
  bool found;
  const Eigen::Isometry3d& result = getTransform(name, found);
  if (!found)
    throw std::runtime_error("No transform found with name: " + name);
  return result;
}

const Eigen::Isometry3d& World::getTransform(const std::string& name, bool& frame_found) const
{
  // assume found
  frame_found = true;

  std::map<std::string, ObjectPtr>::const_iterator it = objects_.find(name);
  if (it != objects_.end())
  {
    if (!it->second->shape_poses_.empty())
      return it->second->shape_poses_[0];
  }
  else  // Search within subframes
  {
    for (const std::pair<const std::string, ObjectPtr>& object : objects_)
    {
      // if "object name/" matches start of object_id, we found the matching object
      if (boost::starts_with(name, object.first) && name[object.first.length()] == '/')
      {
        auto it = object.second->subframe_poses_.find(name.substr(object.first.length() + 1));
        if (it != object.second->subframe_poses_.end())
          return it->second;
      }
    }
  }

  // we need a persisting isometry for the API
  static const Eigen::Isometry3d IDENTITY_TRANSFORM = Eigen::Isometry3d::Identity();
  frame_found = false;
  return IDENTITY_TRANSFORM;
}

bool World::moveShapeInObject(const std::string& object_id, const shapes::ShapeConstPtr& shape,
                              const Eigen::Isometry3d& pose)
{
  auto it = objects_.find(object_id);
  if (it != objects_.end())
  {
    unsigned int n = it->second->shapes_.size();
    for (unsigned int i = 0; i < n; ++i)
      if (it->second->shapes_[i] == shape)
      {
        ensureUnique(it->second);
        ASSERT_ISOMETRY(pose)  // unsanitized input, could contain a non-isometry
        it->second->shape_poses_[i] = pose;

        notify(it->second, MOVE_SHAPE);
        return true;
      }
  }
  return false;
}

bool World::moveObject(const std::string& object_id, const Eigen::Isometry3d& transform)
{
  auto it = objects_.find(object_id);
  if (it == objects_.end())
    return false;
  if (transform.isApprox(Eigen::Isometry3d::Identity()))
    return true;  // object already at correct location
  ensureUnique(it->second);
  for (size_t i = 0, n = it->second->shapes_.size(); i < n; ++i)
  {
    ASSERT_ISOMETRY(transform)  // unsanitized input, could contain a non-isometry
    it->second->shape_poses_[i] = transform * it->second->shape_poses_[i];
  }
  notify(it->second, MOVE_SHAPE);
  return true;
}

bool World::removeShapeFromObject(const std::string& object_id, const shapes::ShapeConstPtr& shape)
{
  auto it = objects_.find(object_id);
  if (it != objects_.end())
  {
    unsigned int n = it->second->shapes_.size();
    for (unsigned int i = 0; i < n; ++i)
      if (it->second->shapes_[i] == shape)
      {
        ensureUnique(it->second);
        it->second->shapes_.erase(it->second->shapes_.begin() + i);
        it->second->shape_poses_.erase(it->second->shape_poses_.begin() + i);

        if (it->second->shapes_.empty())
        {
          notify(it->second, DESTROY);
          objects_.erase(it);
        }
        else
        {
          notify(it->second, REMOVE_SHAPE);
        }
        return true;
      }
  }
  return false;
}

bool World::removeObject(const std::string& object_id)
{
  auto it = objects_.find(object_id);
  if (it != objects_.end())
  {
    notify(it->second, DESTROY);
    objects_.erase(it);
    return true;
  }
  return false;
}

void World::clearObjects()
{
  notifyAll(DESTROY);
  objects_.clear();
}

bool World::setSubframesOfObject(const std::string& object_id, const moveit::core::FixedTransformsMap& subframe_poses)
{
  auto obj_pair = objects_.find(object_id);
  if (obj_pair == objects_.end())
  {
    return false;
  }
  for (const auto& t : subframe_poses)
  {
    ASSERT_ISOMETRY(t.second)  // unsanitized input, could contain a non-isometry
  }
  obj_pair->second->subframe_poses_ = subframe_poses;
  return true;
}

World::ObserverHandle World::addObserver(const ObserverCallbackFn& callback)
{
  auto o = new Observer(callback);
  observers_.push_back(o);
  return ObserverHandle(o);
}

void World::removeObserver(ObserverHandle observer_handle)
{
  for (auto obs = observers_.begin(); obs != observers_.end(); ++obs)
  {
    if (*obs == observer_handle.observer_)
    {
      delete *obs;
      observers_.erase(obs);
      return;
    }
  }
}

void World::notifyAll(Action action)
{
  for (std::map<std::string, ObjectPtr>::const_iterator it = objects_.begin(); it != objects_.end(); ++it)
    notify(it->second, action);
}

void World::notify(const ObjectConstPtr& obj, Action action)
{
  for (Observer* observer : observers_)
    observer->callback_(obj, action);
}

void World::notifyObserverAllObjects(const ObserverHandle observer_handle, Action action) const
{
  for (auto observer : observers_)
  {
    if (observer == observer_handle.observer_)
    {
      // call the callback for each object
      for (const auto& object : objects_)
        observer->callback_(object.second, action);
      break;
    }
  }
}

}  // end of namespace collision_detection
