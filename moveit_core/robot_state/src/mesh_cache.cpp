/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2023, Rivelin Robotics, Ltd.
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
 *   * Neither the name of Rivelin Robotics nor the names of its
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

#include <moveit/robot_state/mesh_cache.h>

namespace moveit
{
namespace core
{
std::string meshString(const shape_msgs::Mesh& mesh)
{
  std::ostringstream oss;
  oss << mesh;
  return oss.str();
}

MeshHash hashMeshString(const std::string& mesh_string)
{
  return std::hash<std::string>()(mesh_string);
}

TimeHash hashTime(const ros::Time& time)
{
  return std::make_pair(time.sec, time.nsec);
}

std::size_t MeshCacheControlBlock::approximateMemoryUse() const
{
  // This is very approximate, assuming the shape has approximately the same memory use as the mesh string.
  return 2 * mesh_string_.size() * sizeof(char);
}

MeshCache::MeshCache(std::size_t min_size_to_cache, std::size_t max_cache_size)
  : min_size_to_cache_(min_size_to_cache), max_cache_size_(max_cache_size), cache_size_(0)
{
}

MeshCache& MeshCache::threadLocalCache(std::size_t min_size_to_cache, std::size_t max_cache_size)
{
  static thread_local MeshCache cache(min_size_to_cache, max_cache_size);
  return cache;
}

shapes::ShapeConstPtr MeshCache::getShape(const shape_msgs::Mesh& mesh)
{
  // Hash the mesh for lookup.
  std::string mesh_string = meshString(mesh);
  MeshHash mesh_hash = hashMeshString(mesh_string);

  // Look for the mesh in the cache.
  auto it = cache_by_mesh_hash_.find(mesh_hash);
  if (it != cache_by_mesh_hash_.end() and it->second->mesh_string_ == mesh_string)
  {
    // If the mesh is found, return the cached shape and update its last used time.
    auto control_block = it->second;
    updateLastUsed(control_block);
    return control_block->shape_;
  }
  else
  {
    // If the mesh is not found, construct a new shape and cache it.
    shapes::ShapeConstPtr shape(shapes::constructShapeFromMsg(mesh));
    ros::Time last_used = ros::Time::now();
    TimeHash last_used_hash = hashTime(last_used);
    auto control_block = std::make_shared<MeshCacheControlBlock>(
        MeshCacheControlBlock{ mesh_string, mesh_hash, shape, last_used, last_used_hash });
    cacheControlBlock(control_block);
    return shape;
  }
}

void MeshCache::cacheControlBlock(const std::shared_ptr<MeshCacheControlBlock>& control_block)
{
  // Do not cache the control block if it is too small or too large.
  if (control_block->approximateMemoryUse() < min_size_to_cache_)
  {
    return;
  }
  if (control_block->approximateMemoryUse() > max_cache_size_)
  {
    return;
  }

  // If there is a hash collision, evict the existing control block.
  if (cache_by_mesh_hash_.count(control_block->mesh_hash_) > 0)
  {
    removeControlBlock(cache_by_mesh_hash_[control_block->mesh_hash_]);
  }
  if (cache_by_last_used_.count(control_block->last_used_hash_) > 0)
  {
    removeControlBlock(cache_by_last_used_[control_block->last_used_hash_]);
  }

  // If the cache is full, evict the least recently used control blocks.
  cache_size_ += control_block->approximateMemoryUse();
  while (cache_size_ > max_cache_size_)
  {
    removeControlBlock(cache_by_last_used_.begin()->second);
  }

  // Add the control block to the cache.
  cache_by_mesh_hash_[control_block->mesh_hash_] = control_block;
  cache_by_last_used_[control_block->last_used_hash_] = control_block;
}

void MeshCache::removeControlBlock(const std::shared_ptr<MeshCacheControlBlock>& control_block)
{
  cache_by_mesh_hash_.erase(control_block->mesh_hash_);
  cache_by_last_used_.erase(control_block->last_used_hash_);
  cache_size_ -= control_block->approximateMemoryUse();
}

void MeshCache::updateLastUsed(const std::shared_ptr<MeshCacheControlBlock>& control_block)
{
  cache_by_last_used_.erase(hashTime(control_block->last_used_));
  control_block->last_used_ = ros::Time::now();
  cache_by_last_used_[hashTime(control_block->last_used_)] = control_block;
}

}  // namespace core
}  // namespace moveit
