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

#pragma once

#include <shape_msgs/Mesh.h>
#include <geometric_shapes/shape_operations.h>

namespace moveit
{
namespace core
{
/// \brief The type used to hash meshes.
using MeshHash = std::size_t;

/// \brief The type used to hash times.
using TimeHash = std::pair<int, int>;

/// \brief Convert a shape_msgs::mesh to a std::string.
std::string meshString(const shape_msgs::Mesh& mesh);

/// \brief Convert a std::string to a MeshHash.
MeshHash hashMeshString(const std::string& mesh_string);

/// \brief Convert a ros::Time to a TimeHash.
TimeHash hashTime(const ros::Time& time);

/** \brief Control block used to store a shapes::ShapeConstPtr, and related metadata.

    The metadata allows for exact comparisons with shape_msgs::Mesh types,
    and for tracking the relative usage times of meshes.*/
struct MeshCacheControlBlock
{
  /// \brief The input shape_msgs::Mesh as a std::string.
  std::string mesh_string_;
  /// \brief The hash of `mesh_string_`.
  MeshHash mesh_hash_;
  /// \brief The cached shape.
  shapes::ShapeConstPtr shape_;
  /// \brief The last time this shape was used.
  ros::Time last_used_;
  /// \brief The hash of `last_used_`.
  TimeHash last_used_hash_;
  /// \brief The approximate memory use of this control block.
  std::size_t approximateMemoryUse() const;
};

/** \brief A cache of shapes::ShapeConstPtr shapes, constructed from shape_msgs::Mesh.

    This cache is designed to allow shapes to be aliased, rather than being regenerated.
    Automatically evicts members to maintain an (approximate) maximum memory usage.
    Uses a least-recently-used cache eviction policy.*/
class MeshCache
{
public:
  /** \brief Cache constructor
      \param min_size_to_cache The minimum size of a shape_msgs::Mesh to cache.
      \param max_cache_size The (approximate) maximum memory usage of the cache.*/
  MeshCache(std::size_t min_size_to_cache = 0, std::size_t max_cache_size = std::numeric_limits<std::size_t>::max());
  /** \brief Returns a reference to a thread-local static MeshCache. */
  static MeshCache& threadLocalCache(std::size_t min_size_to_cache = 0,
                                     std::size_t max_cache_size = std::numeric_limits<std::size_t>::max());
  /** \brief Returns the shapes::ShapeConstrPtr corresponding to `mesh`.

      The returned shape will come from the cache if present.
      The generated shape will be added to the cache if it is not present.*/
  shapes::ShapeConstPtr getShape(const shape_msgs::Mesh& mesh);

private:
  /** \brief Adds `control_block` to the cache.*/
  void cacheControlBlock(const std::shared_ptr<MeshCacheControlBlock>& control_block);
  /** \brief Removes `control_block` from the cache.*/
  void removeControlBlock(const std::shared_ptr<MeshCacheControlBlock>& control_block);
  /** \brief Updates the last used timestamp of `control_block` to now.*/
  void updateLastUsed(const std::shared_ptr<MeshCacheControlBlock>& control_block);

  const std::size_t min_size_to_cache_;
  const std::size_t max_cache_size_;
  /** \brief Control blocks stored by mesh hash, for mesh lookup.*/
  std::map<MeshHash, std::shared_ptr<MeshCacheControlBlock>> cache_by_mesh_hash_;
  /** \brief Control blocks stored by timestamp, for cache eviction.*/
  std::map<TimeHash, std::shared_ptr<MeshCacheControlBlock>> cache_by_last_used_;
  /** \brief The current (approximate) memory usage of the cache.*/
  std::size_t cache_size_;
};
}  // namespace core
}  // namespace moveit
