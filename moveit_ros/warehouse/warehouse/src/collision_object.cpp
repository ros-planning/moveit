
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

/* Authors: Ioan Sucan and Rafael A. Rojas*/

#include <moveit/warehouse/collision_object.h>

#include <utility>

const std::string moveit_warehouse::CollisionObjectStorage::DATABASE_NAME = "collision_objects";

const std::string moveit_warehouse::CollisionObjectStorage::COLLISIONOBJECT_ID_NAME = "collisionobject_id";
const std::string moveit_warehouse::CollisionObjectStorage::COLLISIONOBJECT_GROUP_NAME = "group_id";

using warehouse_ros::Metadata;
using warehouse_ros::Query;

moveit_warehouse::CollisionObjectStorage::CollisionObjectStorage(warehouse_ros::DatabaseConnection::Ptr conn)
  : MoveItMessageStorage(std::move(conn))
{
  createCollections();
}

void moveit_warehouse::CollisionObjectStorage::createCollections()
{
  collision_objects_collection_ = conn_->openCollectionPtr<moveit_msgs::CollisionObject>(DATABASE_NAME, "constraints");
}

void moveit_warehouse::CollisionObjectStorage::reset()
{
  collision_objects_collection_.reset();
  conn_->dropDatabase(DATABASE_NAME);
  createCollections();
}

void moveit_warehouse::CollisionObjectStorage::addCollisionObject(const moveit_msgs::CollisionObject& msg,
                                                                  const std::string& group)
{
  bool replace = false;
  if (hasCollisionObject(msg.id, group))
  {
    removeCollisionObject(msg.id, group);
    replace = true;
  }
  Metadata::Ptr metadata = collision_objects_collection_->createMetadata();
  metadata->append(COLLISIONOBJECT_ID_NAME, msg.id);
  metadata->append(COLLISIONOBJECT_GROUP_NAME, group);
  collision_objects_collection_->insert(msg, metadata);
  ROS_DEBUG("%s constraints '%s'", replace ? "Replaced" : "Added", msg.id.c_str());
}

bool moveit_warehouse::CollisionObjectStorage::hasCollisionObject(const std::string& name,
                                                                  const std::string& group) const
{
  Query::Ptr q = collision_objects_collection_->createQuery();
  q->append(COLLISIONOBJECT_ID_NAME, name);
  if (!group.empty())
    q->append(COLLISIONOBJECT_GROUP_NAME, group);
  std::vector<CollisionObjectWithMetadata> colision_objects = collision_objects_collection_->queryList(q, true);
  return !colision_objects.empty();
}

void moveit_warehouse::CollisionObjectStorage::getKnownCollisionObjects(const std::string& regex,
                                                                        std::vector<std::string>& names,
                                                                        const std::string& group) const
{
  getKnownCollisionObjects(names, group);
  filterNames(regex, names);
}

void moveit_warehouse::CollisionObjectStorage::getKnownCollisionObjects(std::vector<std::string>& names,
                                                                        const std::string& group) const
{
  names.clear();
  Query::Ptr q = collision_objects_collection_->createQuery();
  if (!group.empty())
    q->append(COLLISIONOBJECT_GROUP_NAME, group);
  std::vector<CollisionObjectWithMetadata> colision_objects =
      collision_objects_collection_->queryList(q, true, COLLISIONOBJECT_ID_NAME, true);
  for (const CollisionObjectWithMetadata& it : colision_objects)
    if (it->lookupField(COLLISIONOBJECT_ID_NAME))
      names.push_back(it->lookupString(COLLISIONOBJECT_ID_NAME));
}

bool moveit_warehouse::CollisionObjectStorage::getCollisionObject(CollisionObjectWithMetadata& msg_m,
                                                                  const std::string& name,
                                                                  const std::string& group) const
{
  Query::Ptr q = collision_objects_collection_->createQuery();
  q->append(COLLISIONOBJECT_ID_NAME, name);
  if (!group.empty())
    q->append(COLLISIONOBJECT_GROUP_NAME, group);
  std::vector<CollisionObjectWithMetadata> colision_objects = collision_objects_collection_->queryList(q, false);
  if (colision_objects.empty())
    return false;
  else
  {
    msg_m = colision_objects.back();
    return true;
  }
}

void moveit_warehouse::CollisionObjectStorage::removeCollisionObject(const std::string& name, const std::string& group)
{
  Query::Ptr q = collision_objects_collection_->createQuery();
  q->append(COLLISIONOBJECT_ID_NAME, name);
  if (!group.empty())
    q->append(COLLISIONOBJECT_GROUP_NAME, group);
  unsigned int rem = collision_objects_collection_->removeMessages(q);
  ROS_DEBUG("Removed %u CollisionObject message (named '%s')", rem, name.c_str());
}
