/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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

/* Author: Rafael A. Rojas*/

#include <algorithm>                      // iter_swap
#include <eigen_conversions/eigen_msg.h>  // poseMsgToEigen
#include <kdl_parser/kdl_parser.hpp>
#include <memory>
#include <moveit/collision_detection_bullet/bullet_integration/ros_bullet_utils.h>  // urdfPose2Eigen
#include <moveit_msgs/CollisionObject.h>
#include <robot_state_publisher/robot_state_publisher.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <tf2_eigen/tf2_eigen.h>  // eigenToTransform
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_kdl/tf2_kdl.h>
#include <tf2_ros/buffer.h>
#include <urdf_model/link.h>
#include <xmlrpcpp/XmlRpcValue.h>

#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/shape_extents.h>

#include <kdl/tree.hpp>
#include <urdf/model.h>

#include <map>

#include <utility>

namespace planning_scene
{
/* Auxiliar function, used to compute the direct kinematics
 * Taken from
 * https://github.com/ros/robot_state_publisher/blob/ff29b5cdd0c8c1d69c4203de811120ec593e0b2f/src/robot_state_publisher.cpp#L200
 * */
void addChildren(const KDL::SegmentMap::const_iterator _segment,
                 std::map<std::string, robot_state_publisher::SegmentPair>& _segments_fixed);

/* Auxiliar function, used to fix names */
std::string stripSlash(const std::string& in)
{
  if (not in.empty() && in[0] == '/')
  {
    return in.substr(1);
  }
  return in;
}

moveit_msgs::CollisionObject urdf_to_collision_object(const urdf::Model& _urdf_model)
{
  KDL::Tree kdl_tree;
  moveit_msgs::CollisionObject result;

  moveit_msgs::CollisionObject result_empty;  // empty collision object should be const
  // ----------------------------------
  // -------  Part I ----------
  //  Get the links from the urdf model and extract its geometry.
  //  output: the following maps with names and geometries
  //    - solid_primitives_map
  //    - mesh_map
  // ----------------------------------
  //
  std::map<std::string, std::pair<shape_msgs::SolidPrimitive, Eigen::Isometry3d>> solid_primitives_map;
  std::map<std::string, std::pair<shape_msgs::Mesh, Eigen::Isometry3d>> mesh_map;

  kdl_parser::treeFromUrdfModel(_urdf_model, kdl_tree);
  if (kdl_tree.getSegments().empty())
  {
    ROS_ERROR("The urdf is empty");
    return result_empty;
  }
  std::vector<urdf::LinkSharedPtr> links_in_object;

  const std::string& root_link_name = GetTreeElementSegment(kdl_tree.getRootSegment()->second).getName();

  _urdf_model.getLinks(links_in_object);

  std::vector<urdf::LinkSharedPtr>::iterator it =
      find_if(links_in_object.begin(), links_in_object.end(),
              [&root_link_name](const urdf::LinkSharedPtr& _link) { return _link->name == root_link_name; });

  if (it == links_in_object.end())
  {
    ROS_ERROR("could not find root link");
    return result_empty;
  }

  // Place the root object at the beginnig of the array
  urdf::LinkSharedPtr root_link = *it;
  links_in_object.erase(it);
  links_in_object.insert(links_in_object.begin(), root_link);

  shape_msgs::SolidPrimitive solid_primitive;

  for (const urdf::LinkSharedPtr& link : links_in_object)
  {
    urdf::GeometrySharedPtr geometry;
    urdf::Pose pose;

    // Give priority to the collision object description
    if (link->collision)
    {
      geometry = link->collision->geometry;
      pose = link->collision->origin;
    }
    else if (link->visual)
    {
      geometry = link->visual->geometry;
      pose = link->visual->origin;
    }
    else
    {
      ROS_WARN("Link ill defined: Link %s has not visual or collision tag", link->name.c_str());
    }

    if (link->collision or link->visual)
    {
      switch (geometry->type)
      {
        case urdf::Geometry::BOX:
        {
          solid_primitive.type = solid_primitive.BOX;
          // downcast
          const urdf::BoxConstSharedPtr box = std::dynamic_pointer_cast<const urdf::Box>(geometry);
          // set dimensions
          if (box)
          {
            solid_primitive.dimensions = std::vector<double>{ box->dim.x, box->dim.y, box->dim.z };
            solid_primitives_map[link->name] =
                std::make_pair(solid_primitive, collision_detection_bullet::urdfPose2Eigen(pose));
          }
          else
          {
            ROS_ERROR("Link ill defined: urdf::Geometry could not be casted into urdf::Box");
          }
          break;
        }
        case urdf::Geometry::CYLINDER:
        {
          // set type
          solid_primitive.type = solid_primitive.CYLINDER;
          // downcast
          const urdf::CylinderConstSharedPtr cylinder = std::dynamic_pointer_cast<const urdf::Cylinder>(geometry);
          // set dimensions
          if (cylinder)
          {
            solid_primitive.dimensions = std::vector<double>{ cylinder->length, cylinder->radius };
            solid_primitives_map[link->name] =
                std::make_pair(solid_primitive, collision_detection_bullet::urdfPose2Eigen(pose));
          }
          else
          {
            ROS_ERROR("Link ill defined: urdf::Geometry could not be casted into urdf::Cylinder");
          }
          break;
        }
        case urdf::Geometry::SPHERE:
        {
          // set type
          solid_primitive.type = solid_primitive.SPHERE;
          // downcast
          const urdf::SphereConstSharedPtr sphere = std::dynamic_pointer_cast<const urdf::Sphere>(geometry);
          if (sphere)
          {
            // set dimensions
            solid_primitive.dimensions = std::vector<double>{ sphere->radius };
            solid_primitives_map[link->name] =
                std::make_pair(solid_primitive, collision_detection_bullet::urdfPose2Eigen(pose));
          }
          else
          {
            ROS_ERROR("Link ill defined: urdf::Geometry could not be casted into urdf::Sphere");
          }
          break;
        }
        case urdf::Geometry::MESH:
        {
          const urdf::MeshConstSharedPtr mesh = std::dynamic_pointer_cast<const urdf::Mesh>(geometry);
          if (mesh)
          {
            const Eigen::Vector3d scaling = Eigen::Vector3d(mesh->scale.x, mesh->scale.y, mesh->scale.z);
            // build a shapes::Shape from the file in the URDF
            std::unique_ptr<const shapes::Shape> shape_primitive =
                std::unique_ptr<const shapes::Shape>(shapes::createMeshFromResource(mesh->filename, scaling));
            // build a shapes::ShapeMsg from the shapes:Shape
            shapes::ShapeMsg shape_primitive_msg;
            shapes::constructMsgFromShape(shape_primitive.get(), shape_primitive_msg);
            try
            {
              // build a shape_msgs::Mesh from a shapes::ShapeMsg
              shape_msgs::Mesh mesh_msg = boost::get<shape_msgs::Mesh>(shape_primitive_msg);
              mesh_map[link->name] = std::make_pair(mesh_msg, collision_detection_bullet::urdfPose2Eigen(pose));
            }
            catch (std::exception const& ex)
            {
              ROS_ERROR("Link ill defined: unable to build the mesh. Exeption:  %s", ex.what());
            }
          }
          else
          {
            ROS_ERROR("Link ill defined: could not cast geometry to mesh");
          }
          break;
        }
        default:
          ROS_ERROR("Link ill defined: geometry type unknown");
      }
    }
  }
  // ----------------------------------
  // -------  Part II ----------
  // Get the relative positions of the shapes
  // ----------------------------------

  // 1. build the array of segmenets
  std::map<std::string, robot_state_publisher::SegmentPair> segments;

  addChildren(kdl_tree.getRootSegment(), segments);

  // 2. for each segment compute the required transform
  tf2_ros::Buffer tf_buffer;  // this steps fills this buffer with transforms
  for (std::map<std::string, robot_state_publisher::SegmentPair>::const_iterator segment_pair = segments.begin();
       segment_pair != segments.end(); segment_pair++)
  {
    geometry_msgs::TransformStamped tf_transform = tf2::kdlToTransform(segment_pair->second.segment.pose(0));
    tf_transform.header.stamp = ros::Time::now();

    tf_transform.header.frame_id = stripSlash(segment_pair->second.root);
    tf_transform.child_frame_id = stripSlash(segment_pair->second.tip);
    // 2.1 push the transform into the tf_buffer
    if (not tf_buffer.setTransform(tf_transform, "default_authority", true))
    {
      ROS_ERROR("Cannot compute the transformation between %s and %s", tf_transform.header.frame_id.c_str(),
                tf_transform.child_frame_id.c_str());
    }
  }
  for (const auto& it : solid_primitives_map)
  {
    if (tf_buffer.canTransform(root_link_name, it.first, ros::Time::now()))
    {
      geometry_msgs::TransformStamped primitive_transform =
          tf_buffer.lookupTransform(root_link_name, it.first, ros::Time::now());

      Eigen::Isometry3d aux_isometry;
      tf2::doTransform(it.second.second, aux_isometry, primitive_transform);
      geometry_msgs::Pose pose;
      pose = tf2::toMsg(aux_isometry);
      result.primitive_poses.push_back(pose);
      result.primitives.push_back(it.second.first);
    }
    else
    {
      ROS_ERROR("cannot find a tranform from %s to %s\n", root_link_name.c_str(), it.first.c_str());
    }
  }
  for (const auto& it : mesh_map)
  {
    if (tf_buffer.canTransform(root_link_name, it.first, ros::Time::now()))
    {
      geometry_msgs::TransformStamped primitive_transform =
          tf_buffer.lookupTransform(root_link_name, it.first, ros::Time::now());

      Eigen::Isometry3d aux_isometry;
      tf2::doTransform(it.second.second, aux_isometry, primitive_transform);
      geometry_msgs::Pose pose;
      pose = tf2::toMsg(aux_isometry);
      result.mesh_poses.push_back(pose);
      result.meshes.push_back(it.second.first);
    }
    else
    {
      ROS_ERROR("cannot find a tranform from %s to %s\n", root_link_name.c_str(), it.first.c_str());
    }
  }
  return result;
}

void addChildren(const KDL::SegmentMap::const_iterator _segment,
                 std::map<std::string, robot_state_publisher::SegmentPair>& _segments_fixed)
{
  const std::string& root = GetTreeElementSegment(_segment->second).getName();

  // get children of the segment
  const std::vector<KDL::SegmentMap::const_iterator>& children = GetTreeElementChildren(_segment->second);

  for (auto& element : children)
  {
    const KDL::Segment& child = GetTreeElementSegment(element->second);
    robot_state_publisher::SegmentPair segment_pair(GetTreeElementSegment(element->second), root, child.getName());
    // const KDL::Joint& old_joint = child.getJoint();
    // KDL::Joint new_joint(old_joint.JointOrigin(), old_joint.JointAxis(), KDL::Joint::None);

    _segments_fixed.insert(make_pair(child.getJoint().getName(), segment_pair));
    addChildren(element, _segments_fixed);
  }
}

}  // namespace planning_scene
