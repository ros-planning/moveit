/**
 * @file basic_types.h
 * @brief The tesseract_core package types.
 *
 * @author Levi Armstrong
 * @date April 15, 2018
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2013, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <geometric_shapes/shapes.h>
#include <unordered_map>
#include <vector>
#include <memory>
#include <functional>
#include <map>

#include <moveit/collision_detection/collision_common.h>
//#include <moveit/planning_scene/planning_scene.h>

namespace trajopt_interface
{
template <typename T>
using AlignedVector = std::vector<T, Eigen::aligned_allocator<T>>;
// aligned_allocator provides at least 16 bytes alignment and more

template <typename Key, typename Value>
using AlignedMap = std::map<Key, Value, std::less<Key>, Eigen::aligned_allocator<std::pair<const Key, Value>>>;

template <typename Key, typename Value>
using AlignedUnorderedMap = std::unordered_map<Key, Value, std::hash<Key>, std::equal_to<Key>,
                                               Eigen::aligned_allocator<std::pair<const Key, Value>>>;

using VectorIsometry3d = AlignedVector<Eigen::Isometry3d>;
using VectorVector4d = AlignedVector<Eigen::Vector4d>;
using VectorVector3d = std::vector<Eigen::Vector3d>;
using TransformMap = AlignedMap<std::string, Eigen::Isometry3d>;

typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> TrajArray;

struct AllowedCollisionMatrix
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  virtual ~AllowedCollisionMatrix() = default;

  /**
   * @brief Disable collision between two collision objects
   * @param obj1 Collision object name
   * @param obj2 Collision object name
   * @param reason The reason for disabling collison
   */
  virtual void addAllowedCollision(const std::string& link_name1, const std::string& link_name2,
                                   const std::string& reason)
  {
    auto link_pair = makeOrderedLinkPair(link_name1, link_name2);
    lookup_table_[link_pair] = reason;
  }

  /**
   * @brief Remove disabled collision pair from allowed collision matrix
   * @param obj1 Collision object name
   * @param obj2 Collision object name
   */
  virtual void removeAllowedCollision(const std::string& link_name1, const std::string& link_name2)
  {
    auto link_pair = makeOrderedLinkPair(link_name1, link_name2);
    lookup_table_.erase(link_pair);
  }

  /**
   * @brief This checks if two links are allowed to be in collision
   * @param link_name1 First link name
   * @param link_name2 Second link anme
   * @return True if allowed to be in collision, otherwise false
   */
  virtual bool isCollisionAllowed(const std::string& link_name1, const std::string& link_name2) const
  {
    auto link_pair = makeOrderedLinkPair(link_name1, link_name2);
    return (lookup_table_.find(link_pair) != lookup_table_.end());
  }

  /**
   * @brief Clears the list of allowed collisions, so that no collision will be
   *        allowed.
   */
  void clearAllowedCollisions()
  {
    lookup_table_.clear();
  }

private:
  typedef std::pair<const std::string, const std::string> LinkNamesPair;
  struct PairHash
  {
    std::size_t operator()(const LinkNamesPair& pair) const
    {
      return std::hash<std::string>()(pair.first + pair.second);
    }
  };
  typedef std::unordered_map<LinkNamesPair, std::string, PairHash> AllowedCollisionEntries;
  AllowedCollisionEntries lookup_table_;

  /**
   * @brief Create a pair of strings, where the pair.first is always <= pair.second
   * @param link_name1 First link name
   * @param link_name2 Second link anme
   * @return LinkNamesPair a lexicographically sorted pair of strings
   */
  static inline LinkNamesPair makeOrderedLinkPair(const std::string& link_name1, const std::string& link_name2)
  {
    if (link_name1 <= link_name2)
      return std::make_pair(link_name1, link_name2);
    else
      return std::make_pair(link_name2, link_name1);
  }

public:
  /**
   * @brief Clears the list of allowed collisions
   * @return AllowedCollisionEntries an unordered map containing all allowed
   *         collision entries. The keys of the unordered map are a std::pair
   *         of the link names in the allowed collision pair.
   */
  const AllowedCollisionEntries& getAllAllowedCollisions() const
  {
    return lookup_table_;
  }
};
typedef std::shared_ptr<AllowedCollisionMatrix> AllowedCollisionMatrixPtr;
typedef std::shared_ptr<const AllowedCollisionMatrix> AllowedCollisionMatrixConstPtr;

/**
 * @brief Should return true if contact allowed, otherwise false.
 *
 * Also the order of strings should not matter, the function should handled by the function.
 */
typedef std::function<bool(const std::string&, const std::string&)> IsContactAllowedFn;

namespace CollisionObjectTypes
{
enum CollisionObjectType
{
  UseShapeType = 0, /**< @brief Infer the type from the type specified in the shapes::Shape class */

  // These convert the meshes to custom collision objects
  ConvexHull = 1,  /**< @brief Use the mesh in shapes::Shape but make it a convex hulls collision object. (if not convex
                      it will  be converted) */
  MultiSphere = 2, /**< @brief Use the mesh and represent it by multiple spheres collision object */
  SDF = 3          /**< @brief Use the mesh and rpresent it by a signed distance fields collision object */
};
}
typedef CollisionObjectTypes::CollisionObjectType CollisionObjectType;
typedef std::vector<CollisionObjectType> CollisionObjectTypeVector;

namespace BodyTypes
{
enum BodyType
{
  ROBOT_LINK = 0,    /**< @brief These are links at the creation of the environment */
  ROBOT_ATTACHED = 1 /**< @brief These are links that are added after initial creation */
};
}
typedef BodyTypes::BodyType BodyType;

namespace ContinouseCollisionTypes
{
enum ContinouseCollisionType
{
  CCType_None,
  CCType_Time0,
  CCType_Time1,
  CCType_Between
};
}
typedef ContinouseCollisionTypes::ContinouseCollisionType ContinouseCollisionType;

namespace ContactTestTypes
{
enum ContactTestType
{
  FIRST = 0,   /**< Return at first contact for any pair of objects */
  CLOSEST = 1, /**< Return the global minimum for a pair of objects */
  ALL = 2,     /**< Return all contacts for a pair of objects */
  LIMITED = 3  /**< Return limited set of contacts for a pair of objects */
};
}
typedef ContactTestTypes::ContactTestType ContactTestType;

struct ContactResult
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  double distance;
  int type_id[2];
  std::string link_names[2];
  Eigen::Vector3d nearest_points[2];  // this is an array of two elements of type Vector3d
  Eigen::Vector3d normal;

  Eigen::Vector3d cc_nearest_points[2];
  // this is used when performing continous collision checking. so it should be the nearest point on the
  // swept volume of a robot's link and an obstacle (obstacle coul be fixed in space or moving. in the case
  // of moving, we should calculate the swpet volume of the obstacle too, I guess?)

  double cc_time;
  ContinouseCollisionType cc_type;

  ContactResult()
  {
    clear();
  }
  /// Clear structure data
  void clear()
  {
    distance = std::numeric_limits<double>::max();
    nearest_points[0].setZero();
    nearest_points[1].setZero();
    link_names[0] = "";
    link_names[1] = "";
    type_id[0] = 0;
    type_id[1] = 0;
    normal.setZero();
    cc_nearest_points[0].setZero();
    cc_nearest_points[1].setZero();
    cc_time = -1;
    cc_type = ContinouseCollisionType::CCType_None;
  }

  // convert collision_detection::Contact from MoveIt to trajopt::ContactResult (from tesseract)
  ContactResult(collision_detection::Contact moveit_contact, Eigen::Vector3d moveit_cc_nearest_points[2],
                double moveit_cc_time, ContinouseCollisionType moveit_cc_type)
  {
    distance = moveit_contact.depth;
    nearest_points[0] = moveit_contact.nearest_points[0];
    nearest_points[1] = moveit_contact.nearest_points[1];
    link_names[0] = moveit_contact.body_name_1;
    link_names[1] = moveit_contact.body_name_2;
    // in tesseaerct_message: type_id # ROBOT_LINK = 0, ROBOT_ATTACHED = 1
    type_id[0] = (moveit_contact.body_type_1 == collision_detection::BodyTypes::Type::ROBOT_LINK) ? 0 : 1;
    type_id[1] = (moveit_contact.body_type_2 == collision_detection::BodyTypes::Type::ROBOT_LINK) ? 0 : 1;
    normal = moveit_contact.normal;
    // to find out the correct information for the following, I probably need to look at
    // tesseract_collision/bullet_utils.h
    cc_nearest_points[0] = moveit_cc_nearest_points[0];
    cc_nearest_points[1] = moveit_cc_nearest_points[1];
    cc_time = moveit_cc_time;
    cc_type = moveit_cc_type;
  }
};

// ContactResultVector: is an aligned vector which works with memory. It is a vector containing ContactResult
typedef AlignedVector<ContactResult> ContactResultVector;
typedef AlignedMap<std::pair<std::string, std::string>, ContactResultVector> ContactResultMap;

/// Contact test data and query results information
struct ContactTestData
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ContactTestData(const std::vector<std::string>& active, const double& contact_distance, const IsContactAllowedFn& fn,
                  const ContactTestType& type, ContactResultMap& res)
    : active(active), contact_distance(contact_distance), fn(fn), type(type), res(res), done(false)
  {
  }

  const std::vector<std::string>& active;
  const double& contact_distance;
  const IsContactAllowedFn& fn;
  const ContactTestType& type;

  /// Destance query results information
  ContactResultMap& res;

  /// Indicate if search is finished
  bool done;
};

static inline void moveContactResultsMapToContactResultsVector(ContactResultMap& contact_map,
                                                               ContactResultVector& contact_vector)
{
  std::size_t size = 0;
  for (const auto& contact : contact_map)
    size += contact.second.size();

  contact_vector.reserve(size);
  for (auto& contact : contact_map)
    std::move(contact.second.begin(), contact.second.end(), std::back_inserter(contact_vector));
}

/** @brief This holds a state of the environment */
struct EnvState
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  std::unordered_map<std::string, double> joints;
  TransformMap transforms;
};
typedef std::shared_ptr<EnvState> EnvStatePtr;
typedef std::shared_ptr<const EnvState> EnvStateConstPtr;

/**< @brief Information on how the object is attached to the environment */
struct AttachedBodyInfo
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  AttachedBodyInfo() : transform(Eigen::Isometry3d::Identity())
  {
  }
  std::string object_name;              /**< @brief The name of the AttachableObject being used */
  std::string parent_link_name;         /**< @brief The name of the link to attach the body */
  Eigen::Isometry3d transform;          /**< @brief The transform between parent link and object */
  std::vector<std::string> touch_links; /**< @brief The names of links which the attached body is allowed to be in
                                           contact with */
};

/** @brief Contains visual geometry data */
struct VisualObjectGeometry
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  std::vector<shapes::ShapeConstPtr> shapes; /**< @brief The shape */
  VectorIsometry3d shape_poses;              /**< @brief The pose of the shape */
  VectorVector4d shape_colors;               /**< @brief (Optional) The shape color (R, G, B, A) */
};

/** @brief Contains visual geometry data */
struct CollisionObjectGeometry : public VisualObjectGeometry
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  CollisionObjectTypeVector
      collision_object_types; /**< @brief The collision object type. This is used by the collision libraries */
};

/** @brief Contains data about an attachable object */
struct AttachableObject
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  std::string name;            /**< @brief The name of the attachable object (aka. link name and must be unique) */
  VisualObjectGeometry visual; /**< @brief The objects visual geometry */
  CollisionObjectGeometry collision; /**< @brief The objects collision geometry */
};
typedef std::shared_ptr<AttachableObject> AttachableObjectPtr;
typedef std::shared_ptr<const AttachableObject> AttachableObjectConstPtr;

/** @brief ObjectColorMap Stores Object color in a 4d vector as RGBA*/
struct ObjectColor
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  VectorVector4d visual;
  VectorVector4d collision;
};
typedef AlignedUnorderedMap<std::string, ObjectColor> ObjectColorMap;
typedef std::shared_ptr<ObjectColorMap> ObjectColorMapPtr;
typedef std::shared_ptr<const ObjectColorMap> ObjectColorMapConstPtr;
typedef AlignedUnorderedMap<std::string, AttachedBodyInfo> AttachedBodyInfoMap;
typedef std::unordered_map<std::string, AttachableObjectConstPtr> AttachableObjectConstPtrMap;
}  // namespace trajopt_interface
