#pragma once

#include <trajopt_interface/cache.hxx>
#include <trajopt/common.hpp>
#include <trajopt_sco/modeling.hpp>
#include <trajopt_interface/basic_types.h>
#include <trajopt_utils/utils.hpp>

#include <moveit/robot_model/robot_model.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/collision_detection/collision_common.h>
#include <moveit/collision_detection/world.h>

namespace trajopt_interface
{
// template <typename T>
// using AlignedVector = std::vector<T, Eigen::aligned_allocator<T>>;

using trajopt::DblVec;

struct CollisionEvaluator
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  CollisionEvaluator(planning_scene::PlanningSceneConstPtr& planning_scene, std::string planning_group,
                     util::SafetyMarginData::ConstPtr safety_margin_data)
    : safety_margin_data_(safety_margin_data), planning_group_(planning_group)
  {
    // we need a child of planning scene so I can do the changing robot states and do collsiiong detenction
    planning_scene_ = planning_scene->diff();
  }
  virtual ~CollisionEvaluator() = default;
  virtual void CalcDistExpressions(const DblVec& x, sco::AffExprVector& exprs) = 0;
  virtual void CalcDists(const DblVec& x, DblVec& exprs) = 0;
  // calculate the collision in planning scene
  virtual void CalcCollisions(const DblVec& x, trajopt_interface::ContactResultVector& dist_results) = 0;
  void GetCollisionsCached(const DblVec& x, trajopt_interface::ContactResultVector& dist_results);
  // virtual void Plot(const tesseract::BasicPlottingPtr plotter, const DblVec& x) = 0;
  virtual sco::VarVector GetVars() = 0;

  const util::SafetyMarginData::ConstPtr getSafetyMarginData() const
  {
    return safety_margin_data_;
  }

  // CaclCollisions() : calculates the collisions in the scene
  // CollisionsToDistance(): converts collisions contacts in the scenes to distances
  // CollisionToDistanceExpressions(): is where acttual linearized math expressions is created
  // CalcDistExpressions():

  // CalcDists():
  // Value(): calls CalcDists()

  Cache<size_t, ContactResultVector, 10> m_cache;
  // this calss is not dependent to any  tesseract stuff, I just need to figure out ContactResulVector that is passed to it
  // I do not understand what it is doing exactly though
  // it is creating a buffer of ContactResultVector
  // Cache<size_t, std::vector<collision_detection::Contact>, 10> m_cache;

  // what is ContactResultVector?
  // is an aligned vector which works with memory. Basically, it is a vector containing ContactResult
  // now what is ContactResult? it has all the informatin related to contact between two bodies

protected:
  // from MoveIt macros:
  // typedef std::shared_ptr<const PlanningScene> PlanningSceneConstPtr;
  // the object the pointer is pointing to is const, i.e. the planning scene is constant
  // Also, PlanningScen is noncopyable
  // setActiveCollisionDetector is a non-const member of PlanningScene which is not accisible from
  // a PlanningSceneConstPtr. So I have to deal with the const ptr of planning scene here and
  // let the user set the collision detector to bullet

  planning_scene::PlanningScenePtr planning_scene_;
  std::string planning_group_;
  util::SafetyMarginData::ConstPtr safety_margin_data_;

private:
  CollisionEvaluator()
  {
  }
};

typedef std::shared_ptr<CollisionEvaluator> CollisionEvaluatorPtr;

struct SingleTimestepCollisionEvaluator : public CollisionEvaluator
{
public:
  SingleTimestepCollisionEvaluator(planning_scene::PlanningSceneConstPtr& planning_scene, std::string planning_group,
                                   util::SafetyMarginData::ConstPtr safety_margin_data, const sco::VarVector& vars);
  /**
  @brief linearize all contact distances in terms of robot dofs;
  Do a collision check between robot and environment.
  For each contact generated, return a linearization of the signed distance
  function.
  */
  void CalcDistExpressions(const DblVec& x, sco::AffExprVector& exprs) override;
  /**
   * Same as CalcDistExpressions, but just the distances--not the expressions
   */
  void CalcDists(const DblVec& x, DblVec& exprs) override;
  void CalcCollisions(const DblVec& x, ContactResultVector& dist_results) override;
  sco::VarVector GetVars() override
  {
    return m_vars;
  }

private:
  sco::VarVector m_vars;
};

struct CastCollisionEvaluator : public CollisionEvaluator
{
public:
  CastCollisionEvaluator(planning_scene::PlanningSceneConstPtr planning_scene, std::string planning_group,
                         util::SafetyMarginData::ConstPtr safety_margin_data, const sco::VarVector& vars0,
                         const sco::VarVector& vars1);
  void CalcDistExpressions(const DblVec& x, sco::AffExprVector& exprs) override;
  void CalcDists(const DblVec& x, DblVec& exprs) override;
  void CalcCollisions(const DblVec& x, ContactResultVector& dist_results) override;
  sco::VarVector GetVars() override
  {
    return util::concat(m_vars0, m_vars1);
  }

private:
  // for castcollision, swpt volume, we need two states
  sco::VarVector m_vars0;  // contains joint values for state 0
  sco::VarVector m_vars1;  // contains joint values for state 1
};

// sco::Cost does not depend on tesseract. So whatever dependency is there should be here
class TRAJOPT_API CollisionCost : public sco::Cost
{
public:
  /* constructor for single timestep.
     This constructor initializes m_calc which is type of CollisionEvaluator */
  CollisionCost(planning_scene::PlanningSceneConstPtr& planning_scene, std::string planning_group,
                util::SafetyMarginData::ConstPtr safety_margin_data, const sco::VarVector& vars);
  /* constructor for cast cost */
  CollisionCost(planning_scene::PlanningSceneConstPtr& planning_scene, std::string planning_group,
                util::SafetyMarginData::ConstPtr safety_margin_data, const sco::VarVector& vars0,
                const sco::VarVector& vars1);
  virtual sco::ConvexObjective::Ptr convex(const DblVec& x, sco::Model* model) override;
  virtual double value(const DblVec&) override;
  sco::VarVector getVars() override
  {
    return m_calc->GetVars();
  }

private:
  CollisionEvaluatorPtr m_calc;
};

class TRAJOPT_API CollisionConstraint : public sco::IneqConstraint
{
public:
  /* constructor for single timestep */
  CollisionConstraint(planning_scene::PlanningSceneConstPtr planning_scene, std::string planning_group,
                      util::SafetyMarginData::ConstPtr safety_margin_data, const sco::VarVector& vars);
  /* constructor for cast cost */
  CollisionConstraint(planning_scene::PlanningSceneConstPtr planning_scene, std::string planning_group,
                      util::SafetyMarginData::ConstPtr safety_margin_data, const sco::VarVector& vars0,
                      const sco::VarVector& vars1);
  virtual sco::ConvexConstraints::Ptr convex(const DblVec& x, sco::Model* model) override;
  virtual DblVec value(const DblVec&) override;
  void Plot(const DblVec& x);
  sco::VarVector getVars() override
  {
    return m_calc->GetVars();
  }

private:
  CollisionEvaluatorPtr m_calc;
};
}  // namespace trajopt_interface
