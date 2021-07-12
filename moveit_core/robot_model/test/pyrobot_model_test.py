#!/usr/bin/env python
import pytest
import rospkg
import rospy

from moveit.core import load_robot_model
from moveit.core.robot_model import (
    RobotModel,
    RevoluteJointModel,
    FloatingJointModel,
    PlanarJointModel,
    FixedJointModel,
    LinkModel,
)


class TestPyRobotModel:
    def test_all(self):
        # Load the panda model
        p = rospkg.RosPack()
        urdf_path = (
            p.get_path("moveit_resources_panda_description") + "/urdf/panda.urdf"
        )
        srdf_path = (
            p.get_path("moveit_resources_panda_moveit_config") + "/config/panda.srdf"
        )
        model = load_robot_model(urdf_path, srdf_path)

        # Test every exposed method
        bounds = model.getActiveJointModelsBounds()
        print(bounds[0][0].min_position_)
        model.getMaximumExtent()
        _ = model.getMaximumExtent()
        model.getJointModel("panda_joint1")
        model.getModelFrame()
        model.getName()
        model.getVariableBounds("panda_joint1")
        model.getVariableCount()
        model.getVariableDefaultPositions({"panda_joint1": 0.0})
        model.getVariableDefaultPositions([0.0])
        model.getVariableIndex("panda_joint1")
        model.getVariableNames()
        model.getRootJointName()
        link = model.getRootLink()

        joint = model.getRootJoint()
        joint.getChildLinkModel()
        joint.getDescendantJointModels()
        joint.getDescendantLinkModels()
        joint.getDistanceFactor()
        joint.getMaximumExtent()
        joint.getMimic()
        joint.getMimicFactor()
        joint.getMimicOffset()
        joint.getMimicRequests()
        joint.getNonFixedDescendantJointModels()
        joint.getParentLinkModel()
        joint.getStateSpaceDimension()
        joint.getTypeName()
        joint.isPassive()
        # lifetime of these 'temporaries' is controlled in C++
        joint.setChildLinkModel(LinkModel("fake_child_link"))
        joint.setDistanceFactor(1)
        joint.setMimic(RevoluteJointModel("fake_mimic_joint"), 0, 0)
        joint.setParentLinkModel(LinkModel("fake_parent_link"))
        joint.setPassive(True)
        # do this at the end so it doesn't mess with the above "get" calls
        joint.addDescendantJointModel(FixedJointModel("fake_descendent_joint"))
        joint.addDescendantLinkModel(LinkModel("fake_descendent_link"))

        print(joint)

        model.getJointModelNames()
        model.getJointOfVariable("virtual_joint")
        model.getJointOfVariable(0)
        model.getJointModelCount()
        model.hasJointModelGroup("panda_arm")
        print(model)

        g = model.getJointModelGroup("panda_arm")
        g.canSetStateFromIK("panda_link8")
        g.getActiveJointModelNames()
        g.getActiveJointModels()
        g.getAttachedEndEffectorNames()
        g.getCommonRoot()
        g.getContinuousJointModels()
        g.getDefaultIKTimeout()
        g.getDefaultStateNames()
        g.getEndEffectorName()
        g.getEndEffectorParentGroup()
        link = g.getLinkModel("panda_link1")
        g.getEndEffectorTips([link])
        g.getEndEffectorTips(["panda_link8"])
        g.getFixedJointModels()
        g.getJointModel("panda_joint1")
        g.getJointModelNames()
        g.getJointModels()
        g.getJointRoots()
        g.getLinkModelNames()
        g.getLinkModelNamesWithCollisionGeometry()
        g.getLinkModels()
        g.getMaximumExtent()
        g.getMaximumExtent()
        g.getMimicJointModels()
        g.getName()
        g.getOnlyOneEndEffectorTip()
        g.getParentModel()
        g.getSubgroupNames()
        g.getSubgroups([g])
        g.getUpdatedLinkModelNames()
        g.getUpdatedLinkModels()
        g.getUpdatedLinkModelsSet()
        g.getUpdatedLinkModelsWithGeometry()
        g.getUpdatedLinkModelsWithGeometryNames()
        g.getUpdatedLinkModelsWithGeometryNamesSet()
        g.getUpdatedLinkModelsWithGeometrySet()
        g.getVariableCount()
        g.getVariableGroupIndex("panda_joint1")
        g.getVariableIndexList()
        g.getVariableNames()
        g.hasJointModel("non_existent_joint")
        g.hasLinkModel("non_existent_link")
        g.isChain()
        g.isContiguousWithinState()
        g.isEndEffector()
        g.isLinkUpdated("panda_link1")
        g.isSingleDOFJoints()
        g.isSubgroup("panda_arm")
        g.printGroupInfo()
        g.setDefaultIKTimeout(1)
        g.setEndEffectorName("new_ee")
        g.setEndEffectorParent("panda_arm", "panda_link8")
        g.setRedundantJoints([])
        g.setSubgroupNames([])
        print(g)

        link.getName()
        link.areCollisionOriginTransformsIdentity()
        link.getCenteredBoundingBoxOffset()
        link.getCollisionOriginTransforms()
        link.getFirstCollisionBodyTransformIndex()
        link.getJointOriginTransform()
        link.getLinkIndex()
        link.getShapeExtentsAtOrigin()
        link.getVisualMeshFilename()
        link.getVisualMeshOrigin()
        link.getVisualMeshScale()
        link.jointOriginTransformIsIdentity()
        link.parentJointIsFixed()

        assert True


@pytest.fixture
def rospy_init_fixture():
    rospy.init_node("test_pyrobot_model")
    return TestPyRobotModel()


if __name__ == "__main__":
    pytest.main()
