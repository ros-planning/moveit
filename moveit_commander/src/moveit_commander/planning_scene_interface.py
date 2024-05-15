# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Ioan Sucan, Felix Messmer

import rospy
from rosgraph.names import ns_join

from moveit_msgs.msg import PlanningScene, CollisionObject, AttachedCollisionObject
from moveit_msgs.msg import AllowedCollisionMatrix, AllowedCollisionEntry
from moveit.planning_interface import PlanningSceneInterface as _PlanningSceneInterface
from geometry_msgs.msg import Point
from shape_msgs.msg import SolidPrimitive, Plane, Mesh, MeshTriangle
from .exception import MoveItCommanderException

try:
    from pyassimp import pyassimp
except:
    # support pyassimp > 3.0
    try:
        import pyassimp
    except:
        pyassimp = False
        print(
            "Failed to import pyassimp, see https://github.com/ros-planning/moveit/issues/86 for more info"
        )


class PlanningSceneInterface(object):
    """
    Python interface for a C++ PlanningSceneInterface.
    Uses both C++ wrapped methods and scene manipulation topics
    to manipulate the PlanningScene managed by the PlanningSceneMonitor.
    See wrap_python_planning_scene_interface.cpp for the wrapped methods.
    """

    def __init__(self, ns="", synchronous=True):
        self._psi = _PlanningSceneInterface(ns)
        self.__synchronous = synchronous

        if not self.__synchronous:
            self._pub_co = rospy.Publisher(
                ns_join(ns, "collision_object"), CollisionObject, queue_size=100
            )
            self._pub_aco = rospy.Publisher(
                ns_join(ns, "attached_collision_object"),
                AttachedCollisionObject,
                queue_size=100,
            )

    def __submit(self, collision_object, attach=False):
        if self.__synchronous:
            scene = PlanningScene()
            scene.is_diff = True
            scene.robot_state.is_diff = True
            if attach:
                scene.robot_state.attached_collision_objects = [collision_object]
            else:
                scene.world.collision_objects = [collision_object]
            self._psi.apply_planning_scene(scene)
        else:
            if attach:
                self._pub_aco.publish(collision_object)
            else:
                self._pub_co.publish(collision_object)

    def add_object(self, collision_object):
        """Add an object to the planning scene"""
        self.__submit(collision_object, attach=False)

    def add_sphere(self, name, pose, radius=1):
        """Add a sphere to the planning scene"""
        co = self.make_sphere(name, pose, radius)
        self.__submit(co, attach=False)

    def add_cylinder(self, name, pose, height, radius):
        """Add a cylinder to the planning scene"""
        co = self.make_cylinder(name, pose, height, radius)
        self.__submit(co, attach=False)

    def add_cone(self, name, pose, height, radius):
        """Add a cylinder to the planning scene"""
        co = self.make_cone(name, pose, height, radius)
        self.__submit(co, attach=False)

    def add_mesh(self, name, pose, filename, size=(1, 1, 1)):
        """Add a mesh to the planning scene"""
        co = self.make_mesh(name, pose, filename, size)
        self.__submit(co, attach=False)

    def add_box(self, name, pose, size=(1, 1, 1)):
        """Add a box to the planning scene"""
        co = self.make_box(name, pose, size)
        self.__submit(co, attach=False)

    def add_plane(self, name, pose, normal=(0, 0, 1), offset=0):
        """Add a plane to the planning scene"""
        co = self.make_plane(name, pose, normal, offset)
        self.__submit(co, attach=False)

    def attach_object(self, object, link=None, touch_links=None):
        """Attach an object to the given link"""
        if isinstance(object, str):
            object = self.__make_existing(object)
        if isinstance(object, CollisionObject):
            object = AttachedCollisionObject(object=object)

        if link is not None:
            object.link_name = link
        object.touch_links = (
            touch_links if touch_links is not None else [object.link_name]
        )

        self.__submit(object, attach=True)

    def attach_mesh(
        self, link, name, pose=None, filename="", size=(1, 1, 1), touch_links=None
    ):
        """Create mesh and attach it to the given link"""
        if (pose is not None) and filename:
            co = self.make_mesh(name, pose, filename, size)
        else:
            co = self.__make_existing(name)
        self.attach_object(co, link, touch_links)

    def attach_box(self, link, name, pose=None, size=(1, 1, 1), touch_links=None):
        """Create box and attach it to the given link"""
        if pose is not None:
            co = self.make_box(name, pose, size)
        else:
            co = self.__make_existing(name)
        self.attach_object(co, link, touch_links)

    def attach_sphere(self, link, name, pose=None, radius=1, touch_links=None):
        """Create sphere and attach it to the given link"""
        if pose is not None:
            co = self.make_sphere(name, pose, radius)
        else:
            co = self.__make_existing(name)
        self.attach_object(co, link, touch_links)

    def attach_cylinder(
        self, link, name, pose=None, height=1, radius=1, touch_links=None
    ):
        """Create cylinder and attach it to the given link"""
        if pose is not None:
            co = self.make_cylinder(name, pose, height, radius)
        else:
            co = self.__make_existing(name)
        self.attach_object(co, link, touch_links)

    def clear(self):
        """Remove all objects from the planning scene"""
        self._psi.clear()

    def remove_world_object(self, name=None):
        """
        Remove an object from planning scene, or all if no name is provided
        """
        co = CollisionObject()
        co.operation = CollisionObject.REMOVE
        if name is not None:
            co.id = name
        self.__submit(co, attach=False)

    def remove_attached_object(self, link=None, name=None):
        """
        Remove an attached object from the robot, or all objects attached to the link if no name is provided,
        or all attached objects in the scene if neither link nor name are provided.

        Removed attached objects remain in the scene as world objects.
        Call remove_world_object afterwards to remove them from the scene.
        """
        aco = AttachedCollisionObject()
        aco.object.operation = CollisionObject.REMOVE
        if link is not None:
            aco.link_name = link
        if name is not None:
            aco.object.id = name
        self.__submit(aco, attach=True)

    def get_known_object_names(self, with_type=False):
        """
        Get the names of all known objects in the world. If with_type is set to true, only return objects that have a known type.
        """
        return self._psi.get_known_object_names(with_type)

    def get_known_object_names_in_roi(
        self, minx, miny, minz, maxx, maxy, maxz, with_type=False
    ):
        """
        Get the names of known objects in the world that are located within a bounding region (specified in the frame reported by
        get_planning_frame()). If with_type is set to true, only return objects that have a known type.
        """
        return self._psi.get_known_object_names_in_roi(
            minx, miny, minz, maxx, maxy, maxz, with_type
        )

    def get_object_poses(self, object_ids):
        """
        Get the poses from the objects identified by the given object ids list.
        """
        return self._psi.get_object_poses(object_ids)

    def get_objects(self, object_ids=[]):
        """
        Get the objects identified by the given object ids list. If no ids are provided, return all the known objects.
        """
        return self._psi.get_objects(object_ids)

    def get_attached_objects(self, object_ids=[]):
        """
        Get the attached objects identified by the given object ids list. If no ids are provided, return all the attached objects.
        """
        return self._psi.get_attached_objects(object_ids)

    def get_planning_scene(self, components):
        """Get move_group's current planning scene"""
        return self._psi.get_planning_scene(components)

    def apply_planning_scene(self, planning_scene_message):
        """
        Applies the planning scene message.
        """
        return self._psi.apply_planning_scene(planning_scene_message)

    @staticmethod
    def __make_existing(name):
        """
        Create an empty Collision Object. Used when the object already exists
        """
        co = CollisionObject()
        co.id = name
        return co

    @staticmethod
    def __make_primitive(name, pose, type, shape_args):
        co = CollisionObject()
        co.operation = CollisionObject.ADD
        co.id = name
        co.header = pose.header
        co.pose = pose.pose
        shape = SolidPrimitive()
        shape.type = type
        shape.dimensions = list(shape_args)
        co.primitives = [shape]
        return co

    @staticmethod
    def make_box(name, pose, size):
        return PlanningSceneInterface.__make_primitive(
            name, pose, SolidPrimitive.BOX, size
        )

    @staticmethod
    def make_mesh(name, pose, filename, scale=(1, 1, 1)):
        co = CollisionObject()
        if pyassimp is False:
            raise MoveItCommanderException(
                "Pyassimp needs patch https://launchpadlibrarian.net/319496602/patchPyassim.txt"
            )
        scene = pyassimp.load(filename)
        if not scene.meshes or len(scene.meshes) == 0:
            raise MoveItCommanderException("There are no meshes in the file")
        if len(scene.meshes[0].faces) == 0:
            raise MoveItCommanderException("There are no faces in the mesh")
        co.operation = CollisionObject.ADD
        co.id = name
        co.header = pose.header
        co.pose = pose.pose

        mesh = Mesh()
        first_face = scene.meshes[0].faces[0]
        if hasattr(first_face, "__len__"):
            for face in scene.meshes[0].faces:
                if len(face) == 3:
                    triangle = MeshTriangle()
                    triangle.vertex_indices = [face[0], face[1], face[2]]
                    mesh.triangles.append(triangle)
        elif hasattr(first_face, "indices"):
            for face in scene.meshes[0].faces:
                if len(face.indices) == 3:
                    triangle = MeshTriangle()
                    triangle.vertex_indices = [
                        face.indices[0],
                        face.indices[1],
                        face.indices[2],
                    ]
                    mesh.triangles.append(triangle)
        else:
            raise MoveItCommanderException(
                "Unable to build triangles from mesh due to mesh object structure"
            )
        for vertex in scene.meshes[0].vertices:
            point = Point()
            point.x = vertex[0] * scale[0]
            point.y = vertex[1] * scale[1]
            point.z = vertex[2] * scale[2]
            mesh.vertices.append(point)
        co.meshes = [mesh]
        pyassimp.release(scene)
        return co

    @staticmethod
    def make_sphere(name, pose, radius):
        return PlanningSceneInterface.__make_primitive(
            name, pose, SolidPrimitive.SPHERE, [radius]
        )

    @staticmethod
    def make_cylinder(name, pose, height, radius):
        return PlanningSceneInterface.__make_primitive(
            name, pose, SolidPrimitive.CYLINDER, [height, radius]
        )

    @staticmethod
    def make_cone(name, pose, height, radius):
        return PlanningSceneInterface.__make_primitive(
            name, pose, SolidPrimitive.CONE, [height, radius]
        )

    def make_plane(self, name, pose, normal=(0, 0, 1), offset=0):
        co = CollisionObject()
        co.operation = CollisionObject.ADD
        co.id = name
        co.header = pose.header
        p = Plane()
        p.coef = list(normal)
        p.coef.append(offset)
        co.planes = [p]
        co.plane_poses = [pose.pose]
        return co


def acm_set_default(acm, obj, allow):
    """Set default collision behavior for obj"""
    if obj not in acm.default_entry_names:
        acm.default_entry_names.append(obj)
        acm.default_entry_values.append(allow)
    else:
        idx = acm.default_entry_names.index(obj)
        acm.default_entry_values[idx] = allow


def acm_set_allowed(acm, obj, other=None, allow=True):
    """Allow collisions between obj and other"""
    if other is None:
        acm.set_default(obj, allow)
        return

    other_idx = acm.entry_names.index(other)
    if obj not in acm.entry_names:
        acm.entry_names.append(obj)
        for entry in acm.entry_values:
            entry.enabled.append(allow)
        acm.entry_values.append(
            AllowedCollisionEntry(enabled=[allow for i in range(len(acm.entry_names))])
        )
        acm.entry_values[-1].enabled[other_idx] = allow
    else:
        obj_idx = acm.entry_names.index(obj)
        acm.entry_values[obj_idx].enabled[other_idx] = allow
        acm.entry_values[other_idx].enabled[obj_idx] = allow


AllowedCollisionMatrix.set_default = acm_set_default
AllowedCollisionMatrix.set_allowed = acm_set_allowed
