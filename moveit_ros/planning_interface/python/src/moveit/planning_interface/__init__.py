import moveit.core
from .robot import *
from .planning_scene import *
from . import move_group
from warnings import warn


class MoveGroupInterface(move_group.MoveGroupInterface):
    def place_poses_list(*args, **kwargs):
        warn("Use place([PoseStamped()])", DeprecationWarning)
        return move_group.MoveGroupInterface.place(*args, **kwargs)

    def place_locations_list(*args, **kwargs):
        warn("Use place([PlaceLocation()])", DeprecationWarning)
        return move_group.MoveGroupInterface.place(*args, **kwargs)
