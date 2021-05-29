import numpy as np
from scipy.spatial.transform import Rotation

class Pose:

    def __init__(self, frame="", position=np.array([0,0,0],dtype=np.float64), rotation=Rotation()):
        self.position = position
        self.rotation = rotation
        self.frame = frame
    
    def transformTo(self, destinationFrame):
        ...

    def distance(self, pose, d2=False):
        if d2:
            return np.linalg.norm(self.position[:2]-pose.position[:2])
        else:
            return np.linalg.norm(self.position-pose.position)

class ObjectType:
    def __init__(self, name):
        self._name = name
        self.filters = []
        self.groupers = []


    @property
    def name(self):
        return self._name
    
    def addFilter(self, filter):
        self.filters.append(filter)

    def addGrouper(self, grouper):
        self.groupers.append(grouper)


class Object:
    def __init__(self, pose, obj_type, fixed=False):
        self._pose = pose
        self._type = obj_type
        self._fixed = fixed
    
