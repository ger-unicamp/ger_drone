import numpy as np
from abc import ABC, abstractmethod

class Grouper(ABC):

    @abstractmethod
    def group(self, objects):
        ...

class DistanceGrouper(Grouper):

    def __init__(self, max_distance, n_max_object=0):

        self._max_distance = max_distance
        self._n_max_object = n_max_object

    def group(self, objects):
        '''!
            @todo Implementar
        '''


class DataGrouper(DistanceGrouper):

    def __init__(self, max_distance, n_max_object=0):
        super(DistanceGrouper).__init__(max_distance, n_max_object)

    def group(self, objects):
        '''!
            @todo Implementar
        '''

class DataConsensusGrouper(DistanceGrouper):

    def __init__(self, max_distance, n_max_object=0):
        super(DistanceGrouper).__init__(max_distance, n_max_object)

    def group(self, objects):
        '''!
            @todo Implementar
        '''
