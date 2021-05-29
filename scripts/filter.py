import numpy as np
from abc import ABC, abstractmethod

class Filter(ABC):
    @abstractmethod
    def valid(self, object, objects):
        ...

class DistanceFrom(Filter):

    def __init__(self, close_type, max_distance):
        '''!
            @todo Verificar tipos de close_type e max_distance
        '''

        self._close_type = close_type
        self._max_distance = max_distance
    

    def valid(self, object, objects):
        '''!
            @todo Implementar
        '''
        
class Inside(Filter):

    def __init__(self, top_right, bottom_left):

        if isinstance(top_right, list):
            top_right = np.array(top_right)
        if isinstance(bottom_left, list):
            bottom_left = np.array(bottom_left)

        if (not isinstance(top_right, np.array)) or (not isinstance(bottom_left, np.array)):
            raise AttributeError("top_right and bottom_left must be np.array or list")

        self._top_right = top_right
        self._bottom_left = bottom_left
        
    def valid(self, object, objects):
        '''!
            @todo Implementar
        '''
        