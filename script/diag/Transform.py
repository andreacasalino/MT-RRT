import math

class Trasform:
    def __init__(self, angle, trasl):        
        self.cos = math.cos(angle)
        self.sin = math.sin(angle)
        self.traslation = [trasl[0], trasl[1]]

    @staticmethod
    def fromJson(json):
        return Trasform(json['angle'], json['traslation'])

    def apply(self, point):
        x = self.cos*point[0] - self.sin*point[1] + self.traslation[0]
        y = self.sin*point[0] + self.cos*point[1] + self.traslation[1]
        return [x,y]

    def applyAll(self, points):
        return [self.apply(point) for point in points]
