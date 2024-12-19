#!/usr/bin/env python
class Point:
    """
    Point class consisting of integer coordinates.
    """
    def __init__ (self, x, y):
        assert isinstance(x, int) and isinstance(y, int), "At least one of the input values was not an integer."
        self.x = x
        self.y = y

    def __add__(self, o):
        assert isinstance(o, Point), "The thing you are trying to add is not a Point."
        return Point(self.x + o.x, self.y + o.y)
    
    def __mul__(self, scalar):
        assert isinstance(scalar, int) or isinstance(scalar, float), "The scalar you inputted was not an int or a float."
        return Point(int(self.x * scalar), int(self.y * scalar))
    
    def __eq__(self, other):
        return isinstance(other, Point) and other.x == self.x and other.y == self.y
    
    def __repr__(self):
        return f'({self.x}, {self.y})'
    
    def __string__(self):
        return f'({self.x}, {self.y})'
    
    def __hash__(self):
        return hash(tuple([self.x, self.y]))