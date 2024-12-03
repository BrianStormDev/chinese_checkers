class Point:
    def __init__ (self, x, y):
        self.x = x
        self.y = y

    def __add__(self, o):
        return Point(self.x + o.x, self.y + o.y)
    
    def __mul__(self, scalar):
        return Point(scalar * self.x, scalar * self.y)
    
    def __rmul__(self, scalar):
        return Point(scalar * self.x, scalar * self.y)
    
    def __repr__(self):
        return f'Point({self.x}, {self.y})'
    
    def __string__(self):
        return f'Point({self.x}, {self.y})'

    def __iter__(self):
        return iter((self.x, self.y))
    
    def __eq__(self, other):
        assert type(other) is Point, "Trying to compare a Point with a non-Point"
        if other.x == self.x and other.y == self.y:
            return True
        else:
            return False