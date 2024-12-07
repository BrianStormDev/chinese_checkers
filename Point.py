class Point:
    def __init__ (self, x, y):
        """
        Integer point class
        """
        self.x = x
        self.y = y

    def __add__(self, o):
        return Point(self.x + o.x, self.y + o.y)
    
    def __mul__(self, scalar):
        return Point(int(self.x * scalar), int(self.y * scalar))
    
    def __repr__(self):
        return f'Point ({self.x}, {self.y})'
    
    def __string__(self):
        return f'Point ({self.x}, {self.y})'
    
    def __eq__(self, other):
        return isinstance(other, Point) and other.x == self.x and other.y == self.y
    
    def __hash__(self):
        return 31 * self.x + self.y