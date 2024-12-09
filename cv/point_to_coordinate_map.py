# Now pressuming that we are mapping out the board, we have to do some stuff
# Specifically, we want to sort the y coordinates and based on the x positions, we can figure which one of them comes first
# But it's tricky because it's like we have to find y ranges where the rows are 

# Not really, we can sort by the y coordinate. Then for each rows (1 peg, 3, pegs, etc...) we can sort those slices by x coordinate

def sort_points(points):
    """
    points: An array of point tuples
    """
    # row_lengths = [1, 2, 3, 4, 13, 12, 11, 10, 9, 10, 11, 12, 13, 4, 3, 2, 1]
    row_lengths = [4, 3, 2, 1]
    assert len(points) == sum(row_lengths), f"Not enough points detected! Expected {sum(row_lengths)} but got {len(points)}"

    sorted_points = []
    # Sort the points according to their y coordinate
    y_sorted = sorted(points, key = lambda point: point[1])
    # Note that we sort it based on the idea that the zero coordinate for y is at the top
    
    point_index = 0
    # For each of the rows, sort the points by their x coordinate
    for row_length in row_lengths:
        x_sorted = sorted(y_sorted[point_index: row_lengths + 1], key = lambda point: point[0])
        sorted_points += x_sorted
        point_index += row_length
    return sorted_points

# # We need to have some kind of function that takes a list of points going from top down and return a board from that
# def plot_points(top_down_points):
