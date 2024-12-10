points = [(1, 2), (7, 3), (0, 1), (4, 2), (8, 3), (9, 11)]

y_sorted = sorted(points, key = lambda point: point[1])

print(y_sorted)

row_lengths = [1, 2, 3]

point_index = 0
sorted_points = []
for row_length in row_lengths:
    x_sorted = sorted(y_sorted[point_index: point_index + row_length], key = lambda point: point[0])
    sorted_points += x_sorted
    point_index += row_length

print(sorted_points)
print(sum([1, 2, 3, 4, 13, 12, 11, 10, 9, 10, 11, 12, 13, 4, 3, 2, 1]))