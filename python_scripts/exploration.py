import matplotlib.pyplot as plt
import numpy as np
import cv2
import math

def calculate_distance_between_waypoints(fov, camera_angle, altitude):
    """
    Calculates the distance between waypoints in km
    """
    hypotenuse = (1 / math.cos(deg2rad(camera_angle))) * altitude
    return (2 * math.tan(deg2rad(fov / 2)) * hypotenuse) / 1000

def deg2rad(deg):
    return deg * (math.pi / 180)

def is_point_in_polygon(point, polygon):
    """
    Check if a point is inside a polygon using the Ray-Casting method.

    Args:
        point (tuple): The point to check, e.g., (x, y).
        polygon (list): List of vertices of the polygon in order, e.g., [(x1, y1), (x2, y2), ...].

    Returns:
        bool: True if the point is inside the polygon, False otherwise.
    """
    x, y = point
    n = len(polygon)
    inside = False

    for i in range(n):
        x1, y1 = polygon[i]
        x2, y2 = polygon[(i + 1) % n]

        # Check if point is on an edge of the polygon
        if min(y1, y2) < y <= max(y1, y2) and x <= max(x1, x2):
            if y1 != y2:
                x_intersect = (y - y1) * (x2 - x1) / (y2 - y1) + x1
            if x1 == x2 or x <= x_intersect:
                inside = not inside

    return inside


def generate_points_within_polygon(polygon, step=0.1):
    """
    Seçilen çokgen bir alanı grid hale getirir ve bu griddeki noktaları döndürür.

    Args:
        polygon (list): Çokgenin koordinatlarını içeren [(x1, y1), (x2, y2), ...] formatındaki liste.
        step (float): Her bir nokta arasındaki mesafe.

    Returns:
        list: Çokgen içindeki noktaları [(x, y), ...] formatında döndürür.
    """
    # Çokgenin sınırlarını belirle
    min_x = min(p[0] for p in polygon)
    max_x = max(p[0] for p in polygon)
    min_y = min(p[1] for p in polygon)
    max_y = max(p[1] for p in polygon)

    # Belirtilen aralıklarla bir grid oluştur
    x_coords = frange(min_x, max_x, step)
    y_coords = frange(min_y, max_y, step)

    # Griddeki her bir noktayı kontrol et
    points = []
    for i in range(len(x_coords)):
        if i % 2 == 0:
            for y in y_coords:
                if is_point_in_polygon([x_coords[i], y], polygon):
                    points.append([x_coords[i], y])
        else:
            for y in reversed(y_coords):
                if is_point_in_polygon([x_coords[i], y], polygon):
                    points.append([x_coords[i], y])
    
    return points


def frange(start, stop, step):
    """
    float aralıklar oluşturur.

    Args:
        start (float): Başlangıç değeri.
        stop (float): Bitiş değeri.
        step (float): Adım boyutu.

    Returns:
        generator: float değerlerini üreten bir generator.
    """
    array = []
    while start < stop:
        start += step
        array.append(start)
    return array


def plot_polygon_and_points(polygon, points):
    """
    Plot a polygon and points on a 2D plane.

    Args:
        polygon (list): List of vertices of the polygon in order, e.g., [(x1, y1), (x2, y2), ...].
        points (list): List of points to plot, e.g., [(x1, y1), (x2, y2), ...].
    """
    # Close the polygon by appending the first point at the end
    polygon_closed = polygon + [polygon[0]]
    
    # Separate x and y coordinates
    polygon_x, polygon_y = zip(*polygon_closed)

    # Plot the polygon
    plt.figure(figsize=(6, 6))
    plt.plot(polygon_x, polygon_y, 'b-', label='Polygon')
    plt.fill(polygon_x, polygon_y, color='lightblue', alpha=0.5)

    # Plot the points
    for point in points:
        plt.plot(point[0], point[1], 'go', label='Inside Point')  # Green for inside


    # Draw lines between each pair of points
    for i in range(len(points) - 1):
        x_values = [points[i][0], points[i + 1][0]]
        y_values = [points[i][1], points[i + 1][1]]
        plt.plot(x_values, y_values, 'k-')  # Black line

    # Set labels and title
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Polygon and Points')
    plt.grid(True)
    print("Plotting...")
    plt.show()

def scale_points(points, scale_factor):
    return [(int(x * scale_factor), int(y * scale_factor)) for x, y in points]

def center_points(points, img_size):
    # Calculate the bounding box of the points
    min_x = min(points, key=lambda p: p[0])[0]
    max_x = max(points, key=lambda p: p[0])[0]
    min_y = min(points, key=lambda p: p[1])[1]
    max_y = max(points, key=lambda p: p[1])[1]

    # Calculate the center of the bounding box
    center_x = (min_x + max_x) // 2
    center_y = (min_y + max_y) // 2

    # Calculate the translation needed to center the points in the image
    translate_x = img_size // 2 - center_x
    translate_y = img_size // 2 - center_y

    # Translate the points
    centered_points = [(x + translate_x, y + translate_y) for x, y in points]
    return centered_points

def show_polygon_and_points(polygon, points, img_size=500):
    """
    Plot a polygon and points on a 2D plane using OpenCV.

    Args:
        polygon (list): List of vertices of the polygon in order, e.g., [(x1, y1), (x2, y2), ...].
        points (list): List of points to plot, e.g., [(x1, y1), (x2, y2), ...].
        img_size (int): Size of the image in pixels (img_size x img_size).
    """
    # Determine the scale factor to fit the points within the image size
    max_x = max(max(polygon, key=lambda p: p[0])[0], max(points, key=lambda p: p[0])[0])
    max_y = max(max(polygon, key=lambda p: p[1])[1], max(points, key=lambda p: p[1])[1])
    scale_factor = img_size / max(max_x, max_y)

    # Scale the polygon and points
    scaled_polygon = scale_points(polygon, scale_factor)
    scaled_points = scale_points(points, scale_factor)

    # Center the polygon and points
    centered_polygon = center_points(scaled_polygon, img_size)
    centered_points = center_points(scaled_points, img_size)

    # Create a blank image
    img = np.zeros((img_size, img_size, 3), dtype=np.uint8)

    # Convert polygon points to integer coordinates
    polygon_array = np.array(centered_polygon, np.int32)
    polygon_array = polygon_array.reshape((-1, 1, 2))

    # Draw the polygon
    cv2.polylines(img, [polygon_array], isClosed=True, color=(255, 0, 0), thickness=2)
    cv2.fillPoly(img, [polygon_array], color=(173, 216, 230))  # Light blue fill

    # Draw the points
    for point in centered_points:
        cv2.circle(img, (int(point[0]), int(point[1])), radius=5, color=(0, 255, 0), thickness=-1)  # Green for inside

    # Draw lines between each pair of points
    for i in range(len(centered_points) - 1):
        pt1 = (int(centered_points[i][0]), int(centered_points[i][1]))
        pt2 = (int(centered_points[i + 1][0]), int(centered_points[i + 1][1]))
        cv2.line(img, pt1, pt2, color=(0, 0, 0), thickness=1)  # Black line

    img = cv2.flip(img, 0)

    # Display the image
    cv2.imshow('Polygon and Points', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    polygon = [(1, 1), (5, 1), (5, 5), (3, 7), (1, 5), (0, 4), (0, 2)]  # 7-gen bir çokgen

    generated_points = generate_points_within_polygon(polygon, step=1)

    # rastgele noktalar
    # random_points = [(2, 2), (4, 3), (2, 5), (3, 6), (0, 0), (6, 6)]

    # Çokgen ve noktaları çizdirme
    plot_polygon_and_points(polygon, generated_points)
    # show_polygon_and_points(polygon, generated_points)

