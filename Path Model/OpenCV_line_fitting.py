import cv2
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import splprep, splev
from scipy.spatial.distance import cdist

def filter_contours_by_size(contours, min_area, min_length):
    valid_contours = []
    for contour in contours:
        area = cv2.contourArea(contour)
        length = cv2.arcLength(contour, True)
        if area > min_area and length > min_length:
            valid_contours.append(contour)
    return valid_contours

def find_closest_contour_point(contour, point):
    distances = np.sqrt((contour[:, 0, 0] - point[0]) ** 2 + (contour[:, 0, 1] - point[1]) ** 2)
    min_index = np.argmin(distances)
    return min_index

def fit_spline(contour, smoothness=0.2):
    x = contour[:, 0, 0]
    y = contour[:, 0, 1]
    y = max(y) - y  # Invert y-coordinates for correct plotting
    tck, u = splprep([x, y], s=smoothness)  # Smaller s means more smoothing
    new_points = splev(np.linspace(0, 1, 100), tck)
    return new_points, tck, u

def plot_difference(contour, spline_points):
    contour_points = np.column_stack((contour[:, 0, 0], max(contour[:, 0, 1]) - contour[:, 0, 1]))
    spline_points_transposed = np.column_stack(spline_points)
    distances = cdist(contour_points, spline_points_transposed, 'euclidean')
    min_distances = np.min(distances, axis=1)
    plt.figure()
    plt.plot(min_distances)
    plt.title('Distance from Contour Points to Spline')
    plt.xlabel('Contour Point Index')
    plt.ylabel('Distance (pixels)')
    plt.show()

def display_and_trim_contour(img, contour):
    global points
    points = []
    print("Please select two points to trim the contour.")

    def click_event(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            if len(points) < 2:
                cv2.circle(img, (x, y), 5, (255, 0, 0), -1)
                points.append((x, img.shape[0] - y))  # Save inverted y
                cv2.imshow('Image', img)

    cv2.imshow('Image', img)
    cv2.setMouseCallback('Image', click_event)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    if len(points) == 2:
        real_distance = float(input("Enter the real world distance between the points (in cm): "))
        pixel_distance = np.linalg.norm(np.array(points[0]) - np.array(points[1]))
        scale_factor = real_distance / pixel_distance  # cm per pixel

        start_idx = find_closest_contour_point(contour, points[0])
        end_idx = find_closest_contour_point(contour, points[1])
        trimmed_contour = contour[min(start_idx, end_idx):max(start_idx, end_idx) + 1]

        # Apply scaling to trimmed_contour
        scaled_trimmed_contour = np.array(trimmed_contour) * scale_factor

        spline_points, tck, u = fit_spline(scaled_trimmed_contour)
        plot_difference(scaled_trimmed_contour, spline_points)
        plt.figure()
        plt.plot(spline_points[0], spline_points[1], 'b-', label='Fitted Spline')
        plt.plot(scaled_trimmed_contour[:, 0, 0],
                 max(scaled_trimmed_contour[:, 0, 1]) - scaled_trimmed_contour[:, 0, 1], 'ro', label='Contour Points')
        plt.legend()
        plt.show()

def display_contours(img, contours):
    for i, contour in enumerate(contours):
        img_copy = img.copy()
        cv2.drawContours(img_copy, [contour], -1, (0, 255, 0), 3)
        cv2.imshow('Contour', img_copy)
        response = cv2.waitKey(0)
        if response == ord('y'):
            cv2.destroyAllWindows()
            return contour
        elif response == ord('n'):
            continue
    cv2.destroyAllWindows()
    return None

def main(img_path):
    img = cv2.imread(img_path)
    if img is None:
        print("Error loading the image.")
        return

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, 50, 150, apertureSize=3)
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    filtered_contours = filter_contours_by_size(contours, min_area=100, min_length=200)
    selected_contour = display_contours(img, filtered_contours)
    if selected_contour is not None:
        print("Correct contour selected.")
        display_and_trim_contour(img, selected_contour)
    else:
        print("No contour selected.")

# Usage
main('bead_maze2.jpeg')
