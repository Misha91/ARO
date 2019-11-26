from turtlebot import Turtlebot, detector
import numpy as np
import cv2
import math



def line_intersection(line1, line2):
    xdiff = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
    ydiff = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1]) #Typo was here

    def det(a, b):
        return a[0] * b[1] - a[1] * b[0]

    div = det(xdiff, ydiff)
    if div == 0:
       return None, None

    d = (det(*line1), det(*line2))
    x = det(d, xdiff) / div
    y = det(d, ydiff) / div
    return x, y

def main():
    maximum = 10
    count = 0
    speed = 30
    s = 0.0001

    prev_start = [[0] * 2 for i in range(maximum)]
    prev_end = [[0] * 2 for i in range(maximum)]
    circle = [[0] * 2 for i in range(maximum)]

    turtle = Turtlebot(rgb=True)
    cv2.namedWindow("OKNO")

    while not turtle.is_shutting_down():
        # get point cloud
        image = turtle.get_rgb_image()

        # wait for image to be ready
        if image is None:
            continue

        cv2.rectangle(image, (0, 0), (image.shape[1], int(image.shape[0] / 1.6)), (0, 0, 0), -1)
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        kernel_size = 5
        blur_gray = cv2.GaussianBlur(gray, (kernel_size, kernel_size), 0)

        ret, black = cv2.threshold(blur_gray, 215, 255, cv2.THRESH_BINARY)
        edges = cv2.Canny(black, 200, 255)
        rho = 2  # distance resolution in pixels of the Hough grid
        theta = np.pi / 60  # angular resolution in radians of the Hough grid
        threshold = 55  # minimum number of votes (intersections in Hough grid cell)
        min_line_length = 5  # minimum number of pixels making up a line
        max_line_gap = 40  # maximum gap in pixels between connectable line segments
        line_image = np.copy(image) * 0  # creating a blank to draw lines on

        lines = cv2.HoughLinesP(edges, rho, theta, threshold, np.array([]), min_line_length, max_line_gap)

        left_line_x = []
        left_line_y = []
        right_line_x = []
        right_line_y = []

        Rmotor = 0
        Lmotor = 0

        if lines is not None:
            for line in lines:
                for x1, y1, x2, y2 in line:
                    cv2.line(line_image, (x1, y1), (x2, y2), (0, 0, 255), 1)
                    if (x2 - x1) == 0:
                        continue
                    slope = (y2 - y1) / (x2 - x1)  # <-- Calculating the slope.
                    if math.fabs(slope) < 0.5:  # <-- Only consider extreme slope
                        continue
                    if slope <= 0:  # <-- If the slope is negative, left group.
                        left_line_x.extend([x1, x2])
                        left_line_y.extend([y1, y2])
                    else:  # <-- Otherwise, right group.
                        right_line_x.extend([x1, x2])
                        right_line_y.extend([y1, y2])

        min_y = image.shape[0] / 1.3  # <-- Just below the horizon
        max_y = image.shape[0]  # <-- The bottom of the image

        left_x_start = 0
        left_x_end = 0
        right_x_start= image.shape[1]
        right_x_end = image.shape[1]

        if len(left_line_x) != 0:
            poly_left = np.poly1d(np.polyfit(
                left_line_y,
                left_line_x,
                deg=1
            ))
            left_x_start = int(poly_left(max_y))
            left_x_end = int(poly_left(min_y))
            cv2.line(line_image, (int(left_x_start), int(max_y)), (int(left_x_end), int(min_y)), (0, 255, 0), 2)

        if len(right_line_x) != 0:
            poly_right = np.poly1d(np.polyfit(
                right_line_y,
                right_line_x,
                deg=1
            ))
            right_x_start = int(poly_right(max_y))
            right_x_end = int(poly_right(min_y))
            cv2.line(line_image, (int(right_x_start), int(max_y)), (int(right_x_end), int(min_y)), (0, 255, 0), 2)

        prev_start[count][0] = right_x_start
        prev_start[count][1] = left_x_start
        prev_end[count][0] = right_x_end
        prev_end[count][1] = left_x_end

        # print(prev_start)
        # print(prev_end)
        avg_start = [0] * 2
        avg_end = [0] * 2
        for i in range(maximum):
            avg_start[0] += prev_start[i][0]
            avg_start[1] += prev_start[i][1]
            avg_end[0] += prev_end[i][0]
            avg_end[1] += prev_end[i][1]
        avg_start[0] /= maximum
        avg_start[1] /= maximum
        avg_end[0] /= maximum
        avg_end[1] /= maximum

        center_x_start = int((avg_start[0] + avg_start[1]) / 2)
        center_x_end = int((avg_end[0] + avg_end[1]) / 2)

        Rmotor = speed + int(
            np.arctan2((center_x_start - center_x_end), (max_y - min_y)) * (180.0 / 3.141592653589793238463))
        Lmotor = speed + int(
            np.arctan2((center_x_end - center_x_start), (max_y - min_y)) * (180.0 / 3.141592653589793238463))
        Rmotor = 100 if Rmotor > 100 else Rmotor
        Lmotor = 100 if Lmotor > 100 else Lmotor

        cv2.line(line_image, (center_x_start, int(max_y)), (center_x_end, int(min_y)), (255, 0, 0), 3)
        cv2.line(line_image, (int(image.shape[1] / 2), int(max_y)),
                 (int((image.shape[1] / 2 - center_x_start) + center_x_end), int(min_y)), (255, 255, 0), 4)

        lines_edges = cv2.addWeighted(image, 0.8, line_image, 1, 0)

        cv2.putText(lines_edges, str(Lmotor), (int(image.shape[1] / 6), 200), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255),
                    4);
        cv2.putText(lines_edges, str(Rmotor), (int(image.shape[1] - image.shape[1] / 6), 200), cv2.FONT_HERSHEY_SIMPLEX,
                    1, (0, 0, 255), 4);

        circle[count][0], circle[count][1] = line_intersection(
            ([int(left_x_start), int(max_y)], [int(left_x_end), int(min_y)]),
            ([int(right_x_start), int(max_y)], [int(right_x_end), int(min_y)]))

        circleX = 0
        circleY = 0
        n = 0
        for i in range(maximum):
            if circle[i][0] is None:
                n += 1
                continue
            circleX += circle[i][0]
            circleY += circle[i][1]
        if maximum-n != 0:
            circleX /= (maximum - n)
            circleY /= (maximum - n)

        if circleX != 0 and circleY != 0:
            uhel = int(np.arctan2((image.shape[1] / 2 - circleX), (max_y - circleY)) * (180.0 / 3.141592653589793238463))
            if circleX > image.shape[1] / 2 - 40 and circleX < image.shape[1] / 2 + 40:
                cv2.circle(lines_edges, (int(circleX), int(circleY)), 8, (0, 255, 0), -1)
                turtle.cmd_velocity(linear=0.05+s)
                s += 0.001
            else:
                cv2.circle(lines_edges, (int(circleX), int(circleY)), 8, (0, 0, 255), -1)
                turtle.cmd_velocity(angular=np.sign(uhel)*0.5)
                s = 0.0001
            cv2.line(lines_edges, (int(image.shape[1] / 2 - (circleX - center_x_start)), int(image.shape[0] - 2)),
                     (int(image.shape[1] / 2), int(image.shape[0]) - 2), (0, 0, 255), 3)

            cv2.putText(lines_edges, str(uhel), (int(image.shape[1]/2), 200),cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 4);


        count += 1
        if count == maximum:
            count = 0

        cv2.line(lines_edges, (int(image.shape[1] / 2 - 40), 0), (int(image.shape[1] / 2 - 40), int(image.shape[0])),
                 (0, 211, 255), 1)
        cv2.line(lines_edges, (int(image.shape[1] / 2 + 40), 0), (int(image.shape[1] / 2 + 40), int(image.shape[0])),
                 (0, 211, 255), 1)

        cv2.imshow('lines', line_image)
        cv2.imshow('linesedge', lines_edges)
        # show image
        cv2.imshow("OKNO", image)
        cv2.imshow("OKNO1", edges)


        cv2.waitKey(1)

if __name__ == "__main__":
    main()
