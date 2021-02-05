#!/usr/bin/env python
from __future__ import absolute_import, division, print_function

from std_msgs.msg import Header
from visualization_msgs.msg import Marker
import cv2
import rospy


class LabelColours:
    def __init__(self, colour_list=None):
        if colour_list is None:
            colour_list = [[230, 25, 75], [60, 180, 75], [255, 225, 25], [0, 130, 200], [245, 130, 48], [145, 30, 180],
                           [70, 240, 240], [240, 50, 230], [210, 245, 60], [250, 190, 190], [0, 128, 128],
                           [230, 190, 255], [170, 110, 40], [255, 250, 200], [128, 0, 0], [170, 255, 195],
                           [128, 128, 0], [255, 215, 180], [0, 0, 128], [128, 128, 128], [255, 255, 255], [0, 0, 0]]
        self.colours = colour_list

    def __getitem__(self, item):
        if item < 0:
            item *= -1
        if item >= len(self.colours):
            item = item % len(self.colours)
        return self.colours[item]



class MarkerPublisher:
    def __init__(self, name, frame_id="map", colours=None, queue_size=1):
        self.publisher = rospy.Publisher(name, Marker, queue_size=queue_size)
        self.frame_id = frame_id

        self.clear_marker = Marker(action=3)
        self.clear_marker.header.frame_id = self.frame_id
        self.clear_marker.header.stamp = rospy.Time()

        if colours is None:
            colours = [[1.0, 1.0, 1.0]]
        if max(colours[0]) > 1:
            colours = [[c2 / 255.0 for c2 in c1] for c1 in colours]
        self.colours = colours

        self.__last_object_id = -1

    def clear_markers(self):
        self.publisher.publish(self.clear_marker)
        self.__last_object_id = -1

    def get_object_id(self):
        self.__last_object_id += 1
        return self.__last_object_id

    def visualise_points(self, point_information, header=None, clear=True, duration=None):
        # Create visualisation for markers
        if clear:
            # Clear previous markers
            self.clear_markers()

        # Fill marker array
        if header is None:
            header = Header()
            header.stamp = rospy.Time.now()

        for point_info in point_information:
            if len(point_info) not in (3, 4):
                rospy.logerr("Point array '{}' needs to be [x, y, z] or [x, y, z, colour_index]".format(point_info))
                continue

            x, y, z = point_info[:3]
            colour_id = 0
            if len(point_info) == 4:
                colour_id = point_info[3]

            marker = Marker()
            marker.header = header
            marker.header.frame_id = self.frame_id
            marker.id = self.get_object_id()
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.05
            marker.color.a = 1.0
            colour = self.colours[colour_id]
            marker.color.r = colour[0]
            marker.color.g = colour[1]
            marker.color.b = colour[2]
            marker.lifetime = rospy.Duration() if duration is None else rospy.Duration(duration)
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = z
            self.publisher.publish(marker)


def draw_bounding_box_msgs_on_image(image, bounding_box_msgs, classes=None, font=cv2.FONT_HERSHEY_SIMPLEX,
                                    font_scale=0.5, font_colour=(255, 255, 255), line_type=8):
    label_colours = LabelColours()
    for bounding_box in bounding_box_msgs:
        pt1 = (int(bounding_box.x1), int(bounding_box.y1))
        pt2 = (int(bounding_box.x2), int(bounding_box.y2))
        class_colour = label_colours[bounding_box.class_id]
        cv2.rectangle(image, pt1, pt2, color=class_colour)
        if classes is not None:
            padding = 10
            class_name = "{} {}%".format(classes[bounding_box.class_id], int(bounding_box.score * 100))
            text_width, text_height = cv2.getTextSize(class_name, font, font_scale, line_type)[0]
            pt3 = (pt1[0], pt1[1])
            pt4 = (pt1[0] + text_width + (padding // 2), pt1[1] - text_height - padding)
            cv2.rectangle(image, pt3, pt4, color=class_colour, thickness=-1)
            pt5 = (pt1[0] + (padding // 2), pt1[1] - (text_height // 2))
            cv2.putText(image, class_name, pt5, cv2.FONT_HERSHEY_SIMPLEX, font_scale, font_colour, lineType=line_type)
    return image
