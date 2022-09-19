#!/usr/bin/env python
import rospy
import random
from mae_global_planner.srv import SubtourPlanService, SubtourPlanServiceRequest, SubtourPlanServiceResponse
from mae_global_planner.srv import GlobalPlanService, GlobalPlanServiceRequest, GlobalPlanServiceResponse
from mae_global_planner.msg import PointArray
from geometry_msgs.msg import Point
from std_msgs.msg import Empty


def callback(msg):
    rospy.loginfo("called succesffully")


import math
if __name__ == "__main__":
    rospy.init_node("test_node")
    rospy.loginfo("Starting test_node")

    test_subscriber = rospy.Subscriber("test", Empty, callback)
    rospy.wait_for_service("make_subtour")
    rospy.wait_for_service("make_global_plan")
    rospy.loginfo("Services are available")
    subtour_plan_service = rospy.ServiceProxy("make_subtour", SubtourPlanService)
    global_plan_service = rospy.ServiceProxy("make_global_plan", GlobalPlanService)

    # create 100 random poitns in the range of 0 to 100
    points = []
    for i in range(100):
        points.append(Point(random.uniform(0, 100), random.uniform(0, 100), 0))
    # calculate total distance
    total_distance = 0
    for i in range(len(points) - 1):
        total_distance += math.sqrt((points[i].x - points[i + 1].x)
                                    ** 2 + (points[i].y - points[i + 1].y)**2)

    rospy.loginfo("Total distance: %f", total_distance)
    # create a point array
    point_array = PointArray()
    point_array.points = points

    # call subtour plan service
    subtour_plan_service_request = SubtourPlanServiceRequest()
    subtour_plan_service_request.targets = point_array
    subtour_plan_service_request.starting_position = Point(0.0, 0.0, 0.0)
    subtour_plan_service_request.subtour_length = 10
    subtour_plan_service_request.timeout_ms = 2000
    subtour_plan_service_response = subtour_plan_service(subtour_plan_service_request)

    # calculate distance of response
    subtour_distance = 0
    for i in range(len(subtour_plan_service_response.subtour.points) - 1):
        subtour_distance += math.sqrt((subtour_plan_service_response.subtour.points[i].x - subtour_plan_service_response.subtour.points[i + 1].x)
                                      ** 2 + (subtour_plan_service_response.subtour.points[i].y - subtour_plan_service_response.subtour.points[i + 1].y)**2)
    # print distance
    rospy.loginfo("Subtour distance: %f", subtour_distance)
    rospy.spin()
