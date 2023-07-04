#!/usr/bin/env python

import time
from datetime import datetime
import rospy
from xrrover_backend.srv import CheckSchedule, CheckScheduleResponse
from xrrover_backend.msg import Schedules, Schedule


class SpaceCheckNode:
    def __init__(self):
        rospy.init_node('bim_schedule_server')
        self.result_service = rospy.Service('check_bim_schedule', CheckSchedule, self.handle_calculation)
        self.bim_subscriber = rospy.Subscriber('bim', Schedules, self.handle_bim)
        self.bim_data = None

    def handle_bim(self, msg):
        # Store the BIM schedules data for later calculation
        self.bim_data = msg

    def handle_calculation(self, request):
        if self.bim_data is None:
            rospy.logerr('No BIM data received yet.')
            return CheckScheduleResponse(occupy=False, message='No BIM data received yet.')

        is_occupy = False
        current_time = time.time()
        current_datetime = datetime.fromtimestamp(current_time)
        for s in self.bim_data.schedules:
            if s.parent == request.zone and s.work != "Painting":
                # rospy.loginfo(s.start_time.secs)
                # rospy.loginfo(s.end_time.secs)
                # rospy.loginfo("###################")
                start_datetime = datetime.fromtimestamp(s.start_time.secs)
                end_datetime = datetime.fromtimestamp(s.end_time.secs)
                if start_datetime <= current_datetime <= end_datetime:
                    is_occupy = True
                else:
                    is_occupy = False

        message = f"{request.zone} is " + ("occupy" if is_occupy else "unoccupy")
        return CheckScheduleResponse(occupy=is_occupy, message=message)


if __name__ == '__main__':
    try:
        bim_calculator = SpaceCheckNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
