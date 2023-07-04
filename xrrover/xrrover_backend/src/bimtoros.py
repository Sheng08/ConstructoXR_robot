#!/usr/bin/env python

import csv, time
import rospkg, rospy
from xrrover_backend.msg import Schedule, Schedules

class WorkTime:
  def __init__(self, secs, nsecs):
    self.secs = secs
    self.nsecs = nsecs

class BimSchedule:
  def __init__(self):
    self.start = None
    self.end = None
    self.name = None
    self.parent = None
    self.ok = self.is_schedule_exist()

  def is_schedule_exist(self):
    has_data = self.start and self.end and self.name
    return False if has_data == None else True
  
  def get_schedule(self, data):
    start_str = data['Planned Start']
    end_str = data['Planned End']
    relation_string = data['Attached']

    self.start = self.create_worktime(start_str)
    self.end = self.create_worktime(end_str)
    self.name = data['Name']
    self.parent = self.get_parent_of_work(relation_string)
    self.ok = self.is_schedule_exist()

  def create_worktime(self, time_str):
    if time_str:
      secs, nsecs = self.convert_to_secs_nsecs(time_str)
      work_time = WorkTime(secs, nsecs)
      return work_time
    return None

  def convert_to_secs_nsecs(self, date_string):
    if date_string == '':
      return None, None
    struct_time = time.strptime(date_string, "%m/%d/%Y %I:%M:%S %p")
    timestamp = int(time.mktime(struct_time))
    secs = int(timestamp)
    nsecs = int((timestamp - secs) * 1e9)
    return secs, nsecs
  
  def get_parent_of_work(self, data_string):
    if data_string:
      # jobs, nest-level
      relation_tree = data_string.split("->")
      parent_link = relation_tree[-2]
    else:
      # area, top-level
      parent_link = None
    return parent_link

def read_schedule_csv_data(file_path:str):
  data = []
  with open(file_path, 'r') as file:
    reader = csv.DictReader(file)
    for row in reader:
      data.append(row)
  return data

def generate_schedule_msg(schedule:BimSchedule):
  schedule_msg = Schedule()
  schedule_msg.start_time.secs = schedule.start.secs
  schedule_msg.start_time.nsecs = schedule.start.nsecs
  schedule_msg.end_time.secs = schedule.end.secs
  schedule_msg.end_time.nsecs = schedule.end.nsecs
  schedule_msg.work = schedule.name
  schedule_msg.parent = schedule.parent if schedule.parent else ''

  return schedule_msg
    

if __name__ == "__main__":
  # read schedule file
  backend_path = rospkg.RosPack().get_path('xrrover_backend')
  csv_file = backend_path + '/bim/schedule.csv'
  schedule_table = read_schedule_csv_data(csv_file)

  # gen schedules
  msgs = Schedules()
  for schedule_data in schedule_table:
    schedule = BimSchedule()
    schedule.get_schedule(schedule_data)
    if schedule.ok:
      msg = generate_schedule_msg(schedule)
      msgs.schedules.append(msg)

  # ros node pub schedules
  rospy.init_node('bimschedule')
  pub = rospy.Publisher('bim', Schedules, queue_size=1)
  rate = rospy.Rate(0.5)  # Publish the message at a rate of 0.5 Hz
  while not rospy.is_shutdown():
    pub.publish(msgs)
    rospy.loginfo_once(f"Publishing the bim schedule successfully")
    rate.sleep()
  