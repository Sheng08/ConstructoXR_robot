import time

current_time = time.time()
print(current_time)
secs = int(current_time)
nsecs = int((current_time - secs) * 1e9)

print("Current time: {} seconds {} nanoseconds".format(secs, nsecs))

#######################3

# target_datetime_str = "6/12/2023 9:00:00 AM"
# def convert_to_secs_nsecs(date_string):
#     if date_string == '':
#         return None, None
#     struct_time = time.strptime(date_string, "%m/%d/%Y %I:%M:%S %p")
#     timestamp = int(time.mktime(struct_time))
#     secs = int(timestamp)
#     nsecs = int((timestamp - secs) * 1e9)
#     return secs, nsecs
# secs, nsecs = convert_to_secs_nsecs(target_datetime_str)
# print("Current time: {} seconds {} nanoseconds".format(secs, nsecs))