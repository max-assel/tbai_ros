#!/usr/bin/env python3

import json
from pathlib import Path
import sys
import threading
import time

import yaml


def main():
  import rospy
  import rosgraph
  from geometry_msgs.msg import Twist
  from rosgraph_msgs.msg import Clock
  from tbai_ros_msgs.msg import RbdState

  attempt = Path(sys.argv[1])
  config = yaml.safe_load((attempt / 'metadata.yaml').read_text())['config']
  rospy.init_node('benchmark_observer', disable_signals=True)
  lock = threading.RLock()
  snapshot = {'clock': None, 'clock_wall': 0, 'state': None,
              'motion_start': None, 'armed': False, 'zero_sent': False,
              'services': []}
  publisher = rospy.Publisher(config['monitor']['motion_topic'], Twist, queue_size=1)

  def clock(message):
    with lock:
      value = message.clock.to_sec()
      if value != snapshot['clock']:
        snapshot['clock_wall'] = time.monotonic()
      snapshot['clock'] = value

  def state(message):
    with lock:
      stamp = message.stamp.to_sec()
      if snapshot['state'] is None or stamp != snapshot['state']['stamp']:
        snapshot['state'] = {'stamp': stamp, 'wall': time.monotonic(),
                             'values': list(message.rbd_state)}

  def motion(message):
    with lock:
      if not snapshot['armed'] or snapshot['motion_start'] is not None or snapshot['clock'] is None:
        return
      limits = config['monitor']
      moving = (max(abs(message.linear.x), abs(message.linear.y), abs(message.linear.z)) >
                limits['motion_linear_epsilon_mps'] or
                max(abs(message.angular.x), abs(message.angular.y), abs(message.angular.z)) >
                limits['motion_angular_epsilon_radps'])
      if moving:
        snapshot['motion_start'] = {'sim': snapshot['clock'], 'wall': time.monotonic()}

  subscriptions = [rospy.Subscriber(config['monitor']['clock_topic'], Clock, clock),
                   rospy.Subscriber(config['monitor']['state_topic'], RbdState, state),
                   rospy.Subscriber(config['monitor']['motion_topic'], Twist, motion)]

  def graph():
    master = rosgraph.Master(rospy.get_name())
    while not rospy.is_shutdown():
      try:
        _, _, services = master.getSystemState()
        with lock:
          snapshot['services'] = [name for name, _ in services]
      except Exception:
        pass 
      time.sleep(0.25)

  threading.Thread(target=graph, daemon=True).start()
  zero_start = None
  while not rospy.is_shutdown():
    with lock:
      if (attempt / 'arm').exists():
        snapshot['armed'] = True
      if (attempt / 'zero').exists():
        if zero_start is None:
          zero_start = time.monotonic()
        publisher.publish(Twist())
        snapshot['zero_sent'] = publisher.get_num_connections() > 0 and time.monotonic() - zero_start >= 0.5
      snapshot['wall'] = time.monotonic()
      content = json.dumps(snapshot)
    temporary = attempt / 'observer.json.tmp'
    temporary.write_text(content)
    temporary.replace(attempt / 'observer.json')
    time.sleep(config['readiness']['poll_wall_sec'])


if __name__ == '__main__':
  try:
    main()
  except KeyboardInterrupt:
    pass
