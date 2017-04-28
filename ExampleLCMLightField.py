import lcm
from exlcm import example_t

from LightFieldAPI import LightFieldAPI

LightFieldAPI = LightFieldAPI()

def my_handler(channel, data):
    msg = example_t.decode(data)
    print("Received message on channel \"%s\"" % channel)
    print("   timestamp   = %s" % str(msg.timestamp))
    print("   position    = %s" % str(msg.position))
    print("   orientation = %s" % str(msg.orientation))
    print("   ranges: %s" % str(msg.ranges))
    print("   name        = '%s'" % msg.name)
    print("   enabled     = %s" % str(msg.enabled))
    print("")
    level_list = str(msg.name).split('/')
    print level_list
    LightFieldAPI.addBox(level_list, 10, 10, 10)

import thread
lc = lcm.LCM()
subscription = lc.subscribe("EXAMPLE", my_handler)

def lchandle():
    while True:
        lc.handle()
thread.start_new_thread(lchandle, ())

LightFieldAPI.start()

lc.unsubscribe(subscription)