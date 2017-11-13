import signal
import pexpect
import time

spawn_command = "python test1.py"
child = pexpect.spawn(spawn_command)


while True:
    #print child.expect("Heart rate is:")
    if child.expect("Heart rate is:") < 2:
        status = child.before.split("\r\n")
        print status[0]
    else:
        print "it is not heare"
    
    time.sleep(10)