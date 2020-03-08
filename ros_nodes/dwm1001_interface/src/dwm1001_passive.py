#!/usr/bin/env python
""" For more info on the documentation go to https://www.decawave.com/sites/default/files/dwm1001-api-guide.pdf
"""

__author__     = "Jorge Pena Queralta"
__version__    = "0.1"
__maintainer__ = "Jorge Pena Queralta"
__email__      = "jopequ@utu.fi"
__status__     = "Development"


import rospy, time, serial, os
from dwm1001_apiCommands            import DWM1001_API_COMMANDS

from geometry_msgs.msg import Pose


# initialize ros rate 10hz


# initialize serial port connections



class dwm1001_localizer:

    def __init__(self) :
        """
        Initialize the node, open serial port
        """
        
        # Init node
        rospy.init_node('DWM1001_Passive', anonymous=False)
        
        # Set a ROS rate
        self.rate = rospy.Rate(1)
        
        # Empty dictionary to store topics being published
        self.topics = {}
        
        # Serial port settings
        self.dwm_port = rospy.get_param('~port')
        self.serialPortDWM1001 = serial.Serial(
            port = self.dwm_port,
            baudrate = 115200,
            parity = serial.PARITY_ODD,
            stopbits = serial.STOPBITS_TWO,
            bytesize = serial.SEVENBITS
        )
    

    def main(self) :
        """
        Initialize port and dwm1001 api

        :param:

        :returns: none

        """

        # close the serial port in case the previous run didn't closed it properly
        self.serialPortDWM1001.close()
        # sleep for one sec
        time.sleep(1)
        # open serial port
        self.serialPortDWM1001.open()

        # check if the serial port is opened
        if(self.serialPortDWM1001.isOpen()):
            rospy.loginfo("Port opened: "+ str(self.serialPortDWM1001.name) )
            # start sending commands to the board so we can initialize the board
            self.initializeDWM1001API()
            # give some time to DWM1001 to wake up
            time.sleep(2)
            # send command lec, so we can get positions is CSV format
            self.serialPortDWM1001.write(DWM1001_API_COMMANDS.LEC)
            self.serialPortDWM1001.write(DWM1001_API_COMMANDS.SINGLE_ENTER)
            rospy.loginfo("Reading DWM1001 coordinates")
        else:
            rospy.loginfo("Can't open port: "+ str(self.serialPortDWM1001.name))

        try:

            while not rospy.is_shutdown():
                # just read everything from serial port
                serialReadLine = self.serialPortDWM1001.read_until()

                try:
                    self.publishTagPositions(serialReadLine)

                except IndexError:
                    rospy.loginfo("Found index error in the network array!DO SOMETHING!")



        except KeyboardInterrupt:
            rospy.loginfo("Quitting DWM1001 Shell Mode and closing port, allow 1 second for UWB recovery")
            self.serialPortDWM1001.write(DWM1001_API_COMMANDS.RESET)
            self.serialPortDWM1001.write(DWM1001_API_COMMANDS.SINGLE_ENTER)

        finally:
            rospy.loginfo("Quitting, and sending reset command to dev board")
            # self.serialPortDWM1001.reset_input_buffer()
            self.serialPortDWM1001.write(DWM1001_API_COMMANDS.RESET)
            self.serialPortDWM1001.write(DWM1001_API_COMMANDS.SINGLE_ENTER)
            self.rate.sleep()
            serialReadLine = self.serialPortDWM1001.read_until()
            if "reset" in serialReadLine:
                rospy.loginfo("succesfully closed ")
                self.serialPortDWM1001.close()


    def publishTagPositions(self, serialData):
        """
        Publish anchors and tag in topics using Tag and Anchor Object

        :param networkDataArray:  Array from serial port containing all informations, tag xyz and anchor xyz

        :returns: none

        """

        arrayData = [x.strip() for x in serialData.strip().split(',')]

        # If getting a tag position
        if "POS" in arrayData[0] :

            tag_id = arrayData[2]
            if tag_id not in self.topics :
                self.topics[tag_id] = rospy.Publisher('/dwm1001/tag/'+tag_id+"/position", Pose, queue_size=100)

            p = Pose()
            p.position.x = float(arrayData[3])
            p.position.y = float(arrayData[4])
            p.position.z = float(arrayData[5])
            p.orientation.x = 0.0
            p.orientation.y = 0.0
            p.orientation.z = 0.0
            p.orientation.w = 1.0
            self.topics[tag_id].publish(p)

            rospy.loginfo("Tag " + tag_id + ": "
                          + " x: "
                          + str(p.position.x)
                          + " y: "
                          + str(p.position.y)
                          + " z: "
                          + str(p.position.z))



    def initializeDWM1001API(self):
        """
        Initialize dwm1001 api, by sending sending bytes

        :param:

        :returns: none

        """
        # reset incase previuos run didn't close properly
        self.serialPortDWM1001.write(DWM1001_API_COMMANDS.RESET)
        # send ENTER two times in order to access api
        self.serialPortDWM1001.write(DWM1001_API_COMMANDS.SINGLE_ENTER)
        # sleep for half a second
        time.sleep(0.5)
        self.serialPortDWM1001.write(DWM1001_API_COMMANDS.SINGLE_ENTER)
        # sleep for half second
        time.sleep(0.5)
        # send a third one - just in case
        self.serialPortDWM1001.write(DWM1001_API_COMMANDS.SINGLE_ENTER)




if __name__ == '__main__':
    try:
        dwm1001 = dwm1001_localizer()
        dwm1001.main()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


