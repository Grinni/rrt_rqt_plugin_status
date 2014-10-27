import os
import rospy
import rospkg
import time
import roslib
roslib.load_manifest('rrt_rqt_plugin_status')


from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtCore import *
from python_qt_binding.QtGui import *
from std_msgs.msg import String
from std_msgs.msg import Int16
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Imu
from sensor_msgs.msg import RegionOfInterest
#from rrt_cmd_vel_serial.msg import Accu
from rrt_msgs.msg import Accu
from rrt_msgs.msg import Shell_cmd
from std_msgs.msg import Bool
from std_msgs.msg import Byte

from math import pi

#import roslaunch
#from roslaunch.core import RLException

from geometry_msgs.msg import Twist
from worldmodel_msgs.srv import AddObject
from worldmodel_msgs.msg import ObjectState
from subprocess import call

### LineEdit Background colours
LEDT_COLOUR_WHITE = "QLineEdit { background-color : rgb(255, 255, 255) }"
LEDT_COLOUR_YELLOW = "QLineEdit { background-color : rgb(255, 255, 0) }"
LEDT_COLOUR_ORANGE = "QLineEdit { background-color : rgb(255, 170, 0) }"
LEDT_COLOUR_RED = "QLineEdit { background-color : rgb(255, 0, 0) }"
LEDT_COLOUR_GREEN = "QLineEdit { background-color : rgb(0, 255, 0) }"




#TODO: serial_out checken
WAIT_PUB_SERIAL_OUT = 0.02    #waiting after pub new value at serial_out topic

#rrt_rqt_plugin_status
class RRTRqtPluginStatus(Plugin):
    
    def __init__(self, context):
        super(RRTRqtPluginStatus, self).__init__(context)
        self.setObjectName('RRTRqtPluginStatus')
        
        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns
        
        self._widget = QWidget()
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('rrt_rqt_plugin_status'),
          'resource', 'rrt_rqt_plugin_status.ui')
        loadUi(ui_file,  self._widget)
        self._widget.setObjectName('RRTRqtPluginStatusUI')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + 
              (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

################################################################################

        self._pub_serial_out = rospy.Publisher('serial_out', String)
        self._cmd_vel_publisher = rospy.Publisher('cmd_vel', Twist)
        self._shell_cmd_publisher = rospy.Publisher('shell_cmd', Shell_cmd) 
        self._pub_victim_ok = rospy.Publisher('victim_ok', Bool)
        self._pub_stop_robot = rospy.Publisher('stop_robot', Bool)
        #self._pub_stop_robot = rospy.Publisher('stop_robot_string', String)
        self._pub_states = rospy.Publisher('states', Int16)

        # Shell_cmd message
        self.message = Shell_cmd()
        self.message.Machine = "rrt-cuda2"


######       DEF COUNTDOWN            ##########################################
#COUNTDOWN
        self.timerCountdown = QTimer()
        self.timerCountdown.setInterval(1000)
        self.timerCountdown.setSingleShot(False)
        self.timerCountdown.timeout.connect(self.dec_countdown)
        self._widget.pbtn_countdown_start.pressed.connect(
          self.timerCountdown.start)
        self._widget.pbtn_countdown_pause.pressed.connect(
          self.timerCountdown.stop)
        self._widget.pbtn_countdown_reset.pressed.connect(
          self._on_countdown_reset_button_pressed)
        self._widget.pbtn_countdown_min_plus.pressed.connect(
          self.on_countdown_min_plus_pressed)
        self._widget.pbtn_countdown_min_minus.pressed.connect(
          self.on_countdown_min_minus_pressed)


######       DEF ALLGEMEIN       ##########################################        

          
# ACCUS
        self._topic_name = 'accu'
        self.message_class = Accu
# ACCU1
        self._subscriber_accu1 = rospy.Subscriber(
          self._topic_name, self.message_class, self.callback_accu1)  
        self.connect(self, SIGNAL("new_value_accu1"), self.set_ledt_accu1)           
# ACCU2       
        #self._subscriber_accu2 = rospy.Subscriber(
        #  self._topic_name, self.message_class, self.callback_accu2)  
        #self.connect(self, SIGNAL("new_value_accu2"), self.set_ledt_accu2)   

# CO2
        self._topic_name = 'co2'
        self.message_class = Int16
        self._subscriber_co2 = rospy.Subscriber(
          self._topic_name, self.message_class, self.callback_co2)  
        self.connect(self, SIGNAL("new_value_co2"), self.set_ledt_co2)

# US - Ultraschall
        self._topic_name = 'ultrasonic'
        self.message_class = Int16
        self._subscriber_US = rospy.Subscriber(
          self._topic_name, self.message_class, self.callback_US)  
        self.connect(self, SIGNAL("new_value_US"), self.set_ledt_US)




######       ADD FUNCTIONS          ##########################################

# VICTIM
        # Pushbutton for adding a victim in front of the robot and projecting 
        # it against   the wall       
        self._widget.addVictimInFrontOfRobotPushButton.pressed.connect (
          self.on_add_victim_in_front_of_robot_pressed)
        self._widget.addVictimInFrontOfRobotPushButton.setStyleSheet(
          'QPushButton {color: red}')

	# Pushbutton vor ignoring a victim
        self._widget.ignore_victim_push_button.pressed.connect(
          self.on_ignore_victim_pressed)
        self._widget.ignore_victim_push_button.setStyleSheet(
          'QPushButton {color: red}')
        


# QR-CODE
        # Add a Pushbutton for adding a qr_code in front of the robot and 
        # projecting it against the wall    
   
        self._widget.addQR_CodeInFrontOfRobotPushButton.pressed.connect(
          self.on_add_qr_code_in_front_of_robot_pressed)
        self._widget.addQR_CodeInFrontOfRobotPushButton.setStyleSheet(
          'QPushButton {color: blue}')
        
# subscriber vom QR-Code
        self._subscriber_qr = rospy.Subscriber('qr_code', String, self.callback_qrCode)
        self.connect(self, SIGNAL("new_qrCode"), self.write_QR_Code)


# HAZARD
        # Add a Pushbutton for adding a hazard label in front of the robot 
        # and projecting it against the wall
      
        self._widget.addHazardInFrontOfRobotPushButton.pressed.connect(
          self.on_add_hazard_in_front_of_robot_pressed)
        self._widget.addHazardInFrontOfRobotPushButton.setStyleSheet(
          'QPushButton {color: orange}')
        


######       BUTTONS          ##########################################


# RESET
        #Button for resetting the system state
        #resetPushButton = QPushButton("Reset State")       
        self._widget.resetPushButton.pressed.connect(self.handleReset)
        self._syscommand_pub = rospy.Publisher('syscommand', String)


# STOP -Pushbutton
        self.stop_isRunning = False
        self._widget.stop_push_button.pressed.connect(self.on_stop_pressed)


# START autonom - Button
        self._widget.startAutonom_push_button.pressed.connect(self.on_startAutonom_pressed)

# START manuell - Button
        self._widget.startManuell_button.pressed.connect(self.on_startManuell_pressed)


# SAVE PushButton
        self._widget.savePushButton.pressed.connect(self.save)

# HOKUYO STARTEN
#        self._widget.startHokuyoPushButton.pressed.connect(self.on_start_hokuyo)

# TEST
        # HOKUYO STARTEN
        #self._widget.testPushButton.pressed.connect(self.on_test)


# 3D MAPPING
        self.mapping3D_isRunning = False
        self._widget.start3DMappingPushButton.pressed.connect(self.on_start_3D_mapping)


# 3D PLANNER
        self.planner3D_isRunning = False
        self._widget.start3DPlannerPushButton.pressed.connect(self.on_start_3D_planner)

# 3D OCTOMAP MAPPING
        self.mappingOctomap_isRunning = False
        self._widget.startOctomapPushButton.pressed.connect(self.on_start_octomap_mapping)



# 2D PLANNER
        self.planner2D_isRunning = False
        self._widget.start2DPlannerPushButton.pressed.connect(self.on_start_2D_planner)


# 2D CONTROLLER
        self.controller2D_isRunning = False
        self._widget.start2DControllerPushButton.pressed.connect(self.on_start_2D_controller)


# KILL LASER
        #Button for killing the laser       
        self._widget.kill_laser_push_button.pressed.connect(self.on_kill_laser_pressed)

# KILL Thermocam & Telnet
        #Button for killing the thermocam       
        self._widget.kill_thermo_push_button.pressed.connect(self.on_kill_thermo_pressed)

# shutdown cuda2
        # Button for shutting down cuda
        self._widget.shutdown_cuda_push_button.pressed.connect(self.on_shutdown_cuda)


#start schrotty drive
        self._widget.start_schrotty_drive_push_button.pressed.connect(self.on_start_schrotty_drive)

# KILL MAPPING
        self._widget.kill_mapping_push_button.pressed.connect(self.on_kill_mapping)

# MOTION DETECTION
        self.motion_isRunning = False
        self._widget.motion_push_button.pressed.connect(self.on_motion_pressed)




######       ANZEIGEN          ##########################################

# IMU data
#        self._topic_name = 'imu'
        # alle Werte auslesen
#        self.message_class = Imu
#        self._subscriber = rospy.Subscriber(
#          self._topic_name, self.message_class, self.callback_co2)
        #self.subscriber = rospy.Subscriber('...', String, self.callback_imuData)
#        self.connect(self, SIGNAL("new_imu_data"), self.write_IMU_Data)



# KILL HEAD CAM
# rosnode kill /head_cam/usb_cam
        self._widget.kill_head_cam_push_button.pressed.connect(self.on_kill_head_pressed)


# MOTION MESSAGE
        self._subscriber_motion = rospy.Subscriber(
         'motion_detector', String, self.callback_motion)
        self.connect(self, SIGNAL("new_motion_message"), self.write_motion_message)


# FACE DETECTION MESSAGE
        #self._topic_name = 'roi'
        #self.message_class = RegionOfInterest()
        #self._subscriber_face_roi = rospy.Subscriber(
         #'roi', RegionOfInterest, self.callback_roi)
        #self.connect(self, SIGNAL("new_roi_message"), self.write_roi_message)

        self._subscriber_face_tracker = rospy.Subscriber(
          'face_tracker', String, self.callback_face_tracker)
        self.connect(self, SIGNAL("new_face_tracker"), self.write_face_tracker)



######       MODELS          ##########################################

# CHANGE ROBOT-DESCRIPTION
        self._widget.robot_comboBox.activated.connect(self.on_change_robot)

        


################################################################################
######       FUNCTIONS            ##############################################
################################################################################
      

######       FKT COUNTDOWN         ##########################################
#COUNTDOWN
    def dec_countdown(self):
        if self._widget.lcd_countdown_sec.intValue() == 0:
            if self._widget.lcd_countdown_min.intValue() == 0:
                #TODO: destruct QTimer
                pass
            else:
                self._widget.lcd_countdown_sec.display(59)
                self._widget.lcd_countdown_min.display(
                self._widget.lcd_countdown_min.intValue() - 1)
        else:
            self._widget.lcd_countdown_sec.display(
               self._widget.lcd_countdown_sec.intValue() - 1)

    def _on_countdown_reset_button_pressed(self):
        self._widget.lcd_countdown_sec.display(00)
        self._widget.lcd_countdown_min.display(15)

    def on_countdown_min_plus_pressed(self): 
        self._widget.lcd_countdown_min.display(
          self._widget.lcd_countdown_min.intValue() + 1)

    def on_countdown_min_minus_pressed(self):
        self._widget.lcd_countdown_min.display(
          self._widget.lcd_countdown_min.intValue() - 1)


######       FKT ALLGEMEIN            ##########################################
      
#ACCU1
    def callback_accu1(self, data):
      self.emit(SIGNAL("new_value_accu1"), data.Accu[0])

    def set_ledt_accu1(self, value):
      self._widget.ledt_accu1.setText("%.2f" % float(value) +" Volt")
      if value < 25:    
        self._widget.ledt_accu1.setStyleSheet(LEDT_COLOUR_ORANGE)
      elif value > 25.5:    
        self._widget.ledt_accu1.setStyleSheet(LEDT_COLOUR_WHITE)
      
#ACCU2
    def callback_accu2(self, data):
      self.emit(SIGNAL("new_value_accu2"), data.Accu[1])

    def set_ledt_accu2(self, value):
      self._widget.ledt_accu2.setText("%.2f" % float(value) +" Volt")
      if value < 25:
        self._widget.ledt_accu2.setStyleSheet(LEDT_COLOUR_ORANGE)
      elif value > 25.5:
        self._widget.ledt_accu2.setStyleSheet(LEDT_COLOUR_WHITE)


#CO2
    def callback_co2(self, data):
      self.emit(SIGNAL("new_value_co2"), data.data)

    def set_ledt_co2(self, value):
      self._widget.ledt_co2.setText("%.2f" % float(value) +" ppm")


#US
    def callback_US(self, data):
      self.emit(SIGNAL("new_value_US"), data.data)

    def set_ledt_US(self, value):
      value -= 15;
      self._widget.ledt_US.setText("%.2f" % float(value) +" cm")
      if value < 18:
        self._widget.ledt_US.setStyleSheet(LEDT_COLOUR_RED)
      elif value < 28:
        self._widget.ledt_US.setStyleSheet(LEDT_COLOUR_ORANGE)
      elif value >= 28:
        self._widget.ledt_US.setStyleSheet(LEDT_COLOUR_GREEN)



######       BUTTONS          ##########################################


# save -> Skript ausfuehren
    def save(self):
      #ToDo: skript zum Speichern einfuegen!
      
      self._widget.QR_Codes.append("<font color=\"#00ff00\">save</font>")


# KILL LASER
    def on_kill_laser_pressed(self):
      self.message.Command = "rosnode kill /hokuyo"
      self._shell_cmd_publisher.publish(self.message)
      #self._widget.QR_Codes.append("<font color=\"#00ff00\">kill laser</font>")



# KILL THERMOCAM & TELNET
    def on_kill_thermo_pressed(self):
      self._widget.QR_Codes.append("<font color=\"#00ff00\">kill thermocam & telnet</font>")
      # kill thermocam:
      self.message.Command = "rosnode kill /Thermocam"
      self._shell_cmd_publisher.publish(self.message)
      time.sleep(2)
      # kill telnet:
      self.message.Command = "screen -X -S telnet quit"
      self._shell_cmd_publisher.publish(self.message)


# shutdown cuda
    def on_shutdown_cuda(self):
      self._widget.QR_Codes.append("<font color=\"#00ff00\">shutdown cuda!</font>")

      # kill thermocam:
      self.on_kill_thermo_pressed
      time.sleep(2)
      # shutdwon cuda:
      self.message.Command = "shutdown -h now"
      self._shell_cmd_publisher.publish(self.message)


# start schrotty_drive
    def on_start_schrotty_drive(self):
      #self._widget.QR_Codes.append("<font color=\"#00ff00\">start schrotty drive</font>")
      self.message.Command = "screen -dmS schrotty_drive /home/rrt/autorun/schrotti_drive.sh"
      self._shell_cmd_publisher.publish(self.message)


# KILL MAPPING
    def on_kill_mapping(self):
      self.message.Command = "rosnode kill /hector_mapping"
      self._shell_cmd_publisher.publish(self.message)


# MOTION DETECTION
    def on_motion_pressed(self):
      self.message.Machine = "rrt-commander"
      if self.motion_isRunning == False:
        self.motion_isRunning = True
        self.message.Command = "/home/rrt/rrt_startscripts/start-motion.sh"
        self._shell_cmd_publisher.publish(self.message)
        self._widget.start2DPlannerPushButton.setFlat(true)
        self._widget.QR_Codes.append("<font color=\"#00ff00\">motion_isRunning = True</font>")
      else:
        self.motion_isRunning = False
        #kill 3D planner node 
        self.message.Command = "rosnode kill /motion_detector"
        self._shell_cmd_publisher.publish(self.message)
        self._widget.QR_Codes.append("<font color=\"#00ff00\">motion_isRunning = False</font>")
      self.message.Machine = "RRT-EAGLEEYE"




# Stop-Push Button
    def on_stop_pressed(self):
        self._pub_states.publish(0)
        


# Start-Push Button
    def on_startAutonom_pressed(self):
      self._pub_states.publish(1)


# Start Manuell -Push Button
    def on_startManuell_pressed(self):
      self._pub_states.publish(10)



# Reset State
    def handleReset(self):
      tmp = String("reset")
      self._syscommand_pub.publish(tmp)
      #self._widget.ledt_my.setText("Reset State Pressed")




# KILL HEAD CAM
    def on_kill_head_pressed(self):
      call("rosnode kill /head_cam/usb_cam")
#      call("ls")


######       2D Planner & Controller     ##########################################

# 2D PLANNER starten
    def on_start_2D_planner(self):
      if self.planner2D_isRunning == False:
        self.planner2D_isRunning = True
        #tmp = Shell_cmd()
        #tmp.Machine = "rrt-cuda2"
        self.message.Command = "/home/rrt/autorun/planner_twoD.sh"
        self._shell_cmd_publisher.publish(self.message)
        #self._widget.QR_Codes.append("<font color=\"#00ff00\">planner2D_isRunning = True</font>")
      else:
        self.planner2D_isRunning = False
        #kill 3D planner node 
        self.message.Command = "rosnode kill /hector_exploration_node"
        self._shell_cmd_publisher.publish(self.message)
        #self._widget.QR_Codes.append("<font color=\"#00ff00\">planner2D_isRunning = False</font>")


# 2D CONTROLLER starten
    def on_start_2D_controller(self):
      if self.controller2D_isRunning == False:
        self.controller2D_isRunning = True
        #tmp = Shell_cmd()
        #tmp.Machine = "rrt-cuda2"
        self.message.Command = "/home/rrt/autorun/controller_twoD.sh"
        self._shell_cmd_publisher.publish(self.message)
        #self._widget.QR_Codes.append("<font color=\"#00ff00\">controller2D_isRunning = True</font>")
      else:
        self.controller2D_isRunning = False
        #kill 3D planner node 
        self.message.Command = "rosnode kill /hector_exploration_controller"
        self._shell_cmd_publisher.publish(self.message)
        #self._widget.QR_Codes.append("<font color=\"#00ff00\">controller2D_isRunning = False</font>")




######       2D Planner & Mapping     ##########################################


# 3D MAPPING starten
    def on_start_3D_mapping(self):
      if self.mapping3D_isRunning == False:
        self.mapping3D_isRunning = True
        #tmp = Shell_cmd()
        #tmp.Machine = "rrt-cuda2"
        self.message.Command = "/home/rrt/autorun/mapping_threeD.sh"
        self._shell_cmd_publisher.publish(self.message)
        #self._widget.QR_Codes.append(
          #"<font color=\"#00ff00\">mapping3D_isRunning = True</font>")
      else:
        self.mapping3D_isRunning = False
        #tmp = Shell_cmd()
        #tmp.Machine = "rrt-cuda2"
        #kill 3D planner node 
        self.message.Command = "rosnode kill /keyframe_mapper_node"
        self._shell_cmd_publisher.publish(self.message)
        time.sleep(2)
        self.message.Command = "rosnode kill /rgbd_image_proc"
        self._shell_cmd_publisher.publish(self.message)
        time.sleep(2)
        self.message.Command = "rosnode kill /rgbd_manager"
        self._shell_cmd_publisher.publish(self.message)
        #self._widget.QR_Codes.append("<font color=\"#00ff00\">mapping3D_isRunning = False</font>")



# 3D PLANNER starten
    def on_start_3D_planner(self):
      if self.planner3D_isRunning == False:
        self.planner3D_isRunning = True
        #tmp = Shell_cmd()
        #tmp.Machine = "rrt-cuda2"
        self.message.Command = "/home/rrt/autorun/planner_threeD.sh"
        self._shell_cmd_publisher.publish(self.message)
        #self._widget.QR_Codes.append("<font color=\"#00ff00\">planner3D_isRunning = True</font>")
      else:
        self.planner3D_isRunning = False
        #kill 3D planner node 
        self.message.Command = "rosnode kill /matilda_control"
        self._shell_cmd_publisher.publish(self.message)
        time.sleep(2)
        self.message.Command = "rosnode kill /velocity_imu_control"
        self._shell_cmd_publisher.publish(self.message)
        #self._widget.QR_Codes.append("<font color=\"#00ff00\">planner3D_isRunning = False</font>")






# 3D MAPPING starten
    def on_start_octomap_mapping(self):
      if self.mappingOctomap_isRunning == False:
        self.mappingOctomap_isRunning = True
        self.message.Command = "screen -mdS octo roslaunch octomap_server octomap_mapping.launch"
        #self.message.Command = "/home/rrt/autorun/octomap_mapping.sh"
        self._shell_cmd_publisher.publish(self.message)
      else:
        self.mappingOctomap_isRunning = False
        #kill octomap node 
        self.message.Command = "rosnode kill /octomap_server"
        self._shell_cmd_publisher.publish(self.message)










######       MODELS          ##########################################

# CHANGE ROBOT-DESCRIPTION
    def on_change_robot(self):
      index = self._widget.robot_comboBox.currentIndex()
      text = self._widget.robot_comboBox.currentText()
      if index == 0:
        # Mark
        #message = Shell_cmd()
        #message.Machine = "rrt-cuda2"

        # set Machine for rrt_shell_pub am GUI-ausfuehrenden Rechner
        self.message.Machine = os.environ['ROS_HOSTNAME']

        # kill other model 
        self.message.Command = "rosnode kill /joint_state_publisher"
        self._shell_cmd_publisher.publish(self.message)
        time.sleep(2)
        self.message.Command = "rosnode kill /robot_state_publisher"
        self._shell_cmd_publisher.publish(self.message)
        time.sleep(2)
        self.message.Command = "screen -S model_blackScorpion -p 0 -X quit"
        self._shell_cmd_publisher.publish(self.message)
        time.sleep(2)
        self.message.Command = "screen -S model_redScorpion -p 0 -X quit"
        self._shell_cmd_publisher.publish(self.message)
        time.sleep(2)

        
        # start model of mark
        self.message.Command = "screen -dmS model_mark /home/rrt/ros_workspace/rrt_rqt_plugin_status/sh_files/model_mark.sh"        
        self._shell_cmd_publisher.publish(self.message)
        self._widget.QR_Codes.append("text: "+text)
        self._widget.QR_Codes.append("NOT implementet by now!!!!")

        # set Machine for rrt_shell_pub
        self.message.Machine = "rrt-eagleye"


      elif index == 1:
        # Schrotti
        #message = Shell_cmd()
        #message.Machine = "rrt-cuda2"

        # set Machine for rrt_shell_pub am GUI-ausfuehrenden Rechner
        self.message.Machine = os.environ['ROS_HOSTNAME']
 
        # kill other model 
        self.message.Command = "rosnode kill /joint_state_publisher"
        self._shell_cmd_publisher.publish(self.message)
        time.sleep(2)
        self.message.Command = "rosnode kill /robot_state_publisher"
        self._shell_cmd_publisher.publish(self.message) 
        time.sleep(2)
        self.message.Command = "screen -S model_blackScorpion -p 0 -X quit"
        self._shell_cmd_publisher.publish(self.message)
        time.sleep(2)
        self.message.Command = "screen -S model_redScorpion -p 0 -X quit"
        self._shell_cmd_publisher.publish(self.message)
        time.sleep(2)

        # start model of Schrotty
#        self.message.Command = "/home/rrt/autorun/model_schrotty.sh"
        self.message.Command = "screen -dmS model_schrotti /home/rrt/ros_workspace/rrt_rqt_plugin_status/sh_files/model_schrotti.sh"

        
        self._shell_cmd_publisher.publish(self.message)
        self._widget.QR_Codes.append("text: "+text)

        # set Machine for rrt_shell_pub -> Schrotti
        self.message.Machine = "rrt-cuda2"

      elif index == 2:
        # Black Scorpion
        # set Machine for rrt_shell_pub am GUI-ausfuehrenden Rechner
        self.message.Machine = os.environ['ROS_HOSTNAME']

        # kill other model 
        self.message.Command = "rosnode kill /joint_state_publisher"
        self._shell_cmd_publisher.publish(self.message)
        time.sleep(2)
        self.message.Command = "rosnode kill /robot_state_publisher"
        self._shell_cmd_publisher.publish(self.message) 
        time.sleep(2)
        self.message.Command = "screen -S model_redScorpion -p 0 -X quit"
        self._shell_cmd_publisher.publish(self.message)
        time.sleep(2)

        # start model of Black Scorpion
#        self.message.Command = "/home/rrt/autorun/model_black_scorpion.sh"
        self.message.Command = "screen -dmS model_blackScorpion /home/rrt/ros_workspace/rrt_rqt_plugin_status/sh_files/model_black_scorpion.sh"
        
        self._shell_cmd_publisher.publish(self.message)
        self._widget.QR_Codes.append("no robot! -> red scorpion")

        self.message.Machine = "rrt-devil"

      elif index == 3:
        # Red Scorpion
        # set Machine for rrt_shell_pub am GUI-ausfuehrenden Rechner
        self.message.Machine = os.environ['ROS_HOSTNAME']

        # kill other model 
        self.message.Command = "rosnode kill /joint_state_publisher"
        self._shell_cmd_publisher.publish(self.message)
        time.sleep(2)
        self.message.Command = "rosnode kill /robot_state_publisher"
        self._shell_cmd_publisher.publish(self.message) 
        time.sleep(2)
        self.message.Command = "screen -S model_blackScorpion -p 0 -X quit"
        self._shell_cmd_publisher.publish(self.message)
        time.sleep(2)

        # start model of Black Scorpion
        self.message.Command = "screen -dmS model_redScorpion /home/rrt/ros_workspace/rrt_rqt_plugin_status/sh_files/model_black_scorpion.sh"
#        self.message.Command = "/home/rrt/autorun/model_black_scorpion.sh"
        
        self._shell_cmd_publisher.publish(self.message)
        self._widget.QR_Codes.append("text: "+text)

        # set Machine for rrt_shell_pub -> Red Scorpion
        self.message.Machine = "rrt-devil"







######       ADD DEFINITIONS          ##########################################

# VICTIM
# Add victim
    def on_add_victim_in_front_of_robot_pressed(self):

      #self._widget.ledt_my.setText("Add victim")
      #self._pub_victim_confirmed.publish(100)
      ##self._pub_victim_ok.publish(True)
            
      try:
          add_victim = rospy.ServiceProxy('worldmodel/add_object', AddObject)
          
          req = AddObject._request_class()
          req.object.header.frame_id = 'base_link'
          req.object.header.stamp = rospy.Time(0)
          req.map_to_next_obstacle = True
          req.object.info.class_id = "victim"
          req.object.info.support = 100
          req.object.pose.pose.position.x = 0.1
          req.object.pose.pose.position.y = 0
          req.object.pose.pose.position.z = 0
          req.object.pose.pose.orientation.w = 1
          req.object.state.state = ObjectState.CONFIRMED

          resp = add_victim(req)
            
          status_msg = "added Victim, id = "
          status_msg += resp.object.info.object_id
          print(status_msg)

          self._widget.QR_Codes.append("<font color=\"#ff0000\">" + status_msg + "</font>")
          
          self._pub_states.publish(4);
          #self._pub_victim_ok.publish(True)          

            
            #self._task_id = resp.object.info.object_id
            #self._victimAnswer = VictimAnswer.CONFIRM
            #self._publish_answer()

            
            
      except rospy.ServiceException, e:
        #@TODO: Make this more expressive
        print("service call failed")
 

# add Victim-Positionen to Textbox 
#    def callback_victim(self, data):
#      self.emit(SIGNAL("new_victim"), data.data)


    def on_ignore_victim_pressed(self):
      # send a false on victim ok publisher
      #self._widget.QR_Codes.append("<font color=\"#ff0000\">ignore victim pressed</font>")
      #self._pub_victim_confirmed.publish(0);
      #self._pub_victim_ok.publish(True)
      self._pub_states.publish(5);




# Add hazard
    #-----rrt-fh-wels: Giuliano Roland    
    def on_add_hazard_in_front_of_robot_pressed(self):
            
        try:
            add_hazard = rospy.ServiceProxy('worldmodel/add_object', AddObject)
	    
            req = AddObject._request_class()
            req.object.header.frame_id = 'base_link'
            req.object.header.stamp = rospy.Time(0)
            req.map_to_next_obstacle = True
            req.object.info.class_id = "hazard"
            req.object.info.support = 100
            req.object.pose.pose.position.x = 0.1
            req.object.pose.pose.position.y = 0
            req.object.pose.pose.position.z = 0
            req.object.pose.pose.orientation.w = 1
            req.object.state.state = ObjectState.CONFIRMED

            resp = add_hazard(req)
            
            status_msg = "added Hazard Label, id = "
            status_msg += resp.object.info.object_id
            print(status_msg)

            self._widget.QR_Codes.append("<font color=\"#ff6600\">" + status_msg + "</font>")
            
            #-----rrt-fh-wels: Giuliano Roland   
            #self._task_id = resp.object.info.object_id
            #self._hazardAnswer = HazardAnswer.CONFIRM
            #self._publish_answer()
            
        except rospy.ServiceException, e:
	    print("Hazard service call failed")

# add hazard-position to textbox
#    def callback_hazard(self, data):
#      self.emit(SIGNAL("new_hazard"), data.data)



# Add QR-code
#-----rrt-fh-wels: Giuliano Roland    
    def on_add_qr_code_in_front_of_robot_pressed(self):
            
        try:
            add_qr_code = rospy.ServiceProxy('worldmodel/add_object', AddObject)
	    
            req = AddObject._request_class()
            req.object.header.frame_id = 'base_link'
            req.object.header.stamp = rospy.Time(0)
            req.map_to_next_obstacle = True
            req.object.info.class_id = "qrcode"
            req.object.info.support = 100
            req.object.pose.pose.position.x = 0.1
            req.object.pose.pose.position.y = 0
            req.object.pose.pose.position.z = 0
            req.object.pose.pose.orientation.w = 1
            req.object.state.state = ObjectState.CONFIRMED

            resp = add_qr_code(req)
            
            status_msg = "added QR_Code, id = "
            status_msg += resp.object.info.object_id
            print(status_msg)

            self._widget.QR_Codes.append("<font color=\"#0000ff\">" + status_msg + "</font>")
            
            #-----rrt-fh-wels: Giuliano Roland   
            #self._task_id = resp.object.info.object_id
            #self._qr_codeAnswer = QR_CodeAnswer.CONFIRM
            #self._publish_answer()
            
        except rospy.ServiceException, e:
	    print("QR_Code service call failed") 

# write qr_code to textbox
    def callback_qrCode(self, data):
      self.emit(SIGNAL("new_qrCode"), data.data)

    def write_QR_Code(self, value):
      self._widget.QR_Codes.append("<font color=\"#0000ff\">" + value + "</font>")

#---------------------------------------------------------------------
# Publisher fuer QR-Code-Anzeige

# wenns ned eh drinnen is....
#import rospy
#from std_msgs.msg import String   
# iwo am Anfang im QR-Code Client
#pub = rospy.Publisher('qr_code', String)
#pub = rospy.Publisher('hazards', String)
#pub = rospy.Publisher('victims', String)
# im QR-Code Client nachm Schreiben in die Datei
#pub.publish(String(str))
#--------------------------------------------------------------------
    



######       ANZEIGEN          ##########################################

# IMU Daten
#    def callback_imuData(self, data):
#      self.emit(SIGNAL("new_imu_data"), data.data)

#    def write_IMU_Data(self, value):
#      self._widget.imuBrowser.append(value.angular_velocity.x)   



# MOTION MESSAGE
# write motion message to textbox
    def callback_motion(self, data):
      self.emit(SIGNAL("new_motion_message"), data.data)

    def write_motion_message(self, value):
      #self._widget.textBox_motion_message.append("new MotionMessage")
      self._widget.textBox_motion_message.append("<font color=\"#00ff00\">" + value + "</font>")



# FACE DETECTION
# write roi message to textbox
    def callback_roi(self, data):
      self.emit(SIGNAL("new_roi_message"), data)

    def write_roi_message(self, value):
      self._widget.textBox_motion_message.append("<font color=\"#00ff00\">"
        + "x_offset: %d" % int(value.x_offset) + "  "
        + "y_offset: %d" % int(value.y_offset) + "  "
        + "height: %d" % int(value.height) + "  "
        + "width: %d" % int(value.width) + "</font>")


# write face_tracker message to textbox
    def callback_face_tracker(self, data):
      self.emit(SIGNAL("new_face_tracker"), data.data)

    def write_face_tracker(self, value):
      self._widget.textBox_motion_message.append("<font color=\"#00ff00\">"
        + value + "</font>")





######       JUNK - TESTBUTTONS          ##########################################



# HOKUYO STARTEN
    def on_start_hokuyo(self):
      call("/home/rrt/fuerte_workspace/hokuyo_node.sh")


# HOKUYO STARTEN
    def on_test(self):
      tmp = Shell_cmd()
      tmp.Machine = "rrt-cuda2"
      tmp.Command = "ls"
      self._shell_cmd_publisher.publish(tmp)


   
"""
    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass
        
    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass
        
    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass
        
    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure it
        # Usually used to open a configuration dialog
"""        
