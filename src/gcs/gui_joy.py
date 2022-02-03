#!/usr/bin/env python3.6

from time import time
import rospy
from std_msgs.msg import Bool, Int16
from kivymd.app import MDApp
from kivy.lang import Builder
from evdev import InputDevice, list_devices, categorize, ecodes
import threading
# For attachement simulation
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse

class MainApp(MDApp):
    def __init__(self, **kwargs):
        self.title = "GCS by Ju"
        super().__init__(**kwargs)

        self.screen = Builder.load_file('ros_gui.kv')

        #buttons code
        self.codeBtnH = 304
        self.codeBtnX = 305
        self.codeBtnO = 306
        self.codeBtnA = 307
        self.codeBtnL1 = 308
        self.codeBtnR1 = 309
        self.codeBtnL2 = 310
        self.codeBtnR2 = 311
        self.codeBtnShare = 312
        self.codeBtnMenu = 313
        self.codeBtnL3 = 314
        self.codeBtnR3 = 315
        self.codeBtnPS = 316
        self.codeBtnPad = 317
        self.codeBtnMic = 318

        #axis code
        self.codeAxisXL = 0
        self.codeAxisYL = 1
        self.codeAxisXR = 2
        self.codeAxisYR = 5
        self.codeAxisTrigL = 3
        self.codeAxisTrigR = 4
        self.codeArrowX = 16
        self.codeArrowY = 17

        self.valueAxisXL = 127
        self.valueAxisYL = 127
        self.valueAxisXR = 127
        self.valueAxisYR = 127

        self.joy_period = 0.001
        self.tStart = time()
        
        devices = [InputDevice(path) for path in list_devices()]

        for device in devices:
            #print("path: ", device.path)
            #print("name: ", device.name)
            #print("phys: ", device.phys)
            if device.name == "Sony Interactive Entertainment Wireless Controller":
                print("cool")
                js_path = device.path


        #cree un objet gamepad | creates object gamepad
        self.gamepad = InputDevice(js_path)

        #affiche la liste des device connectes | prints out device info at start
        print(self.gamepad)

        self.thread_joy = threading.Thread(target = self.joy_read, name='thread_joy')
        self.thread_joy.start()

        # Detach init
        #rospy.init_node('detach_links')

        rospy.loginfo("Creating ServiceProxy to /link_attacher_node/detach")
        self.detach_srv = rospy.ServiceProxy('/link_attacher_node/detach', Attach)
        self.detach_srv.wait_for_service()
        rospy.loginfo("Created ServiceProxy to /link_attacher_node/detach")

        # Attach init
        #rospy.init_node('demo_attach_links')
        rospy.loginfo("Creating ServiceProxy to /link_attacher_node/attach")
        self.attach_srv = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
        self.attach_srv.wait_for_service()
        rospy.loginfo("Created ServiceProxy to /link_attacher_node/attach")
        
        
        print("INIT Finished")
        
    def build(self):
        self.theme_cls.theme_style = "Dark"
        return self.screen

    def joy_read(self):
    #affiche les codes interceptes |  display codes
        for event in self.gamepad.read_loop():
            #Boutons | buttons 
            if event.type == ecodes.EV_KEY:
                #print("Key Pressed")
                #print(event)
                if event.code == self.codeBtnH:
                    print("square")
            
                    rospy.loginfo("Detaching end-effector and cube3")
                    req = AttachRequest()
                    req.model_name_1 = "hummingbird"
                    req.link_name_1 = "end-effector"
                    req.model_name_2 = "cube3"
                    req.link_name_2 = "link"
                    self.detach_srv.call(req)
                    
                    #print(event.value)
                elif event.code == self.codeBtnX:
                    print("cross")
                    
                    rospy.loginfo("Attaching end-effector and cube3")
                    req = AttachRequest()
                    req.model_name_1 = "hummingbird"
                    req.link_name_1 = "end-effector"
                    req.model_name_2 = "cube3"
                    req.link_name_2 = "link"
                    self.attach_srv.call(req)
                    
                elif event.code == self.codeBtnMic:
                    self.thread_joy.join()
                    break

            #Gamepad analogique | Analog gamepad
            elif event.type == ecodes.EV_ABS:
                if (time() - self.tStart) < self.joy_period:
                    pass
                else:
                    absevent = categorize(event)
                    if absevent.event.code == self.codeAxisXL:
                        #yaw_flu_cmd_pub.publish(absevent.event.value)
                        self.valueAxisXL = absevent.event.value
                        self.root.ids.slider_x_left.value = absevent.event.value
                    elif absevent.event.code == self.codeAxisYL:
                        #z_flu_cmd_pub.publish(abs(absevent.event.value - 255))
                        self.valueAxisYL = abs(absevent.event.value - 255)
                        self.root.ids.slider_y_left.value = abs(absevent.event.value - 255)
                    elif absevent.event.code == self.codeAxisXR:
                        #y_flu_cmd_pub.publish(absevent.event.value)
                        self.valueAxisXR = absevent.event.value
                        self.root.ids.slider_x_right.value = absevent.event.value
                    elif absevent.event.code == self.codeAxisYR:
                        #x_flu_cmd_pub.publish(abs(absevent.event.value - 255))
                        self.valueAxisYR = abs(absevent.event.value - 255)
                        self.root.ids.slider_y_right.value = abs(absevent.event.value - 255)

                    yaw_flu_cmd_pub.publish(self.valueAxisXL)
                    z_flu_cmd_pub.publish(self.valueAxisYL)
                    y_flu_cmd_pub.publish(self.valueAxisXR)
                    x_flu_cmd_pub.publish(self.valueAxisYR)
                    
                    self.tStart = time()
                    
    def closeEvent(self, **kwargs):
        self.thread_joy.join()
        super(MDApp, self).closeEvent(**kwargs)


if __name__ == "__main__":
    # Cmds init
    x_flu_cmd_pub = rospy.Publisher("x_flu_cmd", Int16, queue_size = 1)
    y_flu_cmd_pub = rospy.Publisher("y_flu_cmd", Int16, queue_size = 1)
    z_flu_cmd_pub = rospy.Publisher("z_flu_cmd", Int16, queue_size = 1)
    yaw_flu_cmd_pub = rospy.Publisher("yaw_flu_cmd", Int16, queue_size = 1)

    rospy.init_node('simple_gui', anonymous = True)

    MainApp().run()
