#!/usr/bin/env python3.6

import rospy
from std_msgs.msg import Bool, Int16
from kivymd.app import MDApp
from kivy.lang import Builder
from evdev import InputDevice, list_devices, categorize, ecodes
import threading
#from kivy.garden.cefpython import CEFBrowser

class TutorialApp(MDApp):
    def __init__(self, **kwargs):
        self.title = "My Tutorial Application"
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
        
        print("INIT Finished")
        
    def build(self):
        return self.screen

    def slider_function(self, slider_value):
        print(int(slider_value))

        msg = int(slider_value)
        slider_pub.publish(msg)
    """
    def my_function(self, *args):
        print("button pressed")

        self.screen.ids.my_label.text = 'button pressed'

        msg = True
        pub.publish(msg)
    """

    def joy_read(self):
    #affiche les codes interceptes |  display codes
        for event in self.gamepad.read_loop():
            #Boutons | buttons 
            if event.type == ecodes.EV_KEY:
                #print("Key Pressed")
                #print(event)
                if event.code == self.codeBtnH:
                    print("square")
                    #print(event.value)
                elif event.code == self.codeBtnX:
                    print("cross")
                elif event.code == self.codeBtnMic:
                    self.thread_joy.join()
                    break

            #Gamepad analogique | Analog gamepad
            elif event.type == ecodes.EV_ABS:
                absevent = categorize(event)
                if absevent.event.code == self.codeAxisXL:
                    self.slider_function(absevent.event.value)
                    self.root.ids.myslider.value = absevent.event.value
                    print("Value slider: {}".format((self.root.ids.myslider.value)))
                    
                elif absevent.event.code == self.codeAxisYR:
                    #print("axis left y")
                    #print(absevent.event.value)
                    pass
                elif absevent.event.code == self.codeAxisXL:
                    #print("axis right x")
                    #print(absevent.event.value)
                    pass
                elif absevent.event.code == self.codeAxisYL:
                    #print("axis right y")
                    #print(absevent.event.value)
                    pass
                    
    def closeEvent(self, **kwargs):
        self.thread_joy.join()
        super(MDApp, self).closeEvent(**kwargs)



if __name__ == "__main__":

    #pub = rospy.Publisher("/button", Bool, queue_size = 1)
    slider_pub = rospy.Publisher("slider", Int16, queue_size = 3)

    rospy.init_node('simple_gui', anonymous = True)

    TutorialApp().run()
