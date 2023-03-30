import cv2 as cv
import urllib.request as ur
import numpy as np
import RPi.GPIO as gpio
from time import sleep
from threading import Thread

gpio.setmode(gpio.BOARD)
net = cv.dnn.readNet("assets/dnn_model/yolov4.weights",
                     "assets/dnn_model/yolov4.cfg")
model = cv.dnn_DetectionModel(net)
model.setInputParams(size=(832, 832), scale=1 / 255)
vehicle_count = []
cv.namedWindow("live transmision", cv.WINDOW_AUTOSIZE)
url = 'http://192.168.8.101'  # add the esp32cam url
rotation_direction = 1
tim = [20, 50, 60, 30]
display_speed = 200
clock_speed = 363
max_time1_3 = max([tim[2]-(tim[2]//3), tim[0]-(tim[0]//3)])
max_time2_4 = max([tim[1]-(tim[1]//3), tim[3]-(tim[3]//3)])
sleep_val = 0.001
yellow_time = 1
rev_seq = [[0, 1, 1, 0, 1, 1, 1, 1],  # 9
           [0, 1, 1, 1, 1, 1, 1, 1],  # 8
           [0, 0, 0, 0, 0, 1, 1, 1],  # 7
           [0, 1, 1, 1, 1, 1, 0, 1],  # 6
           [0, 1, 1, 0, 1, 1, 0, 1],  # 5
           [0, 1, 1, 0, 0, 1, 1, 0],  # 4
           [0, 1, 0, 0, 1, 1, 1, 1],  # 3
           [0, 1, 0, 1, 1, 0, 1, 1],  # 2
           [0, 0, 0, 0, 0, 1, 1, 0],  # 1
           [0, 0, 1, 1, 1, 1, 1, 1],  # 0
           ]


"""
pi74hc595 class for handle the shift registers
"""
class pi74HC595:
    def __init__(
        self, DS: int = 7, ST: int = 22, SH: int = 37, daisy_chain: int = 2,
    ):

        if not (isinstance(DS, int) or isinstance(ST, int) or isinstance(SH, int)):
            raise ValueError("Pins must be int")
        elif DS < 1 or DS > 40 or ST < 1 or ST > 40 or SH < 1 or SH > 40:
            raise ValueError("Pins (DS, ST, SH) must be within pin range")

        if not isinstance(daisy_chain, int):
            raise ValueError("daisy_chain must be int")
        elif daisy_chain < 1:
            raise ValueError("daisy_chain must be positive")

        self.data = DS  # DS
        self.parallel = ST  # ST_CP
        self.serial = SH  # SH_CP
        self.daisy_chain = daisy_chain  # Number of 74HC595s
        self.current = [0, 0, 0, 0, 0, 0, 0, 0] * self.daisy_chain
        self._setup_board()
        self.clear()

    def _setup_board(self):
        gpio.setup(self.data, gpio.OUT)
        gpio.output(self.data, gpio.LOW)
        gpio.setup(self.parallel, gpio.OUT)
        gpio.output(self.parallel, gpio.LOW)
        gpio.setup(self.serial, gpio.OUT)
        gpio.output(self.serial, gpio.LOW)

    def _output(self):  # ST_CP
        gpio.output(self.parallel, gpio.HIGH)
        gpio.output(self.parallel, gpio.LOW)

    def _tick(self):  # SH_CP
        gpio.output(self.serial, gpio.HIGH)
        gpio.output(self.serial, gpio.LOW)

    def _set_values(self, values):
        for bit in values:
            self.current.append(bit)
            del self.current[0]
            if bit == 1:
                gpio.output(self.data, gpio.HIGH)
            elif bit == 0:
                gpio.output(self.data, gpio.LOW)
            self._tick()
        self._output()

    def set_ds(self, pin: int):
        """
        Sets the pin for the serial data input (DS)

        Returns: None

        """
        if not isinstance(pin, int):
            raise ValueError("Argument must be int")
        elif pin < 1 or pin > 40:
            raise ValueError("Argument must be within pin range")

        self.data = DS

    def set_sh(self, pin: int):
        """
        Sets the pin for the shift register clock pin (SH_CP)

        Returns: None

        """
        if not isinstance(pin, int):
            raise ValueError("Argument must be int")
        elif pin < 1 or pin > 40:
            raise ValueError("Argument must be within pin range")

        self.parallel = DS

    def set_st(self, pin: int):
        """
        Sets the pin for the storage register clock pin (ST_CP)

        Returns: None

        """
        if not isinstance(pin, int):
            raise ValueError("Argument must be int")
        elif pin < 1 or pin > 40:
            raise ValueError("Argument must be within pin range")

        self.serial = ST

    def set_daisy_chain(self, num: int):
        """
        Sets the the number of 74HC595s used in Daisy Chain

        Returns: None

        """
        if not isinstance(num, int):
            raise ValueError("Argument must be int")
        elif num < 1:
            raise ValueError("Argument must be positive")

        self.daisy_chain = num

    def set_by_list(self, values):
        """
        Sends values in list to the internal function

        args: [0, 1, 0,...]
            or
            [False, True, False,...]

        Returns: None

        """
        if not isinstance(values, list):
            raise ValueError("Argument must be a list")

        for i in range(len(values)):
            if values[i] == True:
                values[i] = 1
            elif values[i] == False:
                values[i] = 0
            else:
                raise ValueError("Values within list must be 1, 0, or boolean")
        self._set_values(values)

    def clear(self):
        """
        Sets the 74HC595 back to all off

        Returns: None

        """
        self._set_values([0, 0, 0, 0, 0, 0, 0, 0] * self.daisy_chain)


# object for handle 2 digit 7 segment displays shift registers 7,22,37
shift_register = pi74HC595()
# object for handle LEDS shift registers (data,latch,clock)
shift_register_light = pi74HC595(11, 24, 35, 2)

"""
function created for control the rotation of stepper motor
"""
def stepper(rotation_direction):
    controlPin = [5, 11, 13, 15]

    for pin in controlPin:
        gpio.setup(pin, gpio.OUT)
        gpio.output(pin, 0)

    seq = [[1, 0, 0, 0], [1, 1, 0, 0], [0, 1, 0, 0], [0, 1, 1, 0], [0, 0, 1, 0], [0, 0, 1, 1],
           [0, 0, 0, 1], [1, 0, 0, 1]]

    if rotation_direction == 1:
        for i in range(128):
            for halfstep in range(8):
                for pin in range(4):
                    gpio.output(controlPin[pin], seq[halfstep][pin])
                sleep(0.001)
    else:
        for i in range(128):
            for halfstep in range(-1, -9, -1):
                for pin in range(4):
                    gpio.output(controlPin[pin], seq[halfstep][pin])
                sleep(0.001)

"""
function created for get the vehicle count from a image
"""
def get_vehicle_count(img_detect):
    # Detect Objects
    vehicles_boxes = []
    global vehicle_count
    class_ids, scores, boxes = model.detect(img_detect, nmsThreshold=0.4)
    for class_id, score, box in zip(class_ids, scores, boxes):
        if score < 0.5:
            # Skip detection with low confidence
            continue
        # if class_id in classes_allowed:
        vehicles_boxes.append(box)
    vehicle_count.append(len(vehicles_boxes))
    for box in vehicles_boxes:
        x, y, w, h = box
        cv.rectangle(img_detect, (x, y), (x + w, y + h), (25, 0, 180), 3)
    cv.imshow("Cars", img_detect)
    cv.waitKey(0)


"""
function created for get the esp32 camera feed and send it to get_vehicle_count function to count the 
vehicle
"""
def get_esp32_feed():
    img_resp = ur.urlopen(url)
    img_np = np.array(bytearray(img_resp.read()), dtype=np.uint8)
    frame = cv.imcode(img_np, -1)
    cv.imshow("live transmission", frame)
    get_vehicle_count(frame)
    cv.waitKey(0)
    cv.destroyWindow()


"""
function created for allocate the time for each route according to vehicle count
"""
def allocate_time():
    global vehicle_count
    for count in vehicle_count:
        if count > 30:
            tim.append(90)
        if 30 > count > 20:
            tim.append(70)
        if 20 > count > 10:
            tim.append(50)
        if 10 > count:
            tim.append(30)

"""
function created for control 3rd route color lights
"""
def route_light_three():
    for i in range(yellow_time*clock_speed):  # yellow
        shift_register_light.set_by_list(
            [0, 0, 0, 0, 0, 1, 0, 0]+[1, 1, 1, 1, 1, 0, 1, 1])
        sleep(sleep_val)
    for i in range((tim[2]//3)*clock_speed):  # dual green
        shift_register_light.set_by_list(
            [0, 0, 0, 0, 0, 1, 0, 0]+[1, 1, 1, 0, 0, 1, 1, 1])
        sleep(sleep_val)
    for i in range(max_time1_3*clock_speed):  # single green
        shift_register_light.set_by_list(
            [0, 0, 0, 0, 0, 1, 0, 0]+[1, 1, 0, 1, 0, 1, 1, 1])
        sleep(sleep_val)
    for i in range(((tim[2]//3)+tim[1]+tim[3])*clock_speed):  # red
        shift_register_light.set_by_list(
            [0, 0, 0, 0, 0, 1, 0, 0]+[1, 1, 0, 1, 1, 1, 0, 1])
        sleep(sleep_val)

"""
function created for control 1st route color lights
"""
def route_light_one():
    for i in range((tim[2]//3)*clock_speed):  # red
        shift_register_light.set_by_list(
            [0, 0, 0, 1, 0, 0, 0, 0]+[1, 1, 0, 1, 1, 1, 0, 1])
        sleep(sleep_val)
    for i in range(yellow_time*clock_speed):  # yellow
        shift_register_light.set_by_list(
            [0, 0, 0, 1, 0, 0, 0, 0]+[1, 1, 1, 1, 1, 0, 1, 1])
        sleep(sleep_val)
    for i in range(max_time1_3*clock_speed):  # single green
        shift_register_light.set_by_list(
            [0, 0, 0, 1, 0, 0, 0, 0]+[1, 1, 0, 1, 0, 1, 1, 1])
        sleep(sleep_val)
    for i in range((tim[0]//3)*clock_speed):  # dual green
        shift_register_light.set_by_list(
            [0, 0, 0, 1, 0, 0, 0, 0]+[1, 1, 1, 0, 0, 1, 1, 1])
        sleep(sleep_val)
    for i in range((tim[1]+tim[3])*clock_speed):  # red
        shift_register_light.set_by_list(
            [0, 0, 0, 1, 0, 0, 0, 0]+[1, 1, 0, 1, 1, 1, 0, 1])
        sleep(sleep_val)

"""
function created for control 4th route color lights
"""
def route_light_four():
    prev_time = (tim[2]//3)+(tim[0]//3)+max_time1_3
    for i in range(prev_time*clock_speed):  # red
        shift_register_light.set_by_list(
            [0, 0, 0, 0, 0, 0, 1, 0]+[1, 1, 0, 1, 1, 1, 0, 1])
        sleep(sleep_val)
    for i in range(yellow_time*clock_speed):  # yellow
        shift_register_light.set_by_list(
            [0, 0, 0, 0, 0, 0, 1, 0]+[1, 1, 1, 1, 1, 0, 1, 1])
        sleep(sleep_val)
    for i in range((tim[3]//3)*clock_speed):  # dual green
        shift_register_light.set_by_list(
            [0, 0, 0, 0, 0, 0, 1, 0]+[1, 1, 1, 0, 0, 1, 1, 1])
        sleep(sleep_val)
    for i in range(max_time2_4*clock_speed):  # single green
        shift_register_light.set_by_list(
            [0, 0, 0, 0, 0, 0, 1, 0]+[1, 1, 0, 1, 0, 1, 1, 1])
        sleep(sleep_val)
    for i in range((tim[1]//3)*clock_speed):  # red
        shift_register_light.set_by_list(
            [0, 0, 0, 0, 0, 0, 1, 0]+[1, 1, 0, 1, 1, 1, 0, 1])
        sleep(sleep_val)

"""
function created for control 2nd route color lights
"""
def route_light_two():
    prev_time = (tim[2]//3)+(tim[0]//3)+max_time1_3+(tim[3]//3)
    for i in range(prev_time*clock_speed):  # red
        shift_register_light.set_by_list(
            [0, 0, 0, 0, 1, 0, 0, 0]+[1, 1, 0, 1, 1, 1, 0, 1])
        sleep(sleep_val)
    for i in range(yellow_time*clock_speed):  # yellow
        shift_register_light.set_by_list(
            [0, 0, 0, 0, 1, 0, 0, 0]+[1, 1, 1, 1, 1, 0, 1, 1])
        sleep(sleep_val)
    for i in range(max_time2_4*clock_speed):  # single green
        shift_register_light.set_by_list(
            [0, 0, 0, 0, 1, 0, 0, 0]+[1, 1, 0, 1, 0, 1, 1, 1])
        sleep(sleep_val)
    for i in range((tim[1]//3)*clock_speed):  # dual green
        shift_register_light.set_by_list(
            [0, 0, 0, 0, 1, 0, 0, 0]+[1, 1, 1, 0, 0, 1, 1, 1])
        sleep(sleep_val)

"""
function created for control 3rd route displays
"""
def display_3():
    for i in range((10-((tim[2]//3+max_time1_3)//10)), 10):
        for j in range(10):
            for k in range(display_speed):
                shift_register.set_by_list(rev_seq[i]+[1, 1, 1, 1, 1, 0, 1, 1])
                sleep(sleep_val)
                shift_register.set_by_list(rev_seq[j]+[1, 1, 1, 1, 0, 1, 1, 1])
                sleep(sleep_val)

"""
function created for control 1st route displays
"""
def display_1():
    sleep(tim[2]//3)
    for i in range((10-((tim[0]//3+max_time1_3)//10)), 10):
        for j in range(10):
            for k in range(display_speed):
                shift_register.set_by_list(rev_seq[i]+[1, 0, 1, 1, 1, 1, 1, 1])
                sleep(sleep_val)
                shift_register.set_by_list(rev_seq[j]+[0, 1, 1, 1, 1, 1, 1, 1])
                sleep(sleep_val)

"""
function created for control 4th route displays
"""
def display_4():
    sleep((tim[2]//3)+max_time1_3+(tim[0]//3))
    for i in range((10-((tim[3]//3+max_time2_4)//10)), 10):
        for j in range(10):
            for k in range(display_speed):
                shift_register.set_by_list(rev_seq[i]+[1, 1, 1, 0, 1, 1, 1, 1])
                sleep(sleep_val)
                shift_register.set_by_list(rev_seq[j]+[1, 1, 0, 1, 1, 1, 1, 1])
                sleep(sleep_val)

"""
function created for control 2nd route displays
"""
def display_2():
    sleep((tim[2]//3)+max_time1_3+(tim[0]//3)+(tim[3]//3))
    for i in range((10-((tim[1]//3+max_time2_4)//10)), 10):
        for j in range(10):
            for k in range(display_speed):
                shift_register.set_by_list(rev_seq[i]+[1, 1, 1, 1, 1, 1, 1, 0])
                sleep(sleep_val)
                shift_register.set_by_list(rev_seq[j]+[1, 1, 1, 1, 1, 1, 0, 1])
                sleep(sleep_val)

"""
function created for clear all shift register values
"""
def clear():
    shift_register.set_by_list([1, 1, 1, 1, 1, 1, 1, 1]*2)
    shift_register_light.set_by_list([0, 0, 0, 0, 0, 0, 0, 0]*2)


def main_func():
    """img = cv.imread('assets/img_1.png')- for testing purpose
    get_vehicle_count(img)"""
    global rotation_direction
    """
    vehicle_count_thread = Thread(target=get_esp32_feed)
    vehicle_count_thread.start()
    vehicle_count_thread.join()"""
    for i in range(4):
        stepper(rotation_direction)
        rotation_direction = -1 if rotation_direction == 1 else 1
        get_esp32_feed()
    allocate_time()
    clear()
    thread1 = Thread(target=display_1)
    thread2 = Thread(target=display_2)
    thread3 = Thread(target=display_3)
    thread4 = Thread(target=display_4)
    thread5 = Thread(target=route_light_three)
    thread6 = Thread(target=route_light_one)
    thread7 = Thread(target=route_light_four)
    thread8 = Thread(target=route_light_two)

    thread1.start()
    thread2.start()
    thread3.start()
    thread4.start()
    thread5.start()
    thread6.start()
    thread7.start()
    thread8.start()

    thread1.join()
    thread2.join()
    thread3.join()
    thread4.join()
    thread5.join()
    thread6.join()
    thread7.join()
    thread8.join()
    clear()


if __name__ == "__main__":
    main_func()
