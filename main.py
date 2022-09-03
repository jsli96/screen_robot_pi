import time
import socketio
import base64
import pigpio
from picamera import PiCamera
from gpiozero import *
from gpiozero_ext import Motor, PID


# -------------------Here below GPIO are using general gpio library-------
MOTOR_A_P1 = 6  # PWM input for extension motor
MOTOR_A_P2 = 13  # Phase input for extension motor
MOTOR_B_P1 = 5  # PWM input for rotation motor
MOTOR_B_P2 = 12  # Phase input for rotation motor
ROTATION_C1 = 21  # Motor encoder C1
ROTATION_C2 = 20  # Motor encoder C2
ROTATION_VCC = 16  # Encoder power line
IR_1 = 23  # IR Sensor 1
IR_2 = 24  # IR Sensor 2
IR_VCC = 18  # IR Sensor Power line
TOUCH_1_OUT = 27
TOUCH_2_OUT = 22
# --------------------Here below initial function-------------------
camera = PiCamera()
URL_CLOUD = 'https://screen-bot-proj.herokuapp.com/'
IR_SENSOR_1 = DigitalInputDevice(IR_1)  # Set up IR sensor 1
IR_SENSOR_2 = DigitalInputDevice(IR_2)  # Set up IR sensor 2
ENCODER_VCC = DigitalOutputDevice(ROTATION_VCC, initial_value=True)
IR_LED_VCC = DigitalOutputDevice(IR_VCC, initial_value=True)
TOUCH_1 = DigitalOutputDevice(TOUCH_1_OUT, initial_value=False)
TOUCH_2 = DigitalOutputDevice(TOUCH_2_OUT, initial_value=False)
sio = socketio.Client()
sio.connect(URL_CLOUD)
SIO_STATUS = True
POSITION = 0
LENGTH = 0
TARGET = 0
pi = pigpio.pi()
Rotation_Motor = Motor(pi=pi, pwm1=MOTOR_B_P1, pwm2=MOTOR_B_P2, encoder1=ROTATION_C1,
                       encoder2=ROTATION_C2, encoder_ppr=1666)
Extension_Motor = Motor(pwm1=MOTOR_A_P1, pwm2=MOTOR_A_P2)
print("System initialized.")


# ------------------Function------------------------------------
def ir_plus():
    global LENGTH
    LENGTH += 5
    print("Length: ", LENGTH)


def ir_minus():
    global LENGTH
    LENGTH += -5
    print("Length: ", LENGTH)


def close_gpio():
    IR_SENSOR_1.close()
    IR_SENSOR_2.close()
    ENCODER_VCC.close()
    IR_LED_VCC.close()


def send_img(file_path):
    f = open(file_path, "rb")
    img_1 = f.read()  # Read first image
    f.close()
    data = base64.b64encode(img_1)
    if file_path == "img_1.jpg":
        sio.emit('img_data_1', data)
    elif file_path == "img_2.jpg":
        sio.emit('img_data_2', data)
    elif file_path == "img_3.jpg":
        sio.emit('img_data_3', data)
    else:
        print("File name wrong.")


def pid(target):
    global POSITION, Rotation_Motor
    # Setting general parameters
    t_sample = 0.01  # Sampling period (s)
    # Creating PID controller object
    kp = 1
    ki = 0.0
    kd = 0.0001
    tau_pid = 0.01
    motor_pid = PID(t_sample, kp, ki, kd, tau=tau_pid)
    theta_prev = 999
    # Start PID
    print('Start Motor.')
    while True:
        if abs(theta_prev) < 0.1:
            break
        time.sleep(t_sample)  # Pausing for `t_sample` to give CPU time to process encoder signal
        POSITION = Rotation_Motor.get_angle()  # Getting motor shaft angular position
        u_curr = motor_pid.control(target, POSITION)  # Calculating closed-loop output
        Rotation_Motor.set_output(u_curr)  # Assigning motor output
        theta_prev = target - POSITION  # Updating previous values
    print("Done")
    Rotation_Motor.set_output(0, brake=False)


def ir(target):
    global LENGTH, Extension_Motor
    ir_1_prev = IR_SENSOR_1.value
    ir_2_prev = IR_SENSOR_2.value
    t_sample = 0.01
    kp = 1
    ki = 0.0
    kd = 0.0001
    tau_pid = 0.01
    extension_pid = PID(t_sample, kp, ki, kd, u_max=0.3, u_min=-0.3, tau=tau_pid)
    while True:
        ir_1_curr = IR_SENSOR_1.value
        if target == LENGTH:
            break
        elif target > LENGTH and ir_1_prev == 1 and ir_1_curr == 0:
            ir_plus()
        elif target < LENGTH and ir_1_prev == 1 and ir_1_curr == 0:
            ir_minus()
        time.sleep(t_sample)
        u_extension = extension_pid.control(target, LENGTH)
        Extension_Motor.set_output(u_extension)
        ir_1_prev = ir_1_curr
    print("Action done")


# ------------------Socket event--------------------------------
@sio.event
def connect():
    print('my sid is: ' + sio.sid)
    print('connection established')


@sio.event
def connect_error(error):
    print(error)


@sio.event
def disconnect():
    print("disconnected")


@sio.on('request img')
def start_send_img(data):
    print(data)
    return 'OK'


@sio.on('distance')
def reach_target(data):
    global POSITION, LENGTH, TARGET
    print(data)


@sio.on('angle')
def angle(data):
    global TARGET
    print(data)


@sio.on('shut_down')
def shut_down(data):
    global SIO_STATUS
    sio.disconnect()
    SIO_STATUS = False


# -------------------Main code start here----------------------------
sio.emit("This is test in main function", "I am pi.")
print("Start to take pictures")
camera.capture('img_1.jpg')
print("Image captured!")
time.sleep(1)
send_img("img_1.jpg")
pid(30)
time.sleep(0.5)
print("Start to take pictures")
camera.capture('img_2.jpg')
time.sleep(1)
print("Image captured!")
send_img("img_2.jpg")
pid(60)
time.sleep(0.5)
print("Start to take pictures")
camera.capture('img_3.jpg')
time.sleep(1)
print("Image captured!")
send_img("img_3.jpg")
pid(0)
time.sleep(0.5)

# while True:
#     if not SIO_STATUS:
#         print("Shutting done pi.")
#         break


# ------------------End program code---------------------------------
camera.close()
ir(0)
close_gpio()
print("done with script")
del Rotation_Motor
del Extension_Motor

# -------------------Angle test code below----------------------------
# Assigning parameter values
# ppr = 1666  # Pulses Per Revolution of the encoder
# tstop = 20  # Loop execution duration (s)
# tsample = 0.02  # Sampling period for code execution (s)
# tdisp = 0.5  # Sampling period for values display (s)
#
# # Creating encoder object using GPIO pins
# encoder = RotaryEncoder(ROTATION_C1, ROTATION_C2, max_steps=0)
#
# # Initializing previous values and starting main clock
# anglecurr = 0
# tprev = 0
# tcurr = 0
# tstart = time.perf_counter()
#
# # Execution loop that displays the current
# # angular position of the encoder shaft
# print('Running code for', tstop, 'seconds ...')
# print('(Turn the encoder.)')
# while tcurr <= tstop:
#     # Pausing for `tsample` to give CPU time to process encoder signal
#     time.sleep(tsample)
#     # Getting current time (s)
#     tcurr = time.perf_counter() - tstart
#     # Getting angular position of the encoder
#     # roughly every `tsample` seconds (deg.)
#     anglecurr = 360 / ppr * encoder.steps
#     # Printing angular position every `tdisp` seconds
#     if (np.floor(tcurr/tdisp) - np.floor(tprev/tdisp)) == 1:
#         print("Angle = {:0.0f} deg".format(anglecurr))
#     # Updating previous values
#     tprev = tcurr
#
# print('Done.')
# # Releasing GPIO pins
# encoder.close()
