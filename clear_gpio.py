import pigpio
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
SIO_STATUS = False
# --------------------Here below initial function-------------------
IR_SENSOR_1 = DigitalInputDevice(IR_1)  # Set up IR sensor 1
IR_SENSOR_2 = DigitalInputDevice(IR_2)  # Set up IR sensor 2
ENCODER_VCC = DigitalOutputDevice(ROTATION_VCC, initial_value=True)
IR_LED_VCC = DigitalOutputDevice(IR_VCC, initial_value=True)
TOUCH_1 = DigitalOutputDevice(TOUCH_1_OUT, initial_value=False)
TOUCH_2 = DigitalOutputDevice(TOUCH_2_OUT, initial_value=False)
pi = pigpio.pi()
Rotation_Motor = Motor(pi=pi, pwm1=MOTOR_B_P1, pwm2=MOTOR_B_P2, encoder1=ROTATION_C1,
                       encoder2=ROTATION_C2, encoder_ppr=1666)
Extension_Motor = Motor(pwm1=MOTOR_A_P1, pwm2=MOTOR_A_P2)
print("System initialized.")


# ------------------Function------------------------------------
def close_gpio():
    IR_SENSOR_1.close()
    IR_SENSOR_2.close()
    ENCODER_VCC.close()
    IR_LED_VCC.close()


# ------------------End program code---------------------------------
print("Turn off all GPIO")
del Rotation_Motor
del Extension_Motor
close_gpio()
