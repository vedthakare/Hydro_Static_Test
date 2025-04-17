#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32
import RPi.GPIO as GPIO

# Servo configuration
SERVO_PINS = [12, 13]     # GPIO 12 and 13 (BCM numbering)
FREQ = 80

SERVO_MIN_ANGLE = 0
SERVO_MAX_ANGLE = 270
PULSE_MIN_MS = 0.5
PULSE_MAX_MS = 2.0
PWM_PERIOD_MS = 1000.0 / FREQ

SERVO_MIN_DUTY = (PULSE_MIN_MS / PWM_PERIOD_MS) * 100.0
SERVO_MAX_DUTY = (PULSE_MAX_MS / PWM_PERIOD_MS) * 100.0

DEAD_BAND_US = 2
DEAD_BAND = (DEAD_BAND_US / (PWM_PERIOD_MS * 1000)) * 100

SMOOTHING_FACTOR = 0.2
MIN_ANGLE_CHANGE = 1
UPDATE_RATE = 40

class SingleServo:
	def __init__(self, pin):
		self.pin = pin
		self.current_angle = 0
		self.target_angle = 0
		self.last_duty = 0

		GPIO.setup(pin, GPIO.OUT)
		self.pwm = GPIO.PWM(pin, FREQ)
		self.pwm.start(0)

	def angle_to_duty_cycle(self, angle):
		angle = max(SERVO_MIN_ANGLE, min(SERVO_MAX_ANGLE, angle))
		angle_range = SERVO_MAX_ANGLE - SERVO_MIN_ANGLE
		duty_range = SERVO_MAX_DUTY - SERVO_MIN_DUTY
		return SERVO_MIN_DUTY + ((angle - SERVO_MIN_ANGLE) / angle_range) * duty_range

	def set_target_angle(self, angle):
		self.target_angle = angle
		rospy.logdebug(f"Target angle for pin {self.pin}: {angle:.1f}°")

	def update(self):
		if abs(self.target_angle - self.current_angle) < MIN_ANGLE_CHANGE:
			return

		self.current_angle = (SMOOTHING_FACTOR * self.target_angle +
		                      (1 - SMOOTHING_FACTOR) * self.current_angle)

		duty = self.angle_to_duty_cycle(self.current_angle)
		if abs(duty - self.last_duty) > DEAD_BAND:
			self.pwm.ChangeDutyCycle(duty)
			self.last_duty = duty
			rospy.logdebug(f"Pin {self.pin}: angle {self.current_angle:.1f}° → duty {duty:.2f}%")

	def stop(self):
		self.pwm.stop()

class ServoController:
	def __init__(self):
		GPIO.setmode(GPIO.BCM)
		self.servos = [SingleServo(pin) for pin in SERVO_PINS]

	def angle_callback_1(self, msg):
		self.servos[0].set_target_angle(msg.data)

	def angle_callback_2(self, msg):
		self.servos[1].set_target_angle(msg.data)

	def update_all(self, event=None):
		for servo in self.servos:
			servo.update()

	def shutdown(self):
		for servo in self.servos:
			servo.stop()
		GPIO.cleanup()
		rospy.loginfo("PWM stopped, GPIO cleaned.")

def main():
	rospy.init_node('dual_servo_control')
	controller = ServoController()
	
	rospy.Subscriber('servo_angle', Float32, controller.angle_callback_1, queue_size=1)
	rospy.Subscriber('servo_angle_2', Float32, controller.angle_callback_2, queue_size=1)

	timer = rospy.Timer(rospy.Duration(1.0/UPDATE_RATE), controller.update_all)
	rospy.on_shutdown(controller.shutdown)

	rospy.loginfo(f"Dual servo controller running at {UPDATE_RATE}Hz")
	rospy.loginfo("Listening on /servo_angle and /servo_angle_2")

	try:
		rospy.spin()
	except rospy.ROSInterruptException:
		pass

if __name__ == '__main__':
	main()
