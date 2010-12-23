#!/usr/bin/python
import roslib; roslib.load_manifest('irobot_create_2_1')
import rospy
from time import sleep
from irobot import Create
from threading import Thread
from math import sin,cos,pi
from datetime import datetime

from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster

from irobot_create_2_1.msg import SensorPacket
from irobot_create_2_1.srv import *

class CreateDriver:
	def __init__(self):
		port = rospy.get_param('/brown/irobot_create_2_1/port', "/dev/ttyUSB0")
		self.create = Create(port)
		self.packetPub = rospy.Publisher('sensorPacket', SensorPacket)
		self.odomPub = rospy.Publisher('odom',Odometry)
		self.odomBroadcaster = TransformBroadcaster()
		self.fields = ['wheeldropCaster','wheeldropLeft','wheeldropRight','bumpLeft','bumpRight','wall','cliffLeft','cliffFronLeft','cliffFrontRight','cliffRight','virtualWall','infraredByte','advance','play','distance','angle','chargingState','voltage','current','batteryTemperature','batteryCharge','batteryCapacity','wallSignal','cliffLeftSignal','cliffFrontLeftSignal','cliffFrontRightSignal','cliffRightSignal','homeBase','internalCharger','songNumber','songPlaying']
		self.then = datetime.now()
		self.x = 0
		self.y = 0
		self.th = 0
		self.idealTh = 0
		self.create.update = self.sense

	def start(self,req = True):
		if (req == True or req.start):
			self.create.start()
			self.then = datetime.now()
		return StartResponse(True)

	def stop(self,req = True):
		if (req == True or req.stop):
			self.create.stop()
		return StopResponse(True)

	def sense(self):
		now = datetime.now()
		elapsed = now - self.then
		self.then = now
		elapsed = float(elapsed.seconds) + elapsed.microseconds/1000000.
		d = self.create.d_distance / 1000.
		th = self.create.d_angle*pi/180
		dx = d / elapsed
		dth = th / elapsed

		if (abs(self.idealTh) < .5):
			dth = self.idealTh

		if (d != 0):
			x = cos(th)*d
			y = -sin(th)*d
			self.x = self.x + (cos(self.th)*x - sin(self.th)*y)
			self.y = self.y + (sin(self.th)*x + cos(self.th)*y)

		if (th != 0):
			self.th = self.th + th

		quaternion = Quaternion()
		quaternion.x = 0.0
		quaternion.y = 0.0
		quaternion.z = sin(self.th/2)
		quaternion.w = cos(self.th/2)

		self.odomBroadcaster.sendTransform(
			(self.x, self.y, 0),
			(quaternion.x, quaternion.y, quaternion.z, quaternion.w),
			rospy.Time.now(),
			"base_link",
			"odom"
			)

		odom = Odometry()
		odom.header.stamp = rospy.Time.now()
		odom.header.frame_id = "odom"
		odom.pose.pose.position.x = self.x
		odom.pose.pose.position.y = self.y
		odom.pose.pose.position.z = 0
		odom.pose.pose.orientation = quaternion

		odom.child_frame_id = "base_link"
		odom.twist.twist.linear.x = dx
		odom.twist.twist.linear.y = 0
		odom.twist.twist.angular.z = dth

		self.odomPub.publish(odom)

		packet = SensorPacket()
		for field in self.fields:
			packet.__setattr__(field,self.create.__getattr__(field))
		self.packetPub.publish(packet)

	def brake(self,req):
		if (req.brake):
			self.create.brake()
		return BrakeResponse(True)

	def reset(self,req):
		if (req.reset):
			self.create.reset()
		return ResetResponse(True)

	def circle(self,req):
		if (req.clear):
			self.create.clear()
		self.create.forwardTurn(req.speed,req.radius)
		return CircleResponse(True)

	def demo(self,req):
		self.create.demo(req.demo)
		return DemoResponse(True)

	def leds(self,req):
		self.create.leds(req.advance,req.play,req.color,req.intensity)
		return LedsResponse(True)

	def tank(self,req):
		if (req.clear):
			self.create.clear()
		self.create.tank(req.left,req.right)
		return TankResponse(True)

	def turn(self,req):
		if (req.clear):
			self.create.clear()
		self.create.turn(req.turn)
		return TurnResponse(True)

	def twist(self,req):
		x = req.linear.x * 1000. # ie, linear speed passed in as m/s, internally mm/s
		th = req.angular.z # angular speed in rad/s
		if (th == 0):
			x = int(x)
			self.create.tank(x,x)
		else:
			distanceBetweenWheels = 260 # mm (CONSTANT)
			wheelDistanceFromCenter = distanceBetweenWheels / 2
			turnRadius = x/th # from perspective of robot center
			lwSpeed=int((turnRadius-wheelDistanceFromCenter)*th)
			rwSpeed=int((turnRadius+wheelDistanceFromCenter)*th)
			self.create.tank(lwSpeed,rwSpeed)


if __name__ == '__main__':
	node = rospy.init_node('create')
	driver = CreateDriver()

	rospy.Service('brake',Brake,driver.brake)
	rospy.Service('circle',Circle,driver.circle)
	rospy.Service('demo',Demo,driver.demo)
	rospy.Service('leds',Leds,driver.leds)
	rospy.Service('tank',Tank,driver.tank)
	rospy.Service('turn',Turn,driver.turn)
	rospy.Service('reset',Reset,driver.reset)
	rospy.Service('stop',Stop,driver.stop)
	rospy.Service('start',Start,driver.start)

	rospy.Subscriber("cmd_vel", Twist, driver.twist)

	sleep(1)
	driver.start()
	sleep(1)

	rospy.spin()
	driver.stop()
