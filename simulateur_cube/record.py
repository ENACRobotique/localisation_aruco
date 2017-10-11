

from pymorse import Morse

def video_image_callback(data):
	print("got some think")

with Morse() as simu:

	motion = simu.robot.motion
	pose = simu.robot.pose
	video = simu.robot2.videocamera.subscribe(video_image_callback)

print("ok")
