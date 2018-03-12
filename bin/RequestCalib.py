import rospy
from cube_pos.srv import *


def request_calib_client(nb_cam):
    rospy.wait_for_service('CalibrateReporter')
    try:
        calib = rospy.ServiceProxy('CalibrateReporter', CalibRequest)
        #resp1 = calib(nb_cam)
        return calib(nb_cam)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


def calib(nb_cam):
    print "Requesting %s calib"%(nb_cam)
    res=request_calib_client(nb_cam);
    print "Effective calib %s | real nb=%s"%("OK" if res.done else "FAIL",
																res.tot_cam	 )
