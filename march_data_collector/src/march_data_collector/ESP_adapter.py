import os
import sys
import logging
import rospy


try:
    sys.path.append(os.environ["DFESP_HOME"] + '/lib')
    import pubsubApi
    import modelingApi
except ImportError:
    rospy.loginfo("Cannot use ESP on this machine")

import datetime

class ESPAdapter(object):

def __init__(self):
    # joint_names = rospy.get_param('/march/joint_names')
    joint_names = ['left_hip_aa', 'left_hip_fe', 'left_knee', 'left_ankle', 'right_hip_aa',
                   'right_hip_fe', 'right_knee', 'right_ankle']


    ret = pubsubApi.Init(modelingApi.ll_Off, None)
    if ret == 0:
        rospy.logwarn(" Could not initialize pubsub library")
        # destruct
    # below should match with the source windows in the xml file
    sourceWindows = ["sourceWindowJoint"] + ["sourceWindowTemperature_" + joint for joint in joint_names]

    def pubErrCbFunc(failure, code, ctx):
        # don't output anything for client busy
        if failure == pubsubApi.pubsubFail_APIFAIL and code == pubsubApi.pubsubCode_CLIENTEVENTSQUEUED:
            return

        failMsg = pubsubApi.DecodeFailure(failure)
        codeMsg = pubsubApi.DecodeFailureCode(code)
        print('Client services error: ' + failMsg + codeMsg)

    pubErrFunc = pubsubApi.ERRCBFUNC(pubErrCbFunc)



    logger = logging.getLogger()
    logger.addHandler(modelingApi.getLoggingHandler())


    basic_url = 'dfESP://localhost:9901'
    project = basic_url + '/March_test'
    contquery = project + '/March_cq'

    stringv = pubsubApi.QueryMeta(project+"?get=windows_sourceonly")
    sourceWindow_esp = [modelingApi.StringVGet(stringv, i) for i in range(0, modelingApi.StringVSize(stringv))]
    missing_windows = set(sourceWindows)-set(sourceWindow_esp)
    if missing_windows!={}:
        rospy.logwarn("Source windows missing in ESP model: " + str(missing_windows))

    self.esp_publishers = {}
    for source in sourceWindows:
        window_url = contquery + "/" + source
        stringv = pubsubApi.QueryMeta(window_url+"?get=schema")
        if stringv == None:
            rospy.logwarn('Could not get ESP source window schema for window ' +source)
            continue
            # pubsubApi.Shutdown()

        schema = modelingApi.StringVGet(stringv, 0)

        if schema == None:
            rospy.logwarn('Could not get ESP schema from query response for source ' + source)
            continue
        schemaptr = modelingApi.SchemaCreate(source, schema)

        if schemaptr == None:
            rospy.logwarn('Could not build ESP source window schema for source ' + source)
            continue

        pub = pubsubApi.PublisherStart(window_url, pubErrFunc, None)
        if pub == None:
            rospy.logwarn('Could not create ESP publisher client for source' + source)
            continue

        ret = pubsubApi.Connect(pub)
        if ret != 1:
            logger.error('Could not connect ESP publisher client for source ' + source)

            continue

        self.esp_publishers[source] = (pub, schemaptr)

    print(self.esp_publishers)
    modelingApi.StringVFree(stringv)



    pubsubApi.Stop(pub, 0)
    pubsubApi.Shutdown()
    # self._imu_broadcaster = tf2_ros.TransformBroadcaster()
    # self._com_marker_publisher = rospy.Publisher('/march/com_marker', Marker, queue_size=1)
    #
    self._temperature_subscriber = [rospy.Subscriber('/march/temperature/' + joint,
                                                     Temperature,
                                                     self.temperature_callback, joint) for joint in joint_names]

    self._trajectory_state_subscriber = rospy.Subscriber('/march/controller/trajectory/state',
                                                         JointTrajectoryControllerState,
                                                         self.trajectory_state_callback)

    # self._imc_state_subscriber = rospy.Subscriber('/march/imc_states', ImcErrorState, self.imc_state_callback)
    #
    self._imu_subscriber = rospy.Subscriber('/march/imu', Imu, self.imu_callback)



    # pubsubApi.Stop(pub, 1)
    # pubsubApi.Shutdown()

    def send_to_esp(self, csv, source):
        rospy.loginfo(csv)
        csv = 'i, n, 1,' + csv
        pub, schemaptr = self.esp_publishers[source]
        event = modelingApi.EventCreate2(schemaptr, csv, "%Y-%m-%d %H:%M:%S")
        eventVector = modelingApi.EventVCreate()
        modelingApi.EventVPushback(eventVector, event)
        eventBlock = modelingApi.EventBlockNew1(eventVector, modelingApi.ebt_NORMAL)

        ret = pubsubApi.PublisherInject(pub, eventBlock)
        print ("Return value inject = " +str(ret))
        modelingApi.EventBlockDestroy(eventBlock)

    def temperature_callback(self, data, joint):
        # rospy.logwarn("Temperature")
        time = data.header.stamp.secs + data.header.stamp.nsecs * 10**(-9)
        timestr = datetime.datetime.fromtimestamp(time).strftime("%Y-%m-%d %H:%M:%S.%f")
        csv = timestr + ',' + str(data.temperature)
        self.send_to_esp(csv, "sourceWindowTemperature_" + joint)


    def trajectory_state_callback(self, data):
        joint_angles = data.actual.positions
        time = data.header.stamp.secs + data.header.stamp.nsecs * 10**(-9)
        timestr = datetime.datetime.fromtimestamp(time).strftime("%Y-%m-%d %H:%M:%S.%f")
        csv = timestr + ',[' + ";".join([str(value) for value in joint_angles]) + "]"
        self.send_to_esp(csv, "sourceWindowJoint")
        #
        # com = self._com_calculator.calculate_com()
        # self._com_marker_publisher.publish(com)
        # for cp_calculator in self._cp_calculators:
        #     cp_calculator.calculate_cp(com)