import smach

from march_state_machine.states.gait_state import GaitState


def create():
    sm_joint_test_left_leg = smach.StateMachine(outcomes=['succeeded', 'preempted', 'failed'])
    with sm_joint_test_left_leg:

        smach.StateMachine.add('LEFT KNEE', GaitState("joint_test_left_leg", "left_knee"),
                               transitions={'succeeded': 'LEFT HIP', 'aborted': 'failed'})

        smach.StateMachine.add('LEFT HIP', GaitState("joint_test_left_leg", "left_hip"),
                               transitions={'succeeded': 'LEFT LEG STRAIGHT', 'aborted': 'failed'})

        smach.StateMachine.add('LEFT LEG STRAIGHT', GaitState("joint_test_left_leg", "left_leg_straight"),
                               transitions={'succeeded': 'LEFT ANKLE', 'aborted': 'failed'})

        smach.StateMachine.add('LEFT ANKLE', GaitState("joint_test_left_leg", "left_ankle"),
                               transitions={'succeeded': 'LEFT HIP AA', 'aborted': 'failed'})

        smach.StateMachine.add('LEFT HIP AA', GaitState("joint_test_left_leg", "left_hip_aa"),
                               transitions={'succeeded': 'succeeded', 'aborted': 'failed'})

    return sm_joint_test_left_leg
