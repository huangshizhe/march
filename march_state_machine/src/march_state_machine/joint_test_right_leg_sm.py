import smach

from march_state_machine.states.gait_state import GaitState


def create():
    sm_joint_test_right_leg = smach.StateMachine(outcomes=['succeeded', 'preempted', 'failed'])
    with sm_joint_test_right_leg:

        smach.StateMachine.add('RIGHT KNEE', GaitState("joint_test_right_leg", "right_knee"),
                               transitions={'succeeded': 'RIGHT HIP', 'aborted': 'failed'})

        smach.StateMachine.add('RIGHT HIP', GaitState("joint_test_right_leg", "right_hip"),
                               transitions={'succeeded': 'RIGHT LEG STRAIGHT', 'aborted': 'failed'})

        smach.StateMachine.add('RIGHT LEG STRAIGHT', GaitState("joint_test_right_leg", "right_leg_straight"),
                               transitions={'succeeded': 'RIGHT ANKLE', 'aborted': 'failed'})

        smach.StateMachine.add('RIGHT ANKLE', GaitState("joint_test_right_leg", "right_ankle"),
                               transitions={'succeeded': 'RIGHT HIP AA', 'aborted': 'failed'})

        smach.StateMachine.add('RIGHT HIP AA', GaitState("joint_test_right_leg", "right_hip_aa"),
                               transitions={'succeeded': 'succeeded', 'aborted': 'failed'})

    return sm_joint_test_right_leg
