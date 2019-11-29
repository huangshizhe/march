import smach

from march_state_machine.states.gait_state import GaitState


def create():
    sm_joint_test_both_legs = smach.StateMachine(outcomes=['succeeded', 'preempted', 'failed'])
    with sm_joint_test_both_legs:

        smach.StateMachine.add('BOTH KNEES', GaitState("joint_test_both_legs", "both_knees"),
                               transitions={'succeeded': 'BOTH HIPS', 'aborted': 'failed'})

        smach.StateMachine.add('BOTH HIPS', GaitState("joint_test_both_legs", "both_hips"),
                               transitions={'succeeded': 'BOTH LEGS STRAIGHT', 'aborted': 'failed'})

        smach.StateMachine.add('BOTH LEGS STRAIGHT', GaitState("joint_test_both_legs", "both_legs_straight"),
                               transitions={'succeeded': 'BOTH ANKLES', 'aborted': 'failed'})

        smach.StateMachine.add('BOTH ANKLES', GaitState("joint_test_both_legs", "both_ankles"),
                               transitions={'succeeded': 'BOTH HIPS AA', 'aborted': 'failed'})

        smach.StateMachine.add('BOTH HIPS AA', GaitState("joint_test_both_legs", "both_hips_aa"),
                               transitions={'succeeded': 'succeeded', 'aborted': 'failed'})

    return sm_joint_test_both_legs
