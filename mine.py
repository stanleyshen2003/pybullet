## file created by Stanley for testing

import os, inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0, parentdir)

import pybullet as p
from pybullet_robot_envs.envs.panda_envs.panda_env import pandaEnv
from pybullet_robot_envs.envs.world_envs.world_env import WorldEnv
import argparse


def main(cart_control, random_policy):
    
    id = p.connect(p.GUI)
    p.resetDebugVisualizerCamera(2, 90, -70, [0.0, -0.0, -0.0])
    p.setTimeStep(1 / 240.)
    use_IK = 1 if cart_control else 0
    robot = pandaEnv(id, use_IK=use_IK, joint_action_space=7)
    objects = WorldEnv(id, obj_pose_rnd_std=0.1,workspace_lim=robot.get_workspace())
    #robot.reset()

    motorsIds = []

    if not random_policy:
        if use_IK:
            dv = 1
            motorsIds.append(p.addUserDebugParameter("lhPosX", -dv, dv, 0.0))
            motorsIds.append(p.addUserDebugParameter("lhPosY", -dv, dv, 0.0))
            motorsIds.append(p.addUserDebugParameter("lhPosZ", -dv, dv, 0.0))
            motorsIds.append(p.addUserDebugParameter("lhRollx", -dv, dv, 0.0))
            motorsIds.append(p.addUserDebugParameter("lhPitchy", -dv, dv, 0.0))
            motorsIds.append(p.addUserDebugParameter("lhYawz", -dv, dv, 0.0))
            
        else:
            dv = 1
            joint_idxs = tuple(robot._joint_name_to_ids.values())

            for j in joint_idxs[:robot.joint_action_space]:
                info = p.getJointInfo(robot.robot_id, j)
                jointName = info[1]
                motorsIds.append(p.addUserDebugParameter(jointName.decode("utf-8"), -dv, dv, 0.0))

    done = False

    # ##### add element
    # models = models_data.model_lib()
    # wanted_list = ['bowl','doraemon_bowl','green_bowl','yellow_bowl']
    # position = [[-0.15,-0.15],[-0.15,0.15],[0.15,0.15],[0.15,-0.15]]        
    # flags = p.URDF_USE_INERTIA_FROM_FILE
    # # randomly get a model
    # for i in range(len(wanted_list)):
    #     p.loadURDF(models[wanted_list[i]], [position[i][0], position[i][1], 0.8], flags=flags)
    # ##### end
    
    for t in range(10000000):
        #env.render()
        #goal = env.get_goal_observation()
        #print(goal)
        # if not random_policy:
        #     action = []
        #     for motorId in motorsIds:
        #         action.append(p.readUserDebugParameter(motorId))

        # else:
        #     action = env.action_space.sample()

        p.stepSimulation()
        # state, reward, done, info = env.step(action)
        # print("action: ")
        # print(action)
        # if t % 100000 == 0:
        #     print("reward ", reward)

        # if done:
        #     print("done ", done)
        #     env.reset()


def parser_args():
    """
    parse the arguments for running the experiment
    :return: (dict) the arguments
    """
    parser = argparse.ArgumentParser()

    parser.add_argument('--cartesian', action='store_const', const=1, dest="cart_control",
                        help='action is the cartesian end-effector pose. Default: joint space control')

    parser.add_argument('--random_policy', action='store_const', const=1, dest="random_policy",
                        help="Simulate a random policy instead of using sliders to control the action")

    args = parser.parse_args()
    dict_args = vars(args)
    return dict_args


if __name__ == '__main__':
    args = parser_args()
    print('args')
    print(args)
    main(**args)
