import argparse
import gym
import math
import time
from gym.envs.registration import register
from gym.envs.registration import registry

ENV_NAME = 'A1NavigationEnv-v0'

if ENV_NAME not in registry.env_specs:
    register(
        id=ENV_NAME,
        entry_point='motion_imitation.envs.a1_navigation_env:A1NavigationEnv',
        kwargs={'render': True,
                'print_step_log': True,
                'obs_type': 'pos',
                'velocity': 0.5, 'yaw_rate': 4, 'done_crash': True,
                'env_type': 'flat',
                'get_constraint_cost': False,
                'constraint_cost_binary': False,
                'augment_worst_case': False,}
    )
def current_state(act):
    if act == (0,0) or act == (2,0):
        #rotating = 1
        #not moving = 0
        state = 1 #currently rotating
        return state
    else:
        state = 0 #currently stationary
        return state

def current_counter(state, count):
    if state == 0:
        count = count + 1
        return count
    else:
        count = 0
        return count





def your_new_controller(obs, true_yaw, count):

    # implement your controller here.
    # firt variable: 0, 1, 2 (turning(yaw) rate) 0: turn right, 1: no turn, 2: turn left.
    # second variable: 0, 1 (linear velocity)
    
    #Sets x postion and y postion to variables x and y respectively
    x = obs[0]
    y = obs[1]

    #calculates the distance to goal using x^2 + Y^2 = D^2
    distance_to_goal = math.sqrt((x * x) + (y * y))

    #takes robots current yaw
    #note yaw is 0 when parallel to positive x-axis
    current_angle = true_yaw

    #Calculates the desired angle to origin
    #note q1 and q2 have a rang of 0 to 180
    #note q2 and q4 have a range of 0 to -180
    if obs[0] <= 0:
        desired_angle = math.atan((obs[1]) / (obs[0]))
    else:
        desired_angle = math.atan2((0 - obs[1]),(0 - obs[0]))
    
    #convert desired_angle to degrees
    desired_angle_degrees = math.degrees(desired_angle)

    #convert yaw to degree
    yaw_degrees = math.degrees(current_angle)

    print("desired angle = ", desired_angle_degrees)
    print("current yaw = ", yaw_degrees)
    print("abs difference = ", abs(desired_angle_degrees - yaw_degrees))
    
    #get robot pointed towards the origin
    if(abs(desired_angle_degrees - yaw_degrees) > 5) and count < 40:

        #calcualte δ=(T−C)mod360°−180°
        #if positive turn right, if negative turn left
        if (((desired_angle_degrees - yaw_degrees) % 360) - 180) >= 0:
            act = (0,0)
            return act
        else:
            act = (2,0)
            return act

    if count > 50:
        act = (1,1)
        return act
    else:
        act = (1,0)
        return act
    
   

def run_random(env_name):
    env = gym.make(env_name)
    obs = env.reset()
    done = False
    ep_ret = 0
    ep_cost = 0
    count = 0
    i = 0
    while True:
        # if done:
        #     print('Episode Return: %.3f \t Episode Cost: %.3f'%(ep_ret, ep_cost))
        #     ep_ret, ep_cost = 0, 0
        #     obs = env.reset()
        #assert env.observation_space.contains(obs)
        yaw = env.robot.GetBaseRollPitchYaw()[2]
        # act = env.action_space.sample()
        act = your_new_controller(obs, yaw, count)
        state = current_state(act)
        count = current_counter(state, count)
        assert env.action_space.contains(act)
        obs, reward, done, info = env.step(act)
        print(obs)
        #print("pitch,roll,yaw = ", env.robot.GetBaseRollPitchYaw())
        #print("just yaw in radians = ", yaw)
        #print("real yaw in degrees = ", math.degrees(yaw))
        #time.sleep(0.1)
        

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--env', default='A1NavigationEnv-v0')
    args = parser.parse_args()
    run_random(args.env)
