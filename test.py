import gymnasium as gym
from sumo.envs.v2i import V2I
import logging

if __name__ == "__main__":
    #env = V2I(render_mode="human", view_size=20)
    env = gym.make('sumo/v2i-v0', view_size=20)
    episodes = 10

    for epsiode in range(0, episodes):

        sum_reward = 0
        obs, obs_len = env.reset()
        #print(obs)
        done = False
        steps = 0
        while not done:
            act = env.action_space.sample()
            #act = 0.0
            #print(act)
            obs, reward, terminated, truncated, info = env.step(act[0])
            #print(obs)
            #print(reward)
            sum_reward += 1
            steps += 1
            if terminated or truncated:
                done = True
            
            if done:
                break
        print("Epsiode: {}. Reward: {}".format(epsiode, sum_reward))
    
    env.close()