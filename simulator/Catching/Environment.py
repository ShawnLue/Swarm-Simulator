# -*- coding:utf-8 -*-
# __author__ = 'shawnlue'

import numpy as np


class Catching:
    def __init__(self, column, agent_num, max_reborn_per_step=2):
        self.agent_num = agent_num
        self.column = column
        self.max_reborn_per_step = 2

        self.matrix = np.zeros((self.column, self.column))
        self.pos = []

    def reset(self):
        self.matrix = np.zeros((self.column, self.column))
        self.pos = []
        for i in range(self.agent_num):
            i_pos = np.random.randint(0, self.column)
            while i_pos in self.pos:
                i_pos = np.random.randint(0, self.column)
            self.pos.append(i_pos)
            self.matrix[-1][i_pos] = 1
        return self.matrix

    def step(self, action):
        assert len(action) == self.agent_num
        reward_per_agent = []
        total_reward = 0.
        # 1. Update the matrix I: Agent move
        for i in range(self.agent_num):
            if action[i] == -1:
                if self.pos[i] != 0:
                    self.pos[i] -= 1
            elif action[i] == 1:
                if self.pos[i] != self.column - 1:
                    self.pos[i] += 1
        self.matrix[-1, :] = 0
        self.matrix[-1, self.pos] = 1
        # 2. count the reward for next action
        for i in self.pos:
            if self.matrix[-2, i] == 1:
                count = self.pos.count(i)
                reward_per_agent.append(1. / count) # divid up
            else:
                reward_per_agent.append(0.)
        for i in range(self.column):
            if self.matrix[-2, i] == 1:
                if self.matrix[-1, i] == 1:
                    total_reward += 1.
                else:
                    total_reward -= 1.
        # 3. generate new blocks
        new_line = np.array([0] * self.column)
        reborn = np.random.choice(self.max_reborn_per_step, 1)[0]
        if reborn != 0:
            new_line[np.random.choice(self.column, reborn, replace=False)] = 1
        # 4. Update the matrix II: Move down 1 step
        self.matrix = np.insert(np.delete(self.matrix, -2, axis=0), 0, new_line, axis=0)


        # return next_state, total_reward, reward_per_agent
        return self.matrix, total_reward, reward_per_agent


    def render(self):
        pass

if __name__ == '__main__':
    env = Catching(column=10, agent_num=3, max_reborn_per_step=3)
    print env.reset()
    print env.step([0, 0, 0])
    print env.step([0, 0, 0])
    print env.step([0, 0, 0])
