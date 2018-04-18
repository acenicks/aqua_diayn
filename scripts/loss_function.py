import numpy as np

obs_space = 7
action_space = 18

ts = np.array([np.nan, np.nan, 0.0, 5.0, 0.0, 0.0, 0.0]).reshape(obs_space, 1) # target_state
cs = np.array([0.0, 0.0, 0.0, 4.0, 0.0, 0.0, 0.0]).reshape(obs_space, 1) # current_state

c = 1.0 # constant

# Q = np.identity(obs_space)
Q = np.zeros((obs_space, obs_space))

for i in range(ts.shape[0]):
    print(ts[i])
    if not np.isnan(ts[i]):
        Q[i,i] = 1.0

print(Q)

state_diff = np.asarray(cs-ts)

loss = 1 - np.exp(-(1/(2*c))*np.matmul(np.matmul(state_diff.T, Q), state_diff))

print("Loss: " + str(loss[0]))

reward = 1 - loss[0]
print("Reward: " + str(reward))
