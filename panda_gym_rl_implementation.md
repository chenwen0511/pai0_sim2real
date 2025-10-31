---
AIGC: {"Label": "1", "ContentProducer": "001191330101MA27WPYJ18xliu", "ProduceID": "2afa31f3-7701-4c23-a666-dfa51bea2369", "ReserveCode1": "iflow", "ContentPropagator": "iflow", "PropagateID": "iflow", "ReserveCode2": "iflow"}
---

# Panda-Gym Reinforcement Learning Implementation

## Project Overview
panda-gym is an open-source project providing RL environments for Franka Panda robot based on PyBullet physics engine. Key features:

1. **Supported Environments**:
   - PandaReach-v0: End-effector positioning
   - PandaPush-v0: Object pushing
   - PandaSlide-v0: Object sliding
   - PandaPickAndPlace-v0: Grasp and place

2. **Technical Specifications**:
   - Physics engine: PyBullet (240Hz simulation)
   - Observation space: 19 dimensions (joint states + object info)
   - Action space: 4 dimensions (x,y,z + gripper)

## Core Code Structure
```python
import gym
import panda_gym

env = gym.make("PandaPickAndPlace-v0")
obs = env.reset()

for _ in range(1000):
    action = policy(obs)  # Your RL policy
    obs, reward, done, info = env.step(action)
    if done:
        obs = env.reset()
```
【78†】

## Integration with π0 VLA
To combine π0 VLA with panda-gym:

1. **Observation Wrapper**:
```python
class Pi0Wrapper(gym.ObservationWrapper):
    def __init__(self, env):
        super().__init__(env)
        self.vision_model = load_pi0_vision()
        
    def observation(self, obs):
        img = env.render(mode='rgb_array')
        visual_feats = self.vision_model(img)
        return np.concatenate([obs, visual_feats])
```

2. **Action Transformation**:
```python
def pi0_action_to_panda(pi0_action):
    # Convert π0's continuous flow to Panda joint commands
    return pi0_action[:4]  # x,y,z + gripper
```
【78†】

## Training Pipeline
1. **Setup**:
```bash
git clone https://github.com/qgallouedec/panda-gym
pip install -e .
```

2. **Training Example**:
```python
from stable_baselines3 import HER, DDPG

model = HER(
    policy="MultiInputPolicy",
    env=env,
    model_class=DDPG,
    verbose=1
)
model.learn(total_timesteps=100000)
```

## Performance Benchmarks
| Environment | Success Rate (PPO) | Success Rate (SAC) |
|-------------|-------------------|-------------------|
| Reach       | 98%               | 99%               |
| Push        | 85%               | 88%               |
| PickAndPlace| 72%               | 75%               |
【78†】