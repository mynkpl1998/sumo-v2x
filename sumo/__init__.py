from gymnasium.envs.registration import register

register(
    id="sumo/v2i-v0",
    entry_point="sumo.envs:V2I",
)