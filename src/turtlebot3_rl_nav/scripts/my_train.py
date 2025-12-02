import rospy
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3.common.monitor import Monitor
from stable_baselines3 import PPO, DQN, A2C, SAC, TD3, DDPG
from my_env import myEnv
import utils
import os


def make_env():
    env = myEnv()
    env = Monitor(env)
    return env

if __name__ == "__main__":
    rospy.init_node("my_train_node")

    PATH_CONFIG = utils.setup_log_and_model_paths('PPO')
    LOG_DIR = PATH_CONFIG["log_dir"]
    MODEL_DIR = PATH_CONFIG["model_dir"]
    LOG_NAME = PATH_CONFIG["log_name"]
    MODEL_NAME = PATH_CONFIG["model_name"]

    vec_env = DummyVecEnv([make_env])

    model = PPO(
        "MlpPolicy",
        vec_env,
        learning_rate=1e-4,
        n_steps=4096,
        batch_size=64,
        n_epochs=10,
        gamma=0.99,
        gae_lambda=0.95,
        clip_range=0.2,
        ent_coef=0.0,
        vf_coef=0.5,
        device="cpu",
        tensorboard_log=LOG_DIR,
        verbose=1,
    )

    model.learn(
        total_timesteps=1_000_000,
        tb_log_name=LOG_NAME,
        progress_bar=True,
    )


    final_model_path = os.path.join(MODEL_DIR, MODEL_NAME)
    model.save(final_model_path)
    
    rospy.loginfo(f"Training finished. Final model saved to: {final_model_path}.zip")