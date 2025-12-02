# scripts/utils.py

import os
from datetime import datetime

# 获取当前脚本所在目录 (scripts/)
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
# 获取 ROS 包根目录
PACKAGE_ROOT = os.path.dirname(BASE_DIR)

# -------------------------------------------------------------

def setup_log_and_model_paths(algo_name="RL_ALGO"):
    """
    根据算法名称和时间戳设置并创建日志和模型保存路径。
    """
    NOW = datetime.now().strftime("%Y%m%d_%H%M") 
    
    # 构造目录路径
    LOG_DIR = os.path.join(PACKAGE_ROOT, "logs")
    MODEL_DIR = os.path.join(PACKAGE_ROOT, "models")
    
    # 确保目录存在
    os.makedirs(LOG_DIR, exist_ok=True)
    os.makedirs(MODEL_DIR, exist_ok=True)
    
    # 最终的文件和日志名
    LOG_NAME = f"logs_{algo_name}_{NOW}"
    MODEL_NAME = f"models_{algo_name}_{NOW}"
    
    # 返回包含所有必要路径信息的字典
    return {
        "log_dir": LOG_DIR,
        "model_dir": MODEL_DIR,
        "log_name": LOG_NAME,
        "model_name": MODEL_NAME,
        "full_log_path": os.path.join(LOG_DIR, LOG_NAME),
    }