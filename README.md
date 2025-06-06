# Robot-anthropomorphic-movement-history-code

本仓库包含了"机器人关键部位"联合大作业--人形机器人拟人运动及后续活动的历史代码

平台:格物gewu (URL:https://github.com/loongOpen/Unity-RL-Playground.git)

机器人URDF模型下载:URL:https://github.com/linqi-ye/robot-universe.git

训练指令:

E:\robot\loongopen_Unity_RL_PLayground\Unity-RL-Playground\gewu\Project\Assets\Retarget

mlagents-learn trainer_config.yaml --run-id=go2trot --force

mlagents-learn trainer_config.yaml --run-id=go2trot --resume
