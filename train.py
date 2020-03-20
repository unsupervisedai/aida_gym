import gym
import numpy as np

from stable_baselines.common.policies import MlpPolicy
from stable_baselines.common.vec_env import DummyVecEnv, SubprocVecEnv, VecNormalize
from stable_baselines import PPO2
import aida_env.aida_gym_env as e
import pybullet as p
import argparse
import imageio
import time
import os

if __name__ == '__main__':

	workDirectory = "."
	

	parser = argparse.ArgumentParser(description='Aida traning script')

	parser.add_argument('--resume_from', default=None,
                    help='Name of the model to start from')				
	parser.add_argument('--normalize', type=bool, default=False, 
                    help='Normalize the environement for training (default: False)')
	parser.add_argument('--name', default=None, required = True,
                    help='Name of the model (required)')
	parser.add_argument('--total_steps', default=10000000, type=int,
                    help='Total number of steps to train the model (default: 10 000 000)')
	parser.add_argument('--save_every', default=500000, type=int,
                    help='The number of step to train the model before saving (default: 500 000)')
	
	
	parser.add_argument('--gamma', type=float, default=0.99,
                    help='Discount factor (default: 0.99)')
	parser.add_argument('--n_steps', type=int, default=128,
                    help='The number of steps to run for each environment per update (default: 128)')
	parser.add_argument('--ent_coef', type=float, default=0.01,
                    help='Entropy coefficient for the loss calculation (default: 0.01)')
	parser.add_argument('--learning_rate', type=float, default=0.00025,
                    help='The learning rate (default: 0.00025)')
	parser.add_argument('--vf_coef', type=float, default=0.5,
                    help='Value function coefficient for the loss calculation (default: 0.5)')
	parser.add_argument('--max_grad_norm', type=float, default=0.5,
                    help='The maximum value for the gradient clipping (default: 0.5)')
	parser.add_argument('--lam', type=float, default=0.95,
                    help='Factor for trade-off of bias vs variance for Generalized Advantage Estimator (default: 0.95)')
	parser.add_argument('--nminibatches', type=int, default=4,
                    help='Number of training minibatches per update (default: 4)')
	parser.add_argument('--noptepochs', type=int, default=4,
                    help='Number of epoch when optimizing the surrogate (default: 4)')
	parser.add_argument('--cliprange', type=float, default=0.2,
                    help='Clipping parameter (default: 0.2)')
	parser.add_argument('--cliprange_vf', type=float, default=None,
                    help='Clipping parameter for the value function, it can be a function. This is a parameter specific to the OpenAI implementation. If None is passed (default), then cliprange (that is used for the policy) will be used. IMPORTANT: this clipping depends on the reward scaling. To deactivate value function clipping (and recover the original PPO implementation), you have to pass a negative value (e.g. -1). (default: None)')		
	parser.add_argument('--layers', default=[100,100],nargs='+', type=int,
					help='Architecture of the neural network (default: [100,100])')

					
	parser.add_argument('--default_reward', default=2.0, type=float,
					help='Reward aida gets for staying alive at each step (default: 2.0)')	
	parser.add_argument('--height_weight', default=4.0, type=float,
					help='Multiplicator of the height reward (default: 4.0)')		
	parser.add_argument('--orientation_weight', default=1.0, type=float,
					help='Multiplicator of the reward telling if aida stands straight (default: 1.0)')	
	parser.add_argument('--direction_weight', default=1.0, type=float,
					help='Multiplicator of the reward telling if aida faces its objective (default: 1.0)')	
	parser.add_argument('--speed_weight', default=0.0, type=float,
					help='Multiplicator of the speed reward (default: 0.0)')	
				
	args = parser.parse_args()
	
	model_name  = args.name
	name_resume = args.resume_from
	normalize   = args.normalize


	"""
	workDirectory/resultats/name/name.zip
							    /normalizeData/ret_rms.pkl
											  /obs_rms.pkl
								/video
				 /log
	"""
	
	try:
		os.mkdir(workDirectory+"/resultats")
	except FileExistsError:
		print("Directory already exists")
	try:
		os.mkdir(workDirectory+"/resultats/"+model_name)
	except FileExistsError:
		print("Directory already exists")
	if normalize:
		try:
			os.mkdir(workDirectory+"/resultats/"+model_name+"/normalizeData")
		except FileExistsError:
			print("Directory already exists")
	try:
		os.mkdir(workDirectory+"/resultats/"+model_name+"/video")
	except FileExistsError:
		print("Directory already exists")
	try:
		os.mkdir(workDirectory+"/log")
	except FileExistsError:
		print("Directory already exists")
	
	commands = [[1,0]]
	for i in range(5):
		commands += [[commands[-1][0]+np.random.rand(),commands[-1][0]+np.random.rand()]]
		
	env = SubprocVecEnv([lambda:  e.AidaBulletEnv(commands,
												  render  = False, 
												  on_rack = False,
												  default_reward     = args.default_reward,
												  height_weight      = args.height_weight,
												  orientation_weight = args.orientation_weight,
												  direction_weight   = args.direction_weight,
												  speed_weight       = args.speed_weight
												  )
						for i in range(2)])
	
	if normalize:
		env = VecNormalize(env)


	model = PPO2(MlpPolicy, 
				 env, 

				 gamma           = args.gamma,
				 n_steps         = args.n_steps,
				 ent_coef        = args.noptepochs,
				 learning_rate   = args.learning_rate,
				 vf_coef         = args.vf_coef,
				 max_grad_norm   = args.max_grad_norm,
				 lam             = args.lam,
				 nminibatches    = args.nminibatches,
				 noptepochs      = args.noptepochs,
				 cliprange       = args.cliprange,
				 cliprange_vf    = args.cliprange_vf,
				 verbose         = 0,
				 policy_kwargs   = dict(layers=args.layers),
				 tensorboard_log = workDirectory+"/log"
			   )
	
	if name_resume!=None:

		model = PPO2.load(   workDirectory+"/resultats/"+name_resume+"/"+name_resume+".zip",
						     env=env,
						     gamma           = args.gamma,
							 n_steps         = args.n_steps,
							 ent_coef        = args.noptepochs,
							 learning_rate   = args.learning_rate,
							 vf_coef         = args.vf_coef,
							 max_grad_norm   = args.max_grad_norm,
							 lam             = args.lam,
							 nminibatches    = args.nminibatches,
							 noptepochs      = args.noptepochs,
							 cliprange       = args.cliprange,
							 cliprange_vf    = args.cliprange_vf,
							 verbose         = 0,
							 policy_kwargs   = dict(layers=args.layers),
							 tensorboard_log = workDirectory+"/log"
						)
		if normalize:
			env.load_running_average(workDirectory+"/resultats/"+name_resume+"/normalizeData")	 
			
	for i in range(args.total_steps//args.save_every):
		model.learn(total_timesteps=args.save_every, tb_log_name=model_name, reset_num_timesteps=False)
		if normalize:
			env.save_running_average(workDirectory+"/resultats/"+model_name+"/normalizeData")
		model.save(workDirectory+"/resultats/"+model_name+"/"+model_name)
		
		print("\n saved at "+str((i+1)*args.save_every))
		
	env = DummyVecEnv([lambda:  e.AidaBulletEnv(commands,
											  render  = False, 
											  on_rack = False,
												 
											  default_reward     = args.default_reward,
											  height_weight      = args.height_weight,
											  orientation_weight = args.orientation_weight,
											  direction_weight   = args.direction_weight,
											  speed_weight       = args.speed_weight
											  )
					])
	if normalize:
		env = VecNormalize(env)
		env.load_running_average(workDirectory+"/resultats/"+model_name+"/normalizeData")

	images = []
	obs = env.reset()
	img = env.render(mode='rgb_array')
	for i in range(15*2*10):
		images.append(img)
		action, _ = model.predict(obs)
		obs, _, _ ,_ = env.step(action)
		img = env.render(mode='rgb_array')

	imageio.mimsave(workDirectory+"/resultats/"+model_name+"/video/vid.gif", [np.array(img) for i, img in enumerate(images) if i%2 == 0], fps=20)
