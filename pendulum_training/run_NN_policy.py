import numpy as np
import tensorflow as tf
import os
from lqr_env import LQR_Env

from train_pg_f18 import build_mlp
import matplotlib.pyplot as plt

data_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'data')
logdir = 'pendulum_training/data/p_b6000_r1e-3_lqr_03-04-2020_23-13-51/'
logdir = os.path.join(data_path, logdir)
logdir = os.path.join(logdir, 'model.ckpt')

ob_dim = 2
ac_dim = 1
size = 64 # 256
n_layers = 2 # 6

sy_ob_no = tf.placeholder(shape=[None, ob_dim], name="ob", dtype=tf.float32)
ac_prediction = tf.squeeze(build_mlp(
                                input_placeholder = sy_ob_no,
                                output_size = ac_dim,
                                scope = "nn_action",
                                n_layers = n_layers,
                                size = size))
saver = tf.train.Saver()
ac_traj = np.empty((0, 1))
num_sim_iter = 200
# state_now = np.array([[24.613,87.6611,7.5436,0.44745,-0.2202]])
state_now = np.array([[0.5,-0.4]])
state_traj = state_now

with tf.Session() as sess:
  # Restore variables from disk.
  saver.restore(sess, logdir)
  for i in range(num_sim_iter):
      ac_predict = sess.run([ac_prediction], feed_dict={sy_ob_no: state_now})
      ac_predict = np.array(ac_predict)
      
      LQR_Env.state = state_now
      state_next, reward, _, _ = LQR_Env.step(ac_predict) 
      state_next = np.reshape(state_next, (1, ob_dim))
      # print(ac_predict.shape)
      # ac_traj = np.append(ac_traj, ac_predict, axis= 0)
      state_traj = np.vstack((state_traj, state_next))
      ac_traj = np.vstack((ac_traj, ac_predict))

      state_now = state_next

plt.figure(1)
plt.plot(state_traj[:, 0], state_traj[:, 1], label = 'xdot')
plt.show()

plt.figure(2)
plt.plot(ac_traj[:, 0], label='thrust')
plt.show()