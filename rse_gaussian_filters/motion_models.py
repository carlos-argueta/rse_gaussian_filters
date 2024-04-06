# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

import numpy as np

def linear_velocity_motion_model():
	def state_transition_matrix_A():
		A = np.eye(3)

		return A

	# Assuming you have defined your state vector mu_t as [x, y, theta]
	def control_input_matrix_B(mu, delta_t):
		theta = mu[2]
		print(theta, delta_t)
		B = np.array([
		    [np.cos(theta) * delta_t, 0],
		    [np.sin(theta) * delta_t, 0],
		    [0, delta_t]
		])

		return B

	return state_transition_matrix_A, control_input_matrix_B

