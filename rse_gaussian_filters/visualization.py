# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

from matplotlib.patches import Ellipse
import matplotlib.pyplot as plt

import numpy as np
import math

class Visualizer:
    def __init__(self):
        self.fig, self.ax = plt.subplots()
        self.gt_line, = self.ax.plot([], [], 'g-', label='Ground Truth')  # Green line for ground truth path
        self.kf_line, = self.ax.plot([], [], 'b-', label='Kalman Filter')  # Red line for Kalman filter path
        self.ax.legend()

        self.gt_path = []
        self.kf_path = []

        # Ellipse to represent the covariance matrix
        self.cov_ellipse = Ellipse(xy=(0, 0), width=0, height=0, edgecolor='black', fc='None', lw=2)
        self.ax.add_patch(self.cov_ellipse)

        # Set axis labels
        self.ax.set_xlabel('X coordinate')
        self.ax.set_ylabel('Y coordinate')
        self.ax.set_title('Kalman Filter Visualization')

    def update(self, gt_pose, kf_pose, kf_cov, step="update"):
        # Update ground truth path
        self.gt_path.append(gt_pose[:2])
        gt_x, gt_y = zip(*self.gt_path)
        self.gt_line.set_data(gt_x, gt_y)

        # Update Kalman filter path
        self.kf_path.append(kf_pose[:2])
        kf_x, kf_y = zip(*self.kf_path)
        self.kf_line.set_data(kf_x, kf_y)

        # Adjust plot limits if necessary
        self.ax.set_xlim(min(gt_x + kf_x) - 1, max(gt_x + kf_x) + 1)
        self.ax.set_ylim(min(gt_y + kf_y) - 1, max(gt_y + kf_y) + 1)

        # Update the covariance ellipse
        ellipse_color = 'black' if step == "update" else 'gray'
        self._update_covariance_ellipse(kf_pose, kf_cov, ellipse_color)

        # Draw the updated plot
        self.fig.canvas.draw_idle()
        plt.pause(0.01)

    def _update_covariance_ellipse(self, pose, cov, color):
        if cov.shape == (3, 3):  # If full covariance matrix, extract x, y part
            cov = cov[:2, :2]

        eigenvals, eigenvecs = np.linalg.eig(cov)
        angle = np.rad2deg(np.arctan2(*eigenvecs[:, 0][::-1]))
        width, height = 2 * np.sqrt(eigenvals)  # Scale factor for visualization
        self.cov_ellipse.set_center((pose[0], pose[1]))
        self.cov_ellipse.width = width
        self.cov_ellipse.height = height
        self.cov_ellipse.angle = angle
        self.cov_ellipse.set_edgecolor(color)

