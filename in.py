#!/usr/bin/env python
# coding: utf-8

# In[12]:


import numpy as np
from scipy.optimize import minimize
import streamlit as st

# Define the kinematic parameters of the robot
link_lengths = [1, 1, 1, 1, 1, 1]  # Lengths of each link
joint_limits = [(-180, 180), (-180, 180), (-180, 180), (-180, 180), (-180, 180), (-180, 180)]  # Joint limits

# Streamlit app title
st.title("Robot Kinematics Optimization")

# Streamlit input widgets
x_desired = st.number_input("Enter the desired x-coordinate of the end-effector:", value=2.0)
y_desired = st.number_input("Enter the desired y-coordinate of the end-effector:", value=2.0)
z_desired = st.number_input("Enter the desired z-coordinate of the end-effector:", value=2.0)

end_effector_desired = np.array([x_desired, y_desired, z_desired])

# Forward kinematics function
def forward_kinematics(joint_angles):
    # Compute forward kinematics
    x = link_lengths[0] * np.cos(np.radians(joint_angles[0]))
    y = link_lengths[0] * np.sin(np.radians(joint_angles[0]))
    for i in range(1, len(joint_angles)):
        x += link_lengths[i] * np.cos(np.radians(np.sum(joint_angles[:i+1])))
        y += link_lengths[i] * np.sin(np.radians(np.sum(joint_angles[:i+1])))
    return np.array([x, y, 0])  # Simplified to 2D for illustration

# Objective function
def objective_function(joint_angles):
    # Calculate error
    end_effector_actual = forward_kinematics(joint_angles)
    error = end_effector_actual - end_effector_desired
    return np.sum(error**2)

# Inequality constraints function
def inequality_constraints(joint_angles):
    # Check if joint angles are within limits
    constraints = []
    for i in range(len(joint_angles)):
        if joint_angles[i] < joint_limits[i][0] or joint_angles[i] > joint_limits[i][1]:
            constraints.append(1)
        else:
            constraints.append(0)
    return constraints

# Initial guess for joint angles
initial_guess = np.zeros(6)

# Run optimization when the button is clicked
if st.button("Optimize"):
    result = minimize(objective_function, initial_guess, constraints={'type': 'ineq', 'fun': inequality_constraints})
    optimized_joint_angles = result.x
    optimized_end_effector_position = forward_kinematics(optimized_joint_angles)
    
    # Display results
    st.write("Optimized Joint Angles:", optimized_joint_angles)
    st.write("Optimized End-Effector Position:", optimized_end_effector_position)


# In[11]:








