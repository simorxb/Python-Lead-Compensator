import control as ct
import numpy as np
import math
import matplotlib.pyplot as plt

s = ct.TransferFunction.s

# System parameters
m = 10
k = 0.5

# System transfer function
P = 1/(s*(m*s+k))

# Proportional compensator parameters
kp = 0.018

# Proportional compensator transfer function
C_P = kp

# Close-loop transfer function
G_P = ct.feedback(C_P*P, 1)

# Plot root-locus - Proportional compensator
plt.figure()
ct.root_locus(C_P*P)

# Plot Bode diagram
plt.figure()
ct.bode(C_P*P)

# Calculate the gain and phase margins - Proportional compensator
gm, pm, gc, pc = ct.margin(C_P*P)
gm_dB = 20*np.log10(gm)

# Print margins
print(f"Gain Margin: {gm_dB:.3g} dB at frequency {gc:.3g} rad/sec")
print(f"Phase Margin: {pm:.3g} deg at frequency {pc:.3g} rad/sec")
print(f"Delay Margin: {((pm*math.pi/180)/pc):.3g} seconds")

# Ideal step response - Proportional compensator
t_P, yout_P = ct.step_response(G_P, T=250)

# Lead compensator parameters
kl = 0.4
tau_p = 1
tau_z = 18

# Lead compensator transfer function
C_L = kl*(tau_z*s+1)/(tau_p*s+1)

# Close-loop transfer function
G_L = ct.feedback(C_L*P, 1)

# Plot root-locus - Lead compensator
plt.figure()
ct.root_locus(C_L*P)

# Plot Bode diagram
plt.figure()
ct.bode(C_L*P)

# Calculate the gain and phase margins - Lead compensator
gm, pm, gc, pc = ct.margin(C_L*P)
gm_dB = 20*np.log10(gm)

# Print margins
print(f"Gain Margin: {gm_dB:.3g} dB at frequency {gc:.3g} rad/sec")
print(f"Phase Margin: {pm:.3g} deg at frequency {pc:.3g} rad/sec")
print(f"Delay Margin: {((pm*math.pi/180)/pc):.3g} seconds")

# Step response - nominal - output
t_L, yout_L = ct.step_response(G_L, T=250)
plt.figure()
plt.subplot(2, 1, 1)
plt.plot(t_P, yout_P, label="Proportional compensator")
plt.plot(t_L, yout_L, label="Lead compensator")
plt.xlabel("Time [s]")
plt.ylabel("Position [m]")
plt.legend()
plt.grid()
plt.title("Step response - nominal")

# Find transfer function from setpoint to control input (torque)
G_P_u = C_P/(1+C_P*P)
G_L_u = C_L/(1+C_L*P)

# Step response - nominal - control input
t_P, uout_P = ct.step_response(G_P_u, T=250)
t_L, uout_L = ct.step_response(G_L_u, T=250)
plt.subplot(2, 1, 2)
plt.plot(t_P, uout_P, label="Proportional compensator")
plt.plot(t_L, uout_L, label="Lead compensator")
plt.xlabel("Time [s]")
plt.ylabel("Force [N]")
plt.legend()
plt.grid()

# Evaluate robustness with various combinations of m and k on the proportional compensator
m_V = [5, 10, 15]
k_V = [0.25, 0.5, 0.75]
plt.figure()
for m in m_V:
    for k in k_V:

        # System transfer function
        P = 1/(s*(m*s+k))

        # Close-loop transfer function
        G_P = ct.feedback(C_P*P, 1)

        # Step response
        t_L, yout_L = ct.step_response(G_P, T=250)
        
        # Plot response
        plt.plot(t_L, yout_L, label="m = " + "{:.3g}".format(m) + " - k = " + "{:.3g}".format(k))

plt.xlabel("Time [s]")
plt.ylabel("Position [m]")
plt.legend()
plt.grid()
plt.title("Step response - Proportional compensator - Robustness")

# Evaluate robustness with various combinations of m and k on the lead compensator
m_V = [5, 10, 15]
k_V = [0.25, 0.5, 0.75]
plt.figure()
for m in m_V:
    for k in k_V:

        # System transfer function
        P = 1/(s*(m*s+k))

        # Close-loop transfer function
        G_L = ct.feedback(C_L*P, 1)

        # Step response
        t_L, yout_L = ct.step_response(G_L, T=40)
        
        # Plot response
        plt.plot(t_L, yout_L, label="m = " + "{:.3g}".format(m) + " - k = " + "{:.3g}".format(k))

plt.xlabel("Time [s]")
plt.ylabel("Position [m]")
plt.legend()
plt.grid()
plt.title("Step response - Lead compensator - Robustness")

plt.show()