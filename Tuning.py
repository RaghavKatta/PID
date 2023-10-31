import streamlit as st
import numpy as np
import matplotlib.pyplot as plt
import time

# Set the page title
st.title('PID TUNING MECHANISM(LINEAR)')

# Create a layout with three columns
col1, col2, col3 = st.columns(3)

# Input fields in the first column
with col1:
    x_start = st.number_input('X Start', min_value=0, max_value=10, value=0)

# Input fields in the second column
with col2:
    x_end = st.number_input('X End', min_value=0, max_value=10, value=1)

# Input fields in the third column
with col3:
    x_points = st.number_input('Number of Points', min_value=10, max_value=10000, value=100)
# Define the x_range based on user input
x_range = np.linspace(x_start, x_end, x_points)

# Define the equation (e.g., y = x^2)
def equation(x):
    y_values = []
    for x_value in x:
        if x_value < 0:
            y_values.append(0)
        else:
            y_values.append(1)
    return y_values

# Calculate the y values based on the equation
y_values = equation(x_range)

st.header("Here's where you enter your PID Values (2 decimal places)")
# Create a layout with four columns
col1, col2, col3, col4 = st.columns(4)

# Input fields in the first column
with col1:
    Kp = st.number_input('Kp', min_value=0.0, max_value=100.0, value=1.0, step=0.01)
    Disturbance = st.number_input("Disturbance", min_value= -1.00, max_value = 1.0, value = 0.0, step = .0001, format="%.4f")

# Input fields in the second column
with col2:
    Ki = st.number_input('Ki', min_value=0.0, max_value=100.0, value=0.0, step=0.01)
    Mass = st.number_input("Mass(kg)", min_value = .01, max_value = 100.00, step = .01)

# Input fields in the third column
with col3:
    Kd = st.number_input('Kd', min_value=0.0, max_value=100.0, value=0.0, step=0.01)

# Input fields in the fourth column
with col4:
    seconds_delay = st.number_input('Milliseconds', min_value=0, max_value=1000, value=10)

def equation2(x, P, I, D, delay, disturbance, mass):
    # PID control logic with fixed parameters
    Kp = P  # Proportional Gain
    Ki = I  # Integral Gain
    Kd = D  # Derivative Gain
    setpoint = 1  # Setpoint value
    y = 0
    error = setpoint - y
    integral = 0
    prev_error = setpoint - y
    dt = x[1] - x[0]

    # Time delay settings
    #the actual delay in seconds
    seconds_delay = delay/1000
    #x_points is the number of steps across 10 seconds so first divide by 10
    time_delay = int(seconds_delay * x_points/10)  # Adjust this value to set the time delay in time steps
    control_signal_history = [0] * time_delay

    y_values = [0]  # Store the time series of y values
    current_velocity = 0
    for t in x[1::]:
        integral += error * dt
        derivative = (error - prev_error) / dt

        # Control signal calculation
        control_signal = Kp * error + Ki * integral + Kd * derivative
        control_signal_history.append(control_signal)
        delayed_control_signal = control_signal_history.pop(0)
        current_velocity += (delayed_control_signal+disturbance)*dt/mass
        #control signal and disturbance are meant to force so this is where we have to account for force
        y = y_values[-1] + current_velocity * dt  # Assuming y represents the system output
        y_values.append(y)  # Collect the time series of y values

        prev_error = error
        error = setpoint - y

    return y_values

y_values2 = equation2(x_range, Kp, Ki, Kd, seconds_delay, Disturbance, Mass)
# Create a Matplotlib figure and plot the data
#hella customization cause why not, let's ball
fig, ax = plt.subplots()
def setup(fig,ax):
    ax.plot(x_range, y_values, label = 'Signal', color = 'blue')
    ax.plot(x_range, y_values2, label='Line 2', color='red')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_title('PID tuning stuff')
    fig.set_facecolor('black')
    ax.set_facecolor('black')
    ax.set_xlabel('time(s)', color='white')
    ax.set_ylabel('signal', color='white')  
    ax.title.set_color('white') 
    ax.spines['bottom'].set_color('white')  # Change the color of the x-axis line to white
    ax.spines['left'].set_color('white')    # Change the color of the y-axis line to white
    ax.xaxis.set_tick_params(color='white', labelcolor='white')
    ax.yaxis.set_tick_params(color='white', labelcolor='white')
    ax.set_ylim(0)
    ax.set_xlim(0)

# Display the Matplotlib figure using Streamlit
setup(fig,ax)
st.pyplot(fig)

def within_range(start, points_considered, y_values, threshold):
    
    if max(y_values[start: start + points_considered]) - min(y_values[start: start + points_considered]) < threshold:
        return True
    
    return False

def make_calculations(x, P, I, D, delay, disturbance, num_points, mass):
    ## CALCULATE Y VALUES
    # PID control logic with fixed parameters
    Kp = P  # Proportional Gain
    Ki = I  # Integral Gain
    Kd = D  # Derivative Gain
    setpoint = 1  # Setpoint value
    y = 0
    error = setpoint - y
    integral = 0
    prev_error = 0
    dt = x[1] - x[0]

    # Time delay settings
    #the actual delay in seconds
    seconds_delay = delay/1000
    #x_points is the number of steps across 10 seconds so first divide by 10
    time_delay = int(seconds_delay * x_points/10)  # Adjust this value to set the time delay in time steps
    control_signal_history = [0] * time_delay

    y_values = [0]  # Store the time series of y values

    current_velocity = 0
    for t in x[1::]:
        integral += error * dt
        derivative = (error - prev_error) / dt

        # Control signal calculation
        control_signal = Kp * error + Ki * integral + Kd * derivative
        control_signal_history.append(control_signal)
        delayed_control_signal = control_signal_history.pop(0)
        current_velocity += (delayed_control_signal+disturbance)*dt/mass
        #control signal and disturbance are meant to force so this is where we have to account for force
        y = y_values[-1] + current_velocity * dt  # Assuming y represents the system output
        y_values.append(y)  # Collect the time series of y values

        prev_error = error
        error = setpoint - y
    
    ## CALCULATE MAX OVERSHOOT
    max_overshoot = max(y_values)
    ## CALCULATE TIME TO 66.7%
    time = -1
    for i in range(len(y_values)):
        if y_values[i] > 0.67:
            time = i
            break

    ## CALCULATE STEADY STATE VALUE
    steady_state_value = y_values[-1]

    ## CALCULATE STEADY STATE TIME
    threshold = .02
    points_considered = int(num_points * 0.15)
    steady_state_time = -1
    for i in range(len(y_values) - points_considered):
        if within_range(i, points_considered, y_values, threshold):
            steady_state_time = i
            break

    return max_overshoot, time/num_points, steady_state_value, steady_state_time

calcs = make_calculations(x_range, Kp, Ki, Kd, seconds_delay, Disturbance, x_points, Mass)
st.text([("max_overshoot: " + str(calcs[0])), ("time constant: " + str(calcs[1]/(x_end-x_start)))])
st.text([("steady_state_value: " + str(calcs[2])), ("steady_state_time: " + str(calcs[3]/x_points * (x_end-x_start)))])

def get_score(overshoot, tc, steady_state_value, steady_state_time):
    #WE GONNA SCORE EVERYTHING OUTTA 25, BUT IF IT'S UNUSEABLY BAD WE WILL SET AT
    #1000 SO IT WILL BASICALLY BE DISREGARDED, lower scores are better
    overshoot_score = 0
    if overshoot > .5 or overshoot < -.5:
        return 1000
    else: 
        #lets make it proportional to overshoot squared cause that's hella imp
        overshoot_score = overshoot**2

    time_constant_score = 0
    if tc > .5:
        return 1000
    else:
        time_constant_score = tc

    steady_state_value_score = 0
    if steady_state_value > .4:
        return 1000
    else:
        steady_state_value_score = steady_state_value**2

    steady_state_time_constant = 0
    if steady_state_time > 1 or steady_state_time < 0:
        return 1000
    else:
        steady_state_time_constant = steady_state_time

    #we multiply smth to kinda standardize these values and their effects on overall score
    #I tried selecting them based on making a p decent case 2.5
    #ie overshoot 10% would get overshoot_score .01
    return overshoot_score * 250.0 + time_constant_score * 50.0 + steady_state_value_score * 250.0 + steady_state_time_constant * 5.0

st.text(get_score(calcs[0]-1, calcs[1]/(x_end-x_start), calcs[2]-1, calcs[3]/x_points * (x_end-x_start)))


def generate_best(Kpi, Kpf, Kpg, Kii, Kif, Kig, Kdi, kdf, Kdg):
    #we divide by 10 for Kp and Ki so that when we do the precise one we can use decimals
    min_score = 1000
    best_vals = []
    for p in range(Kpi, Kpf, Kpg):
        Kp_tune = p/10
        for i in range(Kii, Kif, Kig):
            Ki_tune = i/10
            for d in range(Kdi, kdf, Kdg):
                Kd_tune = d/1000
                tuning_calcs = make_calculations(x_range, Kp_tune, Ki_tune, Kd_tune, seconds_delay, Disturbance, x_points, Mass)
                score = get_score(tuning_calcs[0]-1, tuning_calcs[1]/(x_end-x_start), tuning_calcs[2]-1, tuning_calcs[3]/x_points * (x_end-x_start))
                if score < min_score:
                    min_score = score
                    best_vals = [Kp_tune, Ki_tune, Kd_tune]

    return best_vals, min_score
    

if st.button("Generate Best PID (Rough Estimate Version)(per 10)"):
    start_time = time.time()
    rough_estimate = generate_best(0, 1000, 100, 0, 1000, 100, 0, 2000, 100)
    end_time = time.time()
    st.text(rough_estimate)
    st.text("time: " + str(end_time-start_time))

st.header("Enter PID rough estimates")
# Create a layout with four columns
col1, col2, col3 = st.columns(3)

# Input fields in the first column
with col1:
    Kp_rough = st.number_input('Kp_rough', min_value=0, max_value=100, value=1, step=1)
    
# Input fields in the second column
with col2:
    Ki_rough = st.number_input('Ki_rough', min_value=0, max_value=100, value=0, step=1)

# Input fields in the third column
with col3:
    Kd_rough = st.number_input('Kd_rough(*100)', min_value=0, max_value=100, value=0, step=1)

if st.button("Generate Best PID (Acceptable)(per 1)"):
    start_time = time.time()
    accurate_estimate = generate_best((Kp_rough-10)*10, (Kp_rough+10)*10, 10, (Ki_rough-10)*10, (Ki_rough+10)*10, 10, (Kd_rough -10)*10, (Kd_rough +10)*10, 10)
    end_time = time.time()
    st.text(accurate_estimate)
    st.text("time: " + str(end_time-start_time))

st.header("Enter PID acceptable estimates")
# Create a layout with four columns
col1, col2, col3 = st.columns(3)

# Input fields in the first column
with col1:
    Kp_accurate = st.number_input('Kp_accurate', min_value=0, max_value=100, value=1, step=1)
    
# Input fields in the second column
with col2:
    Ki_accurate = st.number_input('Ki_accurate', min_value=0, max_value=100, value=0, step=1)

# Input fields in the third column
with col3:
    Kd_accurate = st.number_input('Kd_accurate(*100)', min_value=0, max_value=100, value=0, step=1)

if st.button("Generate Best PID (Precise!!)(per .1)"):
    start_time = time.time()
    precise_estimate = generate_best((Kp_accurate-1)*10, (Kp_accurate+1)*10, 1, (Ki_accurate-1)*10, (Ki_accurate+1)*10, 1, (Kd_accurate -1)*10, (Kd_accurate +1)*10, 1)
    end_time = time.time()
    st.text(precise_estimate)
    st.text("time: " + str(end_time-start_time))

