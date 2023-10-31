import streamlit as st
import numpy as np
import matplotlib.pyplot as plt


# Set the page title
st.title('Plotting an Equation in a Range')

# Create a layout with three columns
col1, col2, col3 = st.columns(3)

# Input fields in the first column
with col1:
    x_start = st.number_input('X Start', min_value=0, max_value=10, value=0)

# Input fields in the second column
with col2:
    x_end = st.number_input('X End', min_value=0, max_value=10, value=10)

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
    Disturbance = st.number_input("Disturbance", min_value= 0.0, max_value = 1.0, step = .01)

# Input fields in the second column
with col2:
    Ki = st.number_input('Ki', min_value=0.0, max_value=100.0, value=1.0, step=0.01)

# Input fields in the third column
with col3:
    Kd = st.number_input('Kd', min_value=0.0, max_value=100.0, value=1.0, step=0.01)

# Input fields in the fourth column
with col4:
    seconds_delay = st.number_input('Milliseconds', min_value=0, max_value=1000, value=10)

def equation2(x, P, I, D, delay, disturbance):
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

    for t in x[1::]:
        integral += error * dt
        derivative = (error - prev_error) / dt

        # Control signal calculation
        control_signal = Kp * error + Ki * integral + Kd * derivative
        control_signal_history.append(control_signal)
        delayed_control_signal = control_signal_history.pop(0)

        y = y_values[-1] + delayed_control_signal * dt + disturbance  # Assuming y represents the system output
        y_values.append(y)  # Collect the time series of y values

        prev_error = error
        error = setpoint - y

    return y_values

y_values2 = equation2(x_range, Kp, Ki, Kd, seconds_delay, Disturbance)
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

def make_calculations(x, P, I, D, delay, disturbance, num_points):
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

    for t in x[1::]:
        integral += error * dt
        derivative = (error - prev_error) / dt

        # Control signal calculation
        control_signal = Kp * error + Ki * integral + Kd * derivative
        control_signal_history.append(control_signal)
        delayed_control_signal = control_signal_history.pop(0)

        y = y_values[-1] + delayed_control_signal * dt + disturbance  # Assuming y represents the system output
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

    ## CALCULATE STEADY STATE VALUE
    steady_state_value = y_values[-1]

    ## CALCULATE STEADY STATE TIME
    threshold = .025
    points_considered = int(num_points * 0.1)
    steady_state_time = -1
    for i in range(len(y_values) - points_considered):
        if within_range(i, points_considered, y_values, threshold):
            steady_state_time = i
            break

    return max_overshoot, time/num_points, steady_state_value, steady_state_time

calcs = make_calculations(x_range, Kp, Ki, Kd, seconds_delay, Disturbance, x_points)
st.text([("max_overshoot: " + str(calcs[0]-1)), ("time constant: " + str(calcs[1]))])
st.text([("steady_state_value: " + str(calcs[2]-1)), ("steady_state_time: " + str(calcs[3]/x_points))])


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
    if steady_state_time > 1:
        return 1000
    else:
        steady_state_time_constant = steady_state_time

    #we multiply smth to kinda standardize these values and their effects on overall score
    #I tried selecting them based on making a p decent case 2.5
    #ie overshoot 10% would get overshoot_score .01
    return overshoot_score * 250 + time_constant_score * 50 + steady_state_value_score * 250 + steady_state_time_constant * 5

st.text(get_score(calcs[0]-1, calcs[1]/(x_end-x_start), calcs[2]-1, calcs[3]/x_points * (x_end-x_start)))

start_time = time.time()

if st.button("Generate Best PID"):
    min_score = 1000
    best_vals = []
    for p in range(0, 100, 1):
        Kp_tune = p
        for i in range(0, 100, 2):
            Ki_tune = i
            for d in range(0,100, 2):
                Kd_tune = d/100
                tuning_calcs = make_calculations(x_range, Kp_tune, Ki_tune, Kd_tune, seconds_delay, Disturbance, x_points)
                score = get_score(tuning_calcs[0]-1, tuning_calcs[1]/(x_end-x_start), tuning_calcs[2]-1, tuning_calcs[3]/x_points * (x_end-x_start))
                if score < min_score:
                    min_score = score
                    best_vals = [Kp_tune, Ki_tune, Kd_tune]
    end_time = time.time()

    st.text(best_vals)
    st.text(min_score)
    st.text(start_time-end_time)

##so far this code can run the brute force with 100 values of KP, 50 for Ki and KD, with 100 num points in 80 ish seconds. 
##if we can speed this up 100x that'd be fire, or we could try better hueristics like get it to refine its search by
##inputting better mins and maxes to get a prediction with more decimals, also if we speed up the actual checking and score calcs
