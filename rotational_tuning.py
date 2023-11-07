import streamlit as st
import numpy as np
import matplotlib.pyplot as plt
import time

# Set the page title
st.title('PID TUNING MECHANISM(ROTATIONAL)')

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
    Kp = st.number_input('Kp', min_value=0.0, max_value=10000000.0, value=1.0, step=0.01)
    Disturbance = st.number_input("Disturbance (Units NM)", min_value= -1000.00, max_value = 1000.0, value = 0.0, step = .0001, format="%.4f")

# Input fields in the second column
with col2:
    Ki = st.number_input('Ki', min_value=0.0, max_value=10000000.0, value=0.0, step=0.01)
    #CODE WILL STILL USE VARIABLE MASS SO i DON'T NEED TO GO BACK AND CHANGE IT FROM THE LINEAR SET UP
    Mass = st.number_input("Inertia(kg*m^2)", min_value = 1.0, max_value = 500.00, step = .01)

# Input fields in the third column
with col3:
    Kd = st.number_input('Kd', min_value=0.0, max_value=10000000.0, value=0.0, step=0.001, format="%.3f")
    Max_Force = st.number_input("Max Torque(N*m)", min_value = .1, max_value = 100000.00, value = 100.0 ,step = .1)

# Input fields in the fourth column
with col4:
    seconds_delay = st.number_input('Milliseconds', min_value=0, max_value=1000, value=10)

def equation2(x, P, I, D, delay, disturbance, mass, max_force):
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
    at_max_force = False
    # Time delay settings
    #the actual delay in seconds
    seconds_delay = delay/1000
    #x_points is the number of steps across 10 seconds so first divide by 10
    time_delay = int(seconds_delay * x_points/10)  # Adjust this value to set the time delay in time steps
    control_signal_history = [0] * time_delay

    y_values = [0]  # Store the time series of y values
    current_velocity = 0
    for t in x[1::]:
        #Now we have to fix the issue of max force
        #Prevent windup by locking integral
        if not at_max_force:
            integral += error * dt
        derivative = (error - prev_error) / dt

        # Control signal calculation
        control_signal = Kp * error + Ki * integral + Kd * derivative
        if control_signal > max_force:
            control_signal = max_force
            at_max_force = True
        elif at_max_force:
            at_max_force = False
        control_signal_history.append(control_signal)
        delayed_control_signal = control_signal_history.pop(0)
        #disturbance is treated as a force with the units N if translational 
        #for rotational is treated as a torque with units NM
        current_velocity += (delayed_control_signal+disturbance)*dt/mass
        #control signal and disturbance are meant to force so this is where we have to account for force
        y = y_values[-1] + current_velocity * dt  # Assuming y represents the system output
        y_values.append(y)  # Collect the time series of y values

        prev_error = error
        error = setpoint - y

    return y_values

y_values2 = equation2(x_range, Kp, Ki, Kd, seconds_delay, Disturbance, Mass, Max_Force)
# Create a Matplotlib figure and plot the data
#hella customization cause why not, let's ball
fig, ax = plt.subplots()
def setup(fig,ax):
    ax.plot(x_range, y_values, label = 'signal', color = 'blue')
    ax.plot(x_range, y_values2, label='PID Results', color='red')
    ax.set_title('PID tuning stuff')
    fig.set_facecolor('black')
    ax.set_facecolor('black')
    ax.set_xlabel('time(s)', color='white')
    ax.set_ylabel('position(theta)', color='white')  
    ax.title.set_color('white') 
    ax.spines['bottom'].set_color('white')  # Change the color of the x-axis line to white
    ax.spines['left'].set_color('white')    # Change the color of the y-axis line to white
    ax.xaxis.set_tick_params(color='white', labelcolor='white')
    ax.yaxis.set_tick_params(color='white', labelcolor='white')
    ax.set_ylim(0)
    ax.set_xlim(0)
    ax.legend(loc='upper center', bbox_to_anchor=(0.1, -0.07), frameon=False, labelcolor = "white")

# Display the Matplotlib figure using Streamlit
setup(fig,ax)
st.pyplot(fig)

def within_range(start, points_considered, y_values, threshold):
    
    if max(y_values[start: start + points_considered]) - min(y_values[start: start + points_considered]) < threshold:
        return True
    
    return False

def make_calculations(x, P, I, D, delay, disturbance, num_points, mass, max_force):
    ## CALCULATE Y VALUES
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
    at_max_force = False
    # Time delay settings
    #the actual delay in seconds
    seconds_delay = delay/1000
    #x_points is the number of steps across 10 seconds so first divide by 10
    time_delay = int(seconds_delay * x_points/10)  # Adjust this value to set the time delay in time steps
    control_signal_history = [0] * time_delay

    y_values = [0]  # Store the time series of y values
    current_velocity = 0
    for t in x[1::]:
        #Now we have to fix the issue of max force
        #Prevent windup by locking integral
        if not at_max_force:
            integral += error * dt
        derivative = (error - prev_error) / dt

        # Control signal calculation
        control_signal = Kp * error + Ki * integral + Kd * derivative
        if control_signal > max_force:
            control_signal = max_force
            at_max_force = True
        elif at_max_force:
            at_max_force = False
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

calcs = make_calculations(x_range, Kp, Ki, Kd, seconds_delay, Disturbance, x_points, Mass, Max_Force)
st.text([("max_overshoot: " + str(calcs[0]-1)), ("time constant: " + str(calcs[1]/(x_end-x_start)))])
st.text([("steady_state_value: " + str(calcs[2]-1)), ("steady_state_time: " + str(calcs[3]/x_points * (x_end-x_start)))])

def get_score(overshoot, tc, steady_state_value, steady_state_time):
    #WE GONNA SCORE EVERYTHING OUTTA 25, BUT IF IT'S UNUSEABLY BAD WE WILL SET AT
    #1000 SO IT WILL BASICALLY BE DISREGARDED, lower scores are better
    overshoot_score = 0
    
    #lets make it proportional to overshoot squared cause that's hella imp
    overshoot_score = overshoot**2


    time_constant_score = tc

    steady_state_value_score = 0
    if steady_state_value > .4:
        return 1000
    else:
        steady_state_value_score = steady_state_value**2

    steady_state_time_constant = 0
    if steady_state_time < 0:
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
        Kp_tune = p/100
        for i in range(Kii, Kif, Kig):
            Ki_tune = i/100
            for d in range(Kdi, kdf, Kdg):
                Kd_tune = d/100
                tuning_calcs = make_calculations(x_range, Kp_tune, Ki_tune, Kd_tune, seconds_delay, Disturbance/Mass, x_points, 1, Max_Force/Mass)
                score = get_score(tuning_calcs[0]-1, tuning_calcs[1]/(x_end-x_start), tuning_calcs[2]-1, tuning_calcs[3]/x_points * (x_end-x_start))
                if score < min_score:
                    min_score = score
                    best_vals = [Kp_tune, Ki_tune, Kd_tune]
    raw_vals = best_vals.copy()
    for i in range(len(best_vals)):
        best_vals[i] = best_vals[i] * Mass
    return raw_vals, best_vals, min_score
    

if st.button("Generate Best PID (Rough Estimate Version)(1-2 digits)"):
    start_time = time.time()
    rough_estimate = generate_best(0, 10000, 1000, 0, 10000, 1000, 0, 100000, 1000)
    end_time = time.time()
    st.text("if any of the raw estimate values are 100 then something might be wrong, tell raghav if you run into it")
    st.text("raw estimate: " + str(rough_estimate[0]) + " Mass adjusted estimate: " + str(rough_estimate[1]) + " Score: " + str(rough_estimate[2]))
    st.text("time: " + str(end_time-start_time))

st.header("Enter PID rough estimates(raw)(1-2)")
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
    Kd_rough = st.number_input('Kd_rough', min_value=0, max_value=100, value=0, step=1)

if st.button("Generate Best PID (Acceptable)(2-3)"):
    start_time = time.time()
    accurate_estimate = generate_best(max(0,(Kp_rough-10)*100), (Kp_rough+10)*100, 100, max(0,(Ki_rough-10)*100), (Ki_rough+10)*100, 100, max(0,(Kd_rough -10)*100), (Kd_rough +10)*100, 100)
    end_time = time.time()
    st.text("accurate estimate: " + str(accurate_estimate[0]) + " Mass adjusted estimate: " + str(accurate_estimate[1]) + " Score: " + str(accurate_estimate[2]))
    st.text("time: " + str(end_time-start_time))

st.header("Enter PID acceptable estimates(raw)")
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
    Kd_accurate = st.number_input('Kd_accurate', min_value=0, max_value=100, value=0, step=1)

if st.button("Generate Better PID (Precise!)(3-4)"):
    start_time = time.time()
    precise_estimate = generate_best(max((Kp_accurate-1)*100,0), (Kp_accurate+1)*100, 10, max(0,(Ki_accurate-1)*100), (Ki_accurate+1)*100, 10, max(0,(Kd_accurate -1)*100), (Kd_accurate +1)*100, 10)
    end_time = time.time()
    st.text("precise estimate: " + str(precise_estimate[0]) + " Mass adjusted estimate: " + str(precise_estimate[1]) + " Score: " + str(precise_estimate[2]))
    st.text("time: " + str(end_time-start_time))

st.header("Enter PID precision estimates(raw)")
# Create a layout with four columns
col1, col2, col3 = st.columns(3)
with col1:
    Kp_precise = st.number_input('Kp_precision', min_value=0.0, max_value=100.0, value=1.0, step=.1)
    
# Input fields in the second column
with col2:
    Ki_precise = st.number_input('Ki_precision', min_value=0.0, max_value=100.0, value=0.0, step=.1)

# Input fields in the third column
with col3:
    Kd_precise = st.number_input('Kd_precision', min_value=0.0, max_value=100.0, value=0.0, step=.1)

if st.button("Generate BETTER PID (Precise!!)(4-5)"):
    start_time = time.time()
    extra_precise_estimate = generate_best(max(int((Kp_precise - .1) * 100.0), 0), int((Kp_precise + .1) * 100.0), 1, max(0, int((Ki_precise - .1) * 100.0)), int((Ki_precise + .1) * 100.0), 1, max(0, int((Kd_precise - .1) * 100.0)), int((Kd_precise + .1) * 100.0), 1)
    end_time = time.time()
    st.text("extra precise estimate: " + str(extra_precise_estimate[0]) + " Mass adjusted estimate: " + str(extra_precise_estimate[1]) + " Score: " + str(extra_precise_estimate[2]))
    st.text("time: " + str(end_time-start_time))

##NOW WE ARE DISPLAYING THE BEST PID VALUES AS THE INERTIA AND THRUST CHANGE
#we can create an insert for starting and ending inertia 
inertia_start = st.number_input('inertia_start', min_value=0.0, max_value=1000.0, value=1.0, step=.0001, format="%.4f")
inertia_end = st.number_input('inertia_end', min_value=0.0, max_value=1000.0, value=1.0, step=.0001, format="%.4f")
inertia_difference = inertia_end - inertia_start
#initalize the points for this rn
seconds_delay_plot = st.number_input('Delay Plot', min_value=0, max_value=1000, value=10)
Disturbance_plot = st.number_input("Disturbance Plot", min_value= -1000.00, max_value = 1000.0, value = 0.0, step = .0001, format="%.4f")
inertia_points = 28
inertia_range = np.linspace(0, 7, inertia_points)
inertia_calc_points = 100
inertia_calc_range = np.linspace(0, 7, inertia_calc_points)
#here we have to use the code for inertia and thrust though(no fun inserts)
def Inertia(time):
    # time in ms
    # check my math on this, I put the base equations in the notes
    # Inertia_difference is the difference between initial and final inertia
    # fuel fraction is the integral of the fuel used up, but it's not divided yet
    # we have to do that at the end, which is why it is divided by 961.8 at the end
    t = time
    fuel_fraction = 0
    if t < 0.125:
        fuel_fraction = (-1300/3) * (t)**3 + 350 * t + t**2 
    elif t < 2.125:
        fuel_fraction = 250 *(t-.125) - 15 * (t-.125)**2 + 35.341145+2.5
    elif t < 3.5:
        fuel_fraction = 190 * (t-2.125) - (9/2) * (t - 2.125)**2 + 475.3411 + 2.5
    elif t < 5.4:
        fuel_fraction = 177.6 * (t-3.5) - 35 * (t - 3.5)**2 + 728.08333 + 2.5
    elif t < 6.4:
        fuel_fraction = 44.6 * (t-5.4) - 22 * (t - 5.4)**2 + 939.1733333 + 2.5
    else:
        fuel_fraction = 961.8 + 2.5

    return inertia_difference * fuel_fraction / (961.8 + 2.5) + inertia_start


def thrust(time):
    # time in ms
    # check my math on this, I put the base equations in the notes
    # Inertia_difference is the difference between initial and final inertia
    # fuel fraction is the integral of the fuel used up, but it's not divided yet
    # we have to do that at the end, which is why it is divided by 961.8 at the end
    t = time
    thrust = 0
    if t < 0.125:
        thrust = -1300 * (t + 0.125)**2 + 350 + 2 * t
    elif t < 2.125:
        thrust = 250 - 30 * (t - 0.125)
    elif t < 3.5:
        thrust = 190 - 9 * (t - 2.125)
    elif t < 5.4:
        thrust = 177.6 - 70 * (t - 3.5)
    elif t < 6.4:
        thrust = 44.6 - 44 * (t - 5.4)
    else:
        thrust = 0

    return thrust


inertia_vals = []
for x in inertia_range:
    inertia_vals.append(Inertia(x))

thrust_vals = []
for x in inertia_range:
    thrust_vals.append(thrust(x))


st.title("")
# Create a layout with four columns
col1, col2 = st.columns(2)
with col1:
    st.title("Thrust Over Time")
    display, axis = plt.subplots()
    def setupPIDdisplay(fig,ax):
        fig.set_facecolor('black')
        ax.set_facecolor('black')
        ax.set_xlabel('time(s)', color='white')
        ax.set_ylabel('PID values', color='white')  
        ax.title.set_color('white') 
        ax.spines['bottom'].set_color('white')  # Change the color of the x-axis line to white
        ax.spines['left'].set_color('white')    # Change the color of the y-axis line to white
        ax.xaxis.set_tick_params(color='white', labelcolor='white')
        ax.yaxis.set_tick_params(color='white', labelcolor='white')

    # Display the Matplotlib figure using Streamlit
    setupPIDdisplay(display,axis)
    ax.set_title("Thrust")
    axis.plot(inertia_range, thrust_vals, label = 'Thrust', color = 'RED')
    axis.set_ylim(0,350)
    axis.set_xlim(0,7)
    st.pyplot(display)

    
# Input fields in the second column
with col2:
    st.title("Inertia Over Time")

    display, axis = plt.subplots()
    # Display the Matplotlib figure using Streamlit
    setupPIDdisplay(display,axis)
    axis.plot(inertia_range, inertia_vals, label = 'Inertia', color = 'blue')
    axis.set_xlim(0,7)
    st.pyplot(display)


@st.cache_data
def generate_best_plot(Kpi, Kpf, Kpg, Kii, Kif, Kig, Kdi, kdf, Kdg, mass, max_force):
    #we divide by 10 for Kp and Ki so that when we do the precise one we can use decimals
    min_score = 1000
    best_vals = []
    for p in range(Kpi, Kpf, Kpg):
        Kp_tune = p/100
        for i in range(Kii, Kif, Kig):
            Ki_tune = i/100
            for d in range(Kdi, kdf, Kdg):
                Kd_tune = d/100
                tuning_calcs = make_calculations(inertia_calc_range, Kp_tune, Ki_tune, Kd_tune, seconds_delay, Disturbance_plot/mass, inertia_calc_points, 1, max_force/mass)
                score = get_score(tuning_calcs[0]-1, tuning_calcs[1]/(7), tuning_calcs[2]-1, tuning_calcs[3]/x_points * (7))
                if score < min_score:
                    min_score = score
                    best_vals = [Kp_tune, Ki_tune, Kd_tune]
    raw_vals = best_vals.copy()
    for i in range(len(best_vals)):
        best_vals[i] = best_vals[i] * mass
    return raw_vals, best_vals, min_score



@st.cache_data
def generate_best_pid(inertia_vals, thrust_vals):
    P_values_plot = []
    I_values_plot = []
    D_values_plot = []
    min_scores_plot = []
    for x in range(28):
        inertia_plot = inertia_vals[x]
        thrust_plot = thrust_vals[x]
        rough_estimate = generate_best_plot(0, 10000, 1000, 0, 10000, 1000, 0, 100000, 1000, inertia_plot, thrust_plot)
        print(rough_estimate)
        P_values_plot.append(rough_estimate[1][0])
        I_values_plot.append(rough_estimate[1][1])
        D_values_plot.append(rough_estimate[1][2])
        min_scores_plot.append(rough_estimate[2])

    return P_values_plot, I_values_plot, D_values_plot, min_scores_plot

if st.button("Recalculate"):
    st.cache_data.clear()

st.title("PID overtime")
start_time = time.time()
PID_values_plot = generate_best_pid(inertia_vals, thrust_vals)
pid_display, pid_axis = plt.subplots()
setupPIDdisplay(pid_display, pid_axis)
pid_axis.set_title("P Values")
P_values_plot = []
end_time = time.time()
st.text(PID_values_plot[0])
P_values_plot = PID_values_plot[0]
st.text("this is time taken")
st.text(end_time-start_time)
#rough_estimate = generate_best_plot(0, 10000, 1000, 0, 10000, 1000, 0, 100000, 1000, 10, 1000)
pid_axis.plot(inertia_range, P_values_plot, label = 'P', color = 'blue')
pid_axis.set_xlim(0,7)
st.pyplot(pid_display)



pid_display, pid_axis = plt.subplots()
setupPIDdisplay(pid_display, pid_axis)
pid_axis.set_title("I Values")
st.text(PID_values_plot[0])
I_values_plot = PID_values_plot[1]
#rough_estimate = generate_best_plot(0, 10000, 1000, 0, 10000, 1000, 0, 100000, 1000, 10, 1000)
pid_axis.plot(inertia_range, I_values_plot, label = 'I', color = 'red')
pid_axis.set_xlim(0,7)
st.pyplot(pid_display)

pid_display, pid_axis = plt.subplots()
setupPIDdisplay(pid_display, pid_axis)
pid_axis.set_title("D Values")
st.text(PID_values_plot[0])
D_values_plot = PID_values_plot[2]
#rough_estimate = generate_best_plot(0, 10000, 1000, 0, 10000, 1000, 0, 100000, 1000, 10, 1000)
pid_axis.plot(inertia_range, D_values_plot, label = 'D', color = 'white')
pid_axis.set_xlim(0,7)
st.pyplot(pid_display)

pid_display, pid_axis = plt.subplots()
setupPIDdisplay(pid_display, pid_axis)
pid_axis.set_title("Score Values")
st.text(PID_values_plot[0])
score_values_plot = PID_values_plot[3]
st.text(end_time-start_time)
#rough_estimate = generate_best_plot(0, 10000, 1000, 0, 10000, 1000, 0, 100000, 1000, 10, 1000)
pid_axis.plot(inertia_range, score_values_plot, label = 'Score', color = 'white')
pid_axis.set_xlim(0,7)
st.pyplot(pid_display)



