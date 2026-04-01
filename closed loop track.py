# Simulation of a single-lane track, which loops on itself.
# Numbers of cars on the track increases with time to get data for various densities.

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.colors as mcolors
import matplotlib.gridspec as gridspec

# PARAMETERS
dt = 0.1 # timestep              
road_length = 1.0 # miles

# injecting vehicles
initial_vehicles = 15 # start with free-flowing traffic
max_vehicles = 250    # fill up to total gridlock
spawn_interval = 1.5  # seconds between injecting a new car
last_spawn_time_loop = 0 

# PARAMETERS FOR MECHANICS
lookahead_dist = 0.1     
jam_gap = 0.006
high_speed_threshold = 40 
random_brake_prob = 0.05  
distraction_speed_drop = 8 

cmap = plt.get_cmap('RdYlGn') 
norm = mcolors.Normalize(vmin=0, vmax=90)

# VARIABLES
vehicles = [] 
current_time = 0

lane_speed_means = {1: 70}
lane_speed_stds = {1: 8}

# starting vehicles in loop
for i in range(initial_vehicles):
    mean = lane_speed_means[1]
    std = lane_speed_stds[1]
    desired_speed = np.clip(np.random.normal(loc=mean, scale=std), a_min=45, a_max=100)
    spawn_speed = desired_speed 
    
    reaction_time = np.random.uniform(0.75, 1.5) 
    reaction_frames = max(1, int(reaction_time / dt)) 
    
    vehicles.append({ # record vehicle characteristics
        'lane': 1, 'current_x': 1.0,
        'y': i * (road_length / initial_vehicles), 
        'speed': spawn_speed, 'desired_speed': desired_speed,
        'reaction_frames': reaction_frames, 'speed_queue': [spawn_speed] * reaction_frames 
    })

# HISTORY VARIABLES FOR GRAPHS
history_time = []
history_global_avg_speed = [] 
history_local_density = []     
history_local_avg_speed = []

num_sections = 20
section_ema_density = [0] * num_sections 
section_ema_speed = [60] * num_sections

# MOTORWAY GRAPH
def initialize_motorway_graph(ax, distance):
    left_edge, right_edge = 0.5, 1.5
    ax.axvspan(left_edge, right_edge, facecolor='lightgray', alpha=0.4)
    ax.axvline(x=left_edge, color='black', linewidth=3)
    ax.axvline(x=right_edge, color='black', linewidth=3)
    ax.set_xlim(0, 2)
    ax.set_xticks([1])
    ax.set_xticklabels(['Lane 1'])
    ax.set_ylim(0, distance)
    ax.set_ylabel('Distance (miles)')

# EACH FRAME FOR ANIMATION
def update(frame):
    global current_time, vehicles, last_spawn_time_loop
    
    current_time += dt
    ax_road.set_title(f'Closed loop (live density: {len(vehicles)} veh/mile)')
    
    # inject cars
    if len(vehicles) < max_vehicles and (current_time - last_spawn_time_loop) >= spawn_interval:
        # sort to easily find gaps
        vehicles.sort(key=lambda x: x['y'])
        
        # find the largest gap to safely spawn a new car
        max_gap = 0
        best_idx = 0
        for i in range(len(vehicles)):
            gap = (vehicles[(i + 1) % len(vehicles)]['y'] - vehicles[i]['y']) % road_length
            if gap > max_gap:
                max_gap = gap
                best_idx = i
                
        # spawn new car in the middle of the largest gap
        new_y = (vehicles[best_idx]['y'] + max_gap / 2) % road_length
        mean = lane_speed_means[1]
        std = lane_speed_stds[1]
        desired_speed = np.clip(np.random.normal(loc=mean, scale=std), a_min=45, a_max=100)
        
        # match the speed of the car behind it
        safe_spawn_speed = vehicles[best_idx]['speed']
        
        reaction_time = np.random.uniform(0.75, 1.5) 
        reaction_frames = max(1, int(reaction_time / dt)) 
        
        vehicles.append({
            'lane': 1, 'current_x': 1.0,
            'y': new_y, 
            'speed': safe_spawn_speed, 'desired_speed': desired_speed,
            'reaction_frames': reaction_frames, 'speed_queue': [safe_spawn_speed] * reaction_frames 
        })
        last_spawn_time_loop = current_time

    # MOVE EXISTING VEHICLES AND LOOP
    for v in vehicles:
        v['y'] = (v['y'] + v['speed'] * (dt / 3600)) % road_length

    # sort vehicles by pos
    vehicles.sort(key=lambda x: x['y'])

    # accel, speed 
    for i, v in enumerate(vehicles):
        max_safe_speed = v['desired_speed']
        
        car_ahead = vehicles[(i + 1) % len(vehicles)]
        gap = (car_ahead['y'] - v['y']) % road_length
        
        if gap < lookahead_dist:
            if v['speed'] > (car_ahead['speed'] + 5) and gap < (lookahead_dist * 0.6):
                approach_speed = max(0, car_ahead['speed'] + 3)
            else:
                approach_speed = v['desired_speed']
            
            actual_time_headway = v['reaction_frames'] * dt
            gap_speed = max(0, (gap - jam_gap) * (3600 / actual_time_headway)) 
            
            max_safe_speed = min(max_safe_speed, gap_speed, approach_speed)
                
        v['speed_queue'].append(max_safe_speed) 
        delayed_safe_speed = v['speed_queue'].pop(0) 

        # random braking
        if v['speed'] > high_speed_threshold and np.random.rand() < random_brake_prob:
            delayed_safe_speed = max(0, delayed_safe_speed - distraction_speed_drop)

        # emergency brake
        if gap < 0.04 and v['speed'] > 10: 
            delayed_safe_speed = min(delayed_safe_speed, max(0, car_ahead['speed'] - 10))

        accel_rate = 2 
        brake_rate = 2 
        
        if v['speed'] < 20:
            if gap < 0.01: 
                brake_rate = 15 
            else: 
                brake_rate = 0  
        else:
            if gap < 0.01: 
                brake_rate = 15
            elif gap < 0.03: 
                brake_rate = 5 
            
        if v['speed'] < delayed_safe_speed: 
            v['speed'] = min(v['speed'] + accel_rate, delayed_safe_speed)  
        elif v['speed'] > delayed_safe_speed: 
            v['speed'] = max(delayed_safe_speed, v['speed'] - brake_rate)

    # COLLECT DATA
    current_global_avg_speed = np.mean([v['speed'] for v in vehicles]) if vehicles else 0 
    
    if current_time > 0.2:
        history_time.append(current_time)
        history_global_avg_speed.append(current_global_avg_speed)
        
        section_length = road_length / num_sections 
        alpha = 0.02 
        
        for j in range(num_sections): 
            y_start = j * section_length
            y_end = (j + 1) * section_length
            
            cars_in_section = [v for v in vehicles if y_start <= v['y'] < y_end]
            instant_density = len(cars_in_section) / section_length
            instant_speed = np.mean([v['speed'] for v in cars_in_section]) if cars_in_section else section_ema_speed[j]
            
            section_ema_density[j] = (alpha * instant_density) + ((1 - alpha) * section_ema_density[j])
            section_ema_speed[j] = (alpha * instant_speed) + ((1 - alpha) * section_ema_speed[j])
            
            if section_ema_density[j] > 0.5 and int(current_time * 10) % 5 == 0: 
                history_local_density.append(section_ema_density[j])
                history_local_avg_speed.append(section_ema_speed[j])

    # UPDATE PLOTS
    if vehicles:
        vehicle_positions = np.array([[v['current_x'], v['y']] for v in vehicles]) 
        speeds = np.array([v['speed'] for v in vehicles])
        colors = cmap(norm(speeds)) 
    else:
        vehicle_positions = np.empty((0, 2)) 
        colors = []
        
    scatter.set_offsets(vehicle_positions) 
    if len(vehicles) > 0:
        scatter.set_facecolors(colors)
        scatter.set_edgecolors('black') 

    line_speed.set_data(history_time, history_global_avg_speed) 
    if history_time:
        ax_speed.set_xlim(0, max(60, history_time[-1] + 5)) 
        
    if history_local_density:
        scatter_plot_points = np.c_[history_local_density, history_local_avg_speed] 
        scatter_density.set_offsets(scatter_plot_points) 

    return scatter, line_speed, scatter_density 

# FIGURES AND ANIMATION
fig = plt.figure(figsize=(14, 8)) 
gs = gridspec.GridSpec(2, 3, width_ratios=[1.2, 2, 2]) 

ax_road = fig.add_subplot(gs[:, 0]) 
initialize_motorway_graph(ax_road, road_length)
scatter = ax_road.scatter([], [], s=60, marker='s', zorder=5) 

# speed time graph
ax_speed = fig.add_subplot(gs[0, 1:]) 
line_speed, = ax_speed.plot([], [], lw=2, color='blue')
ax_speed.set_title('Global average speed')
ax_speed.set_xlabel('Time (s)')
ax_speed.set_ylabel('Average speed (mph)')
ax_speed.set_ylim(0, 130)  
ax_speed.grid(True, linestyle='--', alpha=0.6)

# speed density graph
ax_density = fig.add_subplot(gs[1, 1:])
scatter_density = ax_density.scatter([], [], s=12, color='red', marker='+', alpha=0.15) 
ax_density.set_title('Speed–density scatter plot')
ax_density.set_xlabel('Density (veh/mile)')
ax_density.set_ylabel('Speed (mph)')
ax_density.set_xlim(0, 350) 
ax_density.set_ylim(0, 130) 
ax_density.grid(True, linestyle='--', alpha=0.6)

ani = animation.FuncAnimation(fig, update, interval=50, blit=False, cache_frame_data=False) 
plt.tight_layout()
plt.show()