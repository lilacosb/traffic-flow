# Models the closing of the fast lane on a three-lane motorway.
# Forced yielding for vehicles in lanes 1 and 2

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.colors as mcolors
import matplotlib.gridspec as gridspec

# PARAMETERS
dt = 0.1 # timestep             
safe_spawn_sec = 1.8
road_length = 1 # miles

blockage_start = 0.8
blockage_end = 1.0

# PARAMETERS FOR MECHANICS
lookahead_dist = 0.1     
min_physical_gap = 0.006
safe_time_gap_ahead = 1 
safe_time_gap_behind = 1.5
yield_time_gap = 1.5 # yielding cars to merges leave this time gap

lateral_speed = 0.8      
lane_change_cooldown = 5 
signal_duration = 0.75   

# random braking
high_speed_threshold = 65 
random_brake_prob = 0.05  
distraction_speed_drop = 5 
congestion_threshold = 40 # threshold to decide if lane is congested

cmap = plt.get_cmap('RdYlGn') 
norm = mcolors.Normalize(vmin=0, vmax=120)

# VARIABLES
vehicles = [] 
last_spawn_time = {1: -10, 2: -10, 3: -10} 
current_time = 0

lane_speed_means = {1: 60, 2: 70, 3: 90}
lane_speed_stds = {1: 8, 2: 10, 3: 12}

# HISTORY VARIABLES FOR GRAPHS
history_time = []
history_global_avg_speed = [] 
history_lane1_avg_speed = []
history_lane2_avg_speed = []
history_lane3_avg_speed = []

history_flow = []
history_local_density = []     
history_local_avg_speed = []

num_sections = 20
section_ema_density = [0] * num_sections 
section_ema_speed = [60] * num_sections

# MOTORWAY GRAPH
def initialize_motorway_graph(ax, distance):
    left_edge, lane_1_2, lane_2_3, right_edge = 0.5, 1.5, 2.5, 3.5
    ax.axvspan(left_edge, right_edge, facecolor='lightgray', alpha=0.4)
    ax.axvline(x=left_edge, color='black', linewidth=3)
    ax.axvline(x=right_edge, color='black', linewidth=3)
    ax.axvline(x=lane_1_2, color='white', linestyle='--', linewidth=2)
    ax.axvline(x=lane_2_3, color='white', linestyle='--', linewidth=2)
    
    # lane 3 blockage
    ax.axvspan(lane_2_3, right_edge, ymin=blockage_start/distance, ymax=blockage_end/distance, facecolor='red', hatch='//', alpha=0.6)
    ax.text((lane_2_3 + right_edge)/2, (blockage_start + blockage_end)/2, 'LANE\nCLOSED', ha='center', va='center', color='black', weight='bold', fontsize=8)

    ax.set_xlim(0, 4)
    ax.set_xticks([1, 2, 3])
    ax.set_xticklabels(['Lane 1', 'Lane 2', 'Lane 3'])
    ax.set_ylim(0, distance)
    ax.set_ylabel('Distance (miles)')

# EACH FRAME FOR ANIMATION
def update(frame):
    global current_time, vehicles
    
    current_time += dt
    
    wave_duration = 120 
    cycle = (current_time % wave_duration) / wave_duration 
    raw_wave = 3000 + 2000 * np.sin(cycle * 2 * np.pi) 
    q_current = np.clip(raw_wave, a_min=1000, a_max=5000) 
    ax_road.set_title(f'Motorway (Live flow: {int(q_current)} veh/hr)')
    
    # MOVE EXISTING VEHICLES
    for v in vehicles:
        v['y'] += v['speed'] * (dt / 3600) # move forward
        if v['current_x'] < v['target_x']: # merge right
            v['current_x'] = min(v['current_x'] + lateral_speed * dt, v['target_x'])
        elif v['current_x'] > v['target_x']: # merge left
            v['current_x'] = max(v['current_x'] - lateral_speed * dt, v['target_x'])
            
    vehicles = [v for v in vehicles if v['y'] <= road_length] 
    
    # SIGNALLING
    for v in vehicles:
        if v['current_x'] != v['target_x']:
            v['signal_dir'] = 0 
            continue
            
        can_change_lane = (current_time - v['last_change_time']) >= lane_change_cooldown 
        
        # lane 3 must merge
        if v['lane'] == 3 and v['y'] < blockage_start:
            dist_to_blockage = blockage_start - v['y']
            
            if dist_to_blockage < 0.25 and can_change_lane: 
                left_lane = 2
                left_lane_cars = [other for other in vehicles if other['lane'] == left_lane]
                
                car_alongside = False
                perfect_gap = True
                acceptable_gap = True
                
                for other in left_lane_cars:
                    gap = abs(other['y'] - v['y'])
                    if gap < min_physical_gap:
                        car_alongside = True; perfect_gap = False; acceptable_gap = False
                        break   
                        
                    if other['y'] >= v['y']:
                        time_gap = gap / (max(v['speed'], 1.8) / 3600)
                        if time_gap < safe_time_gap_ahead * 1.5: 
                            perfect_gap = False
                        if time_gap < (safe_time_gap_ahead * 0.8): 
                            acceptable_gap = False
                    else:
                        time_gap = gap / (max(other['speed'], 1.8) / 3600)
                        if time_gap < safe_time_gap_behind * 2: 
                            perfect_gap = False
                        if time_gap < (safe_time_gap_behind * 0.6): 
                            acceptable_gap = False

                if not car_alongside and (perfect_gap or acceptable_gap):
                    if v['signal_dir'] != -1:
                        v['signal_dir'] = -1
                        v['signal_start'] = current_time
                    elif (current_time - v['signal_start']) >= signal_duration:
                        v['lane'] = left_lane
                        v['target_x'] = float(left_lane)
                        v['last_change_time'] = current_time
                        v['signal_dir'] = 0
                else:
                    if v['signal_dir'] != -1:
                        v['signal_dir'] = -1
                        v['signal_start'] = current_time
            else:
                v['signal_dir'] = 0
                
        # lane 2 merge to lane 1
        elif v['lane'] == 2 and can_change_lane:
            same_lane_ahead = [other for other in vehicles if other['lane'] == 2 and other['y'] > v['y']]
            if same_lane_ahead:
                car_ahead = min(same_lane_ahead, key=lambda x: x['y'])
                distance_ahead = car_ahead['y'] - v['y']
                
                if distance_ahead < (lookahead_dist * 1.5) and car_ahead['speed'] < congestion_threshold:
                    left_lane = 1
                    left_lane_cars = [other for other in vehicles if other['lane'] == left_lane]
                    
                    lane_1_is_better = True
                    left_ahead = [other for other in left_lane_cars if other['y'] > v['y'] and (other['y'] - v['y']) < lookahead_dist * 2]
                    
                    if left_ahead:
                        left_car_ahead = min(left_ahead, key=lambda x: x['y'])
                        if left_car_ahead['speed'] <= car_ahead['speed'] + 5: 
                            lane_1_is_better = False
                            
                    if lane_1_is_better:
                        car_alongside = False
                        perfect_gap = True
                        acceptable_gap = True
                        
                        for other in left_lane_cars:
                            gap = abs(other['y'] - v['y'])
                            if gap < min_physical_gap:
                                car_alongside = True; perfect_gap = False; acceptable_gap = False
                                break
                            if other['y'] >= v['y']:
                                time_gap = gap / (max(v['speed'], 1.8) / 3600)
                                if time_gap < safe_time_gap_ahead * 1.5: 
                                    perfect_gap = False
                                if time_gap < (safe_time_gap_ahead * 0.8): 
                                    acceptable_gap = False
                            else:
                                time_gap = gap / (max(other['speed'], 1.8) / 3600)
                                if time_gap < safe_time_gap_behind * 2: 
                                    perfect_gap = False
                                if time_gap < (safe_time_gap_behind * 0.6): 
                                    acceptable_gap = False

                        if car_alongside == False and perfect_gap == False and acceptable_gap == False:
                            v['signal_dir'] = 0 
                        elif v['signal_dir'] == -1:
                            if acceptable_gap and (current_time - v['signal_start']) >= signal_duration:
                                v['lane'] = left_lane
                                v['target_x'] = float(left_lane)
                                v['last_change_time'] = current_time
                                v['signal_dir'] = 0
                            elif (current_time - v['signal_start']) > 6:
                                v['signal_dir'] = 0
                                v['last_change_time'] = current_time
                        elif perfect_gap:
                            v['signal_dir'] = -1
                            v['signal_start'] = current_time
                        else:
                            v['signal_dir'] = 0
                    else:
                        v['signal_dir'] = 0
                else:
                    v['signal_dir'] = 0
            else:
                v['signal_dir'] = 0
        else:
            v['signal_dir'] = 0

    # SPAWN NEW VEHICLES
    prob_arrival = (q_current * dt) / 3600
    if np.random.rand() < prob_arrival:
        valid_lanes = []
        for lane in [1, 2, 3]:
            time_ok = (current_time - last_spawn_time[lane]) >= safe_spawn_sec 
            lane_cars = [v for v in vehicles if v['lane'] == lane]
            space_ok = True
            if lane_cars:
                closest_car = min(lane_cars, key=lambda x: x['y'])
                if closest_car['y'] < 0.01: 
                    space_ok = False 
            if time_ok and space_ok:
                valid_lanes.append(lane)
                
        if valid_lanes:
            weights = {1: 0.5, 2: 0.3, 3: 0.2} 
            valid_weights = [weights[lane] for lane in valid_lanes]
            chosen_lane = np.random.choice(valid_lanes, p=[w / sum(valid_weights) for w in valid_weights]) 
            
            lane_cars = [v for v in vehicles if v['lane'] == chosen_lane]
            
            mean = lane_speed_means[chosen_lane]
            std = lane_speed_stds[chosen_lane]
            desired_speed = np.clip(np.random.normal(loc=mean, scale=std), a_min=55, a_max=130) 
            
            if lane_cars:
                closest_car = min(lane_cars, key=lambda x: x['y'])
                spawn_speed = max(10, closest_car['speed'] + 5)  
            else:
                spawn_speed = desired_speed
            
            reaction_time = np.random.uniform(0.75, 1.5) 
            reaction_frames = max(1, int(reaction_time / dt)) 
            
            vehicles.append({
                'lane': chosen_lane, 'current_x': float(chosen_lane), 'target_x': float(chosen_lane),
                'y': 0, 'speed': spawn_speed, 'desired_speed': desired_speed,
                'last_change_time': current_time, 
                'signal_dir': 0, 'signal_start': 0,
                'reaction_frames': reaction_frames, 'speed_queue': [spawn_speed] * reaction_frames 
            })
            last_spawn_time[chosen_lane] = current_time

    # acceleration and speed
    jam_gap = 0.006 
    for v in vehicles:
        max_safe_speed = v['desired_speed']
        
        same_lane_ahead = [other for other in vehicles if other['lane'] == v['lane'] and other['y'] > v['y']]
        
        if v['lane'] == 3 and v['y'] < blockage_start:
            same_lane_ahead.append({'y': blockage_start, 'speed': 0, 'lane': 3})
        
        if same_lane_ahead:
            car_ahead = min(same_lane_ahead, key=lambda x: x['y'])
            gap = car_ahead['y'] - v['y']
            
            if gap < lookahead_dist:
                if v['speed'] > (car_ahead['speed'] + 5) and gap < (lookahead_dist * 0.6):
                    approach_speed = max(0, car_ahead['speed'] + 3) 
                else:
                    approach_speed = v['desired_speed']
                
                actual_time_headway = v['reaction_frames'] * dt
                gap_speed = max(0, (gap - jam_gap) * (3600 / actual_time_headway)) 
                
                max_safe_speed = min(max_safe_speed, gap_speed, approach_speed)
                
        yielding_cars = [other for other in vehicles 
                         if other['y'] > v['y'] and (other['y'] - v['y']) < (lookahead_dist * 1.1)
                         and (other['lane'] + other['signal_dir'] == v['lane'])]
        
        if yielding_cars:
            yield_car = min(yielding_cars, key=lambda x: x['y'])
            current_gap = yield_car['y'] - v['y']
            time_gap = current_gap / (max(v['speed'], 1) / 3600)
            if time_gap < yield_time_gap: 
                max_safe_speed = min(max_safe_speed, max(0, yield_car['speed'] - 10))

        if v['signal_dir'] != 0:
            target_lane = v['lane'] + v['signal_dir']
            target_lane_cars = [other for other in vehicles if other['lane'] == target_lane]
            target_ahead = [other for other in target_lane_cars if other['y'] > v['y']] 
            if target_ahead:
                target_car_ahead = min(target_ahead, key=lambda x: x['y'])
                if (target_car_ahead['y'] - v['y']) < lookahead_dist:
                    max_safe_speed = min(max_safe_speed, target_car_ahead['speed']) 
            for other in target_lane_cars:
                gap = abs(other['y'] - v['y'])
                if gap < min_physical_gap: 
                    if other['y'] >= v['y']: 
                        max_safe_speed = min(max_safe_speed, max(0, v['speed'] - 5))
                    continue
                if other['y'] >= v['y']:
                    time_gap = gap / (max(v['speed'], 1) / 3600)
                    if time_gap < safe_time_gap_ahead:
                        max_safe_speed = min(max_safe_speed, max(0, v['speed'] - 5))
                    
        v['speed_queue'].append(max_safe_speed) 
        delayed_safe_speed = v['speed_queue'].pop(0) 

        # RANDOM BRAKING
        if v['speed'] > high_speed_threshold and np.random.rand() < random_brake_prob:
            delayed_safe_speed = max(0, delayed_safe_speed - distraction_speed_drop)

        if same_lane_ahead:
            car_ahead = min(same_lane_ahead, key=lambda x: x['y'])
            gap = car_ahead['y'] - v['y']
            if gap < 0.04 and v['speed'] > 10: 
                delayed_safe_speed = min(delayed_safe_speed, max(0, v['speed'] - 10))

        accel_rate = 4 if v['signal_dir'] != 0 else 2 
        
        brake_rate = 2 
        if same_lane_ahead:
            car_ahead = min(same_lane_ahead, key=lambda x: x['y'])
            gap = car_ahead['y'] - v['y']
            
            if v['speed'] < 20:
                if gap < 0.01: brake_rate = 15 
                else: brake_rate = 0  
            else:
                if gap < 0.01: brake_rate = 15
                elif gap < 0.03: brake_rate = 5 
                
        if v['speed'] < delayed_safe_speed: 
            v['speed'] = min(v['speed'] + accel_rate, delayed_safe_speed)  
        elif v['speed'] > delayed_safe_speed: 
            v['speed'] = max(delayed_safe_speed, v['speed'] - brake_rate)

    # COLLECT DATA
    current_global_avg_speed = np.mean([v['speed'] for v in vehicles]) if vehicles else 0 
    
    # speed in each lane
    l1_speeds = [v['speed'] for v in vehicles if v['lane'] == 1]
    l2_speeds = [v['speed'] for v in vehicles if v['lane'] == 2]
    l3_speeds = [v['speed'] for v in vehicles if v['lane'] == 3]
    
    current_l1_avg = np.mean(l1_speeds) if l1_speeds else np.nan
    current_l2_avg = np.mean(l2_speeds) if l2_speeds else np.nan
    current_l3_avg = np.mean(l3_speeds) if l3_speeds else np.nan
    
    if current_time > 0.5:
        history_time.append(current_time)
        history_global_avg_speed.append(current_global_avg_speed)
        history_lane1_avg_speed.append(current_l1_avg)
        history_lane2_avg_speed.append(current_l2_avg)
        history_lane3_avg_speed.append(current_l3_avg)
        history_flow.append(q_current)
        
        section_length = road_length / num_sections 
        alpha = 0.15 
        
        for i in range(num_sections): 
            y_start = i * section_length
            y_end = (i + 1) * section_length
            
            cars_in_section = [v for v in vehicles if y_start <= v['y'] < y_end]
            instant_density = len(cars_in_section) / section_length
            instant_speed = np.mean([v['speed'] for v in cars_in_section]) if cars_in_section else section_ema_speed[i]
            
            section_ema_density[i] = (alpha * instant_density) + ((1 - alpha) * section_ema_density[i])
            section_ema_speed[i] = (alpha * instant_speed) + ((1 - alpha) * section_ema_speed[i])
            
            if section_ema_density[i] > 0.5 and int(current_time * 10) % 5 == 0: 
                history_local_density.append(section_ema_density[i])
                history_local_avg_speed.append(section_ema_speed[i])

    # UPDATE PLOTS
    indicator_positions = [] 
    blink_on = int(current_time * 4) % 2 == 0 

    if vehicles:
        vehicle_positions = np.array([[v['current_x'], v['y']] for v in vehicles]) 
        speeds = np.array([v['speed'] for v in vehicles])
        colors = cmap(norm(speeds)) 
        if blink_on: 
            for v in vehicles:
                if v['signal_dir'] == 1:
                    indicator_positions.append([v['current_x'] + 0.15, v['y']])
                elif v['signal_dir'] == -1:
                    indicator_positions.append([v['current_x'] - 0.15, v['y']])
    else:
        vehicle_positions = np.empty((0, 2)) 
        colors = []
        
    scatter.set_offsets(vehicle_positions) 
    if len(vehicles) > 0:
        scatter.set_facecolors(colors)
        scatter.set_edgecolors('black') 
        
    if indicator_positions: 
        indicator_scatter.set_offsets(indicator_positions)
    else:
        indicator_scatter.set_offsets(np.empty((0, 2)))

    # update graphs
    line_global_speed.set_data(history_time, history_global_avg_speed) 
    line_lane1_speed.set_data(history_time, history_lane1_avg_speed)
    line_lane2_speed.set_data(history_time, history_lane2_avg_speed)
    line_lane3_speed.set_data(history_time, history_lane3_avg_speed)
    line_flow.set_data(history_time, history_flow)
    
    if history_time:
        ax_speed.set_xlim(0.5, max(60, history_time[-1] + 5)) 
        
    if history_local_density:
        scatter_plot_points = np.c_[history_local_density, history_local_avg_speed] 
        scatter_density.set_offsets(scatter_plot_points) 

    return scatter, indicator_scatter, line_global_speed, line_lane1_speed, line_lane2_speed, line_lane3_speed, scatter_density, line_flow

# FIGURES AND ANIMATION
fig = plt.figure(figsize=(14, 8)) 
gs = gridspec.GridSpec(2, 3, width_ratios=[1.2, 2, 2]) 

ax_road = fig.add_subplot(gs[:, 0]) 
initialize_motorway_graph(ax_road, road_length)
scatter = ax_road.scatter([], [], s=60, marker='s', zorder=5) 
indicator_scatter = ax_road.scatter([], [], c='orange', s=30, marker='o', zorder=6) 

# speed time graph
ax_speed = fig.add_subplot(gs[0, 1:]) 

line_global_speed, = ax_speed.plot([], [], lw=3, color='black', label='Global average')
line_lane1_speed, = ax_speed.plot([], [], lw=1.5, color='green', alpha=0.8, label='Lane 1')
line_lane2_speed, = ax_speed.plot([], [], lw=1.5, color='orange', alpha=0.8, label='Lane 2')
line_lane3_speed, = ax_speed.plot([], [], lw=1.5, color='red', alpha=0.8, label='Lane 3')

ax_speed.set_title('Average speeds by lane')
ax_speed.set_xlabel('Time (s)')
ax_speed.set_ylabel('Speed (mph)')
ax_speed.set_ylim(0, 130)  
ax_speed.grid(True, linestyle='--', alpha=0.6)
ax_speed.legend(loc='lower left') 

ax_flow = ax_speed.twinx() 
line_flow, = ax_flow.plot([], [], lw=3, color='black', alpha=0.3, label='Live flow') 
ax_flow.set_ylabel('Live flow (veh/hr)')
ax_flow.set_ylim(0, 6000)

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