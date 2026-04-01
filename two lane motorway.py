# Simulation of a two-lane motorway.

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.colors as mcolors
import matplotlib.gridspec as gridspec

# PARAMETERS
dt = 0.1 # timestep              
safe_spawn_sec = 1.5
road_length = 1 # miles

# PARAMETERS FOR MECHANICS
lookahead_dist = 0.1     
min_physical_gap = 0.006
safe_time_gap_ahead = 0.6 
safe_time_gap_behind = 1 
yield_time_gap = 1.5 # yielding cars to merges leave this time gap

lateral_speed = 0.8      
lane_change_cooldown = 5 # cannot change lanes too often
signal_duration = 0.75   

overtake_speed_advantage = 2 # must be faster than car in front
congestion_threshold = 45 # can undertake below this speed

# random braking
high_speed_threshold = 50 # speeds above this are susceptible to random distracted braking
random_brake_prob = 0.05  # 5% chance per timestep of easing off the accelerator
distraction_speed_drop = 5 # mph drop when distracted

cmap = plt.get_cmap('RdYlGn') # colour map based on speed
norm = mcolors.Normalize(vmin=0, vmax=120)

# VARIABLES
vehicles = [] # lists all present vehicles
last_spawn_time = {1: -10, 2: -10} 
current_time = 0

lane_speed_means = {1: 65, 2: 75}
lane_speed_stds = {1: 8, 2: 10}

# HISTORY VARIABLES FOR GRAPHS
history_time = []
history_global_avg_speed = [] 
history_local_density = []     
history_local_avg_speed = []
history_flow = []

num_sections = 20
section_ema_density = [0] * num_sections # ema considers previous data, starting memory bank for each section
section_ema_speed = [60] * num_sections

# MOTORWAY GRAPH
def initialize_motorway_graph(ax, distance):
    left_edge, lane_1_2, right_edge = 0.5, 1.5, 2.5
    ax.axvspan(left_edge, right_edge, facecolor='lightgray', alpha=0.4)
    ax.axvline(x=left_edge, color='black', linewidth=3)
    ax.axvline(x=right_edge, color='black', linewidth=3)
    ax.axvline(x=lane_1_2, color='white', linestyle='--', linewidth=2)
    ax.set_xlim(0, 3)
    ax.set_xticks([1, 2])
    ax.set_xticklabels(['Lane 1', 'Lane 2'])
    ax.set_ylim(0, distance)
    ax.set_ylabel('Distance (miles)')

# EACH FRAME FOR ANIMATION
def update(frame):
    global current_time, vehicles
    
    current_time += dt
    
    # inflow of vehicles as a sine wave to get varied density values
    wave_duration = 120 
    cycle = (current_time % wave_duration) / wave_duration 
    raw_wave = 3500 + 2500 * np.sin(cycle * 2 * np.pi) 
    q_current = np.clip(raw_wave, a_min=1000, a_max=6000) # cap lower values so not negative
    ax_road.set_title(f'Motorway (live flow: {int(q_current)} veh/hr)')
    
    # MOVE EXISTING VEHICLES
    for v in vehicles:
        v['y'] += v['speed'] * (dt / 3600) # move forward
        if v['current_x'] < v['target_x']: # overtake
            v['current_x'] = min(v['current_x'] + lateral_speed * dt, v['target_x'])
        elif v['current_x'] > v['target_x']: # merge left
            v['current_x'] = max(v['current_x'] - lateral_speed * dt, v['target_x'])
            
    vehicles = [v for v in vehicles if v['y'] <= road_length] # update vehicles list
    
    # SIGNALLING
    for v in vehicles:
        if v['current_x'] != v['target_x']:
            v['signal_dir'] = 0 
            continue
            
        can_change_lane = (current_time - v['last_change_time']) >= lane_change_cooldown # timer cooldown
        same_lane_ahead = [other for other in vehicles if other['lane'] == v['lane'] and other['y'] > v['y']] # considers vehicles ahead in same lane
        wants_to_overtake = False # default
        
        # OVERTAKING
        if same_lane_ahead:
            car_ahead = min(same_lane_ahead, key=lambda x: x['y']) # every car in lane ahead, then finds closest
            distance_ahead = car_ahead['y'] - v['y']
            
            if distance_ahead < (lookahead_dist * 0.7) and v['desired_speed'] > (car_ahead['speed'] + overtake_speed_advantage):
                wants_to_overtake = True  # if close enough and wants to go fast enough
                
                if v['lane'] < 2 and can_change_lane: # able to overtake?
                    target_lane = v['lane'] + 1
                    target_lane_cars = [other for other in vehicles if other['lane'] == target_lane]
                    
                    target_lane_is_better = True
                    target_lane_ahead = [other for other in target_lane_cars if other['y'] > v['y'] and (other['y'] - v['y']) < (lookahead_dist * 10)]
                    # cars further ahead in target lane
                    
                    if target_lane_ahead:
                        target_bottleneck_speed = min([other['speed'] for other in target_lane_ahead])
                        if target_bottleneck_speed <= (car_ahead['speed'] + 4):
                            target_lane_is_better = False # does not want to overtake if there are slower cars ahead in target lane

                    if target_lane_is_better:
                        car_alongside = False # car alongside in next lane?
                        perfect_gap = True # big enough gap between vehicles in next lane for space to merge
                        acceptable_gap = True # smaller gap after singalling timer up
                        
                        for other in target_lane_cars:
                            gap = abs(other['y'] - v['y'])
                            
                            if gap < min_physical_gap: # for cars to not be touching
                                car_alongside = True
                                perfect_gap = False
                                acceptable_gap = False
                                break
                                
                            if other['y'] >= v['y']: # condition to merge behind
                                time_gap = gap / (max(v['speed'], 1.8) / 3600)
                                if time_gap < safe_time_gap_ahead * 1.5: perfect_gap = False
                                if time_gap < (safe_time_gap_ahead * 0.8): acceptable_gap = False
                            else: # merge ahead
                                time_gap = gap / (max(other['speed'], 1.8) / 3600)
                                if time_gap < safe_time_gap_behind * 2: perfect_gap = False
                                if time_gap < (safe_time_gap_behind * 0.6): acceptable_gap = False

                        if car_alongside == False and perfect_gap == False and acceptable_gap == False:
                            v['signal_dir'] = 0 # keep indicator off if cannot merge
                        elif v['signal_dir'] == 1:
                            if acceptable_gap and (current_time - v['signal_start']) >= signal_duration:
                                v['lane'] = target_lane # change lane if there is still a gap and minimum signalling time up
                                v['target_x'] = float(target_lane)
                                v['last_change_time'] = current_time # reset change lane timer
                                v['signal_dir'] = 0 
                            elif (current_time - v['signal_start']) > 6: # gives up
                                v['signal_dir'] = 0
                                v['last_change_time'] = current_time
                        elif perfect_gap:
                            v['signal_dir'] = 1
                            v['signal_start'] = current_time
                        else:
                            v['signal_dir'] = 0
                    else:
                        v['signal_dir'] = 0
                        wants_to_overtake = False 
                else:
                    v['signal_dir'] = 0
                    
        catching_up = False # catching up to vehicle ahead in same lane?
        if same_lane_ahead:
            car_ahead = min(same_lane_ahead, key=lambda x: x['y'])
            if (car_ahead['y'] - v['y']) < 0.1 and v['desired_speed'] > (car_ahead['speed'] + 2.5):
                catching_up = True
                
        # MERGE LEFT
        if not wants_to_overtake:
            if catching_up:
                if v['signal_dir'] == -1:
                    v['signal_dir'] = 0 
            else:
                if v['signal_dir'] == 1:
                    v['signal_dir'] = 0 
                if v['lane'] > 1 and can_change_lane: # can merge left?
                    left_lane = v['lane'] - 1
                    left_lane_cars = [other for other in vehicles if other['lane'] == left_lane]
                    
                    clear_ahead_left = True
                    for other in left_lane_cars:
                        if other['y'] > v['y'] and (other['y'] - v['y']) < (lookahead_dist * 1.3):
                            if v['speed'] > (other['speed'] + 1): 
                                clear_ahead_left = False; break  # not clear if going faster than close cars in left lane
                                
                    # same logic as for overtaking
                    if clear_ahead_left:
                        car_alongside = False
                        perfect_gap = True
                        acceptable_gap = True
                        
                        for other in left_lane_cars:
                            gap = abs(other['y'] - v['y'])
                            
                            if gap < min_physical_gap:
                                car_alongside = True
                                perfect_gap = False
                                acceptable_gap = False
                                break   
                                
                            if other['y'] >= v['y']:
                                time_gap = gap / (max(v['speed'], 1) / 3600)
                                if time_gap < safe_time_gap_ahead * 1.5:
                                    perfect_gap = False
                                if time_gap < (safe_time_gap_ahead * 0.8):
                                    acceptable_gap = False
                            else:
                                time_gap = gap / (max(other['speed'], 1) / 3600)
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

    # SPAWN NEW VEHICLES (Updated for 2 lanes)
    prob_arrival = (q_current * dt) / 3600
    if np.random.rand() < prob_arrival:
        valid_lanes = []
        for lane in [1, 2]:
            time_ok = (current_time - last_spawn_time[lane]) >= safe_spawn_sec # can't spawn too soon after previous spawn
            lane_cars = [v for v in vehicles if v['lane'] == lane]
            space_ok = True
            if lane_cars:
                closest_car = min(lane_cars, key=lambda x: x['y'])
                if closest_car['y'] < 0.01: 
                    space_ok = False # cannot spawn too close to car ahead
            if time_ok and space_ok:
                valid_lanes.append(lane)
                
        if valid_lanes:
            weights = {1: 0.7, 2: 0.3} # likelihoods of spawning in each lane
            valid_weights = [weights[lane] for lane in valid_lanes]
            chosen_lane = np.random.choice(valid_lanes, p=[w / sum(valid_weights) for w in valid_weights]) # makes weights probability sum = 100%, scaled
            
            lane_cars = [v for v in vehicles if v['lane'] == chosen_lane]
            
            # assigns speed to new vehicle
            mean = lane_speed_means[chosen_lane]
            std = lane_speed_stds[chosen_lane]
            desired_speed = np.clip(np.random.normal(loc=mean, scale=std), a_min=55, a_max=130) # capped desired speeds
            
            if lane_cars:
                closest_car = min(lane_cars, key=lambda x: x['y'])
                spawn_speed = max(10, closest_car['speed'] + 5)  # do not spawn at speed much faster than car ahead
            else:
                spawn_speed = desired_speed
            
            reaction_time = np.random.uniform(0.75, 1.5) # each veh assigned a reaction time
            reaction_frames = max(1, int(reaction_time / dt)) # reaction for each timestep
            
            # add the new vehicle in dataset
            vehicles.append({
                'lane': chosen_lane, 'current_x': float(chosen_lane), 'target_x': float(chosen_lane),
                'y': 0, 'speed': spawn_speed, 'desired_speed': desired_speed,
                'last_change_time': current_time, 
                'signal_dir': 0, 'signal_start': 0,
                'reaction_frames': reaction_frames, 'speed_queue': [spawn_speed] * reaction_frames 
            })
            last_spawn_time[chosen_lane] = current_time

    # ACCELERATION AND SPEED BY INTERACTION (Updated for 2 lanes)
    jam_gap = 0.006 
    for v in vehicles:
        max_safe_speed = v['desired_speed']
        
        if v['lane'] == 2: # go faster when pressure behind (now lane 2 is the fast lane)
            same_lane_behind = [other for other in vehicles if other['lane'] == v['lane'] and other['y'] < v['y']]
            if same_lane_behind:
                car_behind = max(same_lane_behind, key=lambda x: x['y'])
                distance_behind = v['y'] - car_behind['y']
                if distance_behind < (lookahead_dist * 1.5):
                    speed_boost = max(v['desired_speed'] + 5, car_behind['speed'] + 2)
                    max_safe_speed = min(120, speed_boost) 
        
        same_lane_ahead = [other for other in vehicles if other['lane'] == v['lane'] and other['y'] > v['y']]
        if same_lane_ahead:
            car_ahead = min(same_lane_ahead, key=lambda x: x['y'])
            gap = car_ahead['y'] - v['y']
            
            if gap < lookahead_dist:
                if v['speed'] > (car_ahead['speed'] + 5) and gap < (lookahead_dist * 0.6):
                    approach_speed = max(0, car_ahead['speed'] + 3) # cap speed when car close ahead
                else:
                    approach_speed = v['desired_speed']
                
                actual_time_headway = v['reaction_frames'] * dt
                gap_speed = max(0, (gap - jam_gap) * (3600 / actual_time_headway)) # based on reaction time, maximum speed with enough room to stop before crashing
                
                max_safe_speed = min(max_safe_speed, gap_speed, approach_speed)
                
        if v['lane'] < 2: # can undertake if in congestion
            right_lane_ahead = [other for other in vehicles if other['lane'] == v['lane'] + 1 and other['y'] > v['y']]
            if right_lane_ahead:
                right_car = min(right_lane_ahead, key=lambda x: x['y'])
                if (right_car['y'] - v['y']) < (lookahead_dist * 1.3):
                    if right_car['speed'] >= congestion_threshold:
                        max_safe_speed = min(max_safe_speed, right_car['speed'])

        
        yielding_cars = [other for other in vehicles # if car nearby indicating towards their lane
                         if other['y'] > v['y'] and (other['y'] - v['y']) < (lookahead_dist * 1.1)
                         and (other['lane'] + other['signal_dir'] == v['lane'])]
        
        # yielding cars leave more room when car is indicating towards their lane
        if yielding_cars:
            yield_car = min(yielding_cars, key=lambda x: x['y'])
            current_gap = yield_car['y'] - v['y']
            time_gap = current_gap / (max(v['speed'], 1) / 3600)
            if time_gap < yield_time_gap: # if not enough space then slow down to make space
                max_safe_speed = min(max_safe_speed, max(0, yield_car['speed'] - 10))

        if v['signal_dir'] != 0:
            target_lane = v['lane'] + v['signal_dir']
            target_lane_cars = [other for other in vehicles if other['lane'] == target_lane]
            target_ahead = [other for other in target_lane_cars if other['y'] > v['y']] # car ahead of where they want to merge
            if target_ahead:
                target_car_ahead = min(target_ahead, key=lambda x: x['y'])
                if (target_car_ahead['y'] - v['y']) < lookahead_dist:
                    max_safe_speed = min(max_safe_speed, target_car_ahead['speed']) # matches speed of target car
            for other in target_lane_cars:
                gap = abs(other['y'] - v['y'])
                if gap < min_physical_gap: # fall back if car directly alongside
                    if other['y'] >= v['y']: 
                        max_safe_speed = min(max_safe_speed, max(0, v['speed'] - 5))
                    continue
                if other['y'] >= v['y']:
                    time_gap = gap / (max(v['speed'], 1) / 3600)
                    if time_gap < safe_time_gap_ahead:
                        max_safe_speed = min(max_safe_speed, max(0, v['speed'] - 5))
                    
        v['speed_queue'].append(max_safe_speed) # list of speeds calculated for a vehicle, based on reaction time
        delayed_safe_speed = v['speed_queue'].pop(0) # the speed calculated before the reaction

        # RANDOM BRAKING
        if v['speed'] > high_speed_threshold and np.random.rand() < random_brake_prob:
            # driver gets distracted and eases off the pedal
            delayed_safe_speed = max(0, delayed_safe_speed - distraction_speed_drop)

        if same_lane_ahead:
            car_ahead = min(same_lane_ahead, key=lambda x: x['y'])
            gap = car_ahead['y'] - v['y']
            if gap < 0.04 and v['speed'] > 10: # when too close at fast enough speed, ignore reaction, brake harshly
                delayed_safe_speed = min(delayed_safe_speed, max(0, v['speed'] - 10))

        accel_rate = 4 if v['signal_dir'] != 0 else 2 # accelerate more to go into target lane, otherwise natural acceleration
        
        # harsh braking conditions
        brake_rate = 2 
        if same_lane_ahead:
            car_ahead = min(same_lane_ahead, key=lambda x: x['y'])
            gap = car_ahead['y'] - v['y']
            
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
                
        if v['speed'] < delayed_safe_speed: # accelerate if going slower than target speed
            v['speed'] = min(v['speed'] + accel_rate, delayed_safe_speed)  
        elif v['speed'] > delayed_safe_speed: # otherwise brake
            v['speed'] = max(delayed_safe_speed, v['speed'] - brake_rate)

    # COLLECT DATA
    current_global_avg_speed = np.mean([v['speed'] for v in vehicles]) if vehicles else 0 # avg speed across all veh
    
    if current_time > 0.2:
        history_time.append(current_time)
        history_global_avg_speed.append(current_global_avg_speed)
        history_flow.append(q_current)
        
        section_length = road_length / num_sections # divide road to find densities
    
        alpha = 0.10 # data points made up of 20% current reality, 80% historicla memory - prevents big spikes from frame to frame
        
        for i in range(num_sections): # calculations for each section
            y_start = i * section_length
            y_end = (i + 1) * section_length
            
            cars_in_section = [v for v in vehicles if y_start <= v['y'] < y_end]
            instant_density = len(cars_in_section) / section_length
            instant_speed = np.mean([v['speed'] for v in cars_in_section]) if cars_in_section else section_ema_speed[i]
            
            # takes 20% of instant density, and adds to 80% of density in the last frame, or speed
            section_ema_density[i] = (alpha * instant_density) + ((1 - alpha) * section_ema_density[i])
            section_ema_speed[i] = (alpha * instant_speed) + ((1 - alpha) * section_ema_speed[i])
            
            if section_ema_density[i] > 0.5 and int(current_time * 10) % 5 == 0: # only plot every 5 frames
                history_local_density.append(section_ema_density[i])
                history_local_avg_speed.append(section_ema_speed[i])

    # UPDATE PLOTS
    indicator_positions = [] # list of vehicle positions indicating
    blink_on = int(current_time * 4) % 2 == 0 # makes flashing indicator

    if vehicles:
        vehicle_positions = np.array([[v['current_x'], v['y']] for v in vehicles]) # positions
        speeds = np.array([v['speed'] for v in vehicles])
        colors = cmap(norm(speeds)) # vehicle colours based on speeds
        if blink_on: # indicator flash moves with vehicle
            for v in vehicles:
                if v['signal_dir'] == 1:
                    indicator_positions.append([v['current_x'] + 0.15, v['y']])
                elif v['signal_dir'] == -1:
                    indicator_positions.append([v['current_x'] - 0.15, v['y']])
    else:
        vehicle_positions = np.empty((0, 2)) # empty list of road empty
        colors = []
        
    scatter.set_offsets(vehicle_positions) # draw vehicles at their coordinates
    if len(vehicles) > 0:
        scatter.set_facecolors(colors)
        scatter.set_edgecolors('black') 
        
    if indicator_positions: # draw indicator
        indicator_scatter.set_offsets(indicator_positions)
    else:
        indicator_scatter.set_offsets(np.empty((0, 2)))

    line_speed.set_data(history_time, history_global_avg_speed) # create graph from time, avg speed
    line_flow.set_data(history_time, history_flow)
    
    if history_time:
        ax_speed.set_xlim(0, max(60, history_time[-1] + 5)) # FIX: continuous expanding axis
        
    if history_local_density:
        scatter_plot_points = np.c_[history_local_density, history_local_avg_speed] # joins density and speed lists
        scatter_density.set_offsets(scatter_plot_points) # plots points

    return scatter, indicator_scatter, line_speed, scatter_density, line_flow # returns after each frame

# FIGURES AND ANIMATION
fig = plt.figure(figsize=(14, 8)) # window size
gs = gridspec.GridSpec(2, 3, width_ratios=[1.2, 2, 2]) 

ax_road = fig.add_subplot(gs[:, 0]) # road plot
initialize_motorway_graph(ax_road, road_length)
scatter = ax_road.scatter([], [], s=60, marker='s', zorder=5) # squares for vehicles, start with no coordinates, on top of other layers
indicator_scatter = ax_road.scatter([], [], c='orange', s=30, marker='o', zorder=6) 

# speed time graph
ax_speed = fig.add_subplot(gs[0, 1:]) # position graph
line_speed, = ax_speed.plot([], [], lw=2, color='blue')
ax_speed.set_title('Global average speed')
ax_speed.set_xlabel('Time (s)')
ax_speed.set_ylabel('Average speed (mph)')
ax_speed.set_ylim(0, 130)  
ax_speed.grid(True, linestyle='--', alpha=0.6)

ax_flow = ax_speed.twinx()
line_flow, = ax_flow.plot([], [], lw=3, color='black', alpha=0.3, label='Live flow') # alpha makes it transparent
ax_flow.set_ylabel('Live flow (veh/hr)')
ax_flow.set_ylim(0, 7000)

# speed density graph
ax_density = fig.add_subplot(gs[1, 1:])
scatter_density = ax_density.scatter([], [], s=12, color='red', marker='+', alpha=0.15) # transparency for each dot
ax_density.set_title('Speed–density scatter plot')
ax_density.set_xlabel('Density (veh/mile)')
ax_density.set_ylabel('Speed (mph)')
ax_density.set_xlim(0, 350) 
ax_density.set_ylim(0, 130) 
ax_density.grid(True, linestyle='--', alpha=0.6)

ani = animation.FuncAnimation(fig, update, interval=50, blit=False, cache_frame_data=False) # creates animation
plt.tight_layout()
plt.show()