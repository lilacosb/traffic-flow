import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np

# TRAFFIC LIGHT
class TrafficLightController:
    def __init__(self):
        self.time = 0
        self.phases = [ 
            {'ns': 'green',  'ew': 'red',   'duration': 120}, 
            {'ns': 'orange', 'ew': 'red',   'duration': 30},  
            {'ns': 'red',    'ew': 'red',   'duration': 60},  
            {'ns': 'red',    'ew': 'green', 'duration': 120}, 
            {'ns': 'red',    'ew': 'orange','duration': 30},  
            {'ns': 'red',    'ew': 'red',   'duration': 60},  
        ]
        self.current_phase = 0 
        self.phase_timer = 0

    def update(self):
        self.time += 1
        self.phase_timer += 1
        if self.phase_timer >= self.phases[self.current_phase]['duration']: 
            self.phase_timer = 0 
            self.current_phase = (self.current_phase + 1) % len(self.phases) 

    def get_colors(self):
        return self.phases[self.current_phase]['ns'], self.phases[self.current_phase]['ew'] 

# visual
fig, ax = plt.subplots(figsize=(8, 8))
fig.canvas.manager.set_window_title("Traffic Throughput Simulation")

tl_controller = TrafficLightController()
vehicles = [] 
veh_counter = 0 

# throughput variables
total_departed = 0 
cumulative_journey_time = 0 

# parameters
spawn_rate = 0.25
car_spacing = 0.70 
min_gap = 0.55  
target_speed_val = 0.18 

# ROUTING 
routes = { 
    'W_Straight': {'start_x': -9, 'target_y': 2.25, 'start_y': 1.5, 'dx': 1,  'dy': 0,  'stop_line': -2.3, 'light': 'ew', 'type': 'straight', 'split_start_x': -8.5, 'split_end_x': -6, 'shift_y': 1.5},
    'E_Straight': {'start_x': 9,  'target_y': -2.25, 'start_y': -1.5, 'dx': -1, 'dy': 0,  'stop_line': 2.3,  'light': 'ew', 'type': 'straight', 'split_start_x': 8.5, 'split_end_x': 6, 'shift_y': -1.5},
    'N_Straight': {'start_x': 1, 'target_x': 1.5, 'start_y': 9, 'dx': 0, 'dy': -1, 'stop_line': 3.3, 'light': 'ns', 'type': 'straight', 'shift_x': 1.0, 'split_start_y': 8.5, 'split_end_y': 6}, 
    'S_Straight': {'start_x': -1, 'target_x': -1.5, 'start_y': -9, 'dx': 0, 'dy': 1, 'stop_line': -3.3, 'light': 'ns', 'type': 'straight', 'shift_x': -1.0, 'split_start_y': -8.5, 'split_end_y': -6}, 

    'W_Left': {'start_x': -9, 'target_y': 2.25, 'start_y': 1.5, 'dx': 1, 'dy': 0, 'stop_line': -2.3, 'light': 'ew', 'type': 'left', 'split_start_x': -8.5, 'split_end_x': -6,
               'turn_start': -2, 'cx': -2, 'cy': 3, 'rx': 1.0, 'ry': 0.75, 'math_ry': 0.5, 'start_angle': -np.pi/2, 'end_angle': 0, 'end_dx': 0, 'end_dy': 1}, 
    'E_Left': {'start_x': 9, 'target_y': -2.25, 'start_y': -1.5, 'dx': -1, 'dy': 0, 'stop_line': 2.3, 'light': 'ew', 'type': 'left', 'split_start_x': 8.5, 'split_end_x': 6,
               'turn_start': 2, 'cx': 2, 'cy': -3, 'rx': 1.0, 'ry': 0.75, 'math_ry': 0.5, 'start_angle': np.pi/2, 'end_angle': np.pi, 'end_dx': 0, 'end_dy': -1}, 
    'N_Left': {'start_x': 1, 'target_x': 1.5, 'start_y': 9, 'dx': 0, 'dy': -1, 'stop_line': 3.3, 'light': 'ns', 'type': 'left', 'split_start_y': 8.5, 'split_end_y': 6, 
               'turn_start': 3, 'cx': 2, 'cy': 3, 'rx': 0.5, 'ry': 1.5, 'math_ry': 1.5, 'start_angle': np.pi, 'end_angle': 3*np.pi/2, 'end_dx': 1, 'end_dy': 0}, 
    'S_Left': {'start_x': -1, 'target_x': -1.5, 'start_y': -9, 'dx': 0, 'dy': 1, 'stop_line': -3.3, 'light': 'ns', 'type': 'left', 'split_start_y': -8.5, 'split_end_y': -6, 
               'turn_start': -3, 'cx': -2, 'cy': -3, 'rx': 0.5, 'ry': 1.5, 'math_ry': 1.5, 'start_angle': 0, 'end_angle': np.pi/2, 'end_dx': -1, 'end_dy': 0}, 

    'W_Right': {'start_x': -9, 'target_y': 0.75, 'start_y': 1.5, 'dx': 1, 'dy': 0, 'stop_line': -2.3, 'light': 'ew', 'type': 'right',
                'turn_start': -1.5, 'cx': -1.5, 'cy': -5.5, 'rx': 2.5, 'ry': 6.25, 'math_ry': 6.0, 'start_angle': np.pi/2, 'end_angle': 0, 'end_dx': 0, 'end_dy': -1, 'split_start_x': -8.5, 'split_end_x': -6}, 
    'E_Right': {'start_x': 9, 'target_y': -0.75, 'start_y': -1.5, 'dx': -1, 'dy': 0, 'stop_line': 2.3, 'light': 'ew', 'type': 'right',
                'turn_start': 1.5, 'cx': 1.5, 'cy': 5.5, 'rx': 2.5, 'ry': 6.25, 'math_ry': 6.0, 'start_angle': 3*np.pi/2, 'end_angle': np.pi, 'end_dx': 0, 'end_dy': 1, 'split_start_x': 8.5, 'split_end_x': 6}, 
    'N_Right': {'start_x': 1, 'target_x': 0.5, 'start_y': 9, 'dx': 0, 'dy': -1, 'stop_line': 3.3, 'light': 'ns', 'type': 'right', 
                'turn_start': 1.5, 'cx': -5.5, 'cy': 1.5, 'rx': 6.0, 'ry': 3.0, 'math_ry': 3.0, 'start_angle': 0, 'end_angle': -np.pi/2, 'end_dx': -1, 'end_dy': 0, 'split_start_y': 8.5, 'split_end_y': 6}, 
    'S_Right': {'start_x': -1, 'target_x': -0.5, 'start_y': -9, 'dx': 0, 'dy': 1, 'stop_line': -3.3, 'light': 'ns', 'type': 'right', 
                'turn_start': -1.5, 'cx': 5.5, 'cy': -1.5, 'rx': 6.0, 'ry': 3.0, 'math_ry': 3.0, 'start_angle': np.pi, 'end_angle': np.pi/2, 'end_dx': 1, 'end_dy': 0, 'split_start_y': -8.5, 'split_end_y': -6}, 
}

conflict_map = { # approach: {conflicting vehicle routes}
    'W_Right': ['E_Straight', 'E_Left'],
    'E_Right': ['W_Straight', 'W_Left'],
    'N_Right': ['S_Straight', 'S_Left'],
    'S_Right': ['N_Straight', 'N_Left']
}

opposing_right_turners = { # nearside to nearside do not need to yield to each other
    'W_Right': 'E_Right',
    'E_Right': 'W_Right',
    'N_Right': 'S_Right',
    'S_Right': 'N_Right'
}

def draw_intersection(): # draw road
    ax.clear()
    ax.set_aspect('equal')
    ax.axis('off')
    ax.set_xlim(-9, 9)
    ax.set_ylim(-9, 9)

    edge_color, edge_lw = 'black', 2 # road side boundary
    center_color, center_lw, center_dash = 'black', 3, (0, (6, 6)) 
    lane_color, lane_lw, lane_dash = 'gray', 1.5, (0, (5, 5)) 
    arrow_color = 'darkgray'

    # outline
    ax.plot((-9, -2), (3, 3), color=edge_color, lw=edge_lw)
    ax.plot((-2, -2), (3, 9), color=edge_color, lw=edge_lw)
    ax.plot((2, 9), (3, 3), color=edge_color, lw=edge_lw)
    ax.plot((2, 2), (3, 9), color=edge_color, lw=edge_lw)
    ax.plot((-9, -2), (-3, -3), color=edge_color, lw=edge_lw)
    ax.plot((-2, -2), (-3, -9), color=edge_color, lw=edge_lw)
    ax.plot((2, 9), (-3, -3), color=edge_color, lw=edge_lw)
    ax.plot((2, 2), (-3, -9), color=edge_color, lw=edge_lw)
    
    # centre dividers
    ax.plot((-2, -9), (0, 0), color=center_color, lw=center_lw, linestyle=center_dash) 
    ax.plot((2, 9), (0, 0), color=center_color, lw=center_lw, linestyle=center_dash)   
    ax.plot((0, 0), (3, 9), color=center_color, lw=center_lw, linestyle=center_dash)
    ax.plot((0, 0), (-3, -9), color=center_color, lw=center_lw, linestyle=center_dash)
    
    # lane dividers
    ax.plot((-2.3, -6.2), (1.5, 1.5), color=lane_color, lw=lane_lw, linestyle=lane_dash)
    ax.plot((2.3, 6.2), (-1.5, -1.5), color=lane_color, lw=lane_lw, linestyle=lane_dash) 
    ax.plot((1, 1), (3.3, 6.2), color=lane_color, lw=lane_lw, linestyle=lane_dash) 
    ax.plot((-1, -1), (-3.3, -6.2), color=lane_color, lw=lane_lw, linestyle=lane_dash) 
    
    # arrow at pos x,y, directions dx,dy
    def draw_arrow(x, y, dx, dy): ax.arrow(x, y, dx, dy, head_width=0.15, head_length=0.25, fc=arrow_color, ec=arrow_color, lw=1.5, length_includes_head=True)
    def draw_turn(x, y, dx1, dy1, dx2, dy2): ax.plot([x, x+dx1], [y, y+dy1], color=arrow_color, lw=1.5); draw_arrow(x+dx1, y+dy1, dx2, dy2)

    draw_turn(-4.5, 0.75, 0.8, 0, 0, -0.4); draw_turn(-4.5, 2.25, 0.8, 0, 0, 0.4); draw_arrow(-4.5, 2.25, 1.2, 0); draw_arrow(-8.5, 1.5, 1.2, 0) 
    draw_turn(4.5, -0.75, -0.8, 0, 0, 0.4); draw_turn(4.5, -2.25, -0.8, 0, 0, -0.4); draw_arrow(4.5, -2.25, -1.2, 0); draw_arrow(8.5, -1.5, -1.2, 0) 
    draw_turn(0.5, 5.0, 0, -0.8, -0.4, 0); draw_turn(1.5, 5.0, 0, -0.8, 0.4, 0); draw_arrow(1.5, 5.0, 0, -1.2); draw_arrow(1, 8.5, 0, -1.2) 
    draw_turn(-0.5, -5.0, 0, 0.8, 0.4, 0); draw_turn(-1.5, -5.0, 0, 0.8, -0.4, 0); draw_arrow(-1.5, -5.0, 0, 1.2); draw_arrow(-1, -8.5, 0, 1.2) 
    
    # ROAD LABELS    
    text_props = dict(fontsize=11, fontweight='bold', color='darkslategray', va='center', ha='center', bbox=dict(facecolor='white', edgecolor='none', alpha=0.7, pad=1))
    ax.text(-2.4, 6.5, "Church Street", rotation=90, **text_props)
    ax.text(6.5, 3.4, "Stockton Road", **text_props) 
    ax.text(2.5, -6.5, "South Road", rotation=90, **text_props) 
    ax.text(-6.5, -3.4, "Quarryheads Lane", **text_props) 

ns_light_pos = [(2, 3.3), (-2, -3.3)] # place traffic lights
ew_light_pos = [(2.3, -3), (-2.3, 3)] 

def update(frame):
    global veh_counter, total_departed, cumulative_journey_time
    
    # redraw roads, update lights
    draw_intersection() 
    tl_controller.update()
    ns_color, ew_color = tl_controller.get_colors()

    ax.scatter([x for x, y in ns_light_pos], [y for x, y in ns_light_pos], s=120, color=ns_color, edgecolors='black', zorder=5)
    ax.scatter([x for x, y in ew_light_pos], [y for x, y in ew_light_pos], s=120, color=ew_color, edgecolors='black', zorder=5)
    
    if np.random.rand() < spawn_rate: #spawn vehicle in random approach
        approach = np.random.choice(['W', 'E', 'N', 'S'])
        
        # turn direction based on given prob
        if approach == 'W':
            route_key = np.random.choice(['W_Right', 'W_Straight', 'W_Left'], p=[0.1, 0.65, 0.25])
        elif approach == 'E':
            route_key = np.random.choice(['E_Right', 'E_Straight', 'E_Left'], p=[0.11, 0.53, 0.36])
        elif approach == 'N':
            route_key = np.random.choice(['N_Right', 'N_Straight', 'N_Left'], p=[0.25, 0.65, 0.1])
        elif approach == 'S':
            route_key = np.random.choice(['S_Right', 'S_Straight', 'S_Left'], p=[0.3, 0.6, 0.1])

        route = routes[route_key]
        if all(np.hypot(v['x'] - route['start_x'], v['y'] - route['start_y']) > 0.8 for v in vehicles): # cannot spawn too close to vehicle ahead
            vehicles.append({ # add new vehicle with following properties
                'id': veh_counter, 'route_key': route_key, 
                'x': route['start_x'], 'y': route['start_y'], 
                'hx': route['dx'], 'hy': route['dy'], 
                'speed': target_speed_val, 'max_speed': np.random.uniform(target_speed_val-0.02, target_speed_val+0.02),
                'state': 'approaching', 'angle': 0,
                'recorded': False, 'spawn_time': tl_controller.time # timestamp on creation
            })
            veh_counter += 1

    for v in vehicles:
        route = routes[v['route_key']]
        current_light = ew_color if route['light'] == 'ew' else ns_color  # relevant tl
        target_v = v['max_speed']
        
        old_x, old_y = v['x'], v['y'] # store pos
        
        # update departures info
        if v['state'] == 'departing' and not v['recorded']:
            total_departed += 1
            v['recorded'] = True
        
        if v['state'] == 'approaching':
            if route['dx'] != 0:
                dist_to_stop = (route['stop_line'] - v['x']) * route['dx'] # if approaching from e or w
            else:
                dist_to_stop = (route['stop_line'] - v['y']) * route['dy'] # for n, s
            
            if 0.0 < dist_to_stop < 5: # if close, look at tl
                is_late_orange = (current_light == 'orange' and tl_controller.phase_timer > 25) # how far through orange phase
                
                # brake if red or orange and safe
                if current_light == 'red' or (current_light == 'orange' and (dist_to_stop > 1.5 or (is_late_orange and dist_to_stop > 0.5) or v['speed'] < v['max_speed'] * 0.85)):
                    target_v = max(0, v['max_speed'] * (dist_to_stop / 4.0))
                    if dist_to_stop < 0.03:
                        target_v = 0
              
            # right turners 
            elif route['type'] == 'right':
                if route['dx'] != 0: 
                    dist_to_turn = (route['turn_start'] - v['x']) * route['dx']
                else: 
                    dist_to_turn = (route['turn_start'] - v['y']) * route['dy']
                
                if 0.0 <= dist_to_turn < 3:
                    must_yield = False
                    for other in vehicles: # check if it is safe to turn based on conflicting routes
                        if other['route_key'] in conflict_map[v['route_key']]:
                            is_blocked = False
                            for blocker in vehicles:
                                b_route = routes[blocker['route_key']] # car maybe infront of other 
                                o_route = routes[other['route_key']] # car to yield to
                                same_origin = (b_route['start_x'] == o_route['start_x'] and b_route['start_y'] == o_route['start_y'])
                                if same_origin and blocker['id'] < other['id']: # blocker is ahead of other
                                    if o_route['dx'] != 0: # in same lane?
                                        l_diff = abs(blocker['y'] - other['y'])
                                    else:
                                        l_diff = abs(blocker['x'] - other['x'])
                                    if l_diff < 0.6: # then in same lane
                                        if b_route['type'] == 'right' and blocker['state'] in ['approaching', 'turning']: 
                                            is_blocked = True
                                            break # break loop, no need to check more cars
                            if is_blocked:
                                continue # can execute turn

                            if other['state'] in ['approaching', 'crossing', 'turning']: # check what it is doing - moving? 
                                o_route = routes[other['route_key']]
                                primary_pos = other['x'] * o_route['dx'] + other['y'] * o_route['dy'] # dist from centre of junction
                                
                                if -6 < primary_pos < -2.5: # can make turn if oncoming car has passed collision point
                                    if current_light == 'green' or (current_light == 'orange' and primary_pos > -4): # yield conditions
                                        must_yield = True
                                        break

                    if must_yield: # then brake, waiting inside box
                        target_v = max(0, v['max_speed'] * ((dist_to_turn - 0.3) / 2.0))
                        if dist_to_turn < 0.4: 
                            target_v = 0
                            v['speed'] = 0 
            
        for other in vehicles:
            if other['id'] == v['id']: # other vehicle is itself
                continue
            if v['route_key'] in opposing_right_turners and other['route_key'] == opposing_right_turners[v['route_key']]:
                continue # nearside to nearside turning
            
            dx = other['x'] - v['x']
            dy = other['y'] - v['y']
            dist = np.hypot(dx, dy) # calc straight line dist to other veh
            vision_length = 1.2 if v['state'] == 'turning' else 3.0 # adjust speed to traffic
            
            if dist < vision_length: # if other car close enough
                ahead_dist = dx * v['hx'] + dy * v['hy'] # dist between the two cars, projected to the direction the car is pointing
                lat_dist = abs(dx * v['hy'] - dy * v['hx']) # sideways dist
                
                radar_width = 0.85 if v['state'] == 'crossing' else 0.45 
                
                if ahead_dist > 0.1 and lat_dist < radar_width:
                    heading_dir = v['hx'] * other['hx'] + v['hy'] * other['hy'] 
                    if heading_dir > 0.5:  
                        if ahead_dist < car_spacing:
                            target_v = min(target_v, other['speed'] * 0.5)
                            if ahead_dist < min_gap:
                                target_v = 0; v['speed'] = 0 
        
        # accel/decel to target speed
        if v['speed'] < target_v: 
            v['speed'] = min(target_v, v['speed'] + 0.012)
        elif v['speed'] > target_v: 
            v['speed'] = max(target_v, v['speed'] - 0.04)
            
        if v['state'] == 'approaching':
            v['x'] += v['speed'] * route['dx'] # going straight
            v['y'] += v['speed'] * route['dy']
            
            # routes with turning
            if 'split_start_y' in route: # lane splits vertically
                if route['dy'] < 0: # driving south
                    if v['y'] <= route['split_start_y']: # crossed starting line?
                        prog = (route['split_start_y'] - v['y']) / (route['split_start_y'] - route['split_end_y']) # percentage of lane vehicle has completed
                        v['x'] = route['start_x'] + (route.get('target_x', route['start_x']) - route['start_x']) * (0.5 - 0.5 * np.cos(np.pi * np.clip(prog, 0, 1))) # ease into desired lane
                else: # driving north
                    if v['y'] >= route['split_start_y']:
                        prog = (v['y'] - route['split_start_y']) / (route['split_end_y'] - route['split_start_y'])
                        v['x'] = route['start_x'] + (route.get('target_x', route['start_x']) - route['start_x']) * (0.5 - 0.5 * np.cos(np.pi * np.clip(prog, 0, 1)))
            
            if 'split_start_x' in route: # lane splits horizontally
                if route['dx'] > 0:
                    if v['x'] >= route['split_start_x']:
                        prog = (v['x'] - route['split_start_x']) / (route['split_end_x'] - route['split_start_x'])
                        v['y'] = route['start_y'] + (route.get('target_y', route['start_y']) - route['start_y']) * (0.5 - 0.5 * np.cos(np.pi * np.clip(prog, 0, 1)))
                else: 
                    if v['x'] <= route['split_start_x']:
                        prog = (route['split_start_x'] - v['x']) / (route['split_start_x'] - route['split_end_x'])
                        v['y'] = route['start_y'] + (route.get('target_y', route['start_y']) - route['start_y']) * (0.5 - 0.5 * np.cos(np.pi * np.clip(prog, 0, 1)))
            
            if route['type'] == 'straight' and ('shift_x' in route or 'shift_y' in route): # going straight but middles of lane do not align
                if (route['dy'] < 0 and v['y'] <= 3.0) or (route['dy'] > 0 and v['y'] >= -3.0) or (route['dx'] > 0 and v['x'] >= -2.0) or (route['dx'] < 0 and v['x'] <= 2.0): 
                    v['state'] = 'crossing'
                    
            elif route['type'] in ['left', 'right']: # turning vehs
                reached = False 
                if route['dx'] > 0 and v['x'] >= route['turn_start']:
                    reached = True # has reached turning point
                if route['dx'] < 0 and v['x'] <= route['turn_start']:
                    reached = True
                if route['dy'] > 0 and v['y'] >= route['turn_start']:
                    reached = True
                if route['dy'] < 0 and v['y'] <= route['turn_start']:
                    reached = True
                
                if reached:
                    v['state'] = 'turning'
                    v['angle'] = route['start_angle']
                    if route['dx'] != 0:
                        v['x'] = route['turn_start'] # go to exact turn start pos if it has gone past
                    if route['dy'] != 0: 
                        v['y'] = route['turn_start']

        elif v['state'] == 'crossing':
            if route['dx'] != 0: 
                progress = (v['x'] - (-2.0)) / 4.0 if route['dx'] > 0 else (2.0 - v['x']) / 4.0 
                progress = np.clip(progress, 0, 1) 
                
                start_pos_y = route.get('target_y', route['start_y'])
                shift_y = route.get('shift_y', start_pos_y) 
                delta_y = shift_y - start_pos_y 
                
                slope = (delta_y * 0.5 * np.pi * np.sin(np.pi * progress)) / 4.0 
                step_x = v['speed'] / np.sqrt(1 + slope**2) 
                v['x'] += step_x * route['dx'] 
                
                progress = (v['x'] - (-2.0)) / 4.0 if route['dx'] > 0 else (2.0 - v['x']) / 4.0 
                progress = np.clip(progress, 0, 1) 
                
                v['y'] = start_pos_y + delta_y * (0.5 - 0.5 * np.cos(np.pi * progress)) 
                
                if progress >= 1:
                    v['state'] = 'departing' 
                    v['y'] = shift_y 
                    
            else: 
                progress = (3.0 - v['y']) / 6.0 if route['dy'] < 0 else (v['y'] - (-3.0)) / 6.0 
                progress = np.clip(progress, 0, 1) 
                
                start_pos_x = route.get('target_x', route['start_x'])  
                shift_x = route.get('shift_x', start_pos_x) 
                delta_x = shift_x - start_pos_x 
                
                slope = (delta_x * 0.5 * np.pi * np.sin(np.pi * progress)) / 6.0 
                step_y = v['speed'] / np.sqrt(1 + slope**2) 
                v['y'] += step_y * route['dy'] 
                
                progress = (3.0 - v['y']) / 6.0 if route['dy'] < 0 else (v['y'] - (-3.0)) / 6.0 
                progress = np.clip(progress, 0, 1) 
                
                v['x'] = start_pos_x + delta_x * (0.5 - 0.5 * np.cos(np.pi * progress)) 
                
                if progress >= 1:
                    v['state'] = 'departing' 
                    v['x'] = shift_x 

        elif v['state'] == 'turning':
            math_rx = route.get('math_rx', route['rx']) 
            math_ry = route.get('math_ry', route['ry']) 
            omega = v['speed'] / np.sqrt((math_rx * np.sin(v['angle']))**2 + (math_ry * np.cos(v['angle']))**2) 
            
            if route['type'] == 'left':
                v['angle'] += omega # anticlockwise move
                finish_turn = v['angle'] >= route['end_angle']
            else: 
                v['angle'] -= omega # turn right, move clockwise
                finish_turn = v['angle'] <= route['end_angle']
            
            v['x'] = route['cx'] + route['rx'] * np.cos(v['angle']) # back to x/y
            v['y'] = route['cy'] + route['ry'] * np.sin(v['angle'])
            
            if finish_turn: # when turn complete change status to departing
                v['state'] = 'departing'
                v['angle'] = route['end_angle'] # if have passed end angle, snap back to it
                v['x'] = route['cx'] + route['rx'] * np.cos(v['angle']) 
                v['y'] = route['cy'] + route['ry'] * np.sin(v['angle'])
                if route.get('end_dx', route['dx']) != 0: 
                    v['y'] = route['cy'] + route['ry'] * np.sin(v['angle']) # ensures central pos in lane
                if route.get('end_dy', route['dy']) != 0: 
                    v['x'] = route['cx'] + route['rx'] * np.cos(v['angle'])

        elif v['state'] == 'departing': # completed intersection, so drive straight
            v['x'] += v['speed'] * route.get('end_dx', route['dx'])
            v['y'] += v['speed'] * route.get('end_dy', route['dy'])
            
        if v['state'] == 'turning':
            if np.hypot(v['x'] - old_x, v['y'] - old_y) > 0.0001: # ensures is moving
                dist = np.hypot(v['x'] - old_x, v['y'] - old_y) # gives collision radar radius
                v['hx'] = (v['x'] - old_x) / dist
                v['hy'] = (v['y'] - old_y) / dist
        elif v['state'] == 'departing':
            v['hx'] = route.get('end_dx', route['dx'])
            v['hy'] = route.get('end_dy', route['dy'])
        else: 
            v['hx'] = route['dx']
            v['hy'] = route['dy']

# JOURNEY TRACKING
    active_vehicles = []
    for v in vehicles:
        if -10 < v['x'] < 10 and -10 < v['y'] < 10: # vehicles on screen
            active_vehicles.append(v)
        else:
            # journey completed, record time taken
            journey_time = tl_controller.time - v['spawn_time']
            global cumulative_journey_time 
            cumulative_journey_time += journey_time
    vehicles[:] = active_vehicles # rewrite list, deleting departed vehicles
    
    if vehicles: # draw vehicles
        ax.scatter([v['x'] for v in vehicles], [v['y'] for v in vehicles], s=100, marker='s', color='blue', edgecolors='black', zorder=6)
        
    blinker_xs, blinker_ys = [], [] # signalling indicator
    if (tl_controller.time % 8) < 4: # flashes, on 4 frames, off 4 frames
        for v in vehicles:
            route = routes[v['route_key']]
            if route['type'] in ['left', 'right'] and v['state'] != 'departing': # do not signal if going straight
                if route['dx'] == 0: 
                    if abs(v['y']) > 8.8: # within visual boundary
                        continue 
                else:                
                    if abs(v['x']) > 8.8: 
                        continue
                
                px, py = -v['hy'], v['hx'] # move indicator with car (instead of rotating car)
                if route['type'] == 'right':
                    px, py = v['hy'], -v['hx'] 
                blinker_xs.append(v['x'] + v['hx'] * 0.3 + px * 0.3) # add relative to centre of vehicle
                blinker_ys.append(v['y'] + v['hy'] * 0.3 + py * 0.3)
                
    if blinker_xs:
        ax.scatter(blinker_xs, blinker_ys, s=30, color='orange', zorder=7)

    # INFO BOX
    departure_rate = int((total_departed / tl_controller.time) * 1000) if tl_controller.time > 0 else 0
    
    # average: total time of all cars / total number of cars
    avg_journey = int(cumulative_journey_time / total_departed) if total_departed > 0 else 0
    
    info_text = (
        f"Throughput\n"
        f"Rate: {departure_rate} vehicles / 1000 frames\n"
        f"Average journey time: {avg_journey} frames"
    )
    
    ax.text(-8.5, 8.5, info_text, color='black', fontsize=8, family='monospace',
            bbox=dict(facecolor='white', alpha=0.85, edgecolor='black', boxstyle='round,pad=0.3'),
            verticalalignment='top', zorder=10)

ani = animation.FuncAnimation(fig, update, interval=30, blit=False)
plt.tight_layout()
plt.show()