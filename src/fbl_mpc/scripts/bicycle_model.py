

import numpy as np
from scipy.spatial import cKDTree
import matplotlib.pyplot as plt

class Bicycle_model:
    def __init__(self, node, horizon, dt):
        self.node = node
        self.horizon = horizon
        self.mpc_time_step = dt

        self.create_params()
        self.get_params()

        # Initial states
        self.current_v_long = 0.0
        self.current_v_lat = 0.0
        self.current_w = 0.0
        self.current_steering_angle = 0.0

        # Future inputs
        self.future_steering_angle = np.zeros(horizon)
        self.future_acc_long = np.zeros(horizon)

    def create_params(self):
        self.node.declare_parameter('bicycle.sample_t', value=0.01) #s
        
        
        # self.node.declare_parameter('bicycle.Iz', value=800.0) # moment of inertia kg*m^2
        # self.node.declare_parameter('bicycle.l_F', value=1.7) #m
        # self.node.declare_parameter('bicycle.l_R', value=1.2)
        # self.node.declare_parameter('bicycle.m', value=787) #kg
        # self.node.declare_parameter('bicycle.D_f', value=1.5)  # pacejka coeff from parameter
        # self.node.declare_parameter('bicycle.D_r', value=1.5) # pacejka coeff from parameter
        # self.node.declare_parameter('bicycle.B_f', value=5.0)
        # self.node.declare_parameter('bicycle.B_r', value=5.0)
        # self.node.declare_parameter('bicycle.C_f', value=2.35)# from simulation parameters
        # self.node.declare_parameter('bicycle.C_r', value=2.35)
        # self.node.declare_parameter('bicycle.C_lf', value=-0.65) # Coefficient of lift, front
        # self.node.declare_parameter('bicycle.C_lr', value=-1.18) # Coefficient of lift, rear
        # self.node.declare_parameter('bicycle.Cor_F', value=55000.0) # N/rad
        # self.node.declare_parameter('bicycle.Cor_R', value=75000.0) # N/rad


        
        #  #awsim
        self.node.declare_parameter('bicycle.Iz', value=800.0) # moment of inertia kg*m^2
        self.node.declare_parameter('bicycle.l_F', value=1.6785) #m
        self.node.declare_parameter('bicycle.l_R', value=1.2933)
        self.node.declare_parameter('bicycle.m', value=815.11) #kg
        self.node.declare_parameter('bicycle.D_f', value=1.5)  # pacejka coeff from parameter
        self.node.declare_parameter('bicycle.D_r', value=1.55) # pacejka coeff from parameter
        self.node.declare_parameter('bicycle.B_f', value=19.85)
        self.node.declare_parameter('bicycle.B_r', value=19.059)
        self.node.declare_parameter('bicycle.C_f', value=1.5)# from simulation parameters
        self.node.declare_parameter('bicycle.C_r', value=1.6) # from simulation parameters        
        
        self.node.declare_parameter('bicycle.C_lf', value=-0.65) # Coefficient of lift, front
        self.node.declare_parameter('bicycle.C_lr', value=-1.18) # Coefficient of lift, rear
        self.node.declare_parameter('bicycle.Cor_F', value=110000.0) # N/rad
        self.node.declare_parameter('bicycle.Cor_R', value=120000.0) # N/rad

    def get_params(self):
        self.sample_t = self.node.get_parameter('bicycle.sample_t').value
        self.vehicle_mass = self.node.get_parameter('bicycle.m').value
        self.l_F = self.node.get_parameter('bicycle.l_F').value
        self.l_R = self.node.get_parameter('bicycle.l_R').value
        self.D_f = self.node.get_parameter('bicycle.D_f').value
        self.D_r = self.node.get_parameter('bicycle.D_r').value
        self.C_f = self.node.get_parameter('bicycle.C_f').value
        self.C_r = self.node.get_parameter('bicycle.C_r').value
        self.B_f = self.node.get_parameter('bicycle.B_f').value
        self.B_r = self.node.get_parameter('bicycle.B_r').value
        self.Iz = self.node.get_parameter('bicycle.Iz').value
        self.C_lf = self.node.get_parameter('bicycle.C_lf').value
        self.C_lr = self.node.get_parameter('bicycle.C_lr').value
        self.Cor_F = self.node.get_parameter('bicycle.Cor_F').value
        self.Cor_R = self.node.get_parameter('bicycle.Cor_R').value

    def normalize_angle(self, yaw):
        while yaw > np.pi / 2:
            yaw -= np.pi
        while yaw < -np.pi / 2:
            yaw += np.pi
        return yaw
    
    def update_current_state(self, current_v_long, current_v_lat, current_w, current_steering_angle, future_steering_angle, future_acc_long):
        self.current_v_long = current_v_long
        self.current_v_lat = current_v_lat
        self.current_w = current_w
        self.current_steering_angle = np.clip(np.deg2rad(current_steering_angle),-0.28,0.28)
        self.future_steering_angle = np.clip(np.deg2rad(future_steering_angle),-0.28,0.28)
        self.future_acc_long = future_acc_long
       
    def cal_vertical_force(self, acc_long, v_lat,v_long, w):
        # Calculate the vertical forces
        # print(v_lat, w, acc_long)
        #TODO: Fix aero
        #TODO: weight transfer?
        if v_long > 30:
            L_f = 0.5*1.225*v_long**2*self.C_lf*0
            L_r = 0.5*1.225*v_long**2*self.C_lr*0
        else:
            L_f = 0
            L_r = 0
       
        F_zf = self.vehicle_mass * 9.8 * self.l_R / (self.l_F + self.l_R)-L_f
        F_zr = self.vehicle_mass * 9.8 * self.l_F / (self.l_F + self.l_R)-L_r
        # F_zf = self.vehicle_mass * 9.8 * self.l_R / (self.l_F + self.l_R) + self.vehicle_mass * acc_long * self.l_F / (self.l_F + self.l_R) - self.vehicle_mass * v_lat * w
        # F_zr = self.vehicle_mass * 9.8 * self.l_F / (self.l_F + self.l_R) - self.vehicle_mass * acc_long * self.l_R / (self.l_F + self.l_R) + self.vehicle_mass * v_lat * w
        return F_zf, F_zr
     
    def cal_slip_angle(self, v_long, v_lat, w, steering_angle):
        # Calculate the slip angle
        # TODO: v_long is velocity of the vehicle in the longitudinal direction (not vector?)
        alpha_f = np.arctan((v_lat + self.l_F * w) / v_long) - steering_angle
        alpha_r = np.arctan((v_lat - self.l_R * w) / v_long)
        return alpha_f, alpha_r
    
    def cal_lat_force(self, v_long, v_lat,acc_long, w, steering_angle):
        F_zf, F_zr = self.cal_vertical_force(acc_long, v_lat, v_long, w)
        alpha_f, alpha_r = self.cal_slip_angle(v_long, v_lat, w, steering_angle)
        Cor_f = self.Cor_F
        Cor_r = self.Cor_R
        F_yf = F_zf * self.D_f * np.sin(self.C_f * np.arctan(self.B_f * (-alpha_f)))
        F_yr = F_zr * self.D_r * np.sin(self.C_r * np.arctan(self.B_r * (-alpha_r)))
        current_v_long = self.current_v_long
        if current_v_long > 60:
            Cor_f = 100000
        if current_v_long > 75:
            Cor_f = 95000
        if current_v_long > 80:
            Cor_f = 90000
        if current_v_long > 85:
            Cor_f = 80000
           

        # F_yf = -Cor_f * alpha_f
        # F_yr = -Cor_r * alpha_r
        
        return F_yf, F_yr, alpha_f, alpha_r #front and rear tire lateral forces
    

    def calculate_next_position(self, v_long, v_lat,acc_long, w, steering_angle, position):
        v_long = np.max([v_long, 5.0])
        dt = self.sample_t
        F_yf, F_yr, alpha_f, alpha_r = self.cal_lat_force(v_long, v_lat, acc_long, w, steering_angle)
        if acc_long < 0:
            F_xf = self.vehicle_mass*acc_long*0.5
            # F_xf = 0
        else:
            F_xf = 0
            
        x,y, yaw = position

        dv_long = acc_long
        dv_lat = (1/self.vehicle_mass) * (F_yr + F_yf * np.cos(steering_angle) + F_xf * np.sin(steering_angle) - self.vehicle_mass * v_long * w)
        dyaw = w  #yaw rate
        dw = (1/self.Iz) * (self.l_F * F_yf * np.cos(steering_angle) - self.l_R * F_yr)
        # R = v_long / w
        # dx = R*(np.sin(yaw + dt*w) - np.sin(yaw))
        # dy = R*(np.cos(yaw) - np.cos(yaw + dt*w))
        dx = v_long * np.cos(yaw) - v_lat * np.sin(yaw)
        dy = v_long * np.sin(yaw) + v_lat * np.cos(yaw)
        new_position = position + np.array([dx,dy,dyaw]) * dt
        new_position[2] = self.normalize_angle(new_position[2])
        new_w = w + dw * dt
        new_v_long = v_long + dv_long * dt
        new_v_lat = v_lat + dv_lat * dt #+ 0.5*v_long * w * dt*dt
        # print("with steering_angle",steering_angle)
        # print("new_w,nw_v_long,new_v_lat is",new_w,new_v_long,new_v_lat)
        
        return new_position, new_w, new_v_long, new_v_lat, new_w,  alpha_f, alpha_r
    
    def interpolate_controls(self, times):
        steering_interpolated = np.interp(times, np.arange(self.horizon) * self.mpc_time_step, self.future_steering_angle)
        acceleration_interpolated = np.interp(times, np.arange(self.horizon) * self.mpc_time_step, self.future_acc_long)
        return steering_interpolated, acceleration_interpolated
    
    def calculate_future_states(self):
        total_time = self.horizon * self.mpc_time_step
        num_samples = int(total_time / self.sample_t) + 1
        times = np.linspace(0, total_time, num=num_samples)

        steering_angles, accelerations = self.interpolate_controls(times)

        positions = []
        v_longs = []
        v_lats = []
        yaw_rates = []
        alpha_fs=[]
        alpha_rs=[]
        position = [0.0, 0.0, 0.0]
        positions.append(position)
        v_long = self.current_v_long
        v_longs.append(v_long)
        v_lat = self.current_v_lat
        v_lats.append(v_lat)
        w = self.current_w
        yaw_rates.append(w)

        for i in range(1, len(times)):
            next_position, w, v_long, v_lat, w, alpha_f, alpha_r = self.calculate_next_position(
                v_long, v_lat, accelerations[i], w, steering_angles[i], position
            )
            alpha_fs.append(alpha_f)
            alpha_rs.append(alpha_r)
            positions.append(next_position)
            v_longs.append(v_long)
            v_lats.append(v_lat)
            yaw_rates.append(w)
            position = next_position

        # Re-interpolating the results to the MPC time step
        mpc_times = np.arange(0, self.horizon + 1) * self.mpc_time_step
        positions_x = np.interp(mpc_times, times, [p[0] for p in positions])
        positions_y = np.interp(mpc_times, times, [p[1] for p in positions])
        positions_theta = np.interp(mpc_times, times, [p[2] for p in positions])
        v_longs = np.interp(mpc_times, times, v_longs)
        v_lats = np.interp(mpc_times, times, v_lats)
        yaw_rates = np.interp(mpc_times, times, yaw_rates)
        # print("yaw_rates",yaw_rates)

        # print(drift_angles)

        # Recomposing the interpolated positions
        interpolated_positions = np.array(list(zip(positions_x, positions_y, positions_theta)))
        #print(interpolated_positions)

        return interpolated_positions, v_longs, v_lats, yaw_rates, alpha_fs,alpha_rs
     