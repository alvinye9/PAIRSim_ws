#!/usr/bin/env python3

from typing import Tuple
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
import csv
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from deep_orange_msgs.msg import JoystickCommand
from geometry_msgs.msg import PoseStamped, PointStamped, Point
from std_msgs.msg import Float32, Float32MultiArray
import osqp
from scipy import sparse
from novatel_oem7_msgs.msg import RAWIMU
from sensor_msgs.msg import Imu
from bicycle_model import Bicycle_model
from scipy.interpolate import interp1d
from scipy.linalg import lstsq
import numpy as np
from scipy.spatial import cKDTree

class MPC(Node):

    def __init__(self):
        super().__init__('MPC_node')
        self.create_subscriptions()

        self.create_publishers()

        self.update_counter = 0 # for test purposes

        #vehicle parameters
        self.create_params()
        self.init_params()
        
        #create timers
        timer_period = 0.01
        self.control_timer = self.create_timer(timer_period, self.update)

        param_timer_period = 1.0
        self.param_timer = self.create_timer(param_timer_period, self.update_params)

        #initialize the state matrix for optimization problem
        
        self.construct_matrix()

        # Current States
        self.current_v = 0.0
        self.current_v_lat = 0.0
        self.current_yaw_rate = 0.0
        self.current_steer_angle = 0.0

        self.desired_vs = []
        self.curvature = []

        self.clear_result()

        self.current_path = []
        self.kdtree = None
        self.desired_Acc = 0.0
        
        #initilieze the bicycle model
        self.bicycle_model = Bicycle_model(self, self.horizon, self.ts)

        
        # print("G1", G1)
        

    def create_publishers(self):
        #publishers
        self.publisher_lateral_error = self.create_publisher(Float32, '/lateral_error', 10)
        self.path_pub = self.create_publisher(Path, '/mpc_debug_path', 10)
        self.desired_w_pub = self.create_publisher(Float32, '/desired_yaw_rate', 10)
        self.publisher_lateral_error_avg = self.create_publisher(Float32, '/lateral_error_avg', 10)
        self.publisher_heading_error = self.create_publisher(Float32, '/heading_error', 10)
        self.publisher_future_steer_angle = self.create_publisher(Float32MultiArray, '/future_steer_angle', 10)
        self.publisher_desired_w_array = self.create_publisher(Float32MultiArray, '/desired_w_array', 10)
        self.publisher_JoystickCommand = self.create_publisher(JoystickCommand, '/control/controller/accel_command', 10)
        self.publisher_current_yaw_rate = self.create_publisher(Float32, '/current_yaw_rate', 10)
        self.publisher_current_frontslip_angle = self.create_publisher(Float32, '/frontslip_angle', 10)
        self.publisher_current_rearslip_angle = self.create_publisher(Float32, '/rearslip_angle', 10)
        
    def create_subscriptions(self):
        # topic subscriptions
        self.path_subscription = self.create_subscription(
            Path,
            '/planning/front_path/offset_path',
            self.path_cb,
            10
        )
        self.vel_array_subscription = self.create_subscription(
            Float32MultiArray,
            '/velocities',
            self.vel_array_cb,
            10
        )
        self.curvature_subscription = self.create_subscription(
            Float32MultiArray,
            'planning/front_path/curvature',
            self.curvature_cb,
            10
        )

        self.vel_subscription = self.create_subscription(
            Float32,
            '/localization/vehicle_speed',
            self.speed_cb,
            10
        )
        self.awsim_lat_v_sub = self.create_subscription(
            Odometry,
            'COM/odom',
            self.lat_v_cb,
            10
        )
        # for cogen sim
        self.imu_subscription = self.create_subscription(
            Imu,
            '/novatel_bottom/imu/data',
            self.rawimu_cb,
            10
        )
        self.lat_v_sub = self.create_subscription(
            Odometry,
            '/odometry/local_filtered',
            self.lat_v_cb,
            10
        )
        # for autonoma sim
        self.RAWIMU_subscription = self.create_subscription(
            RAWIMU,
            '/novatel_top/rawimu',
            self.rawimu_cb,
            10
        )

    def create_params(self):
        self.declare_parameter('prediction_ts', value=0.1)
        self.declare_parameter('velocity_error_weight', value=5.0)
        self.declare_parameter('lateral_error_weight', value=45.0)
        self.declare_parameter('lateral_error_velocity_weight', value=5.0)
        self.declare_parameter('R_weight', value=1.0) #steering rate weight
        self.declare_parameter('acceleration_weight', value=3)
        self.declare_parameter('horizon_length', value=12)
        self.declare_parameter('rho_slack', value=0.0)

    def init_params(self):
        self.horizon = self.get_parameter('horizon_length').value
        self.ts = self.get_parameter('prediction_ts').value
        self.velocity_error_weight = self.get_parameter('velocity_error_weight').value
        self.lateral_error_weight = self.get_parameter('lateral_error_weight').value
        self.lateral_error_velocity_weight = self.get_parameter('lateral_error_velocity_weight').value
        self.accelration_weight = self.get_parameter('acceleration_weight').value
        self.R_weight = self.get_parameter('R_weight').value
        self.rho_slack = self.get_parameter('rho_slack').value

    def construct_matrix(self):
        self.A = np.array([[1, 0, 0      ],
                           [0, 1, self.ts],
                           [0, 0, 1      ]])
        self.B = np.array([[self.ts, 0           ],
                           [0,       self.ts**2/2], 
                           [0,       self.ts     ]])
        self.slack_variable_number = 2
        N = self.horizon
        nu = self.B.shape[1]
        nx = self.A.shape[0]
        ns = self.slack_variable_number
        G1 = np.zeros((1, nx))
        G1[0, 1] = 1  # Constraint on the second state variable for all time steps
        G2 = np.zeros((1, nx))
        G2[0, 1] = -1  # Constraint on the second state variable for all time steps
        # Construct inequality matrices for slack variables
        first_diag = sparse.block_diag([sparse.csc_matrix((1,1)), sparse.eye(N)])
        middle_u_matrix = sparse.csc_matrix(((N+1), N*nu))
        slack_matrix = sparse.block_diag([sparse.csc_matrix((1,1)), sparse.eye(N)])
        empty_matrix = sparse.csc_matrix((N+1, N+1))
        Aineq1 = sparse.hstack([sparse.kron(first_diag, G1), middle_u_matrix, slack_matrix, empty_matrix])
        Aineq2 = sparse.hstack([sparse.kron(first_diag, G2), middle_u_matrix, empty_matrix, slack_matrix])
        Aineq3 = sparse.hstack([sparse.csc_matrix((((N+1)*ns, (N+1)*nx + N*nu))), sparse.eye((N+1)*ns)])
        Aineq_u = sparse.hstack([sparse.csc_matrix(((N)*nu, (N+1)*nx)), sparse.eye(N*nu), sparse.csc_matrix(((N)*nu, (N+1)*ns))])
        self.Aineq = sparse.vstack([Aineq1, Aineq2, Aineq3, Aineq_u])
        self.Ax = sparse.kron(sparse.eye(N+1),-sparse.eye(nx)) + sparse.kron(sparse.eye(N+1,k=-1),self.A)    #state transition matrix Ax(k-1)+Bu(k-1) - x(k) = 0
        self.Bu = sparse.kron(sparse.vstack([sparse.csc_matrix((1, N)), sparse.eye(N)]), self.B)
        self.As = sparse.csc_matrix((self.Ax.shape[0], (N+1) * ns))
    #call back functions
    def vel_array_cb(self, msg):
        self.desired_vs = msg.data
    def curvature_cb(self, msg):
        self.curvature = msg.data

    def speed_cb(self, msg: Float32):
        self.current_v = msg.data
        
    def lat_v_cb(self, msg):
        v_lat_gps = msg.twist.twist.linear.y
        self.current_v_lat = v_lat_gps
    
    def clear_result(self):
        self.eta = np.zeros(2*self.horizon)
        self.optimize_acc = np.zeros(self.horizon)
        self.opt_result = np.zeros(2*self.horizon)
        self.future_steer_angle = np.zeros(self.horizon)
        self.future_acc_array = np.zeros(self.horizon)
        self.ys = np.zeros((self.horizon)*3)
        self.ys_prev = np.zeros((self.horizon)*3)

    def update_params(self):
        lateral_error_weight = self.get_parameter('lateral_error_weight').value
        lateral_error_velocity_weight = self.get_parameter('lateral_error_velocity_weight').value
        R_weight = self.get_parameter('R_weight').value
        horizon = self.get_parameter('horizon_length').value
        ts = self.get_parameter('prediction_ts').value
        velocity_error_weight = self.get_parameter('velocity_error_weight').value
        accelration_weight = self.get_parameter('acceleration_weight').value
        rho = self.get_parameter('rho_slack').value

        if (lateral_error_weight != self.lateral_error_weight 
            or lateral_error_velocity_weight != self.lateral_error_velocity_weight
            or R_weight != self.R_weight 
            or self.horizon != horizon 
            or self.ts != ts
            or self.velocity_error_weight != self.velocity_error_weight
            or self.rho_slack != self.rho_slack
            or self.accelration_weight != self.accelration_weight):

            self.horizon = horizon
            self.ts = ts
            self.lateral_error_weight = lateral_error_weight
            self.lateral_error_velocity_weight = lateral_error_velocity_weight
            self.velocity_error_weight = velocity_error_weight
            self.accelration_weight = accelration_weight
            self.R_weight = R_weight
            self.rho_slack = rho
            self.ys = np.zeros((self.horizon)*3)
            self.ys_prev = np.zeros((self.horizon)*3)
            self.clear_result()
            self.construct_matrix()

    def rawimu_cb(self, msg):
        angular_velocity = msg.angular_velocity
        self.current_yaw_rate = angular_velocity.z

        
    def path_cb(self, msg: Path):
        self.current_path.clear()
        for pose in msg.poses:
            position = pose.pose.position
            self.current_path.append([position.x, position.y])
        self.kdtree = cKDTree(self.current_path)

    def find_closest_point(self, robot_state):
        # Query the closest point to the robot's position

        _, index = self.kdtree.query([robot_state[0], robot_state[1]])
        
        closest_point = self.current_path[index]
        
        return closest_point, index

    def normalize_angle(self, yaw):
        while yaw > np.pi / 2:
            yaw -= np.pi
        while yaw < -np.pi / 2:
            yaw += np.pi
        return yaw
        
    def calculate_path_heading(self, closest_idx):
        # Calculate the heading based on the closest point and its neighbors
        if closest_idx == 0:
            next_point = self.current_path[closest_idx + 1]
            closest_point = self.current_path[closest_idx]
        elif closest_idx == len(self.current_path) - 1:
            next_point = self.current_path[closest_idx]
            closest_point = self.current_path[closest_idx - 1]
        else:
            next_point = self.current_path[closest_idx + 1]
            closest_point = self.current_path[closest_idx - 1]
            
        delta_x = next_point[0] - closest_point[0]
        delta_y = next_point[1] - closest_point[1]
        
        theta_path = np.arctan2(delta_y, delta_x)
        
        return self.normalize_angle(theta_path)
    
    def spatial_to_temporal_velocity(self, v_spatial, curvature_spatial, d, N, dt):
        # v_spatial: Array of desired velocities at each spatial point (spatially distributed every 'd' meters)
        # d: The spatial distance between points in meters12
        # N: Number of timesteps in the prediction horizon
        # dt: Time between each timestep in the temporal control horizon

        v_spatial = np.array(v_spatial)
        curvature_spatial = np.array(curvature_spatial)
        v_avg = (v_spatial[:-1] + v_spatial[1:]) / 2

        t_segment = d / v_avg

        t_cumulative = np.cumsum(t_segment)
        t_cumulative = np.insert(t_cumulative, 0, 0)

        t_control = np.arange(0, N * dt, dt)

        interp_func = interp1d(t_cumulative, v_spatial, kind='linear', bounds_error=False, fill_value='extrapolate')

        v_temporal = interp_func(t_control)\
        
        interp_func = interp1d(t_cumulative, curvature_spatial, kind='linear', bounds_error=False, fill_value='extrapolate')
        curvature_temporal = interp_func(t_control)
        
        return v_temporal,curvature_temporal
    
    def get_error(self, x, y, yaw, drift_angle):
        robot_state = [x, y, yaw]
        closest_point, closest_idx = self.find_closest_point(robot_state)
        theta_path = self.calculate_path_heading(closest_idx)
        e_theta = self.normalize_angle(robot_state[2] - theta_path)
        e_y = (robot_state[1] - closest_point[1]) * np.cos(e_theta) - (robot_state[0] - closest_point[0]) * np.sin(e_theta)
        if e_y > 2.0:
            # print(e_y, e_theta)
            pass
        
        return e_y, e_theta, self.desired_vs[closest_idx],self.curvature[closest_idx]
        
    def update(self):
        self.update_counter += 1
        if self.kdtree is None:
            return
        if len(self.desired_vs) == 0:
            return
        v_desired,curvatures = self.spatial_to_temporal_velocity(self.desired_vs, self.curvature,2.0, self.horizon, self.ts)
        target_v = self.desired_vs[0]
        
        # bicycle model prediction
        self.bicycle_model.update_current_state(0.8*self.current_v+0.2*target_v, self.current_v_lat, 
                                                self.current_yaw_rate, self.current_steer_angle, self.future_steer_angle, self.optimize_acc)
        position, v_longs, v_lats, yaw_rates, alpha_fs,alpha_rs = self.bicycle_model.calculate_future_states()   #record the future position and velocity
        msg = Float32()
        msg.data = alpha_fs[0]
        self.publisher_current_frontslip_angle.publish(msg)
        msg = Float32()
        msg.data = alpha_rs[0]
        self.publisher_current_rearslip_angle.publish(msg)
        path_msg = Path() #MPC predicted path
        path_msg.header.frame_id = "vehicle"
        eh_array = []
        curvatures_array = []
        feed_forward_yaw_rate = []
        for i in range(self.horizon):
            position[i][2] = self.normalize_angle(position[i][2])
            pose = PoseStamped()
            pose.pose.position.x = position[i][0]
            pose.pose.position.y = position[i][1]
            path_msg.poses.append(pose)
            v_lat = v_lats[i]
            v_long = np.maximum(v_longs[i],5.0)
            e_l, e_h, v_long_ref, curvatures_ref = self.get_error(*position[i],-np.arctan2(v_lat, v_long) )
            eh_array.append(e_h)  
            curvatures_array.append(curvatures_ref)
            feed_forward_yaw_rate.append(v_longs[i]*curvatures_ref)
            v_long = np.maximum(v_longs[i],5.0)
            v_lat = v_lats[i]
            d = e_l
            drift_angle = 0*np.arctan2(v_lat, v_long)
            d_dot =  np.hypot(v_long, 0*v_lat) * np.sin(e_h-drift_angle)+0*np.cos(e_h)*v_lat
            self.ys[i*3] = v_long
            self.ys[i*3+1] = d
            self.ys[i*3+2] = d_dot
            if i == 0:
                error = Float32()
                error.data = e_l
                self.publisher_lateral_error.publish(error) #current lat error
                heading_error = Float32()
                heading_error.data = e_h
                self.publisher_heading_error.publish(heading_error) #current heading error
        
        X_0 = np.array(self.ys[0:3]-self.ys_prev[0:3])*0  #initial state
        # print("X_0", X_0)
        self.ys_prev = self.ys.copy()
        A = self.A
        B = self.B
        nx = A.shape[0]  #number of states
        nu = B.shape[1]  #number of inputs
        ns = self.slack_variable_number
        N = self.horizon
        Q = sparse.diags([self.velocity_error_weight,self.lateral_error_weight, self.lateral_error_velocity_weight])  #state wieghts
        # QN = Q*10
        QN = sparse.diags([self.velocity_error_weight,self.lateral_error_weight*2, self.lateral_error_velocity_weight*10]) #final state weights
        R = sparse.diags([self.accelration_weight,self.R_weight])   #input weights
        P = sparse.block_diag([sparse.diags([0,0,0]),sparse.kron(sparse.eye(N-1),Q),QN,
                                sparse.kron(sparse.eye(N),R),
                                sparse.kron(sparse.eye(N+1), self.rho_slack * sparse.eye(1)),
                                sparse.kron(sparse.eye(N+1),self.rho_slack * sparse.eye(1))], format='csc') 
        # self.stop_execution()                     #OSQP P matrix, weights for states and inputs, weight 0 for initial state
        Ax = self.Ax
        Bu = self.Bu
        As = self.As
        Aeq = sparse.hstack([Ax, Bu, As])  #osqp equal constriant matrix
        leq = np.hstack([-X_0, np.zeros(N*nx)]) #osqp equal constraint lower bound
        ueq = leq #osqp equal constraint upper bound
        
        Aineq = self.Aineq
\
        X_0_min = np.zeros(1) #initial soft constraints for lat error
        X_0_max = np.zeros(1) 

        X_min1 = np.ones(N)*0.5 - self.ys[1::3] #soft constraints for lat error
        X_min2 = np.ones(N)*-0.5 + self.ys[1::3] 
        X_max1 = np.ones(N)*np.inf - self.ys[1::3] 
        X_max2 = np.ones(N)*np.inf + self.ys[1::3] 
        s_min = np.zeros((N+1)*ns) #slack variable lower 
        s_max = np.ones((N+1)*ns)*np.inf #slack variable upper
        # for i in range (N-1):
        #     X_min[i*(nx-1)+i] = -999
        # X_min_N = np.array([-999,-self.ys[-2]-100, -self.ys[-1]-100]) #final state lower bound
        # X_max_N = np.array([999,-self.ys[-2]+100, -self.ys[-1]+100]) #final state upper bound
        u_min = np.kron(np.ones(N), np.array([-15.0,-40.0])) - np.kron(np.ones(N),np.array([1,1]))*self.opt_result #input lower bound
        u_max = np.kron(np.ones(N), np.array([15.0,40.0])) - np.kron(np.ones(N),np.array([1,1]))*self.opt_result #input upper bound
        combine_min = -np.ones(2*N)    # ax/axmax + ay/aymax <= 1 constraint, doesnt work well now
        combine_max = np.ones(2*N)    # ax/axmax + ay/aymax <= 1 constraint, doesnt work well now
        # for i in range(N):
        #     combine_min[2*i] -= self.eta[i]/15
        #     combine_max[2*i] -= self.eta[i]/15
        #     combine_min[2*i+1] -= self.eta[i]/20
        #     combine_max[2*i+1] -= self.eta[i]/20
        # lineq = np.hstack([X_0_min, X_min, X_min_N, u_min,combine_min])  #lower bound
        # uineq = np.hstack([X_0_max, X_max, X_max_N,u_max, combine_max])   #upper bound
        # lineq = np.hstack([X_0_min, X_min, X_min_N, u_min])  #lower bound
        # uineq = np.hstack([X_0_max, X_max, X_max_N,u_max])   #upper bound
        # lineq = np.hstack([ u_min])  #lower bound
        # uineq = np.hstack([u_max])   #upper bound
        lineq = np.hstack([X_0_min, X_min1,X_0_min, X_min2,s_min, u_min])  #lower bound
        uineq = np.hstack([X_0_max, X_max1,X_0_max, X_max2,s_max, u_max]) 
        # self.stop_execution()
        A = sparse.vstack([Aeq, Aineq])                      #osqp A matrix
        l = np.hstack([leq, lineq])                          #osqp lower bound
        u = np.hstack([ueq, uineq])                          #osqp upper bound
        
        Q_linear = sparse.block_diag([sparse.kron(sparse.eye(N-1), Q),QN], format='csc')
        q_z = np.zeros(3 * N)
        for i in range(N):
            q_z[3 * i] = self.ys[3*i] -v_desired[i]
            q_z[3 * i + 1] = self.ys[3 * i+1]
            q_z[3 * i + 2] = self.ys[3 * i + 2]
        q_z = Q_linear @ q_z #state linear constraints, for -v_desired, prev_el, prev__el_dot
        
        input_matrix = np.zeros(2*N)  #input linear constraints, since ax is optimized directly, cost for a is 0
        for i in range(N):
            input_matrix[2*i] = self.eta[2*i]
            input_matrix[2*i+1] = self.eta[2*i + 1]
        R_N = sparse.block_diag([sparse.kron(sparse.eye(N), R)], format='csc')
        q_input = R_N @ input_matrix
        q = np.hstack([np.zeros(nx), q_z,q_input, np.zeros(2*(N+1))])  #osqp q matrix, linear cost
        # print("q", q.shape)
        # print("P", P.shape)
        # print("L", l.shape)
        # self.stop_execution()
        ####### OSQP optimization #######
        prob = osqp.OSQP()
        prob.setup(P, q, A, l, u, warm_start=True, verbose=False)
        res = prob.solve()
        if res.info.status_val != "solved":
            # print("OSQP did not solve the problem")
            pass
        # for i in range(N):
        #     self.ys[2*i] += res.x[3*(i+1)+1]  
        #     self.ys[2*i+1] += res.x[3*(i+1)+2]
        # print("ys", self.ys)
        self.path_pub.publish(path_msg)
        opt_state = res.x[3*N+2]+self.ys[-2]
        inital_state = self.ys[-2]
        improved_state = np.abs(opt_state) - np.abs(inital_state)
        # print("improved_state", improved_state)
        # print("initial_state",inital_state)
        # print("opt_last_lateral_error",opt_state)
        # print("opt_state",opt_state)
        self.eta += res.x[nx*(N+1):nx*(N+1)+nu*N]
        # print("opt_input",self.eta)
        self.optimize_acc = self.eta[0::2]
        # print("opt_acc",self.optimize_acc) 
        optimized_speed = res.x[:N*3:3]
        self.opt_result = self.eta.copy()

        # self.eta = np.clip(self.eta, -100, 100)
        # self.stop_execution()
        ##assign the future steer angle
        w_array = np.zeros(self.horizon)
        for i in range(self.horizon):
            e_h = eh_array[i]
            if v_longs[i]*np.cos(e_h) != 0:
                w_array[i] = np.clip((self.eta[2*i+1] +np.sin(e_h)*self.optimize_acc[i])/ (v_longs[i] * np.cos(e_h)), -100.0, 100.0) #+ feed_forward_yaw_rate[i]
                # print("feed_forward_yaw_rate",feed_forward_yaw_rate[i])
            else:
                w_array[i] = 0
            K2 = 0.0004
            if self.current_v > 60:
                K2 = 0.00048
            if self.current_v > 75:
                K2 = 0.0005
            if self.current_v > 80:
                K2 = 0.00055
            if self.current_v > 85:
                K2 = 0.00060

            self.future_steer_angle[i] = np.rad2deg((2.9718 + 1*v_longs[i]**2*K2) *w_array[i] / (max(v_longs[i], 5)))  # add a k2 for the bicycle model, roughly measured
            #not correct, main function is reduce the input cost, hence improve the performance a little, doesnt affect the result too much
        desired_w = Float32()
        desired_w.data = yaw_rates[1]
        self.desired_w_pub.publish(desired_w)
        msg = Float32()
        msg.data = self.eta[1]
        self.desired_w_pub.publish(msg)
        msg = Float32MultiArray()
        msg.data = w_array.tolist()
        self.publisher_desired_w_array.publish(msg)
        self.current_steer_angle = self.future_steer_angle[0]
        msg = Float32MultiArray()
        msg.data = self.future_steer_angle.tolist()
        self.publisher_future_steer_angle.publish(msg)

        self.desired_Acc = self.optimize_acc[0]
        # print("desired_Acc", self.desired_Acc)
        msg = JoystickCommand()
        msg.accelerator_cmd = self.desired_Acc
        msg.steering_cmd = self.current_steer_angle
        self.publisher_JoystickCommand.publish(msg)
        msg = Float32()
        msg.data = self.current_yaw_rate
        self.publisher_current_yaw_rate.publish(msg)
        # self.ys_prev = self.ys ###

        if self.update_counter > 1:
            pass
            # self.stop_execution()
            return
            # ax/axmax + ay/aymax <= 1 constraint, doesnt work well now
        # A1 = np.zeros((N, (N+1)*nx)) 
        # A2 = sparse.kron(sparse.eye(N), np.array([1/15,1/20]))
        # Aineq_1 = sparse.hstack([A1, A2])
        # A21 = np.zeros((N, (N+1)*nx))
        # A22 = sparse.kron(sparse.eye(N), np.array([1/15,-1/20]))
        # Aineq_2 = sparse.hstack([A21, A22])
        # Aineq = sparse.vstack([Aineq, Aineq_1, Aineq_2])
    def stop_execution(self):
        # Method to stop the node and ROS 2 execution
        self.get_logger().info('Stopping execution after two updates.')
        self.destroy_node()
        rclpy.shutdown()
        



def main(args=None):
    rclpy.init(args=args)
    mpc = MPC()

    try:
        rclpy.spin(mpc)
    except KeyboardInterrupt:
        pass
    finally:
        mpc.destroy_node()
        rclpy.shutdown()
    

if __name__ == '__main__':
    main()
