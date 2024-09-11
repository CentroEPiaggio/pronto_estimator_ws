from typing import List
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from pi3hat_moteus_int_msgs.msg import JointsCommand,JointsStates,PacketPass
from rclpy.context import Context
from rclpy.parameter import Parameter
from rclpy.serialization import serialize_message
import numpy as np
import os
import rosbag2_py
import pandas as pd
from scipy.interpolate import interp1d
from datetime import datetime
### this is the node we used to run the exp 



class JumpingTest(Node):
    def __init__(self,
                node_name = "Jumping_node",
                name_folder_day2try = 'sim_17_10_2023', # 'sim_05_09_2023',
                name_folder_test2try = 'test_q0_bimbi_2023_10_17__17_11_33', #'test_q0_bimbi_2023_10_17__17_39_31', # 'VIDEO_test_q0_bimbi_2023_09_05__16_26_31', # 'VIDEO_test_2023_09_05__16_19_05', # 'VIDEO_test_q0_bimbi_2023_09_05__16_26_31', # ''
                period = 1e-3, ## nome di merda 
                pre_time = 5, # sec
                ids = [1, 2],
                n_joint_sim = 3,
                jnt_names = ['HIP','KNEE'], # QUELLI DELLA CONFIG 
                # jnt_names = ['KNEE'], # QUELLI DELLA CONFIG 
                # jnt_names = ['HIP'], # QUELLI DELLA CONFIG 
                test_name = 'prova_jump', 
                WANNA_SET_TORQUE=False, # False, # True
                TORQUE_MANUAL=False,
                TEST_GRADINO=True,
                q_in = [0.0,0.0]):

        pakage_path = '/home/lorenzo/Desktop/LORENZO_WS/softlegjump_ws/src/fatigue_test/' 
        super().__init__(node_name)
        # create publisher with timer 
        self.pub = self.create_publisher(JointsCommand,
                                        "joint_controller/command",
                                        10)
        self.period = period
        self.WANNA_SET_TORQUE = WANNA_SET_TORQUE  
        self.TEST_GRADINO = TEST_GRADINO
        self.TORQUE_MANUAL = TORQUE_MANUAL

        #self.tau_manual = [25.0, -25.0]
        self.tau_manual = [0.0, 0.0]
  
        self.counter_torque = 0
        self.counter_grad = 0
        #self.q_dot_grad = [25.0, -25.0]
        self.q_dot_grad = [0.0, 0.0]
        if os.path.exists(pakage_path):
            pass
        else:
            assert(False)

        self.name_folder_test_2_try = os.path.join(pakage_path, name_folder_day2try, name_folder_test2try)

        if os.path.exists(self.name_folder_test_2_try):
            pass
        else:
            assert(False)

        self.counter = 0
        self.pre_time = pre_time
        self.jnt_names = jnt_names
        self.n_joint_sim = n_joint_sim
        self.ids = ids
      
        self.clock = self.get_clock()
        self.stt_cbk_count = 0
        self.per_cbk_count = 0
        self.start_node = self.time_to_s(self.clock.now(), 0.0)
        self.dict_2b_send = self.read_from_ddp()
        # self.q_grad = [-1.3999997615814, -1.5] ## in rivoluzioni
        self.q_grad = [-1.83, 0.18]            #qui e' il valore a cui va il gradino
        self.q_final_gradino = [-2.69, 1.57] ## già radianti 
        self.amplitude_des = 1.08 #1.15 # 1.08
        if(TEST_GRADINO):
            q0 = q_in
        else:
            q0 = self.dict_2b_send['q0'] * self.amplitude_des
        
        self.pre_time_steps = int(self.pre_time / self.period)
        q0_real = np.zeros(self.n_joint_sim - 1)
        interp_ = interp1d([0, 1], np.vstack((q0_real, q0)), axis=0, kind='linear')
        self.q_homing = interp_(np.linspace(0, 1, self.pre_time_steps))
    
        self.state_sub = self.create_subscription(JointsStates,
                                                "state_broadcaster/joints_state",
                                                self.callback_state,
                                                10)

        self.perf_sub = self.create_subscription(PacketPass,
                                                "state_broadcaster/performance_indexes",
                                                self.callback_loss,
                                                10)
        # allocaion and set up writer to save data via ros2bag
        self.writer_stt = rosbag2_py.SequentialWriter()
        self.writer_per = rosbag2_py.SequentialWriter()
        self.writer_cmd = rosbag2_py.SequentialWriter()
        print('cane1')
        storage_options_stt = rosbag2_py._storage.StorageOptions(uri = 'bags/' + node_name + '_test_bag/stt_' + test_name,
                                                                storage_id = 'sqlite3')
        storage_options_per = rosbag2_py._storage.StorageOptions(uri = 'bags/' + node_name + '_test_bag/per_' + test_name,
                                                                storage_id = 'sqlite3')
        storage_options_cmd = rosbag2_py._storage.StorageOptions(uri = 'bags/' + node_name + '_test_bag/cmd_' + test_name,
                                                                storage_id = 'sqlite3')
                
        converter_options = rosbag2_py._storage.ConverterOptions('', '')
        print('cane2')
        self.writer_stt.open(storage_options_stt, converter_options)
        self.writer_per.open(storage_options_per, converter_options)
        self.writer_cmd.open(storage_options_cmd, converter_options)
        topic_info_stt = rosbag2_py._storage.TopicMetadata(name = 'motors_state',
                                                        type = 'pi3hat_moteus_int_msgs/msg/JointsStates',
                                                        serialization_format = 'cdr')
        self.writer_stt.create_topic(topic_info_stt)
        topic_info_per = rosbag2_py._storage.TopicMetadata(name = 'motors_perf',
                                                        type = 'pi3hat_moteus_int_msgs/msg/PacketPass',
                                                        serialization_format = 'cdr')
        self.writer_per.create_topic(topic_info_per)
        topic_info_cmd = rosbag2_py._storage.TopicMetadata(name = 'motors_cmmd',
                                                        type = 'pi3hat_moteus_int_msgs/msg/JointsCommand',
                                                        serialization_format = 'cdr')
                                    
        self.writer_cmd.create_topic(topic_info_cmd)

        zero = [0.0,0.0]
        self.msg=JointsCommand()
        self.msg.name = jnt_names
        self.msg.position = zero
        self.msg.velocity = zero
        self.msg.kp_scale = zero
        self.msg.kd_scale = zero
        self.msg.effort = zero
        self.timer = self.create_timer(self.period, self.hold_zero_callback)
        print('cane4')
    def hold_zero_callback(self):
        msg = JointsCommand()
        print('cane')
        if(self.counter < 5000):
            for j in range(len(self.jnt_names)):
                msg.header.stamp = self.clock.now().to_msg()
                print(j)
                msg.name.append(self.jnt_names[j])
                msg.position.append(self.q_homing[0, j])
                msg.velocity.append(0.0)
                msg.kp_scale.append(1.0)
                msg.kd_scale.append(1.0)
                msg.effort.append(0.0)
                
            self.writer_cmd.write("motors_cmmd",
                                serialize_message(msg),
                                self.get_clock().now().nanoseconds)
            self.pub.publish(msg)
        else:
            self.timer.destroy()
            print("[INFO]\tend homing ::::::: jump")
            self.timer = self.create_timer(self.period, self.timer_callback_homing) ## starting the real jump 
            self.counter = 0
        
        self.counter = self.counter + 1    

    def time_to_s(self, time, start):
        [sec, ns] = time.seconds_nanoseconds()
        now = float(sec + ns/pow(10, 9))
        return (now - start)

    def timer_callback_homing(self):
        ## LETS GO HOME 
        msg = JointsCommand()
        time = self.time_to_s(self.clock.now(), self.start_node)
       
     
        if (self.counter < self.pre_time_steps - 1):
            for j in range(len(self.jnt_names)):
                msg.header.stamp = self.clock.now().to_msg()
                msg.name.append(self.jnt_names[j])
                msg.position.append(self.q_homing[self.counter, j])
                msg.velocity.append(0.0)
                msg.kp_scale.append(1.0)
                msg.kd_scale.append(1.0)
                msg.effort.append(0.0)
                
            self.writer_cmd.write("motors_cmmd",
                                serialize_message(msg),
                                self.get_clock().now().nanoseconds)
            self.pub.publish(msg)
        ## HOLDING IN THE HOME CONFIG FOR SAFETY REASON 
        elif (self.counter >= self.pre_time_steps - 1) and (self.counter < self.pre_time_steps + 3000):
            for j in range(len(self.jnt_names)):
                msg.header.stamp = self.clock.now().to_msg()
                msg.name.append(self.jnt_names[j])
                msg.position.append(self.q_homing[-1, j])
                msg.velocity.append(0.0)
                msg.kp_scale.append(1.0)
                msg.kd_scale.append(1.0)
                msg.effort.append(0.0)
                
            self.writer_cmd.write("motors_cmmd",
                                serialize_message(msg),
                                self.get_clock().now().nanoseconds)
            self.pub.publish(msg)

        elif self.counter >= self.pre_time_steps + 3000 - 1: 
            self.timer.destroy()
            print("[INFO]\tend homing ::::::: jump")
            self.timer = self.create_timer(self.period, self.timer_callback_jump) ## starting the real jump 
            self.counter = 0
        
        self.counter = self.counter + 1    

    def hold_callback(self):
        msg = JointsCommand()
        q = self.dict_2b_send['q'] * self.amplitude_des
        q_dot = self.dict_2b_send['q_dot'] * self.amplitude_des
        for i in range(len(self.jnt_names)):
            msg.header.stamp = self.clock.now().to_msg()
            msg.name.append(self.jnt_names[i])
            
            if self.TEST_GRADINO:
                msg.position.append(self.q_homing[-1]) ## GRADINO 
                msg.velocity.append(q_dot[-1, i] * 0) ## GRADINO
            else:
                msg.position.append(q[-1, i])
                msg.velocity.append(q_dot[-1, i])

            msg.kp_scale.append(1.0)
            msg.kd_scale.append(1.0)
            msg.effort.append(0.0)

        self.writer_cmd.write("motors_cmmd",
                                serialize_message(msg),
                                self.get_clock().now().nanoseconds)
        self.pub.publish(msg)
        if self.counter >= 5000:
            raise Exception()
        self.counter = self.counter + 1
        
    def timer_callback_jump(self):
        ## REAL JUMP
        msg = JointsCommand()
        time = self.time_to_s(self.clock.now(), self.start_node)
        
        q = self.dict_2b_send['q'] * self.amplitude_des
        q_dot = self.dict_2b_send['q_dot'] * self.amplitude_des
        torque_ddp = self.dict_2b_send['torque_csv'] * self.amplitude_des

        if self.TEST_GRADINO:
            ## GRADINO
            if self.counter >= len(q[:,1]) - 1: 
                self.timer.destroy()
                print("[INFO]\tend homing ::::::: jump")
                self.timer = self.create_timer(self.period, self.hold_callback)
                self.counter = 0            
                # self.timer.destroy()
            else:
                pass
        else:
            if self.counter >= len(q[:,1]) - 1:
                print('[INFO]:\tend jump ::::::: die')
                raise Exception()
                # self.timer.destroy()
            else:
                pass
    
        for i in range(len(self.jnt_names)):
            msg.header.stamp = self.clock.now().to_msg()
            msg.name.append(self.jnt_names[i])
            
            if self.TEST_GRADINO and self.counter_grad < 200:
                msg.position.append(self.q_grad[i])             ## GRADINO 
                msg.velocity.append(self.q_dot_grad[i])   ## GRADINO 
                msg.kp_scale.append(0.01)
                msg.kd_scale.append(0.01)
                msg.effort.append(self.tau_manual[i])
                self.counter_grad = self.counter_grad + 1
            elif self.TEST_GRADINO and self.counter_grad >= 200:
                msg.position.append(self.q_final_gradino[i])                ## GRADINO 
                msg.velocity.append(q_dot[self.counter, i]*0)               ## GRADINO 
                msg.kp_scale.append(1.0)
                msg.kd_scale.append(1.0)
                msg.effort.append(0.0)
            else:
                msg.position.append(q[self.counter, i])
                msg.velocity.append(q_dot[self.counter, i])
                msg.kp_scale.append(1.0)
                msg.kd_scale.append(1.0)

            # msg.velocity.append(0.0)            

            

            
            # if self.WANNA_SET_TORQUE:
            #     msg.effort.append(torque_ddp[self.counter, i])
            # elif self.TORQUE_MANUAL and self.counter_torque < 600:
            #     msg.effort.append(self.tau_manual[i])
            #     self.counter_torque = self.counter_torque + 1
            # else:
            #     msg.effort.append(0.0)
        
        self.counter = self.counter + 1 
        self.writer_cmd.write("motors_cmmd",
                            serialize_message(msg),
                            self.get_clock().now().nanoseconds)
        self.pub.publish(msg)

 
    def read_from_ddp(self, index = [1, 2]):
        state_csv = pd.read_csv(self.name_folder_test_2_try + '/state.csv')
        q = state_csv.iloc[:, index].to_numpy()
        q_dot = state_csv.iloc[:, list(np.array(index) + self.n_joint_sim)].to_numpy()
        q0 = q[0,:]
        
        torque_csv = pd.read_csv(self.name_folder_test_2_try + '/control.csv').to_numpy()
        
        self.ddp_step = len(state_csv)
        time = np.linspace(0, len(state_csv), self.ddp_step) * self.period
        
        dict_data2test = {"time": time,
                        "q" : q,
                        "q_dot" : q_dot,
                        "q0" : q0,
                        "torque_csv" : torque_csv}
        
        return dict_data2test
 
    def callback_state(self, msg):
        # self.writer_stt.write("motors_state",
        #                     serialize_message(msg),
        #                     self.get_clock().now().nanoseconds)       
        pass

    def callback_loss(self, msg):
        self.per_cbk_count = self.per_cbk_count + 1

#######################################################################################################
#######################################################################################################

## ToDo: save node_name using param from launch file 
## REMEBER WHEN YOU CHANGE THE NODE YOU ALSO HAVE TO CHANGE THE NAME OF THE PYTHON SCRIPT setup.py 
## here : /home/mulinexopc/Documents/softlegjump_ws/src/fatigue_test/setup.py
def main(args=None):
    rclpy.init(args=args)
    name_exp = "softleg_jump_" + datetime.now().strftime("%m_%d_%Y_%H_%M_%S")
    node = JumpingTest( test_name=name_exp,q_in=[-3.9,2.84])    
    ex = MultiThreadedExecutor()
    ex.add_node(node)

    try:
        ex.spin()
        print("pass here")
    except:
        print("Closing ROS2 Node")
        ex.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()
    
if __name__ =="__main__":
    main()
            
    
