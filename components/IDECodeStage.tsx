'use client'

import { useState, useEffect } from 'react'
import { motion, AnimatePresence } from 'framer-motion'
import { Circle, Minus, X, FileCode, Folder, Terminal, Cpu, GitBranch } from 'lucide-react'

interface CodeFile {
  id: string
  name: string
  language: string
  content: string
  icon: React.ReactNode
}

interface TerminalLine {
  id: string
  content: string
  type: 'command' | 'output' | 'success' | 'error' | 'info'
}

const codeFiles: CodeFile[] = [
  {
    id: 'slam_node',
    name: 'slam_node.cpp',
    language: 'cpp',
    icon: <FileCode size={14} />,
    content: `#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>

class SLAMNode {
private:
    ros::NodeHandle nh_;
    ros::Subscriber scan_sub_;
    ros::Publisher map_pub_;
    
    // SLAM parameters
    double resolution_;
    int map_width_, map_height_;
    
public:
    SLAMNode() : nh_("~") {
        // Initialize parameters
        nh_.param("resolution", resolution_, 0.05);
        nh_.param("map_width", map_width_, 1000);
        nh_.param("map_height", map_height_, 1000);
        
        // Setup subscribers and publishers
        scan_sub_ = nh_.subscribe("/scan", 1, 
            &SLAMNode::scanCallback, this);
        map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>(
            "/map", 1, true);
            
        ROS_INFO("SLAM Node initialized");
    }
    
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
        // Process laser scan data
        // Update occupancy grid
        // Publish updated map
        
        nav_msgs::OccupancyGrid map_msg;
        // ... map processing logic ...
        
        map_pub_.publish(map_msg);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "slam_node");
    SLAMNode slam_node;
    ros::spin();
    return 0;
}`
  },
  {
    id: 'controller',
    name: 'controller.py',
    language: 'python',
    icon: <FileCode size={14} />,
    content: `#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

class RobotController:
    def __init__(self):
        rospy.init_node('robot_controller')
        
        # PID parameters
        self.kp = 1.0
        self.ki = 0.1
        self.kd = 0.05
        
        # Control variables
        self.prev_error = 0.0
        self.integral = 0.0
        
        # Publishers and subscribers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        self.rate = rospy.Rate(10)  # 10Hz
        
    def scan_callback(self, scan):
        # Process laser scan data
        min_distance = min(scan.ranges)
        if min_distance < 0.5:  # Obstacle detected
            self.avoid_obstacle(scan)
        else:
            self.follow_path()
    
    def avoid_obstacle(self, scan):
        # Simple obstacle avoidance
        cmd_vel = Twist()
        cmd_vel.angular.z = 0.5  # Turn right
        self.cmd_vel_pub.publish(cmd_vel)
    
    def follow_path(self):
        # Path following logic
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.5
        self.cmd_vel_pub.publish(cmd_vel)
    
    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ == '__main__':
    try:
        controller = RobotController()
        controller.run()
    except rospy.ROSInterruptException:
        pass`
  },
  {
    id: 'cmake',
    name: 'CMakeLists.txt',
    language: 'cmake',
    icon: <FileCode size={14} />,
    content: `cmake_minimum_required(VERSION 3.8)
project(robotics_workspace)

# Find required packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
)

# Include directories
include_directories(
  include
  \${catkin_INCLUDE_DIRS}
)

# Add executable
add_executable(slam_node src/slam_node.cpp)
target_link_libraries(slam_node \${catkin_LIBRARIES})

# Add Python scripts
catkin_install_python(PROGRAMS
  scripts/controller.py
  DESTINATION \${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION \${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)`
  },
  {
    id: 'policy',
    name: 'policy_train.py',
    language: 'python',
    icon: <FileCode size={14} />,
    content: `import torch
import torch.nn as nn
import torch.optim as optim
import numpy as np
from collections import deque
import random

class DQN(nn.Module):
    def __init__(self, state_size, action_size):
        super(DQN, self).__init__()
        self.fc1 = nn.Linear(state_size, 64)
        self.fc2 = nn.Linear(64, 64)
        self.fc3 = nn.Linear(64, action_size)
        
    def forward(self, x):
        x = torch.relu(self.fc1(x))
        x = torch.relu(self.fc2(x))
        return self.fc3(x)

class RobotRLAgent:
    def __init__(self, state_size, action_size):
        self.state_size = state_size
        self.action_size = action_size
        self.memory = deque(maxlen=10000)
        self.gamma = 0.95
        self.epsilon = 1.0
        self.epsilon_min = 0.01
        self.epsilon_decay = 0.995
        self.learning_rate = 0.001
        
        self.model = DQN(state_size, action_size)
        self.optimizer = optim.Adam(self.model.parameters(), lr=self.learning_rate)
        
    def remember(self, state, action, reward, next_state, done):
        self.memory.append((state, action, reward, next_state, done))
        
    def act(self, state):
        if np.random.random() <= self.epsilon:
            return random.randrange(self.action_size)
        
        state = torch.FloatTensor(state).unsqueeze(0)
        q_values = self.model(state)
        return np.argmax(q_values.detach().numpy())
        
    def replay(self, batch_size):
        if len(self.memory) < batch_size:
            return
            
        batch = random.sample(self.memory, batch_size)
        for state, action, reward, next_state, done in batch:
            target = reward
            if not done:
                next_state = torch.FloatTensor(next_state).unsqueeze(0)
                target = reward + self.gamma * np.amax(
                    self.model(next_state).detach().numpy()
                )
            
            state = torch.FloatTensor(state).unsqueeze(0)
            target_f = self.model(state)
            target_f[0][action] = target
            
            self.optimizer.zero_grad()
            loss = nn.MSELoss()(self.model(state), target_f)
            loss.backward()
            self.optimizer.step()
            
        if self.epsilon > self.epsilon_min:
            self.epsilon *= self.epsilon_decay

# Training loop
agent = RobotRLAgent(state_size=10, action_size=4)
for episode in range(1000):
    state = env.reset()
    total_reward = 0
    
    for time in range(500):
        action = agent.act(state)
        next_state, reward, done, _ = env.step(action)
        agent.remember(state, action, reward, next_state, done)
        state = next_state
        total_reward += reward
        
        if done:
            break
            
    agent.replay(32)
    print(f"Episode: {episode}, Score: {total_reward}, Epsilon: {agent.epsilon:.2f}")`
  }
]

const terminalLines: TerminalLine[] = [
  { id: '1', content: '$ catkin_make', type: 'command' },
  { id: '2', content: '-- Found catkin: /opt/ros/noetic/share/catkin', type: 'output' },
  { id: '3', content: '-- Configuring done', type: 'output' },
  { id: '4', content: '-- Generating done', type: 'output' },
  { id: '5', content: '-- Build files have been written to: /home/user/catkin_ws/build', type: 'output' },
  { id: '6', content: 'Build completed successfully in 1.7s', type: 'success' },
  { id: '7', content: '$ roslaunch robot_navigation slam.launch', type: 'command' },
  { id: '8', content: 'process[slam_node-1]: started with pid [12345]', type: 'output' },
  { id: '9', content: 'process[rviz-2]: started with pid [12346]', type: 'output' },
  { id: '10', content: '[INFO] [1640995200.123]: SLAM Node initialized', type: 'info' },
  { id: '11', content: '[INFO] [1640995200.456]: Processing laser scan data...', type: 'info' },
  { id: '12', content: '$ python3 policy_train.py', type: 'command' },
  { id: '13', content: 'Episode: 1, Score: -45, Epsilon: 0.995', type: 'output' },
  { id: '14', content: 'Episode: 2, Score: -32, Epsilon: 0.990', type: 'output' },
  { id: '15', content: 'Episode: 3, Score: -18, Epsilon: 0.985', type: 'output' },
  { id: '16', content: 'Episode: 4, Score: 12, Epsilon: 0.980', type: 'success' },
  { id: '17', content: 'Episode: 5, Score: 28, Epsilon: 0.975', type: 'success' },
  { id: '18', content: 'Training progress: 0.5%', type: 'info' },
]

export function IDECodeStage({ isPlaying }: { isPlaying: boolean }) {
  const [activeTab, setActiveTab] = useState('slam_node')
  const [terminalVisible, setTerminalVisible] = useState(true)
  const [currentLine, setCurrentLine] = useState(0)
  const [typedContent, setTypedContent] = useState('')

  useEffect(() => {
    if (!isPlaying) return

    const tabInterval = setInterval(() => {
      const currentIndex = codeFiles.findIndex(file => file.id === activeTab)
      const nextIndex = (currentIndex + 1) % codeFiles.length
      setActiveTab(codeFiles[nextIndex].id)
    }, 4000)

    return () => clearInterval(tabInterval)
  }, [activeTab, isPlaying])

  useEffect(() => {
    if (!isPlaying) return

    const typeInterval = setInterval(() => {
      const currentFile = codeFiles.find(file => file.id === activeTab)
      if (!currentFile) return

      if (typedContent.length < currentFile.content.length) {
        setTypedContent(currentFile.content.slice(0, typedContent.length + 1))
      } else {
        setTimeout(() => {
          setTypedContent('')
        }, 2000)
      }
    }, 50)

    return () => clearInterval(typeInterval)
  }, [activeTab, typedContent, isPlaying])

  useEffect(() => {
    if (!isPlaying) return

    const terminalInterval = setInterval(() => {
      setCurrentLine(prev => (prev + 1) % terminalLines.length)
    }, 2000)

    return () => clearInterval(terminalInterval)
  }, [isPlaying])

  const activeFile = codeFiles.find(file => file.id === activeTab)

  return (
    <div className="ide-window w-full max-w-4xl mx-auto">
      {/* Window Controls */}
      <div className="flex items-center justify-between bg-background-secondary px-4 py-2 border-b border-border">
        <div className="flex items-center space-x-2">
          <div className="flex space-x-2">
            <div className="w-3 h-3 bg-red-500 rounded-full" />
            <div className="w-3 h-3 bg-yellow-500 rounded-full" />
            <div className="w-3 h-3 bg-green-500 rounded-full" />
          </div>
          <span className="text-sm text-foreground-secondary ml-2">robotics_workspace</span>
        </div>
        <div className="flex items-center space-x-4 text-xs text-foreground-secondary">
          <div className="flex items-center space-x-1">
            <GitBranch size={12} />
            <span>main</span>
          </div>
          <div className="flex items-center space-x-1">
            <Cpu size={12} />
            <span>ROS Noetic</span>
          </div>
        </div>
      </div>

      {/* Tabs */}
      <div className="flex bg-background-secondary border-b border-border">
        {codeFiles.map((file) => (
          <motion.button
            key={file.id}
            onClick={() => setActiveTab(file.id)}
            className={`ide-tab ${activeTab === file.id ? 'active' : ''}`}
            whileHover={{ scale: 1.02 }}
            whileTap={{ scale: 0.98 }}
          >
            <div className="flex items-center space-x-2">
              {file.icon}
              <span>{file.name}</span>
            </div>
          </motion.button>
        ))}
      </div>

      {/* Code Editor */}
      <div className="flex">
        {/* File Explorer */}
        <div className="w-48 bg-background-secondary border-r border-border p-2">
          <div className="space-y-1">
            {codeFiles.map((file) => (
              <motion.div
                key={file.id}
                className={`flex items-center space-x-2 px-2 py-1 rounded text-sm cursor-pointer ${
                  activeTab === file.id ? 'bg-background text-foreground' : 'text-foreground-secondary hover:text-foreground'
                }`}
                whileHover={{ x: 2 }}
              >
                {file.icon}
                <span>{file.name}</span>
              </motion.div>
            ))}
          </div>
        </div>

        {/* Code Area */}
        <div className="flex-1">
          <AnimatePresence mode="wait">
            <motion.div
              key={activeTab}
              initial={{ opacity: 0, x: 20 }}
              animate={{ opacity: 1, x: 0 }}
              exit={{ opacity: 0, x: -20 }}
              transition={{ duration: 0.3 }}
              className="p-4"
            >
              <div className="font-mono text-sm leading-relaxed">
                {typedContent.split('\n').map((line, index) => (
                  <div key={index} className="flex">
                    <span className="text-foreground-secondary mr-4 select-none w-8 text-right">
                      {index + 1}
                    </span>
                    <span className="text-foreground">{line}</span>
                    {index === typedContent.split('\n').length - 1 && (
                      <motion.span
                        animate={{ opacity: [1, 0] }}
                        transition={{ duration: 0.8, repeat: Infinity }}
                        className="text-primary ml-1"
                      >
                        |
                      </motion.span>
                    )}
                  </div>
                ))}
              </div>
            </motion.div>
          </AnimatePresence>
        </div>
      </div>

      {/* Terminal */}
      <AnimatePresence>
        {terminalVisible && (
          <motion.div
            initial={{ height: 0 }}
            animate={{ height: 'auto' }}
            exit={{ height: 0 }}
            className="border-t border-border"
          >
            <div className="flex items-center justify-between bg-background-secondary px-4 py-2">
              <div className="flex items-center space-x-2">
                <Terminal size={14} />
                <span className="text-sm text-foreground-secondary">Terminal</span>
              </div>
              <button
                onClick={() => setTerminalVisible(false)}
                className="text-foreground-secondary hover:text-foreground"
              >
                <X size={14} />
              </button>
            </div>
            <div className="bg-code-background p-4 h-32 overflow-y-auto">
              <div className="space-y-1">
                {terminalLines.slice(0, currentLine + 1).map((line) => (
                  <div
                    key={line.id}
                    className={`terminal-line ${
                      line.type === 'success' ? 'terminal-success' :
                      line.type === 'error' ? 'terminal-error' :
                      line.type === 'info' ? 'terminal-info' :
                      'text-foreground-secondary'
                    }`}
                  >
                    {line.content}
                  </div>
                ))}
                <motion.span
                  animate={{ opacity: [1, 0] }}
                  transition={{ duration: 0.8, repeat: Infinity }}
                  className="text-primary"
                >
                  |
                </motion.span>
              </div>
            </div>
          </motion.div>
        )}
      </AnimatePresence>

      {/* Status Bar */}
      <div className="ide-status-bar">
        <div className="flex items-center space-x-4">
          <span>Ln {typedContent.split('\n').length}, Col {typedContent.length % 80}</span>
          <span>{activeFile?.language.toUpperCase()}</span>
        </div>
        <div className="flex items-center space-x-4">
          <span>UTF-8</span>
          <span>Spaces: 2</span>
          <span>12:34:56</span>
        </div>
      </div>
    </div>
  )
} 
