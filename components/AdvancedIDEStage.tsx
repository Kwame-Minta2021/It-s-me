'use client'

import { useState, useEffect } from 'react'
import { motion, AnimatePresence } from 'framer-motion'

interface CodeFile {
  id: string
  name: string
  language: string
  content: string[]
  icon: string
}

interface TerminalLine {
  id: string
  content: string
  type: 'command' | 'output' | 'success' | 'error' | 'info'
  delay: number
}

export default function AdvancedIDEStage() {
  const [activeTab, setActiveTab] = useState('slam_node')
  const [terminalVisible, setTerminalVisible] = useState(true)
  const [currentLine, setCurrentLine] = useState(0)
  const [typedContent, setTypedContent] = useState('')
  const [isPlaying, setIsPlaying] = useState(true)
  const [showDiff, setShowDiff] = useState(false)
  const [showToast, setShowToast] = useState(false)

  const codeFiles: CodeFile[] = [
    {
      id: 'slam_node',
      name: 'slam_node.cpp',
      language: 'cpp',
      icon: '📁',
      content: [
        '#include <rclcpp/rclcpp.hpp>',
        '#include <sensor_msgs/msg/laser_scan.hpp>',
        '#include <geometry_msgs/msg/pose_stamped.hpp>',
        '',
        'class SlamNode : public rclcpp::Node {',
        'public:',
        '  SlamNode() : Node("slam_node") {',
        '    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(',
        '      "/scan", 10, std::bind(&SlamNode::onScan, this, std::placeholders::_1));',
        '    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/pose", 10);',
        '  }',
        'private:',
        '  void onScan(const sensor_msgs::msg::LaserScan::SharedPtr msg) {',
        '    // ... pose update (EKF / graph-optimization hook)',
        '    geometry_msgs::msg::PoseStamped pose; /* ... */',
        '    pose_pub_->publish(pose);',
        '  }',
        '  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;',
        '  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;',
        '};'
      ]
    },
    {
      id: 'policy_train',
      name: 'policy_train.py',
      language: 'python',
      icon: '🐍',
      content: [
        'import torch',
        'import numpy as np',
        'from collections import deque',
        '',
        'class DQNAgent:',
        '    def __init__(self, state_size, action_size):',
        '        self.state_size = state_size',
        '        self.action_size = action_size',
        '        self.memory = deque(maxlen=2000)',
        '        self.gamma = 0.95',
        '        self.epsilon = 1.0',
        '        self.epsilon_min = 0.01',
        '        self.epsilon_decay = 0.995',
        '        self.learning_rate = 0.001',
        '        self.model = self._build_model()',
        '',
        '    def _build_model(self):',
        '        model = torch.nn.Sequential(',
        '            torch.nn.Linear(self.state_size, 24),',
        '            torch.nn.ReLU(),',
        '            torch.nn.Linear(24, 24),',
        '            torch.nn.ReLU(),',
        '            torch.nn.Linear(24, self.action_size)',
        '        )',
        '        return model',
        '',
        'for episode in range(num_episodes):',
        '    obs, done = env.reset(), False',
        '    while not done:',
        '        action = policy(obs)',
        '        obs, r, done, info = env.step(action)',
        '        replay.add(obs, action, r, done)',
        '        if len(replay) > warmup:',
        '            loss = agent.update(replay.sample(batch))',
        '    if episode % 10 == 0:',
        '        print(f"[eval] ep={episode} reward={evaluate(agent):.2f}")'
      ]
    },
    {
      id: 'cmake',
      name: 'CMakeLists.txt',
      language: 'cmake',
      icon: '⚙️',
      content: [
        'cmake_minimum_required(VERSION 3.8)',
        'project(slam_robot)',
        '',
        'if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")',
        '  add_compile_options(-Wall -Wextra -Wpedantic)',
        'endif()',
        '',
        'find_package(ament_cmake REQUIRED)',
        'find_package(rclcpp REQUIRED)',
        'find_package(sensor_msgs REQUIRED)',
        'find_package(geometry_msgs REQUIRED)',
        '',
        'add_executable(slam_node src/slam_node.cpp)',
        'ament_target_dependencies(slam_node rclcpp sensor_msgs geometry_msgs)',
        '',
        'install(TARGETS',
        '  slam_node',
        '  DESTINATION lib/${PROJECT_NAME})',
        '',
        'ament_package()'
      ]
    }
  ]

  const terminalLines: TerminalLine[] = [
    { id: '1', content: '$ colcon build', type: 'command', delay: 0 },
    { id: '2', content: 'Starting >>> slam_pkg', type: 'output', delay: 500 },
    { id: '3', content: 'Finished <<< slam_pkg [1.4s]', type: 'success', delay: 1000 },
    { id: '4', content: 'Summary: 1 package finished', type: 'output', delay: 1500 },
    { id: '5', content: '', type: 'output', delay: 2000 },
    { id: '6', content: '$ pytest', type: 'command', delay: 2500 },
    { id: '7', content: '================ 42 passed in 1.71s =================', type: 'success', delay: 3000 },
    { id: '8', content: '', type: 'output', delay: 3500 },
    { id: '9', content: '$ train_rl --env manipulator-v1 --algo dqn', type: 'command', delay: 4000 },
    { id: '10', content: 'Episode 10: avg_reward=+12.3', type: 'info', delay: 4500 },
    { id: '11', content: 'Episode 50: avg_reward=+38.7', type: 'info', delay: 5000 },
    { id: '12', content: 'Episode 100: avg_reward=+64.1 ✓', type: 'success', delay: 5500 }
  ]

  const [visibleTerminalLines, setVisibleTerminalLines] = useState<TerminalLine[]>([])

  useEffect(() => {
    if (!isPlaying) return

    const interval = setInterval(() => {
      setCurrentLine(prev => {
        const currentFile = codeFiles.find(f => f.id === activeTab)
        if (!currentFile) return prev
        
        if (prev < currentFile.content.length) {
          setTypedContent(currentFile.content[prev])
          return prev + 1
        } else {
          // Switch to next tab
          const currentIndex = codeFiles.findIndex(f => f.id === activeTab)
          const nextIndex = (currentIndex + 1) % codeFiles.length
          setActiveTab(codeFiles[nextIndex].id)
          setCurrentLine(0)
          setTypedContent('')
          setShowDiff(true)
          setTimeout(() => setShowDiff(false), 1000)
          setShowToast(true)
          setTimeout(() => setShowToast(false), 3000)
        }
      })
    }, 200)

    return () => clearInterval(interval)
  }, [isPlaying, activeTab, codeFiles])

  useEffect(() => {
    if (!isPlaying) return

    const interval = setInterval(() => {
      setVisibleTerminalLines(prev => {
        const nextLine = terminalLines[prev.length]
        if (nextLine) {
          return [...prev, nextLine]
        }
        return prev
      })
    }, 500)

    return () => clearInterval(interval)
  }, [isPlaying, terminalLines])

  const getTypeColor = (type: string) => {
    switch (type) {
      case 'command': return 'text-primary'
      case 'success': return 'text-accent'
      case 'error': return 'text-red-400'
      case 'info': return 'text-blue-400'
      default: return 'text-foreground-secondary'
    }
  }

  return (
    <div className="relative">
      <div className="bg-code-background border border-border rounded-lg overflow-hidden shadow-2xl">
        {/* Window Controls */}
        <div className="flex items-center justify-between bg-background-secondary px-4 py-2 border-b border-border">
          <div className="flex items-center space-x-2">
            <div className="flex space-x-2">
              <div className="w-3 h-3 bg-red-500 rounded-full"></div>
              <div className="w-3 h-3 bg-yellow-500 rounded-full"></div>
              <div className="w-3 h-3 bg-green-500 rounded-full"></div>
            </div>
            <span className="text-sm text-foreground-secondary ml-2">robotics_workspace</span>
          </div>
          <div className="flex items-center space-x-2">
            <button
              onClick={() => setIsPlaying(!isPlaying)}
              className="text-xs text-foreground-secondary hover:text-foreground transition-colors"
            >
              {isPlaying ? '⏸️' : '▶️'}
            </button>
          </div>
        </div>

        {/* Tabs */}
        <div className="flex bg-background-secondary border-b border-border">
          {codeFiles.map((file) => (
            <button
              key={file.id}
              onClick={() => setActiveTab(file.id)}
              className={`px-4 py-2 text-sm font-mono border-r border-border last:border-r-0 cursor-pointer transition-colors flex items-center space-x-2 ${
                activeTab === file.id
                  ? 'bg-background text-foreground'
                  : 'bg-background-secondary text-foreground-secondary hover:text-foreground'
              }`}
            >
              <span>{file.icon}</span>
              <span>{file.name}</span>
            </button>
          ))}
        </div>

        {/* Code Area */}
        <div className="p-4">
          <AnimatePresence mode="wait">
            <motion.div
              key={activeTab}
              initial={{ opacity: 0, x: 20 }}
              animate={{ opacity: 1, x: 0 }}
              exit={{ opacity: 0, x: -20 }}
              transition={{ duration: 0.3 }}
              className="font-mono text-sm leading-relaxed"
            >
              {codeFiles
                .find(f => f.id === activeTab)
                ?.content.map((line, index) => (
                  <motion.div
                    key={index}
                    initial={{ opacity: 0 }}
                    animate={{ opacity: 1 }}
                    transition={{ delay: index * 0.05 }}
                    className="flex"
                  >
                    <span className="text-foreground-secondary mr-4 select-none w-8 text-right">
                      {index + 1}
                    </span>
                    <span className={`text-foreground ${showDiff && index === currentLine - 1 ? 'bg-green-500/20' : ''}`}>
                      {line}
                    </span>
                    {index === currentLine - 1 && (
                      <motion.span
                        className="text-primary ml-1"
                        animate={{ opacity: [1, 0] }}
                        transition={{ duration: 0.8, repeat: Infinity }}
                      >
                        |
                      </motion.span>
                    )}
                  </motion.div>
                ))}
            </motion.div>
          </AnimatePresence>
        </div>

        {/* Terminal */}
        <div className="border-t border-border">
          <button
            onClick={() => setTerminalVisible(!terminalVisible)}
            className="w-full px-4 py-2 bg-background-secondary text-left text-sm text-foreground-secondary hover:text-foreground transition-colors flex items-center justify-between"
          >
            <span>Terminal</span>
            <span>{terminalVisible ? '▼' : '▶'}</span>
          </button>
          
          <AnimatePresence>
            {terminalVisible && (
              <motion.div
                initial={{ height: 0 }}
                animate={{ height: 'auto' }}
                exit={{ height: 0 }}
                className="overflow-hidden"
              >
                <div className="p-4 bg-background-secondary max-h-48 overflow-y-auto">
                  {visibleTerminalLines.map((line) => (
                    <motion.div
                      key={line.id}
                      initial={{ opacity: 0, y: 10 }}
                      animate={{ opacity: 1, y: 0 }}
                      className={`font-mono text-sm ${getTypeColor(line.type)}`}
                    >
                      {line.content}
                    </motion.div>
                  ))}
                </div>
              </motion.div>
            )}
          </AnimatePresence>
        </div>
      </div>

      {/* Toast Notification */}
      <AnimatePresence>
        {showToast && (
          <motion.div
            initial={{ opacity: 0, y: 20 }}
            animate={{ opacity: 1, y: 0 }}
            exit={{ opacity: 0, y: -20 }}
            className="absolute top-4 right-4 bg-accent text-background px-4 py-2 rounded-lg shadow-lg"
          >
            Build passed in 1.7s ✓
          </motion.div>
        )}
      </AnimatePresence>
    </div>
  )
}
