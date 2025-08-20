'use client'

import { motion, useScroll, useTransform, useInView } from 'framer-motion'
import { useRef, useEffect, useState } from 'react'

export default function Home() {
  const [mousePosition, setMousePosition] = useState({ x: 0, y: 0 });
  const { scrollY } = useScroll();
  const y = useTransform(scrollY, [0, 1000], [0, -200]);
  const opacity = useTransform(scrollY, [0, 300], [1, 0]);

  useEffect(() => {
    const handleMouseMove = (e: MouseEvent) => {
      setMousePosition({ x: e.clientX, y: e.clientY });
    };
    window.addEventListener('mousemove', handleMouseMove);
    return () => window.removeEventListener('mousemove', handleMouseMove);
  }, []);

      return (
      <main className="min-h-screen bg-background text-foreground overflow-x-hidden">
        {/* Scroll Progress Bar */}
        <motion.div
          className="fixed top-0 left-0 right-0 h-1 bg-gradient-to-r from-primary to-accent z-50 origin-left"
          style={{ scaleX: useTransform(scrollY, [0, 1000], [0, 1]) }}
        />
        
        {/* Floating Cursor Effect */}
        <motion.div
          className="fixed w-4 h-4 bg-primary rounded-full pointer-events-none z-50 mix-blend-difference"
          animate={{
            x: mousePosition.x - 8,
            y: mousePosition.y - 8,
          }}
          transition={{ type: "spring", stiffness: 500, damping: 28 }}
        />
        
        {/* Navigation */}
        <motion.nav 
          className="fixed top-0 left-0 right-0 z-50 bg-background/80 backdrop-blur-sm border-b border-border"
          style={{ opacity }}
          initial={{ y: -100 }}
          animate={{ y: 0 }}
          transition={{ duration: 0.8, ease: "easeOut" }}
        >
        <div className="max-w-7xl mx-auto px-4 sm:px-6 lg:px-8">
          <div className="flex justify-between items-center h-16">
            <div className="flex items-center space-x-2">
              <div className="w-8 h-8 bg-gradient-to-r from-primary to-accent rounded-lg flex items-center justify-center">
                <span className="text-background font-bold text-sm">F</span>
              </div>
              <span className="font-bold text-lg bg-gradient-to-r from-primary to-accent bg-clip-text text-transparent">
                Frederick Kwame Minta
              </span>
            </div>
                                <div className="hidden md:flex items-center space-x-8">
                      <a href="#about" className="text-foreground-secondary hover:text-foreground transition-colors">About</a>
                      <a href="#projects" className="text-foreground-secondary hover:text-foreground transition-colors">Projects</a>
                      <a href="#research" className="text-foreground-secondary hover:text-foreground transition-colors">Research</a>
                      <a href="#experience" className="text-foreground-secondary hover:text-foreground transition-colors">Experience</a>
                      <a href="#contact" className="text-foreground-secondary hover:text-foreground transition-colors">Contact</a>
                    </div>
                    
                    {/* Mobile menu button */}
                    <div className="md:hidden">
                      <motion.button
                        className="p-2 rounded-lg border border-border hover:bg-background-secondary transition-colors"
                        whileHover={{ scale: 1.05 }}
                        whileTap={{ scale: 0.95 }}
                      >
                        <svg className="w-5 h-5" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                          <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M4 6h16M4 12h16M4 18h16" />
                        </svg>
                      </motion.button>
                    </div>
                      </div>
          </div>
        </motion.nav>

                    {/* Hero Section */}
              <motion.section 
                className="min-h-screen flex items-center justify-center pt-16 relative overflow-hidden"
                style={{ y }}
              >
                {/* Enhanced Animated Background Elements */}
                <div className="absolute inset-0 overflow-hidden">
                  {/* Grid Pattern */}
                  <div className="absolute inset-0 opacity-5">
                    <div className="absolute inset-0" style={{
                      backgroundImage: `radial-gradient(circle at 1px 1px, #00D4FF 1px, transparent 0)`,
                      backgroundSize: '50px 50px'
                    }} />
                  </div>
                  
                  {/* Floating particles with more variety */}
                  {[...Array(12)].map((_, i) => (
                    <motion.div
                      key={i}
                      className={`absolute rounded-full ${
                        i % 3 === 0 ? 'bg-primary/30' : 
                        i % 3 === 1 ? 'bg-accent/30' : 'bg-foreground/20'
                      }`}
                      style={{
                        width: `${4 + (i % 3) * 2}px`,
                        height: `${4 + (i % 3) * 2}px`,
                        left: `${10 + i * 8}%`,
                        top: `${20 + i * 6}%`,
                      }}
                      animate={{
                        y: [0, -30, 0],
                        x: [0, 10, 0],
                        opacity: [0.1, 0.8, 0.1],
                        scale: [1, 1.8, 1],
                        rotate: [0, 180, 360],
                      }}
                      transition={{
                        duration: 4 + i * 0.3,
                        repeat: Infinity,
                        ease: "easeInOut",
                        delay: i * 0.2,
                      }}
                    />
                  ))}
                  
                  {/* Animated geometric shapes */}
                  <motion.div
                    className="absolute top-32 left-32 w-40 h-40 border border-primary/20 rounded-full"
                    animate={{
                      rotate: 360,
                      scale: [1, 1.1, 1],
                      opacity: [0.1, 0.3, 0.1],
                    }}
                    transition={{
                      rotate: { duration: 20, repeat: Infinity, ease: "linear" },
                      scale: { duration: 6, repeat: Infinity, ease: "easeInOut" },
                      opacity: { duration: 4, repeat: Infinity, ease: "easeInOut" }
                    }}
                  />
                  
                  <motion.div
                    className="absolute bottom-32 right-32 w-32 h-32 border border-accent/20 rotate-45"
                    animate={{
                      rotate: -360,
                      scale: [1, 1.2, 1],
                      opacity: [0.1, 0.4, 0.1],
                    }}
                    transition={{
                      rotate: { duration: 25, repeat: Infinity, ease: "linear" },
                      scale: { duration: 5, repeat: Infinity, ease: "easeInOut", delay: 1 },
                      opacity: { duration: 3, repeat: Infinity, ease: "easeInOut", delay: 0.5 }
                    }}
                  />
                  
                  {/* Glowing orbs */}
                  <motion.div
                    className="absolute top-1/4 right-1/4 w-20 h-20 bg-primary/10 rounded-full blur-xl"
                    animate={{
                      scale: [1, 1.3, 1],
                      opacity: [0.2, 0.6, 0.2],
                      x: [0, 20, 0],
                      y: [0, -20, 0],
                    }}
                    transition={{
                      duration: 8,
                      repeat: Infinity,
                      ease: "easeInOut"
                    }}
                  />
                  
                  <motion.div
                    className="absolute bottom-1/3 left-1/3 w-16 h-16 bg-accent/10 rounded-full blur-xl"
                    animate={{
                      scale: [1, 1.4, 1],
                      opacity: [0.1, 0.5, 0.1],
                      x: [0, -15, 0],
                      y: [0, 15, 0],
                    }}
                    transition={{
                      duration: 6,
                      repeat: Infinity,
                      ease: "easeInOut",
                      delay: 2
                    }}
                  />
                </div>
        
                        <div className="max-w-7xl mx-auto px-4 sm:px-6 lg:px-8 w-full relative z-10">
                  <div className="grid lg:grid-cols-2 gap-8 lg:gap-12 items-center">
            {/* Left Content */}
            <motion.div
              initial={{ opacity: 0, x: -100 }}
              animate={{ opacity: 1, x: 0 }}
              transition={{ duration: 1, ease: "easeOut" }}
              className="space-y-8"
            >
              <div className="space-y-4">
                                        <motion.h1 
                          className="text-3xl sm:text-4xl md:text-5xl lg:text-6xl font-bold leading-tight"
                          initial={{ opacity: 0, y: 20 }}
                          animate={{ opacity: 1, y: 0 }}
                          transition={{ duration: 0.8, delay: 0.2 }}
                        >
                  <motion.span 
                    className="text-foreground"
                    initial={{ opacity: 0 }}
                    animate={{ opacity: 1 }}
                    transition={{ duration: 0.5, delay: 0.4 }}
                  >
                    Building Intelligent
                  </motion.span>
                  <br />
                  <motion.span 
                    className="bg-gradient-to-r from-primary to-accent bg-clip-text text-transparent"
                    initial={{ opacity: 0, scale: 0.8 }}
                    animate={{ opacity: 1, scale: 1 }}
                    transition={{ duration: 0.8, delay: 0.6, ease: "easeOut" }}
                  >
                    Robotic Systems
                  </motion.span>
                </motion.h1>
                                        <p className="text-lg sm:text-xl text-foreground-secondary max-w-2xl">
                          Robotics software engineer specializing in ROS2, AI-driven control, and sensor-rich systems—from assistive robots to IoT platforms at scale.
                        </p>
              </div>

                                    {/* Quick Facts */}
                      <motion.div 
                        className="grid grid-cols-1 sm:grid-cols-2 gap-3 sm:gap-4"
                        initial={{ opacity: 0, y: 20 }}
                        animate={{ opacity: 1, y: 0 }}
                        transition={{ duration: 0.8, delay: 0.8 }}
                      >
                {[
                  "ROS2", "C++/Python", "SLAM & Perception", 
                  "Reinforcement Learning", "Embedded/IoT", "Linux/CI/CD"
                ].map((skill, index) => (
                  <motion.div
                    key={skill}
                    className="flex items-center space-x-2"
                    initial={{ opacity: 0, x: -20 }}
                    animate={{ opacity: 1, x: 0 }}
                    transition={{ 
                      duration: 0.5, 
                      delay: 1 + index * 0.1,
                      ease: "easeOut"
                    }}
                    whileHover={{ 
                      scale: 1.05,
                      x: 5,
                      transition: { duration: 0.2 }
                    }}
                  >
                    <motion.div 
                      className="w-2 h-2 bg-primary rounded-full"
                      animate={{ 
                        scale: [1, 1.2, 1],
                        opacity: [0.7, 1, 0.7]
                      }}
                      transition={{
                        duration: 2,
                        repeat: Infinity,
                        delay: index * 0.3
                      }}
                    />
                                                <span className="text-xs sm:text-sm text-foreground-secondary">{skill}</span>
                  </motion.div>
                ))}
              </motion.div>

                                    {/* CTA Buttons */}
                      <motion.div 
                        className="flex flex-col sm:flex-row gap-3 sm:gap-4"
                        initial={{ opacity: 0, y: 20 }}
                        animate={{ opacity: 1, y: 0 }}
                        transition={{ duration: 0.8, delay: 1.5 }}
                      >
                                        <motion.a 
                          href="https://github.com/kwame-minta2021"
                          target="_blank"
                          rel="noopener noreferrer"
                          className="flex items-center justify-center space-x-2 px-6 sm:px-8 py-3 sm:py-4 bg-primary text-background rounded-lg font-medium text-sm sm:text-base"
                          whileHover={{ 
                            scale: 1.05,
                            boxShadow: "0 10px 30px rgba(0, 212, 255, 0.3)",
                            transition: { duration: 0.2 }
                          }}
                          whileTap={{ scale: 0.95 }}
                          transition={{ duration: 0.2 }}
                        >
                  <motion.span
                    initial={{ opacity: 0, x: -10 }}
                    animate={{ opacity: 1, x: 0 }}
                    transition={{ delay: 1.7, duration: 0.5 }}
                  >
                    View Projects
                  </motion.span>
                </motion.a>
                                        <motion.button 
                          className="flex items-center justify-center space-x-2 px-6 sm:px-8 py-3 sm:py-4 border border-primary text-primary rounded-lg font-medium text-sm sm:text-base"
                          whileHover={{ 
                            scale: 1.05,
                            backgroundColor: "rgba(0, 212, 255, 0.1)",
                            boxShadow: "0 10px 30px rgba(0, 212, 255, 0.2)",
                            transition: { duration: 0.2 }
                          }}
                          whileTap={{ scale: 0.95 }}
                          transition={{ duration: 0.2 }}
                        >
                  <motion.span
                    initial={{ opacity: 0, x: -10 }}
                    animate={{ opacity: 1, x: 0 }}
                    transition={{ delay: 1.9, duration: 0.5 }}
                  >
                    Download CV
                  </motion.span>
                </motion.button>
              </motion.div>
            </motion.div>

                                {/* Right IDE Stage */}
                    <motion.div
                      initial={{ opacity: 0, x: 100, scale: 0.8 }}
                      animate={{ opacity: 1, x: 0, scale: 1 }}
                      transition={{ duration: 1.2, delay: 0.5, ease: "easeOut" }}
                      className="relative order-first lg:order-last"
                      whileHover={{ 
                        scale: 1.02,
                        transition: { duration: 0.3 }
                      }}
                    >
                                    <div className="bg-code-background border border-border rounded-lg overflow-hidden shadow-2xl max-w-full">
                        {/* Window Controls */}
                        <div className="flex items-center justify-between bg-background-secondary px-3 sm:px-4 py-2 border-b border-border">
                  <div className="flex items-center space-x-2">
                    <div className="flex space-x-2">
                      <div className="w-3 h-3 bg-red-500 rounded-full"></div>
                      <div className="w-3 h-3 bg-yellow-500 rounded-full"></div>
                      <div className="w-3 h-3 bg-green-500 rounded-full"></div>
                    </div>
                                                <span className="text-xs sm:text-sm text-foreground-secondary ml-2">robotics_workspace</span>
                  </div>
                </div>

                                        {/* Tabs */}
                        <div className="flex bg-background-secondary border-b border-border overflow-x-auto">
                          <div className="px-3 sm:px-4 py-2 text-xs sm:text-sm font-mono border-r border-border bg-background text-foreground flex items-center space-x-1 sm:space-x-2 flex-shrink-0">
                            <span>📁</span>
                            <span className="hidden sm:inline">slam_node.cpp</span>
                            <span className="sm:hidden">slam.cpp</span>
                          </div>
                          <div className="px-3 sm:px-4 py-2 text-xs sm:text-sm font-mono border-r border-border bg-background-secondary text-foreground-secondary flex items-center space-x-1 sm:space-x-2 flex-shrink-0">
                            <span>🐍</span>
                            <span className="hidden sm:inline">policy_train.py</span>
                            <span className="sm:hidden">train.py</span>
                          </div>
                          <div className="px-3 sm:px-4 py-2 text-xs sm:text-sm font-mono bg-background-secondary text-foreground-secondary flex items-center space-x-1 sm:space-x-2 flex-shrink-0">
                            <span>⚙️</span>
                            <span className="hidden sm:inline">CMakeLists.txt</span>
                            <span className="sm:hidden">CMake.txt</span>
                          </div>
                        </div>

                                        {/* Code Area */}
                        <div className="p-3 sm:p-4">
                          <div className="font-mono text-xs sm:text-sm leading-relaxed">
                    <div className="flex">
                                                  <span className="text-foreground-secondary mr-2 sm:mr-4 select-none w-6 sm:w-8 text-right">1</span>
                      <span className="text-foreground">#include &lt;rclcpp/rclcpp.hpp&gt;</span>
                    </div>
                    <div className="flex">
                      <span className="text-foreground-secondary mr-4 select-none w-8 text-right">2</span>
                      <span className="text-foreground">#include &lt;sensor_msgs/msg/laser_scan.hpp&gt;</span>
                    </div>
                    <div className="flex">
                      <span className="text-foreground-secondary mr-4 select-none w-8 text-right">3</span>
                      <span className="text-foreground">#include &lt;geometry_msgs/msg/pose_stamped.hpp&gt;</span>
                    </div>
                    <div className="flex">
                      <span className="text-foreground-secondary mr-4 select-none w-8 text-right">4</span>
                      <span className="text-foreground"></span>
                    </div>
                    <div className="flex">
                      <span className="text-foreground-secondary mr-4 select-none w-8 text-right">5</span>
                      <span className="text-foreground">class SlamNode : public rclcpp::Node {'{'}</span>
                    </div>
                    <div className="flex">
                      <span className="text-foreground-secondary mr-4 select-none w-8 text-right">6</span>
                      <span className="text-foreground">public:</span>
                    </div>
                    <div className="flex">
                      <span className="text-foreground-secondary mr-4 select-none w-8 text-right">7</span>
                      <span className="text-foreground">  SlamNode() : Node("slam_node") {'{'}</span>
                    </div>
                    <div className="flex">
                      <span className="text-foreground-secondary mr-4 select-none w-8 text-right">8</span>
                      <span className="text-foreground">    scan_sub_ = create_subscription&lt;</span>
                    </div>
                    <div className="flex">
                      <span className="text-foreground-secondary mr-4 select-none w-8 text-right">9</span>
                      <span className="text-foreground">      sensor_msgs::msg::LaserScan&gt;(</span>
                    </div>
                    <div className="flex">
                      <span className="text-foreground-secondary mr-4 select-none w-8 text-right">10</span>
                      <span className="text-foreground">      "/scan", 10, std::bind(&amp;SlamNode::onScan,</span>
                    </div>
                    <div className="flex">
                      <span className="text-foreground-secondary mr-4 select-none w-8 text-right">11</span>
                      <span className="text-foreground">      this, std::placeholders::_1));</span>
                    </div>
                    <div className="flex">
                      <span className="text-foreground-secondary mr-4 select-none w-8 text-right">12</span>
                      <span className="text-foreground">  {'}'}</span>
                    </div>
                    <div className="flex">
                      <span className="text-foreground-secondary mr-4 select-none w-8 text-right">13</span>
                      <span className="text-foreground">private:</span>
                    </div>
                    <div className="flex">
                      <span className="text-foreground-secondary mr-4 select-none w-8 text-right">14</span>
                      <span className="text-foreground">  void onScan(const sensor_msgs::msg::</span>
                    </div>
                    <div className="flex">
                      <span className="text-foreground-secondary mr-4 select-none w-8 text-right">15</span>
                      <motion.span 
                        className="text-primary"
                        animate={{ opacity: [1, 0, 1] }}
                        transition={{ duration: 1, repeat: Infinity }}
                      >
                        |
                      </motion.span>
                    </div>
                  </div>
                </div>

                {/* Terminal */}
                <div className="border-t border-border">
                  <div className="px-4 py-2 bg-background-secondary text-left text-sm text-foreground-secondary flex items-center justify-between">
                    <span>Terminal</span>
                    <span>▼</span>
                  </div>
                  <div className="p-4 bg-background-secondary max-h-48 overflow-y-auto">
                    <div className="font-mono text-sm text-primary">$ colcon build</div>
                    <div className="font-mono text-sm text-foreground-secondary">Starting &gt;&gt;&gt; slam_pkg</div>
                    <div className="font-mono text-sm text-accent">Finished &lt;&lt;&lt; slam_pkg [1.4s]</div>
                    <div className="font-mono text-sm text-foreground-secondary">Summary: 1 package finished</div>
                    <div className="font-mono text-sm text-foreground-secondary"></div>
                    <div className="font-mono text-sm text-primary">$ pytest</div>
                    <div className="font-mono text-sm text-accent">================ 42 passed in 1.71s =================</div>
                    <div className="font-mono text-sm text-foreground-secondary"></div>
                    <div className="font-mono text-sm text-primary">$ train_rl --env manipulator-v1 --algo dqn</div>
                    <div className="font-mono text-sm text-blue-400">Episode 10: avg_reward=+12.3</div>
                    <div className="font-mono text-sm text-blue-400">Episode 50: avg_reward=+38.7</div>
                    <div className="font-mono text-sm text-accent">Episode 100: avg_reward=+64.1 ✓</div>
                  </div>
                </div>
              </div>
            </motion.div>
          </div>
        </div>
      </motion.section>

                    {/* About Section */}
              <motion.section 
                id="about" 
                className="py-24 bg-background-secondary relative overflow-hidden"
                initial={{ opacity: 0 }}
                whileInView={{ opacity: 1 }}
                transition={{ duration: 1 }}
                viewport={{ once: true }}
              >
                {/* Enhanced Animated background pattern */}
                <div className="absolute inset-0 opacity-5">
                  {/* Circuit-like pattern */}
                  <div className="absolute inset-0" style={{
                    backgroundImage: `linear-gradient(90deg, transparent 98%, #00D4FF 100%), linear-gradient(0deg, transparent 98%, #00D4FF 100%)`,
                    backgroundSize: '50px 50px'
                  }} />
                  
                  <motion.div
                    className="absolute top-20 left-20 w-24 h-24 border border-primary rounded-full"
                    animate={{
                      rotate: 360,
                      scale: [1, 1.3, 1],
                      opacity: [0.1, 0.4, 0.1],
                    }}
                    transition={{
                      rotate: { duration: 25, repeat: Infinity, ease: "linear" },
                      scale: { duration: 6, repeat: Infinity, ease: "easeInOut" },
                      opacity: { duration: 4, repeat: Infinity, ease: "easeInOut" }
                    }}
                  />
                  <motion.div
                    className="absolute bottom-20 right-20 w-20 h-20 border border-accent rounded-full"
                    animate={{
                      rotate: -360,
                      scale: [1, 1.4, 1],
                      opacity: [0.1, 0.3, 0.1],
                    }}
                    transition={{
                      rotate: { duration: 20, repeat: Infinity, ease: "linear" },
                      scale: { duration: 5, repeat: Infinity, ease: "easeInOut", delay: 1 },
                      opacity: { duration: 3, repeat: Infinity, ease: "easeInOut", delay: 0.5 }
                    }}
                  />
                  <motion.div
                    className="absolute top-1/2 left-1/2 w-16 h-16 border border-primary/50 rounded-full"
                    animate={{
                      rotate: 180,
                      scale: [1, 1.2, 1],
                      opacity: [0.05, 0.2, 0.05],
                    }}
                    transition={{
                      rotate: { duration: 30, repeat: Infinity, ease: "linear" },
                      scale: { duration: 7, repeat: Infinity, ease: "easeInOut", delay: 2 },
                      opacity: { duration: 5, repeat: Infinity, ease: "easeInOut", delay: 1 }
                    }}
                  />
                </div>
        <div className="max-w-7xl mx-auto px-4 sm:px-6 lg:px-8">
          <motion.div
            initial={{ opacity: 0, y: 20 }}
            whileInView={{ opacity: 1, y: 0 }}
            transition={{ duration: 0.8 }}
            viewport={{ once: true }}
            className="text-center mb-16"
          >
            <h2 className="text-3xl md:text-4xl font-bold mb-4">
              About Me
            </h2>
            <p className="text-foreground-secondary text-lg max-w-4xl mx-auto">
              I am a robotics software engineer focused on delivering reliable autonomy and human-centered robotic solutions. My work spans assistive robotics, precision agriculture, and environmental monitoring, combining ROS2/C++/Python, reinforcement learning, and embedded IoT to move systems from prototype to field deployment.
            </p>
          </motion.div>

                            <div className="grid md:grid-cols-3 gap-8 lg:gap-12 items-start">
            {/* Photo Section */}
            <motion.div
              initial={{ opacity: 0, y: 20 }}
              whileInView={{ opacity: 1, y: 0 }}
              transition={{ duration: 0.8 }}
              viewport={{ once: true }}
              className="md:col-span-1"
            >
                                    <div className="bg-background rounded-lg border border-border p-4 sm:p-6 text-center">
                        {/* Your Photo */}
                        <motion.div 
                          className="w-32 h-40 sm:w-48 sm:h-64 mx-auto mb-4 rounded-lg overflow-hidden border-2 border-primary/30"
                          whileHover={{ 
                            scale: 1.05,
                            boxShadow: "0 20px 40px rgba(0, 212, 255, 0.3)",
                            transition: { duration: 0.3 }
                          }}
                        >
                  <motion.img 
                    src="/images/Fred.jpg" 
                    alt="Frederick Kwame Minta - Robotics Software Engineer"
                    className="w-full h-full object-cover"
                    whileHover={{ 
                      scale: 1.1,
                      transition: { duration: 0.3 }
                    }}
                  />
                </motion.div>
                                        <h3 className="text-lg sm:text-xl font-bold mb-2">Frederick Kwame Minta</h3>
                        <p className="text-foreground-secondary text-xs sm:text-sm mb-4">B.Sc. Computer Science & Engineering</p>
                <div className="text-xs text-foreground-secondary space-y-1">
                  <p>UMaT • Computer Science & Engineering</p>
                  <p>Graduated 2024</p>
                  <p className="text-primary font-medium">Habakkuk 2:3</p>
                </div>
              </div>
            </motion.div>

            {/* Core Competencies */}
            <motion.div
              initial={{ opacity: 0, x: -50 }}
              whileInView={{ opacity: 1, x: 0 }}
              transition={{ duration: 0.8 }}
              viewport={{ once: true }}
              className="md:col-span-1"
            >
              <h3 className="text-2xl font-bold mb-4">Core Competencies</h3>
              <div className="space-y-4">
                <div className="flex items-center space-x-3">
                  <div className="w-2 h-2 bg-primary rounded-full"></div>
                  <span>ROS2 & MoveIt</span>
                </div>
                <div className="flex items-center space-x-3">
                  <div className="w-2 h-2 bg-primary rounded-full"></div>
                  <span>Reinforcement Learning</span>
                </div>
                <div className="flex items-center space-x-3">
                  <div className="w-2 h-2 bg-primary rounded-full"></div>
                  <span>Computer Vision & Perception</span>
                </div>
                <div className="flex items-center space-x-3">
                  <div className="w-2 h-2 bg-primary rounded-full"></div>
                  <span>Embedded IoT Systems</span>
                </div>
                <div className="flex items-center space-x-3">
                  <div className="w-2 h-2 bg-primary rounded-full"></div>
                  <span>Assistive Robotics</span>
                </div>
              </div>
            </motion.div>

            {/* Quick Facts */}
            <motion.div
              initial={{ opacity: 0, x: 50 }}
              whileInView={{ opacity: 1, x: 0 }}
              transition={{ duration: 0.8 }}
              viewport={{ once: true }}
              className="md:col-span-1"
            >
              <div className="bg-background rounded-lg border border-border p-6">
                <h3 className="text-xl font-bold mb-4">Education & Skills</h3>
                <div className="space-y-3">
                  <div className="flex justify-between">
                    <span className="text-foreground-secondary">Degree:</span>
                    <span className="font-medium">B.Sc. CS&E</span>
                  </div>
                  <div className="flex justify-between">
                    <span className="text-foreground-secondary">University:</span>
                    <span className="font-medium">UMaT</span>
                  </div>

                  <div className="flex justify-between">
                    <span className="text-foreground-secondary">Graduation:</span>
                    <span className="font-medium">2024</span>
                  </div>
                                            <div className="flex justify-between">
                            <span className="text-foreground-secondary">Location:</span>
                            <span className="font-medium">Accra-Legon</span>
                          </div>
                </div>
              </div>
            </motion.div>
          </div>
        </div>
      </motion.section>

                    {/* Projects Section */}
              <motion.section 
                id="projects" 
                className="py-24 bg-background relative overflow-hidden"
                initial={{ opacity: 0 }}
                whileInView={{ opacity: 1 }}
                transition={{ duration: 1 }}
                viewport={{ once: true }}
              >
                {/* Animated background elements */}
                <div className="absolute inset-0 opacity-5">
                  {/* Hexagonal pattern */}
                  <div className="absolute inset-0" style={{
                    backgroundImage: `url("data:image/svg+xml,%3Csvg width='60' height='60' viewBox='0 0 60 60' xmlns='http://www.w3.org/2000/svg'%3E%3Cg fill='none' fill-rule='evenodd'%3E%3Cg fill='%2300D4FF' fill-opacity='0.1'%3E%3Cpath d='M30 0l30 17.32v34.64L30 69.28 0 51.96V17.32L30 0z'/%3E%3C/g%3E%3C/g%3E%3C/svg%3E")`,
                    backgroundSize: '60px 60px'
                  }} />
                  
                  {/* Floating tech icons */}
                  {[...Array(8)].map((_, i) => (
                    <motion.div
                      key={i}
                      className="absolute text-2xl opacity-10"
                      style={{
                        left: `${15 + i * 10}%`,
                        top: `${20 + i * 8}%`,
                      }}
                      animate={{
                        y: [0, -15, 0],
                        rotate: [0, 5, -5, 0],
                        opacity: [0.05, 0.15, 0.05],
                      }}
                      transition={{
                        duration: 5 + i * 0.5,
                        repeat: Infinity,
                        ease: "easeInOut",
                        delay: i * 0.3,
                      }}
                    >
                      {['🤖', '⚡', '🌐', '🔧', '📱', '💻', '🚀', '🎯'][i]}
                    </motion.div>
                  ))}
                </div>
        <div className="max-w-7xl mx-auto px-4 sm:px-6 lg:px-8">
                            <motion.div
                    initial={{ opacity: 0, y: 20 }}
                    whileInView={{ opacity: 1, y: 0 }}
                    transition={{ duration: 0.8 }}
                    viewport={{ once: true }}
                    className="text-center mb-16"
                  >
                    <motion.h2 
                      className="text-3xl md:text-4xl font-bold mb-4"
                      initial={{ opacity: 0, scale: 0.8 }}
                      whileInView={{ opacity: 1, scale: 1 }}
                      transition={{ duration: 0.8, ease: "easeOut" }}
                      viewport={{ once: true }}
                    >
                      <span className="bg-gradient-to-r from-primary to-accent bg-clip-text text-transparent">
                        Featured Projects
                      </span>
                    </motion.h2>
                    <motion.p 
                      className="text-foreground-secondary text-lg max-w-2xl mx-auto"
                      initial={{ opacity: 0, y: 20 }}
                      whileInView={{ opacity: 1, y: 0 }}
                      transition={{ duration: 0.8, delay: 0.2 }}
                      viewport={{ once: true }}
                    >
                      Showcasing my work in robotics, AI, and software engineering.
                    </motion.p>
                    
                    {/* Project Categories Filter */}
                    <motion.div
                      initial={{ opacity: 0, y: 20 }}
                      whileInView={{ opacity: 1, y: 0 }}
                      transition={{ duration: 0.8, delay: 0.4 }}
                      viewport={{ once: true }}
                      className="flex flex-wrap justify-center gap-2 mt-8"
                    >
                      {['All', 'Robotics', 'AI/ML', 'IoT', 'Research'].map((category, index) => (
                        <motion.button
                          key={category}
                          className={`px-4 py-2 rounded-full text-sm font-medium transition-all duration-300 ${
                            index === 0 
                              ? 'bg-primary text-background shadow-lg shadow-primary/25' 
                              : 'bg-background-secondary text-foreground-secondary hover:bg-primary/10 hover:text-primary border border-border'
                          }`}
                          whileHover={{ scale: 1.05, y: -2 }}
                          whileTap={{ scale: 0.95 }}
                          initial={{ opacity: 0, y: 20 }}
                          whileInView={{ opacity: 1, y: 0 }}
                          transition={{ duration: 0.5, delay: 0.6 + index * 0.1 }}
                          viewport={{ once: true }}
                        >
                          {category}
                        </motion.button>
                      ))}
                    </motion.div>
                  </motion.div>

                            <div className="grid sm:grid-cols-1 lg:grid-cols-2 gap-6 sm:gap-8">
                          {/* Project 1 */}
              <motion.div
                initial={{ opacity: 0, y: 50, scale: 0.9 }}
                whileInView={{ opacity: 1, y: 0, scale: 1 }}
                transition={{ duration: 0.8, ease: "easeOut" }}
                viewport={{ once: true }}
                className="bg-background-secondary rounded-lg border border-border p-6 relative overflow-hidden group cursor-pointer"
                whileHover={{ 
                  y: -15,
                  scale: 1.03,
                  transition: { duration: 0.4, ease: "easeOut" }
                }}
              >
                {/* Enhanced hover overlay */}
                <motion.div
                  className="absolute inset-0 bg-gradient-to-r from-primary/10 to-accent/10 opacity-0 group-hover:opacity-100 transition-opacity duration-500"
                  initial={{ opacity: 0 }}
                  whileHover={{ opacity: 1 }}
                />
                {/* Glowing border effect */}
                <motion.div
                  className="absolute inset-0 rounded-lg opacity-0 group-hover:opacity-100"
                  style={{
                    background: 'linear-gradient(45deg, #00D4FF, #00FF88, #00D4FF)',
                    backgroundSize: '200% 200%',
                  }}
                  animate={{
                    backgroundPosition: ['0% 0%', '100% 100%', '0% 0%'],
                  }}
                  transition={{
                    backgroundPosition: { duration: 3, repeat: Infinity, ease: "linear" },
                    opacity: { duration: 0.3 }
                  }}
                />
                <div className="absolute inset-[1px] bg-background-secondary rounded-lg" />
                
                {/* Project Header */}
                <div className="relative z-10">
                  {/* Project Icon with Advanced Animation */}
                  <motion.div 
                    className="w-full h-24 sm:h-32 bg-gradient-to-br from-primary/20 to-accent/20 rounded-lg mb-4 flex items-center justify-center overflow-hidden relative"
                    whileHover={{ 
                      scale: 1.05,
                      transition: { duration: 0.3 }
                    }}
                  >
                    {/* Floating particles around icon */}
                    {[...Array(3)].map((_, i) => (
                      <motion.div
                        key={i}
                        className="absolute w-1 h-1 bg-primary/60 rounded-full"
                        style={{
                          left: `${30 + i * 20}%`,
                          top: `${40 + i * 10}%`,
                        }}
                        animate={{
                          y: [0, -10, 0],
                          opacity: [0.3, 1, 0.3],
                          scale: [1, 1.5, 1],
                        }}
                        transition={{
                          duration: 2 + i * 0.5,
                          repeat: Infinity,
                          ease: "easeInOut",
                          delay: i * 0.3,
                        }}
                      />
                    ))}
                    
                    <motion.span 
                      className="text-3xl sm:text-4xl relative z-10"
                      animate={{ 
                        rotate: [0, 5, -5, 0],
                        scale: [1, 1.1, 1]
                      }}
                      transition={{
                        duration: 4,
                        repeat: Infinity,
                        ease: "easeInOut"
                      }}
                    >
                      🤖
                    </motion.span>
                  </motion.div>
                  
                  {/* Project Title with Category Badge */}
                  <div className="flex items-start justify-between mb-3">
                    <h3 className="text-lg sm:text-xl font-bold flex-1 mr-3">Assistive Robot for Children with Upper-Limb Disabilities</h3>
                    <motion.span 
                      className="px-2 py-1 bg-primary/20 text-primary text-xs rounded-full font-medium flex-shrink-0"
                      initial={{ opacity: 0, scale: 0.8 }}
                      whileInView={{ opacity: 1, scale: 1 }}
                      transition={{ duration: 0.5, delay: 0.3 }}
                      viewport={{ once: true }}
                    >
                      Robotics
                    </motion.span>
                  </div>
                  
                  {/* Project Description */}
                  <p className="text-foreground-secondary text-xs sm:text-sm mb-4 leading-relaxed">
                    Research prototype improving manipulation tasks for children with disabilities. Incorporated user feedback loops with measurable gains in usability.
                  </p>
                  
                  {/* Tech Stack with Hover Effects */}
                  <div className="flex flex-wrap gap-1 sm:gap-2 mb-4">
                    {['ROS2', 'C++/Python', 'Embedded Control', 'Perception'].map((tech, index) => (
                      <motion.span
                        key={tech}
                        className="px-2 py-1 bg-background rounded text-xs border border-border hover:border-primary/50 transition-colors cursor-default"
                        initial={{ opacity: 0, y: 10 }}
                        whileInView={{ opacity: 1, y: 0 }}
                        transition={{ duration: 0.3, delay: 0.1 * index }}
                        viewport={{ once: true }}
                        whileHover={{ 
                          scale: 1.05,
                          backgroundColor: "rgba(0, 212, 255, 0.1)",
                          transition: { duration: 0.2 }
                        }}
                      >
                        {tech}
                      </motion.span>
                    ))}
                  </div>
                  
                  {/* Project Footer */}
                  <div className="flex justify-between items-center">
                    <motion.div
                      className="flex items-center space-x-2"
                      initial={{ opacity: 0, x: -20 }}
                      whileInView={{ opacity: 1, x: 0 }}
                      transition={{ duration: 0.5, delay: 0.4 }}
                      viewport={{ once: true }}
                    >
                      <span className="text-xs text-foreground-secondary">IEEE T-RO (under review)</span>
                      <motion.div
                        className="w-2 h-2 bg-accent rounded-full"
                        animate={{ 
                          scale: [1, 1.2, 1],
                          opacity: [0.7, 1, 0.7]
                        }}
                        transition={{
                          duration: 2,
                          repeat: Infinity,
                          ease: "easeInOut"
                        }}
                      />
                    </motion.div>
                    <motion.a 
                      href="https://github.com/kwame-minta2021" 
                      target="_blank" 
                      rel="noopener noreferrer" 
                      className="text-primary text-sm hover:underline flex items-center space-x-1 group"
                      whileHover={{ x: 5 }}
                      transition={{ duration: 0.2 }}
                    >
                      <span>View Project</span>
                      <motion.span
                        className="inline-block"
                        animate={{ x: [0, 3, 0] }}
                        transition={{
                          duration: 1.5,
                          repeat: Infinity,
                          ease: "easeInOut"
                        }}
                      >
                        →
                      </motion.span>
                    </motion.a>
                  </div>
                </div>
              </motion.div>

                          {/* Project 2 */}
              <motion.div
                initial={{ opacity: 0, y: 50, scale: 0.9 }}
                whileInView={{ opacity: 1, y: 0, scale: 1 }}
                transition={{ duration: 0.8, delay: 0.1, ease: "easeOut" }}
                viewport={{ once: true }}
                className="bg-background-secondary rounded-lg border border-border p-6 relative overflow-hidden group cursor-pointer"
                whileHover={{ 
                  y: -15,
                  scale: 1.03,
                  transition: { duration: 0.4, ease: "easeOut" }
                }}
              >
                {/* Enhanced hover overlay */}
                <motion.div
                  className="absolute inset-0 bg-gradient-to-r from-accent/10 to-primary/10 opacity-0 group-hover:opacity-100 transition-opacity duration-500"
                  initial={{ opacity: 0 }}
                  whileHover={{ opacity: 1 }}
                />
                {/* Glowing border effect */}
                <motion.div
                  className="absolute inset-0 rounded-lg opacity-0 group-hover:opacity-100"
                  style={{
                    background: 'linear-gradient(45deg, #00FF88, #00D4FF, #00FF88)',
                    backgroundSize: '200% 200%',
                  }}
                  animate={{
                    backgroundPosition: ['0% 0%', '100% 100%', '0% 0%'],
                  }}
                  transition={{
                    backgroundPosition: { duration: 3, repeat: Infinity, ease: "linear" },
                    opacity: { duration: 0.3 }
                  }}
                />
                <div className="absolute inset-[1px] bg-background-secondary rounded-lg" />
                
                {/* Project Header */}
                <div className="relative z-10">
                  {/* Project Icon with Advanced Animation */}
                  <motion.div 
                    className="w-full h-24 sm:h-32 bg-gradient-to-br from-accent/20 to-primary/20 rounded-lg mb-4 flex items-center justify-center overflow-hidden relative"
                    whileHover={{ 
                      scale: 1.05,
                      transition: { duration: 0.3 }
                    }}
                  >
                    {/* Floating particles around icon */}
                    {[...Array(3)].map((_, i) => (
                      <motion.div
                        key={i}
                        className="absolute w-1 h-1 bg-accent/60 rounded-full"
                        style={{
                          left: `${40 + i * 15}%`,
                          top: `${30 + i * 15}%`,
                        }}
                        animate={{
                          y: [0, -15, 0],
                          opacity: [0.3, 1, 0.3],
                          scale: [1, 1.8, 1],
                        }}
                        transition={{
                          duration: 2.5 + i * 0.3,
                          repeat: Infinity,
                          ease: "easeInOut",
                          delay: i * 0.4,
                        }}
                      />
                    ))}
                    
                    <motion.span 
                      className="text-3xl sm:text-4xl relative z-10"
                      animate={{ 
                        rotate: [0, -5, 5, 0],
                        scale: [1, 1.15, 1]
                      }}
                      transition={{
                        duration: 5,
                        repeat: Infinity,
                        ease: "easeInOut"
                      }}
                    >
                      🌬️
                    </motion.span>
                  </motion.div>
                  
                  {/* Project Title with Category Badge */}
                  <div className="flex items-start justify-between mb-3">
                    <h3 className="text-lg sm:text-xl font-bold flex-1 mr-3">Air-Quality IoT with Deep Q-Learning Alerts</h3>
                    <motion.span 
                      className="px-2 py-1 bg-accent/20 text-accent text-xs rounded-full font-medium flex-shrink-0"
                      initial={{ opacity: 0, scale: 0.8 }}
                      whileInView={{ opacity: 1, scale: 1 }}
                      transition={{ duration: 0.5, delay: 0.3 }}
                      viewport={{ once: true }}
                    >
                      AI/ML
                    </motion.span>
                  </div>
                  
                  {/* Project Description */}
                  <p className="text-foreground-secondary text-xs sm:text-sm mb-4 leading-relaxed">
                    Scalable IoT-cloud pipeline with DQN-based thresholding and SMS alerting for urban environments. Fielded prototype with published results.
                  </p>
                  
                  {/* Tech Stack with Hover Effects */}
                  <div className="flex flex-wrap gap-1 sm:gap-2 mb-4">
                    {['ESP32', 'Python', 'PyTorch (DQN)', 'Cloud RTDB'].map((tech, index) => (
                      <motion.span
                        key={tech}
                        className="px-2 py-1 bg-background rounded text-xs border border-border hover:border-accent/50 transition-colors cursor-default"
                        initial={{ opacity: 0, y: 10 }}
                        whileInView={{ opacity: 1, y: 0 }}
                        transition={{ duration: 0.3, delay: 0.1 * index }}
                        viewport={{ once: true }}
                        whileHover={{ 
                          scale: 1.05,
                          backgroundColor: "rgba(0, 255, 136, 0.1)",
                          transition: { duration: 0.2 }
                        }}
                      >
                        {tech}
                      </motion.span>
                    ))}
                  </div>
                  
                  {/* Project Footer */}
                  <div className="flex justify-between items-center">
                    <motion.div
                      className="flex items-center space-x-2"
                      initial={{ opacity: 0, x: -20 }}
                      whileInView={{ opacity: 1, x: 0 }}
                      transition={{ duration: 0.5, delay: 0.4 }}
                      viewport={{ once: true }}
                    >
                      <span className="text-xs text-foreground-secondary">IJCA 2025 • IEEE ICECCME'25</span>
                      <motion.div
                        className="w-2 h-2 bg-primary rounded-full"
                        animate={{ 
                          scale: [1, 1.2, 1],
                          opacity: [0.7, 1, 0.7]
                        }}
                        transition={{
                          duration: 2,
                          repeat: Infinity,
                          ease: "easeInOut"
                        }}
                      />
                    </motion.div>
                    <motion.a 
                      href="https://github.com/kwame-minta2021" 
                      target="_blank" 
                      rel="noopener noreferrer" 
                      className="text-primary text-sm hover:underline flex items-center space-x-1 group"
                      whileHover={{ x: 5 }}
                      transition={{ duration: 0.2 }}
                    >
                      <span>View Project</span>
                      <motion.span
                        className="inline-block"
                        animate={{ x: [0, 3, 0] }}
                        transition={{
                          duration: 1.5,
                          repeat: Infinity,
                          ease: "easeInOut"
                        }}
                      >
                        →
                      </motion.span>
                    </motion.a>
                  </div>
                </div>
              </motion.div>

            {/* Project 3 */}
            <motion.div
              initial={{ opacity: 0, y: 20 }}
              whileInView={{ opacity: 1, y: 0 }}
              transition={{ duration: 0.8, delay: 0.2 }}
              viewport={{ once: true }}
              className="bg-background-secondary rounded-lg border border-border p-6 hover:border-primary/50 transition-all duration-300"
            >
              <div className="w-full h-32 bg-gradient-to-br from-primary/20 to-accent/20 rounded-lg mb-4 flex items-center justify-center">
                <span className="text-2xl">🌾</span>
              </div>
              <h3 className="text-xl font-bold mb-2">Precision Agriculture Sensing for Rural Ghana</h3>
              <p className="text-foreground-secondary text-sm mb-4">
                IoT prototypes for crop monitoring with architectures tailored for low-infrastructure settings. Improved monitoring accuracy for national programs.
              </p>
              <div className="flex flex-wrap gap-2 mb-4">
                <span className="px-2 py-1 bg-background rounded text-xs">ESP32</span>
                <span className="px-2 py-1 bg-background rounded text-xs">Sensor Fusion</span>
                <span className="px-2 py-1 bg-background rounded text-xs">Cloud Dashboards</span>
                <span className="px-2 py-1 bg-background rounded text-xs">Python</span>
              </div>
              <div className="flex justify-between items-center">
                <span className="text-xs text-foreground-secondary">IJCA 2024</span>
                <a href="https://github.com/kwame-minta2021" target="_blank" rel="noopener noreferrer" className="text-primary text-sm hover:underline">View Project →</a>
              </div>
            </motion.div>

            {/* Project 4 */}
            <motion.div
              initial={{ opacity: 0, y: 20 }}
              whileInView={{ opacity: 1, y: 0 }}
              transition={{ duration: 0.8, delay: 0.3 }}
              viewport={{ once: true }}
              className="bg-background-secondary rounded-lg border border-border p-6 hover:border-primary/50 transition-all duration-300"
            >
              <div className="w-full h-32 bg-gradient-to-br from-primary/20 to-accent/20 rounded-lg mb-4 flex items-center justify-center">
                <span className="text-2xl">⚡</span>
              </div>
              <h3 className="text-xl font-bold mb-2">Electronic Shot Exploder for Safer Blasting</h3>
              <p className="text-foreground-secondary text-sm mb-4">
                Designed and validated an electronic shot exploder PCB to enhance safety and efficiency in mining operations.
              </p>
              <div className="flex flex-wrap gap-2 mb-4">
                <span className="px-2 py-1 bg-background rounded text-xs">Embedded C</span>
                <span className="px-2 py-1 bg-background rounded text-xs">PCB Design</span>
                <span className="px-2 py-1 bg-background rounded text-xs">Proteus/AutoCAD</span>
                <span className="px-2 py-1 bg-background rounded text-xs">Hardware Validation</span>
              </div>
              <div className="flex justify-between items-center">
                <span className="text-xs text-foreground-secondary">Patent Draft (Under Review)</span>
                <a href="https://github.com/kwame-minta2021" target="_blank" rel="noopener noreferrer" className="text-primary text-sm hover:underline">View Project →</a>
              </div>
                        </motion.div>
          </div>
          
          {/* Project Statistics */}
          <motion.div
            initial={{ opacity: 0, y: 30 }}
            whileInView={{ opacity: 1, y: 0 }}
            transition={{ duration: 0.8, delay: 0.6 }}
            viewport={{ once: true }}
            className="mt-16 grid grid-cols-2 md:grid-cols-4 gap-6"
          >
            {[
              { number: "4", label: "Featured Projects", icon: "🚀" },
              { number: "6", label: "Research Papers", icon: "📄" },
              { number: "5", label: "Years Experience", icon: "⏱️" },
              { number: "100%", label: "Code Quality", icon: "✨" }
            ].map((stat, index) => (
              <motion.div
                key={stat.label}
                className="text-center p-6 bg-background-secondary rounded-lg border border-border hover:border-primary/30 transition-colors"
                initial={{ opacity: 0, y: 20 }}
                whileInView={{ opacity: 1, y: 0 }}
                transition={{ duration: 0.5, delay: 0.8 + index * 0.1 }}
                viewport={{ once: true }}
                whileHover={{ 
                  y: -5,
                  scale: 1.02,
                  transition: { duration: 0.3 }
                }}
              >
                <motion.div
                  className="text-3xl mb-2"
                  animate={{ 
                    rotate: [0, 5, -5, 0],
                    scale: [1, 1.1, 1]
                  }}
                  transition={{
                    duration: 4,
                    repeat: Infinity,
                    ease: "easeInOut",
                    delay: index * 0.5
                  }}
                >
                  {stat.icon}
                </motion.div>
                <motion.div
                  className="text-2xl font-bold text-primary mb-1"
                  initial={{ opacity: 0, scale: 0.8 }}
                  whileInView={{ opacity: 1, scale: 1 }}
                  transition={{ duration: 0.5, delay: 1 + index * 0.1 }}
                  viewport={{ once: true }}
                >
                  {stat.number}
                </motion.div>
                <div className="text-xs text-foreground-secondary">
                  {stat.label}
                </div>
              </motion.div>
            ))}
          </motion.div>
        </div>
      </motion.section>

      {/* Research & Publications Section */}
              <motion.section 
                id="research" 
                className="py-24 bg-background-secondary relative overflow-hidden"
                initial={{ opacity: 0 }}
                whileInView={{ opacity: 1 }}
                transition={{ duration: 1 }}
                viewport={{ once: true }}
              >
                {/* Animated background elements */}
                <div className="absolute inset-0 opacity-5">
                  {/* Academic pattern */}
                  <div className="absolute inset-0" style={{
                    backgroundImage: `linear-gradient(45deg, transparent 40%, #00D4FF 40%, #00D4FF 60%, transparent 60%), linear-gradient(-45deg, transparent 40%, #00FF88 40%, #00FF88 60%, transparent 60%)`,
                    backgroundSize: '100px 100px'
                  }} />
                  
                  {/* Floating document icons */}
                  {[...Array(6)].map((_, i) => (
                    <motion.div
                      key={i}
                      className="absolute text-3xl opacity-10"
                      style={{
                        left: `${20 + i * 12}%`,
                        top: `${30 + i * 10}%`,
                      }}
                      animate={{
                        y: [0, -20, 0],
                        rotate: [0, 10, -10, 0],
                        opacity: [0.05, 0.2, 0.05],
                      }}
                      transition={{
                        duration: 6 + i * 0.4,
                        repeat: Infinity,
                        ease: "easeInOut",
                        delay: i * 0.5,
                      }}
                    >
                      📄
                    </motion.div>
                  ))}
                </div>
        <div className="max-w-7xl mx-auto px-4 sm:px-6 lg:px-8">
          <motion.div
            initial={{ opacity: 0, y: 20 }}
            whileInView={{ opacity: 1, y: 0 }}
            transition={{ duration: 0.8 }}
            viewport={{ once: true }}
            className="text-center mb-16"
          >
            <h2 className="text-3xl md:text-4xl font-bold mb-4">
              Research & Publications
            </h2>
            <p className="text-foreground-secondary text-lg max-w-2xl mx-auto">
              Published work in robotics, IoT, and AI applications.
            </p>
          </motion.div>

          <div className="space-y-6">
            {/* Publication 1 */}
            <motion.div
              initial={{ opacity: 0, y: 20 }}
              whileInView={{ opacity: 1, y: 0 }}
              transition={{ duration: 0.8 }}
              viewport={{ once: true }}
              className="bg-background rounded-lg border border-border p-6"
            >
              <h3 className="text-lg font-bold mb-2">A Scalable IoT-Cloud Architecture with Deep Q-Learning for Air Quality Monitoring and Alerting</h3>
              <p className="text-foreground-secondary text-sm mb-3">Aidoo, B., <strong>Minta, F.K.</strong>, N-yo, A.A., Tettey, D.A.</p>
              <p className="text-foreground-secondary text-sm mb-4">International Journal of Computer Applications, 2025</p>
              <div className="flex gap-2">
                <button className="text-primary text-sm hover:underline">View Paper</button>
                <button className="text-primary text-sm hover:underline">Copy Citation</button>
              </div>
            </motion.div>

            {/* Publication 2 */}
            <motion.div
              initial={{ opacity: 0, y: 20 }}
              whileInView={{ opacity: 1, y: 0 }}
              transition={{ duration: 0.8, delay: 0.1 }}
              viewport={{ once: true }}
              className="bg-background rounded-lg border border-border p-6"
            >
              <h3 className="text-lg font-bold mb-2">Assistive Robot for Children with Disability (Amputated Arms)</h3>
              <p className="text-foreground-secondary text-sm mb-3"><strong>Minta, F.K.</strong>, Sitti, M., Agangiba, W., Essilfie, A.</p>
              <p className="text-foreground-secondary text-sm mb-4">IEEE Transactions on Robotics, under review</p>
              <div className="flex gap-2">
                <button className="text-primary text-sm hover:underline">View Paper</button>
                <button className="text-primary text-sm hover:underline">Copy Citation</button>
              </div>
            </motion.div>

            {/* Publication 3 */}
            <motion.div
              initial={{ opacity: 0, y: 20 }}
              whileInView={{ opacity: 1, y: 0 }}
              transition={{ duration: 0.8, delay: 0.2 }}
              viewport={{ once: true }}
              className="bg-background rounded-lg border border-border p-6"
            >
              <h3 className="text-lg font-bold mb-2">IoT-based Weather Monitoring System for Ghanaian Farmers</h3>
              <p className="text-foreground-secondary text-sm mb-3">Okai, G.E.Y., <strong>Minta, F.K.</strong>, Osman, A.M., Essilfie, A.</p>
              <p className="text-foreground-secondary text-sm mb-4">International Journal of Computer Applications, 2024</p>
              <div className="flex gap-2">
                <button className="text-primary text-sm hover:underline">View Paper</button>
                <button className="text-primary text-sm hover:underline">Copy Citation</button>
              </div>
            </motion.div>
                            </div>
                </div>
              </motion.section>

                    {/* Experience Section */}
              <motion.section 
                id="experience" 
                className="py-24 bg-background relative overflow-hidden"
                initial={{ opacity: 0 }}
                whileInView={{ opacity: 1 }}
                transition={{ duration: 1 }}
                viewport={{ once: true }}
              >
                {/* Animated background elements */}
                <div className="absolute inset-0 opacity-5">
                  {/* Timeline pattern */}
                  <div className="absolute inset-0" style={{
                    backgroundImage: `linear-gradient(90deg, transparent 49%, #00D4FF 49%, #00D4FF 51%, transparent 51%)`,
                    backgroundSize: '100px 100px'
                  }} />
                  
                  {/* Floating experience icons */}
                  {[...Array(5)].map((_, i) => (
                    <motion.div
                      key={i}
                      className="absolute text-2xl opacity-10"
                      style={{
                        left: `${25 + i * 15}%`,
                        top: `${40 + i * 12}%`,
                      }}
                      animate={{
                        y: [0, -25, 0],
                        x: [0, 10, 0],
                        opacity: [0.05, 0.18, 0.05],
                      }}
                      transition={{
                        duration: 7 + i * 0.6,
                        repeat: Infinity,
                        ease: "easeInOut",
                        delay: i * 0.4,
                      }}
                    >
                      {['💼', '🎓', '🔬', '📊', '🌟'][i]}
                    </motion.div>
                  ))}
                </div>
        <div className="max-w-7xl mx-auto px-4 sm:px-6 lg:px-8">
          <motion.div
            initial={{ opacity: 0, y: 20 }}
            whileInView={{ opacity: 1, y: 0 }}
            transition={{ duration: 0.8 }}
            viewport={{ once: true }}
            className="text-center mb-16"
          >
            <h2 className="text-3xl md:text-4xl font-bold mb-4">
              Experience
            </h2>
            <p className="text-foreground-secondary text-lg max-w-2xl mx-auto">
              Professional experience in robotics, IoT, and software engineering.
            </p>
          </motion.div>

          <div className="space-y-8">
            {/* Experience 1 */}
            <motion.div
              initial={{ opacity: 0, x: -50 }}
              whileInView={{ opacity: 1, x: 0 }}
              transition={{ duration: 0.8 }}
              viewport={{ once: true }}
              className="relative pl-8 pb-8 border-l border-border last:border-l-0 last:pb-0"
            >
              <div className="absolute left-0 top-0 w-3 h-3 bg-primary rounded-full border-2 border-background transform -translate-x-1/2"></div>
              <div className="bg-background-secondary rounded-lg border border-border p-6">
                <h3 className="text-xl font-bold mb-2">IoT Researcher — Farmlynco</h3>
                <p className="text-foreground-secondary text-sm mb-2">Sept 2024–Aug 2025 • Legon, Ghana</p>
                <p className="text-foreground-secondary mb-4">
                  Built a real-time farming platform integrating expert knowledge, marketplace access, and IoT decision tools; designed data-driven irrigation and yield optimization.
                </p>
                <div className="flex flex-wrap gap-2">
                  <span className="px-2 py-1 bg-background rounded text-xs">IoT Platform</span>
                  <span className="px-2 py-1 bg-background rounded text-xs">Data Analytics</span>
                  <span className="px-2 py-1 bg-background rounded text-xs">Yield Optimization</span>
                </div>
              </div>
            </motion.div>

            {/* Experience 2 */}
            <motion.div
              initial={{ opacity: 0, x: -50 }}
              whileInView={{ opacity: 1, x: 0 }}
              transition={{ duration: 0.8, delay: 0.1 }}
              viewport={{ once: true }}
              className="relative pl-8 pb-8 border-l border-border last:border-l-0 last:pb-0"
            >
              <div className="absolute left-0 top-0 w-3 h-3 bg-primary rounded-full border-2 border-background transform -translate-x-1/2"></div>
              <div className="bg-background-secondary rounded-lg border border-border p-6">
                <h3 className="text-xl font-bold mb-2">Application Selection Team — Ghana Data Science Summit (IndabaX)</h3>
                <p className="text-foreground-secondary text-sm mb-2">Jul–Aug 2024 • KNUST</p>
                <p className="text-foreground-secondary mb-4">
                  Reviewed 4,000+ applications; selected top 10% with a 15-member team, ensuring fairness and transparency.
                </p>
                <div className="flex flex-wrap gap-2">
                  <span className="px-2 py-1 bg-background rounded text-xs">Application Review</span>
                  <span className="px-2 py-1 bg-background rounded text-xs">Team Leadership</span>
                  <span className="px-2 py-1 bg-background rounded text-xs">Data Science</span>
                </div>
              </div>
            </motion.div>

            {/* Experience 3 */}
            <motion.div
              initial={{ opacity: 0, x: -50 }}
              whileInView={{ opacity: 1, x: 0 }}
              transition={{ duration: 0.8, delay: 0.2 }}
              viewport={{ once: true }}
              className="relative pl-8 pb-8 border-l border-border last:border-l-0 last:pb-0"
            >
              <div className="absolute left-0 top-0 w-3 h-3 bg-primary rounded-full border-2 border-background transform -translate-x-1/2"></div>
              <div className="bg-background-secondary rounded-lg border border-border p-6">
                <h3 className="text-xl font-bold mb-2">Robotics Instructor — Aaenics Engineering Ltd.</h3>
                <p className="text-foreground-secondary text-sm mb-2">Jun 2023–Sept 2024 • Accra</p>
                <p className="text-foreground-secondary mb-4">
                  Instructed 300+ students; designed project-based curricula improving proficiency by ~40%.
                </p>
                <div className="flex flex-wrap gap-2">
                  <span className="px-2 py-1 bg-background rounded text-xs">Teaching</span>
                  <span className="px-2 py-1 bg-background rounded text-xs">Curriculum Design</span>
                  <span className="px-2 py-1 bg-background rounded text-xs">Project-Based Learning</span>
                </div>
              </div>
            </motion.div>
          </div>
        </div>
      </motion.section>

      {/* Contact Section */}
      <motion.section 
        id="contact" 
        className="py-24 bg-background-secondary relative overflow-hidden"
        initial={{ opacity: 0 }}
        whileInView={{ opacity: 1 }}
        transition={{ duration: 1 }}
        viewport={{ once: true }}
      >
        {/* Animated background elements */}
        <div className="absolute inset-0 opacity-5">
          {/* Connection pattern */}
          <div className="absolute inset-0" style={{
            backgroundImage: `radial-gradient(circle at 25% 25%, #00D4FF 1px, transparent 1px), radial-gradient(circle at 75% 75%, #00FF88 1px, transparent 1px)`,
            backgroundSize: '80px 80px'
          }} />
          
          {/* Floating contact icons */}
          {[...Array(4)].map((_, i) => (
            <motion.div
              key={i}
              className="absolute text-2xl opacity-10"
              style={{
                left: `${30 + i * 15}%`,
                top: `${25 + i * 18}%`,
              }}
              animate={{
                y: [0, -30, 0],
                rotate: [0, 15, -15, 0],
                opacity: [0.05, 0.2, 0.05],
              }}
              transition={{
                duration: 8 + i * 0.7,
                repeat: Infinity,
                ease: "easeInOut",
                delay: i * 0.6,
              }}
            >
              {['📧', '📱', '💼', '🌐'][i]}
            </motion.div>
          ))}
        </div>
        <div className="max-w-7xl mx-auto px-4 sm:px-6 lg:px-8">
          <motion.div
            initial={{ opacity: 0, y: 20 }}
            whileInView={{ opacity: 1, y: 0 }}
            transition={{ duration: 0.8 }}
            viewport={{ once: true }}
            className="text-center mb-16"
          >
            <h2 className="text-3xl md:text-4xl font-bold mb-4">
              Get In Touch
            </h2>
            <p className="text-foreground-secondary text-lg max-w-2xl mx-auto">
              For collaborations, research, and roles in robotics software engineering.
            </p>
          </motion.div>

          <div className="grid sm:grid-cols-1 lg:grid-cols-2 gap-8 lg:gap-12">
            <motion.div
              initial={{ opacity: 0, x: -50 }}
              whileInView={{ opacity: 1, x: 0 }}
              transition={{ duration: 0.8 }}
              viewport={{ once: true }}
            >
              <h3 className="text-xl sm:text-2xl font-bold mb-6">Contact Information</h3>
                              <div className="space-y-4">
                  {/* Email */}
                  <motion.a
                    href="mailto:frederickminta@gmail.com"
                    className="relative flex items-center space-x-4 p-4 rounded-xl border border-border hover:border-primary/50 hover:bg-background/50 backdrop-blur-sm transition-all duration-500 group overflow-hidden"
                    initial={{ opacity: 0, x: -50 }}
                    whileInView={{ opacity: 1, x: 0 }}
                    transition={{ duration: 0.6, delay: 0.1 }}
                    viewport={{ once: true }}
                    whileHover={{ 
                      x: 10, 
                      scale: 1.03,
                      boxShadow: "0 20px 40px rgba(0, 212, 255, 0.1)"
                    }}
                    whileTap={{ scale: 0.98 }}
                  >
                    {/* Animated background gradient */}
                    <motion.div
                      className="absolute inset-0 bg-gradient-to-r from-red-500/5 to-pink-500/5 opacity-0 group-hover:opacity-100 transition-opacity duration-500"
                      animate={{
                        backgroundPosition: ['0% 0%', '100% 100%', '0% 0%'],
                      }}
                      transition={{
                        backgroundPosition: { duration: 3, repeat: Infinity, ease: "linear" },
                      }}
                    />
                    
                    {/* Icon container with enhanced animations */}
                    <motion.div 
                      className="relative w-12 h-12 bg-gradient-to-r from-red-500 to-pink-500 rounded-xl flex items-center justify-center group-hover:shadow-xl transition-all duration-300"
                      whileHover={{ 
                        rotate: [0, -5, 5, 0],
                        scale: 1.1
                      }}
                      animate={{
                        boxShadow: [
                          "0 0 0 0 rgba(239, 68, 68, 0.4)",
                          "0 0 0 10px rgba(239, 68, 68, 0)",
                          "0 0 0 0 rgba(239, 68, 68, 0)"
                        ]
                      }}
                      transition={{
                        boxShadow: { duration: 2, repeat: Infinity, ease: "easeInOut" },
                        rotate: { duration: 0.6 }
                      }}
                    >
                      <motion.svg 
                        className="w-6 h-6 text-white" 
                        fill="currentColor" 
                        viewBox="0 0 20 20"
                        animate={{ 
                          y: [0, -2, 0],
                          scale: [1, 1.05, 1]
                        }}
                        transition={{
                          duration: 2,
                          repeat: Infinity,
                          ease: "easeInOut"
                        }}
                      >
                        <path d="M2.003 5.884L10 9.882l7.997-3.998A2 2 0 0016 4H4a2 2 0 00-1.997 1.884z" />
                        <path d="M18 8.118l-8 4-8-4V14a2 2 0 002 2h12a2 2 0 002-2V8.118z" />
                      </motion.svg>
                    </motion.div>
                    
                    {/* Content with enhanced typography */}
                    <div className="flex-1">
                      <motion.div 
                        className="text-sm text-foreground-secondary font-medium"
                        initial={{ opacity: 0, y: 10 }}
                        whileInView={{ opacity: 1, y: 0 }}
                        transition={{ duration: 0.5, delay: 0.2 }}
                        viewport={{ once: true }}
                      >
                        Email
                      </motion.div>
                      <motion.div 
                        className="text-sm font-semibold text-foreground group-hover:text-primary transition-colors duration-300"
                        initial={{ opacity: 0, y: 10 }}
                        whileInView={{ opacity: 1, y: 0 }}
                        transition={{ duration: 0.5, delay: 0.3 }}
                        viewport={{ once: true }}
                      >
                        frederickminta@gmail.com
                      </motion.div>
                    </div>
                    
                    {/* Animated arrow */}
                    <motion.div
                      className="text-primary opacity-0 group-hover:opacity-100 transition-opacity duration-300"
                      animate={{ x: [0, 5, 0] }}
                      transition={{
                        duration: 1.5,
                        repeat: Infinity,
                        ease: "easeInOut"
                      }}
                    >
                      →
                    </motion.div>
                  </motion.a>

                  {/* Phone */}
                  <motion.a
                    href="tel:+233543845970"
                    className="relative flex items-center space-x-4 p-4 rounded-xl border border-border hover:border-accent/50 hover:bg-background/50 backdrop-blur-sm transition-all duration-500 group overflow-hidden"
                    initial={{ opacity: 0, x: -50 }}
                    whileInView={{ opacity: 1, x: 0 }}
                    transition={{ duration: 0.6, delay: 0.2 }}
                    viewport={{ once: true }}
                    whileHover={{ 
                      x: 10, 
                      scale: 1.03,
                      boxShadow: "0 20px 40px rgba(0, 255, 136, 0.1)"
                    }}
                    whileTap={{ scale: 0.98 }}
                  >
                    {/* Animated background gradient */}
                    <motion.div
                      className="absolute inset-0 bg-gradient-to-r from-green-500/5 to-teal-500/5 opacity-0 group-hover:opacity-100 transition-opacity duration-500"
                      animate={{
                        backgroundPosition: ['0% 0%', '100% 100%', '0% 0%'],
                      }}
                      transition={{
                        backgroundPosition: { duration: 3, repeat: Infinity, ease: "linear" },
                      }}
                    />
                    
                    {/* Icon container with enhanced animations */}
                    <motion.div 
                      className="relative w-12 h-12 bg-gradient-to-r from-green-500 to-teal-500 rounded-xl flex items-center justify-center group-hover:shadow-xl transition-all duration-300"
                      whileHover={{ 
                        rotate: [0, -5, 5, 0],
                        scale: 1.1
                      }}
                      animate={{
                        boxShadow: [
                          "0 0 0 0 rgba(34, 197, 94, 0.4)",
                          "0 0 0 10px rgba(34, 197, 94, 0)",
                          "0 0 0 0 rgba(34, 197, 94, 0)"
                        ]
                      }}
                      transition={{
                        boxShadow: { duration: 2, repeat: Infinity, ease: "easeInOut" },
                        rotate: { duration: 0.6 }
                      }}
                    >
                      <motion.svg 
                        className="w-6 h-6 text-white" 
                        fill="currentColor" 
                        viewBox="0 0 20 20"
                        animate={{ 
                          y: [0, -2, 0],
                          scale: [1, 1.05, 1]
                        }}
                        transition={{
                          duration: 2,
                          repeat: Infinity,
                          ease: "easeInOut"
                        }}
                      >
                        <path d="M2 3a1 1 0 011-1h2.153a1 1 0 01.986.836l.74 4.435a1 1 0 01-.54 1.06l-1.548.773a11.037 11.037 0 006.105 6.105l.774-1.548a1 1 0 011.059-.54l4.435.74a1 1 0 01.836.986V17a1 1 0 01-1 1h-2C7.82 18 2 12.18 2 5V3z" />
                      </motion.svg>
                    </motion.div>
                    
                    {/* Content with enhanced typography */}
                    <div className="flex-1">
                      <motion.div 
                        className="text-sm text-foreground-secondary font-medium"
                        initial={{ opacity: 0, y: 10 }}
                        whileInView={{ opacity: 1, y: 0 }}
                        transition={{ duration: 0.5, delay: 0.3 }}
                        viewport={{ once: true }}
                      >
                        Phone
                      </motion.div>
                      <motion.div 
                        className="text-sm font-semibold text-foreground group-hover:text-accent transition-colors duration-300"
                        initial={{ opacity: 0, y: 10 }}
                        whileInView={{ opacity: 1, y: 0 }}
                        transition={{ duration: 0.5, delay: 0.4 }}
                        viewport={{ once: true }}
                      >
                        +233-543-845-970
                      </motion.div>
                    </div>
                    
                    {/* Animated arrow */}
                    <motion.div
                      className="text-accent opacity-0 group-hover:opacity-100 transition-opacity duration-300"
                      animate={{ x: [0, 5, 0] }}
                      transition={{
                        duration: 1.5,
                        repeat: Infinity,
                        ease: "easeInOut"
                      }}
                    >
                      →
                    </motion.div>
                  </motion.a>

                  {/* Location */}
                  <motion.div 
                    className="relative flex items-center space-x-4 p-4 rounded-xl border border-border overflow-hidden"
                    initial={{ opacity: 0, x: -50 }}
                    whileInView={{ opacity: 1, x: 0 }}
                    transition={{ duration: 0.6, delay: 0.3 }}
                    viewport={{ once: true }}
                    whileHover={{ 
                      scale: 1.02,
                      boxShadow: "0 10px 30px rgba(59, 130, 246, 0.1)"
                    }}
                  >
                    {/* Animated background gradient */}
                    <motion.div
                      className="absolute inset-0 bg-gradient-to-r from-blue-500/5 to-indigo-500/5"
                      animate={{
                        backgroundPosition: ['0% 0%', '100% 100%', '0% 0%'],
                      }}
                      transition={{
                        backgroundPosition: { duration: 4, repeat: Infinity, ease: "linear" },
                      }}
                    />
                    
                    {/* Icon container with enhanced animations */}
                    <motion.div 
                      className="relative w-12 h-12 bg-gradient-to-r from-blue-500 to-indigo-500 rounded-xl flex items-center justify-center"
                      animate={{
                        boxShadow: [
                          "0 0 0 0 rgba(59, 130, 246, 0.4)",
                          "0 0 0 8px rgba(59, 130, 246, 0)",
                          "0 0 0 0 rgba(59, 130, 246, 0)"
                        ]
                      }}
                      transition={{
                        boxShadow: { duration: 3, repeat: Infinity, ease: "easeInOut" }
                      }}
                    >
                      <motion.svg 
                        className="w-6 h-6 text-white" 
                        fill="currentColor" 
                        viewBox="0 0 20 20"
                        animate={{ 
                          y: [0, -1, 0],
                          scale: [1, 1.02, 1]
                        }}
                        transition={{
                          duration: 3,
                          repeat: Infinity,
                          ease: "easeInOut"
                        }}
                      >
                        <path fillRule="evenodd" d="M5.05 4.05a7 7 0 119.9 9.9L10 18.9l-4.95-4.95a7 7 0 010-9.9zM10 11a2 2 0 100-4 2 2 0 000 4z" clipRule="evenodd" />
                      </motion.svg>
                    </motion.div>
                    
                    {/* Content with enhanced typography */}
                    <div className="flex-1">
                      <motion.div 
                        className="text-sm text-foreground-secondary font-medium"
                        initial={{ opacity: 0, y: 10 }}
                        whileInView={{ opacity: 1, y: 0 }}
                        transition={{ duration: 0.5, delay: 0.4 }}
                        viewport={{ once: true }}
                      >
                        Location
                      </motion.div>
                      <motion.div 
                        className="text-sm font-semibold text-foreground"
                        initial={{ opacity: 0, y: 10 }}
                        whileInView={{ opacity: 1, y: 0 }}
                        transition={{ duration: 0.5, delay: 0.5 }}
                        viewport={{ once: true }}
                      >
                        Accra-Legon, Ghana
                      </motion.div>
                    </div>
                  </motion.div>

                  {/* LinkedIn */}
                  <motion.a
                    href="https://www.linkedin.com/in/minta-frederick-kwame/"
                    target="_blank"
                    rel="noopener noreferrer"
                    className="relative flex items-center space-x-4 p-4 rounded-xl border border-border hover:border-blue-600/50 hover:bg-background/50 backdrop-blur-sm transition-all duration-500 group overflow-hidden"
                    initial={{ opacity: 0, x: -50 }}
                    whileInView={{ opacity: 1, x: 0 }}
                    transition={{ duration: 0.6, delay: 0.4 }}
                    viewport={{ once: true }}
                    whileHover={{ 
                      x: 10, 
                      scale: 1.03,
                      boxShadow: "0 20px 40px rgba(37, 99, 235, 0.1)"
                    }}
                    whileTap={{ scale: 0.98 }}
                  >
                    {/* Animated background gradient */}
                    <motion.div
                      className="absolute inset-0 bg-gradient-to-r from-blue-600/5 to-blue-700/5 opacity-0 group-hover:opacity-100 transition-opacity duration-500"
                      animate={{
                        backgroundPosition: ['0% 0%', '100% 100%', '0% 0%'],
                      }}
                      transition={{
                        backgroundPosition: { duration: 3, repeat: Infinity, ease: "linear" },
                      }}
                    />
                    
                    {/* Icon container with enhanced animations */}
                    <motion.div 
                      className="relative w-12 h-12 bg-gradient-to-r from-blue-600 to-blue-700 rounded-xl flex items-center justify-center group-hover:shadow-xl transition-all duration-300"
                      whileHover={{ 
                        rotate: [0, -5, 5, 0],
                        scale: 1.1
                      }}
                      animate={{
                        boxShadow: [
                          "0 0 0 0 rgba(37, 99, 235, 0.4)",
                          "0 0 0 10px rgba(37, 99, 235, 0)",
                          "0 0 0 0 rgba(37, 99, 235, 0)"
                        ]
                      }}
                      transition={{
                        boxShadow: { duration: 2, repeat: Infinity, ease: "easeInOut" },
                        rotate: { duration: 0.6 }
                      }}
                    >
                      <motion.svg 
                        className="w-6 h-6 text-white" 
                        fill="currentColor" 
                        viewBox="0 0 24 24"
                        animate={{ 
                          y: [0, -2, 0],
                          scale: [1, 1.05, 1]
                        }}
                        transition={{
                          duration: 2,
                          repeat: Infinity,
                          ease: "easeInOut"
                        }}
                      >
                        <path d="M20.447 20.452h-3.554v-5.569c0-1.328-.027-3.037-1.852-3.037-1.853 0-2.136 1.445-2.136 2.939v5.667H9.351V9h3.414v1.561h.046c.477-.9 1.637-1.85 3.37-1.85 3.601 0 4.267 2.37 4.267 5.455v6.286zM5.337 7.433c-1.144 0-2.063-.926-2.063-2.065 0-1.138.92-2.063 2.063-2.063 1.14 0 2.064.925 2.064 2.063 0 1.139-.925 2.065-2.064 2.065zm1.782 13.019H3.555V9h3.564v11.452zM22.225 0H1.771C.792 0 0 .774 0 1.729v20.542C0 23.227.792 24 1.771 24h20.451C23.2 24 24 23.227 24 22.271V1.729C24 .774 23.2 0 22.222 0h.003z"/>
                      </motion.svg>
                    </motion.div>
                    
                    {/* Content with enhanced typography */}
                    <div className="flex-1">
                      <motion.div 
                        className="text-sm text-foreground-secondary font-medium"
                        initial={{ opacity: 0, y: 10 }}
                        whileInView={{ opacity: 1, y: 0 }}
                        transition={{ duration: 0.5, delay: 0.5 }}
                        viewport={{ once: true }}
                      >
                        LinkedIn
                      </motion.div>
                      <motion.div 
                        className="text-sm font-semibold text-foreground group-hover:text-blue-600 transition-colors duration-300"
                        initial={{ opacity: 0, y: 10 }}
                        whileInView={{ opacity: 1, y: 0 }}
                        transition={{ duration: 0.5, delay: 0.6 }}
                        viewport={{ once: true }}
                      >
                        minta-frederick-kwame
                      </motion.div>
                    </div>
                    
                    {/* Animated arrow */}
                    <motion.div
                      className="text-blue-600 opacity-0 group-hover:opacity-100 transition-opacity duration-300"
                      animate={{ x: [0, 5, 0] }}
                      transition={{
                        duration: 1.5,
                        repeat: Infinity,
                        ease: "easeInOut"
                      }}
                    >
                      →
                    </motion.div>
                  </motion.a>

                  {/* GitHub */}
                  <motion.a
                    href="https://github.com/kwame-minta2021"
                    target="_blank"
                    rel="noopener noreferrer"
                    className="relative flex items-center space-x-4 p-4 rounded-xl border border-border hover:border-gray-800/50 hover:bg-background/50 backdrop-blur-sm transition-all duration-500 group overflow-hidden"
                    initial={{ opacity: 0, x: -50 }}
                    whileInView={{ opacity: 1, x: 0 }}
                    transition={{ duration: 0.6, delay: 0.5 }}
                    viewport={{ once: true }}
                    whileHover={{ 
                      x: 10, 
                      scale: 1.03,
                      boxShadow: "0 20px 40px rgba(31, 41, 55, 0.1)"
                    }}
                    whileTap={{ scale: 0.98 }}
                  >
                    {/* Animated background gradient */}
                    <motion.div
                      className="absolute inset-0 bg-gradient-to-r from-gray-800/5 to-gray-900/5 opacity-0 group-hover:opacity-100 transition-opacity duration-500"
                      animate={{
                        backgroundPosition: ['0% 0%', '100% 100%', '0% 0%'],
                      }}
                      transition={{
                        backgroundPosition: { duration: 3, repeat: Infinity, ease: "linear" },
                      }}
                    />
                    
                    {/* Icon container with enhanced animations */}
                    <motion.div 
                      className="relative w-12 h-12 bg-gradient-to-r from-gray-800 to-gray-900 rounded-xl flex items-center justify-center group-hover:shadow-xl transition-all duration-300"
                      whileHover={{ 
                        rotate: [0, -5, 5, 0],
                        scale: 1.1
                      }}
                      animate={{
                        boxShadow: [
                          "0 0 0 0 rgba(31, 41, 55, 0.4)",
                          "0 0 0 10px rgba(31, 41, 55, 0)",
                          "0 0 0 0 rgba(31, 41, 55, 0)"
                        ]
                      }}
                      transition={{
                        boxShadow: { duration: 2, repeat: Infinity, ease: "easeInOut" },
                        rotate: { duration: 0.6 }
                      }}
                    >
                      <motion.svg 
                        className="w-6 h-6 text-white" 
                        fill="currentColor" 
                        viewBox="0 0 24 24"
                        animate={{ 
                          y: [0, -2, 0],
                          scale: [1, 1.05, 1]
                        }}
                        transition={{
                          duration: 2,
                          repeat: Infinity,
                          ease: "easeInOut"
                        }}
                      >
                        <path d="M12 0c-6.626 0-12 5.373-12 12 0 5.302 3.438 9.8 8.207 11.387.599.111.793-.261.793-.577v-2.234c-3.338.726-4.033-1.416-4.033-1.416-.546-1.387-1.333-1.756-1.333-1.756-1.089-.745.083-.729.083-.729 1.205.084 1.839 1.237 1.839 1.237 1.07 1.834 2.807 1.304 3.492.997.107-.775.418-1.305.762-1.604-2.665-.305-5.467-1.334-5.467-5.931 0-1.311.469-2.381 1.236-3.221-.124-.303-.535-1.524.117-3.176 0 0 1.008-.322 3.301 1.23.957-.266 1.983-.399 3.003-.404 1.02.005 2.047.138 3.006.404 2.291-1.552 3.297-1.23 3.297-1.23.653 1.653.242 2.874.118 3.176.77.84 1.235 1.911 1.235 3.221 0 4.609-2.807 5.624-5.479 5.921.43.372.823 1.102.823 2.222v3.293c0 .319.192.694.801.576 4.765-1.589 8.199-6.086 8.199-11.386 0-6.627-5.373-12-12-12z"/>
                      </motion.svg>
                    </motion.div>
                    
                    {/* Content with enhanced typography */}
                    <div className="flex-1">
                      <motion.div 
                        className="text-sm text-foreground-secondary font-medium"
                        initial={{ opacity: 0, y: 10 }}
                        whileInView={{ opacity: 1, y: 0 }}
                        transition={{ duration: 0.5, delay: 0.6 }}
                        viewport={{ once: true }}
                      >
                        GitHub
                      </motion.div>
                      <motion.div 
                        className="text-sm font-semibold text-foreground group-hover:text-gray-800 transition-colors duration-300"
                        initial={{ opacity: 0, y: 10 }}
                        whileInView={{ opacity: 1, y: 0 }}
                        transition={{ duration: 0.5, delay: 0.7 }}
                        viewport={{ once: true }}
                      >
                        kwame-minta2021
                      </motion.div>
                    </div>
                    
                    {/* Animated arrow */}
                    <motion.div
                      className="text-gray-800 opacity-0 group-hover:opacity-100 transition-opacity duration-300"
                      animate={{ x: [0, 5, 0] }}
                      transition={{
                        duration: 1.5,
                        repeat: Infinity,
                        ease: "easeInOut"
                      }}
                    >
                      →
                    </motion.div>
                  </motion.a>
                </div>
            </motion.div>

            <motion.div
              initial={{ opacity: 0, x: 50 }}
              whileInView={{ opacity: 1, x: 0 }}
              transition={{ duration: 0.8 }}
              viewport={{ once: true }}
            >
              <form className="space-y-4">
                <input
                  type="text"
                  placeholder="Your name"
                  className="w-full px-4 py-3 bg-background border border-border rounded-lg text-foreground placeholder-foreground-secondary focus:outline-none focus:ring-2 focus:ring-primary focus:border-transparent transition-colors"
                />
                <input
                  type="email"
                  placeholder="your.email@example.com"
                  className="w-full px-4 py-3 bg-background border border-border rounded-lg text-foreground placeholder-foreground-secondary focus:outline-none focus:ring-2 focus:ring-primary focus:border-transparent transition-colors"
                />
                <textarea
                  placeholder="Your message"
                  rows={4}
                  className="w-full px-4 py-3 bg-background border border-border rounded-lg text-foreground placeholder-foreground-secondary focus:outline-none focus:ring-2 focus:ring-primary focus:border-transparent transition-colors"
                ></textarea>
                <button
                  type="submit"
                  className="w-full px-8 py-4 bg-primary text-background rounded-lg font-medium hover:bg-primary-hover transition-colors"
                >
                  Send Message
                </button>
              </form>
            </motion.div>
          </div>
        </div>
      </motion.section>
    </main>
  )
} 
