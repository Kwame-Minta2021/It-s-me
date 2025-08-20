'use client'

import { motion } from 'framer-motion'
import { Download, Award, Users, BookOpen, Code, Cpu } from 'lucide-react'

export function About() {
  return (
    <section id="about" className="py-20 bg-background-secondary">
      <div className="max-w-7xl mx-auto px-4 sm:px-6 lg:px-8">
        <motion.div
          initial={{ opacity: 0, y: 20 }}
          whileInView={{ opacity: 1, y: 0 }}
          transition={{ duration: 0.8 }}
          viewport={{ once: true }}
          className="text-center mb-16"
        >
          <h2 className="text-3xl md:text-4xl font-display font-bold mb-4">
            About Me
          </h2>
          <p className="text-foreground-secondary text-lg max-w-2xl mx-auto">
            Passionate robotics software engineer with expertise in autonomous systems, 
            computer vision, and machine learning.
          </p>
        </motion.div>

        <div className="grid lg:grid-cols-3 gap-12 items-start">
          {/* Portrait */}
          <motion.div
            initial={{ opacity: 0, x: -50 }}
            whileInView={{ opacity: 1, x: 0 }}
            transition={{ duration: 0.8 }}
            viewport={{ once: true }}
            className="lg:col-span-1"
          >
            <div className="relative">
              <div className="w-64 h-64 mx-auto bg-gradient-to-br from-primary to-accent rounded-2xl p-1">
                <div className="w-full h-full bg-background rounded-2xl flex items-center justify-center">
                  <div className="text-center">
                    <div className="w-32 h-32 bg-gradient-to-br from-primary/20 to-accent/20 rounded-full flex items-center justify-center mb-4">
                      <Cpu size={48} className="text-primary" />
                    </div>
                    <h3 className="text-xl font-bold mb-2">Robotics Engineer</h3>
                    <p className="text-foreground-secondary text-sm">AI · Control · Perception</p>
                  </div>
                </div>
              </div>
              
              <motion.button
                whileHover={{ scale: 1.05 }}
                whileTap={{ scale: 0.95 }}
                className="mt-6 w-full flex items-center justify-center space-x-2 px-6 py-3 bg-primary text-background rounded-lg font-medium hover:bg-primary-hover transition-colors"
              >
                <Download size={20} />
                <span>Download CV</span>
              </motion.button>
            </div>
          </motion.div>

          {/* Bio */}
          <motion.div
            initial={{ opacity: 0, y: 20 }}
            whileInView={{ opacity: 1, y: 0 }}
            transition={{ duration: 0.8, delay: 0.2 }}
            viewport={{ once: true }}
            className="lg:col-span-2 space-y-8"
          >
            <div className="space-y-6">
              <h3 className="text-2xl font-display font-bold">
                Building the Future of Autonomous Systems
              </h3>
              
              <div className="space-y-4 text-foreground-secondary">
                <p>
                  I'm a robotics software engineer with over 5 years of experience developing 
                  autonomous systems that navigate complex environments. My expertise spans 
                  the full robotics stack, from low-level control systems to high-level 
                  decision-making algorithms.
                </p>
                
                <p>
                  I specialize in ROS (Robot Operating System), C++, and Python, with deep 
                  knowledge in SLAM (Simultaneous Localization and Mapping), computer vision, 
                  and reinforcement learning. My work has been applied to autonomous vehicles, 
                  industrial robots, and research platforms.
                </p>
                
                <p>
                  Currently focused on developing robust perception systems and control 
                  algorithms that enable robots to operate safely and efficiently in 
                  unstructured environments. I'm passionate about open-source robotics 
                  and contributing to the advancement of autonomous technology.
                </p>
              </div>
            </div>

            {/* Quick Facts */}
            <div className="grid grid-cols-2 md:grid-cols-4 gap-6">
              <motion.div
                initial={{ opacity: 0, scale: 0.8 }}
                whileInView={{ opacity: 1, scale: 1 }}
                transition={{ duration: 0.5, delay: 0.1 }}
                viewport={{ once: true }}
                className="text-center p-4 bg-background rounded-lg border border-border"
              >
                <Award className="w-8 h-8 text-primary mx-auto mb-2" />
                <div className="text-2xl font-bold text-foreground">5+</div>
                <div className="text-sm text-foreground-secondary">Years Experience</div>
              </motion.div>
              
              <motion.div
                initial={{ opacity: 0, scale: 0.8 }}
                whileInView={{ opacity: 1, scale: 1 }}
                transition={{ duration: 0.5, delay: 0.2 }}
                viewport={{ once: true }}
                className="text-center p-4 bg-background rounded-lg border border-border"
              >
                <Code className="w-8 h-8 text-primary mx-auto mb-2" />
                <div className="text-2xl font-bold text-foreground">50+</div>
                <div className="text-sm text-foreground-secondary">Projects Delivered</div>
              </motion.div>
              
              <motion.div
                initial={{ opacity: 0, scale: 0.8 }}
                whileInView={{ opacity: 1, scale: 1 }}
                transition={{ duration: 0.5, delay: 0.3 }}
                viewport={{ once: true }}
                className="text-center p-4 bg-background rounded-lg border border-border"
              >
                <BookOpen className="w-8 h-8 text-primary mx-auto mb-2" />
                <div className="text-2xl font-bold text-foreground">15+</div>
                <div className="text-sm text-foreground-secondary">Research Papers</div>
              </motion.div>
              
              <motion.div
                initial={{ opacity: 0, scale: 0.8 }}
                whileInView={{ opacity: 1, scale: 1 }}
                transition={{ duration: 0.5, delay: 0.4 }}
                viewport={{ once: true }}
                className="text-center p-4 bg-background rounded-lg border border-border"
              >
                <Users className="w-8 h-8 text-primary mx-auto mb-2" />
                <div className="text-2xl font-bold text-foreground">10+</div>
                <div className="text-sm text-foreground-secondary">Team Collaborations</div>
              </motion.div>
            </div>

            {/* Core Competencies */}
            <div className="space-y-4">
              <h4 className="text-xl font-display font-bold">Core Competencies</h4>
              <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
                <div className="space-y-2">
                  <div className="flex items-center justify-between">
                    <span className="text-sm text-foreground-secondary">ROS/ROS2</span>
                    <span className="text-sm text-primary">Expert</span>
                  </div>
                  <div className="w-full bg-background rounded-full h-2">
                    <div className="bg-primary h-2 rounded-full" style={{ width: '95%' }} />
                  </div>
                </div>
                
                <div className="space-y-2">
                  <div className="flex items-center justify-between">
                    <span className="text-sm text-foreground-secondary">C++/Python</span>
                    <span className="text-sm text-primary">Expert</span>
                  </div>
                  <div className="w-full bg-background rounded-full h-2">
                    <div className="bg-primary h-2 rounded-full" style={{ width: '90%' }} />
                  </div>
                </div>
                
                <div className="space-y-2">
                  <div className="flex items-center justify-between">
                    <span className="text-sm text-foreground-secondary">SLAM & Navigation</span>
                    <span className="text-sm text-primary">Advanced</span>
                  </div>
                  <div className="w-full bg-background rounded-full h-2">
                    <div className="bg-primary h-2 rounded-full" style={{ width: '85%' }} />
                  </div>
                </div>
                
                <div className="space-y-2">
                  <div className="flex items-center justify-between">
                    <span className="text-sm text-foreground-secondary">Machine Learning</span>
                    <span className="text-sm text-primary">Advanced</span>
                  </div>
                  <div className="w-full bg-background rounded-full h-2">
                    <div className="bg-primary h-2 rounded-full" style={{ width: '80%' }} />
                  </div>
                </div>
              </div>
            </div>
          </motion.div>
        </div>
      </div>
    </section>
  )
} 
