'use client'

import { motion } from 'framer-motion'
import { Code, Cpu, Brain, Settings, Database, Cloud, GitBranch, Shield } from 'lucide-react'

interface SkillCategory {
  id: string
  name: string
  icon: React.ReactNode
  skills: {
    name: string
    level: 'beginner' | 'intermediate' | 'advanced' | 'expert'
    years?: number
  }[]
}

const skillCategories: SkillCategory[] = [
  {
    id: 'programming',
    name: 'Programming Languages',
    icon: <Code size={24} />,
    skills: [
      { name: 'C++', level: 'expert', years: 6 },
      { name: 'Python', level: 'expert', years: 5 },
      { name: 'MATLAB', level: 'advanced', years: 4 },
      { name: 'JavaScript/TypeScript', level: 'intermediate', years: 2 },
      { name: 'Rust', level: 'intermediate', years: 1 },
    ]
  },
  {
    id: 'robotics',
    name: 'Robotics & Control',
    icon: <Cpu size={24} />,
    skills: [
      { name: 'ROS/ROS2', level: 'expert', years: 5 },
      { name: 'MoveIt', level: 'advanced', years: 4 },
      { name: 'Gazebo', level: 'advanced', years: 4 },
      { name: 'SLAM', level: 'expert', years: 5 },
      { name: 'Navigation Stack', level: 'advanced', years: 4 },
      { name: 'PID Control', level: 'expert', years: 5 },
      { name: 'MPC', level: 'advanced', years: 3 },
      { name: 'URDF/XACRO', level: 'advanced', years: 4 },
    ]
  },
  {
    id: 'ai-ml',
    name: 'AI & Machine Learning',
    icon: <Brain size={24} />,
    skills: [
      { name: 'PyTorch', level: 'advanced', years: 4 },
      { name: 'TensorFlow', level: 'intermediate', years: 3 },
      { name: 'Reinforcement Learning', level: 'advanced', years: 3 },
      { name: 'Computer Vision', level: 'expert', years: 5 },
      { name: 'OpenCV', level: 'expert', years: 5 },
      { name: 'PCL (Point Cloud Library)', level: 'advanced', years: 4 },
      { name: 'Deep Learning', level: 'advanced', years: 4 },
      { name: 'Neural Networks', level: 'advanced', years: 4 },
    ]
  },
  {
    id: 'systems',
    name: 'Systems & DevOps',
    icon: <Settings size={24} />,
    skills: [
      { name: 'Linux', level: 'expert', years: 6 },
      { name: 'Docker', level: 'advanced', years: 3 },
      { name: 'Git', level: 'expert', years: 5 },
      { name: 'CI/CD', level: 'advanced', years: 3 },
      { name: 'Real-time Systems', level: 'advanced', years: 4 },
      { name: 'Embedded Systems', level: 'intermediate', years: 2 },
      { name: 'Microcontrollers', level: 'intermediate', years: 2 },
    ]
  },
  {
    id: 'databases',
    name: 'Databases & Storage',
    icon: <Database size={24} />,
    skills: [
      { name: 'SQL', level: 'intermediate', years: 3 },
      { name: 'MongoDB', level: 'intermediate', years: 2 },
      { name: 'Redis', level: 'intermediate', years: 2 },
      { name: 'ROS Bags', level: 'expert', years: 5 },
      { name: 'Data Logging', level: 'advanced', years: 4 },
    ]
  },
  {
    id: 'cloud',
    name: 'Cloud & Deployment',
    icon: <Cloud size={24} />,
    skills: [
      { name: 'AWS', level: 'intermediate', years: 2 },
      { name: 'Google Cloud', level: 'intermediate', years: 2 },
      { name: 'Kubernetes', level: 'beginner', years: 1 },
      { name: 'ROS Cloud', level: 'intermediate', years: 2 },
      { name: 'Edge Computing', level: 'intermediate', years: 2 },
    ]
  },
  {
    id: 'tools',
    name: 'Development Tools',
    icon: <GitBranch size={24} />,
    skills: [
      { name: 'VSCode', level: 'expert', years: 5 },
      { name: 'Vim', level: 'advanced', years: 4 },
      { name: 'CMake', level: 'advanced', years: 4 },
      { name: 'GDB', level: 'intermediate', years: 3 },
      { name: 'Valgrind', level: 'intermediate', years: 2 },
      { name: 'Jupyter Notebooks', level: 'advanced', years: 4 },
    ]
  },
  {
    id: 'security',
    name: 'Security & Safety',
    icon: <Shield size={24} />,
    skills: [
      { name: 'Robot Safety', level: 'advanced', years: 4 },
      { name: 'Cybersecurity', level: 'intermediate', years: 2 },
      { name: 'Functional Safety', level: 'intermediate', years: 2 },
      { name: 'Risk Assessment', level: 'advanced', years: 3 },
    ]
  }
]

const getLevelColor = (level: string) => {
  switch (level) {
    case 'expert':
      return 'text-accent'
    case 'advanced':
      return 'text-primary'
    case 'intermediate':
      return 'text-yellow-500'
    case 'beginner':
      return 'text-foreground-secondary'
    default:
      return 'text-foreground-secondary'
  }
}

const getLevelWidth = (level: string) => {
  switch (level) {
    case 'expert':
      return 'w-full'
    case 'advanced':
      return 'w-3/4'
    case 'intermediate':
      return 'w-1/2'
    case 'beginner':
      return 'w-1/4'
    default:
      return 'w-1/4'
  }
}

export function Skills() {
  return (
    <section id="skills" className="py-20 bg-background-secondary">
      <div className="max-w-7xl mx-auto px-4 sm:px-6 lg:px-8">
        <motion.div
          initial={{ opacity: 0, y: 20 }}
          whileInView={{ opacity: 1, y: 0 }}
          transition={{ duration: 0.8 }}
          viewport={{ once: true }}
          className="text-center mb-16"
        >
          <h2 className="text-3xl md:text-4xl font-display font-bold mb-4">
            Skills & Expertise
          </h2>
          <p className="text-foreground-secondary text-lg max-w-2xl mx-auto">
            Comprehensive technical skills spanning the full robotics software stack, 
            from low-level control to high-level AI applications.
          </p>
        </motion.div>

        <div className="grid grid-cols-1 md:grid-cols-2 gap-8">
          {skillCategories.map((category, categoryIndex) => (
            <motion.div
              key={category.id}
              initial={{ opacity: 0, y: 20 }}
              whileInView={{ opacity: 1, y: 0 }}
              transition={{ duration: 0.8, delay: categoryIndex * 0.1 }}
              viewport={{ once: true }}
              className="bg-background rounded-lg border border-border p-6"
            >
              <div className="flex items-center space-x-3 mb-6">
                <div className="text-primary">
                  {category.icon}
                </div>
                <h3 className="text-xl font-display font-bold">
                  {category.name}
                </h3>
              </div>

              <div className="space-y-4">
                {category.skills.map((skill, skillIndex) => (
                  <motion.div
                    key={skill.name}
                    initial={{ opacity: 0, x: -20 }}
                    whileInView={{ opacity: 1, x: 0 }}
                    transition={{ duration: 0.5, delay: skillIndex * 0.05 }}
                    viewport={{ once: true }}
                    className="space-y-2"
                  >
                    <div className="flex items-center justify-between">
                      <span className="text-sm font-medium text-foreground">
                        {skill.name}
                      </span>
                      <div className="flex items-center space-x-2">
                        <span className={`text-xs font-medium ${getLevelColor(skill.level)}`}>
                          {skill.level}
                        </span>
                        {skill.years && (
                          <span className="text-xs text-foreground-secondary">
                            {skill.years}y
                          </span>
                        )}
                      </div>
                    </div>
                    
                    <div className="w-full bg-background-secondary rounded-full h-2">
                      <motion.div
                        initial={{ width: 0 }}
                        whileInView={{ width: getLevelWidth(skill.level).replace('w-', '').replace('full', '100%').replace('3/4', '75%').replace('1/2', '50%').replace('1/4', '25%') + '%' }}
                        transition={{ duration: 1, delay: skillIndex * 0.1 }}
                        viewport={{ once: true }}
                        className={`h-2 rounded-full ${
                          skill.level === 'expert' ? 'bg-accent' :
                          skill.level === 'advanced' ? 'bg-primary' :
                          skill.level === 'intermediate' ? 'bg-yellow-500' :
                          'bg-foreground-secondary'
                        }`}
                      />
                    </div>
                  </motion.div>
                ))}
              </div>
            </motion.div>
          ))}
        </div>

        {/* Additional Skills */}
        <motion.div
          initial={{ opacity: 0, y: 20 }}
          whileInView={{ opacity: 1, y: 0 }}
          transition={{ duration: 0.8, delay: 0.5 }}
          viewport={{ once: true }}
          className="mt-16"
        >
          <h3 className="text-2xl font-display font-bold text-center mb-8">
            Additional Competencies
          </h3>
          
          <div className="grid grid-cols-2 md:grid-cols-4 lg:grid-cols-6 gap-4">
            {[
              'Agile Development', 'Project Management', 'Technical Writing',
              'System Architecture', 'Performance Optimization', 'Debugging',
              'API Design', 'Microservices', 'REST APIs', 'GraphQL',
              'Web Development', 'Mobile Development', 'IoT', '5G Networks',
              'Computer Networks', 'Parallel Computing', 'GPU Computing',
              'CUDA', 'OpenCL', 'FPGA Programming', 'VHDL', 'Verilog',
              'PCB Design', '3D Printing', 'CAD Software', 'Simulation'
            ].map((skill, index) => (
              <motion.div
                key={skill}
                initial={{ opacity: 0, scale: 0.8 }}
                whileInView={{ opacity: 1, scale: 1 }}
                transition={{ duration: 0.5, delay: index * 0.02 }}
                viewport={{ once: true }}
                className="text-center p-3 bg-background rounded-lg border border-border hover:border-primary/50 transition-colors"
              >
                <span className="text-sm text-foreground-secondary hover:text-foreground transition-colors">
                  {skill}
                </span>
              </motion.div>
            ))}
          </div>
        </motion.div>

        {/* Certifications */}
        <motion.div
          initial={{ opacity: 0, y: 20 }}
          whileInView={{ opacity: 1, y: 0 }}
          transition={{ duration: 0.8, delay: 0.7 }}
          viewport={{ once: true }}
          className="mt-16 text-center"
        >
          <h3 className="text-2xl font-display font-bold mb-8">
            Certifications & Training
          </h3>
          
          <div className="grid grid-cols-1 md:grid-cols-3 gap-6">
            {[
              {
                name: 'ROS Professional Certification',
                issuer: 'Open Robotics',
                year: '2023'
              },
              {
                name: 'AWS Certified Solutions Architect',
                issuer: 'Amazon Web Services',
                year: '2022'
              },
              {
                name: 'Google Cloud Professional ML Engineer',
                issuer: 'Google Cloud',
                year: '2022'
              }
            ].map((cert, index) => (
              <motion.div
                key={cert.name}
                initial={{ opacity: 0, y: 20 }}
                whileInView={{ opacity: 1, y: 0 }}
                transition={{ duration: 0.5, delay: index * 0.1 }}
                viewport={{ once: true }}
                className="p-6 bg-background rounded-lg border border-border"
              >
                <h4 className="font-bold text-foreground mb-2">{cert.name}</h4>
                <p className="text-foreground-secondary text-sm mb-1">{cert.issuer}</p>
                <p className="text-primary text-sm">{cert.year}</p>
              </motion.div>
            ))}
          </div>
        </motion.div>
      </div>
    </section>
  )
}
