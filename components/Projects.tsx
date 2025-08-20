'use client'

import { useState } from 'react'
import { motion, AnimatePresence } from 'framer-motion'
import { ExternalLink, Github, Play, ArrowRight, Cpu, Map, Brain, Zap, BookOpen } from 'lucide-react'

interface Project {
  id: string
  title: string
  description: string
  image: string
  technologies: string[]
  outcome: string
  github?: string
  demo?: string
  paper?: string
  category: 'slam' | 'control' | 'ml' | 'perception'
  icon: React.ReactNode
}

const projects: Project[] = [
  {
    id: 'autonomous-navigation',
    title: 'Autonomous Navigation System',
    description: 'Developed a complete SLAM-based navigation system for mobile robots using ROS, achieving 95% localization accuracy in dynamic environments.',
    image: '/api/placeholder/400/250',
    technologies: ['ROS', 'C++', 'Python', 'SLAM', 'Navigation Stack'],
    outcome: 'Reduced navigation errors by 60% and improved path planning efficiency by 40%',
    github: 'https://github.com',
    demo: 'https://demo.com',
    category: 'slam',
    icon: <Map size={24} />
  },
  {
    id: 'reinforcement-learning',
    title: 'RL for Robot Manipulation',
    description: 'Implemented deep reinforcement learning algorithms for robotic arm control, enabling complex manipulation tasks with minimal human intervention.',
    image: '/api/placeholder/400/250',
    technologies: ['PyTorch', 'ROS', 'Gazebo', 'DDPG', 'SAC'],
    outcome: 'Achieved 85% success rate in complex pick-and-place tasks',
    github: 'https://github.com',
    paper: 'https://arxiv.org',
    category: 'ml',
    icon: <Brain size={24} />
  },
  {
    id: 'sensor-fusion',
    title: 'Multi-Sensor Fusion System',
    description: 'Built a robust sensor fusion framework combining LiDAR, camera, and IMU data for accurate pose estimation in challenging environments.',
    image: '/api/placeholder/400/250',
    technologies: ['C++', 'OpenCV', 'PCL', 'EKF', 'ROS'],
    outcome: 'Improved pose estimation accuracy by 30% in low-visibility conditions',
    github: 'https://github.com',
    category: 'perception',
    icon: <Cpu size={24} />
  },
  {
    id: 'real-time-control',
    title: 'Real-Time Control System',
    description: 'Designed and implemented a high-frequency control system for precision robotics applications with sub-millisecond response times.',
    image: '/api/placeholder/400/250',
    technologies: ['C++', 'Real-time Linux', 'PID Control', 'ROS2'],
    outcome: 'Achieved 1kHz control loop frequency with <1ms latency',
    github: 'https://github.com',
    category: 'control',
    icon: <Zap size={24} />
  },
  {
    id: 'computer-vision',
    title: 'Computer Vision Pipeline',
    description: 'Developed a comprehensive computer vision system for object detection and tracking in industrial robotics applications.',
    image: '/api/placeholder/400/250',
    technologies: ['Python', 'OpenCV', 'YOLO', 'ROS', 'TensorFlow'],
    outcome: 'Increased detection accuracy by 25% while reducing processing time by 40%',
    github: 'https://github.com',
    demo: 'https://demo.com',
    category: 'perception',
    icon: <Cpu size={24} />
  },
  {
    id: 'swarm-robotics',
    title: 'Swarm Robotics Framework',
    description: 'Created a distributed control system for coordinating multiple robots in collaborative tasks using decentralized algorithms.',
    image: '/api/placeholder/400/250',
    technologies: ['ROS', 'Python', 'Distributed Systems', 'Consensus Algorithms'],
    outcome: 'Successfully coordinated 10+ robots in collaborative mapping tasks',
    github: 'https://github.com',
    paper: 'https://arxiv.org',
    category: 'control',
    icon: <Zap size={24} />
  }
]

export function Projects() {
  const [selectedCategory, setSelectedCategory] = useState<string>('all')
  const [selectedProject, setSelectedProject] = useState<Project | null>(null)

  const categories = [
    { id: 'all', name: 'All Projects' },
    { id: 'slam', name: 'SLAM & Navigation' },
    { id: 'control', name: 'Control Systems' },
    { id: 'ml', name: 'Machine Learning' },
    { id: 'perception', name: 'Perception' }
  ]

  const filteredProjects = selectedCategory === 'all' 
    ? projects 
    : projects.filter(project => project.category === selectedCategory)

  return (
    <section id="projects" className="py-20 bg-background">
      <div className="max-w-7xl mx-auto px-4 sm:px-6 lg:px-8">
        <motion.div
          initial={{ opacity: 0, y: 20 }}
          whileInView={{ opacity: 1, y: 0 }}
          transition={{ duration: 0.8 }}
          viewport={{ once: true }}
          className="text-center mb-16"
        >
          <h2 className="text-3xl md:text-4xl font-display font-bold mb-4">
            Featured Projects
          </h2>
          <p className="text-foreground-secondary text-lg max-w-2xl mx-auto">
            A showcase of my work in robotics software engineering, from autonomous navigation 
            to machine learning applications.
          </p>
        </motion.div>

        {/* Category Filter */}
        <motion.div
          initial={{ opacity: 0, y: 20 }}
          whileInView={{ opacity: 1, y: 0 }}
          transition={{ duration: 0.8, delay: 0.2 }}
          viewport={{ once: true }}
          className="flex flex-wrap justify-center gap-4 mb-12"
        >
          {categories.map((category) => (
            <button
              key={category.id}
              onClick={() => setSelectedCategory(category.id)}
              className={`px-6 py-2 rounded-full text-sm font-medium transition-colors ${
                selectedCategory === category.id
                  ? 'bg-primary text-background'
                  : 'bg-background-secondary text-foreground-secondary hover:text-foreground'
              }`}
            >
              {category.name}
            </button>
          ))}
        </motion.div>

        {/* Projects Grid */}
        <motion.div
          layout
          className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-8"
        >
          <AnimatePresence>
            {filteredProjects.map((project, index) => (
              <motion.div
                key={project.id}
                layout
                initial={{ opacity: 0, y: 20 }}
                animate={{ opacity: 1, y: 0 }}
                exit={{ opacity: 0, y: -20 }}
                transition={{ duration: 0.5, delay: index * 0.1 }}
                className="project-card group"
              >
                {/* Project Image */}
                <div className="relative h-48 bg-gradient-to-br from-primary/20 to-accent/20 rounded-t-lg flex items-center justify-center">
                  <div className="text-primary opacity-60 group-hover:opacity-100 transition-opacity">
                    {project.icon}
                  </div>
                  <div className="absolute inset-0 bg-gradient-to-t from-background/80 to-transparent opacity-0 group-hover:opacity-100 transition-opacity" />
                  
                  {/* Action Buttons */}
                  <div className="absolute top-4 right-4 flex space-x-2 opacity-0 group-hover:opacity-100 transition-opacity">
                    {project.github && (
                      <motion.a
                        href={project.github}
                        target="_blank"
                        rel="noopener noreferrer"
                        whileHover={{ scale: 1.1 }}
                        whileTap={{ scale: 0.9 }}
                        className="p-2 bg-background/80 rounded-lg text-foreground-secondary hover:text-foreground"
                      >
                        <Github size={16} />
                      </motion.a>
                    )}
                    {project.demo && (
                      <motion.a
                        href={project.demo}
                        target="_blank"
                        rel="noopener noreferrer"
                        whileHover={{ scale: 1.1 }}
                        whileTap={{ scale: 0.9 }}
                        className="p-2 bg-background/80 rounded-lg text-foreground-secondary hover:text-foreground"
                      >
                        <Play size={16} />
                      </motion.a>
                    )}
                  </div>
                </div>

                {/* Project Content */}
                <div className="p-6">
                  <h3 className="text-xl font-display font-bold mb-2 group-hover:text-primary transition-colors">
                    {project.title}
                  </h3>
                  
                  <p className="text-foreground-secondary text-sm mb-4 line-clamp-3">
                    {project.description}
                  </p>

                  {/* Technologies */}
                  <div className="flex flex-wrap gap-2 mb-4">
                    {project.technologies.slice(0, 3).map((tech) => (
                      <span key={tech} className="skill-badge text-xs">
                        {tech}
                      </span>
                    ))}
                    {project.technologies.length > 3 && (
                      <span className="skill-badge text-xs">
                        +{project.technologies.length - 3} more
                      </span>
                    )}
                  </div>

                  {/* Outcome */}
                  <div className="mb-4">
                    <p className="text-sm text-accent font-medium">
                      {project.outcome}
                    </p>
                  </div>

                  {/* View Details Button */}
                  <motion.button
                    onClick={() => setSelectedProject(project)}
                    whileHover={{ scale: 1.02 }}
                    whileTap={{ scale: 0.98 }}
                    className="flex items-center space-x-2 text-primary hover:text-primary-hover transition-colors text-sm font-medium"
                  >
                    <span>View Details</span>
                    <ArrowRight size={16} />
                  </motion.button>
                </div>
              </motion.div>
            ))}
          </AnimatePresence>
        </motion.div>

        {/* Project Modal */}
        <AnimatePresence>
          {selectedProject && (
            <motion.div
              initial={{ opacity: 0 }}
              animate={{ opacity: 1 }}
              exit={{ opacity: 0 }}
              className="fixed inset-0 bg-background/80 backdrop-blur-sm z-50 flex items-center justify-center p-4"
              onClick={() => setSelectedProject(null)}
            >
              <motion.div
                initial={{ scale: 0.9, opacity: 0 }}
                animate={{ scale: 1, opacity: 1 }}
                exit={{ scale: 0.9, opacity: 0 }}
                className="bg-background-secondary rounded-lg max-w-4xl w-full max-h-[90vh] overflow-y-auto"
                onClick={(e) => e.stopPropagation()}
              >
                <div className="p-8">
                  <div className="flex items-start justify-between mb-6">
                    <div className="flex items-center space-x-4">
                      <div className="text-primary">
                        {selectedProject.icon}
                      </div>
                      <div>
                        <h3 className="text-2xl font-display font-bold">
                          {selectedProject.title}
                        </h3>
                        <p className="text-foreground-secondary">
                          {selectedProject.category.toUpperCase()}
                        </p>
                      </div>
                    </div>
                    <button
                      onClick={() => setSelectedProject(null)}
                      className="text-foreground-secondary hover:text-foreground"
                    >
                      <ArrowRight size={24} className="rotate-45" />
                    </button>
                  </div>

                  <div className="grid lg:grid-cols-2 gap-8">
                    <div>
                      <h4 className="text-lg font-bold mb-3">Project Overview</h4>
                      <p className="text-foreground-secondary mb-6">
                        {selectedProject.description}
                      </p>

                      <h4 className="text-lg font-bold mb-3">Technologies Used</h4>
                      <div className="flex flex-wrap gap-2 mb-6">
                        {selectedProject.technologies.map((tech) => (
                          <span key={tech} className="skill-badge">
                            {tech}
                          </span>
                        ))}
                      </div>

                      <h4 className="text-lg font-bold mb-3">Key Outcome</h4>
                      <p className="text-accent font-medium mb-6">
                        {selectedProject.outcome}
                      </p>
                    </div>

                    <div>
                      <h4 className="text-lg font-bold mb-3">Project Links</h4>
                      <div className="space-y-3">
                        {selectedProject.github && (
                          <a
                            href={selectedProject.github}
                            target="_blank"
                            rel="noopener noreferrer"
                            className="flex items-center space-x-2 text-foreground-secondary hover:text-foreground transition-colors"
                          >
                            <Github size={20} />
                            <span>View on GitHub</span>
                            <ExternalLink size={16} />
                          </a>
                        )}
                        {selectedProject.demo && (
                          <a
                            href={selectedProject.demo}
                            target="_blank"
                            rel="noopener noreferrer"
                            className="flex items-center space-x-2 text-foreground-secondary hover:text-foreground transition-colors"
                          >
                            <Play size={20} />
                            <span>Live Demo</span>
                            <ExternalLink size={16} />
                          </a>
                        )}
                        {selectedProject.paper && (
                          <a
                            href={selectedProject.paper}
                            target="_blank"
                            rel="noopener noreferrer"
                            className="flex items-center space-x-2 text-foreground-secondary hover:text-foreground transition-colors"
                          >
                            <BookOpen size={20} />
                            <span>Research Paper</span>
                            <ExternalLink size={16} />
                          </a>
                        )}
                      </div>
                    </div>
                  </div>
                </div>
              </motion.div>
            </motion.div>
          )}
        </AnimatePresence>
      </div>
    </section>
  )
} 
