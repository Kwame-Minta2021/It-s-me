'use client'

import { motion } from 'framer-motion'
import { Calendar, MapPin, Building, Award, TrendingUp } from 'lucide-react'

interface Experience {
  id: string
  title: string
  company: string
  location: string
  period: string
  description: string
  achievements: string[]
  technologies: string[]
  logo?: string
}

const experiences: Experience[] = [
  {
    id: 'senior-robotics-engineer',
    title: 'Senior Robotics Software Engineer',
    company: 'Autonomous Systems Inc.',
    location: 'San Francisco, CA',
    period: '2022 - Present',
    description: 'Leading the development of autonomous navigation systems for industrial robots, focusing on SLAM algorithms and real-time control systems.',
    achievements: [
      'Led a team of 5 engineers in developing a next-generation SLAM system',
      'Improved navigation accuracy by 40% through advanced sensor fusion algorithms',
      'Reduced system latency by 60% through optimization of real-time control loops',
      'Mentored 3 junior engineers and established best practices for robotics development'
    ],
    technologies: ['ROS2', 'C++', 'Python', 'SLAM', 'Computer Vision', 'Docker', 'AWS']
  },
  {
    id: 'robotics-engineer',
    title: 'Robotics Software Engineer',
    company: 'Tech Robotics Lab',
    location: 'Boston, MA',
    period: '2020 - 2022',
    description: 'Developed perception and control systems for autonomous vehicles and mobile robots, specializing in computer vision and machine learning.',
    achievements: [
      'Implemented a multi-sensor fusion system achieving 95% detection accuracy',
      'Developed a reinforcement learning framework for robot manipulation tasks',
      'Published 3 research papers in top robotics conferences',
      'Contributed to open-source robotics projects with 500+ GitHub stars'
    ],
    technologies: ['ROS', 'Python', 'PyTorch', 'OpenCV', 'Gazebo', 'Git', 'Linux']
  },
  {
    id: 'research-assistant',
    title: 'Research Assistant',
    company: 'Robotics Research Institute',
    location: 'Pittsburgh, PA',
    period: '2018 - 2020',
    description: 'Conducted research in autonomous systems and machine learning, focusing on robotic perception and decision-making algorithms.',
    achievements: [
      'Developed novel SLAM algorithms for dynamic environments',
      'Implemented real-time object detection systems for mobile robots',
      'Presented research at IEEE ICRA and IROS conferences',
      'Collaborated with international research teams on EU-funded projects'
    ],
    technologies: ['MATLAB', 'Python', 'C++', 'ROS', 'TensorFlow', 'OpenCV']
  },
  {
    id: 'software-intern',
    title: 'Software Engineering Intern',
    company: 'Industrial Robotics Corp.',
    location: 'Detroit, MI',
    period: '2017 - 2018',
    description: 'Worked on industrial automation systems and robotic arm control software, gaining hands-on experience with real-world robotics applications.',
    achievements: [
      'Developed control software for 6-DOF robotic arms',
      'Implemented safety systems for human-robot collaboration',
      'Optimized manufacturing processes reducing cycle time by 25%',
      'Received outstanding intern award for technical contributions'
    ],
    technologies: ['C++', 'PLC Programming', 'RobotStudio', 'Safety Systems']
  }
]

export function Experience() {
  return (
    <section id="experience" className="py-20 bg-background">
      <div className="max-w-7xl mx-auto px-4 sm:px-6 lg:px-8">
        <motion.div
          initial={{ opacity: 0, y: 20 }}
          whileInView={{ opacity: 1, y: 0 }}
          transition={{ duration: 0.8 }}
          viewport={{ once: true }}
          className="text-center mb-16"
        >
          <h2 className="text-3xl md:text-4xl font-display font-bold mb-4">
            Professional Experience
          </h2>
          <p className="text-foreground-secondary text-lg max-w-2xl mx-auto">
            A journey through my professional growth in robotics software engineering, 
            from research to industry leadership.
          </p>
        </motion.div>

        <div className="relative">
          {/* Timeline Line */}
          <div className="absolute left-8 top-0 bottom-0 w-0.5 bg-border hidden lg:block" />

          <div className="space-y-12">
            {experiences.map((experience, index) => (
              <motion.div
                key={experience.id}
                initial={{ opacity: 0, x: index % 2 === 0 ? -50 : 50 }}
                whileInView={{ opacity: 1, x: 0 }}
                transition={{ duration: 0.8, delay: index * 0.2 }}
                viewport={{ once: true }}
                className={`relative flex flex-col lg:flex-row gap-8 ${
                  index % 2 === 0 ? 'lg:flex-row' : 'lg:flex-row-reverse'
                }`}
              >
                {/* Timeline Dot */}
                <div className="absolute left-8 top-6 w-4 h-4 bg-primary rounded-full border-4 border-background hidden lg:block" />

                {/* Content */}
                <div className={`flex-1 ${index % 2 === 0 ? 'lg:ml-16' : 'lg:mr-16'}`}>
                  <motion.div
                    initial={{ opacity: 0, y: 20 }}
                    whileInView={{ opacity: 1, y: 0 }}
                    transition={{ duration: 0.8, delay: index * 0.2 + 0.3 }}
                    viewport={{ once: true }}
                    className="bg-background-secondary rounded-lg border border-border p-6 hover:border-primary/50 transition-colors"
                  >
                    {/* Header */}
                    <div className="flex flex-col sm:flex-row sm:items-center sm:justify-between mb-4">
                      <div>
                        <h3 className="text-xl font-display font-bold text-foreground mb-1">
                          {experience.title}
                        </h3>
                        <div className="flex items-center space-x-4 text-sm text-foreground-secondary">
                          <div className="flex items-center space-x-1">
                            <Building size={14} />
                            <span>{experience.company}</span>
                          </div>
                          <div className="flex items-center space-x-1">
                            <MapPin size={14} />
                            <span>{experience.location}</span>
                          </div>
                        </div>
                      </div>
                      <div className="flex items-center space-x-1 text-sm text-primary font-medium mt-2 sm:mt-0">
                        <Calendar size={14} />
                        <span>{experience.period}</span>
                      </div>
                    </div>

                    {/* Description */}
                    <p className="text-foreground-secondary mb-6">
                      {experience.description}
                    </p>

                    {/* Achievements */}
                    <div className="mb-6">
                      <h4 className="text-sm font-bold text-foreground mb-3 flex items-center space-x-2">
                        <Award size={16} />
                        <span>Key Achievements</span>
                      </h4>
                      <ul className="space-y-2">
                        {experience.achievements.map((achievement, achievementIndex) => (
                          <motion.li
                            key={achievementIndex}
                            initial={{ opacity: 0, x: -20 }}
                            whileInView={{ opacity: 1, x: 0 }}
                            transition={{ duration: 0.5, delay: achievementIndex * 0.1 }}
                            viewport={{ once: true }}
                            className="flex items-start space-x-2 text-sm text-foreground-secondary"
                          >
                            <span className="text-primary mt-1">•</span>
                            <span>{achievement}</span>
                          </motion.li>
                        ))}
                      </ul>
                    </div>

                    {/* Technologies */}
                    <div>
                      <h4 className="text-sm font-bold text-foreground mb-3 flex items-center space-x-2">
                        <TrendingUp size={16} />
                        <span>Technologies & Tools</span>
                      </h4>
                      <div className="flex flex-wrap gap-2">
                        {experience.technologies.map((tech, techIndex) => (
                          <motion.span
                            key={tech}
                            initial={{ opacity: 0, scale: 0.8 }}
                            whileInView={{ opacity: 1, scale: 1 }}
                            transition={{ duration: 0.3, delay: techIndex * 0.05 }}
                            viewport={{ once: true }}
                            className="skill-badge text-xs"
                          >
                            {tech}
                          </motion.span>
                        ))}
                      </div>
                    </div>
                  </motion.div>
                </div>
              </motion.div>
            ))}
          </div>
        </div>

        {/* Career Highlights */}
        <motion.div
          initial={{ opacity: 0, y: 20 }}
          whileInView={{ opacity: 1, y: 0 }}
          transition={{ duration: 0.8, delay: 0.5 }}
          viewport={{ once: true }}
          className="mt-20"
        >
          <h3 className="text-2xl font-display font-bold text-center mb-12">
            Career Highlights
          </h3>
          
          <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-4 gap-6">
            {[
              {
                number: '5+',
                label: 'Years Experience',
                description: 'Robotics software development'
              },
              {
                number: '15+',
                label: 'Research Papers',
                description: 'Published in top conferences'
              },
              {
                number: '50+',
                label: 'Projects Delivered',
                description: 'From concept to deployment'
              },
              {
                number: '10+',
                label: 'Team Members Led',
                description: 'Cross-functional engineering teams'
              }
            ].map((highlight, index) => (
              <motion.div
                key={highlight.label}
                initial={{ opacity: 0, scale: 0.8 }}
                whileInView={{ opacity: 1, scale: 1 }}
                transition={{ duration: 0.5, delay: index * 0.1 }}
                viewport={{ once: true }}
                className="text-center p-6 bg-background-secondary rounded-lg border border-border"
              >
                <div className="text-3xl font-bold text-primary mb-2">
                  {highlight.number}
                </div>
                <div className="text-sm font-bold text-foreground mb-1">
                  {highlight.label}
                </div>
                <div className="text-xs text-foreground-secondary">
                  {highlight.description}
                </div>
              </motion.div>
            ))}
          </div>
        </motion.div>
      </div>
    </section>
  )
}
