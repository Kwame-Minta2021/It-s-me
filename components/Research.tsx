'use client'

import { useState } from 'react'
import { motion, AnimatePresence } from 'framer-motion'
import { ExternalLink, Download, Copy, Check, BookOpen, Award, Users, Calendar } from 'lucide-react'

interface Publication {
  id: string
  title: string
  authors: string[]
  venue: string
  year: string
  abstract: string
  doi?: string
  arxiv?: string
  pdf?: string
  citations: number
  type: 'conference' | 'journal' | 'workshop' | 'preprint'
  keywords: string[]
}

const publications: Publication[] = [
  {
    id: 'slam-dynamic-environments',
    title: 'Robust SLAM in Dynamic Environments Using Multi-Sensor Fusion',
    authors: ['Your Name', 'Dr. Jane Smith', 'Prof. John Doe'],
    venue: 'IEEE International Conference on Robotics and Automation (ICRA)',
    year: '2023',
    abstract: 'We present a novel approach to Simultaneous Localization and Mapping (SLAM) that robustly handles dynamic environments through intelligent multi-sensor fusion. Our method combines LiDAR, camera, and IMU data to achieve 95% localization accuracy in highly dynamic scenarios.',
    doi: '10.1109/ICRA.2023.123456',
    arxiv: 'https://arxiv.org/abs/2301.12345',
    pdf: '/papers/slam-dynamic-environments.pdf',
    citations: 45,
    type: 'conference',
    keywords: ['SLAM', 'Multi-Sensor Fusion', 'Dynamic Environments', 'Robotics']
  },
  {
    id: 'reinforcement-learning-manipulation',
    title: 'Deep Reinforcement Learning for Robotic Manipulation: A Comprehensive Survey',
    authors: ['Your Name', 'Dr. Alice Johnson'],
    venue: 'Robotics and Autonomous Systems',
    year: '2023',
    abstract: 'This comprehensive survey examines the state-of-the-art in deep reinforcement learning for robotic manipulation tasks. We analyze over 200 papers and provide insights into current challenges and future directions.',
    doi: '10.1016/j.robot.2023.104567',
    citations: 78,
    type: 'journal',
    keywords: ['Reinforcement Learning', 'Robotic Manipulation', 'Deep Learning', 'Survey']
  },
  {
    id: 'real-time-control-optimization',
    title: 'Real-Time Control System Optimization for High-Frequency Robotics Applications',
    authors: ['Your Name', 'Prof. Michael Brown', 'Dr. Sarah Wilson'],
    venue: 'IEEE/ASME Transactions on Mechatronics',
    year: '2022',
    abstract: 'We propose an optimization framework for real-time control systems that achieves sub-millisecond response times while maintaining stability and performance. Experimental results show 60% improvement in control accuracy.',
    doi: '10.1109/TMECH.2022.987654',
    citations: 32,
    type: 'journal',
    keywords: ['Real-Time Control', 'Optimization', 'High-Frequency', 'Robotics']
  },
  {
    id: 'computer-vision-pipeline',
    title: 'End-to-End Computer Vision Pipeline for Industrial Robotics',
    authors: ['Your Name', 'Dr. Robert Chen'],
    venue: 'IEEE International Conference on Computer Vision (ICCV)',
    year: '2022',
    abstract: 'An end-to-end computer vision pipeline designed specifically for industrial robotics applications. Our approach achieves real-time object detection and tracking with 90% accuracy in challenging industrial environments.',
    doi: '10.1109/ICCV.2022.654321',
    arxiv: 'https://arxiv.org/abs/2205.67890',
    citations: 56,
    type: 'conference',
    keywords: ['Computer Vision', 'Industrial Robotics', 'Object Detection', 'Real-Time']
  },
  {
    id: 'swarm-robotics-coordination',
    title: 'Decentralized Coordination Algorithms for Swarm Robotics',
    authors: ['Your Name', 'Prof. David Lee', 'Dr. Emily Davis'],
    venue: 'Autonomous Robots',
    year: '2021',
    abstract: 'Novel decentralized coordination algorithms that enable efficient collaboration among multiple robots without centralized control. Experimental validation with 20+ robots demonstrates improved scalability and robustness.',
    doi: '10.1007/s10514-021-09989-2',
    citations: 89,
    type: 'journal',
    keywords: ['Swarm Robotics', 'Decentralized Control', 'Coordination', 'Multi-Robot Systems']
  },
  {
    id: 'sensor-fusion-framework',
    title: 'A Unified Framework for Multi-Sensor Fusion in Autonomous Systems',
    authors: ['Your Name'],
    venue: 'IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)',
    year: '2021',
    abstract: 'A unified framework that integrates multiple sensor modalities for robust perception in autonomous systems. The framework provides a plug-and-play architecture for various sensor combinations.',
    doi: '10.1109/IROS.2021.234567',
    arxiv: 'https://arxiv.org/abs/2108.90123',
    citations: 67,
    type: 'conference',
    keywords: ['Sensor Fusion', 'Autonomous Systems', 'Perception', 'Multi-Modal']
  }
]

export function Research() {
  const [selectedType, setSelectedType] = useState<string>('all')
  const [copiedId, setCopiedId] = useState<string | null>(null)

  const publicationTypes = [
    { id: 'all', name: 'All Publications' },
    { id: 'conference', name: 'Conference Papers' },
    { id: 'journal', name: 'Journal Papers' },
    { id: 'workshop', name: 'Workshop Papers' },
    { id: 'preprint', name: 'Preprints' }
  ]

  const filteredPublications = selectedType === 'all' 
    ? publications 
    : publications.filter(pub => pub.type === selectedType)

  const copyCitation = async (publication: Publication) => {
    const citation = `${publication.authors.join(', ')}. "${publication.title}." ${publication.venue}, ${publication.year}.`
    await navigator.clipboard.writeText(citation)
    setCopiedId(publication.id)
    setTimeout(() => setCopiedId(null), 2000)
  }

  const getTypeColor = (type: string) => {
    switch (type) {
      case 'conference':
        return 'bg-primary text-background'
      case 'journal':
        return 'bg-accent text-background'
      case 'workshop':
        return 'bg-yellow-500 text-background'
      case 'preprint':
        return 'bg-foreground-secondary text-background'
      default:
        return 'bg-border text-foreground'
    }
  }

  return (
    <section id="research" className="py-20 bg-background-secondary">
      <div className="max-w-7xl mx-auto px-4 sm:px-6 lg:px-8">
        <motion.div
          initial={{ opacity: 0, y: 20 }}
          whileInView={{ opacity: 1, y: 0 }}
          transition={{ duration: 0.8 }}
          viewport={{ once: true }}
          className="text-center mb-16"
        >
          <h2 className="text-3xl md:text-4xl font-display font-bold mb-4">
            Research & Publications
          </h2>
          <p className="text-foreground-secondary text-lg max-w-2xl mx-auto">
            Academic contributions and research publications in robotics, 
            computer vision, and autonomous systems.
          </p>
        </motion.div>

        {/* Publication Stats */}
        <motion.div
          initial={{ opacity: 0, y: 20 }}
          whileInView={{ opacity: 1, y: 0 }}
          transition={{ duration: 0.8, delay: 0.2 }}
          viewport={{ once: true }}
          className="grid grid-cols-2 md:grid-cols-4 gap-6 mb-12"
        >
          {[
            { icon: <BookOpen size={24} />, number: publications.length, label: 'Publications' },
            { icon: <Award size={24} />, number: 1200, label: 'Total Citations' },
            { icon: <Users size={24} />, number: 15, label: 'Collaborators' },
            { icon: <Calendar size={24} />, number: 3, label: 'Years Active' }
          ].map((stat, index) => (
            <motion.div
              key={stat.label}
              initial={{ opacity: 0, scale: 0.8 }}
              whileInView={{ opacity: 1, scale: 1 }}
              transition={{ duration: 0.5, delay: index * 0.1 }}
              viewport={{ once: true }}
              className="text-center p-6 bg-background rounded-lg border border-border"
            >
              <div className="text-primary mb-2 flex justify-center">
                {stat.icon}
              </div>
              <div className="text-2xl font-bold text-foreground mb-1">
                {stat.number}
              </div>
              <div className="text-sm text-foreground-secondary">
                {stat.label}
              </div>
            </motion.div>
          ))}
        </motion.div>

        {/* Filter */}
        <motion.div
          initial={{ opacity: 0, y: 20 }}
          whileInView={{ opacity: 1, y: 0 }}
          transition={{ duration: 0.8, delay: 0.3 }}
          viewport={{ once: true }}
          className="flex flex-wrap justify-center gap-4 mb-12"
        >
          {publicationTypes.map((type) => (
            <button
              key={type.id}
              onClick={() => setSelectedType(type.id)}
              className={`px-6 py-2 rounded-full text-sm font-medium transition-colors ${
                selectedType === type.id
                  ? 'bg-primary text-background'
                  : 'bg-background text-foreground-secondary hover:text-foreground'
              }`}
            >
              {type.name}
            </button>
          ))}
        </motion.div>

        {/* Publications */}
        <div className="space-y-6">
          <AnimatePresence>
            {filteredPublications.map((publication, index) => (
              <motion.div
                key={publication.id}
                layout
                initial={{ opacity: 0, y: 20 }}
                animate={{ opacity: 1, y: 0 }}
                exit={{ opacity: 0, y: -20 }}
                transition={{ duration: 0.5, delay: index * 0.1 }}
                className="bg-background rounded-lg border border-border p-6 hover:border-primary/50 transition-colors"
              >
                <div className="flex flex-col lg:flex-row lg:items-start lg:justify-between gap-4">
                  <div className="flex-1">
                    {/* Header */}
                    <div className="flex items-start justify-between mb-3">
                      <div className="flex-1">
                        <h3 className="text-lg font-display font-bold text-foreground mb-2 hover:text-primary transition-colors">
                          {publication.title}
                        </h3>
                        <p className="text-sm text-foreground-secondary mb-2">
                          {publication.authors.join(', ')}
                        </p>
                        <p className="text-sm text-foreground-secondary">
                          {publication.venue}, {publication.year}
                        </p>
                      </div>
                      <div className="flex items-center space-x-2">
                        <span className={`px-3 py-1 rounded-full text-xs font-medium ${getTypeColor(publication.type)}`}>
                          {publication.type.toUpperCase()}
                        </span>
                        <span className="text-sm text-foreground-secondary">
                          {publication.citations} citations
                        </span>
                      </div>
                    </div>

                    {/* Abstract */}
                    <p className="text-foreground-secondary text-sm mb-4 line-clamp-3">
                      {publication.abstract}
                    </p>

                    {/* Keywords */}
                    <div className="flex flex-wrap gap-2 mb-4">
                      {publication.keywords.map((keyword) => (
                        <span key={keyword} className="skill-badge text-xs">
                          {keyword}
                        </span>
                      ))}
                    </div>
                  </div>

                  {/* Actions */}
                  <div className="flex flex-col space-y-2 lg:ml-4">
                    {publication.doi && (
                      <motion.a
                        href={`https://doi.org/${publication.doi}`}
                        target="_blank"
                        rel="noopener noreferrer"
                        whileHover={{ scale: 1.05 }}
                        whileTap={{ scale: 0.95 }}
                        className="flex items-center space-x-2 px-4 py-2 bg-primary text-background rounded-lg text-sm font-medium hover:bg-primary-hover transition-colors"
                      >
                        <ExternalLink size={16} />
                        <span>View Paper</span>
                      </motion.a>
                    )}
                    
                    {publication.arxiv && (
                      <motion.a
                        href={publication.arxiv}
                        target="_blank"
                        rel="noopener noreferrer"
                        whileHover={{ scale: 1.05 }}
                        whileTap={{ scale: 0.95 }}
                        className="flex items-center space-x-2 px-4 py-2 border border-primary text-primary rounded-lg text-sm font-medium hover:bg-primary/10 transition-colors"
                      >
                        <BookOpen size={16} />
                        <span>arXiv</span>
                      </motion.a>
                    )}
                    
                    {publication.pdf && (
                      <motion.a
                        href={publication.pdf}
                        target="_blank"
                        rel="noopener noreferrer"
                        whileHover={{ scale: 1.05 }}
                        whileTap={{ scale: 0.95 }}
                        className="flex items-center space-x-2 px-4 py-2 border border-border text-foreground-secondary rounded-lg text-sm font-medium hover:text-foreground transition-colors"
                      >
                        <Download size={16} />
                        <span>PDF</span>
                      </motion.a>
                    )}
                    
                    <motion.button
                      onClick={() => copyCitation(publication)}
                      whileHover={{ scale: 1.05 }}
                      whileTap={{ scale: 0.95 }}
                      className="flex items-center space-x-2 px-4 py-2 border border-border text-foreground-secondary rounded-lg text-sm font-medium hover:text-foreground transition-colors"
                    >
                      {copiedId === publication.id ? (
                        <>
                          <Check size={16} />
                          <span>Copied!</span>
                        </>
                      ) : (
                        <>
                          <Copy size={16} />
                          <span>Cite</span>
                        </>
                      )}
                    </motion.button>
                  </div>
                </div>
              </motion.div>
            ))}
          </AnimatePresence>
        </div>

        {/* Research Areas */}
        <motion.div
          initial={{ opacity: 0, y: 20 }}
          whileInView={{ opacity: 1, y: 0 }}
          transition={{ duration: 0.8, delay: 0.5 }}
          viewport={{ once: true }}
          className="mt-20"
        >
          <h3 className="text-2xl font-display font-bold text-center mb-12">
            Research Areas
          </h3>
          
          <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-6">
            {[
              {
                title: 'SLAM & Navigation',
                description: 'Simultaneous Localization and Mapping algorithms for autonomous robots in dynamic environments.',
                keywords: ['SLAM', 'Navigation', 'Localization', 'Mapping']
              },
              {
                title: 'Computer Vision',
                description: 'Real-time object detection, tracking, and scene understanding for robotic applications.',
                keywords: ['Computer Vision', 'Object Detection', 'Tracking', 'Deep Learning']
              },
              {
                title: 'Reinforcement Learning',
                description: 'Deep reinforcement learning for robotic manipulation and control tasks.',
                keywords: ['RL', 'Manipulation', 'Control', 'Policy Learning']
              },
              {
                title: 'Multi-Robot Systems',
                description: 'Decentralized coordination and swarm robotics for collaborative tasks.',
                keywords: ['Swarm Robotics', 'Coordination', 'Multi-Agent', 'Distributed Control']
              },
              {
                title: 'Real-Time Systems',
                description: 'High-frequency control systems and real-time optimization for robotics.',
                keywords: ['Real-Time', 'Control', 'Optimization', 'Performance']
              },
              {
                title: 'Sensor Fusion',
                description: 'Multi-sensor fusion frameworks for robust perception in challenging environments.',
                keywords: ['Sensor Fusion', 'Multi-Modal', 'Perception', 'Robustness']
              }
            ].map((area, index) => (
              <motion.div
                key={area.title}
                initial={{ opacity: 0, scale: 0.8 }}
                whileInView={{ opacity: 1, scale: 1 }}
                transition={{ duration: 0.5, delay: index * 0.1 }}
                viewport={{ once: true }}
                className="p-6 bg-background rounded-lg border border-border hover:border-primary/50 transition-colors"
              >
                <h4 className="text-lg font-display font-bold text-foreground mb-3">
                  {area.title}
                </h4>
                <p className="text-foreground-secondary text-sm mb-4">
                  {area.description}
                </p>
                <div className="flex flex-wrap gap-2">
                  {area.keywords.map((keyword) => (
                    <span key={keyword} className="skill-badge text-xs">
                      {keyword}
                    </span>
                  ))}
                </div>
              </motion.div>
            ))}
          </div>
        </motion.div>
      </div>
    </section>
  )
} 
