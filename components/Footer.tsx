'use client'

import { motion } from 'framer-motion'
import { Github, Linkedin, Twitter, Mail, Heart, ArrowUp } from 'lucide-react'

export function Footer() {
  const scrollToTop = () => {
    window.scrollTo({ top: 0, behavior: 'smooth' })
  }

  return (
    <footer className="bg-background-secondary border-t border-border">
      <div className="max-w-7xl mx-auto px-4 sm:px-6 lg:px-8 py-12">
        <div className="grid grid-cols-1 md:grid-cols-4 gap-8">
          {/* Brand */}
          <div className="md:col-span-2">
            <div className="flex items-center space-x-2 mb-4">
              <div className="w-8 h-8 bg-gradient-to-r from-primary to-accent rounded-lg flex items-center justify-center">
                <span className="text-background font-bold text-sm">R</span>
              </div>
              <span className="font-display font-bold text-lg gradient-text">
                Robotics Engineer
              </span>
            </div>
            <p className="text-foreground-secondary mb-6 max-w-md">
              Building the future of autonomous systems through innovative robotics software engineering. 
              Specializing in SLAM, computer vision, and machine learning.
            </p>
            <div className="flex space-x-4">
              <motion.a
                href="https://github.com"
                target="_blank"
                rel="noopener noreferrer"
                whileHover={{ scale: 1.1, y: -2 }}
                className="w-10 h-10 bg-background rounded-lg flex items-center justify-center text-foreground-secondary hover:text-foreground transition-colors"
              >
                <Github size={20} />
              </motion.a>
              <motion.a
                href="https://linkedin.com"
                target="_blank"
                rel="noopener noreferrer"
                whileHover={{ scale: 1.1, y: -2 }}
                className="w-10 h-10 bg-background rounded-lg flex items-center justify-center text-foreground-secondary hover:text-foreground transition-colors"
              >
                <Linkedin size={20} />
              </motion.a>
              <motion.a
                href="https://twitter.com"
                target="_blank"
                rel="noopener noreferrer"
                whileHover={{ scale: 1.1, y: -2 }}
                className="w-10 h-10 bg-background rounded-lg flex items-center justify-center text-foreground-secondary hover:text-foreground transition-colors"
              >
                <Twitter size={20} />
              </motion.a>
              <motion.a
                href="mailto:hello@roboticsengineer.com"
                whileHover={{ scale: 1.1, y: -2 }}
                className="w-10 h-10 bg-background rounded-lg flex items-center justify-center text-foreground-secondary hover:text-foreground transition-colors"
              >
                <Mail size={20} />
              </motion.a>
            </div>
          </div>

          {/* Quick Links */}
          <div>
            <h3 className="font-display font-bold text-foreground mb-4">Quick Links</h3>
            <ul className="space-y-2">
              {[
                { name: 'About', href: '#about' },
                { name: 'Projects', href: '#projects' },
                { name: 'Skills', href: '#skills' },
                { name: 'Experience', href: '#experience' },
                { name: 'Research', href: '#research' },
                { name: 'Contact', href: '#contact' }
              ].map((link) => (
                <li key={link.name}>
                  <a
                    href={link.href}
                    className="text-foreground-secondary hover:text-foreground transition-colors text-sm"
                  >
                    {link.name}
                  </a>
                </li>
              ))}
            </ul>
          </div>

          {/* Resources */}
          <div>
            <h3 className="font-display font-bold text-foreground mb-4">Resources</h3>
            <ul className="space-y-2">
              <li>
                <a
                  href="/cv.pdf"
                  target="_blank"
                  rel="noopener noreferrer"
                  className="text-foreground-secondary hover:text-foreground transition-colors text-sm"
                >
                  Download CV
                </a>
              </li>
              <li>
                <a
                  href="https://github.com"
                  target="_blank"
                  rel="noopener noreferrer"
                  className="text-foreground-secondary hover:text-foreground transition-colors text-sm"
                >
                  GitHub
                </a>
              </li>
              <li>
                <a
                  href="https://linkedin.com"
                  target="_blank"
                  rel="noopener noreferrer"
                  className="text-foreground-secondary hover:text-foreground transition-colors text-sm"
                >
                  LinkedIn
                </a>
              </li>
              <li>
                <a
                  href="https://scholar.google.com"
                  target="_blank"
                  rel="noopener noreferrer"
                  className="text-foreground-secondary hover:text-foreground transition-colors text-sm"
                >
                  Google Scholar
                </a>
              </li>
              <li>
                <a
                  href="https://researchgate.net"
                  target="_blank"
                  rel="noopener noreferrer"
                  className="text-foreground-secondary hover:text-foreground transition-colors text-sm"
                >
                  ResearchGate
                </a>
              </li>
            </ul>
          </div>
        </div>

        {/* Bottom Section */}
        <div className="border-t border-border mt-12 pt-8 flex flex-col md:flex-row items-center justify-between">
          <div className="flex items-center space-x-2 text-foreground-secondary text-sm mb-4 md:mb-0">
            <span>© 2024 Robotics Engineer. Made with</span>
            <Heart size={16} className="text-red-500" />
            <span>using Next.js & Tailwind CSS</span>
          </div>
          
          <motion.button
            onClick={scrollToTop}
            whileHover={{ scale: 1.1 }}
            whileTap={{ scale: 0.9 }}
            className="w-10 h-10 bg-background rounded-lg flex items-center justify-center text-foreground-secondary hover:text-foreground transition-colors"
          >
            <ArrowUp size={20} />
          </motion.button>
        </div>
      </div>
    </footer>
  )
}
