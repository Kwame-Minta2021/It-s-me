import type { Metadata } from 'next'
import './globals.css'

export const metadata: Metadata = {
  metadataBase: new URL('http://localhost:3001'),
  title: 'Frederick Kwame Minta — Robotics Software Engineer (ROS2 • AI • IoT)',
  description: 'Portfolio of Frederick Kwame Minta, robotics software engineer specializing in ROS2, AI-driven control, and sensor-rich systems. Projects in assistive robotics, precision agriculture, and IoT platforms; publications and teaching experience.',
  keywords: ['Frederick Kwame Minta', 'robotics', 'software engineer', 'ROS2', 'C++', 'Python', 'SLAM', 'control systems', 'machine learning', 'autonomous systems', 'Computer Science', 'Engineering', 'IoT', 'assistive robotics', 'precision agriculture'],
  authors: [{ name: 'Frederick Kwame Minta' }],
  openGraph: {
    title: 'Frederick Kwame Minta — Robotics Software Engineer (ROS2 • AI • IoT)',
    description: 'Portfolio of Frederick Kwame Minta, robotics software engineer specializing in ROS2, AI-driven control, and sensor-rich systems.',
    type: 'website',
    locale: 'en_US',
  },
  twitter: {
    card: 'summary_large_image',
    title: 'Frederick Kwame Minta — Robotics Software Engineer (ROS2 • AI • IoT)',
    description: 'Portfolio of Frederick Kwame Minta, robotics software engineer specializing in ROS2, AI-driven control, and sensor-rich systems.',
  },
  robots: {
    index: true,
    follow: true,
  },
}

export default function RootLayout({
  children,
}: {
  children: React.ReactNode
}) {
  return (
    <html lang="en">
      <body className="bg-background text-foreground antialiased">
        {children}
      </body>
    </html>
  )
} 
