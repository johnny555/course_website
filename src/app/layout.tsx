import type { Metadata } from 'next'
import { Inter } from 'next/font/google'
import './globals.css'
import { AuthProvider } from '@/components/AuthProvider'
import Navigation from '@/components/Navigation'
import DemoBanner from '@/components/DemoBanner'

const inter = Inter({ subsets: ['latin'] })

export const metadata: Metadata = {
  title: 'Robotics Course',
  description: 'Learn robotics with hands-on projects',
}

export default function RootLayout({
  children,
}: {
  children: React.ReactNode
}) {
  return (
    <html lang="en">
      <body className={inter.className}>
        <AuthProvider>
          <DemoBanner />
          <Navigation />
          {children}
        </AuthProvider>
      </body>
    </html>
  )
}