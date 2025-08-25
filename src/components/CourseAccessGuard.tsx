'use client'

import { useState, useEffect } from 'react'
import { useAuth } from '@/components/AuthProvider'
import { hasUserCourseAccess } from '@/lib/course-access'
import Link from 'next/link'

interface CourseAccessGuardProps {
  children: React.ReactNode
  fallback?: React.ReactNode
}

export default function CourseAccessGuard({ children, fallback }: CourseAccessGuardProps) {
  const { user } = useAuth()
  const [hasAccess, setHasAccess] = useState<boolean | null>(null)
  const [loading, setLoading] = useState(true)

  useEffect(() => {
    async function checkAccess() {
      if (!user?.email) {
        setHasAccess(false)
        setLoading(false)
        return
      }

      try {
        const access = await hasUserCourseAccess(user.email)
        setHasAccess(access)
      } catch (error) {
        console.error('Error checking course access:', error)
        setHasAccess(false)
      } finally {
        setLoading(false)
      }
    }

    checkAccess()
  }, [user?.email])

  if (loading) {
    return (
      <div className="min-h-screen flex items-center justify-center bg-gray-50">
        <div className="text-center">
          <div className="animate-spin rounded-full h-12 w-12 border-b-2 border-blue-600 mx-auto"></div>
          <p className="mt-4 text-gray-600">Checking course access...</p>
        </div>
      </div>
    )
  }

  if (!user) {
    return (
      <div className="min-h-screen flex items-center justify-center bg-gray-50 py-12 px-4 sm:px-6 lg:px-8">
        <div className="max-w-md w-full space-y-8 text-center">
          <div>
            <h2 className="mt-6 text-3xl font-extrabold text-gray-900">
              🔐 Login Required
            </h2>
            <p className="mt-2 text-sm text-gray-600">
              You need to be logged in to access course content.
            </p>
          </div>
          <div className="space-y-4">
            <Link
              href="/login"
              className="w-full flex justify-center py-2 px-4 border border-transparent rounded-md shadow-sm text-sm font-medium text-white bg-blue-600 hover:bg-blue-700 focus:outline-none focus:ring-2 focus:ring-offset-2 focus:ring-blue-500"
            >
              Login to Your Account
            </Link>
            <Link
              href="/signup"
              className="w-full flex justify-center py-2 px-4 border border-gray-300 rounded-md shadow-sm text-sm font-medium text-gray-700 bg-white hover:bg-gray-50 focus:outline-none focus:ring-2 focus:ring-offset-2 focus:ring-blue-500"
            >
              Create New Account
            </Link>
          </div>
        </div>
      </div>
    )
  }

  if (!hasAccess) {
    if (fallback) {
      return <>{fallback}</>
    }

    return (
      <div className="min-h-screen flex items-center justify-center bg-gray-50 py-12 px-4 sm:px-6 lg:px-8">
        <div className="max-w-md w-full space-y-8">
          <div className="text-center">
            <div className="mx-auto flex items-center justify-center h-12 w-12 rounded-full bg-yellow-100">
              <svg className="h-6 w-6 text-yellow-600" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M12 9v2m0 4h.01m-6.938 4h13.856c1.54 0 2.502-1.667 1.732-2.5L13.732 4c-.77-.833-1.964-.833-2.732 0L4.082 15.5c-.77.833.192 2.5 1.732 2.5z" />
              </svg>
            </div>
            <h2 className="mt-6 text-3xl font-extrabold text-gray-900">
              🎓 Course Access Required
            </h2>
            <p className="mt-2 text-sm text-gray-600">
              Hi <strong>{user.email}</strong>! You need to purchase the &quot;Become A Roboticist&quot; course to access this content.
            </p>
          </div>
          
          <div className="bg-blue-50 border border-blue-200 rounded-md p-4">
            <h3 className="text-sm font-medium text-blue-800 mb-2">What you&apos;ll get:</h3>
            <ul className="text-sm text-blue-700 space-y-1">
              <li>• 28-day robotics challenge course</li>
              <li>• Step-by-step video tutorials</li>
              <li>• Bonus advanced content</li>
              <li>• Community Discord access</li>
            </ul>
          </div>
          
          <div className="text-center space-y-3">
            <p className="text-sm text-gray-500">
              Already purchased? Contact support if you believe this is an error.
            </p>
            <Link
              href="/"
              className="block w-full bg-blue-600 hover:bg-blue-700 text-white font-medium py-2 px-4 rounded-md transition-colors"
            >
              Learn More About The Course
            </Link>
          </div>
        </div>
      </div>
    )
  }

  return <>{children}</>
}