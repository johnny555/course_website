'use client'

import { useState } from 'react'
import { useRouter } from 'next/navigation'
import Link from 'next/link'
import { signUp } from '@/lib/auth'
import { useAuth } from '@/components/AuthProvider'
import { isEmailApproved } from '@/lib/approved-users'

export default function SignUpPage() {
  const [email, setEmail] = useState('')
  const [password, setPassword] = useState('')
  const [confirmPassword, setConfirmPassword] = useState('')
  const [loading, setLoading] = useState(false)
  const [error, setError] = useState('')
  const [success, setSuccess] = useState(false)
  const [checkingApproval, setCheckingApproval] = useState(false)
  const router = useRouter()
  const { user } = useAuth()

  // Redirect if already logged in
  if (user) {
    if (user.profile?.role === 'admin') {
      router.push('/admin')
    } else {
      router.push('/modules')
    }
    return null
  }

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault()
    setLoading(true)
    setError('')

    if (password !== confirmPassword) {
      setError('Passwords do not match')
      setLoading(false)
      return
    }

    if (password.length < 6) {
      setError('Password must be at least 6 characters')
      setLoading(false)
      return
    }

    // Check if email is approved
    setCheckingApproval(true)
    const approved = await isEmailApproved(email)
    setCheckingApproval(false)
    
    if (!approved) {
      setError('❌ Email not approved: This email address is not on our approved list. Only students who have purchased the "Become A Roboticist" course can create an account. Please check your email address or contact support if you believe this is an error.')
      setLoading(false)
      return
    }

    const { data, error } = await signUp(email, password)

    if (error) {
      if (error.message.includes('User already registered')) {
        setError('⚠️ Account already exists: This email is already registered. Please try logging in instead.')
      } else if (error.message.includes('Invalid email')) {
        setError('❌ Invalid email: Please enter a valid email address.')
      } else {
        setError(`❌ Signup failed: ${error.message}`)
      }
    } else {
      setSuccess(true)
    }
    
    setLoading(false)
  }

  if (success) {
    return (
      <div className="min-h-screen flex items-center justify-center bg-gray-50 py-12 px-4 sm:px-6 lg:px-8">
        <div className="max-w-md w-full space-y-8">
          <div className="text-center">
            <div className="mx-auto flex items-center justify-center h-12 w-12 rounded-full bg-green-100">
              <svg className="h-6 w-6 text-green-600" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M5 13l4 4L19 7" />
              </svg>
            </div>
            <h2 className="mt-6 text-3xl font-extrabold text-gray-900">
              ✅ Account Created!
            </h2>
            <p className="mt-2 text-sm text-gray-600">
              We&apos;ve sent a confirmation email to:
            </p>
            <p className="mt-1 text-sm font-medium text-gray-900">
              {email}
            </p>
            <div className="mt-4 p-4 bg-blue-50 border border-blue-200 rounded-md">
              <p className="text-sm text-blue-800 font-medium">📧 Next Steps:</p>
              <ul className="mt-2 text-sm text-blue-700 text-left space-y-1">
                <li>1. Check your email inbox (and spam folder)</li>
                <li>2. Click the confirmation link in the email</li>
                <li>3. Once confirmed, you can log in to access the course</li>
              </ul>
            </div>
            <div className="mt-6 space-y-3">
              <Link
                href="/login"
                className="block w-full bg-blue-600 hover:bg-blue-700 text-white font-medium py-2 px-4 rounded-md transition-colors"
              >
                Go to Login Page
              </Link>
              <button
                onClick={() => setSuccess(false)}
                className="block w-full bg-gray-200 hover:bg-gray-300 text-gray-800 font-medium py-2 px-4 rounded-md transition-colors"
              >
                Sign Up Another Account
              </button>
            </div>
          </div>
        </div>
      </div>
    )
  }

  return (
    <div className="min-h-screen flex items-center justify-center bg-gray-50 py-12 px-4 sm:px-6 lg:px-8">
      <div className="max-w-md w-full space-y-8">
        <div>
          <h2 className="mt-6 text-center text-3xl font-extrabold text-gray-900">
            Join Become A Roboticist
          </h2>
          <div className="mt-4 p-4 bg-blue-50 border border-blue-200 rounded-md">
            <p className="text-sm text-blue-800 font-medium">Course Access Required</p>
            <p className="mt-1 text-sm text-blue-700">
              Only students who have purchased the &quot;Become A Roboticist&quot; course can create an account. 
              Your email must be on the approved list.
            </p>
          </div>
          <p className="mt-2 text-center text-sm text-gray-600">
            Already have an account?{' '}
            <Link
              href="/login"
              className="font-medium text-blue-600 hover:text-blue-500"
            >
              Sign in here
            </Link>
          </p>
        </div>
        
        <div className="bg-gray-50 border border-gray-200 rounded-md p-4">
          <p className="text-sm text-gray-700 font-medium">📧 Email Confirmation Required</p>
          <p className="mt-1 text-xs text-gray-600">
            After creating your account, you&apos;ll receive a confirmation email. Click the link to activate your account and access the course.
          </p>
        </div>
        
        <form className="mt-6 space-y-6" onSubmit={handleSubmit}>
          <div className="space-y-4">
            <div>
              <label htmlFor="email" className="block text-sm font-medium text-gray-700">
                Email address
              </label>
              <input
                id="email"
                name="email"
                type="email"
                autoComplete="email"
                required
                value={email}
                onChange={(e) => setEmail(e.target.value)}
                className="mt-1 appearance-none relative block w-full px-3 py-2 border border-gray-300 placeholder-gray-500 text-gray-900 rounded-md focus:outline-none focus:ring-blue-500 focus:border-blue-500 focus:z-10 sm:text-sm"
                placeholder="Email address"
              />
            </div>
            <div>
              <label htmlFor="password" className="block text-sm font-medium text-gray-700">
                Password
              </label>
              <input
                id="password"
                name="password"
                type="password"
                autoComplete="new-password"
                required
                value={password}
                onChange={(e) => setPassword(e.target.value)}
                className="mt-1 appearance-none relative block w-full px-3 py-2 border border-gray-300 placeholder-gray-500 text-gray-900 rounded-md focus:outline-none focus:ring-blue-500 focus:border-blue-500 focus:z-10 sm:text-sm"
                placeholder="Password (min. 6 characters)"
              />
            </div>
            <div>
              <label htmlFor="confirmPassword" className="block text-sm font-medium text-gray-700">
                Confirm Password
              </label>
              <input
                id="confirmPassword"
                name="confirmPassword"
                type="password"
                autoComplete="new-password"
                required
                value={confirmPassword}
                onChange={(e) => setConfirmPassword(e.target.value)}
                className="mt-1 appearance-none relative block w-full px-3 py-2 border border-gray-300 placeholder-gray-500 text-gray-900 rounded-md focus:outline-none focus:ring-blue-500 focus:border-blue-500 focus:z-10 sm:text-sm"
                placeholder="Confirm password"
              />
            </div>
          </div>

          {error && (
            <div className="bg-red-50 border border-red-200 rounded-md p-4">
              <div className="text-red-800 text-sm">{error}</div>
              {error.includes('not approved') && (
                <div className="mt-2 text-xs text-red-600">
                  💡 Need help? Contact support or check if you used the same email address when purchasing the course.
                </div>
              )}
            </div>
          )}

          <div>
            <button
              type="submit"
              disabled={loading}
              className="group relative w-full flex justify-center py-2 px-4 border border-transparent text-sm font-medium rounded-md text-white bg-blue-600 hover:bg-blue-700 focus:outline-none focus:ring-2 focus:ring-offset-2 focus:ring-blue-500 disabled:opacity-50"
            >
              {checkingApproval ? 'Checking approval...' : loading ? 'Creating account...' : 'Create Account'}
            </button>
          </div>
        </form>
      </div>
    </div>
  )
}