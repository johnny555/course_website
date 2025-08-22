'use client'

import { useEffect, useState } from 'react'
import { useParams } from 'next/navigation'
import ProtectedRoute from '@/components/ProtectedRoute'
import ModuleForm from '@/components/ModuleForm'
import { getModule } from '@/lib/modules'
import { Module } from '@/types'

export default function EditModulePage() {
  const params = useParams()
  const [module, setModule] = useState<Module | null>(null)
  const [loading, setLoading] = useState(true)
  const [error, setError] = useState('')

  useEffect(() => {
    async function fetchModule() {
      if (!params.id || typeof params.id !== 'string') {
        setError('Invalid module ID')
        setLoading(false)
        return
      }

      try {
        const data = await getModule(params.id)
        if (data) {
          setModule(data)
        } else {
          setError('Module not found')
        }
      } catch (error) {
        console.error('Error fetching module:', error)
        setError('Failed to load module')
      } finally {
        setLoading(false)
      }
    }

    fetchModule()
  }, [params.id])

  if (loading) {
    return (
      <ProtectedRoute adminOnly>
        <div className="min-h-screen flex items-center justify-center">
          <div className="animate-spin rounded-full h-32 w-32 border-b-2 border-blue-600"></div>
        </div>
      </ProtectedRoute>
    )
  }

  if (error || !module) {
    return (
      <ProtectedRoute adminOnly>
        <div className="min-h-screen flex items-center justify-center">
          <div className="text-center">
            <h1 className="text-2xl font-bold text-gray-900 mb-4">Error</h1>
            <p className="text-gray-600">{error || 'Module not found'}</p>
          </div>
        </div>
      </ProtectedRoute>
    )
  }

  return (
    <ProtectedRoute adminOnly>
      <ModuleForm module={module} isEdit />
    </ProtectedRoute>
  )
}