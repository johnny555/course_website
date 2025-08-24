'use client'

import { useEffect, useState } from 'react'
import { useParams } from 'next/navigation'
import ProtectedRoute from '@/components/ProtectedRoute'
import SubModuleForm from '@/components/SubModuleForm'
import { getSubModule } from '@/lib/modules'
import { SubModule } from '@/types'

export default function EditSubModulePage() {
  const params = useParams()
  const [subModule, setSubModule] = useState<SubModule | null>(null)
  const [loading, setLoading] = useState(true)
  const [error, setError] = useState('')

  useEffect(() => {
    async function fetchSubModule() {
      if (!params.moduleId || !params.subModuleId || 
          typeof params.moduleId !== 'string' || typeof params.subModuleId !== 'string') {
        setError('Invalid module or sub-module ID')
        setLoading(false)
        return
      }

      try {
        const data = await getSubModule(params.subModuleId)
        if (data) {
          setSubModule(data)
        } else {
          setError('Lesson not found')
        }
      } catch (error) {
        console.error('Error fetching sub-module:', error)
        setError('Failed to load lesson')
      } finally {
        setLoading(false)
      }
    }

    fetchSubModule()
  }, [params.moduleId, params.subModuleId])

  if (loading) {
    return (
      <ProtectedRoute adminOnly>
        <div className="min-h-screen flex items-center justify-center">
          <div className="animate-spin rounded-full h-32 w-32 border-b-2 border-blue-600"></div>
        </div>
      </ProtectedRoute>
    )
  }

  if (error || !subModule || !params.moduleId || typeof params.moduleId !== 'string') {
    return (
      <ProtectedRoute adminOnly>
        <div className="min-h-screen flex items-center justify-center">
          <div className="text-center">
            <h1 className="text-2xl font-bold text-gray-900 mb-4">Error</h1>
            <p className="text-gray-600">{error || 'Lesson not found'}</p>
          </div>
        </div>
      </ProtectedRoute>
    )
  }

  return (
    <ProtectedRoute adminOnly>
      <SubModuleForm moduleId={params.moduleId} subModule={subModule} isEdit />
    </ProtectedRoute>
  )
}