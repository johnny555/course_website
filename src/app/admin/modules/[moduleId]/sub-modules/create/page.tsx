'use client'

import { useParams } from 'next/navigation'
import ProtectedRoute from '@/components/ProtectedRoute'
import SubModuleForm from '@/components/SubModuleForm'

export default function CreateSubModulePage() {
  const params = useParams()

  if (!params.moduleId || typeof params.moduleId !== 'string') {
    return (
      <ProtectedRoute adminOnly>
        <div className="min-h-screen flex items-center justify-center">
          <div className="text-center">
            <h1 className="text-2xl font-bold text-gray-900 mb-4">Invalid Module</h1>
            <p className="text-gray-600">Cannot create lesson without valid module ID</p>
          </div>
        </div>
      </ProtectedRoute>
    )
  }

  return (
    <ProtectedRoute adminOnly>
      <SubModuleForm moduleId={params.moduleId} />
    </ProtectedRoute>
  )
}