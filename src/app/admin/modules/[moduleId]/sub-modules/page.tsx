'use client'

import { useEffect, useState, useCallback } from 'react'
import { useParams, useRouter } from 'next/navigation'
import Link from 'next/link'
import Image from 'next/image'
import ProtectedRoute from '@/components/ProtectedRoute'
import { getModuleWithSubModules, deleteSubModule } from '@/lib/modules'
import { ModuleWithSubModules } from '@/types'

export default function SubModulesAdminPage() {
  const params = useParams()
  const router = useRouter()
  const [module, setModule] = useState<ModuleWithSubModules | null>(null)
  const [loading, setLoading] = useState(true)
  const [deleting, setDeleting] = useState<string | null>(null)

  const fetchModule = useCallback(async () => {
    if (!params.moduleId || typeof params.moduleId !== 'string') {
      router.push('/admin')
      return
    }

    try {
      const data = await getModuleWithSubModules(params.moduleId)
      if (data) {
        setModule(data)
      } else {
        router.push('/admin')
      }
    } catch (error) {
      console.error('Error fetching module:', error)
      router.push('/admin')
    } finally {
      setLoading(false)
    }
  }, [params.moduleId, router])

  useEffect(() => {
    fetchModule()
  }, [fetchModule])

  const handleDelete = async (id: string) => {
    if (!confirm('Are you sure you want to delete this lesson?')) {
      return
    }

    setDeleting(id)
    try {
      const success = await deleteSubModule(id)
      if (success && module) {
        setModule({
          ...module,
          sub_modules: module.sub_modules.filter(sub => sub.id !== id)
        })
      } else {
        alert('Failed to delete lesson')
      }
    } catch (error) {
      console.error('Error deleting sub-module:', error)
      alert('Failed to delete lesson')
    } finally {
      setDeleting(null)
    }
  }

  if (loading) {
    return (
      <ProtectedRoute adminOnly>
        <div className="min-h-screen flex items-center justify-center">
          <div className="animate-spin rounded-full h-32 w-32 border-b-2 border-blue-600"></div>
        </div>
      </ProtectedRoute>
    )
  }

  if (!module) {
    return (
      <ProtectedRoute adminOnly>
        <div className="min-h-screen flex items-center justify-center">
          <div className="text-center">
            <h1 className="text-2xl font-bold text-gray-900 mb-4">Module Not Found</h1>
            <Link href="/admin" className="text-blue-600 hover:text-blue-700">
              Back to Admin Dashboard
            </Link>
          </div>
        </div>
      </ProtectedRoute>
    )
  }

  return (
    <ProtectedRoute adminOnly>
      <div className="min-h-screen bg-gray-50 py-8">
        <div className="container mx-auto px-4">
          <div className="mb-8">
            <Link
              href="/admin"
              className="flex items-center text-blue-600 hover:text-blue-700 transition-colors mb-4"
            >
              <svg className="w-5 h-5 mr-2" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M15 19l-7-7 7-7" />
              </svg>
              Back to Admin Dashboard
            </Link>
            
            <div className="flex justify-between items-start">
              <div>
                <h1 className="text-3xl font-bold text-gray-900 mb-2">Manage Lessons</h1>
                <p className="text-gray-600 mb-2">Module: <strong>{module.title}</strong></p>
                <p className="text-gray-500">{module.description}</p>
              </div>
              <Link
                href={`/admin/modules/${module.id}/sub-modules/create`}
                className="bg-blue-600 hover:bg-blue-700 text-white px-6 py-3 rounded-lg font-semibold transition-colors"
              >
                Add New Lesson
              </Link>
            </div>
          </div>

          <div className="bg-white rounded-lg shadow-sm overflow-hidden">
            <div className="px-6 py-4 border-b border-gray-200">
              <h2 className="text-lg font-semibold text-gray-900">
                Lessons ({module.sub_modules.length})
              </h2>
            </div>

            {module.sub_modules.length === 0 ? (
              <div className="text-center py-12">
                <div className="text-6xl mb-4">📚</div>
                <h3 className="text-lg font-semibold text-gray-900 mb-2">No lessons yet</h3>
                <p className="text-gray-600 mb-4">Create your first lesson to get started</p>
                <Link
                  href={`/admin/modules/${module.id}/sub-modules/create`}
                  className="bg-blue-600 hover:bg-blue-700 text-white px-6 py-2 rounded transition-colors"
                >
                  Create Lesson
                </Link>
              </div>
            ) : (
              <div className="overflow-x-auto">
                <table className="min-w-full divide-y divide-gray-200">
                  <thead className="bg-gray-50">
                    <tr>
                      <th className="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider">
                        Lesson
                      </th>
                      <th className="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider">
                        Order
                      </th>
                      <th className="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider">
                        Videos
                      </th>
                      <th className="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider">
                        Created
                      </th>
                      <th className="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider">
                        Actions
                      </th>
                    </tr>
                  </thead>
                  <tbody className="bg-white divide-y divide-gray-200">
                    {module.sub_modules.map((subModule) => (
                      <tr key={subModule.id} className="hover:bg-gray-50">
                        <td className="px-6 py-4">
                          <div className="flex items-center">
                            <div className="flex-shrink-0 h-12 w-12">
                              {subModule.youtube_urls[0] ? (
                                <Image
                                  src={`https://img.youtube.com/vi/${extractYouTubeVideoId(subModule.youtube_urls[0]) || ''}/default.jpg`}
                                  alt={subModule.title}
                                  width={48}
                                  height={48}
                                  className="rounded object-cover"
                                />
                              ) : (
                                <div className="h-12 w-12 bg-gray-200 rounded flex items-center justify-center">
                                  <span className="text-gray-400 text-xs">🎥</span>
                                </div>
                              )}
                            </div>
                            <div className="ml-4">
                              <div className="text-sm font-medium text-gray-900">
                                {subModule.title}
                              </div>
                              <div className="text-sm text-gray-500 max-w-xs truncate">
                                {subModule.description}
                              </div>
                            </div>
                          </div>
                        </td>
                        <td className="px-6 py-4 whitespace-nowrap text-sm text-gray-900">
                          {subModule.order}
                        </td>
                        <td className="px-6 py-4 whitespace-nowrap text-sm text-gray-900">
                          {subModule.youtube_urls.length}
                        </td>
                        <td className="px-6 py-4 whitespace-nowrap text-sm text-gray-500">
                          {new Date(subModule.created_at).toLocaleDateString()}
                        </td>
                        <td className="px-6 py-4 whitespace-nowrap text-sm font-medium space-x-2">
                          <Link
                            href={`/modules/${module.id}/${subModule.id}`}
                            className="text-blue-600 hover:text-blue-900 transition-colors"
                          >
                            View
                          </Link>
                          <Link
                            href={`/admin/modules/${module.id}/sub-modules/${subModule.id}/edit`}
                            className="text-yellow-600 hover:text-yellow-900 transition-colors"
                          >
                            Edit
                          </Link>
                          <button
                            onClick={() => handleDelete(subModule.id)}
                            disabled={deleting === subModule.id}
                            className="text-red-600 hover:text-red-900 transition-colors disabled:opacity-50"
                          >
                            {deleting === subModule.id ? 'Deleting...' : 'Delete'}
                          </button>
                        </td>
                      </tr>
                    ))}
                  </tbody>
                </table>
              </div>
            )}
          </div>
        </div>
      </div>
    </ProtectedRoute>
  )
}

function extractYouTubeVideoId(url: string): string | null {
  const patterns = [
    /(?:https?:\/\/)?(?:www\.)?youtube\.com\/watch\?v=([^&\n?#]+)/,
    /(?:https?:\/\/)?(?:www\.)?youtube\.com\/embed\/([^&\n?#]+)/,
    /(?:https?:\/\/)?(?:www\.)?youtu\.be\/([^&\n?#]+)/,
  ]
  
  for (const pattern of patterns) {
    const match = url.match(pattern)
    if (match) {
      return match[1]
    }
  }
  
  return null
}