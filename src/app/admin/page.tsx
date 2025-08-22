'use client'

import { useEffect, useState } from 'react'
import Link from 'next/link'
import Image from 'next/image'
import ProtectedRoute from '@/components/ProtectedRoute'
import { getModules, deleteModule, getModuleWithSubModules } from '@/lib/modules'
import { Module, ModuleWithSubModules } from '@/types'

export default function AdminDashboard() {
  const [modules, setModules] = useState<Module[]>([])
  const [loading, setLoading] = useState(true)
  const [deleting, setDeleting] = useState<string | null>(null)

  useEffect(() => {
    fetchModules()
  }, [])

  const fetchModules = async () => {
    try {
      const data = await getModules()
      setModules(data)
    } catch (error) {
      console.error('Error fetching modules:', error)
    } finally {
      setLoading(false)
    }
  }

  const handleDelete = async (id: string) => {
    if (!confirm('Are you sure you want to delete this module?')) {
      return
    }

    setDeleting(id)
    try {
      const success = await deleteModule(id)
      if (success) {
        setModules(modules.filter(m => m.id !== id))
      } else {
        alert('Failed to delete module')
      }
    } catch (error) {
      console.error('Error deleting module:', error)
      alert('Failed to delete module')
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

  return (
    <ProtectedRoute adminOnly>
      <div className="min-h-screen bg-gray-50 py-8">
        <div className="container mx-auto px-4">
          <div className="flex justify-between items-center mb-8">
            <div>
              <h1 className="text-3xl font-bold text-gray-900 mb-2">Admin Dashboard</h1>
              <p className="text-gray-600">Manage course modules and content</p>
            </div>
            <Link
              href="/admin/modules/create"
              className="bg-blue-600 hover:bg-blue-700 text-white px-6 py-3 rounded-lg font-semibold transition-colors"
            >
              Create New Module
            </Link>
          </div>

          <div className="bg-white rounded-lg shadow-sm overflow-hidden">
            <div className="px-6 py-4 border-b border-gray-200">
              <h2 className="text-lg font-semibold text-gray-900">Modules ({modules.length})</h2>
            </div>

            {modules.length === 0 ? (
              <div className="text-center py-12">
                <div className="text-6xl mb-4">📚</div>
                <h3 className="text-lg font-semibold text-gray-900 mb-2">No modules yet</h3>
                <p className="text-gray-600 mb-4">Create your first module to get started</p>
                <Link
                  href="/admin/modules/create"
                  className="bg-blue-600 hover:bg-blue-700 text-white px-6 py-2 rounded transition-colors"
                >
                  Create Module
                </Link>
              </div>
            ) : (
              <div className="overflow-x-auto">
                <table className="min-w-full divide-y divide-gray-200">
                  <thead className="bg-gray-50">
                    <tr>
                      <th className="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider">
                        Module
                      </th>
                      <th className="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider">
                        Order
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
                    {modules.map((module) => (
                      <tr key={module.id} className="hover:bg-gray-50">
                        <td className="px-6 py-4 whitespace-nowrap">
                          <div className="flex items-center">
                            <div className="flex-shrink-0 h-12 w-12">
                              {module.thumbnail_url ? (
                                <Image
                                  src={module.thumbnail_url}
                                  alt={module.title}
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
                                {module.title}
                              </div>
                              <div className="text-sm text-gray-500 max-w-xs truncate">
                                {module.description}
                              </div>
                            </div>
                          </div>
                        </td>
                        <td className="px-6 py-4 whitespace-nowrap text-sm text-gray-900">
                          {module.order}
                        </td>
                        <td className="px-6 py-4 whitespace-nowrap text-sm text-gray-500">
                          {new Date(module.created_at).toLocaleDateString()}
                        </td>
                        <td className="px-6 py-4 whitespace-nowrap text-sm font-medium space-x-2">
                          <Link
                            href={`/modules/${module.id}`}
                            className="text-blue-600 hover:text-blue-900 transition-colors"
                          >
                            View
                          </Link>
                          <Link
                            href={`/admin/modules/${module.id}/sub-modules`}
                            className="text-green-600 hover:text-green-900 transition-colors"
                          >
                            Lessons
                          </Link>
                          <Link
                            href={`/admin/modules/${module.id}/edit`}
                            className="text-yellow-600 hover:text-yellow-900 transition-colors"
                          >
                            Edit
                          </Link>
                          <button
                            onClick={() => handleDelete(module.id)}
                            disabled={deleting === module.id}
                            className="text-red-600 hover:text-red-900 transition-colors disabled:opacity-50"
                          >
                            {deleting === module.id ? 'Deleting...' : 'Delete'}
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