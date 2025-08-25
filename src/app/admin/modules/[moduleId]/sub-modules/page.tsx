'use client'

import { useEffect, useState, useCallback } from 'react'
import { useParams, useRouter } from 'next/navigation'
import Link from 'next/link'
import Image from 'next/image'
import ProtectedRoute from '@/components/ProtectedRoute'
import { getModuleWithSubModules, deleteSubModule, reorderSubModules, extractYouTubeVideoId } from '@/lib/modules'
import { ModuleWithSubModules } from '@/types'

export default function SubModulesAdminPage() {
  const params = useParams()
  const router = useRouter()
  const [module, setModule] = useState<ModuleWithSubModules | null>(null)
  const [loading, setLoading] = useState(true)
  const [deleting, setDeleting] = useState<string | null>(null)
  const [draggedItem, setDraggedItem] = useState<number | null>(null)
  const [reordering, setReordering] = useState(false)

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

  const handleDragStart = (e: React.DragEvent, index: number) => {
    setDraggedItem(index)
    e.dataTransfer.effectAllowed = 'move'
  }

  const handleDragOver = (e: React.DragEvent) => {
    e.preventDefault()
    e.dataTransfer.dropEffect = 'move'
  }

  const handleDrop = async (e: React.DragEvent, dropIndex: number) => {
    e.preventDefault()
    
    if (draggedItem === null || draggedItem === dropIndex || !module) {
      setDraggedItem(null)
      return
    }

    setReordering(true)
    
    // Reorder locally first
    const newSubModules = [...module.sub_modules]
    const draggedSubModule = newSubModules[draggedItem]
    newSubModules.splice(draggedItem, 1)
    newSubModules.splice(dropIndex, 0, draggedSubModule)
    
    // Update order numbers
    const reorderedSubModules = newSubModules.map((subModule, index) => ({
      id: subModule.id,
      order: index + 1
    }))
    
    try {
      const success = await reorderSubModules(reorderedSubModules)
      if (success) {
        // Update local state with new order
        const updatedSubModules = newSubModules.map((subModule, index) => ({
          ...subModule,
          order: index + 1
        }))
        setModule({
          ...module,
          sub_modules: updatedSubModules
        })
      } else {
        alert('Failed to reorder lessons')
      }
    } catch (error) {
      console.error('Error reordering sub-modules:', error)
      alert('Failed to reorder lessons')
    } finally {
      setReordering(false)
      setDraggedItem(null)
    }
  }

  const moveSubModule = async (index: number, direction: 'up' | 'down') => {
    const newIndex = direction === 'up' ? index - 1 : index + 1
    if (newIndex < 0 || newIndex >= module!.sub_modules.length) return
    
    const e = new DragEvent('drop')
    setDraggedItem(index)
    await handleDrop(e as any, newIndex)
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

          <div className="mb-6">
            <div className="flex justify-between items-center mb-4">
              <h2 className="text-lg font-semibold text-gray-900">
                Lessons ({module.sub_modules.length})
              </h2>
              {module.sub_modules.length > 0 && (
                <p className="text-sm text-gray-500">
                  {reordering ? 'Reordering...' : 'Drag to reorder lessons'}
                </p>
              )}
            </div>

            {module.sub_modules.length === 0 ? (
              <div className="bg-white rounded-lg shadow-sm p-12 text-center">
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
              <div className="space-y-4">
                {module.sub_modules.map((subModule, index) => (
                  <div
                    key={subModule.id}
                    draggable
                    onDragStart={(e) => handleDragStart(e, index)}
                    onDragOver={handleDragOver}
                    onDrop={(e) => handleDrop(e, index)}
                    className={`bg-white rounded-lg shadow-sm border-2 transition-all cursor-move ${
                      draggedItem === index 
                        ? 'border-blue-300 shadow-lg opacity-50' 
                        : 'border-gray-200 hover:border-gray-300 hover:shadow-md'
                    } ${reordering ? 'pointer-events-none' : ''}`}
                  >
                    <div className="p-6">
                      <div className="flex items-center justify-between">
                        <div className="flex items-center space-x-4">
                          {/* Drag Handle */}
                          <div className="flex flex-col space-y-1 text-gray-400 cursor-move">
                            <div className="w-1 h-1 bg-gray-400 rounded-full"></div>
                            <div className="w-1 h-1 bg-gray-400 rounded-full"></div>
                            <div className="w-1 h-1 bg-gray-400 rounded-full"></div>
                            <div className="w-1 h-1 bg-gray-400 rounded-full"></div>
                            <div className="w-1 h-1 bg-gray-400 rounded-full"></div>
                            <div className="w-1 h-1 bg-gray-400 rounded-full"></div>
                          </div>

                          {/* Order Badge */}
                          <div className="flex-shrink-0 w-8 h-8 bg-green-100 text-green-700 rounded-full flex items-center justify-center text-sm font-semibold">
                            {index + 1}
                          </div>

                          {/* Video Thumbnail */}
                          <div className="flex-shrink-0">
                            {subModule.youtube_urls[0] ? (
                              <Image
                                src={`https://img.youtube.com/vi/${extractYouTubeVideoId(subModule.youtube_urls[0]) || ''}/default.jpg`}
                                alt={subModule.title}
                                width={60}
                                height={45}
                                className="rounded object-cover"
                              />
                            ) : (
                              <div className="w-16 h-12 bg-gray-200 rounded flex items-center justify-center">
                                <span className="text-gray-400 text-xs">🎥</span>
                              </div>
                            )}
                          </div>

                          {/* Content */}
                          <div className="flex-1 min-w-0">
                            <h3 className="text-lg font-semibold text-gray-900 mb-1">
                              {subModule.title}
                            </h3>
                            <p className="text-gray-600 line-clamp-2">
                              {subModule.description.length > 100 
                                ? `${subModule.description.substring(0, 100)}...` 
                                : subModule.description}
                            </p>
                            <div className="flex items-center space-x-4 text-sm text-gray-500 mt-2">
                              <span>{subModule.youtube_urls.length} video{subModule.youtube_urls.length !== 1 ? 's' : ''}</span>
                              {subModule.screenshot_urls.length > 0 && (
                                <>
                                  <span>•</span>
                                  <span>{subModule.screenshot_urls.length} image{subModule.screenshot_urls.length !== 1 ? 's' : ''}</span>
                                </>
                              )}
                              <span>•</span>
                              <span>Created: {new Date(subModule.created_at).toLocaleDateString()}</span>
                            </div>
                          </div>
                        </div>

                        <div className="flex items-center space-x-2">
                          {/* Move Up/Down Buttons */}
                          <div className="flex flex-col space-y-1">
                            <button
                              onClick={() => moveSubModule(index, 'up')}
                              disabled={index === 0 || reordering}
                              className="p-1 text-gray-400 hover:text-gray-600 disabled:opacity-30 transition-colors"
                              title="Move up"
                            >
                              <svg className="w-4 h-4" fill="currentColor" viewBox="0 0 20 20">
                                <path fillRule="evenodd" d="M14.707 12.707a1 1 0 01-1.414 0L10 9.414l-3.293 3.293a1 1 0 01-1.414-1.414l4-4a1 1 0 011.414 0l4 4a1 1 0 010 1.414z" clipRule="evenodd" />
                              </svg>
                            </button>
                            <button
                              onClick={() => moveSubModule(index, 'down')}
                              disabled={index === module.sub_modules.length - 1 || reordering}
                              className="p-1 text-gray-400 hover:text-gray-600 disabled:opacity-30 transition-colors"
                              title="Move down"
                            >
                              <svg className="w-4 h-4" fill="currentColor" viewBox="0 0 20 20">
                                <path fillRule="evenodd" d="M5.293 7.293a1 1 0 011.414 0L10 10.586l3.293-3.293a1 1 0 111.414 1.414l-4 4a1 1 0 01-1.414 0l-4-4a1 1 0 010-1.414z" clipRule="evenodd" />
                              </svg>
                            </button>
                          </div>

                          {/* Action Buttons */}
                          <div className="flex space-x-2">
                            <Link
                              href={`/modules/${module.id}/${subModule.id}`}
                              className="text-blue-600 hover:text-blue-700 px-3 py-1 rounded text-sm transition-colors"
                            >
                              View
                            </Link>
                            <Link
                              href={`/admin/modules/${module.id}/sub-modules/${subModule.id}/edit`}
                              className="text-yellow-600 hover:text-yellow-700 px-3 py-1 rounded text-sm transition-colors"
                            >
                              Edit
                            </Link>
                            <button
                              onClick={() => handleDelete(subModule.id)}
                              disabled={deleting === subModule.id}
                              className="text-red-600 hover:text-red-700 px-3 py-1 rounded text-sm transition-colors disabled:opacity-50"
                            >
                              {deleting === subModule.id ? 'Deleting...' : 'Delete'}
                            </button>
                          </div>
                        </div>
                      </div>
                    </div>
                  </div>
                ))}
              </div>
            )}
          </div>
        </div>
      </div>
    </ProtectedRoute>
  )
}

