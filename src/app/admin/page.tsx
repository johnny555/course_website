'use client'

import { useEffect, useState } from 'react'
import Link from 'next/link'
import Image from 'next/image'
import ProtectedRoute from '@/components/ProtectedRoute'
import { getModules, deleteModule, reorderModules } from '@/lib/modules'
import { Module, ModuleWithSubModules } from '@/types'

export default function AdminDashboard() {
  const [modules, setModules] = useState<Module[]>([])
  const [loading, setLoading] = useState(true)
  const [deleting, setDeleting] = useState<string | null>(null)
  const [draggedItem, setDraggedItem] = useState<number | null>(null)
  const [reordering, setReordering] = useState(false)

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
    
    if (draggedItem === null || draggedItem === dropIndex) {
      setDraggedItem(null)
      return
    }

    setReordering(true)
    
    // Reorder locally first
    const newModules = [...modules]
    const draggedModule = newModules[draggedItem]
    newModules.splice(draggedItem, 1)
    newModules.splice(dropIndex, 0, draggedModule)
    
    // Update order numbers
    const reorderedModules = newModules.map((module, index) => ({
      id: module.id,
      order: index + 1
    }))
    
    try {
      const success = await reorderModules(reorderedModules)
      if (success) {
        // Update local state with new order
        const updatedModules = newModules.map((module, index) => ({
          ...module,
          order: index + 1
        }))
        setModules(updatedModules)
      } else {
        alert('Failed to reorder modules')
      }
    } catch (error) {
      console.error('Error reordering modules:', error)
      alert('Failed to reorder modules')
    } finally {
      setReordering(false)
      setDraggedItem(null)
    }
  }

  const moveModule = async (index: number, direction: 'up' | 'down') => {
    const newIndex = direction === 'up' ? index - 1 : index + 1
    if (newIndex < 0 || newIndex >= modules.length) return
    
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

          <div className="mb-6">
            <div className="flex justify-between items-center mb-4">
              <h2 className="text-lg font-semibold text-gray-900">Modules ({modules.length})</h2>
              {modules.length > 0 && (
                <p className="text-sm text-gray-500">
                  {reordering ? 'Reordering...' : 'Drag to reorder modules'}
                </p>
              )}
            </div>

            {modules.length === 0 ? (
              <div className="bg-white rounded-lg shadow-sm p-12 text-center">
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
              <div className="space-y-4">
                {modules.map((module, index) => (
                  <div
                    key={module.id}
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
                          <div className="flex-shrink-0 w-8 h-8 bg-blue-100 text-blue-700 rounded-full flex items-center justify-center text-sm font-semibold">
                            {index + 1}
                          </div>

                          {/* Thumbnail */}
                          <div className="flex-shrink-0">
                            {module.thumbnail_url ? (
                              <Image
                                src={module.thumbnail_url}
                                alt={module.title}
                                width={60}
                                height={60}
                                className="rounded-lg object-cover"
                              />
                            ) : (
                              <div className="w-16 h-16 bg-gray-200 rounded-lg flex items-center justify-center">
                                <span className="text-gray-400 text-lg">📚</span>
                              </div>
                            )}
                          </div>

                          {/* Content */}
                          <div className="flex-1 min-w-0">
                            <h3 className="text-lg font-semibold text-gray-900 mb-1">
                              {module.title}
                            </h3>
                            <p className="text-gray-600 line-clamp-2">
                              {module.description}
                            </p>
                            <p className="text-sm text-gray-500 mt-2">
                              Created: {new Date(module.created_at).toLocaleDateString()}
                            </p>
                          </div>
                        </div>

                        <div className="flex items-center space-x-2">
                          {/* Move Up/Down Buttons */}
                          <div className="flex flex-col space-y-1">
                            <button
                              onClick={() => moveModule(index, 'up')}
                              disabled={index === 0 || reordering}
                              className="p-1 text-gray-400 hover:text-gray-600 disabled:opacity-30 transition-colors"
                              title="Move up"
                            >
                              <svg className="w-4 h-4" fill="currentColor" viewBox="0 0 20 20">
                                <path fillRule="evenodd" d="M14.707 12.707a1 1 0 01-1.414 0L10 9.414l-3.293 3.293a1 1 0 01-1.414-1.414l4-4a1 1 0 011.414 0l4 4a1 1 0 010 1.414z" clipRule="evenodd" />
                              </svg>
                            </button>
                            <button
                              onClick={() => moveModule(index, 'down')}
                              disabled={index === modules.length - 1 || reordering}
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
                              href={`/modules/${module.id}`}
                              className="text-blue-600 hover:text-blue-700 px-3 py-1 rounded text-sm transition-colors"
                            >
                              View
                            </Link>
                            <Link
                              href={`/admin/modules/${module.id}/sub-modules`}
                              className="text-green-600 hover:text-green-700 px-3 py-1 rounded text-sm transition-colors"
                            >
                              Lessons
                            </Link>
                            <Link
                              href={`/admin/modules/${module.id}/edit`}
                              className="text-yellow-600 hover:text-yellow-700 px-3 py-1 rounded text-sm transition-colors"
                            >
                              Edit
                            </Link>
                            <button
                              onClick={() => handleDelete(module.id)}
                              disabled={deleting === module.id}
                              className="text-red-600 hover:text-red-700 px-3 py-1 rounded text-sm transition-colors disabled:opacity-50"
                            >
                              {deleting === module.id ? 'Deleting...' : 'Delete'}
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