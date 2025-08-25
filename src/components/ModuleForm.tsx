'use client'

import { useState } from 'react'
import { useRouter } from 'next/navigation'
import ReactMarkdown from 'react-markdown'
import rehypeHighlight from 'rehype-highlight'
import { useForm } from 'react-hook-form'
import { createModule, updateModule } from '@/lib/modules'
import { Module } from '@/types'
import 'highlight.js/styles/github.css'

interface ModuleFormData {
  title: string
  description: string
  thumbnail_url?: string
}

interface ModuleFormProps {
  module?: Module
  isEdit?: boolean
}

export default function ModuleForm({ module, isEdit = false }: ModuleFormProps) {
  const router = useRouter()
  const [loading, setLoading] = useState(false)
  const [showPreview, setShowPreview] = useState(false)
  const [error, setError] = useState('')

  const {
    register,
    handleSubmit,
    watch,
    formState: { errors },
  } = useForm<ModuleFormData>({
    defaultValues: {
      title: module?.title || '',
      description: module?.description || '',
      thumbnail_url: module?.thumbnail_url || '',
    },
  })

  const watchedDescription = watch('description')


  const onSubmit = async (data: ModuleFormData) => {
    setLoading(true)
    setError('')

    try {
      const moduleData = {
        ...data,
        thumbnail_url: data.thumbnail_url || null,
        // Keep existing order for edits, auto-assign for new modules (will be handled by createModule function)
        order: isEdit && module ? module.order : 0
      }

      let result
      if (isEdit && module) {
        result = await updateModule(module.id, moduleData)
      } else {
        result = await createModule(moduleData)
      }

      if (result) {
        router.push('/admin')
      } else {
        setError(`Failed to ${isEdit ? 'update' : 'create'} module`)
      }
    } catch (error) {
      console.error('Error saving module:', error)
      setError(`Failed to ${isEdit ? 'update' : 'create'} module`)
    } finally {
      setLoading(false)
    }
  }

  return (
    <div className="min-h-screen bg-gray-50 py-8">
      <div className="container mx-auto px-4">
        <div className="mb-6">
          <button
            onClick={() => router.push('/admin')}
            className="flex items-center text-blue-600 hover:text-blue-700 transition-colors mb-4"
          >
            <svg className="w-5 h-5 mr-2" fill="none" stroke="currentColor" viewBox="0 0 24 24">
              <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M15 19l-7-7 7-7" />
            </svg>
            Back to Admin Dashboard
          </button>
          <h1 className="text-3xl font-bold text-gray-900 mb-2">
            {isEdit ? 'Edit Module' : 'Create New Module'}
          </h1>
        </div>

        <div className="grid lg:grid-cols-2 gap-8">
          <div className="bg-white rounded-lg shadow-sm p-6">
            <form onSubmit={handleSubmit(onSubmit)} className="space-y-6">
              <div>
                <label htmlFor="title" className="block text-sm font-medium text-gray-700 mb-1">
                  Title *
                </label>
                <input
                  {...register('title', { required: 'Title is required' })}
                  type="text"
                  className="w-full px-3 py-2 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-blue-500 focus:border-blue-500"
                  placeholder="Module title"
                />
                {errors.title && (
                  <p className="mt-1 text-sm text-red-600">{errors.title.message}</p>
                )}
              </div>



              <div>
                <label htmlFor="thumbnail_url" className="block text-sm font-medium text-gray-700 mb-1">
                  Thumbnail URL
                </label>
                <input
                  {...register('thumbnail_url')}
                  type="url"
                  className="w-full px-3 py-2 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-blue-500 focus:border-blue-500"
                  placeholder="https://..."
                />
              </div>

              <div>
                <div className="flex justify-between items-center mb-2">
                  <label htmlFor="description" className="block text-sm font-medium text-gray-700">
                    Description * (Markdown supported)
                  </label>
                  <button
                    type="button"
                    onClick={() => setShowPreview(!showPreview)}
                    className="text-sm text-blue-600 hover:text-blue-700 transition-colors"
                  >
                    {showPreview ? 'Hide Preview' : 'Show Preview'}
                  </button>
                </div>
                <textarea
                  {...register('description', { required: 'Description is required' })}
                  rows={12}
                  className="w-full px-3 py-2 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-blue-500 focus:border-blue-500"
                  placeholder="Module description with markdown formatting..."
                />
                {errors.description && (
                  <p className="mt-1 text-sm text-red-600">{errors.description.message}</p>
                )}
              </div>


              {error && (
                <div className="text-red-600 text-sm">{error}</div>
              )}

              <div className="flex space-x-4">
                <button
                  type="submit"
                  disabled={loading}
                  className="bg-blue-600 hover:bg-blue-700 text-white px-6 py-2 rounded font-semibold transition-colors disabled:opacity-50"
                >
                  {loading 
                    ? (isEdit ? 'Updating...' : 'Creating...') 
                    : (isEdit ? 'Update Module' : 'Create Module')
                  }
                </button>
                <button
                  type="button"
                  onClick={() => router.push('/admin')}
                  className="bg-gray-300 hover:bg-gray-400 text-gray-800 px-6 py-2 rounded font-semibold transition-colors"
                >
                  Cancel
                </button>
              </div>
            </form>
          </div>

          {/* Preview Column */}
          <div className="bg-white rounded-lg shadow-sm p-6">
            <h3 className="text-lg font-semibold text-gray-900 mb-4">Preview</h3>
            

            {showPreview && watchedDescription && (
              <div>
                <h4 className="text-sm font-medium text-gray-700 mb-2">Description</h4>
                <div className="prose prose-sm max-w-none border rounded p-4 bg-gray-50 max-h-96 overflow-y-auto">
                  <ReactMarkdown
                    rehypePlugins={[rehypeHighlight]}
                    components={{
                      code: ({ className, children, ...props }: any) => {
                        const match = /language-(\w+)/.exec(className || '')
                        const isCodeBlock = match && className?.includes('language-')
                        return isCodeBlock ? (
                          <pre className="bg-gray-100 rounded p-2 overflow-x-auto">
                            <code className={className} {...props}>
                              {children}
                            </code>
                          </pre>
                        ) : (
                          <code className="bg-gray-100 px-1 py-0.5 rounded text-sm" {...props}>
                            {children}
                          </code>
                        )
                      },
                    }}
                  >
                    {watchedDescription}
                  </ReactMarkdown>
                </div>
              </div>
            )}

          </div>
        </div>
      </div>
    </div>
  )
}