'use client'

import { useState, useRef } from 'react'
import { useRouter } from 'next/navigation'
import Image from 'next/image'
import ReactMarkdown from 'react-markdown'
import rehypeHighlight from 'rehype-highlight'
import { useForm } from 'react-hook-form'
import { createModule, updateModule, extractYouTubeVideoId } from '@/lib/modules'
import { uploadFile } from '@/lib/storage'
import { Module } from '@/types'
import 'highlight.js/styles/github.css'

interface ModuleFormData {
  title: string
  description: string
  youtube_url: string
  order: number
  thumbnail_url?: string
}

interface ModuleFormProps {
  module?: Module
  isEdit?: boolean
}

export default function ModuleForm({ module, isEdit = false }: ModuleFormProps) {
  const router = useRouter()
  const fileInputRef = useRef<HTMLInputElement>(null)
  const [loading, setLoading] = useState(false)
  const [uploading, setUploading] = useState(false)
  const [screenshots, setScreenshots] = useState<string[]>(module?.screenshot_urls || [])
  const [showPreview, setShowPreview] = useState(false)
  const [error, setError] = useState('')

  const {
    register,
    handleSubmit,
    watch,
    setValue,
    formState: { errors },
  } = useForm<ModuleFormData>({
    defaultValues: {
      title: module?.title || '',
      description: module?.description || '',
      youtube_url: module?.youtube_url || '',
      order: module?.order || 1,
      thumbnail_url: module?.thumbnail_url || '',
    },
  })

  const watchedDescription = watch('description')
  const watchedYouTubeUrl = watch('youtube_url')

  const handleFileUpload = async (event: React.ChangeEvent<HTMLInputElement>) => {
    const files = event.target.files
    if (!files || files.length === 0) return

    setUploading(true)
    const newScreenshots: string[] = []

    try {
      for (const file of files) {
        if (!file.type.startsWith('image/')) {
          alert(`${file.name} is not an image file`)
          continue
        }

        if (file.size > 5 * 1024 * 1024) {
          alert(`${file.name} is too large. Maximum size is 5MB`)
          continue
        }

        const timestamp = Date.now()
        const fileName = `${timestamp}-${file.name.replace(/[^a-zA-Z0-9.-]/g, '_')}`
        const filePath = `modules/${fileName}`

        const url = await uploadFile(file, filePath)
        if (url) {
          newScreenshots.push(url)
        } else {
          alert(`Failed to upload ${file.name}`)
        }
      }

      setScreenshots([...screenshots, ...newScreenshots])
    } catch (error) {
      console.error('Error uploading files:', error)
      alert('Failed to upload files')
    } finally {
      setUploading(false)
      if (fileInputRef.current) {
        fileInputRef.current.value = ''
      }
    }
  }

  const removeScreenshot = (index: number) => {
    const newScreenshots = screenshots.filter((_, i) => i !== index)
    setScreenshots(newScreenshots)
  }

  const setAsThumbnail = (url: string) => {
    setValue('thumbnail_url', url)
  }

  const generateThumbnailFromYouTube = () => {
    const videoId = extractYouTubeVideoId(watchedYouTubeUrl)
    if (videoId) {
      const thumbnailUrl = `https://img.youtube.com/vi/${videoId}/maxresdefault.jpg`
      setValue('thumbnail_url', thumbnailUrl)
    }
  }

  const onSubmit = async (data: ModuleFormData) => {
    setLoading(true)
    setError('')

    try {
      const moduleData = {
        ...data,
        screenshot_urls: screenshots,
        thumbnail_url: data.thumbnail_url || null,
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
                <label htmlFor="order" className="block text-sm font-medium text-gray-700 mb-1">
                  Order *
                </label>
                <input
                  {...register('order', { 
                    required: 'Order is required',
                    min: { value: 1, message: 'Order must be at least 1' }
                  })}
                  type="number"
                  min="1"
                  className="w-full px-3 py-2 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-blue-500 focus:border-blue-500"
                  placeholder="1"
                />
                {errors.order && (
                  <p className="mt-1 text-sm text-red-600">{errors.order.message}</p>
                )}
              </div>

              <div>
                <label htmlFor="youtube_url" className="block text-sm font-medium text-gray-700 mb-1">
                  YouTube URL *
                </label>
                <input
                  {...register('youtube_url', { required: 'YouTube URL is required' })}
                  type="url"
                  className="w-full px-3 py-2 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-blue-500 focus:border-blue-500"
                  placeholder="https://www.youtube.com/watch?v=..."
                />
                {errors.youtube_url && (
                  <p className="mt-1 text-sm text-red-600">{errors.youtube_url.message}</p>
                )}
                <button
                  type="button"
                  onClick={generateThumbnailFromYouTube}
                  className="mt-2 text-sm text-blue-600 hover:text-blue-700 transition-colors"
                >
                  Auto-generate thumbnail from YouTube
                </button>
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

              <div>
                <label className="block text-sm font-medium text-gray-700 mb-2">
                  Screenshots
                </label>
                <input
                  ref={fileInputRef}
                  type="file"
                  multiple
                  accept="image/*"
                  onChange={handleFileUpload}
                  className="w-full px-3 py-2 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-blue-500 focus:border-blue-500"
                />
                <p className="mt-1 text-sm text-gray-500">
                  Upload multiple images (max 5MB each)
                </p>
                
                {screenshots.length > 0 && (
                  <div className="mt-4 grid grid-cols-2 gap-4">
                    {screenshots.map((url, index) => (
                      <div key={index} className="relative group">
                        <Image
                          src={url}
                          alt={`Screenshot ${index + 1}`}
                          width={200}
                          height={96}
                          className="w-full h-24 object-cover rounded border"
                        />
                        <div className="absolute inset-0 bg-black bg-opacity-0 group-hover:bg-opacity-50 transition-all rounded flex items-center justify-center">
                          <div className="opacity-0 group-hover:opacity-100 space-x-2">
                            <button
                              type="button"
                              onClick={() => setAsThumbnail(url)}
                              className="bg-blue-600 text-white px-2 py-1 rounded text-xs"
                            >
                              Set as Thumbnail
                            </button>
                            <button
                              type="button"
                              onClick={() => removeScreenshot(index)}
                              className="bg-red-600 text-white px-2 py-1 rounded text-xs"
                            >
                              Remove
                            </button>
                          </div>
                        </div>
                      </div>
                    ))}
                  </div>
                )}
              </div>

              {error && (
                <div className="text-red-600 text-sm">{error}</div>
              )}

              <div className="flex space-x-4">
                <button
                  type="submit"
                  disabled={loading || uploading}
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
            
            {watchedYouTubeUrl && (
              <div className="mb-6">
                <h4 className="text-sm font-medium text-gray-700 mb-2">Video</h4>
                <div className="aspect-video bg-gray-100 rounded overflow-hidden">
                  <iframe
                    src={`https://www.youtube.com/embed/${extractYouTubeVideoId(watchedYouTubeUrl) || ''}`}
                    title="Preview"
                    className="w-full h-full"
                    allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture"
                    allowFullScreen
                  ></iframe>
                </div>
              </div>
            )}

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

            {uploading && (
              <div className="text-blue-600 text-sm">Uploading images...</div>
            )}
          </div>
        </div>
      </div>
    </div>
  )
}