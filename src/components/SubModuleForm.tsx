'use client'

import { useState, useRef } from 'react'
import { useRouter } from 'next/navigation'
import Image from 'next/image'
import ReactMarkdown from 'react-markdown'
import rehypeHighlight from 'rehype-highlight'
import { useForm, useFieldArray } from 'react-hook-form'
import { createSubModule, updateSubModule, getYouTubeEmbedUrl, extractYouTubeVideoId } from '@/lib/modules'
import { uploadFile } from '@/lib/storage'
import { SubModule } from '@/types'
import 'highlight.js/styles/github.css'

interface SubModuleFormData {
  title: string
  description: string
  youtube_urls: { url: string }[]
  order: number
}

interface SubModuleFormProps {
  moduleId: string
  subModule?: SubModule
  isEdit?: boolean
}

export default function SubModuleForm({ moduleId, subModule, isEdit = false }: SubModuleFormProps) {
  const router = useRouter()
  const fileInputRef = useRef<HTMLInputElement>(null)
  const [loading, setLoading] = useState(false)
  const [uploading, setUploading] = useState(false)
  const [screenshots, setScreenshots] = useState<string[]>(subModule?.screenshot_urls || [])
  const [showPreview, setShowPreview] = useState(false)
  const [error, setError] = useState('')

  const {
    register,
    handleSubmit,
    watch,
    control,
    formState: { errors },
  } = useForm<SubModuleFormData>({
    defaultValues: {
      title: subModule?.title || '',
      description: subModule?.description || '',
      youtube_urls: subModule?.youtube_urls?.map(url => ({ url })) || [{ url: '' }],
      order: subModule?.order || 1,
    },
  })

  const { fields, append, remove } = useFieldArray({
    control,
    name: "youtube_urls"
  })

  const watchedDescription = watch('description')
  const watchedYouTubeUrls = watch('youtube_urls')

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
        const filePath = `sub-modules/${fileName}`

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

  const addYouTubeUrl = () => {
    append({ url: '' })
  }

  const removeYouTubeUrl = (index: number) => {
    if (fields.length > 1) {
      remove(index)
    }
  }

  const onSubmit = async (data: SubModuleFormData) => {
    setLoading(true)
    setError('')

    try {
      const subModuleData = {
        ...data,
        module_id: moduleId,
        youtube_urls: data.youtube_urls.map(item => item.url).filter(url => url.trim() !== ''),
        screenshot_urls: screenshots,
      }

      let result
      if (isEdit && subModule) {
        result = await updateSubModule(subModule.id, subModuleData)
      } else {
        result = await createSubModule(subModuleData)
      }

      if (result) {
        router.push(`/admin/modules/${moduleId}/sub-modules`)
      } else {
        setError(`Failed to ${isEdit ? 'update' : 'create'} lesson`)
      }
    } catch (error) {
      console.error('Error saving sub-module:', error)
      setError(`Failed to ${isEdit ? 'update' : 'create'} lesson`)
    } finally {
      setLoading(false)
    }
  }

  return (
    <div className="min-h-screen bg-gray-50 py-8">
      <div className="container mx-auto px-4">
        <div className="mb-6">
          <button
            onClick={() => router.push(`/admin/modules/${moduleId}/sub-modules`)}
            className="flex items-center text-blue-600 hover:text-blue-700 transition-colors mb-4"
          >
            <svg className="w-5 h-5 mr-2" fill="none" stroke="currentColor" viewBox="0 0 24 24">
              <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M15 19l-7-7 7-7" />
            </svg>
            Back to Lessons
          </button>
          <h1 className="text-3xl font-bold text-gray-900 mb-2">
            {isEdit ? 'Edit Lesson' : 'Create New Lesson'}
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
                  placeholder="Lesson title"
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
                <div className="flex justify-between items-center mb-2">
                  <label className="block text-sm font-medium text-gray-700">
                    YouTube URLs
                  </label>
                  <button
                    type="button"
                    onClick={addYouTubeUrl}
                    className="text-sm text-blue-600 hover:text-blue-700 transition-colors"
                  >
                    + Add Video
                  </button>
                </div>
                {fields.map((field, index) => (
                  <div key={field.id} className="flex items-center space-x-2 mb-2">
                    <input
                      {...register(`youtube_urls.${index}.url` as const)}
                      type="url"
                      className="flex-1 px-3 py-2 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-blue-500 focus:border-blue-500"
                      placeholder="https://www.youtube.com/watch?v=..."
                    />
                    {fields.length > 1 && (
                      <button
                        type="button"
                        onClick={() => removeYouTubeUrl(index)}
                        className="text-red-600 hover:text-red-700 transition-colors p-2"
                      >
                        ×
                      </button>
                    )}
                  </div>
                ))}
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
                  placeholder="Lesson content with markdown formatting..."
                />
                {errors.description && (
                  <p className="mt-1 text-sm text-red-600">{errors.description.message}</p>
                )}
              </div>

              <div>
                <label className="block text-sm font-medium text-gray-700 mb-2">
                  Screenshots & Resources
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
                          <button
                            type="button"
                            onClick={() => removeScreenshot(index)}
                            className="opacity-0 group-hover:opacity-100 bg-red-600 text-white px-2 py-1 rounded text-xs"
                          >
                            Remove
                          </button>
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
                    : (isEdit ? 'Update Lesson' : 'Create Lesson')
                  }
                </button>
                <button
                  type="button"
                  onClick={() => router.push(`/admin/modules/${moduleId}/sub-modules`)}
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
            
            {/* Videos Preview */}
            {watchedYouTubeUrls.some(item => item.url?.trim()) && (
              <div className="mb-6">
                <h4 className="text-sm font-medium text-gray-700 mb-2">Videos</h4>
                <div className="space-y-4">
                  {watchedYouTubeUrls.filter(item => item.url?.trim()).map((item, index) => (
                    <div key={index} className="aspect-video bg-gray-100 rounded overflow-hidden">
                      <iframe
                        src={getYouTubeEmbedUrl(item.url)}
                        title={`Preview Video ${index + 1}`}
                        className="w-full h-full"
                        allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture"
                        allowFullScreen
                      ></iframe>
                    </div>
                  ))}
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