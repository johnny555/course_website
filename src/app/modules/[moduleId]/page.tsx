'use client'

import { useEffect, useState } from 'react'
import { useParams, useRouter } from 'next/navigation'
import Link from 'next/link'
import Image from 'next/image'
import ProtectedRoute from '@/components/ProtectedRoute'
import { getModuleWithSubModules } from '@/lib/modules'
import { ModuleWithSubModules } from '@/types'

export default function ModuleOverviewPage() {
  const params = useParams()
  const router = useRouter()
  const [module, setModule] = useState<ModuleWithSubModules | null>(null)
  const [loading, setLoading] = useState(true)
  const [error, setError] = useState('')

  useEffect(() => {
    async function fetchModule() {
      if (!params.moduleId || typeof params.moduleId !== 'string') {
        setError('Invalid module ID')
        setLoading(false)
        return
      }

      try {
        const data = await getModuleWithSubModules(params.moduleId)
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
  }, [params.moduleId])

  if (loading) {
    return (
      <ProtectedRoute requiresCourseAccess={true}>
        <div className="min-h-screen flex items-center justify-center">
          <div className="animate-spin rounded-full h-32 w-32 border-b-2 border-blue-600"></div>
        </div>
      </ProtectedRoute>
    )
  }

  if (error || !module) {
    return (
      <ProtectedRoute requiresCourseAccess={true}>
        <div className="min-h-screen flex items-center justify-center">
          <div className="text-center">
            <h1 className="text-2xl font-bold text-gray-900 mb-4">Error</h1>
            <p className="text-gray-600 mb-4">{error || 'Module not found'}</p>
            <button
              onClick={() => router.push('/modules')}
              className="bg-blue-600 hover:bg-blue-700 text-white px-6 py-2 rounded transition-colors"
            >
              Back to Modules
            </button>
          </div>
        </div>
      </ProtectedRoute>
    )
  }

  return (
    <ProtectedRoute requiresCourseAccess={true}>
      <div className="min-h-screen bg-gray-50">
        <div className="container mx-auto px-4 py-8">
          <div className="mb-6">
            <button
              onClick={() => router.push('/modules')}
              className="flex items-center text-blue-600 hover:text-blue-700 transition-colors mb-4"
            >
              <svg className="w-5 h-5 mr-2" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M15 19l-7-7 7-7" />
              </svg>
              Back to All Modules
            </button>
            
            {/* Module Header */}
            <div className="bg-white rounded-lg shadow-sm p-8 mb-8">
              <div className="flex items-start space-x-6">
                {module.thumbnail_url && (
                  <div className="flex-shrink-0">
                    <Image
                      src={module.thumbnail_url}
                      alt={module.title}
                      width={200}
                      height={150}
                      className="rounded-lg object-cover"
                    />
                  </div>
                )}
                <div className="flex-1">
                  <h1 className="text-3xl font-bold text-gray-900 mb-4">{module.title}</h1>
                  <p className="text-gray-600 text-lg leading-relaxed">{module.description}</p>
                  <div className="mt-4 text-sm text-gray-500">
                    {module.sub_modules.length} lessons in this module
                  </div>
                </div>
              </div>
            </div>
          </div>

          {/* Sub-modules Grid */}
          <div className="mb-6">
            <h2 className="text-2xl font-bold text-gray-900 mb-6">Course Lessons</h2>
            
            {module.sub_modules.length === 0 ? (
              <div className="text-center py-12 bg-white rounded-lg shadow-sm">
                <div className="text-6xl mb-4">📚</div>
                <h3 className="text-lg font-semibold text-gray-900 mb-2">No lessons yet</h3>
                <p className="text-gray-600">This module doesn&apos;t have any lessons yet.</p>
              </div>
            ) : (
              <div className="grid md:grid-cols-2 lg:grid-cols-3 gap-6">
                {module.sub_modules.map((subModule, index) => (
                  <Link
                    key={subModule.id}
                    href={`/modules/${module.id}/${subModule.id}`}
                    className="bg-white rounded-lg shadow-sm hover:shadow-md transition-shadow overflow-hidden group"
                  >
                    <div className="aspect-video bg-gray-200 relative">
                      {subModule.youtube_urls[0] && (
                        <Image
                          src={`https://img.youtube.com/vi/${extractYouTubeVideoId(subModule.youtube_urls[0]) || ''}/maxresdefault.jpg`}
                          alt={subModule.title}
                          fill
                          className="object-cover group-hover:scale-105 transition-transform duration-200"
                        />
                      )}
                      <div className="absolute inset-0 bg-black bg-opacity-0 group-hover:bg-opacity-10 transition-all duration-200 flex items-center justify-center">
                        <div className="w-12 h-12 bg-red-600 rounded-full flex items-center justify-center opacity-0 group-hover:opacity-100 transition-opacity duration-200">
                          <svg className="w-6 h-6 text-white ml-1" fill="currentColor" viewBox="0 0 20 20">
                            <path d="M6.3 2.841A1.5 1.5 0 004 4.11V15.89a1.5 1.5 0 002.3 1.269l9.344-5.89a1.5 1.5 0 000-2.538L6.3 2.84z" />
                          </svg>
                        </div>
                      </div>
                      {/* Lesson Number Badge */}
                      <div className="absolute top-3 left-3 bg-blue-600 text-white text-sm font-semibold px-2 py-1 rounded">
                        Lesson {index + 1}
                      </div>
                    </div>
                    <div className="p-6">
                      <h3 className="text-lg font-semibold text-gray-900 mb-2 group-hover:text-blue-600 transition-colors">
                        {subModule.title}
                      </h3>
                      <p className="text-gray-600 text-sm line-clamp-3">
                        {subModule.description.length > 120
                          ? `${subModule.description.substring(0, 120)}...`
                          : subModule.description}
                      </p>
                      <div className="mt-4 flex items-center text-sm text-gray-500">
                        <span>{subModule.youtube_urls.length} video{subModule.youtube_urls.length !== 1 ? 's' : ''}</span>
                        {subModule.screenshot_urls.length > 0 && (
                          <>
                            <span className="mx-2">•</span>
                            <span>{subModule.screenshot_urls.length} image{subModule.screenshot_urls.length !== 1 ? 's' : ''}</span>
                          </>
                        )}
                      </div>
                    </div>
                  </Link>
                ))}
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