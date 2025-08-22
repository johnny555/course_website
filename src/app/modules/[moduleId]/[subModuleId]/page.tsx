'use client'

import { useEffect, useState } from 'react'
import { useParams, useRouter } from 'next/navigation'
import Link from 'next/link'
import Image from 'next/image'
import ReactMarkdown from 'react-markdown'
import rehypeHighlight from 'rehype-highlight'
import ProtectedRoute from '@/components/ProtectedRoute'
import { getSubModule, getModuleWithSubModules, getYouTubeEmbedUrl } from '@/lib/modules'
import { SubModule, ModuleWithSubModules } from '@/types'
import 'highlight.js/styles/github.css'

export default function SubModuleDetailPage() {
  const params = useParams()
  const router = useRouter()
  const [subModule, setSubModule] = useState<SubModule | null>(null)
  const [module, setModule] = useState<ModuleWithSubModules | null>(null)
  const [loading, setLoading] = useState(true)
  const [error, setError] = useState('')

  useEffect(() => {
    async function fetchData() {
      if (!params.moduleId || !params.subModuleId || 
          typeof params.moduleId !== 'string' || typeof params.subModuleId !== 'string') {
        setError('Invalid module or sub-module ID')
        setLoading(false)
        return
      }

      try {
        const [subModuleData, moduleData] = await Promise.all([
          getSubModule(params.subModuleId),
          getModuleWithSubModules(params.moduleId)
        ])

        if (subModuleData && moduleData) {
          setSubModule(subModuleData)
          setModule(moduleData)
        } else {
          setError('Content not found')
        }
      } catch (error) {
        console.error('Error fetching data:', error)
        setError('Failed to load content')
      } finally {
        setLoading(false)
      }
    }

    fetchData()
  }, [params.moduleId, params.subModuleId])

  if (loading) {
    return (
      <ProtectedRoute>
        <div className="min-h-screen flex items-center justify-center">
          <div className="animate-spin rounded-full h-32 w-32 border-b-2 border-blue-600"></div>
        </div>
      </ProtectedRoute>
    )
  }

  if (error || !subModule || !module) {
    return (
      <ProtectedRoute>
        <div className="min-h-screen flex items-center justify-center">
          <div className="text-center">
            <h1 className="text-2xl font-bold text-gray-900 mb-4">Error</h1>
            <p className="text-gray-600 mb-4">{error || 'Content not found'}</p>
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

  const currentIndex = module.sub_modules.findIndex(sub => sub.id === subModule.id)
  const prevSubModule = currentIndex > 0 ? module.sub_modules[currentIndex - 1] : null
  const nextSubModule = currentIndex < module.sub_modules.length - 1 ? module.sub_modules[currentIndex + 1] : null

  return (
    <ProtectedRoute>
      <div className="min-h-screen bg-gray-50">
        <div className="flex">
          {/* Table of Contents Sidebar */}
          <div className="hidden lg:block w-80 bg-white shadow-sm border-r min-h-screen">
            <div className="sticky top-0 p-6">
              {/* Module Header */}
              <div className="mb-6">
                <Link
                  href={`/modules/${module.id}`}
                  className="text-sm text-blue-600 hover:text-blue-700 transition-colors mb-2 block"
                >
                  ← Back to Module
                </Link>
                <h2 className="text-lg font-semibold text-gray-900">{module.title}</h2>
                <p className="text-sm text-gray-600 mt-1">{module.sub_modules.length} lessons</p>
              </div>

              {/* Sub-modules List */}
              <div className="space-y-2">
                <h3 className="text-sm font-medium text-gray-500 uppercase tracking-wide mb-3">
                  Course Content
                </h3>
                {module.sub_modules.map((sub, index) => (
                  <Link
                    key={sub.id}
                    href={`/modules/${module.id}/${sub.id}`}
                    className={`block p-3 rounded-lg transition-colors ${
                      sub.id === subModule.id
                        ? 'bg-blue-100 text-blue-700 border-l-4 border-blue-600'
                        : 'text-gray-700 hover:bg-gray-100'
                    }`}
                  >
                    <div className="flex items-start space-x-3">
                      <span className={`text-xs font-medium px-2 py-1 rounded ${
                        sub.id === subModule.id ? 'bg-blue-200 text-blue-800' : 'bg-gray-200 text-gray-600'
                      }`}>
                        {index + 1}
                      </span>
                      <div className="flex-1 min-w-0">
                        <div className="text-sm font-medium truncate">{sub.title}</div>
                        <div className="text-xs text-gray-500 mt-1">
                          {sub.youtube_urls.length} video{sub.youtube_urls.length !== 1 ? 's' : ''}
                        </div>
                      </div>
                    </div>
                  </Link>
                ))}
              </div>
            </div>
          </div>

          {/* Main Content */}
          <div className="flex-1">
            <div className="container mx-auto px-4 py-8 max-w-4xl">
              {/* Mobile Navigation */}
              <div className="lg:hidden mb-6">
                <Link
                  href={`/modules/${module.id}`}
                  className="flex items-center text-blue-600 hover:text-blue-700 transition-colors mb-4"
                >
                  <svg className="w-5 h-5 mr-2" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                    <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M15 19l-7-7 7-7" />
                  </svg>
                  Back to {module.title}
                </Link>
              </div>

              {/* Sub-module Header */}
              <div className="mb-8">
                <div className="flex items-center space-x-2 text-sm text-gray-500 mb-2">
                  <span>{module.title}</span>
                  <span>→</span>
                  <span>Lesson {currentIndex + 1}</span>
                </div>
                <h1 className="text-3xl font-bold text-gray-900 mb-4">{subModule.title}</h1>
              </div>

              {/* Videos Section */}
              {subModule.youtube_urls.length > 0 && (
                <div className="mb-8">
                  <h2 className="text-xl font-semibold text-gray-900 mb-4">
                    Video{subModule.youtube_urls.length > 1 ? 's' : ''}
                  </h2>
                  <div className="space-y-6">
                    {subModule.youtube_urls.map((url, index) => (
                      <div key={index} className="bg-white rounded-lg shadow-sm overflow-hidden">
                        <div className="aspect-video">
                          <iframe
                            src={getYouTubeEmbedUrl(url)}
                            title={`${subModule.title} - Video ${index + 1}`}
                            className="w-full h-full"
                            allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture"
                            allowFullScreen
                          ></iframe>
                        </div>
                        {subModule.youtube_urls.length > 1 && (
                          <div className="p-3 bg-gray-50 text-sm text-gray-600">
                            Video {index + 1} of {subModule.youtube_urls.length}
                          </div>
                        )}
                      </div>
                    ))}
                  </div>
                </div>
              )}

              {/* Content Section */}
              <div className="bg-white rounded-lg shadow-sm p-8 mb-8">
                <h2 className="text-xl font-semibold text-gray-900 mb-6">Lesson Content</h2>
                <div className="prose prose-blue max-w-none">
                  <ReactMarkdown
                    rehypePlugins={[rehypeHighlight]}
                    components={{
                      code: ({ className, children, ...props }: any) => {
                        const match = /language-(\w+)/.exec(className || '')
                        const isCodeBlock = match && className?.includes('language-')
                        return isCodeBlock ? (
                          <pre className="bg-gray-100 rounded p-4 overflow-x-auto">
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
                    {subModule.description}
                  </ReactMarkdown>
                </div>
              </div>

              {/* Screenshots Section */}
              {subModule.screenshot_urls && subModule.screenshot_urls.length > 0 && (
                <div className="bg-white rounded-lg shadow-sm p-8 mb-8">
                  <h2 className="text-xl font-semibold text-gray-900 mb-6">Screenshots & Resources</h2>
                  <div className="grid md:grid-cols-2 gap-4">
                    {subModule.screenshot_urls.map((url, index) => (
                      <div key={index} className="relative aspect-video rounded-lg overflow-hidden">
                        <Image
                          src={url}
                          alt={`Screenshot ${index + 1}`}
                          fill
                          className="object-cover hover:scale-105 transition-transform duration-200"
                        />
                      </div>
                    ))}
                  </div>
                </div>
              )}

              {/* Navigation */}
              <div className="flex justify-between items-center py-8">
                <div>
                  {prevSubModule ? (
                    <Link
                      href={`/modules/${module.id}/${prevSubModule.id}`}
                      className="flex items-center text-blue-600 hover:text-blue-700 transition-colors"
                    >
                      <svg className="w-5 h-5 mr-2" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                        <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M15 19l-7-7 7-7" />
                      </svg>
                      <div className="text-left">
                        <div className="text-sm text-gray-500">Previous</div>
                        <div className="font-medium">{prevSubModule.title}</div>
                      </div>
                    </Link>
                  ) : (
                    <div></div>
                  )}
                </div>

                <div>
                  {nextSubModule ? (
                    <Link
                      href={`/modules/${module.id}/${nextSubModule.id}`}
                      className="flex items-center text-blue-600 hover:text-blue-700 transition-colors"
                    >
                      <div className="text-right">
                        <div className="text-sm text-gray-500">Next</div>
                        <div className="font-medium">{nextSubModule.title}</div>
                      </div>
                      <svg className="w-5 h-5 ml-2" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                        <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M9 5l7 7-7 7" />
                      </svg>
                    </Link>
                  ) : (
                    <Link
                      href={`/modules/${module.id}`}
                      className="bg-green-600 hover:bg-green-700 text-white px-6 py-3 rounded-lg font-medium transition-colors"
                    >
                      Complete Module
                    </Link>
                  )}
                </div>
              </div>
            </div>
          </div>
        </div>
      </div>
    </ProtectedRoute>
  )
}