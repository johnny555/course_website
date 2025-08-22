import { createClient } from './supabase-client'
import { Module } from '@/types'

export async function getModules(): Promise<Module[]> {
  const supabase = createClient()
  
  // Check if we're using placeholder credentials
  const isPlaceholder = process.env.NEXT_PUBLIC_SUPABASE_URL?.includes('placeholder') || 
                       process.env.NEXT_PUBLIC_SUPABASE_URL === 'https://placeholder.supabase.co'
  
  if (isPlaceholder) {
    // Return demo data for placeholder mode
    return [
      {
        id: 'demo-1',
        title: 'Introduction to Robotics',
        description: '# Welcome to Robotics!\n\nThis is a **demo module** to showcase the platform.\n\n```python\n# Sample robotics code\nimport robot\n\ndef move_forward():\n    robot.forward(speed=50)\n```\n\nTo see real content, set up your Supabase credentials!',
        youtube_url: 'https://www.youtube.com/watch?v=dQw4w9WgXcQ',
        screenshot_urls: [],
        thumbnail_url: 'https://img.youtube.com/vi/dQw4w9WgXcQ/maxresdefault.jpg',
        order: 1,
        created_at: new Date().toISOString(),
        updated_at: new Date().toISOString(),
      },
      {
        id: 'demo-2',
        title: 'Building Your First Robot',
        description: '# Building Your First Robot\n\nLearn the basics of robot construction.\n\n## What You\'ll Need:\n- Arduino board\n- Sensors\n- Motors\n- Patience!\n\nThis is **demo content** - configure Supabase to add real modules.',
        youtube_url: 'https://www.youtube.com/watch?v=dQw4w9WgXcQ',
        screenshot_urls: [],
        thumbnail_url: 'https://img.youtube.com/vi/dQw4w9WgXcQ/maxresdefault.jpg',
        order: 2,
        created_at: new Date().toISOString(),
        updated_at: new Date().toISOString(),
      }
    ]
  }
  
  const { data, error } = await supabase
    .from('modules')
    .select('*')
    .order('order', { ascending: true })
  
  if (error) {
    console.error('Error fetching modules:', error)
    return []
  }
  
  return data || []
}

export async function getModule(id: string): Promise<Module | null> {
  const supabase = createClient()
  
  // Check if we're using placeholder credentials
  const isPlaceholder = process.env.NEXT_PUBLIC_SUPABASE_URL?.includes('placeholder') || 
                       process.env.NEXT_PUBLIC_SUPABASE_URL === 'https://placeholder.supabase.co'
  
  if (isPlaceholder) {
    // Return demo data for placeholder mode
    const demoModules = await getModules()
    return demoModules.find(module => module.id === id) || null
  }
  
  const { data, error } = await supabase
    .from('modules')
    .select('*')
    .eq('id', id)
    .single()
  
  if (error) {
    console.error('Error fetching module:', error)
    return null
  }
  
  return data
}

export async function createModule(module: Omit<Module, 'id' | 'created_at' | 'updated_at'>): Promise<Module | null> {
  const supabase = createClient()
  
  const { data, error } = await supabase
    .from('modules')
    .insert(module)
    .select()
    .single()
  
  if (error) {
    console.error('Error creating module:', error)
    return null
  }
  
  return data
}

export async function updateModule(id: string, updates: Partial<Omit<Module, 'id' | 'created_at' | 'updated_at'>>): Promise<Module | null> {
  const supabase = createClient()
  
  const { data, error } = await supabase
    .from('modules')
    .update(updates)
    .eq('id', id)
    .select()
    .single()
  
  if (error) {
    console.error('Error updating module:', error)
    return null
  }
  
  return data
}

export async function deleteModule(id: string): Promise<boolean> {
  const supabase = createClient()
  
  const { error } = await supabase
    .from('modules')
    .delete()
    .eq('id', id)
  
  if (error) {
    console.error('Error deleting module:', error)
    return false
  }
  
  return true
}

export function getYouTubeEmbedUrl(url: string): string {
  // Convert various YouTube URL formats to embed format
  const videoId = extractYouTubeVideoId(url)
  if (!videoId) return url
  
  return `https://www.youtube.com/embed/${videoId}`
}

export function extractYouTubeVideoId(url: string): string | null {
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