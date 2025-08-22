import { createClient } from './supabase-client'
import { Module } from '@/types'

export async function getModules(): Promise<Module[]> {
  const supabase = createClient()
  
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