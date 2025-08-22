import { createClient } from './supabase-client'

export async function uploadFile(file: File, path: string): Promise<string | null> {
  const supabase = createClient()
  
  const { data, error } = await supabase.storage
    .from('screenshots')
    .upload(path, file, {
      cacheControl: '3600',
      upsert: false
    })
  
  if (error) {
    console.error('Error uploading file:', error)
    return null
  }
  
  const { data: publicData } = supabase.storage
    .from('screenshots')
    .getPublicUrl(data.path)
  
  return publicData.publicUrl
}

export async function deleteFile(path: string): Promise<boolean> {
  const supabase = createClient()
  
  const { error } = await supabase.storage
    .from('screenshots')
    .remove([path])
  
  if (error) {
    console.error('Error deleting file:', error)
    return false
  }
  
  return true
}

export function getFilePathFromUrl(url: string): string {
  // Extract the file path from a Supabase storage URL
  const urlParts = url.split('/storage/v1/object/public/screenshots/')
  return urlParts[1] || ''
}