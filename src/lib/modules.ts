import { createClient } from './supabase-client'
import { Module, SubModule, ModuleWithSubModules } from '@/types'

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
        description: 'Learn the fundamentals of robotics, from basic concepts to practical applications.',
        thumbnail_url: 'https://img.youtube.com/vi/dQw4w9WgXcQ/maxresdefault.jpg',
        order: 1,
        created_at: new Date().toISOString(),
        updated_at: new Date().toISOString(),
      },
      {
        id: 'demo-2',
        title: 'Building Your First Robot',
        description: 'Hands-on guide to constructing your first robot from scratch.',
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

export async function getModuleWithSubModules(id: string): Promise<ModuleWithSubModules | null> {
  const supabase = createClient()
  
  // Check if we're using placeholder credentials
  const isPlaceholder = process.env.NEXT_PUBLIC_SUPABASE_URL?.includes('placeholder') || 
                       process.env.NEXT_PUBLIC_SUPABASE_URL === 'https://placeholder.supabase.co'
  
  if (isPlaceholder) {
    // Return demo data for placeholder mode
    const demoSubModules = [
      {
        id: 'demo-sub-1',
        module_id: 'demo-1',
        title: 'What is a Robot?',
        description: '# What is a Robot?\n\nRobots are programmable machines that can perform tasks autonomously.\n\n```python\n# Basic robot control\nrobot.move_forward(100)\nrobot.turn_left(90)\n```',
        youtube_urls: ['https://www.youtube.com/watch?v=dQw4w9WgXcQ'],
        screenshot_urls: [],
        order: 1,
        created_at: new Date().toISOString(),
        updated_at: new Date().toISOString(),
      },
      {
        id: 'demo-sub-2',
        module_id: 'demo-1',
        title: 'Types of Robots',
        description: '# Types of Robots\n\nThere are many different types of robots:\n\n- Industrial robots\n- Service robots\n- Humanoid robots\n- Mobile robots',
        youtube_urls: ['https://www.youtube.com/watch?v=dQw4w9WgXcQ', 'https://www.youtube.com/watch?v=dQw4w9WgXcQ'],
        screenshot_urls: [],
        order: 2,
        created_at: new Date().toISOString(),
        updated_at: new Date().toISOString(),
      }
    ]
    
    const moduleData = await getModule(id)
    if (!moduleData) return null
    
    return {
      ...moduleData,
      sub_modules: demoSubModules.filter(sub => sub.module_id === id)
    }
  }
  
  const { data: moduleData, error: moduleError } = await supabase
    .from('modules')
    .select('*')
    .eq('id', id)
    .single()
  
  if (moduleError) {
    console.error('Error fetching module:', moduleError)
    return null
  }
  
  const { data: subModulesData, error: subModulesError } = await supabase
    .from('sub_modules')
    .select('*')
    .eq('module_id', id)
    .order('order', { ascending: true })
  
  if (subModulesError) {
    console.error('Error fetching sub-modules:', subModulesError)
    return null
  }
  
  return {
    ...moduleData,
    sub_modules: subModulesData || []
  }
}

export async function getSubModule(id: string): Promise<SubModule | null> {
  const supabase = createClient()
  
  // Check if we're using placeholder credentials
  const isPlaceholder = process.env.NEXT_PUBLIC_SUPABASE_URL?.includes('placeholder') || 
                       process.env.NEXT_PUBLIC_SUPABASE_URL === 'https://placeholder.supabase.co'
  
  if (isPlaceholder) {
    // Return demo data
    const demoSubModules = [
      {
        id: 'demo-sub-1',
        module_id: 'demo-1',
        title: 'What is a Robot?',
        description: '# What is a Robot?\n\nRobots are programmable machines that can perform tasks autonomously.\n\n```python\n# Basic robot control\nrobot.move_forward(100)\nrobot.turn_left(90)\n```\n\nThis is **demo content** - set up Supabase to add real content!',
        youtube_urls: ['https://www.youtube.com/watch?v=dQw4w9WgXcQ'],
        screenshot_urls: [],
        order: 1,
        created_at: new Date().toISOString(),
        updated_at: new Date().toISOString(),
      },
      {
        id: 'demo-sub-2',
        module_id: 'demo-1',
        title: 'Types of Robots',
        description: '# Types of Robots\n\nThere are many different types of robots:\n\n- **Industrial robots**: Used in manufacturing\n- **Service robots**: Help with daily tasks\n- **Humanoid robots**: Look and act like humans\n- **Mobile robots**: Can move around autonomously\n\n```arduino\n// Arduino robot code\nvoid setup() {\n  Serial.begin(9600);\n}\n\nvoid loop() {\n  moveForward();\n  delay(1000);\n}\n```',
        youtube_urls: ['https://www.youtube.com/watch?v=dQw4w9WgXcQ', 'https://www.youtube.com/watch?v=dQw4w9WgXcQ'],
        screenshot_urls: [],
        order: 2,
        created_at: new Date().toISOString(),
        updated_at: new Date().toISOString(),
      }
    ]
    
    return demoSubModules.find(sub => sub.id === id) || null
  }
  
  const { data, error } = await supabase
    .from('sub_modules')
    .select('*')
    .eq('id', id)
    .single()
  
  if (error) {
    console.error('Error fetching sub-module:', error)
    return null
  }
  
  return data
}

export async function getNextModuleOrder(): Promise<number> {
  const supabase = createClient()
  
  const { data, error } = await supabase
    .from('modules')
    .select('order')
    .order('order', { ascending: false })
    .limit(1)
  
  if (error) {
    console.error('Error getting max order:', error)
    return 1
  }
  
  return data && data.length > 0 ? data[0].order + 1 : 1
}

export async function createModule(module: Omit<Module, 'id' | 'created_at' | 'updated_at'>): Promise<Module | null> {
  const supabase = createClient()
  
  // Auto-assign order if not provided
  const moduleData = {
    ...module,
    order: module.order || await getNextModuleOrder()
  }
  
  const { data, error } = await supabase
    .from('modules')
    .insert(moduleData)
    .select()
    .single()
  
  if (error) {
    console.error('Error creating module:', error)
    return null
  }
  
  return data
}

export async function createSubModule(subModule: Omit<SubModule, 'id' | 'created_at' | 'updated_at'>): Promise<SubModule | null> {
  const supabase = createClient()
  
  const { data, error } = await supabase
    .from('sub_modules')
    .insert(subModule)
    .select()
    .single()
  
  if (error) {
    console.error('Error creating sub-module:', error)
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

export async function reorderModules(modules: { id: string; order: number }[]): Promise<boolean> {
  const supabase = createClient()
  
  try {
    for (const moduleItem of modules) {
      const { error } = await supabase
        .from('modules')
        .update({ order: moduleItem.order })
        .eq('id', moduleItem.id)
      
      if (error) {
        console.error('Error updating module order:', error)
        return false
      }
    }
    return true
  } catch (error) {
    console.error('Error reordering modules:', error)
    return false
  }
}

export async function updateSubModule(id: string, updates: Partial<Omit<SubModule, 'id' | 'created_at' | 'updated_at'>>): Promise<SubModule | null> {
  const supabase = createClient()
  
  const { data, error } = await supabase
    .from('sub_modules')
    .update(updates)
    .eq('id', id)
    .select()
    .single()
  
  if (error) {
    console.error('Error updating sub-module:', error)
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

export async function deleteSubModule(id: string): Promise<boolean> {
  const supabase = createClient()
  
  const { error } = await supabase
    .from('sub_modules')
    .delete()
    .eq('id', id)
  
  if (error) {
    console.error('Error deleting sub-module:', error)
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