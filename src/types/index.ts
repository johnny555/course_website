export interface Module {
  id: string
  title: string
  description: string
  youtube_url: string
  screenshot_urls: string[]
  thumbnail_url: string | null
  order: number
  created_at: string
  updated_at: string
}

export interface Profile {
  id: string
  user_id: string
  role: 'learner' | 'admin'
  created_at: string
  updated_at: string
}

export interface User {
  id: string
  email: string
  profile?: Profile
}