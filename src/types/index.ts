export interface Module {
  id: string
  title: string
  description: string
  thumbnail_url: string | null
  order: number
  created_at: string
  updated_at: string
  sub_modules?: SubModule[]
}

export interface SubModule {
  id: string
  module_id: string
  title: string
  description: string
  youtube_urls: string[]
  screenshot_urls: string[]
  order: number
  created_at: string
  updated_at: string
}

export interface ModuleWithSubModules extends Module {
  sub_modules: SubModule[]
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