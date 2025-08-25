import { createClient } from './supabase-client'

/**
 * Check if the current authenticated user has course access
 * This uses the authenticated user's session to check their email against approved_users
 */
export async function hasCurrentUserCourseAccess(): Promise<boolean> {
  try {
    const supabase = createClient()
    
    // Get current user
    const { data: { user } } = await supabase.auth.getUser()
    if (!user?.email) return false
    
    // Check if user's email is in approved_users
    // This works because RLS policy allows users to check their own email
    const { data, error } = await supabase
      .from('approved_users')
      .select('id')
      .eq('email', user.email.toLowerCase().trim())
      .single()
    
    if (error) {
      if (error.code === 'PGRST116') {
        return false // No matching record found
      }
      console.error('Error checking course access:', error)
      return false
    }
    
    return !!data
  } catch (error) {
    console.error('Error in hasCurrentUserCourseAccess:', error)
    return false
  }
}

/**
 * Check if a user has access to course content based on their email being in approved_users
 */
export async function hasUserCourseAccess(email: string): Promise<boolean> {
  try {
    const supabase = createClient()
    
    const { data, error } = await supabase
      .from('approved_users')
      .select('id, status')
      .eq('email', email.toLowerCase().trim())
      .single()
    
    if (error) {
      // If no matching record found, user doesn't have access
      if (error.code === 'PGRST116') {
        return false
      }
      console.error('Error checking course access:', error)
      return false
    }
    
    return !!data
  } catch (error) {
    console.error('Error in hasUserCourseAccess:', error)
    return false
  }
}

/**
 * Get user's course access details
 */
export async function getUserCourseAccessDetails(email: string) {
  try {
    const supabase = createClient()
    
    const { data, error } = await supabase
      .from('approved_users')
      .select('*')
      .eq('email', email.toLowerCase().trim())
      .single()
    
    if (error) {
      if (error.code === 'PGRST116') {
        return null // No access
      }
      console.error('Error getting course access details:', error)
      return null
    }
    
    return data
  } catch (error) {
    console.error('Error in getUserCourseAccessDetails:', error)
    return null
  }
}