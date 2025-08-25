import { createClient } from './supabase-client'

/**
 * Check if an email is approved to sign up for the course
 */
export async function isEmailApproved(email: string): Promise<boolean> {
  try {
    const supabase = createClient()
    
    const { data, error } = await supabase
      .from('approved_users')
      .select('id, status')
      .eq('email', email.toLowerCase().trim())
      .single()
    
    if (error) {
      console.error('Error checking approved user:', error)
      return false
    }
    
    return !!data
  } catch (error) {
    console.error('Error in isEmailApproved:', error)
    return false
  }
}

/**
 * Get approved user details
 */
export async function getApprovedUser(email: string) {
  try {
    const supabase = createClient()
    
    const { data, error } = await supabase
      .from('approved_users')
      .select('*')
      .eq('email', email.toLowerCase().trim())
      .single()
    
    if (error) {
      console.error('Error getting approved user:', error)
      return null
    }
    
    return data
  } catch (error) {
    console.error('Error in getApprovedUser:', error)
    return null
  }
}