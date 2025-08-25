import { createClient } from './supabase-client'

/**
 * Check if an email is approved to sign up for the course
 * Uses PostgreSQL function for security if available, falls back to direct query
 */
export async function isEmailApproved(email: string): Promise<boolean> {
  try {
    const supabase = createClient()
    const cleanEmail = email.toLowerCase().trim()
    
    // Method 1: Try using the PostgreSQL function (more secure)
    try {
      const { data, error: rpcError } = await supabase.rpc('check_email_approved', {
        email_to_check: cleanEmail
      })
      
      if (!rpcError) {
        return data === true
      }
      
      console.log('RPC method failed, falling back to direct query:', rpcError.message)
    } catch (rpcErr) {
      console.log('RPC method not available, using direct query')
    }
    
    // Method 2: Fallback to direct table query
    const { data, error } = await supabase
      .from('approved_users')
      .select('id')
      .eq('email', cleanEmail)
      .single()
    
    if (error) {
      // If it's a "no rows" error, email is not approved
      if (error.code === 'PGRST116') {
        return false
      }
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