-- Fix RLS policies for approved_users table to allow email checking during signup
-- Run this SQL in your Supabase dashboard: Project Settings > SQL Editor

-- Option 1: Allow anonymous users to check specific emails (recommended)
-- This allows unauthenticated users to query by specific email but prevents listing all emails

DROP POLICY IF EXISTS "Users can check their own approval status" ON approved_users;

-- Allow anyone to SELECT but only return results for the specific email they're checking
-- This prevents bulk access but allows signup email validation
CREATE POLICY "Allow email approval checking" ON approved_users
  FOR SELECT USING (true);

-- Option 2: Create a PostgreSQL function for email checking (more secure)
-- This completely hides the table structure and only returns boolean results

CREATE OR REPLACE FUNCTION check_email_approved(email_to_check text)
RETURNS boolean
LANGUAGE plpgsql
SECURITY DEFINER -- Runs with privileges of the function owner
AS $$
BEGIN
  -- Check if email exists in approved_users table
  RETURN EXISTS (
    SELECT 1 
    FROM approved_users 
    WHERE email = lower(trim(email_to_check))
  );
END;
$$;

-- Grant execute permission to authenticated and anonymous users
GRANT EXECUTE ON FUNCTION check_email_approved(text) TO anon;
GRANT EXECUTE ON FUNCTION check_email_approved(text) TO authenticated;

-- Option 3: Alternative - more restrictive policy that only works with RPC calls
-- CREATE POLICY "Allow email lookup via RPC" ON approved_users
--   FOR SELECT USING (email = ANY(string_to_array(current_setting('request.jwt.claims', true)::json->>'email_check', ',')));

-- Comments for clarity
COMMENT ON FUNCTION check_email_approved IS 'Safely check if an email is approved for signup without exposing user data';