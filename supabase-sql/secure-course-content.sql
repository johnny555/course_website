-- Secure course content at database level with RLS policies
-- Run this SQL in your Supabase dashboard: Project Settings > SQL Editor

-- Enable RLS on modules table
ALTER TABLE modules ENABLE ROW LEVEL SECURITY;

-- Enable RLS on sub_modules table  
ALTER TABLE sub_modules ENABLE ROW LEVEL SECURITY;

-- Policy: Allow admin users full access to modules
CREATE POLICY "Admins can manage all modules" ON modules
  FOR ALL USING (
    EXISTS (
      SELECT 1 FROM profiles 
      WHERE profiles.id = auth.uid() 
      AND profiles.role = 'admin'
    )
  );

-- Policy: Allow approved users to view modules
CREATE POLICY "Approved users can view modules" ON modules
  FOR SELECT USING (
    -- Check if user's email is in approved_users table
    EXISTS (
      SELECT 1 FROM approved_users
      WHERE approved_users.email = (
        SELECT email FROM auth.users 
        WHERE auth.users.id = auth.uid()
      )
    )
  );

-- Policy: Allow admin users full access to sub_modules
CREATE POLICY "Admins can manage all sub_modules" ON sub_modules
  FOR ALL USING (
    EXISTS (
      SELECT 1 FROM profiles 
      WHERE profiles.id = auth.uid() 
      AND profiles.role = 'admin'
    )
  );

-- Policy: Allow approved users to view sub_modules
CREATE POLICY "Approved users can view sub_modules" ON sub_modules
  FOR SELECT USING (
    -- Check if user's email is in approved_users table
    EXISTS (
      SELECT 1 FROM approved_users
      WHERE approved_users.email = (
        SELECT email FROM auth.users 
        WHERE auth.users.id = auth.uid()
      )
    )
  );

-- Also secure the approved_users table properly
-- Drop the old policy and create a better one
DROP POLICY IF EXISTS "Allow email approval checking" ON approved_users;
DROP POLICY IF EXISTS "Users can check their own approval status" ON approved_users;

-- Policy: Service role can manage approved_users (for imports)
CREATE POLICY "Service role can manage approved_users" ON approved_users
  FOR ALL USING (auth.jwt() ->> 'role' = 'service_role');

-- Policy: Authenticated users can check their own email status only
CREATE POLICY "Users can check own approval status" ON approved_users
  FOR SELECT USING (
    email = (
      SELECT email FROM auth.users 
      WHERE auth.users.id = auth.uid()
    )
  );

-- Create index for performance on email lookups
CREATE INDEX IF NOT EXISTS idx_auth_users_email ON auth.users(email);

-- Comments for clarity
COMMENT ON POLICY "Admins can manage all modules" ON modules IS 'Allow admin users full CRUD access to modules';
COMMENT ON POLICY "Approved users can view modules" ON modules IS 'Allow users with approved emails to view module content';
COMMENT ON POLICY "Admins can manage all sub_modules" ON sub_modules IS 'Allow admin users full CRUD access to sub_modules';
COMMENT ON POLICY "Approved users can view sub_modules" ON sub_modules IS 'Allow users with approved emails to view lesson content';
COMMENT ON POLICY "Users can check own approval status" ON approved_users IS 'Allow authenticated users to check if their own email is approved';