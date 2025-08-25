-- Create approved_users table for managing course access
-- Run this SQL in your Supabase dashboard: Project Settings > SQL Editor

CREATE TABLE approved_users (
  id uuid DEFAULT gen_random_uuid() PRIMARY KEY,
  email text UNIQUE NOT NULL,
  first_name text,
  status text DEFAULT 'active',
  tags text,
  city text,
  state text,  
  country text,
  created_at timestamp with time zone,
  imported_at timestamp with time zone DEFAULT now(),
  CONSTRAINT email_format CHECK (email ~* '^[A-Za-z0-9._%+-]+@[A-Za-z0-9.-]+\.[A-Za-z]{2,}$')
);

-- Create indexes for better performance
CREATE INDEX idx_approved_users_email ON approved_users(email);
CREATE INDEX idx_approved_users_status ON approved_users(status);

-- Enable Row Level Security
ALTER TABLE approved_users ENABLE ROW LEVEL SECURITY;

-- Create policies
CREATE POLICY "Service role can manage approved users" ON approved_users
  FOR ALL USING (auth.jwt() ->> 'role' = 'service_role');

CREATE POLICY "Users can check their own approval status" ON approved_users
  FOR SELECT USING (auth.jwt() ->> 'email' = email);

-- Add some comments for clarity
COMMENT ON TABLE approved_users IS 'Users who are approved to sign up for the course';
COMMENT ON COLUMN approved_users.email IS 'User email address (unique)';
COMMENT ON COLUMN approved_users.status IS 'User status: active, cancelled, etc.';
COMMENT ON COLUMN approved_users.tags IS 'Course tags and cohort information';