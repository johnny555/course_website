#!/usr/bin/env node

/**
 * Create approved_users table in Supabase
 * This table will store emails of users who are approved to sign up
 */

const { createClient } = require('@supabase/supabase-js');

// Load environment variables
require('dotenv').config({ path: '.env.local' });

const supabaseUrl = process.env.NEXT_PUBLIC_SUPABASE_URL;
const supabaseServiceKey = process.env.SUPABASE_SERVICE_ROLE_KEY;

if (!supabaseUrl || !supabaseServiceKey) {
  console.error('Missing Supabase credentials. Need NEXT_PUBLIC_SUPABASE_URL and SUPABASE_SERVICE_ROLE_KEY in .env.local');
  process.exit(1);
}

const supabase = createClient(supabaseUrl, supabaseServiceKey);

async function createApprovedUsersTable() {
  console.log('🔐 Creating approved_users table...');
  
  try {
    // Create the table using SQL
    const { error } = await supabase.rpc('exec_sql', {
      sql: `
        -- Create approved_users table
        CREATE TABLE IF NOT EXISTS approved_users (
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
          CONSTRAINT email_format CHECK (email ~* '^[A-Za-z0-9._%+-]+@[A-Za-z0-9.-]+\\.[A-Za-z]{2,}$')
        );

        -- Create index on email for fast lookups
        CREATE INDEX IF NOT EXISTS idx_approved_users_email ON approved_users(email);
        
        -- Create index on status
        CREATE INDEX IF NOT EXISTS idx_approved_users_status ON approved_users(status);

        -- Enable RLS (Row Level Security)
        ALTER TABLE approved_users ENABLE ROW LEVEL SECURITY;

        -- Create policy to allow service role to manage all data
        DROP POLICY IF EXISTS "Service role can manage approved users" ON approved_users;
        CREATE POLICY "Service role can manage approved users" ON approved_users
          FOR ALL USING (auth.jwt() ->> 'role' = 'service_role');

        -- Create policy to allow authenticated users to read their own approval status
        DROP POLICY IF EXISTS "Users can check their own approval status" ON approved_users;
        CREATE POLICY "Users can check their own approval status" ON approved_users
          FOR SELECT USING (auth.jwt() ->> 'email' = email);
      `
    });

    if (error) {
      console.error('❌ Error creating table:', error);
      return false;
    }

    console.log('✅ approved_users table created successfully!');
    return true;

  } catch (error) {
    console.error('❌ Error during table creation:', error);
    return false;
  }
}

// Alternative approach using direct SQL if rpc doesn't work
async function createTableDirectSQL() {
  console.log('🔐 Creating approved_users table with direct SQL...');
  
  try {
    // Check if we can create the table by trying a simple query first
    const { data, error } = await supabase
      .from('approved_users')
      .select('count(*)')
      .limit(1);
    
    if (!error) {
      console.log('✅ approved_users table already exists!');
      return true;
    }
    
    console.log('Table does not exist, please create it manually in Supabase Dashboard:');
    console.log(`
-- SQL to run in Supabase SQL Editor:

CREATE TABLE IF NOT EXISTS approved_users (
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
  CONSTRAINT email_format CHECK (email ~* '^[A-Za-z0-9._%+-]+@[A-Za-z0-9.-]+\\\\.[A-Za-z]{2,}$')
);

-- Create indexes
CREATE INDEX IF NOT EXISTS idx_approved_users_email ON approved_users(email);
CREATE INDEX IF NOT EXISTS idx_approved_users_status ON approved_users(status);

-- Enable RLS
ALTER TABLE approved_users ENABLE ROW LEVEL SECURITY;

-- Create policies
CREATE POLICY "Service role can manage approved users" ON approved_users
  FOR ALL USING (auth.jwt() ->> 'role' = 'service_role');

CREATE POLICY "Users can check their own approval status" ON approved_users
  FOR SELECT USING (auth.jwt() ->> 'email' = email);
    `);
    
    return false;
    
  } catch (error) {
    console.error('❌ Error during table check:', error);
    return false;
  }
}

// Run the table creation
async function main() {
  const success = await createApprovedUsersTable();
  if (!success) {
    await createTableDirectSQL();
  }
}

main();