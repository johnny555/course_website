#!/usr/bin/env node

/**
 * Import approved users from paid_subs.csv
 * This script will also create the table if it doesn't exist
 */

const fs = require('fs');
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

function parseCSV(csvContent) {
  const lines = csvContent.trim().split('\n');
  const headers = lines[0].split(',');
  const data = [];

  for (let i = 1; i < lines.length; i++) {
    const line = lines[i];
    if (line.trim() === '') continue;
    
    // Simple CSV parsing (handles basic cases)
    const values = [];
    let current = '';
    let inQuotes = false;
    
    for (let j = 0; j < line.length; j++) {
      const char = line[j];
      if (char === '"') {
        inQuotes = !inQuotes;
      } else if (char === ',' && !inQuotes) {
        values.push(current);
        current = '';
      } else {
        current += char;
      }
    }
    values.push(current); // Add the last value
    
    // Create object from headers and values
    const row = {};
    headers.forEach((header, index) => {
      row[header] = values[index] || '';
    });
    
    data.push(row);
  }
  
  return data;
}

async function checkAndCreateTable() {
  console.log('🔍 Checking if approved_users table exists...');
  
  try {
    // Try to query the table
    const { data, error } = await supabase
      .from('approved_users')
      .select('id')
      .limit(1);
    
    if (!error) {
      console.log('✅ approved_users table already exists');
      return true;
    }
    
    if (error.code === 'PGRST106' || error.message.includes('does not exist')) {
      console.log('❌ Table does not exist. Please create it manually in Supabase:');
      console.log(`
🔧 Go to https://supabase.com/dashboard/project/${supabaseUrl.split('//')[1].split('.')[0]}/sql

Run this SQL:

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
  imported_at timestamp with time zone DEFAULT now()
);

CREATE INDEX idx_approved_users_email ON approved_users(email);
CREATE INDEX idx_approved_users_status ON approved_users(status);

ALTER TABLE approved_users ENABLE ROW LEVEL SECURITY;

CREATE POLICY "Service role can manage approved users" ON approved_users
  FOR ALL USING (auth.jwt() ->> 'role' = 'service_role');
      `);
      return false;
    }
    
    console.error('Unexpected error checking table:', error);
    return false;
    
  } catch (error) {
    console.error('Error during table check:', error);
    return false;
  }
}

async function importApprovedUsers() {
  console.log('📧 Starting import of approved users...');
  
  // Check if table exists
  const tableExists = await checkAndCreateTable();
  if (!tableExists) {
    console.log('⏸️  Please create the table first, then run this script again.');
    return;
  }
  
  try {
    // Read and parse CSV
    const csvPath = 'paid_subs.csv';
    if (!fs.existsSync(csvPath)) {
      console.error('❌ paid_subs.csv file not found');
      return;
    }
    
    const csvContent = fs.readFileSync(csvPath, 'utf-8');
    const userData = parseCSV(csvContent);
    
    console.log(`📊 Found ${userData.length} users to import`);
    
    // Filter for users with valid emails (regardless of status)
    const validUsers = userData.filter(user => user.email && user.email.includes('@'));
    
    console.log(`✅ Importing ${validUsers.length} users with valid emails (all statuses included)`);
    
    // Import users in batches
    const batchSize = 10;
    let imported = 0;
    let skipped = 0;
    
    for (let i = 0; i < validUsers.length; i += batchSize) {
      const batch = validUsers.slice(i, i + batchSize);
      
      for (const user of batch) {
        try {
          const { data, error } = await supabase
            .from('approved_users')
            .insert({
              email: user.email.toLowerCase().trim(),
              first_name: user.first_name || null,
              status: user.status || 'active',
              tags: user.tags || null,
              city: user.city || null,
              state: user.state || null,
              country: user.country || null,
              created_at: user.created_at ? new Date(user.created_at).toISOString() : null,
            });
          
          if (error) {
            if (error.code === '23505') { // Unique constraint violation
              console.log(`   ⏭️  Skipped ${user.email} (already exists)`);
              skipped++;
            } else {
              console.error(`   ❌ Error importing ${user.email}:`, error.message);
            }
          } else {
            console.log(`   ✅ Imported ${user.email}`);
            imported++;
          }
        } catch (err) {
          console.error(`   ❌ Error importing ${user.email}:`, err.message);
        }
      }
    }
    
    console.log('\n🎉 Import completed!');
    console.log(`📊 Statistics:`);
    console.log(`   - Imported: ${imported} users`);
    console.log(`   - Skipped: ${skipped} users`);
    console.log(`   - Total processed: ${imported + skipped} users`);
    
  } catch (error) {
    console.error('❌ Error during import:', error);
  }
}

// Run the import
importApprovedUsers();