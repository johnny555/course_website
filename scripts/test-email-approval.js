#!/usr/bin/env node

/**
 * Test script to verify email approval checking works correctly
 */

const { createClient } = require('@supabase/supabase-js');

// Load environment variables
require('dotenv').config({ path: '.env.local' });

const supabaseUrl = process.env.NEXT_PUBLIC_SUPABASE_URL;
const supabaseAnonKey = process.env.NEXT_PUBLIC_SUPABASE_ANON_KEY;

if (!supabaseUrl || !supabaseAnonKey) {
  console.error('Missing Supabase credentials in .env.local');
  process.exit(1);
}

// Use the anon key (same as client-side) to test RLS policies
const supabase = createClient(supabaseUrl, supabaseAnonKey);

async function testEmailApproval() {
  console.log('🧪 Testing email approval checking...\n');
  
  // Test with a known approved email
  const approvedEmail = 'badwolf.johnnyv@gmail.com';
  
  // Test with a non-approved email
  const notApprovedEmail = 'test@example.com';
  
  console.log('📧 Testing approved email:', approvedEmail);
  
  try {
    // Method 1: Test RPC function
    try {
      const { data: rpcResult, error: rpcError } = await supabase.rpc('check_email_approved', {
        email_to_check: approvedEmail
      });
      
      if (!rpcError) {
        console.log('✅ RPC function result:', rpcResult);
      } else {
        console.log('❌ RPC function error:', rpcError.message);
      }
    } catch (err) {
      console.log('❌ RPC function not available:', err.message);
    }
    
    // Method 2: Test direct table query
    const { data, error } = await supabase
      .from('approved_users')
      .select('id')
      .eq('email', approvedEmail.toLowerCase().trim())
      .single();
    
    if (error) {
      console.log('❌ Direct query error:', error.message, 'Code:', error.code);
    } else {
      console.log('✅ Direct query result: Found user with ID', data.id);
    }
    
  } catch (error) {
    console.error('❌ Test failed:', error);
  }
  
  console.log('\n📧 Testing non-approved email:', notApprovedEmail);
  
  try {
    // Test with non-approved email
    const { data, error } = await supabase
      .from('approved_users')
      .select('id')
      .eq('email', notApprovedEmail)
      .single();
    
    if (error) {
      if (error.code === 'PGRST116') {
        console.log('✅ Correctly rejected non-approved email (no rows found)');
      } else {
        console.log('❌ Unexpected error:', error.message);
      }
    } else {
      console.log('❌ Non-approved email was found (this should not happen):', data);
    }
    
  } catch (error) {
    console.error('❌ Test failed:', error);
  }
  
  console.log('\n🔍 Testing RLS by trying to list all emails...');
  
  try {
    const { data, error } = await supabase
      .from('approved_users')
      .select('email')
      .limit(5);
    
    if (error) {
      console.log('✅ RLS correctly blocked listing all emails:', error.message);
    } else {
      console.log('⚠️  RLS may not be working - got results:', data?.length, 'emails');
      if (data && data.length > 0) {
        console.log('First few emails:', data.map(u => u.email));
      }
    }
  } catch (error) {
    console.log('✅ RLS correctly blocked with error:', error.message);
  }
}

testEmailApproval();