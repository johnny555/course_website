#!/usr/bin/env node

/**
 * Admin bulk import script for "Become A Roboticist - Day Plans"
 * This script uses the service role key to bypass RLS
 */

const fs = require('fs');
const path = require('path');
const { createClient } = require('@supabase/supabase-js');

// Load environment variables
require('dotenv').config({ path: '.env.local' });

const supabaseUrl = process.env.NEXT_PUBLIC_SUPABASE_URL;
const supabaseServiceKey = process.env.SUPABASE_SERVICE_ROLE_KEY; // Need this for admin operations

if (!supabaseUrl || !supabaseServiceKey) {
  console.error('Missing Supabase credentials. Need NEXT_PUBLIC_SUPABASE_URL and SUPABASE_SERVICE_ROLE_KEY in .env.local');
  console.error('You can find the service role key in your Supabase project settings under API > service_role');
  process.exit(1);
}

const supabase = createClient(supabaseUrl, supabaseServiceKey);

// Function to extract YouTube URLs from markdown
function extractYouTubeUrls(content) {
  const youtubeRegex = /(?:https?:\/\/)?(?:www\.)?(?:youtube\.com\/watch\?v=|youtu\.be\/)([a-zA-Z0-9_-]+)/g;
  const urls = [];
  let match;
  
  while ((match = youtubeRegex.exec(content)) !== null) {
    // Normalize to full YouTube URL
    const videoId = match[1];
    urls.push(`https://www.youtube.com/watch?v=${videoId}`);
  }
  
  return urls;
}

// Function to parse day number from filename
function parseDayNumber(filename) {
  const dayMatch = filename.match(/Day\s*(\d+)/i);
  return dayMatch ? parseInt(dayMatch[1]) : 999; // Put non-day files at the end
}

// Function to clean markdown content (remove YouTube links since they'll be in the video section)
function cleanMarkdownContent(content) {
  // Remove YouTube URLs since they'll be displayed as embedded videos
  const youtubeRegex = /(?:https?:\/\/)?(?:www\.)?(?:youtube\.com\/watch\?v=|youtu\.be\/)([a-zA-Z0-9_-]+)/g;
  
  // Also remove the video section headers that typically precede YouTube links
  let cleaned = content.replace(youtubeRegex, '');
  cleaned = cleaned.replace(/# (?:Day \d+ Video|Watch Day \d+ Video|Video)\s*/gi, '');
  cleaned = cleaned.replace(/Here is a quick walkthrough.*$/gm, '');
  
  return cleaned.trim();
}

// Function to generate title from filename
function generateTitle(filename) {
  let title = filename.replace('.md', '');
  
  // Remove "Day X - " prefix but keep the day number info
  const dayMatch = title.match(/^Day\s*(\d+)\s*-\s*(.+)$/i);
  if (dayMatch) {
    const dayNum = dayMatch[1];
    const content = dayMatch[2];
    title = `Day ${dayNum}: ${content}`;
  }
  
  return title;
}

async function importContent() {
  console.log('🤖 Starting import of "Become A Roboticist" content...');
  
  const contentDir = path.join(__dirname, '..', 'Become A Roboticist - Day Plans');
  
  try {
    // Check if module already exists
    const { data: existingModule } = await supabase
      .from('modules')
      .select('id, title')
      .eq('title', 'Become A Roboticist - 28 Day Challenge')
      .single();
    
    let module = existingModule;
    
    if (!existingModule) {
      // Create the main module
      console.log('📚 Creating main module...');
      const { data: newModule, error: moduleError } = await supabase
        .from('modules')
        .insert({
          title: 'Become A Roboticist - 28 Day Challenge',
          description: 'A comprehensive 28-day journey to learn robotics from the ground up. Master ROS 2, robot simulation, CAD design, autonomous navigation, and robot arm control through hands-on daily lessons.',
          thumbnail_url: 'https://img.youtube.com/vi/njl_vYRr8vs/maxresdefault.jpg', // Use Day 1 thumbnail
          order: 1
        })
        .select()
        .single();
      
      if (moduleError) {
        console.error('Error creating module:', moduleError);
        return;
      }
      
      module = newModule;
      console.log('✅ Module created:', module.title);
    } else {
      console.log('📚 Using existing module:', module.title);
    }
    
    // Read all markdown files
    const files = fs.readdirSync(contentDir)
      .filter(file => {
        return file.endsWith('.md') && 
               !file.includes('Goals.md') && 
               !file.includes('Things To Be Done.md') &&
               !file.includes('Script for teaser') &&
               !file.includes('Bonuses') &&
               !file.includes('Making Realsense');
      })
      .sort((a, b) => {
        // Sort by day number
        const dayA = parseDayNumber(a);
        const dayB = parseDayNumber(b);
        return dayA - dayB;
      });
    
    console.log(`📄 Found ${files.length} markdown files to import`);
    
    // Process each file
    for (let i = 0; i < files.length; i++) {
      const file = files[i];
      const filePath = path.join(contentDir, file);
      const content = fs.readFileSync(filePath, 'utf-8');
      
      // Extract information
      const youtubeUrls = extractYouTubeUrls(content);
      const cleanContent = cleanMarkdownContent(content);
      const title = generateTitle(file);
      
      console.log(`📝 Processing: ${title}`);
      if (youtubeUrls.length > 0) {
        console.log(`   📺 Found ${youtubeUrls.length} video(s): ${youtubeUrls.join(', ')}`);
      }
      
      // Check if sub-module already exists
      const { data: existingSubModule } = await supabase
        .from('sub_modules')
        .select('id')
        .eq('module_id', module.id)
        .eq('title', title)
        .single();
      
      if (existingSubModule) {
        console.log(`   ⏭️  Skipping (already exists): ${title}`);
        continue;
      }
      
      // Create sub-module
      const { data: subModule, error: subModuleError } = await supabase
        .from('sub_modules')
        .insert({
          module_id: module.id,
          title: title,
          description: cleanContent,
          youtube_urls: youtubeUrls,
          screenshot_urls: [],
          order: i + 1
        })
        .select()
        .single();
      
      if (subModuleError) {
        console.error(`❌ Error creating sub-module for ${title}:`, subModuleError);
        continue;
      }
      
      console.log(`✅ Created lesson: ${title} (${youtubeUrls.length} videos)`);
    }
    
    console.log('🎉 Import completed successfully!');
    console.log(`📊 Module created with ${files.length} lessons`);
    console.log('🌐 Visit your admin panel at http://localhost:3000/admin to see the imported content');
    
  } catch (error) {
    console.error('❌ Error during import:', error);
  }
}

// Run the import
importContent();