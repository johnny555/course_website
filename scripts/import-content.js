#!/usr/bin/env node

/**
 * Bulk import script for "Become A Roboticist - Day Plans"
 * This script will:
 * 1. Read all markdown files from the content directory
 * 2. Parse YouTube links and content
 * 3. Create a module and sub-modules in the database
 */

const fs = require('fs');
const path = require('path');
const { createClient } = require('@supabase/supabase-js');

// Load environment variables
require('dotenv').config({ path: '.env.local' });

const supabaseUrl = process.env.NEXT_PUBLIC_SUPABASE_URL;
const supabaseKey = process.env.NEXT_PUBLIC_SUPABASE_ANON_KEY;

if (!supabaseUrl || !supabaseKey) {
  console.error('Missing Supabase credentials in .env.local');
  process.exit(1);
}

const supabase = createClient(supabaseUrl, supabaseKey);

// Function to extract YouTube URLs from markdown
function extractYouTubeUrls(content) {
  const youtubeRegex = /(?:https?:\/\/)?(?:www\.)?(?:youtube\.com\/watch\?v=|youtu\.be\/)([a-zA-Z0-9_-]+)/g;
  const urls = [];
  let match;
  
  while ((match = youtubeRegex.exec(content)) !== null) {
    urls.push(match[0]);
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
  return content.replace(youtubeRegex, '').trim();
}

// Function to generate a short description from content
function generateDescription(content) {
  // Get first paragraph or first 150 characters
  const firstLine = content.split('\n').find(line => line.trim() && !line.startsWith('#'));
  if (firstLine) {
    return firstLine.length > 150 ? firstLine.substring(0, 150) + '...' : firstLine;
  }
  return 'Robotics lesson content';
}

async function importContent() {
  console.log('🤖 Starting import of "Become A Roboticist" content...');
  
  const contentDir = path.join(__dirname, '..', 'Become A Roboticist - Day Plans');
  
  try {
    // Create the main module
    console.log('📚 Creating main module...');
    const { data: module, error: moduleError } = await supabase
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
    
    console.log('✅ Module created:', module.title);
    
    // Read all markdown files
    const files = fs.readdirSync(contentDir)
      .filter(file => file.endsWith('.md') && !file.includes('Goals.md') && !file.includes('Things To Be Done.md'))
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
      const title = file.replace('.md', '').replace(/^Day\s*\d+\s*-\s*/i, '');
      const description = generateDescription(cleanContent);
      
      console.log(`📝 Processing: ${title}`);
      
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
        console.error(`Error creating sub-module for ${title}:`, subModuleError);
        continue;
      }
      
      console.log(`✅ Created lesson: ${title} (${youtubeUrls.length} videos)`);
    }
    
    console.log('🎉 Import completed successfully!');
    console.log('🌐 Visit your admin panel to see the imported content');
    
  } catch (error) {
    console.error('Error during import:', error);
  }
}

// Run the import
importContent();