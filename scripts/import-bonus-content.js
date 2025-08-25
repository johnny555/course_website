#!/usr/bin/env node

/**
 * Admin bulk import script for "Become A Roboticist - Bonus Content"
 * This script uses the service role key to bypass RLS
 */

const fs = require('fs');
const path = require('path');
const { createClient } = require('@supabase/supabase-js');

// Load environment variables
require('dotenv').config({ path: '.env.local' });

const supabaseUrl = process.env.NEXT_PUBLIC_SUPABASE_URL;
const supabaseServiceKey = process.env.SUPABASE_SERVICE_ROLE_KEY;

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

// Function to clean markdown content (remove YouTube links since they'll be in the video section)
function cleanMarkdownContent(content) {
  // Remove YouTube URLs since they'll be displayed as embedded videos
  const youtubeRegex = /(?:https?:\/\/)?(?:www\.)?(?:youtube\.com\/watch\?v=|youtu\.be\/)([a-zA-Z0-9_-]+)/g;
  
  let cleaned = content.replace(youtubeRegex, '');
  // Also remove common video section headers
  cleaned = cleaned.replace(/# (?:Video walkthrough|Watch.*Video|Video).*$/gm, '');
  cleaned = cleaned.replace(/Here is a quick walkthrough.*$/gm, '');
  
  return cleaned.trim();
}

// Function to generate title from filename and directory structure
function generateTitle(filePath, baseDir) {
  const relativePath = path.relative(baseDir, filePath);
  const parts = relativePath.split(path.sep);
  
  // Get the filename without extension
  let filename = parts[parts.length - 1].replace('.md', '');
  
  // If we have a category directory, include it
  if (parts.length > 1) {
    const category = parts[0];
    
    // Handle coded filenames like "G1 - Title", "C1 - Title", etc.
    const codeMatch = filename.match(/^([A-Z]\d+)\s*-\s*(.+)$/);
    if (codeMatch) {
      const code = codeMatch[1];
      const title = codeMatch[2];
      return `${category}: ${code} - ${title}`;
    } else {
      return `${category}: ${filename}`;
    }
  }
  
  return filename;
}

// Function to recursively find all markdown files
function findMarkdownFiles(dir, baseDir = dir) {
  const files = [];
  
  try {
    const items = fs.readdirSync(dir);
    
    for (const item of items) {
      const fullPath = path.join(dir, item);
      const stat = fs.statSync(fullPath);
      
      if (stat.isDirectory()) {
        files.push(...findMarkdownFiles(fullPath, baseDir));
      } else if (item.endsWith('.md')) {
        // Check if file has YouTube content
        const content = fs.readFileSync(fullPath, 'utf-8');
        const youtubeUrls = extractYouTubeUrls(content);
        
        if (youtubeUrls.length > 0) {
          files.push({
            path: fullPath,
            relativePath: path.relative(baseDir, fullPath),
            content: content,
            youtubeUrls: youtubeUrls,
            title: generateTitle(fullPath, baseDir)
          });
        }
      }
    }
  } catch (error) {
    console.error(`Error reading directory ${dir}:`, error.message);
  }
  
  return files;
}

async function importBonusContent() {
  console.log('🎁 Starting import of "Become A Roboticist - Bonus Content"...');
  
  const bonusContentDir = path.join(__dirname, '..', 'Become A Roboticist - Day Plans', 'bonus content');
  
  try {
    // Check if module already exists
    const { data: existingModule } = await supabase
      .from('modules')
      .select('id, title')
      .eq('title', 'Become A Roboticist - Bonus Content')
      .single();
    
    let module = existingModule;
    
    if (!existingModule) {
      // Create the bonus module
      console.log('📚 Creating bonus content module...');
      const { data: newModule, error: moduleError } = await supabase
        .from('modules')
        .insert({
          title: 'Become A Roboticist - Bonus Content',
          description: 'Additional robotics content covering advanced topics including Gazebo plugins, computer vision, robot modeling, real-world deployment, and advanced manipulation techniques.',
          thumbnail_url: 'https://img.youtube.com/vi/CPQf-HPnyHc/maxresdefault.jpg', // Use G1 video thumbnail
          order: 2
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
    
    // Find all markdown files with videos in the bonus content directory
    const files = findMarkdownFiles(bonusContentDir);
    
    // Sort files by category and then by filename
    files.sort((a, b) => {
      const categoryA = a.relativePath.split(path.sep)[0];
      const categoryB = b.relativePath.split(path.sep)[0];
      
      if (categoryA !== categoryB) {
        return categoryA.localeCompare(categoryB);
      }
      
      return a.title.localeCompare(b.title);
    });
    
    console.log(`📄 Found ${files.length} markdown files with videos to import`);
    
    // Process each file
    for (let i = 0; i < files.length; i++) {
      const file = files[i];
      
      // Extract information
      const cleanContent = cleanMarkdownContent(file.content);
      
      console.log(`📝 Processing: ${file.title}`);
      if (file.youtubeUrls.length > 0) {
        console.log(`   📺 Found ${file.youtubeUrls.length} video(s): ${file.youtubeUrls.join(', ')}`);
      }
      
      // Check if sub-module already exists
      const { data: existingSubModule } = await supabase
        .from('sub_modules')
        .select('id')
        .eq('module_id', module.id)
        .eq('title', file.title)
        .single();
      
      if (existingSubModule) {
        console.log(`   ⏭️  Skipping (already exists): ${file.title}`);
        continue;
      }
      
      // Create sub-module
      const { data: subModule, error: subModuleError } = await supabase
        .from('sub_modules')
        .insert({
          module_id: module.id,
          title: file.title,
          description: cleanContent,
          youtube_urls: file.youtubeUrls,
          screenshot_urls: [],
          order: i + 1
        })
        .select()
        .single();
      
      if (subModuleError) {
        console.error(`❌ Error creating sub-module for ${file.title}:`, subModuleError);
        continue;
      }
      
      console.log(`✅ Created lesson: ${file.title} (${file.youtubeUrls.length} videos)`);
    }
    
    console.log('🎉 Bonus content import completed successfully!');
    console.log(`📊 Module created with ${files.length} bonus lessons`);
    console.log('🌐 Visit your admin panel at http://localhost:3000/admin to see the imported content');
    
  } catch (error) {
    console.error('❌ Error during import:', error);
  }
}

// Run the import
importBonusContent();