-- Migration: Restructure modules to support sub-modules
-- Run this after backing up your existing data

-- Step 1: Rename existing modules table to preserve data
ALTER TABLE modules RENAME TO modules_old;

-- Step 2: Create new modules table (parent modules)
CREATE TABLE modules (
  id UUID DEFAULT gen_random_uuid() PRIMARY KEY,
  title TEXT NOT NULL,
  description TEXT NOT NULL,
  thumbnail_url TEXT,
  "order" INTEGER NOT NULL DEFAULT 0,
  created_at TIMESTAMP WITH TIME ZONE DEFAULT timezone('utc'::text, now()) NOT NULL,
  updated_at TIMESTAMP WITH TIME ZONE DEFAULT timezone('utc'::text, now()) NOT NULL
);

-- Step 3: Create sub_modules table (individual pages)
CREATE TABLE sub_modules (
  id UUID DEFAULT gen_random_uuid() PRIMARY KEY,
  module_id UUID REFERENCES modules(id) ON DELETE CASCADE NOT NULL,
  title TEXT NOT NULL,
  description TEXT NOT NULL,
  youtube_urls TEXT[] DEFAULT '{}', -- Array of YouTube URLs for multiple videos
  screenshot_urls TEXT[] DEFAULT '{}',
  "order" INTEGER NOT NULL DEFAULT 0,
  created_at TIMESTAMP WITH TIME ZONE DEFAULT timezone('utc'::text, now()) NOT NULL,
  updated_at TIMESTAMP WITH TIME ZONE DEFAULT timezone('utc'::text, now()) NOT NULL
);

-- Step 4: Enable Row Level Security
ALTER TABLE modules ENABLE ROW LEVEL SECURITY;
ALTER TABLE sub_modules ENABLE ROW LEVEL SECURITY;

-- Step 5: Create RLS policies for modules
CREATE POLICY "Modules are viewable by authenticated users" ON modules
  FOR SELECT TO authenticated USING (true);

CREATE POLICY "Only admins can insert modules" ON modules
  FOR INSERT TO authenticated WITH CHECK (
    EXISTS (
      SELECT 1 FROM profiles 
      WHERE user_id = auth.uid() AND role = 'admin'
    )
  );

CREATE POLICY "Only admins can update modules" ON modules
  FOR UPDATE TO authenticated USING (
    EXISTS (
      SELECT 1 FROM profiles 
      WHERE user_id = auth.uid() AND role = 'admin'
    )
  );

CREATE POLICY "Only admins can delete modules" ON modules
  FOR DELETE TO authenticated USING (
    EXISTS (
      SELECT 1 FROM profiles 
      WHERE user_id = auth.uid() AND role = 'admin'
    )
  );

-- Step 6: Create RLS policies for sub_modules
CREATE POLICY "Sub-modules are viewable by authenticated users" ON sub_modules
  FOR SELECT TO authenticated USING (true);

CREATE POLICY "Only admins can insert sub-modules" ON sub_modules
  FOR INSERT TO authenticated WITH CHECK (
    EXISTS (
      SELECT 1 FROM profiles 
      WHERE user_id = auth.uid() AND role = 'admin'
    )
  );

CREATE POLICY "Only admins can update sub-modules" ON sub_modules
  FOR UPDATE TO authenticated USING (
    EXISTS (
      SELECT 1 FROM profiles 
      WHERE user_id = auth.uid() AND role = 'admin'
    )
  );

CREATE POLICY "Only admins can delete sub-modules" ON sub_modules
  FOR DELETE TO authenticated USING (
    EXISTS (
      SELECT 1 FROM profiles 
      WHERE user_id = auth.uid() AND role = 'admin'
    )
  );

-- Step 7: Add updated_at triggers
CREATE TRIGGER update_modules_updated_at BEFORE UPDATE ON modules
    FOR EACH ROW EXECUTE PROCEDURE update_updated_at_column();

CREATE TRIGGER update_sub_modules_updated_at BEFORE UPDATE ON sub_modules
    FOR EACH ROW EXECUTE PROCEDURE update_updated_at_column();

-- Step 8: Migrate existing data (if any)
-- This converts your existing modules into modules with single sub-modules
INSERT INTO modules (title, description, thumbnail_url, "order", created_at, updated_at)
SELECT 
  title,
  'Module description', -- You can update these manually later
  thumbnail_url,
  "order",
  created_at,
  updated_at
FROM modules_old;

-- Create sub-modules from old modules
INSERT INTO sub_modules (module_id, title, description, youtube_urls, screenshot_urls, "order", created_at, updated_at)
SELECT 
  m.id,
  mo.title || ' - Introduction', -- Sub-module gets a modified title
  mo.description,
  ARRAY[mo.youtube_url], -- Convert single URL to array
  mo.screenshot_urls,
  1, -- First sub-module in each module
  mo.created_at,
  mo.updated_at
FROM modules_old mo
JOIN modules m ON m.title = mo.title;

-- Step 9: Clean up (uncomment after verifying migration worked)
-- DROP TABLE modules_old;