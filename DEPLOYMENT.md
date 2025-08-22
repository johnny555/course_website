# Deployment Guide

This guide walks you through deploying the Robotics Course Website to production.

## Prerequisites

1. **Supabase Account**: Create a free account at [supabase.com](https://supabase.com)
2. **Vercel Account**: Create a free account at [vercel.com](https://vercel.com)
3. **GitHub Repository**: Push your code to a GitHub repository

## Step 1: Set Up Supabase Project

1. **Create a new Supabase project**:
   - Go to [app.supabase.com](https://app.supabase.com)
   - Click "New project"
   - Choose your organization
   - Enter project name: "robotics-course"
   - Generate a secure database password
   - Select a region close to your users
   - Click "Create new project"

2. **Configure the database**:
   - Go to the SQL Editor in your Supabase dashboard
   - Copy and paste the contents of `supabase-schema.sql`
   - Run the script to create tables, policies, and triggers

3. **Set up Storage**:
   - Go to Storage in your Supabase dashboard
   - The "screenshots" bucket should be created automatically by the SQL script
   - If not, create it manually with public access

4. **Get your project credentials**:
   - Go to Settings → API
   - Copy the Project URL and anon public key
   - Copy the service_role secret key (you'll need this for admin operations)

## Step 2: Update Environment Variables

1. **Update `.env.local`** for local development:
```env
NEXT_PUBLIC_SUPABASE_URL=https://your-project-ref.supabase.co
NEXT_PUBLIC_SUPABASE_ANON_KEY=your-anon-key
SUPABASE_SERVICE_ROLE_KEY=your-service-role-key
```

2. **Test locally**:
```bash
npm run dev
```

## Step 3: Create Admin User

1. **Sign up through the app**:
   - Go to your local app (http://localhost:3000)
   - Click "Sign Up" and create an account
   - Check your email for confirmation link

2. **Set admin role**:
   - Go to Supabase dashboard → Authentication → Users
   - Find your user and copy the User ID
   - Go to Table Editor → profiles
   - Update your user's role from 'learner' to 'admin'

## Step 4: Deploy to Vercel

1. **Connect to Vercel**:
   - Go to [vercel.com](https://vercel.com)
   - Click "New Project"
   - Import your GitHub repository
   - Select the repository containing your app

2. **Configure environment variables**:
   - In the deployment configuration, add Environment Variables:
     ```
     NEXT_PUBLIC_SUPABASE_URL = https://your-project-ref.supabase.co
     NEXT_PUBLIC_SUPABASE_ANON_KEY = your-anon-key
     SUPABASE_SERVICE_ROLE_KEY = your-service-role-key
     ```

3. **Deploy**:
   - Click "Deploy"
   - Wait for the build to complete
   - Your app will be available at `https://your-project-name.vercel.app`

## Step 5: Configure Domain (Optional)

1. **Custom domain**:
   - Go to your Vercel project dashboard
   - Navigate to Settings → Domains
   - Add your custom domain
   - Configure DNS records as instructed

## Step 6: Post-Deployment Setup

1. **Test all functionality**:
   - Sign up flow
   - Login/logout
   - Module creation (admin)
   - Image uploads
   - YouTube embeds

2. **Create sample content**:
   - Log in as admin
   - Create 2-3 sample modules
   - Upload screenshots
   - Test the complete user experience

## Step 7: Configure Production Settings

1. **Supabase Settings**:
   - Add your Vercel domain to allowed origins in Supabase
   - Go to Authentication → Settings
   - Add your domain to "Site URL" and "Redirect URLs"

2. **Security considerations**:
   - Review Row Level Security policies
   - Ensure file upload limits are appropriate
   - Monitor usage to stay within free tier limits

## Monitoring and Maintenance

### Usage Monitoring
- **Supabase**: Monitor database usage, storage, and bandwidth
- **Vercel**: Track function executions and bandwidth usage

### Free Tier Limits
- **Supabase Free Tier**: 500MB database, 1GB storage, 50K MAU
- **Vercel Hobby**: 100GB bandwidth/month, 100 deployments

### Backup Strategy
1. **Database backups**: Set up regular database backups in Supabase
2. **Storage backups**: Consider backing up uploaded images
3. **Code backups**: Ensure your code is committed to version control

## Troubleshooting

### Common Issues

1. **Build failures**:
   - Check environment variables are set correctly
   - Ensure all dependencies are installed
   - Review build logs for specific errors

2. **Authentication issues**:
   - Verify Supabase URL and keys
   - Check redirect URLs configuration
   - Ensure middleware is working correctly

3. **File upload failures**:
   - Check storage bucket permissions
   - Verify file size limits
   - Ensure storage policies are configured

4. **Database connection issues**:
   - Verify database is running
   - Check connection string
   - Review network policies

### Support Resources
- [Supabase Documentation](https://supabase.com/docs)
- [Vercel Documentation](https://vercel.com/docs)
- [Next.js Documentation](https://nextjs.org/docs)

## Production Checklist

- [ ] Supabase project created and configured
- [ ] Database schema deployed
- [ ] Storage bucket created
- [ ] Admin user created and role assigned
- [ ] Environment variables configured
- [ ] App deployed to Vercel
- [ ] Domain configured (if using custom domain)
- [ ] Authentication flow tested
- [ ] Admin functionality tested
- [ ] File uploads tested
- [ ] Sample content created
- [ ] Monitoring set up
- [ ] Backup strategy implemented

Your Robotics Course Website is now live and ready for learners!