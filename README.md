# Robotics Course Website

A modern, responsive web platform for hosting robotics course content built with Next.js and Supabase.

## Features

- 🔐 **Authentication** - Email/password authentication with Supabase Auth
- 👥 **Role-based Access** - Learner and Admin roles with appropriate permissions
- 📚 **Module Management** - Full CRUD operations for course modules
- 🎥 **YouTube Integration** - Embedded video content with automatic thumbnail generation
- 📝 **Markdown Support** - Rich text content with syntax highlighting for code blocks
- 🖼️ **Image Upload** - Screenshot management with Supabase Storage
- 📱 **Responsive Design** - Mobile-first design with Tailwind CSS
- ⚡ **Performance Optimized** - Built with Next.js App Router for optimal performance

## Tech Stack

- **Frontend**: Next.js 15, React 18, TypeScript
- **Styling**: Tailwind CSS
- **Backend**: Supabase (Database, Auth, Storage)
- **Deployment**: Vercel
- **Markdown**: react-markdown with syntax highlighting

## Getting Started

### Prerequisites

- Node.js 18+ 
- npm or yarn
- Supabase account

### Environment Setup

1. Clone the repository:
```bash
git clone <repository-url>
cd course_website
```

2. Install dependencies:
```bash
npm install
```

3. Set up environment variables:
Copy `.env.example` to `.env.local` and fill in your Supabase credentials:
```env
NEXT_PUBLIC_SUPABASE_URL=your_supabase_project_url
NEXT_PUBLIC_SUPABASE_ANON_KEY=your_supabase_anon_key
SUPABASE_SERVICE_ROLE_KEY=your_supabase_service_role_key
```

4. Set up Supabase database:
Run the SQL commands in `supabase-schema.sql` in your Supabase SQL editor to create the required tables and policies.

5. Create an admin user:
- Sign up through the application
- In Supabase dashboard, go to the `profiles` table
- Update the user's role from 'learner' to 'admin'

6. Run the development server:
```bash
npm run dev
```

Open [http://localhost:3000](http://localhost:3000) to view the application.

## Project Structure

```
src/
├── app/                    # Next.js App Router pages
│   ├── admin/             # Admin dashboard and module management
│   ├── login/             # Authentication pages
│   ├── modules/           # Course module pages
│   └── signup/
├── components/            # Reusable React components
│   ├── AuthProvider.tsx   # Authentication context
│   ├── Navigation.tsx     # Main navigation component
│   ├── ProtectedRoute.tsx # Route protection wrapper
│   └── ModuleForm.tsx     # Module creation/editing form
├── lib/                   # Utility functions and configurations
│   ├── supabase*.ts      # Supabase client configurations
│   ├── auth.ts           # Authentication helpers
│   ├── modules.ts        # Module data operations
│   └── storage.ts        # File upload/storage helpers
└── types/                 # TypeScript type definitions
```

## Key Features

### Authentication & Authorization
- Email/password authentication via Supabase Auth
- Role-based access control (learner/admin)
- Protected routes with automatic redirects
- Session management with middleware

### Module Management (Admin)
- Create, edit, and delete course modules
- Upload multiple screenshot images
- Set module order and thumbnails
- Real-time markdown preview
- YouTube URL validation and embed generation

### Course Content (Learners)
- Browse available modules
- Watch embedded YouTube videos
- Read formatted course content with code syntax highlighting
- View screenshots and supplementary materials
- Responsive design for mobile learning

### File Storage
- Secure image uploads to Supabase Storage
- Automatic file optimization and validation
- Public access for course materials
- Admin-only upload permissions

## Database Schema

### Tables
- `profiles`: User roles and metadata
- `modules`: Course content and structure

### Storage
- `screenshots`: Image storage bucket for module media

See `supabase-schema.sql` for complete schema with Row Level Security policies.

## Deployment

### Vercel Deployment

1. Connect your repository to Vercel
2. Set environment variables in Vercel dashboard
3. Deploy automatically on push to main branch

### Production Considerations

- Configure Supabase production settings
- Set up custom domain (if needed)
- Monitor usage to stay within free tier limits
- Set up backup strategies for data

## Free Tier Limits

This application is designed to work within free tier constraints:

- **Vercel**: Hobby plan (100GB bandwidth/month)
- **Supabase**: Free tier (500MB database, 1GB storage, 50K MAU)

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test thoroughly
5. Submit a pull request

## License

MIT License - see LICENSE file for details

## Support

For questions or issues, please create an issue in the GitHub repository.