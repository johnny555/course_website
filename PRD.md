Product Requirements Document (PRD): Robotics Course Hosting Website MVP1. Document Overview1.1 PurposeThis PRD outlines the requirements for building a Minimum Viable Product (MVP) of a robotics course hosting website. The site will serve as a platform for users to access online course content after logging in. The MVP focuses on a basic scaffold with essential pages and functionality, prioritizing simplicity to stay within free tiers of Vercel and Supabase. This update includes an admin backend for uploading and managing course content.1.2 ScopeIn Scope: Basic user authentication (email/password for MVP), public overview and login pages, protected module overview and detail pages, embedding YouTube videos, rendering markdown text (including code blocks), admin backend for content management (CRUD on modules, including uploading screenshot images to Supabase Storage).

Out of Scope: Advanced OAuth integration (e.g., Google login—planned for future iterations), user registration flows beyond basic signup, payment processing, advanced analytics, mobile responsiveness beyond basic, search functionality, user progress tracking.

Admin Scope: Role-based access for admins to create/edit/delete modules, upload YouTube links, markdown text, and screenshot images. Content will be stored in Supabase database and storage.



1.3 Version HistoryVersion 1.0: Initial draft for MVP development (August 22, 2025).

Version 1.1: Added admin backend for content upload and management (August 22, 2025).



2. Business Goals and Objectives2.1 GoalsProvide a simple, accessible platform for users to view robotics course content.

Enable secure login to access protected course modules.

Allow admins to easily upload and manage content (YouTube links, markdown, screenshots) without manual database intervention.

Demonstrate core functionality (video embedding, markdown rendering, content management) to validate the concept.

Minimize costs by leveraging free tiers of Vercel (hosting) and Supabase (auth, database, storage).



2.2 Success MetricsMVP deployed and functional within free tier limits (e.g., no exceeding Supabase's free database rows, storage, or Vercel's bandwidth).

Users can sign up, log in, and access at least 2-3 sample modules without errors.

Admins can upload new modules with all content types and see them reflected in user views.

Page load times under 3 seconds on standard connections.

Positive feedback from initial testers on ease of navigation, content display, and admin usability.



3. Target AudiencePrimary Users: Hobbyists, students, and beginners interested in robotics who want free access to structured course content.

Admin Users: Course creators (e.g., the product owner) who need to manage content.

User Personas:Learner Persona: 18-35 years old, tech-savvy but new to robotics, accesses site via desktop or mobile browser, expects simple login and clear content layout.

Admin Persona: Tech-familiar individual (e.g., developer/educator), accesses via desktop, expects straightforward forms for uploading content.


4. Technical Stack and Constraints4.1 StackFrontend/Hosting: Next.js (React-based) deployed on Vercel for serverless hosting and easy deployment.

Backend/Database/Auth/Storage: Supabase for PostgreSQL database, authentication (use Supabase Auth for email/password), storage for screenshot images (Supabase Storage buckets).

Other Libraries:React-Markdown or similar for rendering markdown with code block support (e.g., syntax highlighting via Prism.js or Rehype).

YouTube embed via iframe or react-youtube component.

File uploads: Use Supabase JS client for storage uploads.

Basic styling: Tailwind CSS for rapid development.

Form handling: React Hook Form or similar for admin upload forms.


4.2 ConstraintsFree Tiers Only:Vercel: Hobby plan (unlimited sites, 100GB bandwidth/month, serverless functions up to 10s execution).

Supabase: Free tier (500MB database, 1GB file storage, 50,000 monthly active users for auth, limited compute—ensure queries are optimized; storage for images should stay under 1GB).


No Custom Domains for MVP: Use Vercel/Supabase default URLs.

Scalability: Design for low traffic (under 100 users/day initially).

Security: Use Supabase's built-in row-level security (RLS) to protect user data, course content, and admin access. Restrict storage uploads to authenticated admins.

Browser Support: Modern browsers (Chrome, Firefox, Safari latest versions).



5. Functional Requirements5.1 User FlowsPublic Access:Visitor lands on Course Overview Page.

Navigates to Login Page to access protected content.


Authentication:User signs up or logs in with email/password.

Redirect to Module Overview Page on success (or Admin Dashboard if admin role).


Protected Access (Learners):View list of modules.

Click a module to view details (video + markdown + screenshots).


Admin Access:After login, redirect to Admin Dashboard if user has admin role.

Create new module: Upload YouTube link, markdown text, screenshot images.

Edit/Delete existing modules.


5.2 Pages and FeaturesUse a simple navigation bar (e.g., Home, Login/Logout, Admin Dashboard if applicable) across all pages.Page

Access Level

Description

Key Components

Course Overview Page

Public

Landing page introducing the robotics course.

- Hero section with course title, description, and call-to-action button to login/signup.
- Static text/images about course benefits.
- No dynamic content.

Course Login Page

Public

Handles user authentication.

- Form for email/password login.
- Link to signup form (email/password).
- Error handling for invalid credentials.
- Redirect based on role: Module Overview for learners, Admin Dashboard for admins.
- Forgot password link (basic Supabase reset flow).

Course Module Overview Page

Protected (Logged In)

Lists all available modules.

- Grid or list of module cards (title, thumbnail, short description).
- Clickable to navigate to module detail.
- Fetch modules from Supabase database.
- Logout button in nav.

Module Detail Page

Protected (Logged In)

In-depth view of a single module.

- Embedded YouTube video (iframe with autoplay disabled).
- Markdown-rendered text description (support headers, lists, code blocks with syntax highlighting for languages like Python/C++).
- Display screenshot images (responsive gallery or inline embeds).
- Back button to Module Overview.
- Dynamic routing based on module ID (e.g., /modules/[id]).

Admin Dashboard Page

Protected (Admin Only)

Overview for managing modules.

- List of existing modules with edit/delete buttons.
- Button to create new module.
- Fetch modules with admin privileges.

Admin Module Create/Edit Page

Protected (Admin Only)

Form for adding or updating a module.

- Inputs: Title (text), YouTube URL (text), Markdown description (textarea with preview option).
- File upload for screenshot images (multiple files, e.g., PNG/JPG, max 5 per module; upload to Supabase Storage and store URLs).
- Thumbnail selection (optional, from uploaded images).
- Order field (integer for sorting).
- Submit button to save to database.
- Validation: Required fields, valid URL, file size limits (e.g., <5MB per image to fit free tier).
- Dynamic routing for edit (e.g., /admin/modules/[id]).



5.3 Data Model (Supabase Database)Tables:Users: Handled by Supabase Auth (auto-managed). Add a profiles table for roles:user_id (UUID, foreign key to auth.users).

role (string, e.g., 'learner' or 'admin'; default 'learner').


Modules:id (UUID, primary key).

title (string).

description (text, markdown format).

youtube_url (string, e.g., "https://www.youtube.com/embed/video_id").

screenshot_urls (array of strings, URLs from Supabase Storage).

thumbnail_url (string, optional, selected from screenshot_urls).

order (integer, for sorting in overview).


Initial Data: Seed admin user manually via Supabase dashboard (set role to 'admin'). Sample modules can now be added via admin backend.

Queries: Use Supabase JS client for fetching/upserting modules (authenticated queries; RLS to allow read for all auth users, write for admins only).



5.4 Authentication and AuthorizationUse Supabase Auth for email/password.

Session management: Store session in cookies/local storage via Supabase client.

Role Check: On login, query profiles table for role; redirect accordingly.

Protected Routes: Use Next.js middleware or getServerSideProps to check auth status and role; redirect unauthenticated to login, non-admins from admin pages to module overview.

Admin Identification: For MVP, manually set admin role in database for specific users.



5.5 Content Rendering and UploadYouTube Embed: Use responsive iframe with aspect ratio preservation.

Markdown: Parse and render with support for:Headers (H1-H6).

Bold/italic/links.

Lists (ordered/unordered).

Code blocks (inline and fenced, with language-specific highlighting, e.g., python, arduino).


Screenshot Images: Upload to public Supabase Storage bucket (signed URLs for security if needed); display as <img> tags in module detail page.

Upload Flow: In admin form, use Supabase storage client to upload files, get public URLs, and store in module record.



6. Non-Functional Requirements6.1 PerformancePage loads: <3 seconds.

Optimize images/videos: Use lazy loading for embeds and images; compress uploads if possible.



6.2 SecurityHTTPS enforced via Vercel/Supabase.

Input validation on forms to prevent XSS/SQL injection (e.g., sanitize markdown).

RLS on Supabase: Read modules for all auth users; write/delete for admins only. Storage: Authenticated uploads only.

File Security: Validate file types (images only), scan for malware if possible (Supabase handles basic).



6.3 UsabilityIntuitive UI: Clean, minimal design; preview markdown in admin form.

Accessibility: Alt text for images (prompt admin to add), keyboard navigation, ARIA labels for embeds.

Error Handling: User-friendly messages (e.g., "Invalid login credentials", "File upload failed").



6.4 TestingUnit Tests: Auth flows, markdown rendering, file uploads.

Integration Tests: Page navigation, data fetching, CRUD operations.

Manual Tests: Cross-browser compatibility, admin content upload end-to-end.



7. Dependencies and Risks7.1 DependenciesVercel account for deployment.

Supabase project setup with auth, database, and storage enabled.

YouTube API/terms compliance for embeds.



7.2 Risks and MitigationsFree Tier Limits: Monitor usage (e.g., storage for images); mitigate by limiting uploads.

Auth/Role Issues: Test role-based redirects thoroughly.

Upload Failures: Handle large files with client-side validation.

Markdown Rendering Bugs: Test with sample content including code blocks and images.



8. Timeline and Milestones (Suggested)Week 1: Setup project (Next.js + Supabase integration), build public/user pages.

Week 2: Implement auth, roles, and protected user pages.

Week 3: Add admin backend, upload features, seed initial admin.

Week 4: Testing, deployment to Vercel, final QA.



9. Future EnhancementsIntegrate Google OAuth via Supabase.

Advanced admin features (e.g., bulk uploads, user management).

User progress tracking (e.g., completed modules).

Mobile optimization and PWA features.

Analytics integration (e.g., Vercel Analytics).



This PRD serves as a blueprint for development. Any changes should be documented in updated versions. For questions, refer to the product owner.




