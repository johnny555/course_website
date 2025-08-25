export default function Home() {
  return (
    <main className="min-h-screen bg-gray-50">
      <div className="container mx-auto px-4 py-16">
        <div className="text-center">
          <h1 className="text-5xl font-bold text-gray-900 mb-6">
            Become a Roboticist
          </h1>
          <p className="text-xl text-gray-600 mb-4 max-w-3xl mx-auto">
            A 28 Day challenge course about building a robot simulation with the Robotics Operating System.
          </p>
          <div className="bg-blue-50 border border-blue-200 rounded-lg p-6 mb-8 max-w-4xl mx-auto">
            <h2 className="text-lg font-semibold text-blue-900 mb-4">You will:</h2>
            <div className="grid md:grid-cols-3 gap-4 text-blue-800">
              <div className="flex items-center">
                <span className="text-2xl mr-2">🧩</span>
                <span>Solve robotics problems</span>
              </div>
              <div className="flex items-center">
                <span className="text-2xl mr-2">🤖</span>
                <span>Create your own robot</span>
              </div>
              <div className="flex items-center">
                <span className="text-2xl mr-2">📱</span>
                <span>Daily share on LinkedIn or X</span>
              </div>
            </div>
            <h3 className="text-lg font-semibold text-blue-900 mt-6 mb-4">This will give you:</h3>
            <ul className="text-blue-800 space-y-2">
              <li>• The confidence to create a robot simulation from scratch</li>
              <li>• A unique robot to describe in interviews or startup pitches</li>
              <li>• Evidence of your new skills by posting on social media</li>
            </ul>
          </div>
          <div className="space-x-4">
            <a
              href="/login"
              className="bg-blue-600 hover:bg-blue-700 text-white font-semibold py-3 px-6 rounded-lg transition-colors"
            >
              Access Course Content
            </a>
            <a
              href="/signup"
              className="bg-gray-200 hover:bg-gray-300 text-gray-800 font-semibold py-3 px-6 rounded-lg transition-colors"
            >
              Start Now
            </a>
          </div>
        </div>
        
        <div className="mt-16 grid md:grid-cols-3 gap-8">
          <div className="bg-white p-6 rounded-lg shadow-sm">
            <div className="text-3xl mb-4">⚡</div>
            <h3 className="text-lg font-semibold mb-3">Don&apos;t Waste Months Getting Started</h3>
            <p className="text-gray-600">Most roboticists spend months slogging through basic tutorials. I&apos;ll give you everything you need upfront, then we&apos;ll slowly peel back the layers.</p>
          </div>
          <div className="bg-white p-6 rounded-lg shadow-sm">
            <div className="text-3xl mb-4">💪</div>
            <h3 className="text-lg font-semibold mb-3">Build Confidence Through Practice</h3>
            <p className="text-gray-600">Break imposter syndrome by building robots. When you see the robot you created, you&apos;ll know you&apos;ve become a roboticist.</p>
          </div>
          <div className="bg-white p-6 rounded-lg shadow-sm">
            <div className="text-3xl mb-4">🌐</div>
            <h3 className="text-lg font-semibold mb-3">Become a Social Roboticist</h3>
            <p className="text-gray-600">Build your profile by publishing daily to LinkedIn or Twitter. Get noticed by robot companies and make friends along the way.</p>
          </div>
        </div>

        <div className="mt-16 grid md:grid-cols-2 gap-8">
          <div className="bg-white p-6 rounded-lg shadow-sm">
            <div className="text-3xl mb-4">🏆</div>
            <h3 className="text-lg font-semibold mb-3">Join A Community</h3>
            <p className="text-gray-600">Learn with friends during cohorts. My best robotics memories come from learning together - whether in summer school or with PhD buddies.</p>
          </div>
          <div className="bg-white p-6 rounded-lg shadow-sm">
            <div className="text-3xl mb-4">🪟</div>
            <h3 className="text-lg font-semibold mb-3">Use Windows 11</h3>
            <p className="text-gray-600">Most ROS courses are Linux-first. This course works entirely on Windows 11 using Docker and Windows Subsystem for Linux.</p>
          </div>
          <div className="bg-white p-6 rounded-lg shadow-sm">
            <div className="text-3xl mb-4">📚</div>
            <h3 className="text-lg font-semibold mb-3">Just Enough Information</h3>
            <p className="text-gray-600">Skip the boring setup tutorials. We start by building a robot, then explore packages and configs as we need them.</p>
          </div>
          <div className="bg-white p-6 rounded-lg shadow-sm">
            <div className="text-3xl mb-4">🎨</div>
            <h3 className="text-lg font-semibold mb-3">Create Digital Twins</h3>
            <p className="text-gray-600">Use FreeCAD for models, Blender for finishing touches, and Scaniverse iPhone app to capture 3D scenes for simulation.</p>
          </div>
        </div>
        
        <div className="mt-16 text-center">
          <div className="bg-gradient-to-r from-blue-600 to-indigo-600 rounded-lg p-8 text-white">
            <h2 className="text-3xl font-bold mb-4">Start Now!</h2>
            <p className="text-xl mb-6">Start the challenge now! Chat with me on Discord to get help and meet others along their journey.</p>
            <a
              href="/login"
              className="bg-white text-blue-600 hover:bg-gray-100 font-semibold py-3 px-8 rounded-lg transition-colors inline-block mr-4"
            >
              Access Course Content
            </a>
            <a
              href="/signup"
              className="border-2 border-white text-white hover:bg-white hover:text-blue-600 font-semibold py-3 px-8 rounded-lg transition-colors inline-block"
            >
              Join Course
            </a>
          </div>
        </div>
      </div>
    </main>
  )
}