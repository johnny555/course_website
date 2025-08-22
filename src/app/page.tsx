export default function Home() {
  return (
    <main className="min-h-screen bg-gray-50">
      <div className="container mx-auto px-4 py-16">
        <div className="text-center">
          <h1 className="text-5xl font-bold text-gray-900 mb-6">
            Robotics Course
          </h1>
          <p className="text-xl text-gray-600 mb-8 max-w-2xl mx-auto">
            Learn robotics with hands-on projects, from basic concepts to advanced programming.
            Build real robots and bring your ideas to life.
          </p>
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
              Sign Up Free
            </a>
          </div>
        </div>
        
        <div className="mt-16 grid md:grid-cols-3 gap-8">
          <div className="bg-white p-6 rounded-lg shadow-sm">
            <div className="text-3xl mb-4">🤖</div>
            <h3 className="text-lg font-semibold mb-3">Hands-on Learning</h3>
            <p className="text-gray-600">Build real robots with step-by-step video tutorials and detailed guides.</p>
          </div>
          <div className="bg-white p-6 rounded-lg shadow-sm">
            <div className="text-3xl mb-4">💻</div>
            <h3 className="text-lg font-semibold mb-3">Programming Focus</h3>
            <p className="text-gray-600">Learn to program robots using Python, Arduino, and modern robotics frameworks.</p>
          </div>
          <div className="bg-white p-6 rounded-lg shadow-sm">
            <div className="text-3xl mb-4">🚀</div>
            <h3 className="text-lg font-semibold mb-3">Project-Based</h3>
            <p className="text-gray-600">Complete real-world projects that you can showcase in your portfolio.</p>
          </div>
        </div>

        <div className="mt-16 bg-white rounded-lg p-8 shadow-sm">
          <h2 className="text-2xl font-bold text-gray-900 mb-4 text-center">What You&apos;ll Learn</h2>
          <div className="grid md:grid-cols-2 gap-6">
            <div>
              <h4 className="font-semibold text-gray-900 mb-2">Foundation Concepts</h4>
              <ul className="text-gray-600 space-y-1">
                <li>• Robot mechanics and electronics</li>
                <li>• Sensors and actuators</li>
                <li>• Control systems basics</li>
              </ul>
            </div>
            <div>
              <h4 className="font-semibold text-gray-900 mb-2">Advanced Topics</h4>
              <ul className="text-gray-600 space-y-1">
                <li>• Computer vision for robots</li>
                <li>• Path planning algorithms</li>
                <li>• Machine learning integration</li>
              </ul>
            </div>
          </div>
        </div>
      </div>
    </main>
  )
}