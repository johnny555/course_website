import ProtectedRoute from '@/components/ProtectedRoute'
import ModuleForm from '@/components/ModuleForm'

export default function CreateModulePage() {
  return (
    <ProtectedRoute adminOnly>
      <ModuleForm />
    </ProtectedRoute>
  )
}