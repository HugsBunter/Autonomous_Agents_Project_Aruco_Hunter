#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

namespace gazebo
{
  class ModelPush : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
      // Model pointer
      this->model = _parent;
      
      // The event is broadcast every simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&ModelPush::OnUpdate, this));
    }

    public: void OnUpdate()
    {
      // Get the current simulation time in seconds
      double currentTime = this->model->GetWorld()->SimTime().Double();

      // // Print the simulation time
      // std::string timeString = std::to_string(currentTime);
      // std::cout << "Simulation time: " << timeString << std::endl;

      // Duration of each movement phase in sec
      double phaseDuration = 8.0;

      // Current phase of the movement (0 = forward, 1 = left, 2 = back, 3 = right)
      int phase = static_cast<int>(currentTime / phaseDuration) % 4;

      switch (phase)
      {
        case 0: // Move forward
          this->model->SetLinearVel(ignition::math::Vector3d(2, 0, 0));
          break;
        case 1: // Move left
          this->model->SetLinearVel(ignition::math::Vector3d(0, 2, 0));
          this->model->SetAngularVel(ignition::math::Vector3d(0, 0, 0.202));
          break;
        case 2: // Move back
          this->model->SetLinearVel(ignition::math::Vector3d(-2, 0, 0));
          this->model->SetAngularVel(ignition::math::Vector3d(0, 0, 0.202));
          break;
        case 3: // Move right
          this->model->SetLinearVel(ignition::math::Vector3d(0, -2, 0));
          this->model->SetAngularVel(ignition::math::Vector3d(0, 0, 0.202));
          break;
      }

    }

    private: physics::ModelPtr model;
    private: event::ConnectionPtr updateConnection;
  };

  // Register the plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}