#include "gazebo_gripping_plugin/gazebo_gripping_plugin.hpp"

namespace gazebo
{
  class GazeboGrippingPlugin : public ModelPlugin
  {
    public: GazeboGrippingPlugin() : ModelPlugin()
    {
        std::cout << "Loading Gazebo Gripping Plugin\n";
    }

    public: void Load(physics::ModelPtr model_, sdf::ElementPtr sdf_)
    {
        // Get Physics Engine and Contact Manager
        world_ = model_->GetWorld();
        physics_ = world_->Physics();
        physics_mutex_ = physics_->GetPhysicsUpdateMutex();
        contact_manager_ = physics_->GetContactManager();

        // Get sdf elements
        if (!sdf_->HasElement("gripper_link_name"))
        {
            ROS_ERROR("<gripper_link_name> tag not set, can't initialize gripper plugin");
            std::cout << "<gripper_link_name> tag not set, can't initialize gripper plugin" << std::endl;
            return;
        }
        gripper_link_name_ = sdf_->GetElement("gripper_link_name")->Get<std::string>() + "_collision";

        // Set initial gripping status 
        attach_ = false;
        joint_created_ = false;

        // Setup ROS subscriber for attaching 
        sub_ = nh_.subscribe("/gripper_attach_cmd", 1, &GazeboGrippingPlugin::attachClbk, this);

        // Log successful plugin loading
        ROS_INFO("[Gripping Plugin] plugin successfully loaded");

        // Set the update connection events
        update_connection_ = event::Events::ConnectWorldUpdateEnd(boost::bind(&GazeboGrippingPlugin::OnUpdate, this));

    }

    /* Private class members */
    private:

        void OnUpdate()
        {   
            if (!attach_)
            {
                if (joint_created_)
                {
                    boost::recursive_mutex::scoped_lock lock(*physics_mutex_);
                    gripping_joint_->Detach();
                }
                return;
            }

            for (auto& contact : contact_manager_->GetContacts())
            {
                if (gripper_link_name_ == contact->collision1->GetName() || 
                    gripper_link_name_ == contact->collision2->GetName())
                {
                    if(!joint_created_)
                    {
                        gripping_joint_ = physics_->CreateJoint("fixed", contact->collision1->GetModel());
                        gripping_joint_->Init();
                        joint_created_ = true;
                    }
                    
                    gripping_joint_->Attach(contact->collision1->GetLink(), 
                            contact->collision2->GetLink());
                    gripping_joint_->Load(contact->collision1->GetLink(), 
                        contact->collision2->GetLink(), ignition::math::Pose3d());
                    gripping_joint_->SetModel(contact->collision2->GetModel());
                    gripping_joint_->Update();
                }
            }
            return;
        }

        void attachClbk(const std_msgs::BoolConstPtr& msg)
        {   
            attach_ = msg->data;
            return;
        }

        // Gazebo communication, physics, and joint interfaces
        event::ConnectionPtr update_connection_;
        physics::PhysicsEnginePtr physics_;
        physics::WorldPtr world_;
        boost::recursive_mutex* physics_mutex_;
        physics::ContactManager* contact_manager_;
        physics::JointPtr gripping_joint_;

        // Gripper parameters
        std::string gripper_link_name_;

        // ROS comm
        ros::NodeHandle nh_;
        ros::Subscriber sub_;

        // Gripper status 
        bool attach_;
        bool joint_created_;

  };
  GZ_REGISTER_MODEL_PLUGIN(GazeboGrippingPlugin)
}