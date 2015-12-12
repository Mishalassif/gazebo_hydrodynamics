/**
 *  @file BaseModelPlugin.h
 *  @class BaseModelPlugin
 *  @brief Base class for all subsequent Model Plugins.
 *  @copyright (c) 2015 Mishal Assif
 */

#ifndef _AUV_SIMULATOR_BASE_MODEL_PLUGIN_H_
#define _AUV_SIMULATOR_BASE_MODEL_PLUGIN_H_

#include <boost/bind.hpp>

#include <ros/ros.h>
#include <ros/console.h>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/common/Event.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>

#include <geometry_msgs/Wrench.h>

namespace gazebo
{
    class BaseModelPlugin : public ModelPlugin
    {
      public:

        /**
         * @brief BaseModelPlugin Constructor
         * @param[in] pluginName    name of the plugin, while publishing debug messages, this name will be used
         */
        explicit BaseModelPlugin(std::string pluginName = "default_plugin");

        /**
         * @brief BaseModelPlugin Destructor
         */
        virtual ~BaseModelPlugin();

        /**
         * @brief Called when a Plugin is first created, and after the World has been loaded. This function should not be blocking.
         *        Checks if any of the pointer, world model or sdf is null and also sets the baseLink pointer. 
         * @param[in]  model  Pointer to Model
         * @param[in]  sdf    Pointer to SDF element of the plugin
         */
        virtual void Load( physics::ModelPtr model, sdf::ElementPtr sdf );

      protected:
        std::string _pluginName;      ///< Name of the topic to which debug messages are published  
        
        physics::ModelPtr _model;     ///< Pointer to the model 
        physics::WorldPtr _world;     ///< Pointer to the model        
        physics::LinkPtr  _baseLink;  ///< Pointer to the link with name "base_link"    
        sdf::ElementPtr   _sdf;       ///< Pointer to the SDF element within the plugin

        event::ConnectionPtr _updateConnection;   ///< Connection to World Update events.

        ros::NodeHandle _nh;
        ros::Publisher _debugWrenchPublisher;     ///< Publishes force and torque applied by the plugin
        geometry_msgs::Wrench _debugWrenchMsg;    ///< Message for _debugWrenchPublisher

        /**
         * @brief  Callback for world update.Force and torque is computed and applied in this function
         */
        virtual void _onUpdate() = 0;

        /**
         * @brief  Publishes the force and torque applied to auv_simulator/_pluginName/wrench
         */
        void _publishDebug(math::Vector3 force, math::Vector3 torque = math::Vector3());

        /**
         * @brief  Rounds of forces and torques below e-10 to 0, so as to remove compound effects of such small errors
         */
        math::Vector3 _roundError(math::Vector3 vector);
    };
}

#endif // _AUV_SIMULATOR_BASE_MODEL_PLUGIN_H_
