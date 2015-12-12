/**
 *  @file BaseModelPlugin.cpp
 *  @class BaseModelPlugin
 *  @brief Base Model Plugin
 *  @copyright (c) 2015 Mishal Assif
 */

#include <BaseModelPlugin.h>

namespace gazebo
{
    BaseModelPlugin::BaseModelPlugin(std::string pluginName): _pluginName(pluginName)
    {
        ROS_INFO_STREAM("Constructing " << _pluginName.c_str());

        std::string _debugTopic = "/auv_simulator/" + _pluginName + "/wrench";
        _debugWrenchPublisher = _nh.advertise<geometry_msgs::Wrench>(_debugTopic, 2);
    }

    BaseModelPlugin::~BaseModelPlugin()
    {}

    void BaseModelPlugin::Load( physics::ModelPtr model, sdf::ElementPtr sdf )
    {
        ROS_INFO_STREAM("Started loading " << _pluginName.c_str());
        std::string msg = _pluginName + std::string(" Received NULL model pointer");
        GZ_ASSERT(model != NULL, msg.c_str());
        this->_model = model;

        msg = _pluginName + std::string(" Received Model in a NULL world");
        _world = _model->GetWorld();
        GZ_ASSERT(_world != NULL, msg.c_str());

        msg = _pluginName + std::string(" Received NULL SDF Pointer");
        GZ_ASSERT(sdf != NULL, msg.c_str());
        this->_sdf = sdf;

        msg = _pluginName + std::string(" Received NULL baseLink pointer");
        this->_baseLink = _model->GetLink("base_link");  // @TODO: remove Hardcoding
        GZ_ASSERT(_baseLink != NULL, msg.c_str());

        if  (!ros::isInitialized())
        {
            ROS_FATAL_STREAM("A ROS node for gazebo has not been initialized"
                    <<", unable to load plugin: BuoyancyPlugin. " // Wrong error msg
                    <<"Load Gazebo system plugin 'libgazebo_ros_api_plugin.so' "
                    <<"in the gazebo_ros package");
            return;
        }

        ROS_INFO_STREAM("Initialized Base class of plugin " << _pluginName.c_str());
    }

    void BaseModelPlugin::_publishDebug(math::Vector3 force, math::Vector3 torque)
    {
        _debugWrenchMsg.force.x = force.x;
        _debugWrenchMsg.force.y = force.y;
        _debugWrenchMsg.force.z = force.z;
        _debugWrenchMsg.torque.x = torque.x;
        _debugWrenchMsg.torque.y = torque.y;
        _debugWrenchMsg.torque.z = torque.z;
        _debugWrenchPublisher.publish(_debugWrenchMsg);
    }

    math::Vector3 BaseModelPlugin::_roundError(math::Vector3 vector)
    {
        if( vector.x < 1e-10 && vector.x > -1e-10 )
        {
            vector.x = 0;
        }
        if( vector.y < 1e-10 && vector.y > -1e-10 )
        {
            vector.y = 0;
        }
        if( vector.z < 1e-10 && vector.z > -1e-10 )
        {
            vector.z = 0;
        }
        return vector;
    }
}
