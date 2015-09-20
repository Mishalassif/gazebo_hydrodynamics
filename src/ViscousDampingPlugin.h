/**
 *  @file ViscousDampingPlugin.h
 *  @class ViscousDampingPlugin
 *  @brief Model Plugin that simulates buoyancy forces
 *  @author 2015 Mishal Assif
 */

#ifndef _GAZEBO_VISCOUS_DAMPING_PLUGIN_H_
#define _GAZEBO_VISCOUS_DAMPING_PLUGIN_H_

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

#include <eigen3/Eigen/Dense>

#define NO_OF_DAMPINGCOEFFS 12 

typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;

const std::string DampingCoefficientIndex[] =

{
    "X_u",
    "Y_v",
    "Z_w",
    "K_p",
    "L_q",
    "M_r",
    "X_uu",
    "Y_vv",
    "Z_ww",
    "K_pp",
    "L_qq",
    "M_rr"
};

namespace gazebo
{
    class ViscousDampingPlugin : public ModelPlugin
    {
        public:

            ViscousDampingPlugin();
            ~ViscousDampingPlugin();

            void Load( physics::ModelPtr model, sdf::ElementPtr sdf );
                        
        private:

            std::map<int, std::map<std::string, double> > _dampingCoeff;
            std::map<int, Matrix6d> _linearDampingMatrix;
            std::map<int, Matrix6d> _quadraticDampingMatrix;
    
            physics::PhysicsEnginePtr _physicsEngine;
            physics::ModelPtr _model;
            physics::WorldPtr _world;
            physics::LinkPtr  _baseLink;
            sdf::ElementPtr   _sdf;

            event::ConnectionPtr _updateConnection;

            ros::NodeHandle _nh;
            ros::Publisher _debugWrenchPublisher; 
            geometry_msgs::Wrench _debugWrenchMsg;

            void _OnUpdate();
            Vector6d _getForceTorque(int linkId, math::Vector3 linearVel, math::Vector3 angularVel);

    };

}

#endif  // _GAZEBO_VISCOUS_DAMPING_PLUGIN_H_
