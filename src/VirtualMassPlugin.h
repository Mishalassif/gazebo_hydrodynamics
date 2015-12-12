/**
 *  @file VirtualMassPlugin.h
 *  @class VirtualMassPlugin
 *  @brief Model Plugin that simulates virtual mass forces
 *  @author Mishal Assif
 */

#ifndef _GAZEBO_VIRTUAL_MASS_PLUGIN_H_
#define _GAZEBO_VIRTUAL_MASS_PLUGIN_H_

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

#define NO_OF_HYDCOEFFS 18

typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;

const std::string HydroCoefficientIndex[] =

{
    "X_udot",
    "Y_vdot",
    "Z_wdot",
    "K_pdot",
    "M_qdot",
    "N_rdot",
    "X_wdot",
    "Y_pdot",
    "Z_qdot",
    "K_rdot",
    "M_udot",
    "N_vdot",
    "X_qdot",
    "Y_rdot",
    "Z_udot",
    "K_vdot",
    "M_wdot",
    "N_vdot"
};

namespace gazebo
{
    class VirtualMassPlugin : public ModelPlugin
    {
        public:

            VirtualMassPlugin();
            ~VirtualMassPlugin();

            void Load( physics::ModelPtr model, sdf::ElementPtr sdf );

        private:

            std::map<int, std::map<std::string, double> > _hydroCoeff;
            std::map<int, Matrix6d> _addedMassMatrix;
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
            Vector6d _getForceTorque(int linkId, math::Vector3 linearVel, math::Vector3 angularVel,
                                                 math::Vector3 linearAcc, math::Vector3 angularAcc );
    };

}

#endif  //  _GAZEBO_VIRTUAL_MASS_PLUGIN_H_
