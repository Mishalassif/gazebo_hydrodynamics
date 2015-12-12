/**
 *  @file viscous_damping_plugin.h
 *  @class ViscousDampingPlugin
 *  @brief Model Plugin that simulates viscous forces
 *         Refer this paper for details
 *         http://cdn.intechopen.com/pdfs/6230/InTech-Dynamic_modelling_and_motion_control_for_underwater_vehicles_with_fins.pdf
 *  @copyright (c) 2015 Mishal Assif
 */

#ifndef _AUV_SIMULATOR_VISCOUS_DAMPING_PLUGIN_H_
#define _AUV_SIMULATOR_VISCOUS_DAMPING_PLUGIN_H_

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

#include <BaseModelPlugin.h>

#define NO_OF_DAMPINGCOEFFS 12

typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;

const std::string DampingCoefficientIndex[] =

{
    "X_u",
    "Y_v",
    "Z_w",
    "K_p",
    "M_q",
    "N_r",
    "X_uu",
    "Y_vv",
    "Z_ww",
    "K_pp",
    "M_qq",
    "N_rr"
};

namespace gazebo
{
    class ViscousDampingPlugin : public BaseModelPlugin
    {
      public:

        /**
         * @brief ViscousDampingPlugin Constructor
         */
        ViscousDampingPlugin();
        
        /**
         * @brief ViscousDampingPlugin Destructor
         */
        ~ViscousDampingPlugin();

        /**
         * @brief Calls BaseClass load and extracts the damping coeffs for
         *        the links specified in the sdf
         */
        void Load( physics::ModelPtr model, sdf::ElementPtr sdf );

      private:

        std::map<int, std::map<std::string, double> > _dampingCoeff;   ///< Maps LinkId to a map of all damping coeffs
        std::map<int, Matrix6d> _linearDampingMatrix;      ///< Maps LinkId to a 6 matrix of linearDampingCoeffs
        std::map<int, Matrix6d> _quadraticDampingMatrix;   ///< Maps LinkId to a 6 matrix of quadraticDampingCoeffs 

        /*
         * @brief  Gets force and torque by calling _computeForceTorque and applies it on each link
         */
        void _onUpdate();

        /*
         * @brief  Computes force and torque on the link.For details, refer paper mentioned in the beginning
         * @param[in] linkId     Id of the link,required to get the damping coefficients
         * @param[in] linearVel  Linear velocity of the link
         * @param[in] angularVel Angular velocity of the link
         */
        Vector6d _computeForceTorque(int linkId, math::Vector3 linearVel, math::Vector3 angularVel);

    };

}

#endif  // _AUV_SIMULATOR_VISCOUS_DAMPING_PLUGIN_H_
