/**
 *  @file viscous_damping_plugin.cpp
 *  @class ViscousDampingPlugin
 *  @brief Model Plugin that simulates the effect of viscous_damping
 *  @copyright (c) 2015 Mishal Assif
 */

#include <ViscousDampingPlugin.h>

namespace gazebo
{

    ViscousDampingPlugin::ViscousDampingPlugin() : BaseModelPlugin("viscous_damping_plugin")
    {
    }

    ViscousDampingPlugin::~ViscousDampingPlugin()
    {
    }

    void ViscousDampingPlugin::Load( physics::ModelPtr model, sdf::ElementPtr sdf )
    {
        BaseModelPlugin::Load(model, sdf);

        if (this->_sdf->HasElement("link"))
        {
            for (sdf::ElementPtr linkElem = this->_sdf->GetElement("link"); linkElem;
                    linkElem = this->_sdf->GetNextElement("link"))
            {
                int id = -1;
                std::string name = "";
                double dampparam;
                std::map<std::string, double> dampingCoefficient;
                if (linkElem->HasAttribute("name"))
                {
                    name = linkElem->Get<std::string>("name");
                    physics::LinkPtr link = this->_model->GetLink(name);
                    if (!link)
                    {
                        gzwarn << "Specified link [" << name << "] not found." << std::endl;
                        continue;
                    }
                    id = link->GetId();
                }
                else
                {
                    gzwarn << "Required attribute name missing from link [" << name
                        << "] in VirtualMassPlugin SDF" << std::endl;
                    continue;
                }

                if (this->_dampingCoeff.count(id) != 0)
                {
                    gzwarn << "Properties for link [" << name << "] already set, skipping "
                        << "second property block" << std::endl;
                    continue;
                }

                for(int i=0; i < NO_OF_DAMPINGCOEFFS; i++)
                {
                    if(linkElem->HasElement(DampingCoefficientIndex[i]))
                    {
                        dampparam = linkElem->GetElement(DampingCoefficientIndex[i])->Get<double>();
                    }
                    else
                    {
                        dampparam = 0;
                    }
                    dampingCoefficient[DampingCoefficientIndex[i]] = dampparam;
                    this->_dampingCoeff[id] = dampingCoefficient;
                }
            }
        }

        for(std::map<int, std::map<std::string, double> >::iterator it = _dampingCoeff.begin();
                it != _dampingCoeff.end(); ++it)
        {
            _linearDampingMatrix[it->first] << it->second["X_u"], 0, 0, 0, 0, 0,
                                               0, it->second["Y_v"], 0, 0, 0, 0,
                                               0, 0, it->second["Z_w"], 0, 0, 0,
                                               0, 0, 0, it->second["K_p"], 0, 0,
                                               0, 0, 0, 0, it->second["M_q"], 0,
                                               0, 0, 0, 0, 0, it->second["N_r"];
            
            _quadraticDampingMatrix[it->first] << it->second["X_uu"], 0, 0, 0, 0, 0,
                                                  0, it->second["Y_vv"], 0, 0, 0, 0,
                                                  0, 0, it->second["Z_ww"], 0, 0, 0,
                                                  0, 0, 0, it->second["K_pp"], 0, 0,
                                                  0, 0, 0, 0, it->second["M_qq"], 0,
                                                  0, 0, 0, 0, 0, it->second["N_rr"];

        }

        this->_updateConnection = event::Events::ConnectWorldUpdateBegin(
                boost::bind(&ViscousDampingPlugin::_onUpdate, this));
    }

    void ViscousDampingPlugin::_onUpdate()
    {
        std::vector<physics::LinkPtr> links;
        links = this->_model->GetLinks();
        math::Vector3 force;
        math::Vector3 torque;
        for (std::vector<physics::LinkPtr>::iterator it = links.begin();
                it != links.end(); ++it)
        {
            int id = (*it)->GetId();
            if (this->_dampingCoeff.find(id) != this->_dampingCoeff.end())
            {

                Vector6d dampingForceTorque =  _computeForceTorque(id, (*it)->GetRelativeLinearVel(),
                                                                   (*it)->GetRelativeAngularVel());
                force.x = dampingForceTorque(0,0);
                force.y = dampingForceTorque(1,0);
                force.z = dampingForceTorque(2,0);

                torque.x = dampingForceTorque(3,0);
                torque.y = dampingForceTorque(4,0);
                torque.z = dampingForceTorque(5,0);

                force = _roundError(force);
                torque = _roundError(torque);
                _publishDebug(force, torque);
                (*it)->AddRelativeForce(force);
                (*it)->AddRelativeTorque(torque);
            }
        }
    }

    Vector6d ViscousDampingPlugin::_computeForceTorque(int linkId, math::Vector3 linearVel, math::Vector3 angularVel)
    {

        Matrix6d dampingMatrix;
        Matrix6d velocityMatrix;
        Vector6d velocityVector;
        Vector6d forceTorqueVector;

        velocityVector << linearVel.x, linearVel.y, linearVel.z,
                          angularVel.x, angularVel.y, angularVel.z;
        velocityMatrix = velocityVector.asDiagonal();
        velocityMatrix = velocityMatrix.array().abs();
        dampingMatrix = _linearDampingMatrix[linkId] +
                        _quadraticDampingMatrix[linkId]*velocityMatrix;

        forceTorqueVector = -1*(dampingMatrix * velocityVector);
        return forceTorqueVector;
    }
    GZ_REGISTER_MODEL_PLUGIN(ViscousDampingPlugin);
}
