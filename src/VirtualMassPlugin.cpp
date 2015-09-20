/**
 *  @file VirtualMassPlugin.cpp
 *  @class VirtualMassPlugin
 *  @brief Model Plugin that simulates the effect of Virtual Mass.
 *         Refer this paper http://cdn.intechopen.com/pdfs/6230/InTech-Dynamic_modelling_and_motion_control_for_underwater_vehicles_with_fins.pdf 
 *  @author Mishal Assif
 */

#include<VirtualMassPlugin.h>

namespace gazebo 
{

    VirtualMassPlugin::VirtualMassPlugin()
    {
        _debugWrenchPublisher = _nh.advertise<geometry_msgs::Wrench>("/gazebo/virtual_mass_plugin/wrench", 2);
    }

    VirtualMassPlugin::~VirtualMassPlugin()
    {
    }

    void VirtualMassPlugin::Load( physics::ModelPtr model, sdf::ElementPtr sdf )
    {
        GZ_ASSERT(model != NULL, "Received NULL model pointer");
        this->_model = model;

        _world = _model->GetWorld();
        GZ_ASSERT(_world != NULL, "Model is in a NULL world");

        GZ_ASSERT(sdf != NULL, "Received NULL SDF pointer");
        this->_sdf = sdf;

        this->_baseLink = _model->GetLink("base_link");
        GZ_ASSERT(_baseLink != NULL, "Received Null baseLink pointer");

        ROS_INFO_STREAM("Started plugin !!!!!!!!!!!!");
        std::cout<<"Started plugin !!!!!!!!!!!!";

        if(!ros::isInitialized())
        {
            ROS_FATAL_STREAM("A ROS node for gazebo has not been initialized"
                    <<", unable to load plugin: VirtualMassPlugin. "
                    <<"Load Gazebo system plugin 'libgazebo_ros_api_plugin.so' "
                    <<"in the gazebo_ros package");
            return;
        }

        if (this->_sdf->HasElement("link"))
        {
            for (sdf::ElementPtr linkElem = this->_sdf->GetElement("link"); linkElem;
                    linkElem = this->_sdf->GetNextElement("link"))
            {
                int id = -1;
                std::string name = "";
                double hydroparam;
                std::map<std::string, double> hydrodynamicCoefficient;
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

                if (this->_hydroCoeff.count(id) != 0)
                {
                    gzwarn << "Properties for link [" << name << "] already set, skipping "
                        << "second property block" << std::endl;
                    continue;
                }
                
                for(int i=0; i < NO_OF_HYDCOEFFS; i++)
                {
                    if(linkElem->HasElement(HydroCoefficientIndex[i]))
                    {
                        hydroparam = linkElem->GetElement(HydroCoefficientIndex[i])->Get<double>();
                    }
                    else
                    {
                        hydroparam = 0;
                    }
                    hydrodynamicCoefficient[HydroCoefficientIndex[i]] = hydroparam;
                    this->_hydroCoeff[id] = hydrodynamicCoefficient;
                }
            }
        }

        for(std::map<int, std::map<std::string, double> >::iterator it = _hydroCoeff.begin();
                it != _hydroCoeff.end(); ++it)
        {
            _addedMassMatrix[it->first] << it->second["X_udot"], 0, it->second["X_wdot"],
                                        0, it->second["X_qdot"], 0,
                                        0, it->second["Y_vdot"], 0,
                                        it->second["Y_pdot"], 0, it->second["Y_rdot"],
                                        it->second["Z_udot"], 0, it->second["Z_wdot"],
                                        0, it->second["Z_qdot"], 0,
                                        0, it->second["K_vdot"], 0,
                                        it->second["K_pdot"], 0, it->second["K_rdot"],
                                        it->second["M_udot"], 0, it->second["M_wdot"],
                                        0, it->second["M_qdot"], 0,
                                        0, it->second["N_vdot"], 0,
                                        it->second["N_pdot"], 0, it->second["N_rdot"];
        }

        this->_updateConnection = event::Events::ConnectWorldUpdateBegin(
                boost::bind(&VirtualMassPlugin::_OnUpdate, this));
    }


    void VirtualMassPlugin::_OnUpdate()
    {
        std::vector<physics::LinkPtr> links;
        links = this->_model->GetLinks();
        math::Vector3 force;
        math::Vector3 torque;
        for (std::vector<physics::LinkPtr>::iterator it = links.begin();
                it != links.end(); ++it)
        {
            int id = (*it)->GetId();
            if (this->_hydroCoeff.find(id) != this->_hydroCoeff.end())
            {

                Vector6d virtualMassForceTorque =  _getForceTorque(id, (*it)->GetRelativeLinearVel(),
                                                                    (*it)->GetRelativeAngularVel(),
                                                                    (*it)->GetRelativeLinearAccel(),
                                                                    (*it)->GetRelativeAngularAccel());
                force.x = virtualMassForceTorque(0,0);
                force.y = virtualMassForceTorque(1,0);
                force.z = virtualMassForceTorque(2,0);

                torque.x = virtualMassForceTorque(3,0);
                torque.y = virtualMassForceTorque(4,0);
                torque.z = virtualMassForceTorque(5,0);

                (*it)->AddRelativeForce(force);
                (*it)->AddRelativeTorque(torque);
            }
        }
    }

    Vector6d VirtualMassPlugin::_getForceTorque(int linkId, math::Vector3 linearVel, math::Vector3 angularVel,
                                                 math::Vector3 linearAcc, math::Vector3 angularAcc )
    {
        double a1 = _hydroCoeff.find(linkId)->second["X_udot"]*linearVel.x  +
                    _hydroCoeff.find(linkId)->second["X_wdot"]*linearVel.z  +
                    _hydroCoeff.find(linkId)->second["X_qdot"]*angularVel.y ;

        double a2 = _hydroCoeff.find(linkId)->second["Y_vdot"]*linearVel.y  +
                    _hydroCoeff.find(linkId)->second["Y_pdot"]*angularVel.x +
                    _hydroCoeff.find(linkId)->second["Y_rdot"]*angularVel.z ;
        
        double a3 = _hydroCoeff.find(linkId)->second["Z_udot"]*linearVel.x +
                    _hydroCoeff.find(linkId)->second["Z_wdot"]*linearVel.z +
                    _hydroCoeff.find(linkId)->second["Z_qdot"]*angularVel.y;
        
        double b1 = _hydroCoeff.find(linkId)->second["K_vdot"]*linearVel.y  +
                    _hydroCoeff.find(linkId)->second["K_pdot"]*angularVel.x +
                    _hydroCoeff.find(linkId)->second["K_rdot"]*angularVel.z ;
        
        double b2 = _hydroCoeff.find(linkId)->second["L_udot"]*linearVel.x +
                    _hydroCoeff.find(linkId)->second["L_wdot"]*linearVel.z +
                    _hydroCoeff.find(linkId)->second["L_qdot"]*angularVel.y;
        
        double b3 = _hydroCoeff.find(linkId)->second["M_vdot"]*linearVel.y  +
                    _hydroCoeff.find(linkId)->second["M_pdot"]*angularVel.x +
                    _hydroCoeff.find(linkId)->second["M_rdot"]*angularVel.z ;

        Matrix6d coriolisMatrix;
        Vector6d velocityVector;
        Vector6d accelarationVector;
        Vector6d forceTorqueVector;

        coriolisMatrix << 0, 0, 0, 0, a3, -1*a2,
                          0, 0, 0, -1*a3, 0, a1,
                          0, 0, 0, a2, -1*a1, 0,
                          0, a3, -1*a2, 0, b3, -1*b2,
                          -1*a3, 0, a1, -1*b3, 0, b1,
                          a2, -1*a1, 0, b2, -1*b1, 0;

        velocityVector << linearVel.x, linearVel.y, linearVel.z,
                          angularVel.x, angularVel.y, angularVel.z; 
        accelarationVector << linearAcc.x, linearAcc.y, linearAcc.z,
                              angularAcc.x, angularAcc.y, angularAcc.z; 

        forceTorqueVector = -1*(_addedMassMatrix[linkId]*accelarationVector + 
                                         coriolisMatrix*velocityVector);
        return forceTorqueVector;
    }
}
