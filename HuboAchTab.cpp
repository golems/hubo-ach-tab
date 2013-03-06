 /*
 * Copyright (c) 2010, Georgia Tech Research Corporation
 * 
 * Humanoid Robotics Lab      Georgia Institute of Technology
 * Director: Mike Stilman     http://www.golems.org
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *     * Redistributions of source code must retain the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials
 *       provided with the distribution.
 *     * Neither the name of the Georgia Tech Research Corporation nor
 *       the names of its contributors may be used to endorse or
 *       promote products derived from this software without specific
 *       prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY GEORGIA TECH RESEARCH CORPORATION ''AS
 * IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL GEORGIA
 * TECH RESEARCH CORPORATION BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @author Saul Reynolds-Haertle
 */

#include "HuboAchTab.h"
#include "HuboController.h"

#include <cstdlib>
#include <inttypes.h>
#include <ctime>
#include <ach.h>
#include <hubo-jointparams.h>
#include <hubo.h>
#include <iostream>
#include <iomanip>
#include <queue>

#include <wx/wx.h>
#include <GUI/Viewer.h>
#include <GUI/GUI.h>
#include <GUI/GRIPSlider.h>
#include <GUI/GRIPFrame.h>
#include <kinematics/Dof.h>
using namespace std;

namespace HACHT {
    //###########################################################
    //###########################################################
    //#### Event table
    //###########################################################
    //###########################################################

    // Control IDs
    enum HuboAchTabEvents {
    };

    BEGIN_EVENT_TABLE(HuboAchTab, wxPanel)
    END_EVENT_TABLE()

    //###########################################################
    //###########################################################
    //#### Constructor
    //###########################################################
    //###########################################################

    IMPLEMENT_DYNAMIC_CLASS(HuboAchTab, GRIPTab)

    HuboAchTab::HuboAchTab(wxWindow *parent,
                               const wxWindowID id,
                               const wxPoint& pos,
                               const wxSize& size,
                               long style)
    : GRIPTab(parent, id, pos, size, style) {
        memset( &H_param, 0, sizeof(H_param));
        memset( &H_param, 0, sizeof(H_state));
    }
    
    //###########################################################
    //###########################################################
    //#### wxwidgets UI event handlers
    //###########################################################
    //###########################################################

    //###########################################################
    //###########################################################
    //#### GRIPTab hooks
    //###########################################################
    //###########################################################
    
    void HuboAchTab::GRIPEventRender() {
        static bool loaded = false;
        if (!loaded)
        {
            loaded = true;
            std::cout << "trying to load installed world from /usr/share/hubo-ach-tab" << std::endl;
            frame->DoLoad("/usr/share/hubo-ach-tab/hubo-models/huboplus-empty-world.urdf", false);
            if (mWorld == NULL) {
                std::cout << "Failed to load installed world. Please load a world with a hubo in it." << std::endl;
            }
        }
    }
    
    // tree view selection changed
    void HuboAchTab::GRIPStateChange() {
    }

    // scene loaded
    void HuboAchTab::GRIPEventSceneLoaded() {
        std::string huboname = "huboplus";
        for(int i = 0; i < mWorld->getNumRobots(); i++)
            if (mWorld->getRobot(i)->getName().compare(huboname) == 0)
                hubo = mWorld->getRobot(i);
        if (hubo == NULL) {
            std::cout << "Could not find hubo!" << std::endl;
            return;
        }
        
        Eigen::VectorXd controller_mask = Eigen::VectorXd::Ones(hubo->getNumDofs());
        for (int i = 0; i < 5; i++) { controller_mask[i] = 0; }
        
        Eigen::VectorXd K_p = 1000.0 * Eigen::VectorXd::Ones(hubo->getNumDofs());
        Eigen::VectorXd K_i = 100.0 * Eigen::VectorXd::Ones(hubo->getNumDofs());
        Eigen::VectorXd K_d = 100.0 * Eigen::VectorXd::Ones(hubo->getNumDofs());
        contr = new HuboController(hubo, K_p, K_i, K_d, controller_mask, mWorld->mTime - mWorld->mTimeStep);
        contr->ref_pos = Eigen::VectorXd::Zero(hubo->getNumDofs());

        if (!HuboInit()) {
            std::cout << "Could not start simulating. Did you load the right world and create all your ach channels?" << std::endl;
        }
        else {
            // our channels are open and our robot is loaded, so why
            // don't we just start simulating immediately?
            std::cout << "Automatically starting simulation" << std::endl;
            frame->continueSimulation = true;
            wxYield();
            int type = 0;
            wxCommandEvent evt(wxEVT_GRIP_SIMULATE_FRAME,GetId());
            evt.SetEventObject(this);
            evt.SetClientData((void*)&type);
            frame->SimulateFrame(evt);
        }
    }

    // scene unloaded
    void HuboAchTab::GRIPEventSceneUnLoaded() {
        ach_close(&chan_hubo_ref);
        ach_close(&chan_hubo_state);
        delete contr;
    }

    // Before simulation timestep
    void HuboAchTab::GRIPEventSimulationBeforeTimestep() {
        ReadRefs();
        hubo->setInternalForces(contr->getTorques(hubo->getPose(),
                                                  hubo->getQDotVector(),
                                                  mWorld->mTime));
    }

    // After simulation timestep
    void HuboAchTab::GRIPEventSimulationAfterTimestep() {
        WriteState();
    }
        
    //###########################################################
    //###########################################################
    //#### HUBO emulation - core functions
    //###########################################################
    //###########################################################

    // set up HUBO structures and open and initialize ach channels
    // and structs.
    bool HuboAchTab::HuboInit() {
        setJointParams(&H_param, &H_state);
        setSensorDefaults(&H_param);
        
        int r;
        // open hubo reference channel
        r = ach_open(&chan_hubo_ref, HUBO_CHAN_REF_NAME, NULL);
        if(r != ACH_OK) {
            std::cout << "Error: failed to open reference channel" << std::endl;
            return false;
        }
        // open hubo state channel
        r = ach_open(&chan_hubo_state, HUBO_CHAN_STATE_NAME, NULL);
        if(r != ACH_OK) {
            std::cout << "Error: failed to open state channel" << std::endl;
            return false;
        }

        // Map joints from physical hubo to virtual hubo

        // these are the special cases - joints where teh names don't
        // match up perfectly and we have to manually define
        // correspondences.
        map<std::string, std::string> special_cases = {
            { "RKN", "RKP" }, // Right Knee Pitch
            { "LKN", "LKP" }, // Left Knee Pitch
            { "REB", "REP" }, // Right Elbow Pitch
            { "LEB", "LEP" }, // Left Elbow Pitch
        };
        bool foundajoint = false;
        for(int i = 0; i < HUBO_JOINT_COUNT; i++) {
            std::string name = std::string(H_param.joint[i].name);
            if (special_cases.count(name)) { name = special_cases[name]; }
            int i_vir = FindNamedDof(name);
            if ((H_state.joint[i].active) && (i_vir != -1)) {
                jointmap_phys_to_virtual[i] = i_vir;
                jointmap_virtual_to_phys[i_vir] = i;
                foundajoint = true;
            }
        }
        
        if (!foundajoint) {
            std::cout << "Could not find find any joints/dofs with corresponding names. Is this the right hubo?" << std::endl;
            return false;
        }

        // find some links that we'll be putting sensors on
        hubo_waist = FindNamedNode("Body_Hip");
        if (hubo_waist == NULL) {
            std::cout << "Could not find waist. Is this the right hubo?" << std::endl;
            return false;
        }
        hubo_foot_left = FindNamedNode("Body_LAR");
        hubo_foot_right = FindNamedNode("Body_RAR");
        if (hubo_foot_left == NULL || hubo_foot_right == NULL) {
            std::cout << "Could not find feet. Is this the right hubo?" << std::endl;
            return false;
        }

        // and done
        return true;
    }

    // read new refs out of ach channels
    void HuboAchTab::ReadRefs() {
        // define variables
        hubo_ref_t H_ref;
        memset(&H_ref, 0, sizeof(H_ref));

        // get data from channel
        ach_status_t r;                  // result
        size_t fs;              // received frame size
        r = ach_get(&chan_hubo_ref, &H_ref, sizeof(H_ref), &fs, NULL, ACH_O_LAST);
        switch(r) {
        case ACH_OK:
            // std::cout << "Got message" << std::endl;
            break;
        case ACH_STALE_FRAMES:
            break;
        default:
            std::cout << "Get reference failed: " << ach_result_to_string(r) << std::endl;
            break;
        }
        
        for (int i = 0; i < HUBO_JOINT_COUNT; i++) {
            int i_vir = jointmap_phys_to_virtual[i];
            if (i_vir != -1) {
                contr->ref_pos[i_vir] = H_ref.ref[i];
            }
        }
    }

    // write new state into ach channels
    void HuboAchTab::WriteState() {
        // fill out joints
        for (int i = 0; i < hubo->getNumDofs(); i++) {
            Eigen::VectorXd pos = hubo->getPose();
            Eigen::VectorXd vel = hubo->getQDotVector();
            
            int i_phys = jointmap_virtual_to_phys[i];
            if (i_phys != -1) {
                H_state.joint[i_phys].ref = contr->ref_pos[i];
                H_state.joint[i_phys].pos = pos[i];
                H_state.joint[i_phys].cur = 0.0;
                H_state.joint[i_phys].vel = vel[i];
                H_state.joint[i_phys].heat = 0.0;
                H_state.joint[i_phys].tmp = 0.0;
            }
        }
        

        // fill out IMU
        // fill out force-torque
        // fill out joint statuses
        // fill out motor controller states
        // fill out rest of state struct
        H_state.time = mWorld->mTime;
        H_state.refWait = 0.0;
        // send data to channel
        ach_put( &chan_hubo_state, &H_state, sizeof(H_state));
    }

    //###########################################################
    //###########################################################
    //#### HUBO emulation - helpers
    //###########################################################
    //###########################################################

    int HuboAchTab::FindNamedDof(std::string name) {
        for (int i = 0; i < hubo->getNumDofs(); i++) {
            if (name.compare(hubo->getDof(i)->getName()) == 0) {
                return i;
            }
        }
        return -1;
    }
    dynamics::BodyNodeDynamics* HuboAchTab::FindNamedNode(std::string name) {
        for (int i = 0; i < hubo->getNumNodes(); i++) {
            if (name.compare(hubo->getNode(i)->getName()) == 0) {
                return (dynamics::BodyNodeDynamics*)hubo->getNode(i);
            }
        }
        return NULL;
    }
}
