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


#include "HuboAchTab.h"
#include "HuboController.h"

#include <cstdlib>
#include <inttypes.h>
#include <ctime>
#include <ach.h>

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
        id_button_startHUBO = wxID_HIGHEST+1,
        id_button_stopHUBO
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
    }
    
    //###########################################################
    //###########################################################
    //#### wxwidgets UI event handlers
    //###########################################################
    //###########################################################

    void onButtonStartHUBO(wxCommandEvent & _evt) {
    }

    void onButtonStopHUBO(wxCommandEvent & _evt) {
    }

    //###########################################################
    //###########################################################
    //#### GRIPTab hooks
    //###########################################################
    //###########################################################

    // tree view selection changed
    void HuboAchTab::GRIPStateChange() {
    }

    // scene loaded
    void HuboAchTab::GRIPEventSceneLoaded() {
        std::string huboname = "GolemHubo";
        for(int i = 0; i < mWorld->getNumRobots(); i++)
            if (mWorld->getRobot(i)->getName().compare(huboname) == 0)
                hubo = mWorld->getRobot(i);
        if (hubo == NULL) {
            std::cout << "Could not find hubo!" << std::endl;
            return;
        }

        if (!HuboInit()) {
            std::cout << "Failed to initialize hubo. Did you load the right world?" << std::endl;
        }
        
        Eigen::VectorXd K_p = 1000.0 * Eigen::VectorXd::Ones(hubo->getNumDofs());
        Eigen::VectorXd K_i = 100.0 * Eigen::VectorXd::Ones(hubo->getNumDofs());
        Eigen::VectorXd K_d = 100.0 * Eigen::VectorXd::Ones(hubo->getNumDofs());
        contr = new HuboController(hubo, K_p, K_i, K_d, mWorld->mTime - mWorld->mTimeStep);
        
        contr->ref_pos = Eigen::VectorXd::Zero(hubo->getNumDofs());
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

        jointmap_phys_to_virtual = {
            { RHY, FindNamedLink("RHY") }, // Right Hip Yaw
            { RHR, FindNamedLink("RHR") }, // Right Hip Roll
            { RHP, FindNamedLink("RHP") }, // Right Hip Pitch
            { RKN, FindNamedLink("RKP") }, // Right Knee Pitch
            { RAP, FindNamedLink("RAP") }, // Right Ankle Pitch
            { RAR, FindNamedLink("RAR") }, // Right Ankle Roll

            { LHY, FindNamedLink("LHY") }, // Left Hip Yaw
            { LHR, FindNamedLink("LHR") }, // Left Hip Roll
            { LHP, FindNamedLink("LHP") }, // Left Hip Pitch
            { LKN, FindNamedLink("LKP") }, // Left Knee Pitch
            { LAP, FindNamedLink("LAP") }, // Left Ankle Pitch
            { LAR, FindNamedLink("LAR") }, // Left Ankle Roll

            { RSP, FindNamedLink("RSP") }, // Right Shoulder Pitch
            { RSR, FindNamedLink("RSR") }, // Right Shoulder Roll
            { RSY, FindNamedLink("RSY") }, // Right Shoulder Yaw
            { REB, FindNamedLink("REP") }, // Right Elbow Pitch
            { RWY, FindNamedLink("RWY") }, // right wrist yaw
            { RWP, FindNamedLink("RWP") }, // right wrist Pitch

            { LSP, FindNamedLink("LSP") }, // Left Shoulder Pitch
            { LSR, FindNamedLink("LSR") }, // Left Shoulder Yaw
            { LSY, FindNamedLink("LSY") }, // Left Shoulder Roll
            { LEB, FindNamedLink("LEP") }, // Left Elbow Pitch
            { LWY, FindNamedLink("LWY") }, // left wrist yaw
            { LWP, FindNamedLink("LWP") }, // left wrist pitch

            { WST, FindNamedLink("HNR") }, // Trunk Yaw
        };
        for (auto it = jointmap_phys_to_virtual.begin(); it != jointmap_phys_to_virtual.end(); it++)
            jointmap_virtual_to_phys[it->second] = it->first;

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
        // define variables
        hubo_state_t H_state;
        memset(&H_state, 0, sizeof(H_state));

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
                H_state.joint[i_phys].active = 1;
                H_state.joint[i_phys].zeroed = false;
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

    int HuboAchTab::FindNamedLink(std::string lname) {
        for (int i = 0; i < hubo->getNumDofs(); i++) {
            if (lname.compare(hubo->getDof(i)->getName()) == 0) {
                // std::cout << "Link " << lname << " has index " << i << std::endl;
                return i;
            }
        }
        // std::cout << "Did not find link " << lname << std::endl;
        return -1;
    }
}
