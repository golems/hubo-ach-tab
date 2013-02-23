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

#include <wx/wx.h>
#include <GUI/Viewer.h>
#include <GUI/GUI.h>
#include <GUI/GRIPSlider.h>
#include <GUI/GRIPFrame.h>
#include "HuboController.h"
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

        HuboInit();
        
        Eigen::VectorXd K_p = 1000.0 * Eigen::VectorXd::Ones(hubo->getNumDofs());
        Eigen::VectorXd K_i = 100.0 * Eigen::VectorXd::Ones(hubo->getNumDofs());
        Eigen::VectorXd K_d = 100.0 * Eigen::VectorXd::Ones(hubo->getNumDofs());
        contr = new HuboController(hubo, K_p, K_i, K_d, mWorld->mTime - mWorld->mTimeStep);
        
        contr->ref_pos = Eigen::VectorXd::Zero(hubo->getNumDofs());
    }

    // scene unloaded
    void HuboAchTab::GRIPEventSceneUnLoaded() {
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
    void HuboAchTab::HuboInit() {
    }

    // read new refs out of ach channels
    void HuboAchTab::ReadRefs() {
    }

    // write new state into ach channels
    void HuboAchTab::WriteState() {
    }

    //###########################################################
    //###########################################################
    //#### HUBO emulation - helpers
    //###########################################################
    //###########################################################
}
