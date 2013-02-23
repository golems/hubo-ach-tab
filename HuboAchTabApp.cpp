#include <GRIPApp.h>
#include "HuboAchTab.h"

extern wxNotebook* tabView;

class HuboAchTabApp : public GRIPApp {
    virtual void AddTabs() {
        tabView->AddPage(new HACHT::HuboAchTab(tabView), wxT("HuboAchTab"));
    }
};

IMPLEMENT_APP(HuboAchTabApp)

