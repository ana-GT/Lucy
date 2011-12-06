#include "RSTApp.h"
#include "HopeTab.h"

extern wxNotebook* tabView;

class HopeApp : public RSTApp {
	virtual void AddTabs() 
         {
           tabView->AddPage( new HopeTab(tabView), wxT("Hope"));
         }

};

IMPLEMENT_APP( HopeApp )
