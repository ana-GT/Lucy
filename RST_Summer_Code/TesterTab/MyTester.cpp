#include "RSTApp.h"
#include "MyTesterTab.h"

extern wxNotebook* tabView;

class MyTesterApp : public RSTApp {
	virtual void AddTabs() 
         {
           tabView->AddPage( new MyTesterTab(tabView), wxT("My Tester"));
         }

};

IMPLEMENT_APP( MyTesterApp )
