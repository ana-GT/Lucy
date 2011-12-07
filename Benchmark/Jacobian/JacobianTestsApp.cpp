/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Humanoid Robotics Lab      Georgia Institute of Technology
 * Director: Mike Stilman     http://www.golems.org
 */

/**
 * @author A. Huaman
 * @date 2011-12-06
 */
#include "GRIPApp.h"
#include "JacobianTests.h"

extern wxNotebook* tabView;

class JacobianTestsApp : public GRIPApp {
	virtual void AddTabs() {
		tabView->AddPage(new JacobianTests(tabView), wxT("Jacobian Tests"));
	}
};

IMPLEMENT_APP(JacobianTestsApp)
