/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Humanoid Robotics Lab      Georgia Institute of Technology
 * Director: Mike Stilman     http://www.golems.org
 */

/**
 * @author A. Huaman
 * @date 2011-11-06
 */
#include "GRIPApp.h"
#include "BenchmarkTests.h"

extern wxNotebook* tabView;

class BenchmarkTestsApp : public GRIPApp {
	virtual void AddTabs() {
		tabView->AddPage(new BenchmarkTests(tabView), wxT("Benchmark Tests"));
	}
};

IMPLEMENT_APP(BenchmarkTestsApp)
