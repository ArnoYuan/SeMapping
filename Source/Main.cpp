/*
 * Main.cpp
 *
 *  Created on: 2016年8月17日
 *      Author: seeing
 */

#include "GMapping/GMappingApplication.h"
#include "Hector/HectorMappingApplication.h"

using namespace NS_GMapping;
//using namespace NS_HectorMapping;

int main(int argc, char* argv[]) {
	GMappingApplication app;
	//HectorMappingApplication app;

	if (!app.initialize(argc, argv)) {
		return -1;
	}

	app.run();

	app.pending();

	return 0;
}

