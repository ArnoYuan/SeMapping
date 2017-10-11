/*
 * Main.cpp
 *
 *  Created on: 2016年8月17日
 *      Author: seeing
 */

#include "GMapping/GMappingApplication.h"
#include "Hector/HectorMappingApplication.h"
#include <stdlib.h>
#include <signal.h>
#include <sys/wait.h>

using namespace NS_GMapping;
//using namespace NS_HectorMapping;

GMappingApplication* app;

static void signalAction(int signal)
{
  printf("received term signal, quitting!\n");
  app->quit();
  app->terminate();
}

void registerSignal()
{
  //signal (SIGINT, signalAction);
  //signal (SIGKILL, signalAction);
  //signal (SIGQUIT, signalAction);
  //signal (SIGTERM, signalAction);
  signal(SIGUSR1, signalAction);
}

int main(int argc, char* argv[])
{
  app = new GMappingApplication;

  registerSignal();

  if(!app->initialize(argc, argv))
  {
    exit(EXIT_FAILURE);
    return 0;
  }

  app->run();

  if(!app->isRunning())
  {
    exit(EXIT_FAILURE);
    return 0;
  }

  app->pending();

  exit(EXIT_SUCCESS);
  return 0;
}
