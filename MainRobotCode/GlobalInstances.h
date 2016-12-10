#ifndef GLOBAL_INSTANCES_H
#define GLOBAL_INSTANCES_H

#include "DriveController.h"
#include "WallFollowState.h"
#include "ForwardState.h"
#include "TurnState.h"
#include "SpecCaseState.h"

static DriveController* myDriveControl;
static WallFollowState* myWallState;
static ForwardState* myForwardState;
static TurnState* myTurnState;
static SpecCaseState* mySpecCaseState;

static void initalizeInstances() {
  myDriveControl = DriveController::getInstance();
  myDriveControl->setup();
  myWallState = WallFollowState::getInstance();
  myForwardState = ForwardState::getInstance();
  myTurnState = TurnState::getInstance();
  mySpecCaseState = SpecCaseState::getInstance();
}

typedef enum RobotState {
  WALL_FOLLOW,
  TURN,
  STANDBY,
  SPEC_CASE, //Detected that front wall sensor maxed and other isn't
  FORWARD_DIST
};

static RobotState currentState = STANDBY;
static RobotState previousState = STANDBY;

#endif
