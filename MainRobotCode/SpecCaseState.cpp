#include "SpecCaseState.h"
#include "GlobalInstances.h"
#include "RobotOdometry.h"

SpecCaseState::SpecCaseState() {
}

void SpecCaseState::handle() {
  static enum SpecCaseStep{
    FORWARD_FIRST,
    TURN_FIRST,
    FORWARD_SECOND,
    TURN_SECOND,
    FORWARD_THIRD,
    DONE
  } myState = FORWARD_FIRST;
  switch(myState)
  {
    case FORWARD_FIRST:
    previousState = currentState;
    //set forward target
    myForwardState->setForwardTarget(25.4*10);
    currentState = FORWARD_DIST;
    myState = TURN_FIRST;
    break;
    case TURN_FIRST:
    previousState = currentState;
    //set turn target
    myTurnState->setTurnHeading(getTheta()-(PI/2));
    currentState = TURN;
    myState = FORWARD_SECOND;
    break;
    case FORWARD_SECOND:
    previousState = currentState;
    //set forward target
    myForwardState->setForwardTarget(25.4*22);
    currentState = FORWARD_DIST;
    myState = TURN_SECOND;
    break;
    case TURN_SECOND:
    previousState = currentState;
    //set turn target
    myTurnState->setTurnHeading(getTheta()-(PI/2));
    currentState = TURN;
    myState = FORWARD_THIRD;
    break;
    case FORWARD_THIRD:
    previousState = currentState;
    //set forward target
    myForwardState->setForwardTarget(25.4*15);
    currentState = FORWARD_DIST;
    myState = DONE;
    break;
    case DONE:
    previousState = currentState;
    currentState = WALL_FOLLOW;
    myState = FORWARD_FIRST;
    break;
    default:
    break;
  }
}
