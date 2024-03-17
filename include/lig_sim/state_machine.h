#pragma once

#include <cstdlib>
#include <stdio.h>
#include <unistd.h>

/*
State Machine decide the state of UAV.
Input : Current state, Conditions
Output : Next state
S1 : Idle state, auto flight mode
S2 : Priority check state
S3 : Mode change state
S4 : Avoidance state, Offboard flight mode
*/

// Structures
typedef struct conditions {
    int state;
    bool threat;
    bool priority;
    bool mode_changed;
    bool offboard;
    int prev_state;
} conditions_t;

// Class
class StateMachine
{
public:
    StateMachine();
    ~StateMachine();

    // Variables
    conditions_t condition;

    // Functions
    void state_shifter();

private:

    // Variables

    // Functions
    
};