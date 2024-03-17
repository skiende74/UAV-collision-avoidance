/*
State Machine decide the state of UAV.
Input : Current state, Conditions
Output : Next state
S1 : Idle state, auto flight mode
S2 : Priority check state
S3 : Mode change state
S4 : Avoidance state, Offboard flight mode
*/

#include "state_machine.h"


StateMachine::
StateMachine() {
    condition.state = 1;
    condition.threat = false;
    condition.priority = false;
    condition.mode_changed = true;
    condition.offboard = false;
}

StateMachine::
~StateMachine() {

}

void 
StateMachine::
state_shifter() {
    switch(condition.state) {
        case 1:
            if (condition.threat==true) {
                condition.state = 2;
            }
            break;
        case 2:
            if (condition.priority==true) {
                condition.state = 1;
            } else {
                condition.state = 3;
                condition.prev_state = 2;
                condition.mode_changed = false;
            }
            break;
        case 3:
            if (condition.mode_changed==true) {
                if (condition.prev_state==2) {
                    condition.state = 4;
                } else {
                    condition.state = 1;
                }
            }
            break;
        case 4:
            if (condition.threat==false) {
                condition.state = 3;
                condition.prev_state = 4;
                condition.mode_changed = false;
            }
            break;
        default:
            printf("ERROR : No such state!\n");
            break;
    }
    

}