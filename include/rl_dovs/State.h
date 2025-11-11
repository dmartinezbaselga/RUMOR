/*
 * State.h
 * Author: Andrew
 */

#ifndef STATE_H
#define STATE_H

class State {
    
    // Agent states
    int vel; 
    int angular;
    int closestFreeVel;

    // Goal states
    int dGoal;
    int thGoal;

    // Obstacle states
    int relativeTh;
    int relativeVel;
    int collision;

    // State code
    long code;

public:

    // Constructor
    State() {}

    // Constructor
    State(int vel, int angular, int closestFreeVel, int dGoal, int thGoal, int relativeTh, 
          int relativeVel, int collision): vel(vel), angular(angular), 
          closestFreeVel(closestFreeVel), dGoal(dGoal), thGoal(thGoal), collision(collision), relativeTh(relativeTh), 
          relativeVel(relativeVel) {
        
        // Calculate state's identifier code
        this->code = vel + angular*10 + closestFreeVel*100 + dGoal*1000 + thGoal*10000 + collision*100000 +
                   + relativeTh*1000000 + relativeVel*10000000;
    }

    // Constructor
    State(long code): code(code) {}

    // Return true if the agent is in danger
    bool getTowardsCollision() {
        return this->collision > 0;
    }

    // Return distance to goal state
    int getDistanceToGoal() {
        return this->dGoal;
    }

    // Return state's identifier
    long getCode() const {
        return this->code;
    }

    // Define higher than property to sort states in hash table
	bool operator<(const State &state) const {
		return this->code < state.code;
	}

};

#endif