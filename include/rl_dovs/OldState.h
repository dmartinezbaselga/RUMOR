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
    int dObs;
    int thObs;
    int relativeTh;
    int relativeVel;
    int collision;

    // State code
    long code;

public:

    State() {}

    State(int vel, int angular, int closestFreeVel, int dGoal, int thGoal, int thObs, int relativeTh, int relativeVel, int dObs): vel(vel), angular(angular), closestFreeVel(closestFreeVel), dGoal(dGoal), thGoal(thGoal), thObs(thObs), relativeTh(relativeTh), relativeVel(relativeVel), collision(collision), dObs(dObs)/*, thCB1(thCB1), thCB2(thCB2)*/ {
        this->code = vel + angular*10 + closestFreeVel*100 + dGoal*1000 + thGoal*10000 + thObs*100000 + relativeTh*1000000 + relativeVel*10000000 + (long) dObs*100000000;
    }

    State(long code): code(code) {}

    bool getTowardsCollision() {
        return this->dObs > 0;
    }

    int getDistanceToGoal() {
        return this->dGoal;
    }

    long getCode() const {
        return this->code;
    }

	bool operator<(const State &state) const {
		return this->code < state.code;
	}

    void printState() {
        std::cout << "* State " << this->code << std::endl;
        std::cout << "thObs: " << this->thObs << std::endl;
        std::cout << "thRel: " << this->relativeTh << std::endl;
        std::cout << "collision: " << this->collision << std::endl;
        std::cout << "dObs: " << this->dObs << std::endl;
        std::cout << "vObs: " << this->relativeVel << std::endl;
        std::cout << "v: " << this->vel << std::endl;
        std::cout << "w: " << this->angular << std::endl;
    }

};

#endif