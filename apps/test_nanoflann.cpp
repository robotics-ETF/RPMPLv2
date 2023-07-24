//
// Created by dinko on 24.5.21.
// Taken from nanoflann examples
//

#include <nanoflann.hpp>

#include <ctime>
#include <cstdlib>
#include <iostream>
#include "RealVectorSpaceState.h"
#include "RealVectorSpace.h"
#include "State.h"

// TODO: NOT working with smart pointer changes

void generateRandomStates(base::StateSpace* ss, std::vector<base::State*>& states, int N = 20)
{

}

void buildKDTree(std::vector<base::State*>& states)
{

}

int main()
{
	/*base::StateSpace *ss = new base::RealVectorSpace(2);
	std::vector<base::State*> states;
	generateRandomStates(ss, states, 100);
	buildKDTree(states);
	return 0;*/
}

