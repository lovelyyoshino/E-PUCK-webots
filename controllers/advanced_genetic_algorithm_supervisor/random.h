#ifndef RANDOM_H
#define RANDOM_H

//   File:          random.h
//   Description:   Random number functions
//   Project:       Advanced exercises in Cyberbotics' Robot Curriculum
//   Author:        Yvan Bourquin - www.cyberbotics.com
//   Date:          January 6, 2010

// returns a random integer number between [0;max-1] from a uniform distribution
int random_get_integer(int max);

// returns a random number between [0;1] from a uniform distribution
double random_get_uniform();

// returns a random number from a Gaussian distribution with mean 0 and standard deviation 1
double random_get_gaussian();

#endif
