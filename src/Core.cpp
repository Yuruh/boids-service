//
// Created by antoine on 10/07/20.
//

#include "../include/Core.h"

// could use ncurses to redraw
void Core::displayNextMovements(int n, float refreshRateSec) {
    this->map.display();
}

//The main method should take as param the refresh rate, and the number of seconds to simulate.
//It should return the map state for every possibility
// E.G 60 FPS 2 seconds, 120 maps for every (1 / 60) sec