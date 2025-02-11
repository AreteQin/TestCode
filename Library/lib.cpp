//
// Created by qin on 12/1/22.
//

#include "lib.h"
#include <iostream>

void lib::StaticFunc() {
    std::cout<< "StaticFunc" << std::endl;
}

void lib::Execute() {
    std::cout<< "Execute" << std::endl;
    StaticFunc();
}

