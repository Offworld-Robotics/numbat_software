//
// Created by hjed on 15/02/17.
//

#include "owr_mapping/AbstractLocalMap.hpp"

template <class T>
void AbstractLocalMap<T>::associate_grid(ScaledOccupancyGrid newGrid, int scale) {
    grid = newGrid;
}

template <class T>
ScaledOccupancyGrid AbstractLocalMap<T>::get_grid() {
    return grid;
}