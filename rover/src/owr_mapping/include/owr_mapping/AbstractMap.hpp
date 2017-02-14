/*
 * Date Started: 4/02/2017
 * Original Author: Harry J.E Day
 * Editors: 
 * ROS Node Name: owr_mapping
 * ROS Package: owr_package_name
 * Purpose: this represents an abstract map class. It is mainly used to access Scalled Occupany Grid
 */
#include <vector>
#include "owr_mapping/AbstractFeature.hpp"
#include "owr_mapping/ScaledOccupancyGrid.hpp"

class AbstractMap {
    
    public:
        virtual ScaledOccupancyGrid get_grid() = 0;
        virtual std::vector<AbstractFeature> get_features();
        
  
};