#include "owr_mapping/AbstractMap.hpp"

class AbstractLocalMap<class T extends AbstractFeature> : public AbstractMap {
    public:
        virtual void insert_many(T[] features) = 0;
        virtual void insert(T : feature) = 0;
        virtual void associate_grid(ScalledOccupancyGrid grid, int sclae) = 0;
        // used to finalise editing of the map
        virtual void finalise() = 0;
        
}