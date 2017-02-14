#include "owr_mapping/AbstractMap.hpp"

template<class T> class AbstractLocalMap : public AbstractMap {
    static_assert(std::is_base_of<AbstractFeature, T>::value, "forces valid type paramter on compile");
    public:
        virtual void insert_many(T[] features) = 0;
        virtual void insert(T : feature) = 0;
        virtual void associate_grid(ScalledOccupancyGrid grid, int sclae) = 0;
        // used to finalise editing of the map
        virtual void finalise() = 0;
        
};