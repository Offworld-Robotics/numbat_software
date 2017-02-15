#include <boost/type_traits/is_base_of.hpp>

#include "owr_mapping/AbstractMap.hpp"

template<class T> class AbstractLocalMap : public AbstractMap {
    // requires this class to have a template parameter of AbstractFeature as a compile time check
    BOOST_STATIC_ASSERT((boost::is_base_of<AbstractFeature, T>::value));
    public:
        virtual void insert_many(T * features) = 0;
        virtual void insert(T feature) = 0;
        void associate_grid(ScaledOccupancyGrid grid, int scale) = 0;
        // used to finalise editing of the map
        virtual void finalise() = 0;
        virtual ScaledOccupancyGrid get_grid();

    protected:
        ScaledOccupancyGrid grid;
        
};