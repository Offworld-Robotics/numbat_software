/*
 * Date Started: 4/02/2017
 * Original Author: Harry J.E Day
 * Editors: 
 * ROS Node Name: owr_mapping
 * ROS Package: owr_package_name
 * Purpose: Represents a Graph of AbstractMaps, 
 */
#include <geometry_msgs/Pose.h>

#include "owr_mapping/AbstractMap.hpp"

typedef struct _edge {

} Edge;
 
class MapGraph : public AbstractMap {
    public:
        std::iterator<Edge> edges;
        /**
        * Addes an edge to the graph of maps, connecting two maps
        * @param map1_coord the cordinate of where the two maps overlap in the frame of the first map
        * @param map2_coord the cordinate of where the two maps overlap in the frame of the second map
        * @param map1 the first map
        * @param map2 the second map
        * @param weighting the weighting of the edge
        */
        void add_edge(geometry_msgs::Pose map1_coord, geometry_msgs::Pose map2_coord, AbstractMap * map1, AbstractMap * map2, double weighting);
        /**
         * Removes the given edge from the edges list
         * @param edge the edge to remove
         */
        void drop_edge(Edge * edge);

        void calculate_graph();
        void publish();
        void estimate_pose();
    private:
        void publish_pose();

};