#include <explore/frontier_search.h>

#include <mutex>

#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/Point.h>

#include <explore/costmap_tools.h>

namespace frontier_exploration {
using costmap_2d::FREE_SPACE;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;

FrontierSearch::FrontierSearch(costmap_2d::Costmap2D* costmap, double potential_scale, double gain_scale,
                               double min_frontier_size)
    : costmap_(costmap)
    , potential_scale_(potential_scale)
    , gain_scale_(gain_scale)
    , min_frontier_size_(min_frontier_size) {}

std::vector<Frontier> FrontierSearch::searchFrom(geometry_msgs::Point position) {
    //store the frontier data
    std::vector<Frontier> frontier_list;

    // while search the frontier check the robot exist in the costmap
    unsigned int mx, my;
    if (!costmap_->worldToMap(position.x, position.y, mx, my)) { 
        ROS_ERROR("Robot out of costmap bounds, cannot search for frontiers");
        return frontier_list;
    }

    // make sure the cost map is valid
    std::lock_guard<costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));

    // the size of the costmap
    map_ = costmap_->getCharMap();
    size_x_ = costmap_->getSizeInCellsX();
    size_y_ = costmap_->getSizeInCellsY();

    
    std::vector<bool> frontier_flag(size_x_ * size_y_, false); 
    std::vector<bool> visited_flag(size_x_ * size_y_, false);

    // breadth first search, bfs
    std::queue<unsigned int> bfs;
     // transfer the map matrix into a one dimension linked list
    unsigned int clear, pos = costmap_->getIndex(mx, my);
    // check the nearest cell
    if (nearestCell(clear, pos, FREE_SPACE,
                    *costmap_)) { 
        bfs.push(clear);         
    } else {
        bfs.push(pos); 
        ROS_WARN("Could not find nearby clear cell to start search");
    }
    visited_flag[bfs.front()] = true; // mark the searched cell

    
    // template of bfs
    while (!bfs.empty()) {
        unsigned int idx = bfs.front();
        bfs.pop();

        // traversal 4 nearest cell
        for (unsigned nbr : nhood4(idx, *costmap_)) {
            
            // if the cost of cell is less than bfs.front(),and not searched before,
            // put them onto the list for the future search
            if (map_[nbr] <= map_[idx] && !visited_flag[nbr]) {
                visited_flag[nbr] = true;
                bfs.push(nbr);
            }
            
            else if (isNewFrontierCell(
                         nbr, frontier_flag)) { //check the another four cell of the current cell has 
            	                                // valid area
                frontier_flag[nbr] = true;

                //buildNewFrontiler,use bfs make the current cell as the center and extend until find the farest cell
                //take all valid cells put them into the new_frontier
                Frontier new_frontier = buildNewFrontier(nbr, pos, frontier_flag);
                if (new_frontier.size * costmap_->getResolution() >= min_frontier_size_) {
                    frontier_list.push_back(new_frontier);
                }
            }
        }
    }

    // set the cost of frontier
    for (auto& frontier : frontier_list) {
        frontier.cost = frontierCost(frontier); //less frontier, more distance from robot, more cost
    }
    // recursive sort to the frontier
    std::sort(frontier_list.begin(), frontier_list.end(),
              [](const Frontier& f1, const Frontier& f2) { return f1.cost < f2.cost; });

    return frontier_list;
}

Frontier FrontierSearch::buildNewFrontier(unsigned int initial_cell, unsigned int reference,
                                          std::vector<bool>& frontier_flag) {
    // initialze the frontier structural
    Frontier output;
    output.centroid.x = 0;
    output.centroid.y = 0;
    output.size = 1;
    output.min_distance = std::numeric_limits<double>::infinity(); 

    // record all the points have connection with frontier
        unsigned int ix, iy;
    costmap_->indexToCells(initial_cell, ix,   //dimension transfer
                           iy); 
    costmap_->mapToWorld(ix, iy, output.initial.x,
                         output.initial.y); 

    // push the initial cell into the list
    std::queue<unsigned int> bfs;
    bfs.push(initial_cell);

    // store robot co-ordinate
    unsigned int rx, ry;
    double reference_x, reference_y;
    costmap_->indexToCells(reference, rx, ry);
    costmap_->mapToWorld(rx, ry, reference_x, reference_y);

    while (!bfs.empty()) {
        unsigned int idx = bfs.front();
        bfs.pop();

       
        for (unsigned int nbr : nhood8(idx, *costmap_)) {
            
            if (isNewFrontierCell(nbr, frontier_flag)) {
              
                frontier_flag[nbr] = true;
                unsigned int mx, my;
                double wx, wy;
                costmap_->indexToCells(nbr, mx, my);
                costmap_->mapToWorld(mx, my, wx, wy);
                geometry_msgs::Point point;
                point.x = wx;
                point.y = wy;
                output.points.push_back(point);

                // update the size of frontier
                output.size++;
                output.centroid.x += wx;
                output.centroid.y += wy;

                //define the distane of robot current position to the frontier
                double distance =
                    sqrt(pow((double(reference_x) - double(wx)), 2.0) + pow((double(reference_y) - double(wy)), 2.0));
                if (distance < output.min_distance) {
                    output.min_distance = distance;
                    output.middle.x = wx;
                    output.middle.y = wy;
                }

                bfs.push(nbr);
            }
        }
    }

    // add each frontier's co-ordinate and calculate the average
    output.centroid.x /= output.size;
    output.centroid.y /= output.size;
    return output;
}

bool FrontierSearch::isNewFrontierCell(unsigned int idx, const std::vector<bool>& frontier_flag) {
    
    if (map_[idx] != NO_INFORMATION || frontier_flag[idx]) {
        return false;
    }

    
    for (unsigned int nbr : nhood4(idx, *costmap_)) {
        if (map_[nbr] == FREE_SPACE) {
            return true;
        }
    }

    return false;
}

double FrontierSearch::frontierCost(const Frontier& frontier) {
    return (potential_scale_ * frontier.min_distance * costmap_->getResolution()) -
           (gain_scale_ * frontier.size * costmap_->getResolution());
}
} 
