//
// Created by Paolo on 26/04/2022.
//

#ifndef BULKROBUSTMATCH_INSTANCE_H
#define BULKROBUSTMATCH_INSTANCE_H

#include <fstream>
#include <vector>

struct Instance{

    // Problem data and constructor
    int i_nVertex;
    std::vector<std::vector<double>> i_cost;
    explicit Instance(const std::string& fileName);

};


#endif //BULKROBUSTMATCH_INSTANCE_H
