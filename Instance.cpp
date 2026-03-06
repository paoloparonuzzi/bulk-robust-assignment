//
// Created by Paolo on 26/04/2022.
//

#include "Instance.h"

#include <iostream>

using namespace std;

Instance::Instance(const std::string& fileName){

    ifstream file(fileName);
    if(!file.is_open()){
        cout << "wrong instance path. check it:" << fileName << endl;
        exit(-1);
    }
	i_nVertex = -1;
    file >> i_nVertex;

    i_cost.resize(i_nVertex);
    for(int u=0; u < i_nVertex; u++){
        i_cost[u].resize(i_nVertex);
        for(int w = 0; w<i_nVertex; w++)
            i_cost[u][w] = 0;
    }

    // read assignment costs
    for(int u=0; u < i_nVertex; u++){
        for(int w=0; w<i_nVertex; w++)
            file >> i_cost[u][w];
    }
}