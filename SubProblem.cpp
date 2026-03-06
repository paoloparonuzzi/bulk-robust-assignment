//
// Created by Paolo on 05/05/2022.
//

#include "Problem.h"

using namespace std;

void Problem::buildSubProblem(int i){

    for(int u=0; u<nVertex; u++){
        for(int w = 0; w < nVertex; w++){
	        modelSub[i].addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS,
	                           "xSub_" + to_string(i) + "_" + to_string(u) + "_" + to_string(w));
	        modelSub[i].addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS,
	                           "zSub_" + to_string(i) + "_" + to_string(u) + "_" + to_string(w));
        }
    }
	// modelSub[i].set(GRB_IntAttr_ModelSense, GRB_MAXIMIZE);
	modelSub[i].update();

    // build left constraints
    for(int u = 0; u < nVertex; u++){
        for(int w = 0; w < nVertex; w++)
            expr += getSubX(i, u, w);
        modelSub[i].addConstr(expr <= 1, "U_" + to_string(u));
        expr.clear();
    }

    // build right constraints
    for(int w = 0; w < nVertex; w++){
        for(int u = 0; u < nVertex; u++)
            expr += getSubX(i, u, w);
        modelSub[i].addConstr(expr <= 1, "W_" + to_string(w));
        expr.clear();
    }

    // impose a matching of size equal to N
    for(int w = 0; w < nVertex; w++)
        for(int u = 0; u < nVertex; u++)
            expr += getSubX(i, u, w);
    modelSub[i].addConstr(expr >= nVertex, "Match");
    expr.clear();

    // build activation constraints
    for(int u = 0; u < nVertex; u++)
        for(int w = 0; w < nVertex; w++)
            modelSub[i].addConstr(getSubX(i, u, w) <= getSubZ(i, u, w), "Act_" + to_string(u) + "_" + to_string(w));

    // forbid scenario edges
    for(auto e : scenarioUnsafeEdge[i])
	    getSubX(i, e.u, e.w).set(GRB_DoubleAttr_UB, 0.0);

	modelSub[i].set(GRB_IntParam_OutputFlag, 0);
    modelSub[i].set(GRB_DoubleParam_TimeLimit, 600);
    modelSub[i].set(GRB_IntParam_Threads, 1);
    modelSub[i].set(GRB_IntParam_InfUnbdInfo, 1);
    modelSub[i].update();
    // modelSub[i].write("matchSubProblem"+to_string(i)+".lp");
}

bool Problem::buildFeasibilityCut(int i, vector<vector<double>>&zVal){

	for(int u = 0; u < nVertex; u++){
		for(int w = 0; w < nVertex; w++){
			getSubZ(i, u, w).set(GRB_DoubleAttr_LB, zVal[u][w]);
			getSubZ(i, u, w).set(GRB_DoubleAttr_UB, zVal[u][w]);
		}
	}

	modelSub[i].optimize();
	int nSol = modelSub[i].get(GRB_IntAttr_SolCount);
	if(nSol == 0){
		for(int u = 0; u < nVertex; u++){
			for(int w = 0; w < nVertex; w++){
				GRBConstr c = modelSub[i].getConstrByName("Act_" + to_string(u) + "_" + to_string(w));
				double dualRay = c.get(GRB_DoubleAttr_FarkasDual);
				if(dualRay != 0)
					expr += dualRay*getZ(u, w);
			}
		}
		double rhs = 0.0;
		int nCon = modelSub[i].get(GRB_IntAttr_NumConstrs);
		for(int c = 0; c < nCon; c++){
			GRBConstr con = modelSub[i].getConstr(c);
			double dualRay = con.get(GRB_DoubleAttr_FarkasDual);
			if(dualRay != 0)
				rhs += con.get(GRB_DoubleAttr_RHS)*dualRay;
		}
		expr += rhs;
		return true;
	}
	return false;
}

void Problem::printSubProblemSolution(int i){
	try{
		cout << "Assignment in Scenario " << i << endl;
		for(int u = 0; u < nVertex; u++){
			for(int w = 0; w < nVertex; w++){
				if(getSubX(i, u, w).get(GRB_DoubleAttr_X) > 0.1)
					cout << u << " -> " << w << " : " << cost[u][w] << endl;
			}
		}
		cout << endl;
	}
	catch(GRBException e) {
		cout << "Print solution - Error code: " << e.getErrorCode() << endl;
		cout << "Print solution - Error Message: " << e.getMessage() << endl;
	}
}
