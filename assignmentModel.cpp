//
// Created by Paolo on 28/04/2022.
//


#include "Problem.h"
#include <cmath>

using namespace std;

void Problem::buildAssModel(double timeLimit){

    for(int u=0; u<nVertex; u++){
        for(int w = 0; w < nVertex; w++)
            assModel.addVar(0.0, GRB_INFINITY, cost[u][w], GRB_CONTINUOUS, "xAss_" + to_string(u) + "_" + to_string(w));
    }
	assModel.update();

    // build left constraints
    for(int u = 0; u < nVertex; u++){
        for(int w = 0; w < nVertex; w++)
            expr += getAssX(u, w);
        assModel.addConstr(expr == 1, "U_" + to_string(u));
        expr.clear();
    }

    // build right constraints
    for(int w = 0; w < nVertex; w++){
        for(int u = 0; u < nVertex; u++)
            expr += getAssX(u, w);
        assModel.addConstr(expr == 1, "W_" + to_string(w));
        expr.clear();
    }

	assModel.set(GRB_IntParam_OutputFlag, 0);
    assModel.set(GRB_DoubleParam_TimeLimit, timeLimit);
    assModel.set(GRB_IntParam_Threads, 1);
    assModel.update();
    // assModel.write("assModel.lp");
}

void Problem::solveAssModel(){
    try{
        assModel.optimize();
        int nSol = assModel.get(GRB_IntAttr_SolCount);
        if(nSol == 0){
            cout << "something wrong in gurobi assModel" << endl;
            compactModel.computeIIS();
            compactModel.write("infAssModel.ilp");
        }
    }
    catch(GRBException&e) {
        cout << "Error code = " << e.getErrorCode() << endl;
        cout << e.getMessage() << endl;
    } catch(...) {
        cout << "Exception during optimization" << endl;
    }
}

void Problem::printAssModelSolution(){
    try{
        cout << "Assignment: " << endl;
        for(int u = 0; u < nVertex; u++){
            for(int w = 0; w < nVertex; w++){
                if(getAssX(u, w).get(GRB_DoubleAttr_X) > 0.5)
                    cout << u << " -> " << w << " : " << cost[u][w] << endl;
            }
        }
        cout << endl;
    }
    catch(GRBException&e) {
        cout << "Error code = " << e.getErrorCode() << endl;
    }
}

int Problem::runAssModel(double timeLimit){

	try{
		buildAssModel(timeLimit);
		clock_t timeStart = clock();
		solveAssModel();
		double time = (double) (clock() - timeStart)/CLOCKS_PER_SEC;
		cout << "time assignment: " << time << endl;
		int status = assModel.get(GRB_IntAttr_Status);
		int nSol = assModel.get(GRB_IntAttr_SolCount);
		int optSol = -1;
		if(nSol > 0 && status == 2){
			optSol = (int) lround(assModel.get(GRB_DoubleAttr_ObjVal));
			// printAssModelSolution();
		}
		return optSol;
	}
	catch(GRBException&e) {
		cout << "Error code = " << e.getErrorCode() << endl;
	}
	return -1;
}