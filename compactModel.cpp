//
// Created by Paolo on 27/04/2022.
//

#include "Problem.h"

using namespace std;

void Problem::buildCompactModel(double timeLimit, bool lp){


    for(int i = 0; i<nScenario; i++)
        for(int u = 0; u < nVertex; u++)
            for(int w = 0; w < nVertex; w++)
                compactModel.addVar(0.0, 1.0, 0.0, GRB_CONTINUOUS, "x_" + to_string(u) + "_" + to_string(w) + "_" + to_string(i));

    for(int u=0; u<nVertex; u++)
        for(int w = 0; w < nVertex; w++){
	        if(lp)
		        compactModel.addVar(0.0, 1.0, cost[u][w], GRB_CONTINUOUS, "z_" + to_string(u) + "_" + to_string(w));
	        else
		        compactModel.addVar(0.0, 1.0, cost[u][w], GRB_INTEGER, "z_" + to_string(u) + "_" + to_string(w));
        }

	compactModel.update();

    // build left constraints
    for(int u = 0; u < nVertex; u++){
        for(int i = 0; i < nScenario; i++){
            for(int w = 0; w < nVertex; w++)
                expr += getComX(i,u,w);
            compactModel.addConstr(expr == 1, "U_" + to_string(u) + "_" + to_string(i));
            expr.clear();
        }
    }

    // build right constraints
    for(int w = 0; w < nVertex; w++){
        for(int i = 0; i < nScenario; i++){
            for(int u = 0; u < nVertex; u++)
                expr += getComX(i,u,w);
            compactModel.addConstr(expr == 1, "W_" + to_string(w) + "_" + to_string(i));
            expr.clear();
        }
    }

    // build activation constraints
    for(int i=0; i<nScenario; i++)
        for(int u = 0; u < nVertex; u++)
            for(int w = 0; w < nVertex; w++)
                compactModel.addConstr(getComX(i,u,w) <= getComZ(u, w), "Act_" + to_string(u) + "_" + to_string(w) + "_" + to_string(i));

    // forbid scenario edges
    for(int i=0; i<nScenario; i++){
        for(auto e : scenarioUnsafeEdge[i])
	        getComX(i,e.u,e.w).set(GRB_DoubleAttr_UB, 0.0);
    }

	// additional valid constraints
	for(int i=0; i<nScenario; i++){
		for(int u = 0; u < nVertex; u++){
			for(int w = 0; w < nVertex; w++)
				expr += getComZ(u, w);

			for(auto e : scenarioUnsafeEdge[i])
				if(e.u == u)
					expr -= getComZ(u, e.w);

			compactModel.addConstr(expr >= 1, "U_Add_"+to_string(u));
			expr.clear();
		}
		for(int w = 0; w < nVertex; w++){
			for(int u = 0; u < nVertex; u++)
				expr += getComZ(u, w);

			for(auto e : scenarioUnsafeEdge[i])
				if(e.w == w)
					expr -= getComZ(e.u, w);

			compactModel.addConstr(expr >= 1, "W_Add_"+to_string(w));
			expr.clear();
		}
	}

	compactModel.set(GRB_IntParam_OutputFlag, 0);
    compactModel.set(GRB_DoubleParam_TimeLimit, timeLimit);
    compactModel.set(GRB_IntParam_Threads, 1);
    compactModel.update();
    // compactModel.write("compactModel.lp");
}

void Problem::solveCompactModel(){
    try{
        compactModel.optimize();
        int nSol = compactModel.get(GRB_IntAttr_SolCount);
        if(nSol == 0){
            cout << "No solution found by Gurobi" << endl;
            // compactModel.computeIIS();
	        // compactModel.write("infcompactModel.ilp");
        }
		else
			printCompactModelSolution();
    }
    catch(GRBException&e) {
        cout << "Error code = " << e.getErrorCode() << endl;
        cout << e.getMessage() << endl;
    } catch(...) {
        cout << "Exception during optimization" << endl;
    }
}

void Problem::printCompactModelSolution(){
    try{
        cout << "Edge activated: " << endl;
        for(int u = 0; u < nVertex; u++){
            for(int w = 0; w < nVertex; w++){
                if(getComZ(u,w).get(GRB_DoubleAttr_X) > 0.001)
                    cout << getComZ(u,w).get(GRB_DoubleAttr_X) <<  " (" << u << ", " << w << "); ";
            }
            cout << endl;
        }
        cout << endl;
    }
    catch(GRBException&e) {
        cout << "Error code = " << e.getErrorCode() << endl;
    }
}

void Problem::runCompactModel(double timeLimit, bool lp){

    clock_t timeStart = clock();
    buildCompactModel(timeLimit, lp);
    double buildingTime = (double) (clock() - timeStart)/CLOCKS_PER_SEC;
    cout << "Model has been built in [s] " << buildingTime << endl;

    timeStart = clock();
    solveCompactModel();
    double time = (double) (clock() - timeStart)/CLOCKS_PER_SEC;
    cout << "gurobiEffTime: " << time << endl;

    int status = compactModel.get(GRB_IntAttr_Status);
    int optFound = 0;
    if(status == 2)
        optFound = 1;

    int nVar = compactModel.get(GRB_IntAttr_NumVars);
    int nCon = compactModel.get(GRB_IntAttr_NumConstrs);
    int nSol = compactModel.get(GRB_IntAttr_SolCount);
    int LB = 0.0;
    double UB = 0.0;
    double gap = 100.0;
    int nNode = 0;
    if(nSol > 0){
        try{
			if(lp)
				UB = compactModel.get(GRB_DoubleAttr_ObjVal);
			else{
				nNode = (int) lround(compactModel.get(GRB_DoubleAttr_NodeCount));
				UB = (int) lround(compactModel.get(GRB_DoubleAttr_ObjVal));
				LB = (int) lround(compactModel.get(GRB_DoubleAttr_ObjBound));
				if((UB - LB)*100.0/UB >= 0)
					gap = (UB - LB)*100.0/UB;
			}
        }
        catch(GRBException&e) {
            cout << "Error code = " << e.getErrorCode() << endl;
        }
        // printCompactModelSolution();
    }
    cout << "nVar: " << nVar << endl;
    cout << "nCon: " << nCon << endl;
    cout << "nNode: " << nNode << endl;
    cout << "UB: " << UB << endl;
    cout << "LB: " << LB << endl;
    cout << "gap: " << gap << endl;

	if(lp){
		outputLine += to_string(nVar) + "\t" + to_string(nCon) + "\t" + to_string(optFound) + "\t" + to_string(time)
		              + "\t-\t" + to_string(UB) + "\t-\t-";
	}
	else{
		outputLine += to_string(nVar) + "\t" + to_string(nCon) + "\t" + to_string(optFound) + "\t" + to_string(time)
		              + "\t" + to_string(UB) + "\t" + to_string(LB) + "\t" + to_string(gap) + "\t" + to_string(nNode);
	}
}