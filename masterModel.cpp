//
// Created by Paolo on 04/05/2022.
//

#include "BendersCut.h"
#include "Problem.h"

using namespace std;

void Problem::runBenders(double timeLimit){

    if(algorithm == 2){
	    for(int i = 0; i < nScenario; i++){
		    modelSub.emplace_back(env);
		    buildSubProblem(i);
	    }
    }

    try{
        for(int u = 0; u < nVertex; u++)
            for(int w = 0; w < nVertex; w++)
	            masterModel.addVar(0.0, 1.0, cost[u][w], GRB_BINARY, "z_" + to_string(u) + "_" + to_string(w));

	    masterModel.update();

        // additional valid constraints
        for(int i=0; i<nScenario; i++){
            for(int u = 0; u < nVertex; u++){
                for(int w = 0; w < nVertex; w++)
                    expr += getZ(u,w);

                for(auto e : scenarioUnsafeEdge[i]){
                    if(e.u == u)
                        expr -= getZ(u,e.w);
                }
                masterModel.addConstr(expr >= 1, "U_Add_"+to_string(u));
                expr.clear();
            }
            for(int w = 0; w < nVertex; w++){
                for(int u = 0; u < nVertex; u++){
                    expr += getZ(u,w);
                }
                for(auto e : scenarioUnsafeEdge[i]){
                    if(e.w == w)
                        expr -= getZ(e.u,w);
                }
                masterModel.addConstr(expr >= 1, "W_Add_"+to_string(w));
                expr.clear();
            }
        }

	    masterModel.set(GRB_IntParam_OutputFlag, 0);
        masterModel.set(GRB_DoubleParam_TimeLimit, timeLimit);
        masterModel.set(GRB_IntParam_Threads, 1);
        masterModel.set(GRB_DoubleParam_Heuristics, 0);

	    // masterModel.set(GRB_IntParam_StrongCGCuts, 2);
	    // masterModel.set(GRB_IntParam_ZeroHalfCuts, 0);
	    // masterModel.set(GRB_IntParam_ModKCuts, 0);
	    // masterModel.set(GRB_IntParam_GomoryPasses, 0);

        masterModel.set(GRB_IntParam_LazyConstraints, 1);
	    BendersCut lazyCallback = BendersCut(this);
	    masterModel.setCallback(&lazyCallback);
	    masterModel.update();

        clock_t timeStart = clock();
        masterModel.optimize();
        double time = (double) (clock() - timeStart)/CLOCKS_PER_SEC;

        cout << "gurobiEffTime: " << time << endl;

        int status = masterModel.get(GRB_IntAttr_Status);
        bool optFound = false;
        if(status == 2)
            optFound = true;

        int nVar = masterModel.get(GRB_IntAttr_NumVars);
        int nCon = masterModel.get(GRB_IntAttr_NumConstrs);
        int nSol = masterModel.get(GRB_IntAttr_SolCount);
        int LB = 0.0;
        int UB = 0.0;
        double gap = 100.0;
        int nNode = 0;
        if(nSol > 0){
            nNode = (int) lround(masterModel.get(GRB_DoubleAttr_NodeCount));
            UB = (int) lround(masterModel.get(GRB_DoubleAttr_ObjVal));
            LB = (int) lround(masterModel.get(GRB_DoubleAttr_ObjBound));
            if((UB - LB)*100.0/UB >= 0)
                gap = (UB - LB)*100.0/UB;
        }
        cout << "nVar: " << nVar << endl;
        cout << "nCon: " << nCon << endl;
        cout << "nNode: " << nNode << endl;
        cout << "UB: " << UB << endl;
        cout << "LB: " << LB << endl;
        cout << "gap: " << gap << endl;
		cout << "Status: " << masterModel.get(GRB_IntAttr_Status) << endl;

	    /*if(nSol > 0){
		    for(int u = 0; u < nVertex; u++){
			    for(int w = 0; w < nVertex; w++){
				    if(getZ(u, w).get(GRB_DoubleAttr_X) > 0.001)
					    cout << getZ(u, w).get(GRB_DoubleAttr_X) << " (" << u << ", " << w << ") : " << cost[u][w]
					         << endl;
			    }
		    }
		    cout << endl;
	    }*/
        outputLine += to_string(nVar) + "\t" + to_string(nCon) + "\t" + to_string(optFound) + "\t" + to_string(time)
                      + "\t" + to_string(UB) + "\t" + to_string(LB) + "\t" + to_string(gap) + "\t" + to_string(nNode)
						+ "\t" + to_string(nAddedCut) + "\t" + to_string(callbackTime);

        // masterModel.write("masterModel.lp");
    }
    catch(GRBException&e) {
        cout << "Master Error code: " << e.getErrorCode() << endl;
        cout << "Master Error Message: "<< e.getMessage() << endl;
    } catch(...) {
        cout << "Exception during optimization" << endl;
    }

}
