//
// Created by Paolo on 25/11/2020.
//


#include "BendersCut.h"

BendersCut::BendersCut(Problem*problemRef){

	problem = problemRef;
	zVal.resize(problem->nVertex);
	for(int u = 0; u < problem->nVertex; u++){
		zVal[u].resize(problem->nVertex);
		for(int w = 0; w < problem->nVertex; w++)
			zVal[u][w] = 0.0;
	}
}

void BendersCut::callback(){

	clock_t timeStartCallback = clock();

	try{

		if(where == GRB_CB_MIPSOL){

			for(int u = 0; u < problem->nVertex; u++)
				for(int w = 0; w < problem->nVertex; w++)
					zVal[u][w] = abs(getSolution(problem->getZ(u, w)));

			// try to solve assignment problem for each scenario
			shuffle(problem->shuffleScenario.begin(), problem->shuffleScenario.end(), problem->rng);
			for(int j = 0; j < problem->nScenario; j++){
				const int i = problem->shuffleScenario[j];
				bool cutFound;
				// using Gurobi solving the LP
				if(problem->algorithm == 2)
					cutFound = problem->buildFeasibilityCut(i, zVal);
				// using Lemon solving max flow
				else
					cutFound = problem->buildFeasCutPoly(i, zVal);
				if(cutFound){
					addLazy(problem->expr >= 0);
					problem->expr.clear();
					problem->nAddedCut++;
					problem->callbackTime += (double) (clock() - timeStartCallback)/CLOCKS_PER_SEC;
					return;
				}
			}
		}
		if(where == GRB_CB_MIPNODE && getIntInfo(GRB_CB_MIPNODE_STATUS) == GRB_OPTIMAL){

			if(getDoubleInfo(GRB_CB_MIPNODE_NODCNT) >= 1)
				return;

			for(int u = 0; u < problem->nVertex; u++)
				for(int w = 0; w < problem->nVertex; w++)
					zVal[u][w] = abs(getNodeRel(problem->getZ(u, w)));

			// try to solve assignment problem for each scenario
			shuffle(problem->shuffleScenario.begin(), problem->shuffleScenario.end(), problem->rng);
			for(int j = 0; j < problem->nScenario; j++){
				const int i = problem->shuffleScenario[j];
				bool cutFound;
				if(problem->algorithm == 2)
					// using Gurobi solving the LP
					cutFound = problem->buildFeasibilityCut(i, zVal);
				else
					// using Lemon solving max flow
					cutFound = problem->buildFeasCutPoly(i, zVal);
				if(cutFound){
					addCut(problem->expr >= 0);
					// cout << "expr: " << problem->expr << endl;
					problem->expr.clear();
					problem->nAddedCut++;
					problem->callbackTime += (double) (clock() - timeStartCallback)/CLOCKS_PER_SEC;
					return;
				}
			}
		}
	}
	catch(GRBException& e){
		cout << "Error callback number : " << e.getErrorCode() << endl;
		cout << "Error callback: " << e.getMessage() << endl;
	}catch(...){
		cout << " Error during callback " << endl;
	}

	problem->callbackTime += (double) (clock() - timeStartCallback)/CLOCKS_PER_SEC;

}