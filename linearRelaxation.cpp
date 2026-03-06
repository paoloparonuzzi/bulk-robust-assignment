//
// Created by Paolo on 26/01/2024.
//

#include "Problem.h"

using namespace std;

void Problem::linearRelaxation(double timeLimit){

	try{

		if(algorithm == 2){
			for(int i = 0; i < nScenario; i++){
				modelSub.emplace_back(env);
				buildSubProblem(i);
			}
		}

		for(int u = 0; u < nVertex; u++)
			for(int w = 0; w < nVertex; w++)
			lpModel.addVar(0.0, 1.0, cost[u][w], GRB_CONTINUOUS, "z_" + to_string(u) + "_" + to_string(w));
		lpModel.update();

		// additional valid constraints
		for(int i=0; i<nScenario; i++){
			for(int u = 0; u < nVertex; u++){
				for(int w = 0; w < nVertex; w++)
					expr += getZlp(u,w);

				for(auto e : scenarioUnsafeEdge[i]){
					if(e.u == u)
						expr -= getZlp(u,e.w);
				}
				lpModel.addConstr(expr >= 1, "U_Add_"+to_string(u));
				expr.clear();
			}
			for(int w = 0; w < nVertex; w++){
				for(int u = 0; u < nVertex; u++){
					expr += getZlp(u,w);
				}
				for(auto e : scenarioUnsafeEdge[i]){
					if(e.w == w)
						expr -= getZlp(e.u,w);
				}
				lpModel.addConstr(expr >= 1, "W_Add_"+to_string(w));
				expr.clear();
			}
		}

		lpModel.set(GRB_IntParam_OutputFlag, 0);
		lpModel.set(GRB_DoubleParam_TimeLimit, timeLimit);
		lpModel.set(GRB_IntParam_Threads, 1);
		// lpModel.write("master.lp");

		clock_t timeStart = clock();
		double checkTime = (double) (clock() - timeStart)/CLOCKS_PER_SEC;
		bool cutAdded = true;
		vector<vector<double>> zValLp(nVertex, vector<double>(nVertex));
		while(cutAdded && checkTime < timeLimit){
			cutAdded = false;
			lpModel.optimize();
			clock_t timeStartCut = clock();
			int nSol = lpModel.get(GRB_IntAttr_SolCount);
			if(nSol > 0){
				for(int u = 0; u < nVertex; u++)
					for(int w = 0; w < nVertex; w++){
						zValLp[u][w] = getZlp(u, w).get(GRB_DoubleAttr_X);
					}

				shuffle(shuffleScenario.begin(), shuffleScenario.end(), rng);
				if(algorithm == 2){
					for(int j = 0; j < nScenario; j++){
						int i = shuffleScenario[j];

						for(int u = 0; u < nVertex; u++){
							for(int w = 0; w < nVertex; w++){
								getSubZ(i, u, w).set(GRB_DoubleAttr_LB, zValLp[u][w]);
								getSubZ(i, u, w).set(GRB_DoubleAttr_UB, zValLp[u][w]);
							}
						}

						modelSub[i].optimize();
						int nSubSol = modelSub[i].get(GRB_IntAttr_SolCount);
						if(nSubSol == 0){
							double maxFlow = modelSub[i].get(GRB_DoubleAttr_ObjVal);
							if(maxFlow < nVertex - tol){
								cutAdded = true;
								for(int u = 0; u < nVertex; u++){
									for(int w = 0; w < nVertex; w++){
										GRBConstr c = modelSub[i].getConstrByName(
												"Act_" + to_string(u) + "_" + to_string(w));
										double dualRay = c.get(GRB_DoubleAttr_FarkasDual);
										if(dualRay != 0)
											expr += dualRay*getZlp(u, w);
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

								lpModel.addConstr(expr >= -rhs, "cut_" + to_string(nAddedCut));
								expr.clear();
								nAddedCut++;
							}
						}
						if(cutAdded)
							break;
					}
				}

				if(algorithm >= 3){
					for(int j = 0; j < nScenario; j++){

						int i = shuffleScenario[j];
						double maxFlow = myGoldberg(i, zValLp);
						cutAdded = (maxFlow < nVertex - tol);
						if(cutAdded){
							char bestC = 's';
							int nVar_cut_s = 0;
							int nVar_cut_t = 0;
							for(int u = 0; u < nVertex; u++){
								for(int w = 0; w < nVertex; w++) {
									if(cut_s[u] && !cut_s[w + nVertex])
										nVar_cut_s++;
									if(cut_t[u] && !cut_t[w + nVertex])
										nVar_cut_t++;
								}
							}
							for(const auto e: scenarioUnsafeEdge[i]) {
								if(cut_s[e.u] && !cut_s[e.w + nVertex])
									nVar_cut_s--;
								if(cut_t[e.u] && !cut_t[e.w + nVertex])
									nVar_cut_t--;
							}
							if(algorithm == 3 && nVar_cut_t < nVar_cut_s)
								bestC = 't';
							auto& cut = (bestC == 's') ? cut_s : cut_t;
							double rhs = nVertex;
							for(int u = 0; u < nVertex; u++){
								for(int w = 0; w < nVertex; w++){
									if(cut[u] && !cut[w + nVertex])
										expr += getZlp(u, w);
								}
								if(!cut[u])
									rhs--;
								if(cut[u + nVertex])
									rhs--;
							}
							for(const auto e: scenarioUnsafeEdge[i]){
								if(cut[e.u] && !cut[e.w + nVertex])
									expr -= getZlp(e.u, e.w);
							}
							expr -= rhs;

							lpModel.addConstr(expr >= 0, "cut_" + to_string(nAddedCut));
							// cout << "expr: " << expr << " >= " << rhs << endl << endl;
							expr.clear();
							nAddedCut++;
						}
						if(cutAdded)
							break;
					}
				}
			}
			callbackTime += (double) (clock() - timeStartCut)/CLOCKS_PER_SEC;
			checkTime = (double) (clock() - timeStart)/CLOCKS_PER_SEC;
		}
		double time = (double) (clock() - timeStart)/CLOCKS_PER_SEC;

		cout << "gurobiEffTime: " << time << endl;

		int optFound = 0;
		if(cutAdded == false)
			optFound = 1;

		int nVar = lpModel.get(GRB_IntAttr_NumVars);
		int nCon = lpModel.get(GRB_IntAttr_NumConstrs);
		int nSol = lpModel.get(GRB_IntAttr_SolCount);
		double objVal = 0.0;
		if(nSol > 0 && !cutAdded){
			objVal = lpModel.get(GRB_DoubleAttr_ObjVal);
			cout << "Edge activated: " << endl;
			for(int u = 0; u < nVertex; u++)
				for(int w = 0; w < nVertex; w++)
					if(getZlp(u, w).get(GRB_DoubleAttr_X) > 0.001)
						cout << getZlp(u, w).get(GRB_DoubleAttr_X) << " (" << u << ", " << w << "); ";

			cout << endl;
		}
		cout << "nVar: " << nVar << endl;
		cout << "nCon: " << nCon << endl;
		cout << "objVal: " << objVal << endl;
		outputLine += to_string(nVar) + "\t" + to_string(nCon) + "\t" + to_string(optFound) + "\t" + to_string(time)
		              + "\t-\t" + to_string(objVal) + "\t-\t-\t" + to_string(nAddedCut) + "\t" +
		              to_string(callbackTime);

		// lpModel.write("lpModel.lp");
	}
	catch(GRBException& e){
		cout << "Error code = " << e.getErrorCode() << endl;
		cout << e.getMessage() << endl;
	}catch(...){
		cout << "Exception during optimization" << endl;
	}

}