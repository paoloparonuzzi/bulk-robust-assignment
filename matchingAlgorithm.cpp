
#include "Problem.h"

using namespace std;

bool Problem::buildFeasCutPoly(int i, std::vector<std::vector<double>>& zVal){

	double maxFlow = myGoldberg(i, zVal);
	if(maxFlow < nVertex - tol){
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
					expr += getZ(u, w);
			}
			if(!cut[u])
				rhs--;
			if(cut[u + nVertex])
				rhs--;
		}
		for(const auto e: scenarioUnsafeEdge[i]){
			if(cut[e.u] && !cut[e.w + nVertex])
				expr -= getZ(e.u, e.w);
		}
		expr -= rhs;
		return true;
		// cout << "Expr: " << expr << endl;
	}
	return false;
}