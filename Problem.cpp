//
// Created by Paolo on 26/04/2022.
//

#include "Problem.h"

#include <set>
#include <lemon/preflow.h>

using namespace std;

Problem::Problem(Instance& instance, int v_algorithm, int v_nMaxEdgeScenario) : lemon_capacity(lemon_graph), nodeIndex(lemon_graph) {
	nScenario = -1;
	algorithm = v_algorithm;
	nVertex = instance.i_nVertex;
	nEdgeScenario = v_nMaxEdgeScenario;
	callbackTime = 0;
	tol = 0.001;
	cost.resize(nVertex);
	for (int u = 0; u < nVertex; u++) {
		cost[u].resize(nVertex);
		for (int w = 0; w < nVertex; w++) {
			cost[u][w] = instance.i_cost[u][w];
		}
	}
	nAddedCut = 0;
	int countEdge = 0;
	cut_s.resize(2 * nVertex + 2, false);
	cut_t.resize(2 * nVertex + 2, false);

	for (int u = 0; u < 2 * nVertex + 2; u++) {
		lemon_nodes.push_back(lemon_graph.addNode());
		nodeIndex[lemon_nodes[u]] = u;
	}
	for (int u = 0; u < nVertex; u++) {
		for (int w = 0; w < nVertex; w++) {
			auto arc = lemon_graph.addArc(lemon_nodes[u], lemon_nodes[w + nVertex]);
			lemon_arcs.push_back(arc);
			lemon_capacity[arc] = 0;
			pairIndexMap[make_pair(u, w)] = countEdge;
			countEdge++;
		}
	}
	for (int u = 0; u < nVertex; u++) {
		auto arc = lemon_graph.addArc(lemon_nodes[2 * nVertex], lemon_nodes[u]);
		lemon_arcs.push_back(arc);
		lemon_capacity[arc] = 1;
		countEdge++;
	}
	for (int w = 0; w < nVertex; w++) {
		auto arc = lemon_graph.addArc(lemon_nodes[w + nVertex], lemon_nodes[2 * nVertex + 1]);
		lemon_arcs.push_back(arc);
		lemon_capacity[arc] = 1;
		countEdge++;
	}
	/////////////////////////////////////////////////
}

Edge::Edge(int uu, int ww){
    u = uu;
    w = ww;
}

double Problem::myGoldberg(int i, const std::vector<std::vector<double>>& zVal){

	int countEdge = 0;
	for(int u = 0; u < nVertex; u++){
		for(int w = 0; w < nVertex; w++){
			lemon_capacity[lemon_arcs[countEdge]] = zVal[u][w];
			countEdge++;
		}
	}
	// forbid scenario edges
	for(auto e : scenarioUnsafeEdge[i])
		lemon_capacity[lemon_arcs[pairIndexMap[make_pair(e.u, e.w)]]] = 0;

	Preflow<ListDigraph, ListDigraph::ArcMap<double>> preflow(lemon_graph, lemon_capacity, lemon_nodes[2 * nVertex], lemon_nodes[2 * nVertex + 1]);
	preflow.run();

	const int N = 2 * nVertex + 2;
	for (int k = 0; k < N; ++k)
		cut_s[k] = preflow.minCut(lemon_nodes[k]);

	ListDigraph::NodeMap<char> vis(lemon_graph, 0);
	std::queue<ListDigraph::Node> q;
	const auto t = lemon_nodes[2 * nVertex + 1];
	vis[t] = 1;
	q.push(t);
	while(!q.empty()){
		auto u = q.front(); q.pop();
		for (ListDigraph::InArcIt a(lemon_graph, u); a != INVALID; ++a) {
			auto v = lemon_graph.source(a);
			if (!vis[v] && (lemon_capacity[a] - preflow.flow(a)) > 1e-9) {
				vis[v] = 1;
				q.push(v);
			}
		}
		for (ListDigraph::OutArcIt a(lemon_graph, u); a != INVALID; ++a) {
			auto v = lemon_graph.target(a);
			if (!vis[v] && preflow.flow(a) > 1e-9) {
				vis[v] = 1;
				q.push(v);
			}
		}
	}
	for (int k = 0; k < N; ++k)
		cut_t[k] = !vis[lemon_nodes[k]];

	return preflow.flowValue();
}

void Problem::buildBulkInstance(){

    int nSol = assModel.get(GRB_IntAttr_SolCount);
    if(nSol > 0){
		double costSum = 0;
        for(int u = 0; u < nVertex; u++){
            for(int w = 0; w < nVertex; w++){
                if(getAssX(u, w).get(GRB_DoubleAttr_X) > 0.5){
                    vector<Edge> scenario;
                    scenario.emplace_back(u, w);
                    scenarioUnsafeEdge.push_back(scenario);
                }
	            costSum += cost[u][w];
            }
        }
        nScenario = static_cast<int>(scenarioUnsafeEdge.size());

	    for(auto&scenario : scenarioUnsafeEdge){
			set<pair<int, int>> alreadyInserted;
		    alreadyInserted.insert(make_pair(scenario[0].u, scenario[0].w));
			int n = 1;
			while(n < nEdgeScenario){
				int randU = rand()%nVertex;
				int randW = rand()%nVertex;
				if(!alreadyInserted.count(make_pair(randU, randW))){
					scenario.emplace_back(randU, randW);
					alreadyInserted.insert(make_pair(randU, randW));
					n++;
				}
			}
		}
	    for(int i = 0; i <nScenario; i++)
			shuffleScenario.push_back(i);
    }
    else{
        cout << "no solution to be used!" << endl;
        exit(2);
    }
}

GRBVar Problem::getComZ(int u, int w){
	return compactModel.getVarByName("z_" + to_string(u) + "_" + to_string(w));
}

GRBVar Problem::getComX(int i, int u, int w){
	return compactModel.getVarByName("x_" + to_string(u) + "_" + to_string(w) + "_" + to_string(i));
}

GRBVar Problem::getZ(int u, int w){
	return masterModel.getVarByName("z_" + to_string(u) + "_" + to_string(w));
}

GRBVar Problem::getZlp(int u, int w){
	return lpModel.getVarByName("z_" + to_string(u) + "_" + to_string(w));
}

GRBVar Problem::getAssX(int u, int w){
	return assModel.getVarByName("xAss_" + to_string(u) + "_" + to_string(w));
}

GRBVar Problem::getSubX(int i, int u, int w){
	return modelSub[i].getVarByName("xSub_" + to_string(i) + "_" + to_string(u) + "_" + to_string(w));
}

GRBVar Problem::getSubZ(int i, int u, int w){
	return modelSub[i].getVarByName("zSub_" + to_string(i) + "_" + to_string(u) + "_" + to_string(w));
}

void Problem::writeInstanceFile(const string& seed){
	ofstream instanceFile;
	instanceFile.open("bulkInstances/BulkMatch_" + to_string(nVertex) + "_" + to_string(nScenario) + "_" +
	                  to_string(nEdgeScenario) + "_" + seed + ".txt", ios_base::trunc);

	if(instanceFile.is_open()){
		instanceFile << nVertex << endl;
		for(int u = 0; u<nVertex; u++){
			for(int v = 0; v < nVertex; v++)
				instanceFile << cost[u][v] << "\t";
			instanceFile << endl;
		}
		instanceFile << nScenario << endl;
		for(const auto& scenario : scenarioUnsafeEdge){
			for(auto e: scenario)
				instanceFile << e.u << "\t" << e.w << "; ";
			instanceFile << endl;
		}
		instanceFile << endl;
	}
	instanceFile.close();
}