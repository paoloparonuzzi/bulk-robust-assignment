//
// Created by Paolo on 26/04/2022.
//

#ifndef BULKROBUSTMATCH_PROBLEM_H
#define BULKROBUSTMATCH_PROBLEM_H

#include <gurobi_c++.h>
#include <vector>
#include <fstream>
#include <map>
#include <bits/stdc++.h>
#include <random>
#include "Instance.h"

#include <lemon/list_graph.h>

using namespace lemon;

struct Edge{
    Edge(int uu, int ww);
    int u;
    int w;
};

struct Problem{

    // Problem data and constructor
	int algorithm;
    int nVertex;
    int nScenario;
	int nEdgeScenario;
	double tol;
	std::vector<std::vector<double>> cost;
    std::vector<std::vector<Edge>> scenarioUnsafeEdge;
	std::vector<int> shuffleScenario;
	std::default_random_engine rng;

    GRBEnv env;

    Problem(Instance& instance, int v_algorithm, int v_nMaxEdgeScenario);
    void buildBulkInstance();
	double myGoldberg(int i, const std::vector<std::vector<double>>& zVal);

    // functions to build and solve assignment modelSub
    GRBModel assModel = GRBModel(env);
	GRBVar getAssX(int u, int w);
    void buildAssModel(double timeLimit);
    void solveAssModel();
    void printAssModelSolution();
    int runAssModel(double timeLimit);

    // functions to build and solve descriptive modelSub
    GRBModel compactModel = GRBModel(env);
	GRBVar getComZ(int u, int w);
	GRBVar getComX(int i, int u, int w);
    void buildCompactModel(double timeLimit, bool lp);
    void solveCompactModel();
    void printCompactModelSolution();
    void runCompactModel(double timeLimit, bool lp);

    // functions to build and solve decomposition method
	int nAddedCut;
	double callbackTime;
    GRBModel masterModel = GRBModel(env);
	std::vector<GRBModel> modelSub;
	GRBVar getZ(int u, int w);
	GRBVar getSubX(int i, int u, int w);
	GRBVar getSubZ(int i, int u, int w);
	GRBLinExpr expr;
    void buildSubProblem(int i);
    bool buildFeasibilityCut(int i, std::vector<std::vector<double>>&zVal);
    void runBenders(double timeLimit);
	void printSubProblemSolution(int i);

	std::vector<bool> cut_s;
	std::vector<bool> cut_t;
	std::map<std::pair<int,int>, int> pairIndexMap;
	bool buildFeasCutPoly(int i, std::vector<std::vector<double>>&zVal);

	// functions to build and solve linear relaxations
	void linearRelaxation(double timeLimit);
	GRBModel lpModel = GRBModel(env);
	GRBVar getZlp(int u, int w);

    // output
    std::ofstream output;
    std::string outputLine;
	void writeInstanceFile(const std::string& seed);

	// LEMON
	ListDigraph lemon_graph;
	std::vector<ListDigraph::Node> lemon_nodes;
	std::vector<ListDigraph::Arc> lemon_arcs;
	ListDigraph::ArcMap<double> lemon_capacity;
	ListDigraph::NodeMap<int> nodeIndex;
};

#endif //BULKROBUSTMATCH_PROBLEM_H