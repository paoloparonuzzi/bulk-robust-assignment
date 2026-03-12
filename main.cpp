#include <iostream>

#include "Instance.h"
#include "Problem.h"

using namespace std;

int main(int argc, char**argv){

    string fileName;
    double timeLimit;
    int algorithm;
	int lp;
	int nEdgeScenario;
    char*fileOutput;

    if(argc == 7){
        fileName = argv[1];
        timeLimit = atof(argv[2]);
        algorithm = atoi(argv[3]);
	    lp = atoi(argv[4]);
	    nEdgeScenario = atoi(argv[5]);
	    fileOutput = argv[6];
    }
    else{
        cout << "Wrong number of parameter" << endl;
        return 1;
    }

	if(nEdgeScenario <= 0){
		cout << "At least one edge per scenario" << endl;
		return 1;
	}

    Instance instance(fileName);
    Problem problem(instance, algorithm, nEdgeScenario);

	if(nEdgeScenario > problem.nVertex){
		cout << "Maybe, too many edge per scenario" << endl;
		return 1;
	}
	if(algorithm == 0 && problem.nVertex > 120){
		cout << "Jump: too many vertex for Compact Model" << endl;
		return 1;
	}

	int assSol = problem.runAssModel(timeLimit);
	problem.buildBulkInstance();

	std::size_t found = fileName.find_last_of('_');
	std::size_t found2 = fileName.find_last_of('.');
	string seed = fileName.substr(found+1, found2 - (found+1));

	// problem.writeInstanceFile(seed);
	// exit(1);
	double sProp = (double) nEdgeScenario/problem.nVertex;
	problem.outputLine = fileName + "\t" + to_string(problem.nVertex) + "\t" + to_string(problem.nScenario) + "\t"
			+ to_string(nEdgeScenario) + "\t" + to_string(sProp) + "\t" + to_string(assSol) + "\t";
    if(algorithm == 0){
        problem.outputLine += "COMP\t" + to_string(lp)+"\t";
        problem.runCompactModel(timeLimit, lp);
    }
    if(algorithm >= 2){
		if(algorithm == 2)
			problem.outputLine += "B-LP\t" + to_string(lp)+"\t";
	    if(algorithm == 3)
		    problem.outputLine += "B-flow*\t" + to_string(lp)+"\t";
    	if(algorithm == 4)
    		problem.outputLine += "B-flow\t" + to_string(lp)+"\t";
	    if(lp)
		    problem.linearRelaxation(timeLimit);
	    else
		    problem.runBenders(timeLimit);
    }

    problem.output.open(fileOutput, ios_base::app);
    if(problem.output.is_open()){
        problem.output << problem.outputLine << endl;
        problem.output.close();
    }
    return 0;
}
