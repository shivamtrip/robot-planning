import numpy as np
import pdb
import argparse
import subprocess # For executing c++ executable
import pandas as pd
from timeit import default_timer as timer

plannerList = ["RRT", "RRTCONNECT", "RRTSTAR", "PRM"]

###############################################################
################### Util Functions Below ######################

def convertPIs(aString):
    """ Input: A comma seperated string like "pi/2,pi/4,pi/2,pi/4,pi/2,"
            or 1.2,4,5.3 etc
    Output: string replacing the pis 1.57079,...,...
    """
    if aString[-1] == ",":  # Remove training comma if there is one
        aString = aString[:-1]
    aString = aString.replace("pi", "3.141592") # Replace pi with 3.14... if needed
    vecOfStrings = aString.split(",")
    ans = []
    for anExpression in vecOfStrings:
        ans.append(str(eval(anExpression))) # Evaluate expressions if needed
    return ans

###############################################################
################### Main Functions Below ######################


def graderMain(executablePath, gradingCSV):
    # problems = [["./map1.txt", "1.570796,0.785398,1.570796,0.785398,1.570796",
    #                             "0.392699,2.356194,3.141592,2.8274328,4.712388"],
    #         ["./map2.txt", "0.392699,2.356194,3.141592",
    #                             "1.570796,0.785398,1.570796"]]

    problems = [["./map2.txt", "0.642525,2.2704,0.250386,0.689152,2.36339",
                            "0.166726,1.7795,2.59392,0.573029,1.93726"],
            ["./map2.txt", "1.54947,0.890728,0.0495911,3.09367,2.05592",
                            "1.86571,1.79118,0.722037,2.81064,0.73908"],
            ["./map2.txt", "1.72784,2.15852,1.02216,0.630641,2.15858",
                            "0.938706,2.71964,1.99038,0.410013,1.92244"],
            ["./map2.txt", "1.83329,2.92921,0.252182,2.02108,0.787943",
                            "1.27687,1.21649,1.28773,2.29902,1.84713"],
            ["./map2.txt", "0.7884,1.4363,2.07477,1.91098,0.55907",
                            "1.09203,2.92405,0.445266,1.35113,2.50591"],
            ["./map2.txt", "0.00456442,2.03713,2.17039,2.08743,0.0271258",
                            "1.57131,3.02,0.957827,0.60652,2.52145"],
            ["./map2.txt", "0.0777222,0.784317,3.11493,2.12167,1.40738",
                            "1.84928,1.78755,2.74539,0.736825,1.14517"],
            ["./map2.txt", "0.329204,2.82979,0.923222,2.15598,1.23287",
                            "0.460689,3.13187,1.20725,3.01816,1.924"],
            ["./map2.txt", "1.78537,2.78438,1.26997,0.0236462,3.07326",
                            "0.588743,1.60255,3.05667,1.35758,1.32202"],
            ["./map2.txt", "1.74082,2.69833,1.59148,0.740573,1.32528",
                            "0.799128,2.68867,1.73008,2.46934,0.24233"],
            ["./map2.txt", "1.25575,2.01416,0.878571,2.84723,2.75473",
                            "0.659097,2.25461,1.90581,1.19803,2.39601"],
            ["./map2.txt", "1.59567,0.143467,2.51881,2.23579,2.77445",
                            "1.04523,1.60884,2.84331,3.02006,3.10236"],
            ["./map2.txt", "0.443,1.78767,1.12824,2.57732,1.63786",
                            "0.163751,0.0590298,1.08591,2.68256,2.29482"],
            ["./map2.txt", "1.63593,0.58863,2.36917,2.38681,2.96065",
                            "1.02962,0.185544,0.91449,2.11553,2.86811"],
            ["./map2.txt", "0.0677131,2.8343,0.684779,1.88644,0.121013",
                            "0.10389,2.56109,2.35459,0.0934422,1.31122"],
            ["./map2.txt", "1.01024,2.93773,2.34798,1.71321,2.83467",
                            "0.122004,1.8171,2.25416,2.4766,1.91054"],
            ["./map2.txt", "0.423792,2.37378,1.8126,1.00558,2.20566",
                            "1.61045,0.879717,2.8277,2.33372,0.231759"],
            ["./map2.txt", "1.37278,0.722203,2.71554,0.460973,2.66397",
                            "1.19737,1.75483,3.07459,0.558917,2.73479"],
            ["./map2.txt", "1.2228,2.28652,2.52995,0.796749,2.74749",
                            "0.95393,1.52108,2.70526,0.398279,0.474339"],
            ["./map2.txt", "1.23901,2.664,2.01615,1.67134,2.77119",
                            "1.17402,2.32026,2.67569,1.99297,1.25166"]]


    scores = []
    for aPlanner in [0, 1, 2, 3]:
        print("\nTESTING " + plannerList[aPlanner] + "\n")
        for i, data in enumerate(problems):
            inputMap, startPos, goalPos = [*data]
            numDOFs = len(startPos.split(","))
            outputSolutionFile = "grader_out/tmp.txt"
            commandPlan = "{} {} {} {} {} {} {}".format(
                executablePath,
                inputMap, numDOFs, startPos, goalPos,
                aPlanner, outputSolutionFile)
            print("EXECUTING: " + str(commandPlan))
            commandVerify = "./verifier.out {} {} {} {} {}".format(
                inputMap, numDOFs, startPos, goalPos,
                outputSolutionFile)
            print("EXECUTING: " + str(commandVerify))
            try:
                start = timer()
                subprocess.run(commandPlan.split(" "), check=True) # True if want to see failure errors
                timespent = timer() - start
                returncode = subprocess.run(commandVerify.split(" "), check=False).returncode
                if returncode != 0:
                    print("Returned an invalid solution")
                
                ### Calculate the cost from their solution
                with open(outputSolutionFile) as f:
                    line = f.readline().rstrip()  # filepath of the map
                    solution = []
                    for line in f:
                        solution.append(line.split(",")[:-1]) # :-1 to drop trailing comma
                    solution = np.asarray(solution).astype(float)
                    numSteps = solution.shape[0]

                    ## Cost is sum of all joint angle movements
                    difsPos = np.abs(solution[1:,]-solution[:-1,])
                    cost = np.minimum(difsPos, np.abs(2*np.pi - difsPos)).sum()

                    success = returncode == 0
                    scores.append([aPlanner, inputMap, i, numSteps, cost, timespent, success])
            
                ### Visualize their results
                # commandViz = "python visualizer.py grader_out/tmp.txt --gifFilepath=grader_out/grader_{}{}.gif".format(plannerList[aPlanner], i)
                # commandViz += " --incPrev=1"
                # subprocess.run(commandViz.split(" "), check=True) # True if want to see failure errors
            except Exception as exc:
                print("Failed: {} !!".format(exc))
                scores.append([aPlanner, inputMap, i, -1, -1, timespent, False])

    ### Save all the scores into a csv to compute overall grades
    df = pd.DataFrame(scores, columns=["planner", "mapName", "problemIndex", "numSteps", "cost", "timespent", "success"])
    df.to_csv(gradingCSV, index=False)
            

if __name__ == "__main__":
    graderMain("./planner.out", "grader_out/grader_results.csv")