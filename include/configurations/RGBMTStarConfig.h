//
// Created by nermin on 28.02.22.
//

class RGBMTStarConfig
{
public:
    static unsigned long MAX_NUM_ITER;          // Maximal number of algorithm iterations
    static unsigned long MAX_NUM_STATES;        // Maximal number of considered states
    static int MAX_PLANNING_TIME;               // Maximal algorithm runtime in [ms]
    static int TERMINATE_WHEN_PATH_IS_FOUND;    // Whether to terminate when path is found (default: false)

};