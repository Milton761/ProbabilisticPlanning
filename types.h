#ifndef TYPES_H
#define TYPES_H

#include <iostream>
#include <vector>
#include <map>
#include <list>
#include <algorithm>
#include <cmath>
#include <tuple>
#include <limits>
#include <iterator>
#include <string>
#include <ctime>
#include <iomanip>
#include <stack>
#include <chrono>
#include <fstream>
#include <cmath>

using namespace std::chrono;
using namespace std;


namespace MDPspace
{
    
     
    
    #define INF_P numeric_limits<int>::max();
    #define BELLMAN_ERROR 0.000006
    
    typedef string state;
    typedef string action;
    typedef int layer;
    typedef double cost;
    typedef double probabillity;
    typedef vector<state> states;
    typedef vector<action> actions;
    typedef map<state, actions> apFunction;
    
    typedef pair<state,action> state_action;
    
    typedef map< tuple<state,action,state>,probabillity > transFunction;
    typedef map< state_action, cost> costFunction;
    
    typedef map<state,action> policy;
    typedef map<state,cost> policyValue;
    
    typedef double belmanError;
    typedef map<state, belmanError> belmanResidual;
    
    enum _mdp_{_states_,_actions_,_apFunction_,_transFunction_,_costFunction_};
    typedef tuple<states,actions,apFunction,transFunction,costFunction> MDP;
    
    typedef tuple<MDP,state,state> SSP;
    
    typedef map<state,layer> state_layer;
    typedef map<layer,states> layer_states;
    
    typedef map<state,states> Graph;
    
    typedef vector<states> SCC;
    typedef states component;
    
    typedef chrono::microseconds time_measure;

}
    


#endif

