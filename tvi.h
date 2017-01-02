#ifndef TVI_H
#define TVI_H

#include "types.h"
#include "utilities.h"

namespace MDPspace
{
    
    tuple<policy,policyValue,time_measure> topologicalValueIteration(MDP &mdp, Graph &Gr, belmanError threshold)
    {
        actions A = get<_actions_>(mdp);
        transFunction T = get<_transFunction_>(mdp);
        apFunction Ap = get<_apFunction_>(mdp);
        costFunction C = get<_costFunction_>(mdp);
        states S = get<_states_>(mdp);
       
        high_resolution_clock::time_point t1 = high_resolution_clock::now();
        
        
        policy P;  
        policyValue V = initV(S);
        
        SCC scc = sccAlgorithm(Gr);
        
        //high_resolution_clock::time_point t1 = high_resolution_clock::now();
        
        
        
        for(int i=0;i<scc.size();i++)
        {
            states S1;
    
                for(int j=0; j<scc[i].size(); j++)
                {
                    state s = scc[i][j];
                    S1.push_back(s);
                }
               
                
                belmanResidual Belman_residual;
    
                bool flag = true;
                while(flag)
                {
                    belmanError Belman_error = 0.00000;
                    
                    for(int i=0; i<S1.size(); i++)
                    {
                        state s = S1[i];
                        cost oldV = V[s];
                        
                        tuple<cost,action> c_a = minCost(C,s,Ap,T,V,S);
                        V[s] = get<0>(c_a);
                        P[s] = get<1>(c_a);
                        
                        Belman_residual[s] = fabs(V[s]-oldV);
                        Belman_error = max(Belman_error,Belman_residual[s]);
                        
                        if(Belman_error < threshold)
                        {
                            flag = false;
                            break;
                        }
                    }
                    
                }
        }
        
        high_resolution_clock::time_point t2 = high_resolution_clock::now();
        
        time_measure D = duration_cast<milliseconds>( t2 - t1 );
    
        return forward_as_tuple(P,V,D);
        
    }
    
}

#endif