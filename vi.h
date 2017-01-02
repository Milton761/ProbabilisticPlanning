#ifndef VI_H
#define VI_H

#include "types.h"
#include "utilities.h"

namespace MDPspace
{
    

    tuple<policy,policyValue,time_measure> valueIterarion(states &S, actions &A,apFunction &Ap, transFunction &T,costFunction &C, belmanError threshold)
    {
        
        high_resolution_clock::time_point t1 = high_resolution_clock::now();
        
        policy P;   
        policyValue V = initV(S);
        
        belmanResidual Belman_residual;
        while(true)
        {
            belmanError Belman_error = 0.00000;
            //cout<<"Size "<<S.size()<<endl;
            for(int i=0; i<S.size(); i++)
            {
                state s = S[i];
                cost oldV = V[s];
                
                tuple<cost,action> c_a = minCost(C,s,Ap,T,V,S);
                V[s] = get<0>(c_a);
                P[s] = get<1>(c_a);
                
                Belman_residual[s] = fabs(V[s]-oldV);
                Belman_error = max(Belman_error,Belman_residual[s]);
                //cout<<fixed<<setprecision(6)<<Belman_error<<endl;
                if(Belman_error < threshold)
                {
                    
                        
                        high_resolution_clock::time_point t2 = high_resolution_clock::now();
                        time_measure D = duration_cast<milliseconds>( t2 - t1 );
                        
                        
                        return forward_as_tuple(P,V,D);
                         
                }
            
            }
            
        }
        
       
    
      
        
    }
    
    tuple<policy,policyValue,time_measure> valueIterarion(MDP &mdp, belmanError threshold)
    {
        //cout<<"VI-s1"<<endl;
        states S = get<_states_>(mdp);
        actions A = get<_actions_>(mdp);
        transFunction T = get<_transFunction_>(mdp);
        apFunction Ap = get<_apFunction_>(mdp);
        costFunction C = get<_costFunction_>(mdp);
        //cout<<"VI-s2"<<endl;
        return valueIterarion(S,A,Ap,T,C,threshold);
    }

}


#endif