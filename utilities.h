#ifndef UTILITIES_H
#define UTILITIES_H

#include "types.h"

using namespace MDPspace;

using namespace std;

namespace MDPspace
{
    
    
    layer randomLayer(int min, int max)
    {
        return rand()%(max-min + 1) + min;
    }
    
    state randomState(states mdp_s)
    {
        int id = rand()%(mdp_s.size()) + 1;
        return mdp_s[id-1];
    }
    
    probabillity randomProbability()
    {
        
        return ((double) rand() / (RAND_MAX));
    }
    
    
    policyValue initV(states &S)
    {
        policyValue Vrandom;
        
        for(int i=0; i<S.size(); i++)
        {
            Vrandom[S[i]] = 0.0;
        }
        return Vrandom;
    }


    
    tuple<cost,action> minCost(costFunction &C, state &s, apFunction &Ap, transFunction &T, policyValue &V, states &S)
    {
        
        cost cost_result = INF_P;
        action action_result;
        
        vector< tuple<cost,action> > allCost_Action;
        
        for(int i=0; i<Ap[s].size(); i++)
        {
            action a = Ap[s][i];
            cost SUM = 0;
            
            for(int j=0; j<S.size(); j++)
            {
                state s1= S[j];
                SUM += T[make_tuple(s,a,s1)]*V[s1];
                //cout<<T[make_tuple(s,a,s1)]<<endl;
            }
            
            state_action s_a = make_pair(s,a);
            allCost_Action.push_back(make_tuple(C[s_a]+SUM,a));
        }
        
        if(allCost_Action.size()!=0)
        {
            
            for(int i=0; i<allCost_Action.size(); i++)
            {
                cost currentCost = get<0>(allCost_Action[i]);
                if(currentCost < cost_result)
                {
                    cost_result = currentCost;
                    action_result = get<1>(allCost_Action[i]);
                }
                
            }
    
        }else{
            cost_result = 0;
        }
        return forward_as_tuple(cost_result,action_result);
    }
    
    tuple<cost,action> minCostHeuristic(costFunction &C, state &s, apFunction &Ap, transFunction &T, policyValue &V, states &S)
    {
        
        cost cost_result = INF_P;
        action action_result;
        
        vector< tuple<cost,action> > allCost_Action;
        
        
        for(int i=0; i<Ap[s].size(); i++)
        {
            action a = Ap[s][i];
            state s1;
            
            probabillity hmin = INF_P;
            
            for(int j=0; j<S.size(); j++)
            {
                s1= S[j];
                probabillity currentP = T[make_tuple(s,a,s1)];
                
                if(currentP < hmin && currentP > 0)
                {
                    hmin = V[s1];
                }
            
            }
            
            
            state_action s_a = make_pair(s,a);
            allCost_Action.push_back(make_tuple(C[s_a] + hmin ,a));
        }
        
        
        
        if(allCost_Action.size()!=0)
        {
            
            for(int i=0; i<allCost_Action.size(); i++)
            {
                cost currentCost = get<0>(allCost_Action[i]);
                if(currentCost<cost_result)
                {
                    cost_result = currentCost;
                    action_result = get<1>(allCost_Action[i]);
                }
                
            }
    
        }else{
            cost_result = 0;
        }
        
        return forward_as_tuple(cost_result,action_result);
    }
    
    void reachabilityGraph(Graph &G, state &s, map<state,bool> &V, Graph &Gresult)
    {
        if(V[s])
        {
            return;
        }else{
            V[s] = true;
            
            for(int i=0; i<G[s].size(); i++)
            {
                state currentState = G[s][i];
                Gresult[s].push_back(currentState);
                reachabilityGraph(G,currentState,V, Gresult);
            }
        }
        
        return;
    }
    
    
    Graph reachabilityGraph(Graph &G, state &s)
    {
        map<state,bool> V;
        for(auto it=G.begin(); it!=G.end(); it++)
        {
            state currentState = it->first;
            V[currentState] = false;
        }
        
        Graph Gresult;
        
        reachabilityGraph(G, s, V, Gresult);
        
        return Gresult;
        
        
    }
    
   
    tuple<MDP,Graph> mdpGenerator(int s, int nl, int na,int ns)
    {
        
        //cout<<"mdpGenerator"<<endl;
        Graph Gr;
        MDP mdp;
        
        //1.creating states
        for(int i=0; i<s; i++)
        {
            string state_name = to_string(i+1);
            get<_states_>(mdp).push_back(state_name);
            //cout<<get<_states_>(mdp)[i]<<endl;
        }
        for(int i=0; i<na; i++)
        {
            action actionTag = to_string(i+1);
         //set the actions to de MDP
            get<_actions_>(mdp).push_back(actionTag);
        }
        
        //2.partition space in nl layers
        state_layer stateLayer;
        layer_states layerStates;
        
        if(nl!=1)
        {
            
        
        int states_per_layer = s/nl;
        
        //cout<<states_per_layer<<endl;
        
        layer currentLayer;
        int currentState;
        
        for(int i=0; i<nl-1; i++)
        {
            for(int j=0; j<states_per_layer; j++)
            {
                currentState = i*states_per_layer+j;
                currentLayer = i+1;
                state stateTag = get<_states_>(mdp)[currentState];
                
                stateLayer[stateTag] = currentLayer;
                layerStates[currentLayer].push_back(stateTag);
                //cout<<stateTag<<" "<<currentLayer<<endl;
                
                
            }
        }
        
        int stateSize = currentState+1;
        currentLayer++;
        
        for(int i= currentState+1; i< s; i++)
        {
            state stateTag = get<_states_>(mdp)[i];
            
            stateLayer[stateTag] = currentLayer;
            layerStates[currentLayer].push_back(stateTag);
            
            //cout<<get<_states_>(mdp)[i]<<" "<<currentLayer<<endl;
        }
        
        //3.set succesors
        //cout<<"Set Succesors"<<endl;
        
        for(int i=0; i<stateSize; i++)
        {
            //cout<<get<_states_>(mdp)[i]<<endl;
            /*
            if(i%1000==0)
                cout<<"state "<<get<_states_>(mdp)[i]<<endl;
            */
            state stateTag = get<_states_>(mdp)[i];
            
            //actions per state
            for(int j=0; j< na; j++)
            {
                cost costPerAction = 1.0;
                action actionTag = to_string(j+1);
                //fill apFunction
                get<_apFunction_>(mdp)[stateTag].push_back(actionTag);
                //Fill cost function
                get<_costFunction_>(mdp)[make_pair(stateTag,actionTag)] = costPerAction;
                
                //maximun succesor per action
                for(int k=0; k<ns; k++)
                {
                    layer layerRandom = randomLayer(stateLayer[stateTag]+1,nl);
                    
                    //cout<<"CurrentLayer - RandomLayer : "<<stateLayer[stateTag]<<" - "<<layerRandom<<endl;
                    state stateRandom = randomState(layerStates[layerRandom]);
                    //cout<<stateTag<<"-"<<actionTag<<"->"<<stateRandom<<" : "<<randomProbability()<<endl;
                    
                    //Fill transition function
                    get<_transFunction_>(mdp)[make_tuple(stateTag,actionTag,stateRandom)] = randomProbability();
                    
                    //Building the graph
                    Gr[stateTag].push_back(stateRandom);
                    
                }
            }
        }
        
        }
        //cout<<"END"<<endl;
        // goal state = last state = s
        return forward_as_tuple(mdp,Gr);
    }

    void printPolicyValue(policyValue V)
    {
        for(auto it=V.begin(); it!=V.end(); it++)
        {
            cout<<it->first<<" "<<it->second<<endl;
        }
    
    }


    void printPolicy(policy P)
    {
        for(auto it=P.begin(); it!=P.end(); it++)
        {
            state s = it->first;
            action a = P[s];
            
            cout<<s<<" "<<a<<endl;
            
        }
    }

    bool comparePolicyValue(tuple<policy,policyValue,time_measure> &V_1, tuple<policy,policyValue,time_measure> &V_2)
    {
        policy V1 = get<0>(V_1);
        policy V2 = get<0>(V_2);
        
        auto t1 = get<2>(V_1).count();
        auto t2 = get<2>(V_2).count();
        
        cout<<t1<<" "<<t2<<endl;
        
        for(auto it=V1.begin(); it!=V1.end(); it++)
        {
            //cout<<it->first<<" "<<it->second<<" * "<<V2[it->first]<<endl;
            //cout<<"Compare state "<<it->first<<endl;
            //cout<<it->second<<" "<<V2[it->first]<<endl;
            if(it->second != V2[it->first])    
            {
            //    cout<<it->first<<" NO "<<it->second<<" * "<<V2[it->first]<<endl;
                //return false;
            }
        
        }
        
        return true;
    
    }

    
    void scc_dfs(Graph &G, state &s, map<state,int> &id, int &cnt, int &cpntnum, states &postI, component &cmp)
    {
        id[s] = cpntnum;
        cmp.push_back(s);
        
        for(int i=0; i < G[s].size() ; i++)
        {
            state s1 = G[s][i];
            if(id[s1] == -1)
            {
                scc_dfs(G,s1, id, cnt, cpntnum, postI,cmp);
            }
        }
        postI[cnt] = s;
        cnt = cnt+1;
    }
    
    SCC sccAlgorithm(Graph &G)
    {
        //construing reverse graph Gr of G
        Graph Gr;
        for(auto it=G.begin(); it!=G.end(); it++)
        {
            state s = it->first;
            for(int i=0; i<G[s].size(); i++)
            {
                state succ = G[s][i];
                Gr[succ].push_back(s);
            }
        }   
        
        // size <- number of states in G
        int size = G.size();
        map<state,int> id;
        for(auto it=G.begin(); it!=G.end(); it++)
        {
            state s = it->first;
            id[s] = -1;
        }
        states postR(size+1);
        states postI(size+1);
        
        int cnt = 1;
        int cpntnum = 1;
        
        
        for(auto it=G.begin(); it!=G.end(); it++)
        {
            state s = it->first;
            if(id[s] == -1)
            {
                component cmp;
                scc_dfs(Gr,s,id,cnt,cpntnum,postI,cmp);
                
            }
        }
    
        for(int i=1; i<size+1; i++)
        {
            postR[i] = postI[i];
        }
        
        cnt = 1;
        cpntnum = 1;
        
        //reset id
        for(auto it=G.begin(); it!=G.end(); it++)
        {
            state s = it->first;
            id[s] = -1;
        }
        
        SCC scc;
        for(int i=0; i<postR.size(); i++) 
        {
            state s = postR[i];
            
            if(id[s] == -1)
            {
                component cmp;
                scc_dfs(G,s,id,cnt,cpntnum,postI,cmp);
                cpntnum = cpntnum + 1;
                scc.push_back(cmp);
            }
            
        }
    
        return scc;
    }


    SSP loadNet(string name_file)
    {
        
        Graph G;
        fstream file(name_file.c_str());
        
        string keyword;
        state s;
        action a;
        cost c;
        probabillity p;
        
        
        states S;
        actions A;
        apFunction Ap;
        transFunction T;
        costFunction C;
        
        
        //read 'state' keyword
        file>>keyword;
        
        while(true)
        {
            file>>s;
            S.push_back(s);
           // cout<<s<<endl;
            if(s=="endstates")
                break;
        }
        
        
        int acts=9;
        
        for(int i=0; i<acts ;i++)
        {
            //read 'action' keyword
            file>>keyword;
            file>>a;
            state s1,s2;
            A.push_back(a);
            //cout<<"   "<<a<<endl;
            while(true)
            {
                file>>s1;
                
                if(s1=="endaction")
                    break;
                
                
                file>>s2;
                file>>p;
                
                T[make_tuple(s1,a,s2)] = p;
                //cout<<s1<<" "<<s2<<" "<<p<<endl;
                
                
            }
        }
        
        //read 'cost' keyword
        file>>keyword;
        //cout<<keyword<<endl;
        
        while(true)
        {
            file>>s;
                
            if(s=="endcost")
                break;
                
            file>>a;
            file>>c;
            C[make_pair(s,a)] = c;
            //cout<<s<<" "<<a<<" "<<c<<endl;
        }
        
        
        for(int i=0; i<S.size(); i++)
        {
            
            for(int j=0; j<A.size(); j++)
            {
                 
                Ap[S[i]].push_back(A[j]);
            }
        }
        
        double diFactor;
        file>>keyword>>keyword>>diFactor;
        //cout<<diFactor<<endl;
        
        state s0;
        file>>keyword>>s0>>keyword;
        //cout<<s0<<endl;
        
        state sg;
        file>>keyword>>sg>>keyword;
        //cout<<sg<<endl;
        
        MDP mdp = forward_as_tuple(S,A,Ap,T,C);
        
        return forward_as_tuple(mdp,s0,sg);
    }
    
    
    Graph getGraph(MDP &mdp)
    {
        transFunction T = get<_transFunction_>(mdp);
        
        Graph G;
        
        for(auto it = T.begin(); it!= T.end(); it++)
        {
            auto s_a_s = it->first;
            
            state s1 = get<0>(s_a_s);
            state s2 = get<2>(s_a_s);
            
            G[s1].push_back(s2);
            
        }
        
        return G;
           
    }
    
   
    
    
}


#endif      