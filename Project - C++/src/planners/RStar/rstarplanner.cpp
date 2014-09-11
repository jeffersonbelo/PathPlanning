/*
 * Copyright (c) 2008, Maxim Likhachev
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Pennsylvania nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include <iostream>
using namespace std;

#include "../../sbpl/headers.h"


//TODO - define RSTAR_DEBUG_PRINTLOWLEVELEXP and RSTAR_DEBUG_PRINTHIGHLEVELEXP. Make them dependent on DEBUG
//use them to print expands into two separate file.

//-----------------------------------------------------------------------------------------------------

// Construtor que inicializa algumas variaveis de controle e faz copia o conteudo das variaveis passada por parametro para manipul�-las durante a execu��o do programa.
RSTARPlanner::RSTARPlanner(DiscreteSpaceInformation* environment, bool bSearchForward){ // CONSTRUTOR Chamado pelo MAIN

	bforwardsearch = bSearchForward;	                      // Variavel passada por parametro pelo o m�todo MAIN. Se TRUE a pesquisa ser� FORWARD, se falso BACKWARD. Esse valor � guardado na variavel "bforwardsearch"
										                      // Para poder se manipulado livremente
 
	environment_ = environment;			                      // Apenas c�pia o ambiente do arquivo CFG passado por parametro para poder manipul�-lo sem receio.
	
	bsearchuntilfirstsolution = false;                        //se FALSE, ent�o o planejador alocar� o tempo m�ximo para localizar e melhorar a solu��o "allocatime_time_sec", independentemente de ele encontrar uma solu��o ou n�o (modo padr�o)
										                      // se TRUE, ent�o o planejador pesquisar� at� encontrar a primeira solu��o e p�ra, n�o gastando tempo na melhoria da solu��o, mesmo se houver tempo disponivel, 
										                      // normalmente bSearchUntilFirstSolution deve ser definido como false 
	
    
	finitial_eps = RSTAR_DEFAULT_INITIAL_EPS;                  // 5.0 ( valor inicial de W ) o mesmo � decrescido na import�ncia de 0.2 at� chegar 1.0 
    
	highlevel_searchexpands = 0;                               // N�mero de expansoes de alto nivel j� realizadas ( gr�fico GAMA )
    
	lowlevel_searchexpands = 0;                                // N�mero de expansoes de baixo nivel j� realizadas ( busca local )

	//In other words, state structure for high level states in Gamma graph and low-level (local) search state in R*
    
	MaxMemoryCounter = 0;                                      // Contador de mem�ria utilizada - [Por algum bug d� negativo - Verificar isso depois]	



#ifndef ROS							                           // [IGNORAR] Diretivas usadas para setar algumas configuras, caso o sistema seja ROS // DESPREZAR
    const char* debug = "debug.txt";                           // [IGNORAR] DESPREZAR
#endif								                           // [IGNORAR] DESPREZAR

    fDeb = SBPL_FOPEN(debug, "w");                             // [IGNORAR] Fdeb � do tipo arquivo, por dedu��o o primeiro parametro � o arquivo que deve ser aberto, no caso debug.txt(gerado automaticamente) e o segundo paramentro parece ser o comando de escrita [S� dedu��o, pode n�o ser isso]
   
	if(fDeb == NULL){				                           // [IGNORAR] Exce��o simples caso o arquivo n�o exista ou n�o possa ser aberto [ o arquivo DEBUG n�o � prioridade ]
      SBPL_ERROR("ERROR: could not open planner debug file\n");// [IGNORAR] Desprezar
      throw new SBPL_Exception();                              // [IGNORAR] Desprezar
    }
   
	SBPL_PRINTF("debug on\n");		                           // [IGNORAR] Apenas imprimi no prompt a mensagem passada por parametro.
    
	//create global searchstatespace
    pSearchStateSpace = new RSTARSearchStateSpace_t;           // Cria uma busca global com os seguintes com os seguinte atributos, 
													           // eps, eps_satisfied, OPEN, seachitation, callnumber, searchgoalstate, searchstartstate, seachMDP, 
													           // bReevalueatefVals, bReinitializeSearchStateSpace, bNewSeachIteration
	
	MaxMemoryCounter += sizeof(RSTARSearchStateSpace_t);       // [IGNORAR] Atualiza o contador de mem�ria, Sizeof  determines the size, in bytes, of a variable or data type. 
														       // can be used to get the size of classes, structures, unions and any other user defined data type.
    
	//create local searchstatespace
	pLSearchStateSpace = new RSTARLSearchStateSpace_t;         // Cria uma busca local com os seguintes com os seguinte atributos, 
   														       // MDP (grafico construido pela pesquisa local), StartState, goalState, iteration, 
														       //[lista] open, [lista] incons (usada na busca suboptimal) 

	MaxMemoryCounter += sizeof(RSTARLSearchStateSpace_t);      // [IGNORAR] Atualiza novamente o contador de mem�ria,
        
    //create the RSTAR planner
    if(CreateSearchStateSpace() != 1){                         //Instancia a lista aberta pertencente pSearchStateSpace, Seta o estado Start e Goal como nulo e por fim a variavel 
															   //bReinitializeSearchStateSpace = false (Reseta o numero de iteracoes, limpa a lista aberta, reinializa Eps e eps_satisfied ... Entre outras coisas)
															   // se tudo de certo retorna 1
       SBPL_ERROR("ERROR: failed to create statespace\n");     
     return;
    }
    
    //set the start and goal states								//Linha 1005
    if(InitializeSearchStateSpace() != 1){                      // Verifica se a lista aberta est� vazia caso n�o lan�a uma exce��o. � definido algumas variaveis, tais como: eps(w) = 5,0, eps_satisfied = INFINITO, 
																// searchIterator = 0  � incrementado a cada pesquisa R* (e resetado ap�s cada incremento de CallNumber) informa que essa � uma nova itera��o
		SBPL_ERROR("ERROR: failed to create statespace\n"); 	// , callnumber =0; � incrementado a cada chamada do R*, bReevaluatefvals = false; // need to reevaluate fvals, Linha 983 
	return;										                //e ent�o reinicializa o estado de busca - breinitializeSearchStateSpace = true; // Reinicializa o espa�o de busca - Linha 1080 - ReInitializeSearchStateSpace()
	}									                        // ainda n�o define GOAL e START = NULL    
}

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

RSTARPlanner::~RSTARPlanner() // DESTRUTOR
{
  if(pSearchStateSpace != NULL){
    //delete the statespace
    DeleteSearchStateSpace(); // Linha 1023 deallocates memory used by SearchStateSpace 
    delete pSearchStateSpace;
  }
  SBPL_FCLOSE(fDeb);
}

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

// QUal a diferenca de CMDP e RState						// [Esse m�todo � chamado na linha 183 pelo m�todo CREATE STATE]
															
void RSTARPlanner::Initialize_searchinfo(CMDPSTATE* state){ // O m�todo recebe um estado como parametro. O mesmo � formado por: StateID; vector<CMDPACTION*> Actions; vector<int> PredsID; void* PlannerSpecificData e � respons�vel em inicializar 
															// algumas informa��es sobre o mesmo, para isso ele faz ...

	RSTARState* searchstateinfo = (RSTARState*)state->PlannerSpecificData; // Cria uma variavel que armazenara o conteudo do atributo PlannerSpecificData ( que pode ser ANAStar, RStar, ADStar ... Dependendo de quem o instancie)

	searchstateinfo->MDPstate = state; // atualiza as informa��es sobre o proprio estado MDP recebidas por parametro
	
	InitializeSearchStateInfo(searchstateinfo); // Linha 514 -  M�todo respons�vel em inicializar alguns atributos de controle do estado, como G= INFINITO, IterationClosed = 0;
												//CallNumberAcessed = pSearchStateSpace->callnumber; heapindex = 0; bestpredaction = NULL; Bem como calcula o valor da heuristica H, para isso ... 
													//Se (searchgoalstate != NULL - J� exista o estado meta) 
														//state->h = ComputeHeuristic(state->MDPstate); - Linha 480 
															//Primeiro verifica se GoalState = NULL, se for retorna 0 (que signifca que a heuristica ser� recalculada de qualquer forma quando o estado objetivo for atualizado. 
															//Depois � verificado se bforwardseach = true (FORWARD) 
															//Se for forward, ser� criado uma variavel inteira para receber o resultado do metodo GetFromToHeuristic, e depois � invocado o m�todo 
																//GetFromToHeuristic(MDPstate->StateID, pSearchStateSpace->searchgoalstate->StateID);
																//(int EnvironmentNAVXYTHETALAT::GetFromToHeuristic(int FromStateID, int ToStateID) - Linha 2836 (environment_navyxythetalat.cpp) � pego X e Y do estado e faz um calculo de distancia euclidiana)
															//Se n�o for forward e sim backward, ser� executado:
																//GetFromToHeuristic(pSearchStateSpace->searchgoalstate->StateID, MDPstate->StateID); 
													//Se SearchGoalState == null - N�o existir ainda estado objetivo)
												// E por fim executa o m�todo state->predactionV.clear(); limpando o conjunto de a��es antecessores - linha 564
}

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

CMDPSTATE* RSTARPlanner::CreateState(int stateID){				// [Esse m�todo � chamado na linha 191 pelo m�todo GetState]

	CMDPSTATE* state = NULL; // Inicialmente o estado � criado como NULL, tal estado � composto de: int StateID; vector<CMDPACTION*> Actions; vector<int> PredsID; 	void* PlannerSpecificData;

#if DEBUG
	if(environment_->StateID2IndexMapping[stateID][RSTARMDP_STATEID2IND] != -1)
	{
		SBPL_ERROR("ERROR in CreateState: state already created\n");
		throw new SBPL_Exception();
	}
#endif

	//adds to the tail a state
	state = pSearchStateSpace->searchMDP.AddState(stateID); // Aciona o ID do estado, recebido por paramentro no grafo de alto n�vel pertencente a pSearchStateSpace. (StateID = ID; PlannerSpecificData = NULL;)

	//remember the index of the state
	environment_->StateID2IndexMapping[stateID][RSTARMDP_STATEID2IND] = pSearchStateSpace->searchMDP.StateArray.size()-1;
	
	//NOMECLATURA: used in environment to contain the coordinates of a state, say x,y or x,y,theta, usado para mapear o ambiente.
	//StateID2IndexMapping [100][0] = 5 significa que hashentry com stateID 100 � mapeado para o �ndice de pesquisa = 5 em busca 0 
    //O valor -1 significa que nenhum Estado busca foi criado ainda para este hashentry

#if DEBUG
	if(state != pSearchStateSpace->searchMDP.StateArray[environment_->StateID2IndexMapping[stateID][RSTARMDP_STATEID2IND]])
	{
		SBPL_ERROR("ERROR in CreateState: invalid state index\n");
		throw new SBPL_Exception();
	}
#endif


	//create search specific info
	state->PlannerSpecificData = new RSTARState; // Seta a variavel de informa��es especifica do planejador como RStarState.	
	MaxMemoryCounter += sizeof(RSTARState); // IGNORA
	Initialize_searchinfo(state); // Chama o m�todo de inicializacao da informa��o de busca na linha 125

	return state; // Ap�s, retorna o estado.

}

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// [Esse m�todo � chamado na linha 602, GetGVal; 654-ImprovePath; 1147-SetSearchGoalState; 1178 - SetSearchStartState; 1207-GetHeurValue; 1215- GetSearchPath]
CMDPSTATE* RSTARPlanner::GetState(int stateID){	 // M�todo respons�vel em retornar o estado com base em seu ID, 

	if(stateID >= (int)environment_->StateID2IndexMapping.size())	{  // Inicialmente � verificado se o indice passado est� dentro dos limites de estados existentes. 
          SBPL_ERROR("ERROR int GetState: stateID %d is invalid\n", stateID);
		throw new SBPL_Exception();
	}

	if(environment_->StateID2IndexMapping[stateID][RSTARMDP_STATEID2IND] == -1) //O valor -1 significa que nenhum estado busca foi criado ainda para este hashentry e nesse caso ser� criado um estado com esse ID
		return CreateState(stateID); //retorna o estado criado pelo m�todo CreateState na linha 149
	else
		return pSearchStateSpace->searchMDP.StateArray[environment_->StateID2IndexMapping[stateID][RSTARMDP_STATEID2IND]]; //Retorna o estado com o indice StateID

}

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------



//----------------------------------------functions related to local searches---------------------------------------------
void RSTARPlanner::Initialize_rstarlsearchdata(CMDPSTATE* state){			// M�todo chamado na linha 228 pelo o outro metodo CreateLSearchState
																			// Esse � respons�vel em iniciar as variaveis usadas em uma pesquisa local
	RSTARLSearchState* rstarlsearch_data = (RSTARLSearchState*)state->PlannerSpecificData; // Passa as informa�oes do planejador para a variavel RStarLSearch_data

	rstarlsearch_data->bestpredstate = NULL;
	rstarlsearch_data->bestpredstateactioncost = 0;
	rstarlsearch_data->iteration = 0;
    rstarlsearch_data->iterationclosed = 0;
	rstarlsearch_data->g = INFINITECOST; 
    rstarlsearch_data->heapindex = 0;
    rstarlsearch_data->listelem[0] = NULL;
    rstarlsearch_data->listelem[1] = NULL;
    
    //pointer to itself
    rstarlsearch_data->MDPstate = state; //Seta o grafo MDP do estado com o estado passado por parametro
}

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
CMDPSTATE* RSTARPlanner::CreateLSearchState(int stateID){            //Esse m�todo � chamado pelo m�todo GetLSearchState (linha 262)
																	 // O mesmo � respons�vel em criar uma pesquisa local
	CMDPSTATE* state = NULL;										 // Inicialmente cria um estado nulo.

#if DEBUG
	if(environment_->StateID2IndexMapping[stateID][RSTARMDP_LSEARCH_STATEID2IND] != -1)
	{
		SBPL_ERROR("ERROR in CreateState: state already created\n");
		throw new SBPL_Exception();
	}
#endif

	//adds to the tail a state
	state = pLSearchStateSpace->MDP.AddState(stateID); // Adciona o iD do estado ao grafo MDP e esse a variavel State

	//remember the index of the state
	environment_->StateID2IndexMapping[stateID][RSTARMDP_LSEARCH_STATEID2IND] = pLSearchStateSpace->MDP.StateArray.size()-1;

#if DEBUG
	if(state != pLSearchStateSpace->MDP.StateArray[environment_->StateID2IndexMapping[stateID][RSTARMDP_LSEARCH_STATEID2IND]])
	{
		SBPL_ERROR("ERROR in CreateState: invalid state index\n");
		throw new SBPL_Exception();
	}
#endif

	//create and initialize rstarlsearch_data
	state->PlannerSpecificData = new RSTARLSearchState; //Cria  um estado de busca local com os seguintes atributos: MDP; StartState; GoalState; iteration; OPEN; (incons list - used for suboptimal search) INCONS;
	Initialize_rstarlsearchdata(state); // Inicializa o estado local, Linha 210

	return state; //Ap�s a inicializa��o das informa��es retorna o estado

}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
CMDPSTATE* RSTARPlanner::GetLSearchState(int stateID){	// m�todo muito parecido com o GetState  da linha 191. Este m�todo respons�vel em retornar o estado local com base em seu ID 
														// [Esse m�todo � chamado na linha 295 pelo m�todo ComputeLocalPath]

	if(stateID >= (int)environment_->StateID2IndexMapping.size())	{ // Inicialmente � verificado se o indice passado est� dentro dos limites de estados existentes. 

		SBPL_ERROR("ERROR int GetLSearchState: stateID is invalid\n");
		throw new SBPL_Exception();
	}

	if(environment_->StateID2IndexMapping[stateID][RSTARMDP_LSEARCH_STATEID2IND] == -1) // Logo em seguida � verificado se h� algum estado de busca criado (O valor -1 significa que nenhum estado busca foi criado ainda para este hashentry e nesse caso ser� criado um estado com esse ID)
		return CreateLSearchState(stateID);
	else
		return pLSearchStateSpace->MDP.StateArray[environment_->StateID2IndexMapping[stateID][RSTARMDP_LSEARCH_STATEID2IND]]; // Em seguida � retornado o estado local

}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
CKey RSTARPlanner::LocalSearchComputeKey(RSTARLSearchState* rstarlsearchState){ // [Esse m�todo � chamado na linha 295 pelo m�todo ComputeLocalPath]

    CKey retkey; // Fazendo papel de F, F = G + H

	int h;
	if(bforwardsearch) // Se a pesquisa for forward
		h = environment_->GetFromToHeuristic(rstarlsearchState->MDPstate->StateID, pLSearchStateSpace->GoalState->StateID); // Envia o ID da posi��o atual e da posi��o objetivo e ent�o � calculado o H chamando o m�todo GetFromHeuristic. Linha 2836(environment_navXYTHETALAT.cpp)
	else
		h = environment_->GetFromToHeuristic(pLSearchStateSpace->GoalState->StateID, rstarlsearchState->MDPstate->StateID); // Envia o ID da posi��o objetivo e da posi��o atual e ent�o � calculado o H.

	retkey.key[0] = rstarlsearchState->g + (int)(pSearchStateSpace->eps * h); //Ent�o � calculado o F = G+eps(w) *h

    return retkey; 
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

											// M�todo chamado na linha 756 pelo m�todo ImprovePath
bool RSTARPlanner::ComputeLocalPath(int StartStateID, int GoalStateID, int maxc, int maxe, int *pCost, int *pCostLow, int *pExp, vector<int>* pPathIDs, int* pNewGoalStateID, double maxnumofsecs){
//ComputeLocalPath(rstarpredstate->MDPstate->StateID, rstarstate->MDPstate->StateID, maxc, maxe,&computedaction->Costs[0], &computedactiondata->clow, &computedactiondata->exp, &computedactiondata->pathIDs, &NewGoalStateID, MaxNumofSecs);

    vector<int> SuccIDV; // Conjunto de K sucessores escolhidos randomicamente
    vector<int> CostV;              

    if(pLSearchStateSpace->OPEN == NULL) // Verifica se a lista aberta e a lista de Incons n�o foi criada, se isso for verdade ele cria
        pLSearchStateSpace->OPEN = new CHeap;
    if(pLSearchStateSpace->INCONS == NULL) // Cria aqui a INCONSLIST, mas n�o utiliza e n�o preenche
        pLSearchStateSpace->INCONS = new CList;

    int local_expands = 0;
    *pNewGoalStateID = GoalStateID;

    //increase iteration
    pLSearchStateSpace->iteration++;

    //set the start and goal states
    pLSearchStateSpace->StartState = GetLSearchState(StartStateID);
    pLSearchStateSpace->GoalState = GetLSearchState(GoalStateID);

    RSTARLSearchState* rstarlsearchstate = (RSTARLSearchState*)pLSearchStateSpace->StartState->PlannerSpecificData; 
    RSTARLSearchState* rstarlsearchgoalstate = (RSTARLSearchState*)pLSearchStateSpace->GoalState->PlannerSpecificData;

    //OPEN=0 (it shouldn't be necessary since the memory is cleared before each search)
    pLSearchStateSpace->OPEN->makeemptyheap(); // Limpara as listas a cada iteracao
    pLSearchStateSpace->INCONS->makeemptylist(RSTAR_INCONS_LIST_ID); // Esvazia a lista de incons

    //CLOSED = 0 because iteration is increased
    
    //g(start) = 0
    rstarlsearchstate->g = 0;

    //insert start into open
    pLSearchStateSpace->OPEN->insertheap(rstarlsearchstate, LocalSearchComputeKey(rstarlsearchstate)); // Insere start na lista aberta

    //TODO - prove that min_{OPEN and INCONS} (g + eps*h) <= eps*c*. (proof: take min_{OPEN and INCONS} (g+h) <= c* and
    //multiply both sides by eps and then bring eps into min and drop one by g. I still need to implement INCONS and minimum tracker for it - then change 
    //the minkey to min over two sets
    while(rstarlsearchgoalstate->g > pLSearchStateSpace->OPEN->getminkeyheap().key[0]  && local_expands < maxe &&
        pLSearchStateSpace->OPEN->getminkeyheap().key[0] <= maxc)    {

        //pop the min element
        rstarlsearchstate = (RSTARLSearchState*)pLSearchStateSpace->OPEN->deleteminheap();
        
        //close the state
        rstarlsearchstate->iterationclosed = pLSearchStateSpace->iteration;

        //expansion
        local_expands++;
        //environment_->PrintState(rstarlsearchstate->MDPstate->StateID, false, fLowLevelExp);

        //generate SUCCS state
        SuccIDV.clear();
        CostV.clear();
        //this setting makes it to get all successors - since this is a deterministic search
		if(bforwardsearch == false)
	        environment_->GetPreds(rstarlsearchstate->MDPstate->StateID, &SuccIDV, &CostV); 
		else
	        environment_->GetSuccs(rstarlsearchstate->MDPstate->StateID, &SuccIDV, &CostV); 

        //iterate over states in SUCCS set
		for(int i = 0; i < (int)SuccIDV.size(); i++) {

            RSTARLSearchState* rstarlsearchSuccState = (RSTARLSearchState*)GetLSearchState(SuccIDV.at(i))->PlannerSpecificData;
            
            //skip if the state is already closed - TODO fix this with INCONS list - it seems to make five times less expansions!
            //if(rstarlsearchSuccState->iterationclosed == pLSearchStateSpace->iteration)
            //continue;
        
            //see if we can improve g-value of successor
            if(rstarlsearchstate->g + CostV[i] < rstarlsearchSuccState->g) {

                rstarlsearchSuccState->bestpredstate = rstarlsearchstate->MDPstate;
                rstarlsearchSuccState->bestpredstateactioncost = CostV[i]; 
                rstarlsearchSuccState->g = rstarlsearchstate->g + CostV[i];
                if(rstarlsearchSuccState->heapindex == 0)
                    pLSearchStateSpace->OPEN->insertheap(rstarlsearchSuccState, LocalSearchComputeKey(rstarlsearchSuccState));
                else
                    pLSearchStateSpace->OPEN->updateheap(rstarlsearchSuccState, LocalSearchComputeKey(rstarlsearchSuccState));


                if(environment_->AreEquivalent(rstarlsearchSuccState->MDPstate->StateID, rstarlsearchgoalstate->MDPstate->StateID) == true && rstarlsearchSuccState->g < rstarlsearchgoalstate->g) {

                    //swap the goal
                    rstarlsearchgoalstate = rstarlsearchSuccState;
                    GoalStateID = rstarlsearchgoalstate->MDPstate->StateID;
                }
            }
        }

		if(local_expands%10000 == 0){

			if((clock()-TimeStarted) >= maxnumofsecs*(double)CLOCKS_PER_SEC) 
			{
				SBPL_PRINTF("breaking local search because global planning time expires\n");
				break;
			}
		}


    }//while

    lowlevel_searchexpands += local_expands;
    SBPL_FPRINTF(fDeb, "local search: expands=%d\n", local_expands);

    //set the return path and other path related variables
    vector<int> tempPathID;
    pPathIDs->clear();
    if(rstarlsearchgoalstate->g < INFINITECOST)    {

        int pathcost = 0;

        //path exists
        rstarlsearchstate = rstarlsearchgoalstate;
        while(rstarlsearchstate->bestpredstate != NULL && rstarlsearchstate->MDPstate != pLSearchStateSpace->StartState)        {

            tempPathID.push_back(rstarlsearchstate->MDPstate->StateID);
            pathcost += rstarlsearchstate->bestpredstateactioncost;
            rstarlsearchstate = (RSTARLSearchState*)rstarlsearchstate->bestpredstate->PlannerSpecificData;
        }
        //store into pPathIDs so that the path is always forward path w.r.t. the original graph 
		//this requires us to reverse order in case of forward search
        for(int i = 0; i < (int)tempPathID.size(); i++)        {

			if(bforwardsearch == false)
				pPathIDs->push_back(tempPathID.at(i));
			else
	            pPathIDs->push_back(tempPathID.at(tempPathID.size() - i - 1));
        }
        *pCost = pathcost;
        *pCostLow = rstarlsearchgoalstate->g;
        SBPL_FPRINTF(fDeb, "pathcost=%d while g=%d\n", pathcost, rstarlsearchgoalstate->g);
    }
    else
    {
        *pCost = INFINITECOST;
        *pCostLow = pLSearchStateSpace->OPEN->getminkeyheap().key[0]; 
        if(*pCostLow != pLSearchStateSpace->OPEN->getminkeyheap().key[0]){
	  SBPL_FPRINTF(fDeb, "after localsearch clow=%d while keymin=%d\n", (int)(*pCostLow), 
		  (int)pLSearchStateSpace->OPEN->getminkeyheap().key[0]);
        }
    }

    //set all other return variables
    *pExp = local_expands;
    *pNewGoalStateID = GoalStateID;

    return true;

}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
bool RSTARPlanner::DestroyLocalSearchMemory(){ //Esse m�todo � chamado na linha 654 pelo m�todo improvePath

    pLSearchStateSpace->OPEN->currentsize = 0; // Seta o tamanho da lista aberta para 0
    pLSearchStateSpace->StartState = pLSearchStateSpace->GoalState = NULL; // Coloca O ponto inicial e o objetivo como nulo.

    //remove the states in the MDP itself
    for(int i = 0; i < (int)pLSearchStateSpace->MDP.StateArray.size(); i++)    { // varra todo o Grafo MDP

        CMDPSTATE* state = pLSearchStateSpace->MDP.StateArray.at(i); // guarda o elemento da vez na variavel state
        RSTARLSearchState* rstarlsearchstatedata = (RSTARLSearchState*)state->PlannerSpecificData;    //Armazena o tipo de busca da variavel state na variavel RstarL ...
        delete rstarlsearchstatedata; // Apaga
        state->PlannerSpecificData = NULL; // Seta o tipo como nulo
        environment_->StateID2IndexMapping[state->StateID][RSTARMDP_LSEARCH_STATEID2IND] = -1; // seta como -1 que indica que ainda nao tem elementos
    }
    //now we can delete the states themselves
    if(pLSearchStateSpace->MDP.Delete() == false)
    {
        SBPL_ERROR("ERROR: failed to delete local search MDP\n");
        throw new SBPL_Exception();
    }

    //TODO - ask Env to delete the new memory allocated during the local search (usings the fact that stateID's for local search continuously and no states were
    //allocated for global search before local search exits.
	//pedir para apagar a nova mem�ria alocada durante a busca local (usando o fato de que o estado de de busca local de forma cont�nua e sem estados foram // alocada para pesquisa global antes de sa�das de pesquisa local.
	return true;
}

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

//----------------------------------------------------------------------------------------------------------------------


// M�todo chamado na linha 514 pelo m�todo InitializeSearchStateInfo, bem como na linha 539 pelo m�todo ReinitializeSearchStateInfo e por fim, na linha 1147 pelo m�todo SetSeachGoalState
int RSTARPlanner::ComputeHeuristic(CMDPSTATE* MDPstate){

	//compute heuristic for search

	if(pSearchStateSpace->searchgoalstate == NULL) // Se ainda n�o for definido o ponto objetivo, retorne 0
		return 0; //the heuristics will be re-computed anyway when searchgoalstate is updated

	if(bforwardsearch) // Se a dire��o de busca for forward
	{

#if MEM_CHECK == 1
		//int WasEn = DisableMemCheck();
#endif
		// Bem parecido com o m�todo LocalSearchComputeKey - Linha 278
		//forward search: heur = distance from state to searchgoal which is Goal RSTARState
		int retv =  environment_->GetFromToHeuristic(MDPstate->StateID, pSearchStateSpace->searchgoalstate->StateID);

#if MEM_CHECK == 1
		//if (WasEn)
		//	EnableMemCheck();
#endif

		return retv;

	}
	else
	{
		//backward search: heur = distance from searchgoal to state
		return environment_->GetFromToHeuristic(pSearchStateSpace->searchgoalstate->StateID, MDPstate->StateID);
	}
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

//Esse m�todo � chamado na linha 132 pelo metodo Initialize_searchinfo
void RSTARPlanner::InitializeSearchStateInfo(RSTARState* state)
{
	state->g = INFINITECOST;
	state->iterationclosed = 0;                                //searchiteration � incrementado a cada pesquisa do R* (e reinicia ap�s cada incremento de CallNumber)
	state->callnumberaccessed = pSearchStateSpace->callnumber; //CallNumber � incrementado a cada chamada ao R* (sim, ele pode ser chamado varias vezes dentro do planejador desde que que o R* seja executado v�rias vezes )
	state->heapindex = 0;
	state->bestpredaction = NULL;

	//compute heuristics
#if USE_HEUR
	if(pSearchStateSpace->searchgoalstate != NULL) // Se j� existir ponto destino chame computeHeuristic (Linha 480), caso n�o diga que a heuristica � apenas 0
		state->h = ComputeHeuristic(state->MDPstate); 
	else 
		state->h = 0;
#else
	state->h = 0;
#endif

	state->predactionV.clear(); // Conjunto dos caminhos preantecedidos e outras informa��es s�o limpas

}

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

//re-initialization of a state    [ Esse m�todo � chamado na linha 654 pelo m�todo ImprovePath e em s�guida pelo m�todo ReinitializeSearchStateSpace, na linha 1078 ]
void RSTARPlanner::ReInitializeSearchStateInfo(RSTARState* state){

	state->g = INFINITECOST;
	state->iterationclosed = 0; // tem o comportamente muito parecido com o m�todo InitializeSearchStateInfo 514, modificando apenas a partir da linha 565
	state->callnumberaccessed = pSearchStateSpace->callnumber;
	state->heapindex = 0;

	state->bestpredaction = NULL;

	//compute heuristics
#if USE_HEUR

	if(pSearchStateSpace->searchgoalstate != NULL)
	{
		state->h = ComputeHeuristic(state->MDPstate); 
	}
	else 
		state->h = 0;

#else

	state->h = 0;

#endif
	//deleta o PlannerSpecificData para cada caminho (a��o)
	state->predactionV.clear();
	for(int i = 0; i < (int)state->MDPstate->Actions.size(); i++){
		if(state->MDPstate->Actions.at(i)->PlannerSpecificData != NULL){ // Varre todo o grafo MDP escolhendo os estados com Informa��es do planejador j� setadas
			DeleteSearchActionData((RSTARACTIONDATA*)state->MDPstate->Actions.at(i)->PlannerSpecificData); //�m�todo sem nenhuma implementacao na linha 594
			delete (RSTARACTIONDATA*)(state->MDPstate->Actions.at(i)->PlannerSpecificData); // Deleta a informa��o do planejador 
			state->MDPstate->Actions.at(i)->PlannerSpecificData = NULL; // seta a mesma para nula
		}
	} //Necess�rio deletar os caminhos predecessores
	state->MDPstate->RemoveAllActions(); // deleta o array de caminhos. 
	
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

																		//M�todo chamado na linha 1021 pelo m�todo DeleteSearchStateSpace
void RSTARPlanner::DeleteSearchStateData(RSTARState* state){

	//delete PlannerSpecificData for each action
	state->predactionV.clear();
	for(int i = 0; i < (int)state->MDPstate->Actions.size(); i++){
		if(state->MDPstate->Actions.at(i)->PlannerSpecificData != NULL){ // Varre todo o grafo MDP escolhendo os estados com Informa��es do planejador j� setadas
			DeleteSearchActionData((RSTARACTIONDATA*)state->MDPstate->Actions.at(i)->PlannerSpecificData);//�m�todo sem nenhuma implementacao na linha 594
			delete (RSTARACTIONDATA*)state->MDPstate->Actions.at(i)->PlannerSpecificData; // Deleta a informa��o do planejador 
			state->MDPstate->Actions.at(i)->PlannerSpecificData = NULL; // seta a mesma para nula
		}
	}
	state->MDPstate->RemoveAllActions(); // Deleta o array de caminhos ( Linha 209 mdp.cpp )
	
	return;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void RSTARPlanner::DeleteSearchActionData(RSTARACTIONDATA* actiondata) // Esse m�todo � chamado na linha 539 pelo m�todo ReinitializeSearchStateInfo, na linha 578 no m�todo DeleteSearchSteteData e por fim na linha 1021, pelo m�todo DeleteSearchStateSpace
{
	//no memory was allocated for actiondata

	return;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

int RSTARPlanner::GetGVal(int StateID){ // N�o encontrei quem chama esse m�todo no RStar

	 CMDPSTATE* cmdp_state = GetState(StateID); // � pego o estado com o id passado por paramentro e armazenado na variavel local 
	 RSTARState* state = (RSTARState*)cmdp_state->PlannerSpecificData; //Em seguida � armazenado em outra variavel temporaria o resultado do PlannerSpecificData
	 return state->g; //� retornado o G armazenado na variavel estado
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void  RSTARPlanner::SetBestPredecessor(RSTARState* rstarState, RSTARState* rstarPredState, CMDPACTION* action){ // m�todo responsavel em setar o melhor caminho antecessor
																												// [M�todo chamado na linha 654 pelo m�todo ImprovePath]
    rstarState->bestpredaction = action;
    rstarState->g = rstarPredState->g + ((RSTARACTIONDATA*)(action->PlannerSpecificData))->clow; // Atualizar o G ( G do predecessor mais o G atual )
    if(rstarState->heapindex == 0) // Se o index da pilha for igual a 0, n�o tem nada incluso, se nao for (tem algo) atualize a heap
        pSearchStateSpace->OPEN->insertheap(rstarState, ComputeKey(rstarState));
    else
        pSearchStateSpace->OPEN->updateheap(rstarState, ComputeKey(rstarState));

}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
CKey RSTARPlanner::ComputeKey(RSTARState* rstarState){ // [M�todo chamado na linha 610 pelo m�todo SetBestPredecessor; Bem como na linha 654 pelo m�todo ImprovePath; tamb�m na linha 981 pelo m�todo Reevaluete e por fim na linha 1078 pelo m�todo ReInitializeSearchStateSpace

    CKey retkey;

	int h, starttostateh; // S�o criadas duas variaveis locais
	if(bforwardsearch) // Se for FORWARD
	{
		h = environment_->GetFromToHeuristic(rstarState->MDPstate->StateID, pSearchStateSpace->searchgoalstate->StateID); // � calculado o H considerando a distancia Euclidiana do ponto atual at� a meta e armazenado na variavel h
		starttostateh = environment_->GetFromToHeuristic(pSearchStateSpace->searchstartstate->StateID, rstarState->MDPstate->StateID); // Depois � calculado o H considerando a distancia Euclidiana do ponto inicial at� o ponto atual e armazenado na variavel startTostateH
	else
	{
		h = environment_->GetFromToHeuristic(pSearchStateSpace->searchgoalstate->StateID, rstarState->MDPstate->StateID); // � calculado o H considerando a distancia Euclidiana do ponto objetivo at� o ponto atual e armazenado na variavel h
		starttostateh = environment_->GetFromToHeuristic(rstarState->MDPstate->StateID, pSearchStateSpace->searchstartstate->StateID); // Depois � calculado o H considerando a distancia Euclidiana do ponto atual at� o ponto inicial e armazenado na variavel startTostateH
	}

	//compute 2nd element of the key   
    retkey.key[1] = rstarState->g + (int)(pSearchStateSpace->eps*h); // Pode-se considerar retkey.key[x] = as prioridade de K. // k[1, g(s) + w h(start, s) = AVOID
	//retkey.key[1] =  (int)(pSearchStateSpace->eps*h);
	
	//compute the 1st element
    if(rstarState->g > pSearchStateSpace->eps*starttostateh || // Se G(s) > w h(start,s) (1� condi��o para evitar um estado)
        (rstarState->bestpredaction != NULL && // No artigo vinha dizendo que era para ser =Null e n�o diferente
		((RSTARACTIONDATA*)rstarState->bestpredaction->PlannerSpecificData)->pathIDs.size() == 0 &&
         ((RSTARACTIONDATA*)rstarState->bestpredaction->PlannerSpecificData)->exp >= RSTAR_EXPTHRESH)) // Se expandir mais que 150 deve ser marcado como EVITAR
        retkey.key[0] = 1; 
    else
        retkey.key[0] = 0;
   
	return retkey;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

//returns 1 if the solution is found, 0 if the solution does not exist and 2 if it ran out of time
int RSTARPlanner::ImprovePath(double MaxNumofSecs){ // M�todo chamado na linha 1327 pelo m�todo Search

	int expands;
	RSTARState *rstarstate, *searchgoalstate, *searchstartstate;
	CKey key, minkey;
	CKey goalkey;

    vector<int> SuccIDV;
    vector<int> CLowV;
	int highlevelexpands = 0;

	expands = 0;

	if(pSearchStateSpace->searchgoalstate == NULL) // Se ainda n�o foi setado um objetivo, lance uma exce��o
	{
		SBPL_ERROR("ERROR searching: no goal state is set\n");
		throw new SBPL_Exception();
	}

	//goal state
	searchgoalstate = (RSTARState*)(pSearchStateSpace->searchgoalstate->PlannerSpecificData); //Caso j� tenha setado o objetivo, armazene essa informa��o na variavel SearchGoalState
	if(searchgoalstate->callnumberaccessed != pSearchStateSpace->callnumber) // Se numero de chamadas de acesso no estado objetivo for diferente do numero de chamadas, reinicialize as informa��es de busca para a partir daqui j� est� tudo ok
		ReInitializeSearchStateInfo(searchgoalstate); // Linha 539

	//get the start state
	searchstartstate = (RSTARState*)(pSearchStateSpace->searchstartstate->PlannerSpecificData); // Sete as informa��es do estado inicial e armazene essa informa��o na variavel SearchStartState

	//set goal key
    if(searchgoalstate != searchstartstate){
        goalkey.key[0] = 1;                     // Se n�o chegou no ponto objetivo
        goalkey.key[1] = INFINITECOST;
    }
    else{
		goalkey = ComputeKey(searchstartstate);  //Se Goal = Start (calcule a Key) 621 - [ Est� referenciando errado ao ADPlanner - 166 adplanner.cpp]
    }			  


	//expand states until done
	minkey = pSearchStateSpace->OPEN->getminkeyheap(); // Pega o primeiro estado do Heap e retorna (linha 305 - heap.cpp)
	while(!pSearchStateSpace->OPEN->emptyheap() &&
		(clock()-TimeStarted) < MaxNumofSecs*(double)CLOCKS_PER_SEC) { // Enquanto a lista aberta nao estiver vazia e o tempo de execu��o do algoritmo n�o ultrapassar o limite de 1000 segundos Continue. Obs timeStarted() � instanciado na linha 1331

      
      //recompute minkey
      minkey = pSearchStateSpace->OPEN->getminkeyheap();
      
      //recompute goalkey if necessary
      goalkey = ComputeKey(searchgoalstate); // Retornar� 1 se for para evitar o estado ou 0 para consideradr
      
      if(goalkey < minkey)
        break; //termination condition


        //pop the min element
        rstarstate = (RSTARState*)pSearchStateSpace->OPEN->deleteminheap();

        SBPL_FPRINTF(fDeb, "ComputePath:  selected state %d g=%d\n", rstarstate->MDPstate->StateID, rstarstate->g);
        environment_->PrintState(rstarstate->MDPstate->StateID, true, fDeb);
        
        if (rstarstate->MDPstate != pSearchStateSpace->searchstartstate &&
            ((RSTARACTIONDATA*)rstarstate->bestpredaction->PlannerSpecificData)->pathIDs.size() == 0) 
        {
            SBPL_FPRINTF(fDeb, "re-compute path\n");
  
            int maxe = INFINITECOST;
            int maxc = INFINITECOST;

            //predecessor
            RSTARState* rstarpredstate = (RSTARState*)GetState(rstarstate->bestpredaction->SourceStateID)->PlannerSpecificData;
            CMDPACTION* computedaction = rstarstate->bestpredaction;
            RSTARACTIONDATA* computedactiondata = (RSTARACTIONDATA*)computedaction->PlannerSpecificData;

            
            if(computedactiondata->exp < RSTAR_EXPTHRESH)
            {
                maxe = RSTAR_EXPTHRESH;
            }
            else
            {
                SBPL_PRINTF("Trying to compute hard-to-find path\n");
                SBPL_FPRINTF(fDeb, "Trying to compute hard-to-find path\n");
                /* TODO
                CKey nextkey = rstarPlanner.OPEN->getminkeyheap();
                
				if(bforwardsearch)
					h = environment_->GetFromToHeuristic(rstarstate->MDPstate->StateID, pSearchStateSpace->GoalState->StateID);
				else
					h = environment_->GetFromToHeuristic(pSearchStateSpace->GoalState->StateID, rstarstate->MDPstate->StateID);

                maxc = nextkey[1] -
                     (rstarpredstate->g + rstarPlanner.epsilon*h) + RSTAR_COSTDELTA;
                */
            }


            SBPL_FPRINTF(fDeb, "recomputing path from bp %d to state %d with maxc=%d maxe=%d\n", 
                rstarpredstate->MDPstate->StateID, rstarstate->MDPstate->StateID, maxc, maxe);
            SBPL_FPRINTF(fDeb, "bp state:\n");
            environment_->PrintState(rstarpredstate->MDPstate->StateID, true, fDeb);

            //re-compute the path
            int NewGoalStateID = rstarstate->MDPstate->StateID;
	       ComputeLocalPath(rstarpredstate->MDPstate->StateID, rstarstate->MDPstate->StateID, maxc, maxe, 
                &computedaction->Costs[0], &computedactiondata->clow, &computedactiondata->exp, &computedactiondata->pathIDs, &NewGoalStateID, MaxNumofSecs);

            SBPL_FPRINTF(fDeb, "return values: pathcost=%d clow=%d exp=%d\n", 
                computedaction->Costs[0], computedactiondata->clow, computedactiondata->exp);
            bool bSwitch = false;
            if(NewGoalStateID != rstarstate->MDPstate->StateID)
            {
                bSwitch = true;
                SBPL_FPRINTF(fDeb, "targetstate was switched from %d to %d\n", rstarstate->MDPstate->StateID, NewGoalStateID);
                SBPL_FPRINTF(stdout, "targetstate was switched from %d to %d\n", rstarstate->MDPstate->StateID, NewGoalStateID);
                environment_->PrintState(NewGoalStateID, true, fDeb);

                RSTARState* rstarNewTargetState = (RSTARState*)GetState(NewGoalStateID)->PlannerSpecificData;
				
				//re-initialize the state if necessary
				if(rstarNewTargetState->callnumberaccessed != pSearchStateSpace->callnumber)
					ReInitializeSearchStateInfo(rstarNewTargetState);

                SBPL_FPRINTF(fDeb, "predstate.g=%d actual actioncost=%d clow=%d newtartetstate.g=%d\n", rstarpredstate->g, computedaction->Costs[0], 
                        ((RSTARACTIONDATA*)computedaction->PlannerSpecificData)->clow, rstarNewTargetState->g);

                //add the successor to our graph
                CMDPACTION* action = rstarpredstate->MDPstate->AddAction(rstarpredstate->MDPstate->Actions.size());
                action->AddOutcome(rstarNewTargetState->MDPstate->StateID, computedaction->Costs[0], 1.0);
                action->PlannerSpecificData = new RSTARACTIONDATA;
				MaxMemoryCounter += sizeof(RSTARACTIONDATA);
                ((RSTARACTIONDATA*)action->PlannerSpecificData)->clow = computedactiondata->clow;
                ((RSTARACTIONDATA*)action->PlannerSpecificData)->exp = computedactiondata->exp;
                ((RSTARACTIONDATA*)action->PlannerSpecificData)->pathIDs = computedactiondata->pathIDs;
                
                //add the corresponding predaction
                rstarNewTargetState->predactionV.push_back(action);
    
                //the action was not found to the old state
                computedaction->Costs[0] = INFINITECOST;
				if(bforwardsearch)
	                computedactiondata->clow = environment_->GetFromToHeuristic(rstarpredstate->MDPstate->StateID, rstarstate->MDPstate->StateID);
				else
	                computedactiondata->clow = environment_->GetFromToHeuristic(rstarstate->MDPstate->StateID, rstarpredstate->MDPstate->StateID);

                computedactiondata->pathIDs.clear();

                rstarstate = rstarNewTargetState;
                computedaction = action; 
                computedactiondata = (RSTARACTIONDATA*)action->PlannerSpecificData;
            }

            //clean up local search memory
            DestroyLocalSearchMemory();
           
            RSTARState* stateu = NULL;
            CMDPACTION* utosaction = NULL;
			int hfromstarttostate;
			if(bforwardsearch)
				hfromstarttostate = environment_->GetFromToHeuristic(searchstartstate->MDPstate->StateID, rstarstate->MDPstate->StateID);
			else
				hfromstarttostate = environment_->GetFromToHeuristic(rstarstate->MDPstate->StateID,searchstartstate->MDPstate->StateID);
            if (computedactiondata->pathIDs.size() == 0 || 
                rstarpredstate->g + computedactiondata->clow > pSearchStateSpace->eps*hfromstarttostate)
            {
               SBPL_FPRINTF(fDeb, "selecting best pred\n");
               //SBPL_FPRINTF(stdout, "selecting best pred\n");

                //select other best predecessor
                unsigned int minQ = INFINITECOST;
                for(int i = 0; i < (int)rstarstate->predactionV.size(); i++)
                {
                    CMDPACTION* predaction = rstarstate->predactionV.at(i);
                    rstarpredstate = (RSTARState*)GetState(predaction->SourceStateID)->PlannerSpecificData;    
                    if(minQ >= rstarpredstate->g + ((RSTARACTIONDATA*)predaction->PlannerSpecificData)->clow)
                    {
                        minQ = rstarpredstate->g + ((RSTARACTIONDATA*)predaction->PlannerSpecificData)->clow;
                        stateu = rstarpredstate;
                        utosaction = predaction;
                    }
                }

                //set the predecessor
                SetBestPredecessor(rstarstate, stateu, utosaction);
            }
            else if(rstarpredstate->g + computedactiondata->clow < rstarstate->g || bSwitch == false)
            {
               SBPL_FPRINTF(fDeb, "keeping the same computedaction\n");
               //SBPL_FPRINTF(stdout, "keeping the same best pred\n");

                stateu = rstarpredstate;
                utosaction = computedaction;

                //set the predecessor
                SetBestPredecessor(rstarstate, stateu, utosaction);

            }
            else
            {
                SBPL_FPRINTF(fDeb, "keeping the same bestpredaction even though switch of targetstates happened\n");
            }           
        }
        else
        {
            SBPL_FPRINTF(fDeb, "high-level normal expansion of state %d\n", rstarstate->MDPstate->StateID);
            environment_->PrintState(rstarstate->MDPstate->StateID, true, fDeb);
            highlevelexpands++;

            //close the state
            rstarstate->iterationclosed = pSearchStateSpace->searchiteration;

            //expansion
            expands++;

            //generate SUCCS state
            SuccIDV.clear();
            CLowV.clear();
			if(bforwardsearch)
	            environment_->GetRandomSuccsatDistance(rstarstate->MDPstate->StateID, &SuccIDV, &CLowV); 
			else
	            environment_->GetRandomPredsatDistance(rstarstate->MDPstate->StateID, &SuccIDV, &CLowV); 


            SBPL_FPRINTF(fDeb, "%d succs were generated at random\n", (unsigned int)SuccIDV.size());

            //iterate over states in SUCCS set
            int notclosed = 0;
            for(int i = 0; i < (int)SuccIDV.size(); i++)
            {
                RSTARState* rstarSuccState = (RSTARState*)GetState(SuccIDV.at(i))->PlannerSpecificData;

                SBPL_FPRINTF(fDeb, "succ %d:\n", i);
                environment_->PrintState(rstarSuccState->MDPstate->StateID, true, fDeb);
                
                //re-initialize the state if necessary
                if(rstarSuccState->callnumberaccessed != pSearchStateSpace->callnumber)
                    ReInitializeSearchStateInfo(rstarSuccState);
                    
                //skip if the state is already closed
                if(rstarSuccState->iterationclosed == pSearchStateSpace->searchiteration)
                {
                    SBPL_FPRINTF(fDeb, "this succ was already closed -- skipping it\n");
                    continue;
                }
                notclosed++;
 				
                //if(rstarSuccState->MDPstate->StateID == 3838){
                //    SBPL_FPRINTF(fDeb, "generating state %d with g=%d bp=%d:\n", rstarSuccState->MDPstate->StateID, rstarSuccState->g, 
                //            (rstarSuccState->bestpredaction==NULL?-1:rstarSuccState->bestpredaction->SourceStateID));
                //    Env_PrintState(rstarSuccState->MDPstate->StateID, true, false, fDeb);
                //}


                //add the successor to our graph
                CMDPACTION* action = rstarstate->MDPstate->AddAction(i);
                action->AddOutcome(rstarSuccState->MDPstate->StateID, INFINITECOST, 1.0);
                action->PlannerSpecificData = new RSTARACTIONDATA;
				MaxMemoryCounter += sizeof(RSTARACTIONDATA);
                ((RSTARACTIONDATA*)action->PlannerSpecificData)->clow = CLowV[i];
                ((RSTARACTIONDATA*)action->PlannerSpecificData)->exp = 0;
                ((RSTARACTIONDATA*)action->PlannerSpecificData)->pathIDs.clear();

                //add the corresponding predaction
                rstarSuccState->predactionV.push_back(action);

                //see if we can improve g-value of successor
                if(rstarSuccState->bestpredaction == NULL || rstarstate->g + CLowV[i] < rstarSuccState->g)
                {
                    SetBestPredecessor(rstarSuccState, rstarstate, action);
                    SBPL_FPRINTF(fDeb, "bestpred was set for the succ (clow=%d)\n", CLowV[i]);
                }
                else
                {
                    SBPL_FPRINTF(fDeb, "bestpred was NOT modified - old one is better\n");
                }

            }

            //SBPL_PRINTF("%d successors were not closed\n", notclosed);
        }//else


		//recompute minkey
		minkey = pSearchStateSpace->OPEN->getminkeyheap();

		//recompute goalkey if necessary
		goalkey = ComputeKey(searchgoalstate);

		if(expands%10 == 0 && expands > 0)
		{
			SBPL_PRINTF("high-level expands so far=%u\n", expands);
		}

	}
    SBPL_PRINTF("main loop done\n");
    SBPL_FPRINTF(fDeb, "main loop done\n");

	int retv = 1;
	if(searchgoalstate->g == INFINITECOST && pSearchStateSpace->OPEN->emptyheap())
	{
		SBPL_PRINTF("solution does not exist: search exited because heap is empty\n");
		retv = 0;
	}
	else if(!pSearchStateSpace->OPEN->emptyheap() && goalkey > minkey)
	{
		SBPL_PRINTF("search exited because it ran out of time\n");
		retv = 2;
	}
	else if(searchgoalstate->g == INFINITECOST && !pSearchStateSpace->OPEN->emptyheap())
	{
		SBPL_PRINTF("solution does not exist: search exited because all candidates for expansion have infinite heuristics\n");
		retv = 0;
	}
	else
	{
		SBPL_PRINTF("search exited with a solution for eps=%.3f\n", pSearchStateSpace->eps);
		retv = 1;
	}

	SBPL_FPRINTF(fDeb, "high-level expanded=%d\n", expands);

	highlevel_searchexpands += expands;

	return retv;		
}

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

//note this does NOT re-compute heuristics, only re-orders OPEN list based on current eps and h-vals
void RSTARPlanner::Reevaluatefvals(){ // M�todo chamado na linha 1327 pelo m�todo Search
	// M�todo respons�vel em reordenar a lista aberta considerando  os valores de E e H
	CKey key;
	int i;
	CHeap* pheap = pSearchStateSpace->OPEN; // Copia a lista aberta para a variavel pheap
	
	//re-compute priorities for states in OPEN and reorder it
	for (i = 1; i <= pheap->currentsize; ++i) // Recalcula as prioridades dos estados em ABERTO e reordena os mesmos
	  {
		RSTARState* state = (RSTARState*)pheap->heap[i].heapstate; // Pegue todos os estados da lista aberta
		pheap->heap[i].key = ComputeKey(state); // Calcule a Key do estado atual
	  }
	pheap->makeheap(); // Ap�s calcular todos os estados executar o m�todo Makeheap

	pSearchStateSpace->bReevaluatefvals = false; // Volta a variavel bReevalueatefvals para false
}


// ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

//creates (allocates memory) search state space
//does not initialize search statespace
int RSTARPlanner::CreateSearchStateSpace() { // M�todo chamado pelo construtor na linha 41


	//create a heap
	pSearchStateSpace->OPEN = new CHeap; // Cria a lista aberta
	MaxMemoryCounter += sizeof(CHeap); // Atualiza a m�moria
	//pSearchStateSpace->inconslist = new CList;
	//MaxMemoryCounter += sizeof(CList);   // IGNORA NOVAMENTE AS LISTAS ICONS

	pSearchStateSpace->searchgoalstate = NULL; // Defini GOAL como nulo momentaneamente
	pSearchStateSpace->searchstartstate = NULL; // Defini START como nulo memomentaneamente

    pSearchStateSpace->bReinitializeSearchStateSpace = false; //Seta a variavel  como false informando que n�o reinicializar�  a busca agora 
	
	return 1;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//deallocates memory used by SearchStateSpace
void RSTARPlanner::DeleteSearchStateSpace() { // M�todo chamado pelo destrutor na linha 111

	if(pSearchStateSpace->OPEN != NULL) // Se existir lista ABERTA
	{
		pSearchStateSpace->OPEN->makeemptyheap(); //Limpe a lista
		delete pSearchStateSpace->OPEN; // Delete
		pSearchStateSpace->OPEN = NULL; // E coloque a mesma para nula
	}

	//if(pSearchStateSpace->inconslist != NULL)
	//{
	//	pSearchStateSpace->inconslist->makeemptylist(RSTAR_INCONS_LIST_ID);  // Ignora, N�O SEI o PORQUE, as listas Incons
	//	delete pSearchStateSpace->inconslist;
	//	pSearchStateSpace->inconslist = NULL;
	//}

	//delete the states themselves [ Exclui os pr�prios estados ]
	int iend = (int)pSearchStateSpace->searchMDP.StateArray.size(); // Pega o tamanho da lista MDP
	for(int i=0; i < iend; i++)
	{
		CMDPSTATE* state = pSearchStateSpace->searchMDP.StateArray[i];
		if(state != NULL && state->PlannerSpecificData != NULL){ // Se o estado da vez n�o for nulo e j� tiver PlannerSpecificData
			DeleteSearchStateData((RSTARState*)state->PlannerSpecificData); // M�todo sem implementa��o na linha 594
			delete (RSTARState*)state->PlannerSpecificData; // Delete o planner
			state->PlannerSpecificData = NULL; // Coloque o planner para nulo
		}
		if(state != NULL) // Se apenas existir o estado e n�o estiver setado o planner
		{
			for(int aind = 0; aind < (int)state->Actions.size(); aind++) 
			{
				if(state->Actions[aind]->PlannerSpecificData != NULL) // Revisar essa outra parte (Action,Planner ... )
				{
					DeleteSearchActionData((RSTARACTIONDATA*)state->Actions[aind]->PlannerSpecificData); // N�o faz nada esse m�todo
					delete (RSTARACTIONDATA*)state->Actions[aind]->PlannerSpecificData; // Deleta as informa��es especifica do planejador
					state->Actions[aind]->PlannerSpecificData = NULL; // Seta as mesmas como nulo
				}
			}//over actions
		}
	}

	pSearchStateSpace->searchMDP.Delete(); // Depois deleta o grafo GAMA
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


//reset properly search state space
//needs to be done before deleting states
int RSTARPlanner::ResetSearchStateSpace() { // [Ningu�m chama esse m�todo]

	pSearchStateSpace->OPEN->makeemptyheap(); // Seta o indice de todos os estados da pilha para 0 [ heap.cpp 152 ]
	//pSearchStateSpace->inconslist->makeemptylist(RSTAR_INCONS_LIST_ID); // IGNORA NOVAMENTE A LISTA INCONS
	return 1;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

//initialization before each search
void RSTARPlanner::ReInitializeSearchStateSpace() //[M�todo chamado na linha 1327 pelo m�todo Search]
{
	//increase callnumber
	pSearchStateSpace->callnumber++; // Incrementa o n�mero de chamadas do R*

	//reset iteration
	pSearchStateSpace->searchiteration = 0; // Como de costume ele � resetado a cada incremento do CallNumber
	pSearchStateSpace->bNewSearchIteration = true; // Seta a variavel bNewSearchIteration como true informando que � uma nova itera��o de busca para que o n�mero de iteracoes seja incrementado na linha 1391

#if DEBUG
    SBPL_FPRINTF(fDeb, "reinitializing search state-space (new call number=%d search iter=%d)\n", 
            pSearchStateSpace->callnumber,pSearchStateSpace->searchiteration );
#endif

	pSearchStateSpace->OPEN->makeemptyheap(); // Seta o indice de todos os estados da pilha ABERTO para 0
	//pSearchStateSpace->inconslist->makeemptylist(RSTAR_INCONS_LIST_ID); // NOVAMENTE IGNORA A LISTA INCOS

    //reset 
	pSearchStateSpace->eps = this->finitial_eps; // Reseta o valor inicial de eps para 5,0
    pSearchStateSpace->eps_satisfied = INFINITECOST; // e o de eps_satisfied para infinito

	//initialize start state
	RSTARState* startstateinfo = (RSTARState*)(pSearchStateSpace->searchstartstate->PlannerSpecificData); // Inicializa o estado inicial passando as informa��es do planejador RStar
	if(startstateinfo->callnumberaccessed != pSearchStateSpace->callnumber) //verifica se o numero de acessos � diferente do nu�mero de chamadas feitas, se sim reinicialize as informa��es de busca para atualizar essas informa�~eos
		ReInitializeSearchStateInfo(startstateinfo); // Executa o m�todo da linha 539 passando o estado atual, nele ser� resetado as configura��es padroes do estado
	
	startstateinfo->g = 0; // No m�todo ReInitializeSearchStateInfo o G � setado para infito e aqui resetado para 0


	//insert start state into the heap
	pSearchStateSpace->OPEN->insertheap(startstateinfo, ComputeKey(startstateinfo)); // Insere o estado inicial na lista de aberto, passando o estado e sua chave

    pSearchStateSpace->bReinitializeSearchStateSpace = false; // Volta o valor da variavel bReinitializeSearchStateSpace para falso
	pSearchStateSpace->bReevaluatefvals = false; // E informa que n�o h� necessidade de reavaliar os valores de F - Linha 981
}

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//very first initialization
int RSTARPlanner::InitializeSearchStateSpace()
{

	if(pSearchStateSpace->OPEN->currentsize != 0) 
//		|| pSearchStateSpace->inconslist->currentsize != 0)
	{
		SBPL_ERROR("ERROR in InitializeSearchStateSpace: OPEN or INCONS is not empty\n");
		throw new SBPL_Exception();
	}

	pSearchStateSpace->eps = this->finitial_eps; 
    pSearchStateSpace->eps_satisfied = INFINITECOST;
	pSearchStateSpace->searchiteration = 0;
	pSearchStateSpace->bNewSearchIteration = true;
	pSearchStateSpace->callnumber = 0;
	pSearchStateSpace->bReevaluatefvals = false;


	//create and set the search start state
	pSearchStateSpace->searchgoalstate = NULL;
	//pSearchStateSpace->searchstartstate = GetState(SearchStartStateID);
    pSearchStateSpace->searchstartstate = NULL;
	

    pSearchStateSpace->bReinitializeSearchStateSpace = true;

	return 1;

}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

int RSTARPlanner::SetSearchGoalState(int SearchGoalStateID)
{
	if(pSearchStateSpace->searchgoalstate == NULL || 
		pSearchStateSpace->searchgoalstate->StateID != SearchGoalStateID)
	{
		pSearchStateSpace->searchgoalstate = GetState(SearchGoalStateID);

		//should be new search iteration
		pSearchStateSpace->eps_satisfied = INFINITECOST;
		pSearchStateSpace->bNewSearchIteration = true;
		pSearchStateSpace->eps = this->finitial_eps;

		//recompute heuristic for the heap if heuristics are used
#if USE_HEUR
		for(int i = 0; i < (int)pSearchStateSpace->searchMDP.StateArray.size(); i++)
		{
			CMDPSTATE* MDPstate = pSearchStateSpace->searchMDP.StateArray[i];
			RSTARState* state = (RSTARState*)MDPstate->PlannerSpecificData;
			state->h = ComputeHeuristic(MDPstate);
		}
		
		pSearchStateSpace->bReevaluatefvals = true;
#endif
	}


	return 1;

}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

int RSTARPlanner::SetSearchStartState(int SearchStartStateID)
{

	CMDPSTATE* MDPstate = GetState(SearchStartStateID);

	if(MDPstate !=  pSearchStateSpace->searchstartstate)
	{	
		pSearchStateSpace->searchstartstate = MDPstate;
		pSearchStateSpace->bReinitializeSearchStateSpace = true;
		pSearchStateSpace->eps_satisfied = INFINITECOST;
	}

	return 1;

}

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void RSTARPlanner::PrintSearchState(RSTARState* state, FILE* fOut)
{
	SBPL_FPRINTF(fOut, "state %d: h=%d g=%u iterc=%d callnuma=%d heapind=%d \n",
		state->MDPstate->StateID, state->h, state->g, 
		state->iterationclosed, state->callnumberaccessed, state->heapindex);
	environment_->PrintState(state->MDPstate->StateID, true, fOut);

}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


int RSTARPlanner::getHeurValue(int StateID)
{
	CMDPSTATE* MDPstate = GetState(StateID);
	RSTARState* searchstateinfo = (RSTARState*)MDPstate->PlannerSpecificData;
	return searchstateinfo->h;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

vector<int> RSTARPlanner::GetSearchPath(int& solcost)
{

	vector<int> wholePathIds;
	RSTARState* rstargoalstate = (RSTARState*)pSearchStateSpace->searchgoalstate->PlannerSpecificData;
	
    //set the return path and other path related variables
    vector<CMDPACTION*> tempPathID;
	
	//initially no path
	solcost = INFINITECOST;
	wholePathIds.clear();

	//special case when we are already at the goal
	if(rstargoalstate->MDPstate == pSearchStateSpace->searchstartstate)
	{
		solcost = 0;
		return wholePathIds;
	}

    if(rstargoalstate->g >= INFINITECOST || rstargoalstate->bestpredaction == NULL || 
		((RSTARACTIONDATA*)rstargoalstate->bestpredaction->PlannerSpecificData)->pathIDs.size() == 0)
		return wholePathIds; //no path to goal state was found	
		
    //path exists
    int pathcost = 0;
	RSTARState* rstarstate = rstargoalstate;
    while(rstarstate->bestpredaction != NULL && rstarstate->MDPstate != pSearchStateSpace->searchstartstate)
    {
		//get action data
		RSTARACTIONDATA* bestpredactiondata = (RSTARACTIONDATA*)rstarstate->bestpredaction->PlannerSpecificData;

		//get predecessor in the search tree
        RSTARState* predstate = (RSTARState*)GetState(rstarstate->bestpredaction->SourceStateID)->PlannerSpecificData;

		//check validity
        if(predstate->g + bestpredactiondata->clow != rstarstate->g)
        {
            SBPL_ERROR("ERROR: clow(=%d) + predstate.g(=%d) = %d != succstate.g = %d (callnum=%d, iter=%d)\n",
                bestpredactiondata->clow, predstate->g, bestpredactiondata->clow + predstate->g, rstarstate->g,
                pSearchStateSpace->callnumber, pSearchStateSpace->searchiteration);
            SBPL_PRINTF("predstate: ");
            environment_->PrintState(predstate->MDPstate->StateID, true, stdout);
            SBPL_PRINTF("succstate: ");
            environment_->PrintState(rstarstate->MDPstate->StateID, true, stdout);
            SBPL_PRINTF("PredState: stateID=%d g=%d calln=%d iterc=%d h=%d\n",
                predstate->MDPstate->StateID, predstate->g, predstate->callnumberaccessed, predstate->iterationclosed, predstate->h);
            SBPL_PRINTF("Succstate: stateID=%d g=%d calln=%d iterc=%d h=%d\n",
                rstarstate->MDPstate->StateID, rstarstate->g, rstarstate->callnumberaccessed, rstarstate->iterationclosed, rstarstate->h);
            fflush(fDeb);
            
            throw new SBPL_Exception();
        }
		
		//store the action and its cost
        tempPathID.push_back(rstarstate->bestpredaction);
        pathcost += rstarstate->bestpredaction->Costs[0];

		//go to the predecessor
        rstarstate = predstate;

		//another check
        if(pathcost + rstarstate->g > rstargoalstate->g)
        {
            SBPL_ERROR("ERROR: pathcost+rstarstate.g = %d > goalstate.g = %d\n", pathcost + rstarstate->g, rstargoalstate->g);
            throw new SBPL_Exception();
        }
	}


    //now recover the actual path
	RSTARACTIONDATA* actiondata;
	for(int aind = 0; aind < (int)tempPathID.size(); aind++)
    {
		if(bforwardsearch)
			actiondata = (RSTARACTIONDATA*)tempPathID.at(tempPathID.size() - aind - 1)->PlannerSpecificData; //getting path in reverse
		else
			actiondata = (RSTARACTIONDATA*)tempPathID.at(aind)->PlannerSpecificData; //getting path in reverse

		//get the states that correspond to the high-level action
		for(int j = 0; j < (int)actiondata->pathIDs.size(); j++)
        {
            wholePathIds.push_back(actiondata->pathIDs.at(j)); //note: path corresponding to the action is already in right direction
        }
    }
    //add the goal state
    if(bforwardsearch)
        wholePathIds.push_back(rstargoalstate->MDPstate->StateID);
    else
        wholePathIds.push_back(pSearchStateSpace->searchstartstate->StateID);
        
    SBPL_FPRINTF(fDeb, "high-level pathcost=%d and high-level g(searchgoal)=%d\n", pathcost, rstargoalstate->g);

	//get the solcost
    solcost = pathcost;
	return wholePathIds;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void RSTARPlanner::PrintSearchPath(FILE* fOut)
{
	vector<int> pathIds;
	int solcost;

	pathIds = GetSearchPath(solcost);
	
	SBPL_FPRINTF(fOut, "high-level solution cost = %d, solution length=%d\n", solcost, (unsigned int)pathIds.size());
	for(int sind = 0; sind < (int)pathIds.size(); sind++)
	{
		environment_->PrintState(pathIds.at(sind), false, fOut);
	}

}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
bool RSTARPlanner::Search(vector<int>& pathIds, int & PathCost, bool bFirstSolution, bool bOptimalSolution, double MaxNumofSecs)
{
	CKey key;
	TimeStarted = clock();
    highlevel_searchexpands = 0;
    lowlevel_searchexpands = 0;

    bFirstSolution = true; //TODO-remove this but then fix crashing because later
    //searches within cycle re-initialize g-vals and test path found fails if last search ran out of time
    //so we need to save solutions between iterations
    //also, we need to call change callnumber before each search

#if DEBUG
	SBPL_FPRINTF(fDeb, "new search call (call number=%d)\n", pSearchStateSpace->callnumber);
#endif

    if(pSearchStateSpace->bReinitializeSearchStateSpace == true){
        //re-initialize state space 
        ReInitializeSearchStateSpace();
    }


	if(bOptimalSolution)
	{
		pSearchStateSpace->eps = 1;
		MaxNumofSecs = INFINITECOST;
	}
	else if(bFirstSolution)
	{
		MaxNumofSecs = INFINITECOST;
	}

	//get the size of environment that is already allocated
	int oldenvsize = environment_->StateID2IndexMapping.size()*sizeof(int);

	//the main loop of R*
	int prevexpands = 0;
	clock_t loop_time;
	//TODO - change FINAL_EPS and DECREASE_EPS onto a parameter
	while(pSearchStateSpace->eps_satisfied > RSTAR_FINAL_EPS && 
		(clock()- TimeStarted) < MaxNumofSecs*(double)CLOCKS_PER_SEC)
	{
        loop_time = clock();

		//decrease eps for all subsequent iterations
		if(fabs(pSearchStateSpace->eps_satisfied - pSearchStateSpace->eps) < ERR_EPS && !bFirstSolution)
		{
			pSearchStateSpace->eps = pSearchStateSpace->eps - RSTAR_DECREASE_EPS;
			if(pSearchStateSpace->eps < RSTAR_FINAL_EPS)
				pSearchStateSpace->eps = RSTAR_FINAL_EPS;

			//the priorities need to be updated
			pSearchStateSpace->bReevaluatefvals = true; 

			//it will be a new search. Since R* is non-incremental, it will have to be a new call
			pSearchStateSpace->bNewSearchIteration = true;
			pSearchStateSpace->bReinitializeSearchStateSpace = true;
		}

		//if(pSearchStateSpace->bReinitializeSearchStateSpace == true){
			//re-initialize state space 
			ReInitializeSearchStateSpace(); //TODO - we have to do it currently since g-vals from old searches are invalid
		//}

		if(pSearchStateSpace->bNewSearchIteration)
		{
			pSearchStateSpace->searchiteration++;
			pSearchStateSpace->bNewSearchIteration = false;
		}

		//re-compute f-values if necessary and reorder the heap
		if(pSearchStateSpace->bReevaluatefvals) 
			Reevaluatefvals();

		//improve or compute path
		if(ImprovePath(MaxNumofSecs) == 1){
            pSearchStateSpace->eps_satisfied = pSearchStateSpace->eps; //note: eps is satisfied probabilistically
        }

		//print the solution cost and eps bound
		SBPL_PRINTF("eps=%f highlevel expands=%d g(searchgoal)=%d time=%.3f\n", pSearchStateSpace->eps_satisfied, highlevel_searchexpands - prevexpands,
							((RSTARState*)pSearchStateSpace->searchgoalstate->PlannerSpecificData)->g,double(clock()-loop_time)/CLOCKS_PER_SEC);

#if DEBUG
        SBPL_FPRINTF(fDeb, "eps=%f highlevel expands=%d g(searchgoal)=%d time=%.3f\n", pSearchStateSpace->eps_satisfied, highlevel_searchexpands - prevexpands,
							((RSTARState*)pSearchStateSpace->searchgoalstate->PlannerSpecificData)->g,double(clock()-loop_time)/CLOCKS_PER_SEC);
		PrintSearchState((RSTARState*)pSearchStateSpace->searchgoalstate->PlannerSpecificData, fDeb);
#endif
		prevexpands = highlevel_searchexpands;


		//if just the first solution then we are done
		if(bFirstSolution)
			break;

		//no solution exists
		if(((RSTARState*)pSearchStateSpace->searchgoalstate->PlannerSpecificData)->g == INFINITECOST)
			break;
	}


#if DEBUG
	SBPL_FFLUSH(fDeb);
#endif

	PathCost = ((RSTARState*)pSearchStateSpace->searchgoalstate->PlannerSpecificData)->g;
	if(PathCost == INFINITECOST || ((RSTARACTIONDATA*)((RSTARState*)pSearchStateSpace->searchgoalstate->PlannerSpecificData)->bestpredaction->PlannerSpecificData)->pathIDs.size() == 0)
		PathCost = INFINITECOST; //the path to the goal is not found, it is just goal has been generated but the last edge to it wasn't computed yet

	MaxMemoryCounter += (oldenvsize - environment_->StateID2IndexMapping.size()*sizeof(int));
	
	SBPL_PRINTF("MaxMemoryCounter = %d\n", MaxMemoryCounter); // d� negativo

	int solcost = INFINITECOST;
	bool ret = false;
	if(PathCost == INFINITECOST)
	{
		SBPL_PRINTF("could not find a solution\n");
		ret = false;
	}
	else
	{
		SBPL_PRINTF("solution is found\n");      
    	pathIds = GetSearchPath(solcost);
        ret = true;
	}

	SBPL_PRINTF("total highlevel expands this call = %d, planning time = %.3f secs, solution cost=%d\n", 
           highlevel_searchexpands, (clock()-TimeStarted)/((double)CLOCKS_PER_SEC), solcost);
    

    //SBPL_FPRINTF(fStat, "%d %d\n", highlevel_searchexpands, solcost);

	return ret;

}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

//-----------------------------Interface function-----------------------------------------------------
//returns 1 if found a solution, and 0 otherwise
int RSTARPlanner::replan(double allocated_time_secs, vector<int>* solution_stateIDs_V)
{
	int solcost;

	return replan(allocated_time_secs, solution_stateIDs_V, &solcost);
	
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//returns 1 if found a solution, and 0 otherwise
int RSTARPlanner::replan(double allocated_time_secs, vector<int>* solution_stateIDs_V, int* psolcost)
{
  vector<int> pathIds; 
  bool bFound = false;
  int PathCost;
  bool bFirstSolution = this->bsearchuntilfirstsolution;
  bool bOptimalSolution = false;
  *psolcost = 0;
  
  SBPL_PRINTF("planner: replan called (bFirstSol=%d, bOptSol=%d)\n", bFirstSolution, bOptimalSolution);
  
  //plan
  if((bFound = Search(pathIds, PathCost, bFirstSolution, bOptimalSolution, allocated_time_secs)) == false) 
    {
      SBPL_PRINTF("failed to find a solution\n");
    }
  
  //copy the solution
  *solution_stateIDs_V = pathIds;
  *psolcost = PathCost;
  
  return (int)bFound;

}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

int RSTARPlanner::set_goal(int goal_stateID)
{

	SBPL_PRINTF("planner: setting goal to %d\n", goal_stateID);
	environment_->PrintState(goal_stateID, true, stdout);

	if(bforwardsearch)
	{	
		if(SetSearchGoalState(goal_stateID) != 1)
			{
				SBPL_ERROR("ERROR: failed to set search goal state\n");
				return 0;
			}
	}
	else
	{
	    if(SetSearchStartState(goal_stateID) != 1)
        {
            SBPL_ERROR("ERROR: failed to set search start state\n");
            return 0;
        }
	}

    return 1;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

int RSTARPlanner::set_start(int start_stateID)
{

	SBPL_PRINTF("planner: setting start to %d\n", start_stateID);
	environment_->PrintState(start_stateID, true, stdout);

	if(bforwardsearch)
	{	

	    if(SetSearchStartState(start_stateID) != 1)
        {
            SBPL_ERROR("ERROR: failed to set search start state\n");
            return 0;
        }
	}
	else
	{
	    if(SetSearchGoalState(start_stateID) != 1)
        {
            SBPL_ERROR("ERROR: failed to set search goal state\n");
            return 0;
        }
	}

    return 1;

}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


void RSTARPlanner::costs_changed(StateChangeQuery const & stateChange)
{
	//since R* is non-incremental
    pSearchStateSpace->bReinitializeSearchStateSpace = true;

}

void RSTARPlanner::costs_changed()
{
	//since R* is non-incremental
    pSearchStateSpace->bReinitializeSearchStateSpace = true;

}

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

int RSTARPlanner::force_planning_from_scratch()
{
	SBPL_PRINTF("planner: forceplanfromscratch set\n");

    pSearchStateSpace->bReinitializeSearchStateSpace = true;

    return 1;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

int RSTARPlanner::set_search_mode(bool bSearchUntilFirstSolution)
{

	SBPL_PRINTF("planner: search mode set to %d\n", bSearchUntilFirstSolution);

	bsearchuntilfirstsolution = bSearchUntilFirstSolution;

	return 1;
}


void RSTARPlanner::print_searchpath(FILE* fOut)
{
	PrintSearchPath(fOut);
}


//---------------------------------------------------------------------------------------------------------

