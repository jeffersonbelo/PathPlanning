unit RStar;

{$mode objfpc}{$H+}


interface

uses
  Classes, SysUtils,HeapRstar;

type
   TipoAmbiente = Record   {Será setado posteriormente, conforme as configurações do ambiente dinâmico}
end;

type
    {tCMDPAction - MDP.h - 43 - OK}
    tCMDPAction = Class
    var
       ActionID: integer;
       sourceStateID:integer;
       SuccsID: array of integer;
       Costs: array of integer;
       SuccsProb: array of Single;   {vector<float> SuccsProb;}
       PlannerSpecificData:Pointer;  {Pointer = Void* in C++}
   public
       constructor create(ID:integer; _sourcestateid:integer);
       procedure addOutCome(OutcomeStateID:integer; OutcomeCost:integer; OutcomeProb: single);

end;

type
    pCMDPAction = ^tCMDPAction; {Criando um ponteiro para tCMDPAction}
    {tCMDPState - MDP.h - 91 - OK}
    tCMDPState = Class
    var
       StateID:integer;
       Actions: array of pCMDPAction; {vector<CMDPACTION*> Actions;}
       PredsID: array of integer;
       PlannerSpecificData:Pointer; {Pointer = Void*}
    public
       constructor create(ID:integer);

       function addAction(ID:integer): pCMDPAction;
end;

type
    {brief low-level (local) search state in R*}
    {TRstarState - RStarPlanner.h = 60 - OK}
    pCMDPState = ^tCMDPState;  {Criando um ponteiro para tCMDPState}
    TRstarState = Class(tAbstractSearchState)
    var
       MDPState: pCMDPState;   {CMDPSTATE* MDPstate; }
       g:Cardinal;   {unsigned int g;}
       iterationClosed:word;{short unsigned int iterationclosed;}
       callnumberaccessed:word;{short unsigned int callnumberaccessed;}
       bestPredAction: pCMDPAction; {CMDPSTATE *bestpredstate; O ponteiro está na variavel }
       predActionV: array of pCMDPAction;  {vector<CMDPACTION*> predactionV;}
       h:integer;
       //heapindex:integer;    {atributo abstrato - Proveniente de Planner.h}
    public
       constructor create();
end;

type
    pRStarState = ^TRstarState;  {Criando um ponteiro para tRStarState}
    {MDP.h - 135 - OK}
    tCMDP = Class
    var
       StateArray: array of pCMDPState; {vector<CMDPSTATE*>}
    public
       function addState(StateID:integer):pCMDPState;  {CMDPSTATE* AddState(int StateID);}

       function delete():boolean;
end;
type
    {brief action in Gamma graph. Each action in Gamma graph corresponds to a path}
    {RStarActionData - RStarPlanner.h - 100 - OK}
    tRstarActionData = class (tAbstractSearchState)
       clow:integer;
       exp:integer;
       pathIDs: array of integer;
end;

type
    pRstarActionData = ^tRstarActionData;
    {brief low-level (local) search state in R*}
    {RStarLSearchStateData - RStarPlanner.h - 118 - OK}
    tRStarLSearchState = Class (tAbstractSearchState)
       MDPState: pCMDPState;
       g:integer;
       iteration: Cardinal; {unsigned int}
       iterationClosed: Cardinal;
       bestPredState: pCMDPState;
       bestPredStateActionCost: integer;
end;
type
    pRStarLSearchState = ^tRStarLSearchState;
     pCHeap = ^tCHeap;
    {brief local search statespace in R*}
    {RStarLSearchStateData - RStarPlanner.h - 151 - OK}
    tRstarLSearchStateSpace = Class
       MDP: tCMDP;
       StartState:pCMDPState;
       GoalState: pCMDPState;
       iteration:integer;
       OPEN: pCHeap;  {IMPLEMENTAR DEPOIS}
       //CList* INCONS;
    public
       constructor create();
end;


type
    {Pesquisa GLOBAL  - RStarPlanner.h - 191 - OK}
    tRstarSearchStateSpace = class
       eps:double;
       eps_satisfied:double;
       OPEN: pCHeap;
       searchIteration:word;  {short unsigned int searchiteration}
       callNumber:word;       {short unsigned int callnumber; }
       searchGoalState: pCMDPState;  {CMDPSTATE* searchgoalstate;}
       searchStartState: pCMDPState;  {CMDPSTATE* searchstartstate;}
       searchMDP:tCMDP;
       bReevaluateFvals:boolean;
       bReinitializeSearchStateSpace:boolean;
       bNewSearchIteration:boolean;
end;

type
    {Class Main}

    { tRstarPlanner }
    tIntegerArray = array of integer;
    tRstarPlanner = Class
    public

       constructor create(environment: Pointer; bSearchForward:boolean);   {Modificar depois o ambiente, para assim atender as especificidades do ambiente do simulador}

       function ComputeHeuristic(MDPState: pCMDPState):integer;
       function ComputeKey(rstarState:pRstarState):tCKey;
       function CreateLSearchState(stateID:integer): pCMDPState;
       function CreateSearchStateSpace(): integer;
       function CreateState(stateID:integer): pCMDPState;
       function DestroyLocalSearchMemory():boolean;
       function GetLSearchState(stateID:integer):pCMDPState;
       function GetState(StateID:integer): pCMDPState;
       function GetSearchPath(solCost: pInteger): tIntegerArray;
       function ImprovePath(MaxNumofSecs:double):integer;
       procedure InitializeSearchStateInfo(state:pRStarState);
       function InitializeSearchStateSpace():integer;
       procedure Initialize_rstarlsearchdata(state: pCMDPState);
       procedure Initialize_searchInfo(state: pCMDPState);
       function LocalSearchComputeKey(rstarlsearchState: pRStarLSearchState):tCkey;
       function replan(allocated_time_secs:double; solution_stateIDs_v: array of pInteger):integer;
       function replan(allocated_time_secs:double; solution_stateIDs_v: array of pInteger; pSolCost:pInteger):integer;
       procedure ReInitializeSearchStateInfo(state:pRStarState);
       procedure ReInitializeSearchStateSpace();
       procedure Reevaluatefvals();
       function Search(pathIds: tIntegerArray; PathCost: Integer; bFirstSolution:boolean; bOptimalSolution:boolean; MaxNumofSecs:double):boolean;
       function set_search_mode(_bSearchUntilFirstSolution:boolean):integer;
       function set_goal(goal_stateID:integer):integer;
       function set_start(start_stateID:integer):integer;
       procedure SetBestPredecessor(rstarState:pRstarState; rstarPredState:pRstarState; action:pCMDPAction);
       procedure set_initialsolution_eps(initialsolution_eps:double);
       function SetSearchStartState(SearchStartStateID:integer):integer;
       function SetSearchGoalState(SearchGoalStateID:integer):integer;










    private
    TimeStarted: TDateTime;
end;

const
     RSTAR_DEFAULT_INITIAL_EPS = 5.0;
     INFINITECOST =  1000000000;
     MAXSTATESPACESIZE = 20000000;
     RSTAR_FINAL_EPS = 1.0;
     RSTAR_DECREASE_EPS = 0.2;
     ERR_EPS = 0.0000001;
     RSTAR_EXPTHRESH  = 150;
     HEAPSIZE_INIT = 5000;

var
     bForwardSearch:boolean;
     environment_:pointer; {Tipo do ambiente que é um ponteiro do ambiente dinamico}
     bSearchUntilFirstSolution:boolean;
     finitial_eps:double;
     highLevel_searchExpands:integer;
     lowLevel_searchExpands:integer;
     pSearchStateSpace:TRstarSearchStateSpace;
     pLSearchStateSpace:tRstarLSearchStateSpace;

implementation

{tRstarLSeachStateSpace - RStarPlanner.h - 175 - OK}
constructor tRstarLSearchStateSpace.create;
begin
   self.OPEN := Nil;
 // self.Incons = nil;
    self.StartState:= nil;
    self.GoalState := nil;
end;

{tRstarState - RStarPlanner.h - 93 - OK }
constructor TRstarState.create;
begin
  {NADA PARA COLOCAR AQUI, PROVENIENTE DE UMA ABSTRACT CLASS}
end;

{Construtor - tCMDPState - MDP.h - 102 - OK }
constructor tCMDPState.create(ID: integer);
begin
     StateID:=ID;
     PlannerSpecificData:=nil;
end;

function tCMDPState.addAction(ID: integer): pCMDPAction;
var
   action: pCMDPAction;
begin
   action^:= tCMDPAction.create(ID, self.StateID);
   //Actions.push_back(action);
     SetLength(Actions, Length(Actions)+1);
     Actions[High(Actions)]:=action;
   result:= action;

end;

{ Construtor - tCMDPAction - MDP.h - 57 - OK}
constructor tCMDPAction.create(ID:integer;_sourcestateid:integer);
begin
     self.ActionID := ID;
     self.SourceStateID := _sourcestateid;
     self.PlannerSpecificData := Nil;
end;

procedure tCMDPAction.addOutCome(OutcomeStateID: integer; OutcomeCost: integer;
  OutcomeProb: single);
begin
   {SuccsID.push_back(OutcomeStateID);}
    SetLength(SuccsID, Length(SuccsID)+1);
    SuccsID[High(SuccsID)]:=OutcomeStateID;

   {Costs.push_back(OutcomeCost);}
    SetLength(Costs, Length(Costs)+1);
    Costs[High(Costs)]:=OutcomeCost;

   {SuccsProb.push_back(OutcomeProb);}
    SetLength(SuccsProb, Length(SuccsProb)+1);
    SuccsProb[High(SuccsProb)]:=OutcomeProb;
end;

{Construtor - TRstarPlanner.cpp - 41 - OK}
constructor tRstarPlanner.create(environment: Pointer; bSearchForward: boolean);
begin
     bForwardSearch:=bSearchForward; {Se True a pesquisa é FORWARD, se falso BACKWARD}
     environment_:=environment;
     bSearchUntilFirstSolution:=false;   {Se TRUE, pesquisará apenas até encontrar a primeira solução e pronto (pára), caso FALSE, continuará pesquisando mesmo após encontrar a primeira solução.}
     finitial_eps:= RSTAR_DEFAULT_INITIAL_EPS;
     highLevel_searchExpands:=0;
     lowLevel_searchExpands:=0;

     //create global searchstatespace
     pSearchStateSpace := tRstarSearchStateSpace.create();

     //create local searchstatespace
     pLSearchStateSpace := TRstarLSearchStateSpace.Create();

     //create the RSTAR planner
     if(CreateSearchStateSpace()<>1) then
         //ShowMessage('Failed to create stateSpace');

     //set the start and goal states
     if(InitializeSearchStateSpace()<> 1) then
         //ShowMessage('Failed to create stateSpace');
end;



{does not initialize search state space - RStarPlanner 963 - OK}
function tRstarPlanner.CreateSearchStateSpace: integer;
begin

     //create a heap
     pSearchStateSpace.open^ := tCheap.create();

     pSearchStateSpace.searchGoalState:=nil;
     pSearchStateSpace.searchStartState:=nil;

     pSearchStateSpace.bReinitializeSearchStateSpace:=false;

     result:=1;
end;

{first initialization - RStarPlanner - 1076 - OK}
function tRstarPlanner.InitializeSearchStateSpace: integer;
begin

     if(pSearchStateSpace.OPEN^.currentsize <> 0) then
         { ShowMessage('ERROR in InitializeSearchStateSpace: OPEN is not empty');}

     pSearchStateSpace.eps := finitial_eps;
     pSearchStateSpace.eps_satisfied := INFINITECOST;
     pSearchStateSpace.searchIteration := 0;
     pSearchStateSpace.bNewSearchIteration := true;
     pSearchStateSpace.callNumber := 0;
     pSearchStateSpace.bReevaluateFvals := false;

     {create and set the search start state}
     pSearchStateSpace.searchGoalState:=nil;
     pSearchStateSpace.searchStartState:=nil;

     pSearchStateSpace.bReinitializeSearchStateSpace := true;

     result:=1;
end;

{RstarPlanner - 1545 - OK}
function tRstarPlanner.set_search_mode(_bSearchUntilFirstSolution: boolean
  ): integer;
begin

     bsearchuntilfirstsolution := _bSearchUntilFirstSolution;
     result:=1;
end;

{RStarPlanner - 1491 - OK}
function tRstarPlanner.set_start(start_stateID: integer): integer;
begin
     if (bforwardsearch) then
          if(SetSearchStartState(start_stateID) <> 1) then
               result:= 0
     else
     if(SetSearchGoalState(start_stateID) <> 1) then
          result:=0;     {backward}
     result:=1;
end;

{RStarPlanner - 1138 - OK}
function tRstarPlanner.SetSearchStartState(SearchStartStateID: integer): integer;
var
MDPstate: pCMDPState;
begin

     MDPState := GetState(SearchStartStateID);

     if(MDPstate <> pSearchStateSpace.searchstartstate)then
     begin
          pSearchStateSpace.searchStartState := MDPstate;
          pSearchStateSpace.bReinitializeSearchStateSpace := true;
          pSearchStateSpace.eps_satisfied := INFINITECOST;
     end;

     result:=1;
end;

{RStarPlanner.cpp - 1107 -  OK}
function tRstarPlanner.SetSearchGoalState(SearchGoalStateID: integer): integer;
var
   i:integer;
   MDPState: pCMDPState;
   state: pRStarState;
begin

    if ((pSearchStateSpace.searchGoalState = nil) OR
(pSearchStateSpace.searchGoalState^.StateID <> SearchGoalStateID) ) then
   begin

      pSearchStateSpace.searchGoalState := GetState(SearchGoalStateID);

      //should be new search iteration
      pSearchStateSpace.eps_satisfied:=INFINITECOST;
      pSearchStateSpace.bNewSearchIteration:=true;
      pSearchStateSpace.eps:= finitial_eps;

      //recompute heuristic for the heap if heuristics are used
      for i:=0 to Length(pSearchStateSpace.searchMDP.StateArray)  do
      begin
           MDPState:= pSearchStateSpace.searchMDP.StateArray[i];
           state := pRStarState(MDPState^.PlannerSpecificData);
           state^.h:= ComputeHeuristic(MDPState);
      end;
      pSearchStateSpace.bReevaluateFvals:=true;
   end;

   Result:=1;

end;

{RStarPlanner.cpp - 149 - OK}
function tRstarPlanner.GetState(StateID: integer): pCMDPState; {^tCMDPState}
begin
     if ({Apagar o true}true {stateID >= (int)environment_->StateID2IndexMapping.size()}) then
           //ShowMessage('ERRO int GetState: stateID');
     if ({Apagar o true}true {environment_->StateID2IndexMapping[stateID][RSTARMDP_STATEID2IND] == -1}) then
           result := createState(stateID)
     else
           result:= pSearchStateSpace.searchMDP.StateArray[{[environment_->StateID2IndexMapping[stateID][RSTARMDP_STATEID2IND]}{apagar o 0}0];
end;

{RStarPlanner - 110 - OK}
function tRstarPlanner.CreateState(stateID: integer): pCMDPState; {^tCMDPState}
var
     state: pCMDPState;
begin
      if({environment_->StateID2IndexMapping[stateID][RSTARMDP_STATEID2IND] != -1}true) then
	{SBPL_ERROR("ERROR in CreateState: state already created\n"); throw new SBPL_Exception();}

      //adds to the tail a state
      state := pSearchStateSpace.searchMDP.addState(stateID);

      //remember the index of the state
      {environment_->StateID2IndexMapping[stateID][RSTARMDP_STATEID2IND] = pSearchStateSpace->searchMDP.StateArray.size()-1;}

      //create search specific info
      state^.PlannerSpecificData := TRstarState.create(); {Inicializando a classe abstrata}
      Initialize_searchInfo(state);

      result:= state;
end;

{tCMDP - AddState = MDP.cpp - 294 - OK}
function tCMDP.addState(StateID: integer): pCMDPState;
var
state: pCMDPState;
begin
     if( integer(Length(StateArray) + 1) > MAXSTATESPACESIZE ) then
         {ShowMessage('ERROR: Maximum of states is reached in MDP'); Lança alguma exceção para tratar ( Se for o caso )}

     state^ := tCMDPState.create(StateID);

     {Implementação da função "StateArray.push_back(state)";}
     SetLength(StateArray, Length(StateArray)+1);  {aumenta o tamanho do vetor}
     StateArray[High(StateArray)] := state;        {adciona state no mesmo, na calda}

     result:= state;
end;

function tCMDP.delete: boolean;
var
   state: pCMDPState;
begin
   while( Length(StateArray) > 0) do
   begin
     state := StateArray[High(StateArray)];
     StateArray[Length(StateArray)-1] := nil;
     SetLength(StateArray, Length(StateArray)-1);
     state:=nil;
   end;
   result:=true;
end;

{RStarPlanner - 100 - OK}
procedure tRstarPlanner.Initialize_searchInfo(state: pCMDPState);{tCMDPState}
var
     searchStateInfo: pRStarState;
begin
     searchStateInfo := pRStarState(state^.PlannerSpecificData);
     searchStateInfo^.MDPState := state;

     InitializeSearchStateInfo(searchstateinfo);
end;

{RStarPlanner - 475 - OK}
procedure tRstarPlanner.InitializeSearchStateInfo(state: pRStarState); {TRstarState}
begin
     state^.g:=INFINITECOST;
     state^.iterationClosed:=0;
     state^.callnumberaccessed:=pSearchStateSpace.callNumber;
     state^.heapindex :=0;
     state^.bestPredAction:=nil;

     if (pSearchStateSpace.searchGoalState <> nil) then
          state^.h:= ComputeHeuristic(state^.MDPState)
     else
          state^.h:=0;

     {Implementação da função "state^.predActionV.clear()"}
     {Fillchar (array,tamanho, elemento)}
     FillChar(state^.predActionV,Length(state^.predActionV),0);
end;

procedure tRstarPlanner.Initialize_rstarlsearchdata(state: pCMDPState);
var
     rstarlsearch_data: pRStarLSearchState;
begin
   rstarlsearch_data := pRStarLSearchState(state^.PlannerSpecificData); // Passa as informaçoes do planejador para a variavel RStarLSearch_data

   rstarlsearch_data^.bestPredState := nil;
   rstarlsearch_data^.bestPredStateActionCost := 0;
   rstarlsearch_data^.iteration := 0;
   rstarlsearch_data^.iterationClosed := 0;
   rstarlsearch_data^.g := INFINITECOST;
   rstarlsearch_data^.heapindex := 0;
   rstarlsearch_data^.listElem[0] := Nil; {erro -  Não está querendo receber NULL}
   rstarlsearch_data^.listelem[1] := Nil; {erro -  Não está querendo receber NULL}

    //pointer to itself
    rstarlsearch_data^.MDPState := state; //Seta o grafo MDP do estado com o estado passado por parametro
end;

{RStarPlanner - 441}
function tRstarPlanner.ComputeHeuristic(MDPState: pCMDPState): integer;
begin

     {compute heuristic for search}
     if(pSearchStateSpace.searchGoalState = nil) then
          result:= 0;
     if(bForwardSearch) then
     begin
          {result:= environment_->GetFromToHeuristic(MDPstate->StateID, pSearchStateSpace->searchgoalstate->StateID);}
     end
     else
          {backward search: heur = distance from searchgoal to state}
          {result:= environment_->GetFromToHeuristic(pSearchStateSpace->searchgoalstate->StateID, MDPstate->StateID);}
end;

{RStarPlanner - 1463 - OK}
function tRstarPlanner.set_goal(goal_stateID: integer): integer;
begin
     if (bforwardsearch) then
          if(SetSearchGoalState(goal_stateID) <> 1) then
               result:= 0
     else
          if(SetSearchStartState(goal_stateID) <> 1) then
               result:=0;     {backward}
     result:=1;
end;

{RStarPlanner - 1428 - OK}
function tRstarPlanner.replan(allocated_time_secs: double;
  solution_stateIDs_v: array of pInteger): integer;
var
     solcost: integer;
begin
     result:= replan(allocated_time_secs, solution_stateIDs_v, @solcost);
end;

{RStarPlanner - 1437 - Return 1 if found a solution and 0 otherwise - OK}
function tRstarPlanner.replan(allocated_time_secs: double;
  solution_stateIDs_v: array of pInteger; pSolCost: pInteger): integer;
var
     pathIDs: array of integer;
     PathCost,i:integer;
     bFirstSolution, bOptimalSolution, bFound : boolean;

begin
     bFound:=false;
     bFirstSolution:=bSearchUntilFirstSolution;
     bOptimalSolution:=false;
     pSolCost^:=0;

     if( (bFound = Search(pathIds, PathCost, bFirstSolution, bOptimalSolution, allocated_time_secs)=false )) then
          // writeln('Failed to find a solution');

     //copy the solution
     for i:=0 to Length(pathIDs) do;
     begin
          solution_stateIDs_v[i]^:=pathIDs[i];
     end;

     pSolCost^ := PathCost;

     result:= integer(bFound);
end;

{RStarPlanner - 1288 - OK}
function tRstarPlanner.Search(pathIds: array of Integer; PathCost: Integer;
  bFirstSolution: boolean; bOptimalSolution: boolean; MaxNumofSecs: double
  ): boolean;

var
    key:tCKey;
    loop_time:TDateTime; //Clock()   {verificar depois se clock também tem em Delphi}
    solcost,highlevel_searchexpands, lowlevel_searchexpands, oldenvsize, prevexpands :integer;
    ret:boolean;
begin

     Self.TimeStarted:=Now;
     highlevel_searchexpands := 0;
     lowlevel_searchexpands := 0;
     bFirstSolution := true;

     if(pSearchStateSpace.bReinitializeSearchStateSpace = true) then
          ReInitializeSearchStateSpace();   //re-initialize state space
     if(bOptimalSolution) then
     begin
          pSearchStateSpace.eps := 1;
          MaxNumofSecs := INFINITECOST;
     end
     else if(bFirstSolution) then
          MaxNumofSecs := INFINITECOST;

     //get the size of environment that is already allocated
     {oldenvsize = environment_->StateID2IndexMapping.size()*sizeof(int);}

     //the main loop of R*
     prevexpands := 0;
     //TODO - change FINAL_EPS and DECREASE_EPS onto a parameter
     while( (pSearchStateSpace.eps_satisfied > RSTAR_FINAL_EPS) AND
     ((Now - TimeStarted) < MaxNumofSecs {*double(CLOCKS_PER_SEC)})) do
     begin
        loop_time := Now;  // clock()

        //decrease eps for all subsequent iterations
        if (((abs(pSearchStateSpace.eps_satisfied - pSearchStateSpace.eps)) < ERR_EPS) and not(bFirstSolution)) then
        begin
             pSearchStateSpace.eps := pSearchStateSpace.eps - RSTAR_DECREASE_EPS;
             if(pSearchStateSpace.eps < RSTAR_FINAL_EPS) then
                  pSearchStateSpace.eps := RSTAR_FINAL_EPS;

             //the priorities need to be updated
             pSearchStateSpace.bReevaluatefvals := true;

             //it will be a new search. Since R* is non-incremental, it will have to be a new call
             pSearchStateSpace.bNewSearchIteration := true;
             pSearchStateSpace.bReinitializeSearchStateSpace := true;
        end;

        ReInitializeSearchStateSpace(); //TODO - we have to do it currently since g-vals from old searches are invalid

        if(pSearchStateSpace.bNewSearchIteration) then
        begin
           pSearchStateSpace.searchiteration:= pSearchStateSpace.searchiteration+1;
           pSearchStateSpace.bNewSearchIteration := false;
        end;

        //re-compute f-values if necessary and reorder the heap
        if(pSearchStateSpace.bReevaluatefvals) then
           Reevaluatefvals();

        //improve or compute path
        if(ImprovePath(MaxNumofSecs) = 1) then
           pSearchStateSpace.eps_satisfied := pSearchStateSpace.eps; //note: eps is satisfied probabilistically

        prevexpands := highlevel_searchexpands;

        //if just the first solution then we are done
        if(bFirstSolution) then
           Break;
        //no solution exists
        if( pRStarState(pSearchStateSpace.searchgoalstate^.PlannerSpecificData)^.g = INFINITECOST) then
           break;

     end;   // end while

     PathCost := pRstarState(pSearchStateSpace.searchgoalstate^.PlannerSpecificData)^.g;

     if((PathCost=INFINITECOST) OR
     (Length(pRstarActionData(pRstarState(pSearchStateSpace.searchgoalstate^.PlannerSpecificData)^.bestPredAction^.PlannerSpecificData)^.pathIDs) = 0) ) then
        PathCost := INFINITECOST; //the path to the goal is not found, it is just goal has been generated but the last edge to it wasn't computed yet

     solcost := INFINITECOST;
     ret := false;
     if(PathCost = INFINITECOST) then
       //SBPL_PRINTF("could not find a solution\n");
       ret := false
     else
     begin  //SBPL_PRINTF("solution is found\n");
       //pathIds := GetSearchPath(@solcost);
       {Coloco uma variavel para receber o resultado, faco um for e copio}
       ret := true;
     end;
     //SBPL_FPRINTF(fStat, "%d %d\n", highlevel_searchexpands, solcost);
     result:=ret;
end;

{ RStarPlanner.cpp - 1038 - initialization before each search - OK}
procedure tRstarPlanner.ReInitializeSearchStateSpace;
var
    startstateinfo: pRStarState;
begin

    //increase callnumber
    pSearchStateSpace.callnumber:=pSearchStateSpace.callnumber +1;

    //reset iteration
    pSearchStateSpace.searchiteration := 0;
    pSearchStateSpace.bNewSearchIteration := true;

    pSearchStateSpace.OPEN^.makeemptyheap();

    //reset
    pSearchStateSpace.eps := finitial_eps;
    pSearchStateSpace.eps_satisfied := INFINITECOST;

    //initialize start state
    startstateinfo := pRStarState(pSearchStateSpace.searchstartstate^.PlannerSpecificData);
    if(startstateinfo^.callnumberaccessed <> pSearchStateSpace.callnumber) then
        ReInitializeSearchStateInfo(startstateinfo);

     startstateinfo^.g := 0;

     //insert start state into the heap
     pSearchStateSpace.OPEN^.insertheap(startstateinfo, ComputeKey(startstateinfo));

     pSearchStateSpace.bReinitializeSearchStateSpace := false;
     pSearchStateSpace.bReevaluatefvals := false;
end;

{RStarPlanner.cpp - 500 - OK}
procedure tRstarPlanner.ReInitializeSearchStateInfo(state: pRStarState); // arranjar um jeito de já setar o ponteiro por parametro
var
     i:integer;
begin
     state^.g := INFINITECOST;
     state^.iterationclosed := 0; // tem o comportamente muito parecido com o método InitializeSearchStateInfo 514, modificando apenas a partir da linha 565
     state^.callnumberaccessed := pSearchStateSpace.callnumber;
     state^.heapindex := 0;
     state^.bestpredaction := nil;

     //compute heuristics
     if(pSearchStateSpace.searchgoalstate <> nil) then
        state^.h:= ComputeHeuristic(state^.MDPstate)
     else
        state^.h := 0;

     {Implementando o método 'state->predactionV.clear();'}
     FillChar(state^.predActionV,Length(state^.predActionV),0);

     for i:=0 to Length(state^.MDPState^.Actions) do
           if(state^.MDPState^.Actions[i]^.PlannerSpecificData <> nil) then // Varre todo o grafo MDP escolhendo os estados com Informações do planejador já setadas
              state^.MDPstate^.Actions[i]^.PlannerSpecificData := nil; // seta a mesma para nula

     {state.MDPstate.RemoveAllActions(); // deleta o array de caminhos.}
     SetLength(state^.MDPState^.Actions, 0);
end;

procedure tRstarPlanner.Reevaluatefvals;
var
key: tCKey;
i:integer;
pheap: pCHeap;
state: ^TRstarState;
begin
  pheap := pSearchStateSpace.OPEN;
  //re-compute priorities for states in OPEN and reorder it
  for i := 1 to pheap^.currentsize do
  begin
       state := pRStarState(pheap^.heap[i].heapstate);
       pheap^.heap[i].key := ComputeKey(state);
  end;
  pheap^.makeheap();
  pSearchStateSpace.bReevaluatefvals := false;
end;

{RStarPlanner.cpp - 582}
function tRstarPlanner.ComputeKey(rstarState: pRstarState): tCKey; // [Método chamado na linha 610 pelo método SetBestPredecessor; Bem como na linha 654 pelo método ImprovePath; também na linha 981 pelo método Reevaluete e por fim na linha 1078 pelo método ReInitializeSearchStateSpace
var
      retkey:tCKey;
      h, starttostateh:integer;   // São criadas duas variaveis locais

begin

     if(bforwardsearch) then  // Se for FORWARD
        {h = environment_->GetFromToHeuristic(rstarState->MDPstate->StateID, pSearchStateSpace->searchgoalstate->StateID); // É calculado o H considerando a distancia Euclidiana do ponto atual até a meta e armazenado na variavel h
        starttostateh = environment_->GetFromToHeuristic(pSearchStateSpace->searchstartstate->StateID, rstarState->MDPstate->StateID); // Depois é calculado o H considerando a distancia Euclidiana do ponto inicial até o ponto atual e armazenado na variavel startTostateH

     else
        h = environment_->GetFromToHeuristic(pSearchStateSpace->searchgoalstate->StateID, rstarState->MDPstate->StateID); // É calculado o H considerando a distancia Euclidiana do ponto objetivo até o ponto atual e armazenado na variavel h
     	starttostateh = environment_->GetFromToHeuristic(rstarState->MDPstate->StateID, pSearchStateSpace->searchstartstate->StateID); // Depois é calculado o H considerando a distancia Euclidiana do ponto atual até o ponto inicial e armazenado na variavel startTostateH
     }

     //compute 2nd element of the key
     retkey.key[1] := rstarState^.g + Round(pSearchStateSpace.eps * h); // Pode-se considerar retkey.key[x] = as prioridade de K. // k[1, g(s) + w h(start, s) = AVOID

     //compute the 1st element
     if( (rstarState^.g > pSearchStateSpace.eps * starttostateh) OR // Se G(s) > w h(start,s) (1ª condição para evitar um estado)
     (rstarState^.bestpredaction <> nil) AND // No artigo vinha dizendo que era para ser =Null e não diferente
     (Length(pRstarActionData(rstarState^.bestpredaction^.PlannerSpecificData)^.pathIDs) = 0 )
     AND (pRstarActionData(rstarState^.bestpredaction^.PlannerSpecificData)^.exp >= RSTAR_EXPTHRESH)) then // Se expandir mais que 150 deve ser marcado como EVITAR
         retkey.key[0] := 1
     else
         retkey.key[0] := 0;

     result:=retkey;
end;
{returns 1 if the solution is found, 0 if the solution does not exist and 2 if it ran out of time}
{RStarPlanner.cpp - 614 - OK}
function tRstarPlanner.ImprovePath(MaxNumofSecs: double): integer; // Método chamado na linha 1327 pelo método Search
var
   expands, maxe, maxc,NewGoalStateID,hfromstarttostate,i,retv,highlevelexpands,notclosed:integer;
   rstarstate, searchgoalstate, searchstartstate,rstarpredstate,rstarNewTargetState: pRStarState;
   stateu, rstarSuccState: pRStarState;
   key, minkey, goalkey: tCKey;
   SuccIDV,ClowV: array of integer;
   computedaction, utosaction, predaction, action: pCMDPAction;
   computedactiondata: pRstarActionData;
   bSwitch:boolean;
   minQ:Cardinal;

begin
     highlevelexpands := 0;
     expands := 0;

     if(pSearchStateSpace.searchgoalstate = nil) then
     	{SBPL_ERROR("ERROR searching: no goal state is set\n");
     	throw new SBPL_Exception();}

     //goal state
     searchgoalstate := pRStarState(pSearchStateSpace.searchgoalstate^.PlannerSpecificData);
     if(searchgoalstate^.callnumberaccessed <> pSearchStateSpace.callnumber) then
     	ReInitializeSearchStateInfo(searchgoalstate);

     //get the start state
     searchstartstate := pRStarState(pSearchStateSpace.searchstartstate^.PlannerSpecificData);

     //set goal key
     if(searchgoalstate <> searchstartstate) then
     begin
        goalkey.key[0] := 1;
        goalkey.key[1] := INFINITECOST;
     end
     else
         goalkey := ComputeKey(searchstartstate);

     {expand states until done}
     minkey := pSearchStateSpace.OPEN^.getminkeyheap(); // Pega o primeiro estado do Heap e retorna (linha 305 - heap.cpp)

     while( not(pSearchStateSpace.OPEN^.emptyheap()) and ((NOW -TimeStarted) < MaxNumofSecs{*double(CLOCKS_PER_SEC)})) do  // Enquanto a lista aberta nao estiver vazia e o tempo de execução do algoritmo não ultrapassar o limite de 1000 segundos Continue. Obs timeStarted() é instanciado na linha 1331
     begin
         {recompute minkey}
         minkey := pSearchStateSpace.OPEN^.getminkeyheap();

         //recompute goalkey if necessary
         goalkey := ComputeKey(searchgoalstate);

         if (goalkey < minkey) then  // Sobrecarga de operadores
             Break;

         {pop the min element}
         rstarstate := pRStarState(pSearchStateSpace.OPEN^.deleteminheap());

         if ( (rstarstate^.MDPState <> pSearchStateSpace.searchstartstate) AND
         (Length(pRstarActionData(rstarstate^.bestPredAction^.PlannerSpecificData)^.pathIDs) = 0) ) then
         begin
            maxe := INFINITECOST;
            maxc := INFINITECOST;

            {predecessor}
            rstarpredstate := pRStarState(GetState(pCMDPAction(rstarstate^.bestPredAction^)^.sourceStateID)^.PlannerSpecificData);
            computedaction := pCMDPAction(rstarstate^.bestpredaction);      // Incluir dois CAST que não tinha no original
            computedactiondata := pRstarActionData(computedaction^.PlannerSpecificData);

            if(computedactiondata^.exp < RSTAR_EXPTHRESH) then
               maxe := RSTAR_EXPTHRESH
            else
                {Toda essa parte está comentada no código original}

                      { SBPL_PRINTF("Trying to compute hard-to-find path\n");
                      SBPL_FPRINTF(fDeb, "Trying to compute hard-to-find path\n");
                      /* TODO
                      CKey nextkey = rstarPlanner.OPEN->getminkeyheap();
     	              if(bforwardsearch)
     	              h = environment_->GetFromToHeuristic(rstarstate->MDPstate->StateID, pSearchStateSpace->GoalState->StateID);
     	              else
     	              h = environment_->GetFromToHeuristic(pSearchStateSpace->GoalState->StateID, rstarstate->MDPstate->StateID);
                      maxc = nextkey[1] -
                      rstarpredstate->g + rstarPlanner.epsilon*h) + RSTAR_COSTDELTA;
                      */
                      SBPL_FPRINTF(fDeb, "recomputing path from bp %d to state %d with maxc=%d maxe=%d\n",
                      rstarpredstate->MDPstate->StateID, rstarstate->MDPstate->StateID, maxc, maxe);
                      SBPL_FPRINTF(fDeb, "bp state:\n");
                      environment_->PrintState(rstarpredstate->MDPstate->StateID, true, fDeb);}

            //re-compute the path
            NewGoalStateID := rstarstate^.MDPState^.StateID;

           { ComputeLocalPath(rstarpredstate.MDPstate.StateID, rstarstate.MDPstate.StateID, maxc, maxe,
            @computedaction.Costs[0], @computedactiondata.clow, @computedactiondata.exp, @computedactiondata.pathIDs, @NewGoalStateID, MaxNumofSecs);
            }

            bSwitch := false;

            if(NewGoalStateID <> rstarstate^.MDPstate^.StateID) then
            begin
                 bSwitch := true;

                 rstarNewTargetState := pRStarState(GetState(NewGoalStateID)^.PlannerSpecificData);

                 //re-initialize the state if necessary
                 if(rstarNewTargetState^.callnumberaccessed <> pSearchStateSpace.callnumber) then
                     ReInitializeSearchStateInfo(rstarNewTargetState);

                 {add the successor to our graph}
                 action := rstarpredstate^.MDPstate^.AddAction(Length(rstarpredstate^.MDPState^.Actions)); // Falta implementar ADD ACTION
                 action^.AddOutcome(rstarNewTargetState^.MDPstate^.StateID, computedaction^.Costs[0], 1.0);
                 action^.PlannerSpecificData := tRstarActionData.Create; // CREATE

     	         pRstarActionData(action^.PlannerSpecificData)^.clow := computedactiondata^.clow;
                 pRstarActionData(action^.PlannerSpecificData)^.exp := computedactiondata^.exp;
                 pRstarActionData(action^.PlannerSpecificData)^.pathIDs := computedactiondata^.pathIDs;

                 //add the corresponding predaction
                // rstarNewTargetState.predactionV.push_back(action); Implementar o ADD
                   SetLength(rstarNewTargetState^.predActionV, Length(rstarNewTargetState^.predActionV)+1);
                   rstarNewTargetState^.predActionV[High(rstarNewTargetState^.predActionV)]:=action;

                 //the action was not found to the old state
                 computedaction^.Costs[0] := INFINITECOST;

                 if(bforwardsearch) then
                      //computedactiondata^.clow := environment_.GetFromToHeuristic(rstarpredstate.MDPstate.StateID, rstarstate.MDPstate.StateID)   Calcular a heuristica
     	         else
                 begin
                     //computedactiondata^.clow := environment_.GetFromToHeuristic(rstarstate.MDPstate.StateID, rstarpredstate.MDPstate.StateID);

                    //computedactiondata.pathIDs.clear(); // Depois resolver esse problema aqui
                        FillChar(computedactiondata^.pathIDs, Length(computedactiondata^.pathIDs),0);

                    rstarstate := rstarNewTargetState;
                    computedaction := action;
                    computedactiondata := pRstarActionData(action^.PlannerSpecificData);
                 end;
             end;

             //clean up local search memory
             DestroyLocalSearchMemory();

             stateu := nil;
             utosaction := nil;

             if(bforwardsearch) then
                // hfromstarttostate := environment_.GetFromToHeuristic(searchstartstate.MDPstate.StateID, rstarstate.MDPstate.StateID)
             else
                 //hfromstarttostate := environment_.GetFromToHeuristic(rstarstate.MDPstate.StateID, searchstartstate.MDPstate.StateID);
             if ( (Length(computedactiondata^.pathIDs)=0) OR
             (rstarpredstate^.g + computedactiondata^.clow > pSearchStateSpace.eps * hfromstarttostate)) then
             begin
                  //select other best predecessor
                  minQ := INFINITECOST;
                  for i := 0 to Length(rstarstate^.predActionV) do
                  begin
                       predaction := rstarstate^.predActionV[i];
                       rstarpredstate :=  pRStarState(GetState(predaction^.sourceStateID)^.PlannerSpecificData);

                       if(minQ >= rstarpredstate^.g + pRstarActionData(predaction^.PlannerSpecificData)^.clow) then
                       begin
                            minQ := rstarpredstate^.g + pRstarActionData(predaction^.PlannerSpecificData)^.clow;
                            stateu := rstarpredstate;
                            utosaction := predaction;
                       end;
                  end;

                  //set the predecessor
                  SetBestPredecessor(rstarstate, stateu, utosaction)
             end
             else if( (rstarpredstate^.g + computedactiondata^.clow < rstarstate^.g) OR (bSwitch = false) ) then
             begin
                  stateu := rstarpredstate;
                  utosaction := computedaction;

                  //set the predecessor
                  SetBestPredecessor(rstarstate, stateu, utosaction);
             end;
         end
         else
         begin

              highlevelexpands := highlevelexpands+1;

              //close the state
              rstarstate^.iterationClosed := pSearchStateSpace.searchiteration;

              //expansion
              expands:=expands+1;

              //generate SUCCS state
              //SuccIDV.clear(); // pensar depois como implementar isso
                 FillChar(SuccIDV, Length(SuccIDV),0);
              //CLowV.clear();
                 FillChar(ClowV, Length(ClowV),0);

     	      if(bforwardsearch) then
                  //  environment_.GetRandomSuccsatDistance(rstarstate.MDPstate.StateID, @SuccIDV, @CLowV);
     	      else
                  //  environment_.GetRandomPredsatDistance(rstarstate.MDPstate.StateID, @SuccIDV, @CLowV);

              //iterate over states in SUCCS set
              notclosed := 0;
              for i := 0 to Length(SuccIDV) do
              begin

                   rstarSuccState := pRStarState(GetState(SuccIDV[i])^.PlannerSpecificData);

                    //re-initialize the state if necessary
                    if(rstarSuccState^.callnumberaccessed <> pSearchStateSpace.callnumber) then
                        ReInitializeSearchStateInfo(rstarSuccState);

                    //skip if the state is already closed
                    if(rstarSuccState^.iterationclosed = pSearchStateSpace.searchiteration) then
                       continue;

                    notclosed:= notclosed+1;

                    //add the successor to our graph
                    action := rstarstate^.MDPstate^.AddAction(i);
                    action^.AddOutcome(rstarSuccState^.MDPstate^.StateID, INFINITECOST, 1.0);
                    action^.PlannerSpecificData := tRstarActionData.Create; // create

                    pRstarActionData(action^.PlannerSpecificData)^.clow := CLowV[i];
                    pRstarActionData(action^.PlannerSpecificData)^.exp := 0;

                    //tRstarActionData(action^.PlannerSpecificData).pathIDs.clear();
                       FillChar(pRstarActionData(action^.PlannerSpecificData)^.pathIDs, Length(pRstarActionData(action^.PlannerSpecificData)^.pathIDs),0);

                    //add the corresponding predaction
                    //rstarSuccState->predactionV.push_back(action);
                        SetLength(rstarSuccState^.predactionV, Length(rstarSuccState^.predactionV)+1);
                        rstarSuccState^.predactionV[High(rstarSuccState^.predactionV)]:=action;

                    //see if we can improve g-value of successor
                    if( (rstarSuccState^.bestPredAction = nil) OR
                    ( (rstarstate^.g + CLowV[i]) < rstarSuccState^.g) ) then

                        SetBestPredecessor(rstarSuccState, rstarstate, action)
                        //SBPL_FPRINTF(fDeb, "bestpred was set for the succ (clow=%d)\n", CLowV[i]);
                    else
                        {SBPL_FPRINTF(fDeb, "bestpred was NOT modified - old one is better\n");}
              end;
         end;

         //recompute minkey
         minkey := pSearchStateSpace.OPEN^.getminkeyheap();

         //recompute goalkey if necessary
         goalkey := ComputeKey(searchgoalstate);

         if( (expands mod 10 = 0) AND (expands > 0) ) then
     	     {SBPL_PRINTF("high-level expands so far=%u\n", expands); }

     end; //end while

     //SBPL_PRINTF("main loop done\n");
     //SBPL_FPRINTF(fDeb, "main loop done\n");

     retv := 1;

     if( (searchgoalstate^.g = INFINITECOST) AND (pSearchStateSpace.OPEN^.emptyheap())) then
         //SBPL_PRINTF("solution does not exist: search exited because heap is empty\n");
         retv := 0
     else if( not(pSearchStateSpace.OPEN^.emptyheap()) AND (goalkey > minkey) ) then // Sobrecarga de operadores
         //SBPL_PRINTF("search exited because it ran out of time\n");
         retv := 2
     else if( (searchgoalstate^.g = INFINITECOST) AND not(pSearchStateSpace.OPEN^.emptyheap()) ) then
          //SBPL_PRINTF("solution does not exist: search exited because all candidates for expansion have infinite heuristics\n");
          retv := 0
     else
          //SBPL_PRINTF("search exited with a solution for eps=%.3f\n", pSearchStateSpace->eps);
          retv := 1;

     highlevel_searchexpands := highlevel_searchexpands + expands;

     result:= retv;

end;

function tRstarPlanner.DestroyLocalSearchMemory: boolean;
var
   i:integer;
   state: pCMDPState;
   RstarLSearchStateData: pRStarLSearchState;
begin

  pLSearchStateSpace.OPEN^.currentsize:=0;
  pLSearchStateSpace.StartState:= nil;
  pLSearchStateSpace.GoalState:= nil;

  //remove the states in the MDP itself
  for i:=0 to Length(pLSearchStateSpace.MDP.StateArray) do;
  begin
    state := pLSearchStateSpace.MDP.StateArray[i];
    RstarLSearchStateData:= pRStarLSearchState(state^.PlannerSpecificData);
    RstarLSearchStateData^.Destroy;
    state^.PlannerSpecificData:= nil;
    //environment_->StateID2IndexMapping[state->StateID][RSTARMDP_LSEARCH_STATEID2IND] = -1;
  end;

  //now we can delete the states themselves
  if(pLSearchStateSpace.MDP.Delete() = false) then
    //SBPL_ERROR("ERROR: failed to delete local search MDP\n");

  result:= true;
end;

function tRstarPlanner.GetSearchPath(solCost: pInteger): tIntegerArray;
var
   wholePathIds: tIntegerArray;
   rstarGoalState, rstarState,predState: pRStarState;
   tempPathID: array of pCMDPAction;
   pathCost,aind,j:integer;
   bestpredactiondata, actiondata: pRstarActionData;
begin
  rstargoalstate := pRStarState(pSearchStateSpace.searchGoalState^.PlannerSpecificData);

  //initially no path
  solCost^:=INFINITECOST;
  {wholePathIds.clear();}
    FillChar(wholePathIds, Length(wholePathIds),0);

  //special case when we are already at the goal
  if(rstargoalstate^.MDPState = pSearchStateSpace.searchStartState) then
  begin
    solcost^ := 0;
    result:=wholePathIds;
  end;

  //no path to goal state was found
  if( (rstargoalstate^.g >= INFINITECOST) OR (rstargoalstate^.bestPredAction = Nil)
  OR (Length(pRstarActionData(rstargoalstate^.bestpredaction^.PlannerSpecificData)^.pathIDs)= 0)) then
     result:= wholePathIds;

   //path exists
   pathCost:=0;
   rstarState := rstarGoalState;
   while( (rstarState^.bestPredAction <> nil) AND (rstarState^.MDPState <> pSearchStateSpace.searchStartState) ) do
   begin

        //get action data
        bestpredactiondata := pRstarActionData(rstarState^.bestPredAction^.PlannerSpecificData);

        //get predecessor in the search tree
        predState:= pRStarState(GetState(rstarState^.bestPredAction^.SourceStateID)^.PlannerSpecificData);

        //check validity
        if( (predstate^.g + bestpredactiondata^.clow) <> (rstarstate^.g) ) then
            // exception

       //store the action and its cost
       {tempPathID.push_back(rstarstate->bestpredaction);}
          SetLength(tempPathID, Length(tempPathID)+1);
          tempPathID[High(tempPathID)]:=rstarState^.bestPredAction;
        pathCost := pathcost + rstarstate^.bestpredaction^.Costs[0];

       //go to the predecessor
       rstarstate := predstate;

       //another check
       if(pathcost + rstarstate^.g > rstargoalstate^.g) then
           //exception
   end; // End while

    //now recover the actual path
    for aind := 0 to Length(tempPathID) do
    begin
      if(bforwardsearch) then
           //getting path in reverse
           actiondata := pRstarActionData(tempPathID[Length(tempPathID) - aind - 1]^.PlannerSpecificData)
      else
           actiondata:= pRstarActionData(tempPathID[aind]^.PlannerSpecificData);

      //get the states that correspond to the high-level action
      for j := 0 to Length(actiondata^.pathIDs) do
      begin
         //note: path corresponding to the action is already in right direction
         {wholePathIds.push_back(actiondata->pathIDs.at(j));}
            SetLength(wholePathIds, Length(wholePathIds)+1);
            wholePathIds[High(wholePathIds)]:=actiondata^.pathIDs[j];
      end; // end for internal
    end;  // end for external

    //add the goal state
    if(bforwardsearch) then
    begin
       {wholePathIds.push_back(rstargoalstate->MDPstate->StateID);}
       SetLength(wholePathIds, Length(wholePathIds)+1);
       wholePathIds[High(wholePathIds)]:=rstargoalstate^.MDPState^.StateID;
    end
    else
       {wholePathIds.push_back(pSearchStateSpace->searchstartstate->StateID);}
       SetLength(wholePathIds, Length(wholePathIds)+1);
       wholePathIds[High(wholePathIds)]:=pSearchStateSpace.searchStartState^.StateID;

    //get the solcost
    solcost^ := pathcost;
    result:=wholePathIds;

end;

function tRstarPlanner.LocalSearchComputeKey(
  rstarlsearchState: pRStarLSearchState): tCkey;
var
   retKey:tCKey;
   h:integer;
begin
  if(bforwardsearch) then
    // h = environment_->GetFromToHeuristic(rstarlsearchState->MDPstate->StateID, pLSearchStateSpace->GoalState->StateID); // Envia o ID da posição atual e da posição objetivo e então é calculado o H chamando o método GetFromHeuristic. Linha 2836(environment_navXYTHETALAT.cpp)
  else
    // h = environment_->GetFromToHeuristic(pLSearchStateSpace->GoalState->StateID, rstarlsearchState->MDPstate->StateID); // Envia o ID da posição objetivo e da posição atual e então é calculado o H.
   retKey.key[0] := rstarlsearchState^.g + round(pSearchStateSpace.eps * h);
  result:= retKey;

end;

function tRstarPlanner.GetLSearchState(stateID: integer): pCMDPState;
begin
   if(stateID >=0 {apagar o 0 (int)environment_->StateID2IndexMapping.size()})then
         {SBPL_ERROR("ERROR int GetLSearchState: stateID is invalid\n"); throw new SBPL_Exception();}

	if({environment_->StateID2IndexMapping[stateID][RSTARMDP_LSEARCH_STATEID2IND] == -1}true) then // Logo em seguida é verificado se há algum estado de busca criado (O valor -1 significa que nenhum estado busca foi criado ainda para este hashentry e nesse caso será criado um estado com esse ID)
		result:= CreateLSearchState(stateID);
	else
		result:= pLSearchStateSpace.MDP.StateArray[0{apagar o 0  environment_->StateID2IndexMapping[stateID][RSTARMDP_LSEARCH_STATEID2IND]}]; // Em seguida é retornado o estado local

end;

function tRstarPlanner.CreateLSearchState(stateID: integer): pCMDPState;
var
    state: pCMDPState;
begin
   state := NULL;										 // Inicialmente cria um estado nulo.


	if({environment_->StateID2IndexMapping[stateID][RSTARMDP_LSEARCH_STATEID2IND] != -1apagar o true} true) then
	      {SBPL_ERROR("ERROR in CreateState: state already created\n"); throw new SBPL_Exception();}


	//adds to the tail a state
	state := pLSearchStateSpace.MDP.addState(stateID); // Adciona o iD do estado ao grafo MDP e esse a variavel State

	//remember the index of the state
	//environment_->StateID2IndexMapping[stateID][RSTARMDP_LSEARCH_STATEID2IND] = pLSearchStateSpace->MDP.StateArray.size()-1;


	if(state <> pLSearchStateSpace.MDP.StateArray[{environment_->StateID2IndexMapping[stateID][RSTARMDP_LSEARCH_STATEID2IND apagar o 0} 0]) then
	     {SBPL_ERROR("ERROR in CreateState: invalid state index\n");throw new SBPL_Exception();}

	//create and initialize rstarlsearch_data
	state^.PlannerSpecificData := tRStarLSearchState.create(); //Cria  um estado de busca local com os seguintes atributos: MDP; StartState; GoalState; iteration; OPEN; (incons list - used for suboptimal search) INCONS;
	Initialize_rstarlsearchdata(state); // Inicializa o estado local, Linha 210

	result:= state; //Após a inicialização das informações retorna o estado
end;

procedure tRstarPlanner.SetBestPredecessor(rstarState: pRstarState;
  rstarPredState: pRstarState; action: pCMDPAction);
begin
   rstarState^.bestPredAction := action;
   rstarState^.g := rstarPredState^.g + pRstarActionData(action^.PlannerSpecificData)^.clow;
    if(rstarState^.heapindex = 0) then
        pSearchStateSpace.OPEN^.insertheap(rstarState, ComputeKey(rstarState))
    else
        pSearchStateSpace.OPEN^.updateheap(rstarState, ComputeKey(rstarState));
end;

procedure tRstarPlanner.set_initialsolution_eps(initialsolution_eps: double);
begin
  finitial_eps := initialsolution_eps;
end;

end.
