unit Main;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, FileUtil, Forms, Controls, Graphics, Dialogs,Rstar;

type
  TForm1 = class(TForm)
  private
    { private declarations }
  public
    { public declarations }
  end;

  type


    { Main }

    tMain = class

      public
        function plan2D( envCfgFilename: pChar; forwardSearch: boolean): integer;
    end;

type
  tMDPConfig = record
	startstateid:integer;
	goalstateid:integer;
end;

var
  Form1: TForm1;

implementation

{$R *.lfm}

{ Main }

function tMain.plan2D(envCfgFilename: pChar; forwardSearch: boolean): integer;
var
  bRet:integer;
  allocated_time_secs: double;
  initialEpsilon: double;
  MDPCfg: tMDPConfig;
  bsearchuntilfirstsolution:boolean;
  bforwardsearch:boolean;
  solution_stateIDs_V: tIntegerArray;
  planner:tRstarPlanner;
begin
   bRet := 0;
   allocated_time_secs := 100.0; // in seconds
   initialEpsilon := 3.0;
   bsearchuntilfirstsolution := false;
   bforwardsearch := forwardSearch;

   {Initialize Environment (should be called before initializing anything else)}
	//EnvironmentNAV2D environment_nav2D;
	//if (!environment_nav2D.InitializeEnv(envCfgFilename))
		{printf("ERROR: InitializeEnv failed\n"); throw new SBPL_Exception(); }

        // Initialize MDP Info
	{if (!environment_nav2D.InitializeMDPCfg(&MDPCfg))
		printf("ERROR: InitializeMDPCfg failed\n");
		throw new SBPL_Exception();
	}

        {printf("Initializing RSTARPlanner...\n"); }
	planner := tRstarPlanner.create({@environment_nav2D}nil, bforwardsearch);

        // set search mode
	planner.set_search_mode(bsearchuntilfirstsolution);

        if (planner.set_start(MDPCfg.startstateid) = 0) then
	   {printf("ERROR: failed to set start state\n");throw new SBPL_Exception();}

        if (planner.set_goal(MDPCfg.goalstateid) = 0) then
           {printf("ERROR: failed to set start state\n"); throw new SBPL_Exception();}

        planner.set_initialsolution_eps(initialEpsilon);

        {printf("start planning...\n"); }
	//bRet :=
        {erro -  Não está querendo aceitar o ponteiro}
        planner.replan(allocated_time_secs, @solution_stateIDs_V);
        //replan(allocated_time_secs: double; solution_stateIDs_v: array of pInteger): integer;

        {printf("done planning\n");
	std::cout << "size of solution=" << solution_stateIDs_V.size() << std::endl;}

        //print a path
	if (bRet) then
		//print the solution
		//printf("Solution is found\n");

	else
		//printf("Solution does not exist\n");


	//delete planner;

	result := bRet;


end;
{










	;





end.

