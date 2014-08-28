program projectcontrol;

{$mode objfpc}{$H+}

uses
  {$IFDEF UNIX}{$IFDEF UseCThreads}
  cthreads,
  {$ENDIF}{$ENDIF}
  Interfaces, // this includes the LCL widgetset
  Forms
  { you can add units after this }, control, lnetvisual, LResources,
configSimTwo, robotmainfunctions, AStar, RStar, DecConsts, AStarModified, RStar;

//{$IFDEF WINDOWS}{$R projectcontrol.rc}{$ENDIF}

begin
  //{$I projectcontrol.lrs}
  Application.Initialize;
  Application.CreateForm(TFcontrol, Fcontrol);
  Application.CreateForm(TFconfig, Fconfig);
  Application.Run;
end.

