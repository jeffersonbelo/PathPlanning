unit robotmainfunctions;

{$mode objfpc}{$H+}

interface


uses
 Classes, SysUtils, math, LCLtype, IniFiles,dynmatrix,dynmatrixutils,Forms,DecConsts,
 FileUtil, LResources, Controls, Graphics, Dialogs, ExtCtrls, StdCtrls, ComCtrls,
 ActnList, Spin, Menus, IniPropStorage, EditBtn,Grids, LCLIntf, Messages, DateUtils;

const NumWheels = 3;
      speedRate = 0.05;
      maxSpeed = 40;
      pi=3.1415726;
      kC=0.1;
      PARADO = 0;
      Kimpulsos = 1190;
      MaxTrajectoryCount=256;

type
  TKeyState = record
  up,
  down,
  left,
  right: boolean;
end;

Type
  TAgvUpdate = record
  inf: integer;
end;

Type
  TDriverState = record
  pwm,
  ref :double;
  out_pwm : integer;
  Vact,
  Iact: double;
  enc: Word;
end;

  type
  TRobotSpeeds=record
    vx,vy,tetap: double;
    v,vn,w: double;
    v1,v2,v3: double;
    vmax : double;
  end;

type
  TRobotPose = record
  x,y,theta: double;
  thetarad : double;
end;

//------------------------------------------------------
// TMotorState
// Model parameters for simulation
//------------------------------------------------------
TMotorState = record
  v : double                //motor speed, in m/s
end;

//------------------------------------------------------
// TSimRobot
// Robot for simulation
//------------------------------------------------------
TSimRobot = record
  Pose : TRobotPose;
  Vel: TRobotSpeeds;
  MotorState : Array[0 .. 2] of TMotorState;
end;


type Tavoid = array [0..4,0..11] of double; //matrix de posição das esferas

type
  TTrajectoryPoint=record //Tipo Vetor Ponto de Trajetória (x,y,teta,w)
    x,y:double;
    teta,teta_power:double;
    vRef : double;
  end;

  TTrajectory=record //Tipo trajetória gerada
    count: integer; //contagem de pontos
    currentPoint: integer; //ponto atual
    distance: double; //distância
  //  static: boolean; //Geração de trajetória estática ou dinâmica
    index : integer;
    predIndex : integer;
    varSpeed : boolean;
    pts: array[0..MaxTrajectoryCount-1] of TTrajectoryPoint; //nº de pontos do
  end;

type

  //------------------------------------------------------
  // TSimParameters
  // holds parameters for MPC controller simulation part
  //------------------------------------------------------
  TSimParameters=record
    N1,N2,Nu : integer;               //control and prediction horizons
    lambda : array[0 .. 2] of double; //weights for the cost function calculation
  end;

  //------------------------------------------------------
  // TOptParameters
  // Optimizer parameters
  //------------------------------------------------------
  TOptParameters=record
    MaxIterations : double;        //Max iterations of the minimization algorithm
    delta : double;                //step for variable change for calculation of grad(J)
    Jstop : double;                //Stop criteria for cost
    alpha : double;                //Step size for steepest gradient
  end;

  //------------------------------------------------------
  // TOptData
  // Optimizer data structures
  //------------------------------------------------------
  TOptData=record
   Jsteps : TDMatrix;  //intermediate matrix for antigradient calculation
   Jgradient : TDMatrix;
   Jgradient_prev: TDMatrix;
   Jstep_prev: TDMatrix;
   iterationCount : integer; //Current iteration number
  end;

var irobot, isensor: integer;
    t: double;
    trajS: TTrajectory;
    ODO1_actual,ODO2_actual,ODO3_actual: integer;
    ODO1_past,ODO2_past,ODO3_past,ODO1_real,ODO2_real,ODO3_real: integer;
    sx, sy, re,velx,vely: array [0..11] of double;
    sum1,sum2,sum3:integer;
    goState:integer;                  // State of the function that defines robots pose
    THETA2,THETA,deltaTHETA:double;   // robot's angles
    x,y:double;                       // robot's position
    vaxis1,vaxis2,vaxis3:double;             // robot's axis actual velocity
    vlim1, vlim2,vlim3: double;             // robot's limited velocity command
    vmax1,vmax2,vmax3: double;              // maximun robot velocity
    orientationPoint: double;         // orientation of a point in the map
    px,py,ptheta: double;
    LoadReady,UnLoadReady: integer;            // robo´t  go to point ,final orientation  and  velocity
    alpha:double;                     // angle of the line to follow
    ControlMode: string;
    RPoseSimTwo: TRobotPose;
    RVelSimTwo, RDataDesejada: TRobotSpeeds;
    DStates : array[0..3] of TDriverState;
    KeyState : TKeyState;
    gData: TStringList;
    gData_Real: TStringList;
    AgvUpdate : array[0..1] of TAgvUpdate ;
    TTavoid : TAvoid;

    //MPC Controller
    SimRobot: TSimRobot;
    progReset: boolean;
    AlgTimeMax, AlgTimeMin, AlgTimeSum : longword;
    AlgItCount : integer;

    //Controller structures
    SimParameters : TSimParameters;
    OptParameters : TOptParameters;
    OptData : TOptData;

     // Variáveis usadas no decorrer da análise do tempo
    interactionControl : integer;
    initialTime, finalTime, tempExecution,totalTime : TDateTime;

Procedure Inicialize_Variables;
Procedure Stop;
procedure setSpeed(v1,v2,v3: double);
procedure Control_Keys(v: double);
procedure ProcessSimTwoMsg(Data: string);
function GotoXYTheta(x, y, teta, speed, sot: double; avoid: Tavoid; tip: integer):TTrajectory;
function GotoXYThetaP(x, y, teta, speed, sot: double; avoid: Tavoid; tip: integer):TTrajectory;
procedure GTXYTCcontroller(speed, speed_on_target: double; var RPose: TRobotPose; var RVel: TRobotSpeeds; var traj: TTrajectory);
function Dist(x,y: double): double;
procedure ProportionalSat(var v1,v2,v3: double; vmax: double);
procedure SetVelRobotRef(v1,v2,v3: double);
procedure TrajectoryController(speed, speed_on_target: double; var traj: TTrajectory);
procedure VVnToVxy(teta,v,vn: double; var Vx,Vy: double);
procedure SetTacticCommandsInRobotRef(v,vn,tetap: double);
function ATan2(y,x: double): double;

//reset model state
procedure resetModel();
//inverse kinematic (v,vn,w->v1,v2,v3)
procedure IK(v,vn,w: double; var v1,v2,v3 : double);
//direct kinematic (v1,v2,v3->v,vn,w)
procedure DK(v1, v2, v3: double; var v, vn, w: double);
//updates robot position
procedure simRobotMovement(var Robot : TSimRobot; var v,vn,w : double);

//main controller loop
procedure MPCcontroller(var traj : TTrajectory; vRefModule: double);
//state predictor, returns cost value (J) of current inputs
function predSimulator(var Robot : TSimRobot; var U : TDMatrix; refTraj : TTrajectory) : double;
//generate deltaU matrix
procedure calcUSteps(var U, deltaU : TDMatrix);
//calculate gradient of J from Jsteps vector
procedure calcGradient(var Jsteps, Jgradient, Jgradient_prev : TDMatrix);
//calculate a steepest descent step
procedure calcSteepestDescentStep(var Jgradient, Uref : TDmatrix);
//get reference trajectory for controller
procedure calcRefTraj(var traj,trajPred: TTrajectory; V : double);
//load configuration from form
procedure loadConfig();
//keeps vector norms if one saturates
procedure ProportionalSatMPC(var v1,v2,v3: double; vmax: double);
//prevents motor saturation
procedure scaleForSaturation(var U : TDmatrix);
function DistPointInLine(x1,y1,x2,y2,x3,y3: double): double;


implementation

uses control,configSimTwo,AStar, AStarModified,RStar ;

var FirstTimeValSec: LongInt;

function FMod(x,d: double): double;
begin
  result:=Frac(x/d)*d;
end;

function DiffAngle(a1,a2: double): double;
begin
  result:=a1-a2;

  if result<0 then begin
    result:=-FMod(-result,2*Pi);
    if result<-Pi then result:=result+2*Pi;
  end else begin
    result:=FMod(result,2*Pi);
    if result>Pi then result:=result-2*Pi;
  end;
end;

function ATan2(y,x: double): double;
var ax,ay: double;
begin
  ax:=Abs(x);
  ay:=Abs(y);

  if (ax<1e-10) and (ay<1e-10) then begin;
    result:=0.0;
    exit;
  end;
  if ax>ay then begin
    if x<0 then begin
      result:=ArcTan(y/x)+pi;
      if result>pi then result:=result-2*pi;
    end else begin
      result:=ArcTan(y/x);
    end;
  end else begin
    if y<0 then begin
      result:=ArcTan(-x/y)-pi/2
    end else begin
      result:=ArcTan(-x/y)+pi/2;
    end;
  end;
end;

procedure ProcessSimTwoMsg(Data: string);
 var i,j: integer;
    v,Vd,R,dir,tx,ty : double;
    m: integer;
    trajSa: TTrajectory;
    x,y: integer;
    pnt,pnt2,pnti: TGridCoord;
    msn: String;
    FSettings: TFormatSettings;
 begin
  FSettings.DecimalSeparator:='.';
  gData := TStringList.Create;
  gData.Text := Data;
  try
  if (gData[0][1] = chr(35)) and (gData[67][1] = chr(42)) then begin

     RPoseSimTwo.x := StrToFloat(gData[1], FSettings);
     RPoseSimTwo.y := StrToFloat(gData[2], FSettings);
     RPoseSimTwo.theta := StrToFloat(gData[3], FSettings)*180/pi;
     RPoseSimTwo.thetarad :=  RPoseSimTwo.theta*pi/180;

     RVelSimTwo.v1 := round(StrToInt(gData[4])*ticktorad);
     RVelSimTwo.v2 := round(StrToInt(gData[5])*ticktorad);
     RVelSimTwo.v3 := round(StrToInt(gData[6])*ticktorad);

     FControl.Edit28.text := Format('%.1f', [RVelSimTwo.v1]);
     FControl.Edit29.text := Format('%.1f', [RVelSimTwo.v2]);
     FControl.Edit30.text := Format('%.1f', [RVelSimTwo.v3]);

     SetVelRobotRef(RVelSimTwo.v1,RVelSimTwo.v2,RVelSimTwo.v3);
     VVnToVxy(RPoseSimTwo.theta, RVelSimTwo.v, RVelSimTwo.vn, RVelSimTwo.vx, RVelSimTwo.vy);

     sx[0]:= StrToFloat(gData[7], FSettings);
     sx[1]:= StrToFloat(gData[8], FSettings);
     sx[2]:= StrToFloat(gData[9], FSettings);
     sx[3]:= StrToFloat(gData[10], FSettings);
     sx[4]:= StrToFloat(gData[11], FSettings);
     sx[5]:= StrToFloat(gData[12], FSettings);
     sx[6]:= StrToFloat(gData[13], FSettings);
     sx[7]:= StrToFloat(gData[14], FSettings);
     sx[8]:= StrToFloat(gData[15], FSettings);
     sx[9]:= StrToFloat(gData[16], FSettings);
     sx[10]:= StrToFloat(gData[17], FSettings);
     sx[11]:= StrToFloat(gData[18], FSettings);
     sy[0]:= StrToFloat(gData[19], FSettings);
     sy[1]:= StrToFloat(gData[20], FSettings);
     sy[2]:= StrToFloat(gData[21], FSettings);
     sy[3]:= StrToFloat(gData[22], FSettings);
     sy[4]:= StrToFloat(gData[23], FSettings);
     sy[5]:= StrToFloat(gData[24], FSettings);
     sy[6]:= StrToFloat(gData[25], FSettings);
     sy[7]:= StrToFloat(gData[26], FSettings);
     sy[8]:= StrToFloat(gData[27], FSettings);
     sy[9]:= StrToFloat(gData[28], FSettings);
     sy[10]:= StrToFloat(gData[29], FSettings);
     sy[11]:= StrToFloat(gData[30], FSettings);
     re[0]:= StrToFloat(gData[31], FSettings);
     re[1]:= StrToFloat(gData[32], FSettings);
     re[2]:= StrToFloat(gData[33], FSettings);
     re[3]:= StrToFloat(gData[34], FSettings);
     re[4]:= StrToFloat(gData[35], FSettings);
     re[5]:= StrToFloat(gData[36], FSettings);
     re[6]:= StrToFloat(gData[37], FSettings);
     re[7]:= StrToFloat(gData[38], FSettings);
     re[8]:= StrToFloat(gData[39], FSettings);
     re[9]:= StrToFloat(gData[40], FSettings);
     re[10]:= StrToFloat(gData[41], FSettings);
     re[11]:= StrToFloat(gData[42], FSettings);
     velx[0]:= StrToFloat(gData[43], FSettings);
     velx[1]:= StrToFloat(gData[44], FSettings);
     velx[2]:= StrToFloat(gData[45], FSettings);
     velx[3]:= StrToFloat(gData[46], FSettings);
     velx[4]:= StrToFloat(gData[47], FSettings);
     velx[5]:= StrToFloat(gData[48], FSettings);
     velx[6]:= StrToFloat(gData[49], FSettings);
     velx[7]:= StrToFloat(gData[50], FSettings);
     velx[8]:= StrToFloat(gData[51], FSettings);
     velx[9]:= StrToFloat(gData[52], FSettings);
     velx[10]:= StrToFloat(gData[53], FSettings);
     velx[11]:= StrToFloat(gData[54], FSettings);
     vely[0]:= StrToFloat(gData[55], FSettings);
     vely[1]:= StrToFloat(gData[56], FSettings);
     vely[2]:= StrToFloat(gData[57], FSettings);
     vely[3]:= StrToFloat(gData[58], FSettings);
     vely[4]:= StrToFloat(gData[59], FSettings);
     vely[5]:= StrToFloat(gData[60], FSettings);
     vely[6]:= StrToFloat(gData[61], FSettings);
     vely[7]:= StrToFloat(gData[62], FSettings);
     vely[8]:= StrToFloat(gData[63], FSettings);
     vely[9]:= StrToFloat(gData[64], FSettings);
     vely[10]:= StrToFloat(gData[65], FSettings);
     vely[11]:= StrToFloat(gData[66], FSettings);

     FControl.EB_PositionX.text := Format('%.4f', [ RPoseSimTwo.x]);
     FControl.EB_PositionY.text := Format('%.4f', [ RPoseSimTwo.y]);
     FControl.EB_Theta.text := Format('%.4f', [RPoseSimTwo.theta]);

     FControl.Edit4.text := Format('%.4f', [sx[0]]);
     FControl.Edit5.text := Format('%.4f', [sx[1]]);
     FControl.Edit6.text := Format('%.4f', [sx[2]]);
     FControl.Edit7.text := Format('%.4f', [sx[3]]);
     FControl.Edit8.text := Format('%.4f', [sx[4]]);
     FControl.Edit9.text := Format('%.4f', [sx[5]]);
     FControl.Edit10.text := Format('%.4f', [sx[6]]);
     FControl.Edit11.text := Format('%.4f', [sx[7]]);
     FControl.Edit12.text := Format('%.4f', [sx[8]]);
     FControl.Edit13.text := Format('%.4f', [sx[9]]);
     FControl.Edit14.text := Format('%.4f', [sx[10]]);
     FControl.Edit15.text := Format('%.4f', [sx[11]]);
     FControl.Edit16.text := Format('%.4f', [sy[0]]);
     FControl.Edit17.text := Format('%.4f', [sy[1]]);
     FControl.Edit18.text := Format('%.4f', [sy[2]]);
     FControl.Edit19.text := Format('%.4f', [sy[3]]);
     FControl.Edit20.text := Format('%.4f', [sy[4]]);
     FControl.Edit21.text := Format('%.4f', [sy[5]]);
     FControl.Edit22.text := Format('%.4f', [sy[6]]);
     FControl.Edit23.text := Format('%.4f', [sy[7]]);
     FControl.Edit24.text := Format('%.4f', [sy[8]]);
     FControl.Edit25.text := Format('%.4f', [sy[9]]);
     FControl.Edit26.text := Format('%.4f', [sy[10]]);
     FControl.Edit27.text := Format('%.4f', [sy[11]]);

     for i:=0 to 11 do begin
         TTavoid[0][i]:=sx[i];
     end;
     for i:=0 to 11 do begin
         TTavoid[1][i]:=sy[i];
     end;
     for i:=0 to 11 do begin
         TTavoid[2][i]:=re[i];
     end;
     for i:=0 to 11 do begin
         TTavoid[3][i]:=velx[i];
     end;
     for i:=0 to 11 do begin
         TTavoid[4][i]:=vely[i];
     end;

  end;
  finally
    gData.Free;
  end;



   if Fcontrol.RadioButton1.Checked=True then begin
     Fcontrol.CBM.Checked:=False;
     Fcontrol.CBMPC.Checked:=False;
     stop;  //Stop Mode
   end else if Fcontrol.RadioButton3.Checked=True then begin
     Fcontrol.CBM.Checked:=False;
     Fcontrol.CBMPC.Checked:=False;
     Control_Keys(StrTofloat(Fcontrol.EB_Vref.Text, FSettings));
   end else if Fcontrol.RadioButton2.Checked=True then begin
       v:=StrTofloat(Fcontrol.EB_Vref.Text, FSettings);
       tx:=4;
       ty:=1.5;
       if Fcontrol.CBM.Checked=True then begin
         trajSa:=GotoXYTheta(tx, ty, 0, v, 0, TTavoid, 0);
       end;
       if Fcontrol.CBMPC.Checked=True then begin
         trajSa:=GotoXYTheta(tx, ty, 0, v, 0, TTavoid, 1);
       end;
       if Fcontrol.GTXYT.Checked=True then begin
         trajSa:=GotoXYTheta(tx, ty, 0, v, 0, TTavoid, 2);     // AQUI
       end;

   end else if Fcontrol.RadioButton4.Checked=True then begin
       v:=StrTofloat(Fcontrol.EB_Vref.Text, FSettings);
       tx:=4;
       ty:=1.5;
       if Fcontrol.CBM.Checked=True then begin
         trajSa:=GotoXYThetaP(tx, ty, 0, v, 0, TTavoid, 0);
       end;
       if Fcontrol.CBMPC.Checked=True then begin
         trajSa:=GotoXYThetaP(tx, ty, 0, v, 0, TTavoid, 1);
       end;
       if Fcontrol.GTXYT.Checked=True then begin
         trajSa:=GotoXYThetaP(tx, ty, 0, v, 0, TTavoid, 2);
       end;
   end;

  if Fcontrol.CB_SimTwo.Checked=True then begin        //envia para o SimTwo
    msn:=chr(35)+chr(13)+floattostr(DStates[0].ref, FSettings)+chr(13)+
                         floattostr(DStates[1].ref, FSettings)+chr(13)+
                         floattostr(DStates[2].ref, FSettings)+chr(13);
                         //inttostr(trajSa.count)+chr(13);
    //for m:=0 to trajSa.count-1 do begin
    //  msn:=msn+floattostr(trajSa.pts[m].x)+chr(13);
    //  msn:=msn+floattostr(trajSa.pts[m].y)+chr(13);
    //end;
    msn:=msn+chr(42);
    Fconfig.UDP.SendMessage(msn,Fconfig.Edit1.Text+':'+Fconfig.Edit2.Text);
  end;

  if Fcontrol.CB_SimTwo.Checked=False then begin
    Fconfig.UDP.Disconnect;
  end;

end;

Procedure Inicialize_Variables;
begin
  goState:=0;
  irobot := 0;
  THETA:=0;
  t := 0;
  y:=0.8;
  x:=0;
end;

///////////////////////Function STOP Vehicle///////////////////
procedure Stop;
begin
    Fcontrol.Edit36.Text:=inttostr(0);
    vlim1:=0;
    vlim2:=0;
    vlim3:=0;
    SetSpeed(vlim1,vlim2,vlim3);
    goState:=0;
    KeyState.up:=false;
    KeyState.down:=false;
    KeyState.left:=false;
    KeyState.right:=false;
end;

procedure Control_Keys(v: double);
var v1,v2,v3 : double;
begin

  v1:=0;
  v2:=0;

  if KeyState.down then begin
    v1 := v1 - 1;
    v2 := v2 + 1;
  end;
  if KeyState.up then begin
    v1 := +1;
    v2 := -1;
  end;
  if KeyState.left then begin
    v1 := v1 + 1;
    v2 := v2 + 1;
  end;
  if KeyState.right then begin
    v1 := v1 - 1;
    v2 := v2 - 1;
  end;

  v1 := v1*v;
  v2 := v2*v;
  v3:= 0;
  setSpeed(v1,v2,v3);
end;

function sqrt2(x1,x2,y1,y2:double):double;
begin
  result:=sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
end;

/////////////////////////////////////////////////////////////////
//                      SET SPEED                              //
//         This procedure sets the speed on the axis           //
/////////////////////////////////////////////////////////////////
procedure setSpeed(v1,v2,v3: double);

begin
   DStates[0].ref := v1;
   DStates[1].ref := v2;
   DStates[2].ref := v3;

   FControl.Edit1.text := Format('%.1f', [v1]);
   FControl.Edit2.text := Format('%.1f', [v2]);
   FControl.Edit3.text := Format('%.1f', [v3]);

end;

// GoToXYTetaPStar
//
// Moves the robot to the position defined initially
// following a trajectory calculated by RobotBestPath().
//-------------------------------------------------------------------------
function GotoXYThetaP(x, y, teta, speed, sot: double; avoid: Tavoid; tip: integer):TTrajectory;
var tx,ty,rx,ry,d: double;
    i,idx: integer;
    traj: TTrajectory;
    it: integer;
    tm64i, tm64f, pf: int64;
    Img:TImage;

begin
  tx:=x;
  ty:=y;


  //Get Robot trajectory (A*)
  RobotBestPathP(tx,ty,traj,avoid);

  //Displays the number of points generated by A*
  FControl.Edit39.text := inttostr(traj.count);

  for i:=0 to traj.count - 1 do begin
    traj.pts[i].teta := teta;
    traj.pts[i].teta_power := 1;

  end;

  //Call Trajectory Controller
  if tip=0 then begin
    TrajectoryController(speed, sot, traj);
  end;
  if tip=1 then begin
    SimRobot.Vel.vmax:=speed;
    MPCcontroller(traj,SimRobot.Vel.vmax);
  end;
  if tip=2 then begin
    GTXYTCcontroller(speed, sot, RPoseSimTwo, RVelSimTwo, traj);
  end;

    result:=traj;
end;

/////////////////////////////////////////////////////////////////
//                      GO TO XY THETA                         //
//    This procedure will set the robot in the pose defined    //
/////////////////////////////////////////////////////////////////
//-------------------------------------------------------------------------
// GoToXYTeta
//
// Moves the robot to the position defined initially
// following a trajectory calculated by RobotBestPath().
//-------------------------------------------------------------------------
function GotoXYTheta(x, y, teta, speed, sot: double; avoid: Tavoid; tip: integer):TTrajectory;
var tx,ty,rx,ry,d: double;
    i,idx: integer;
    traj: TTrajectory;
    it: integer;
    tm64i, tm64f, pf: int64;
    Img:TImage;
begin
  tx:=x;
  ty:=y;

  //Get Robot trajectory (A*)
  //interactionControl : integer;
  //initialTime, finalTime, tempExecution,totalTime : TDateTime;

  //interactionControl := 0;
  initialTime := TimeOf(Now);

  RobotBestPath(tx,ty,traj,avoid);


  finalTime := TimeOf(Now);                                            // TimeOf(Now) -   Função que captura o tempo real do computador em milisegundos e atribui a variavel tempoFinal.
  tempExecution:= finalTime - initialTime;                             // A variavel tempExecução receberá a diferenca de tempo entre o inicio do programa e fim, desse modo, o tempo real de execução da tarefa.
  totalTime := totalTime + tempExecution;

  //Displays the number of points generated by A*
  FControl.Edit39.text := inttostr(traj.count);

  for i:=0 to traj.count - 1 do begin
    traj.pts[i].teta := teta;
    traj.pts[i].teta_power := 1;

  end;


  //Call Trajectory Controller
  if tip=0 then begin
    TrajectoryController(speed, sot, traj);
  end;
  if tip=1 then begin
    SimRobot.Vel.vmax:=speed;
    MPCcontroller(traj,SimRobot.Vel.vmax);
  end;
  if tip=2 then begin
    GTXYTCcontroller(speed, sot, RPoseSimTwo, RVelSimTwo, traj);
  end;

  result:=traj;
end;

function Dist(x,y: double): double;
begin
  result:=sqrt(x*x+y*y);
end;

//----------------------------------------------------------------------
//  Controller
//----------------------------------------------------------------------

//-------------------------------------------------------------------------
// VxyToVVn()
//
// Convert speeds in Vxy to VVn
//-------------------------------------------------------------------------
procedure VxyToVVn(teta,vx,vy: double; var V,Vn: double);
var ct,st: double;
begin
  ct:=cos(teta);
  st:=sin(teta);
  v:=vx*ct+vy*st;
  vn:=-vx*st+vy*ct;
end;


//-------------------------------------------------------------------------
// VVnToVxy()
//
// Convert speeds in VVn to Vxy
//-------------------------------------------------------------------------
procedure VVnToVxy(teta,v,vn: double; var Vx,Vy: double);
var ct,st: double;
begin
  ct:=cos(teta);
  st:=sin(teta);
  vx:=v*ct-vn*st;
  vy:=v*st+vn*ct;
end;


function SatVal(v, vmax: double): double;
begin
  if v > vmax then v := vmax;
  if v < -vmax then v := -vmax;
  result := v;
end;

//-------------------------------------------------------------------------
// ProportionalSat
//
// Keeps vector directions if norms have to be scaled
//-------------------------------------------------------------------------
procedure ProportionalSat(var v1,v2,v3: double; vmax: double);
var maxv,minv: double;
    scale,scalemax,scalemin: double;
begin
  maxv:=Max(v1,Max(v2,v3));
  minv:=Min(v1,Min(v2,v3));

  if maxv>vmax then scalemax:=maxv/vmax else scalemax:=1.0;
  if minv<-vmax then scalemin:=minv/(-vmax) else scalemin:=1.0;

  scale:=Max(scalemin,scalemax);

  v1:=v1/scale;
  v2:=v2/scale;
  v3:=v3/scale;

  setSpeed(v1,v2,v3);


end;

//-------------------------------------------------------------------------
// SetTacticCommandsInRobotRef
//
// Defines the calculated v, vn and w in the TTacticCommand object of the
// current Robot. This is the object that's read to send the speeds to
// the motors
//-------------------------------------------------------------------------
procedure SetTacticCommandsInRobotRef(v,vn,tetap: double);
var v1,v2,v3: double;
begin

    v1 := v*cos(alfa) + vn*sin(alfa) + 0.195*tetap;
    v2 := -v*cos(alfa) + vn*sin(alfa) + 0.195*tetap;
    v3 := -vn + 0.195*tetap;

    ProportionalSat(v1,v2,v3,strtofloat(Fcontrol.EB_Vref.text));
end;

// Calculates the received v, vn and w
//-------------------------------------------------------------------------
procedure SetVelRobotRef(v1,v2,v3: double);
begin
    RVelSimTwo.v := (RVelSimTwo.v1*0.5774)-(RVelSimTwo.v2*0.5774);
    RVelSimTwo.vn := (RVelSimTwo.v1*0.3333)+(RVelSimTwo.v2*0.3333)-(RVelSimTwo.v3*0.6667);
    RVelSimTwo.w := (RVelSimTwo.v1*1.7094)+(RVelSimTwo.v2*1.7094)+(RVelSimTwo.v3*1.7094);

end;

procedure controleMV(teta,ex,ey,evx,evy,eteta,ew,vx,vy:double);
var ki,kp,st,ct:double;
begin

     ki:=16;
     kp:=0.8;
     ct:=cos(teta);
     st:=sin(teta);
     RDataDesejada.vx:=(ex*ki*ct)-(ew*(ct*(vy*ct+vx*st)-st*(vx*ct-vy*st)))+(evx*kp*ct)+(ey*ki*st)+(evy*kp*st);
     RDataDesejada.vy:=((ew*(ct*(vx*ct-vy*st)+st*(vy*ct+vx*st)))+(ey*ki*ct)+(evy*kp*ct)-(ex*ki*st)-(evx*kp*st));
     RDataDesejada.w:=(eteta*ki)+(ew*kp);

end;


//-------------------------------------------------------------------------
// TrajectoryControllerMultivariavel
//
// Calculates the correct v, vn, and w to follow a static trajectory
//-------------------------------------------------------------------------
procedure TrajectoryController(speed, speed_on_target: double; var traj: TTrajectory);
var statev, statevn, v, vn, dteta, teta_power: double;
    vmax, pw, tx, ty, segteta, td: double;
    idx: integer;
    statew, statevx, statevy, dx, dy, distance: double;
    next_x, next_y, prev_x, prev_y, next_teta: double;
    stv,stvn,b,k,y_diff,x_diff,y_cross, eteta,ew, ex, ey, evx, evy, svx, svy: double;
begin

  idx := Min(3, traj.count-1); //target is the 2nd trajectory point from current localization

  //calcula o x e y final e o delta teta total assim como o teta ponto
  tx := traj.pts[idx].x;
  ty := traj.pts[idx].y;
  //tx:=4;
  //ty:=1.5;
  //teta_power := traj.pts[idx].teta_power;

  //calcula o erro de x, y, teta e w
  //ex := tx-RPoseSimTwo.x;
  //ey := ty-RPoseSimTwo.y;
  //eteta := 0-RPoseSimTwo.theta;
  ex := traj.pts[idx].x-RPoseSimTwo.x;
  ey := traj.pts[idx].y-RPoseSimTwo.y;
  eteta := DiffAngle(traj.pts[idx].teta, RPoseSimTwo.theta);

  evx := RVelSimTwo.vx;
  evy := RVelSimTwo.vy;
  ew := 0-RVelSimTwo.w;

  td:=Dist(ex, ey);
  //FControl.Edit39.text := Format('%.1f', [td]);

  controleMV(RPoseSimTwo.theta,ex,ey,evx,evy,eteta,ew,RVelSimTwo.vx,RVelSimTwo.vy);
  VxyToVVn(RPoseSimTwo.theta, RDataDesejada.vx, RDataDesejada.vy, RDataDesejada.v, RDataDesejada.vn);

  FControl.Edit31.text := Format('%.1f', [tx]);
  FControl.Edit32.text := Format('%.1f', [ty]);
  FControl.Edit33.text := Format('%.1f', [RDataDesejada.v]);
  FControl.Edit34.text := Format('%.1f', [RDataDesejada.vn]);
  FControl.Edit35.text := Format('%.1f', [RDataDesejada.w]);

  //envia o valor de v, vn e pw finais para o robô
  if (Dist(tx-RPoseSimTwo.x, ty-RPoseSimTwo.y)<0.05)or((RPoseSimTwo.x=4)and(RPoseSimTwo.y=1.5)) then begin
    SetTacticCommandsInRobotRef(0, 0, 0);
    stop;
    exit;
  end else begin
    SetTacticCommandsInRobotRef(RDataDesejada.v, RDataDesejada.vn, RDataDesejada.w);
  end;
end;

//----------------------------------------------------------------------------
//
// resetModel()
//
//----------------------------------------------------------------------------
// reset model state
//
//----------------------------------------------------------------------------
procedure resetModel();
begin

               simRobot.Pose.x := 0;
               simRobot.Pose.y := 0;
               simRobot.Pose.theta := 0;
               simRobot.Vel.v:=0;
               simRobot.Vel.vn:=0;
               simRobot.Vel.w:=0;

               SimRobot.MotorState[0].v := 0;
               SimRobot.MotorState[1].v := 0;
               SimRobot.MotorState[2].v := 0;
end;

//----------------------------------------------------------------------------
//
// IK()
//
//----------------------------------------------------------------------------
// inverse kinematic (v,vn,w->v1,v2,v3)
//
//----------------------------------------------------------------------------
procedure IK(v, vn, w: double; var v1, v2, v3: double);
begin

    v1 := v*cos(alfa) + vn*sin(alfa) + 0.195*w;
    v2 := -v*cos(alfa) + vn*sin(alfa) + 0.195*w;
    v3 := -vn + 0.195*w;

end;

//----------------------------------------------------------------------------
//
// MPCcontroller()
//
//----------------------------------------------------------------------------
// Main loop for predictive controller
//
//----------------------------------------------------------------------------
procedure MPCcontroller(var traj : TTrajectory; vRefModule: double);
var
   Uaux,Uref,Usteps,Ubest: TDMatrix;
   Jprev,Jcurrent,Jbest,J : double;
   i,k, m, g : integer;
   algTime : longword;
   refTraj : TTrajectory;
   v1,v2,v3 : double;
   initMotorSpeeds : array [0..2] of double;
   tau : double;
   currentV : double;
   tx, ty: double;
   idx: integer;

begin

     //EXPERIMENTAL
     currentV:=sqrt(power(RVelSimTwo.v,2)+power(RVelSimTwo.vn,2));

     //Trajectory Index
     idx := Min(2, traj.count-1); //target is the 2nd trajectory point from current localization

     //calcula o x e y final e o delta teta total assim como o teta ponto
     tx := traj.pts[idx].x;
     ty := traj.pts[idx].y;
     //tx:=4;
     //ty:=1.5;

     //Calculate optimal tau
     tau := 0.55*currentV-0.1;
     //limit minimal tau
     if tau < 0.4 then
       tau := 0.4;

     //initial time
     //algTime := GetTickCount;
     //AlgItCount:=AlgItCount+1;

     //Load optimizer and simulation parameters
     loadConfig();
     //loadparameters do modelo (l, vaxm e tao)
     resetModel();

     //U vectors and matrixes
     Uaux.SetSize(3,SimParameters.Nu);
     Uref.SetSize(3,SimParameters.Nu);
     Usteps.SetSize(3,SimParameters.Nu*6);
     Ubest.setSize(3,SimParameters.Nu);

     //Set initial reference input as current robot speed reference
     for i:= 0 to (SimParameters.Nu-1) do begin
       Uref.setv(0,i,RVelSimTwo.v);    //v
       Uref.setv(1,i,RVelSimTwo.vn);   //vn
       Uref.setv(2,i,RVelSimTwo.w);    //w
     end;

     //Get current motor speeds (estimated from speed references)
     //IK(RVelSimTwo.v,RVelSimTwo.vn,RVelSimTwo.w,v1,v2,v3);

     //Current state of SimRobot is actual robot
     //(keep motor speeds, as they are updated at the end of the mpc loop)
     simRobot.Pose.x := RPoseSimTwo.x;
     simRobot.Pose.y := RPoseSimTwo.y;
     simRobot.Pose.theta := RPoseSimTwo.theta;
     //simRobot.Pose.theta := traj.pts[idx].teta;
     simRobot.Vel.v:=RVelSimTwo.v;
     simRobot.Vel.vn:=RVelSimTwo.vn;
     simRobot.Vel.w:=RVelSimTwo.w;
     simRobot.MotorState[0].v := RVelSimTwo.v1;
     simRobot.MotorState[1].v := RVelSimTwo.v2;
     simRobot.MotorState[2].v := RVelSimTwo.v3;

     //save initial motor speeds
     initMotorSpeeds[0] := SimRobot.MotorState[0].v;
     initMotorSpeeds[1] := SimRobot.MotorState[1].v;
     initMotorSpeeds[2] := SimRobot.MotorState[2].v;

     //if speed is variable get speed reference from trajectory points
     //if traj.varSpeed then begin
        //vRefModule:= traj.pts[traj.index].vRef;
//        vRefModule:= traj.pts[idx].vRef;
     //end;

     //Calculate reference trajectory for this step
     calcRefTraj(traj,refTraj,vRefModule);

     //Limit wheel speed references in case of motor saturation (update references)
     scaleForSaturation(Uref);

     //Base cost values
     Jcurrent := predSimulator(SimRobot,Uref,refTraj);
     Jprev := Jcurrent + 1;
     Jbest := Jcurrent;

     //---------------------------------------------------
     //              Optimization loop
     //---------------------------------------------------
     while (OptData.iterationCount < 5) or (OptData.iterationCount < OptParameters.MaxIterations)
                                                    and (Jcurrent > OptParameters.Jstop) do begin

           //get Usteps matrix
           calcUSteps(Uref,Usteps);

           //Calculate Jsteps vector (do one simulation for each input set)
           for k:= 0 to (SimParameters.Nu-1) do begin
             for i:= 0 to 5 do begin
                 //get current U for simulation from Usteps
                 for m := 0 to (SimParameters.Nu-1) do begin
                     if m=k then begin
                       Uaux.setv(0,m,Usteps.getv(0,i+6*k));
                       Uaux.setv(1,m,Usteps.getv(1,i+6*k));
                       Uaux.setv(2,m,Usteps.getv(2,i+6*k));
                     end else begin
                       Uaux.setv(0,m,Uref.getv(0,0));
                       Uaux.setv(1,m,Uref.getv(1,0));
                       Uaux.setv(2,m,Uref.getv(2,0))
                     end;
                 end;

                 //Reset robot initial state for each simulation
                 simRobot.Pose.x := RPoseSimTwo.x;
                 simRobot.Pose.y := RPoseSimTwo.y;
                 simRobot.Pose.theta := RPoseSimTwo.theta;
                 //simRobot.Pose.theta := traj.pts[idx].teta;
                 simRobot.Vel.v:=RVelSimTwo.v;
                 simRobot.Vel.vn:=RVelSimTwo.vn;
                 simRobot.Vel.w:=RVelSimTwo.w;
                 SimRobot.MotorState[0].v := initMotorSpeeds[0];
                 SimRobot.MotorState[1].v := initMotorSpeeds[1];
                 SimRobot.MotorState[2].v := initMotorSpeeds[2];

                 //Limit wheel speed references in case of motor saturation (update references)
                 scaleForSaturation(Uaux);

                 //Do simulation with current Uaux and add to Jsteps vector
                 //Switches between trajectory controller and formation controller
                 J := predSimulator(SimRobot,Uaux,refTraj);

                 //Add J to Jsteps
                 OptData.Jsteps.setv(i,0,J);
             end;
           end;

           //Compute gradient of J from Jsteps
           calcGradient(OptData.Jsteps, OptData.Jgradient, OptData.Jgradient_prev);

           //Minimization algorithm
           calcSteepestDescentStep(OptData.Jgradient,Uref);

           //previous costs and inputs
           Jprev := Jcurrent;

           //Reset robot initial position
           simRobot.Pose.x := RPoseSimTwo.x;
           simRobot.Pose.y := RPoseSimTwo.y;
           simRobot.Pose.theta := RPoseSimTwo.theta;
           //simRobot.Pose.theta := traj.pts[idx].teta;
           simRobot.Vel.v:=RVelSimTwo.v;
           simRobot.Vel.vn:=RVelSimTwo.vn;
           simRobot.Vel.w:=RVelSimTwo.w;
           SimRobot.MotorState[0].v := initMotorSpeeds[0];
           SimRobot.MotorState[1].v := initMotorSpeeds[1];
           SimRobot.MotorState[2].v := initMotorSpeeds[2];

          //Limit wheel speed references in case of motor saturation (update references)
          scaleForSaturation(Uref);

           //Calculate new current cost (do simulation)
           Jcurrent := predSimulator(SimRobot,Uref,refTraj);

           //Update JBest
           if Jcurrent < Jbest then begin
              Jbest := Jcurrent;
              Ubest.setV(0,0,Uref.getv(0,0));
              Ubest.setV(1,0,Uref.getv(1,0));
              Ubest.setV(2,0,Uref.getv(2,0));
           end;
           OptData.iterationCount += 1;
     end;

     //Set best output
     //RDataDesejada.vx := Ubest.getv(0,0);
     //RDataDesejada.vy := Ubest.getv(1,0);
     RDataDesejada.v := Ubest.getv(0,0);
     RDataDesejada.vn := Ubest.getv(1,0);
     //RDataDesejada.w := Ubest.getv(2,0);
     RDataDesejada.w :=0;

     //VxyToVVn(RPoseSimTwo.theta, RDataDesejada.vx, RDataDesejada.vy, RDataDesejada.v, RDataDesejada.vn);

     FControl.Edit31.text := Format('%.1f', [tx]);
     FControl.Edit32.text := Format('%.1f', [ty]);
     FControl.Edit33.text := Format('%.1f', [RDataDesejada.v]);
     FControl.Edit34.text := Format('%.1f', [RDataDesejada.vn]);
     FControl.Edit35.text := Format('%.1f', [RDataDesejada.w]);

     //envia o valor de v, vn e pw finais para o robô
     if (Dist(tx-RPoseSimTwo.x, ty-RPoseSimTwo.y)<0.05)or((RPoseSimTwo.x=4)and(RPoseSimTwo.y=1.5)) then begin
       SetTacticCommandsInRobotRef(0, 0, 0);
       stop;
       exit;
     end else begin
       SetTacticCommandsInRobotRef(RDataDesejada.v, RDataDesejada.vn, RDataDesejada.w);
     end;
end;

//----------------------------------------------------------------------------
//
// SimRobotMovement()
//
//----------------------------------------------------------------------------
// Updates the simulated robot's position taking into account the current
// position, and input values. Uses 10ms intervals (run 4 times for 40 ms)
//----------------------------------------------------------------------------
procedure simRobotMovement(var Robot : TSimRobot; var v,vn,w : double);
var
   cteta,steta : double;
begin
          cteta := cos(Robot.Pose.theta);
          steta := sin(Robot.Pose.theta);


          if Robot.Pose.theta > pi then
             Robot.Pose.theta := Robot.Pose.theta - 2*pi;

          Robot.Pose.theta := Robot.Pose.theta + simTimeStep*w;
          Robot.Pose.x := Robot.Pose.x + simTimeStep*(v*cteta-vn*steta);
          Robot.Pose.y := Robot.Pose.y + simTimeStep*(v*steta+vn*cteta);

end;

//----------------------------------------------------------------------------
//
// predSimulator()
//
//---------------------------------------------------------------------------
// Simulates the evolution of the moving robot across a path with given set
// of input values.
//
// RETURNS: J, the value of the cost function obtained with the current input
//          values.
//----------------------------------------------------------------------------
function predSimulator(var Robot : TSimRobot; var U : TDMatrix; refTraj : TTrajectory) : double;
var
   sum_x,sum_y,sum_teta : double;
   deltaU : double;
   i,j : integer;
   v_ref,vn_ref,w_ref : double;
   v_real,vn_real,w_real : double;
   U_limit : TDMatrix;
begin

     //initialize
     sum_x := 0;
     sum_y := 0;
     sum_teta := 0;

     //simulate robot progression for prediction instant, calculate J
     for i:= SimParameters.N1 to SimParameters.N2 do begin

         //change control signal for control horizon
         if i <= SimParameters.Nu then begin
              v_ref := U.getv(0,i-1);
              vn_ref := U.getv(1,i-1);
              w_ref := U.getv(2,i-1);
         end else begin
              v_ref := U.getv(0,SimParameters.Nu-1);
              vn_ref := U.getv(1,SimParameters.Nu-1);
              w_ref := U.getv(2,SimParameters.Nu-1);
         end;

         //Simulate behavior for the next 40 ms (simulations timestep is 10 ms)
         for j:= 0 to 3 do begin

             //Dynamics simulation (v references -> v real)
                 v_real := v_ref;
                 vn_real := vn_ref;
                 w_real := w_ref;

             //Move simulated robot with v real values
             simRobotMovement(Robot,v_real,vn_real,w_real);
         end;

         //calculate errors
         sum_x := sum_x + (refTraj.pts[i].x-Robot.Pose.x)*(refTraj.pts[i].x-Robot.Pose.x);
         sum_y := sum_y + (refTraj.pts[i].y-Robot.Pose.y)*(refTraj.pts[i].y-Robot.Pose.y);
         sum_teta := sum_teta + DiffAngle(refTraj.pts[i].teta,
              Robot.Pose.theta)*DiffAngle(refTraj.pts[i].teta,Robot.Pose.theta);


     end;

     deltaU:= power((Robot.Vel.v-U.getv(0,0)),2)+
             power((Robot.Vel.vn-U.getv(1,0)),2)+
              power((Robot.Vel.w-U.getv(2,0)),2);

     //Output cost
     result := SimParameters.lambda[0]*(sum_x+sum_y)+SimParameters.lambda[1]*sum_teta+
                                              SimParameters.lambda[2]*(deltaU*deltaU);
end;

//----------------------------------------------------------------------------
//
// calcUSteps()
//
//----------------------------------------------------------------------------
// Generates the Ustep matrix, whose columns are used inputs to the simulator
// to calculate the Js vector
//----------------------------------------------------------------------------
procedure calcUSteps(var U, deltaU : TDMatrix);
var
   i : integer;
begin

    for i:= 0 to (SimParameters.Nu-1) do begin
      //col 1 (v+d,vn,w)
      deltaU.setv(0,0+6*i,U.getv(0,i)+OptParameters.delta);
      deltaU.setv(1,0+6*i,U.getv(1,i));
      deltaU.setv(2,0+6*i,U.getv(2,i));
      //col 2 (v-d,vn,w)
      deltaU.setv(0,1+6*i,U.getv(0,i)-OptParameters.delta);
      deltaU.setv(1,1+6*i,U.getv(1,i));
      deltaU.setv(2,1+6*i,U.getv(2,i));
      //col 3 (v,vn+EditDelta,w)
      deltaU.setv(0,2+6*i,U.getv(0,i));
      deltaU.setv(1,2+6*i,U.getv(1,i)+OptParameters.delta);
      deltaU.setv(2,2+6*i,U.getv(2,i));
      //col 4 ...
      deltaU.setv(0,3+6*i,U.getv(0,i));
      deltaU.setv(1,3+6*i,U.getv(1,i)-OptParameters.delta);
      deltaU.setv(2,3+6*i,U.getv(2,i));
      //col 5 ...
      deltaU.setv(0,4+6*i,U.getv(0,0));
      deltaU.setv(1,4+6*i,U.getv(1,0));
      deltaU.setv(2,4+6*i,U.getv(2,0)+OptParameters.delta);
      //col 6 ...
      deltaU.setv(0,5+6*i,U.getv(0,i));
      deltaU.setv(1,5+6*i,U.getv(1,i));
      deltaU.setv(2,5+6*i,U.getv(2,i)-OptParameters.delta);
    end;
end;

//----------------------------------------------------------------------------
//
// calcGradient()
//
//---------------------------------------------------------------------------
// Calculates the gradient of the cost function from the Jsteps vector
//----------------------------------------------------------------------------
procedure calcGradient(var Jsteps, Jgradient, Jgradient_prev : TDmatrix);
var
   i : integer;
begin

     for i := 0 to (SimParameters.Nu-1) do begin
         Jgradient_prev.setv(3*i+0,0,Jgradient.getv(3*i+0,0));
         Jgradient.setv(3*i+0,0,Jsteps.getv(6*i+0,0)-Jsteps.getv(6*i+1,0));
         Jgradient_prev.setv(3*i+1,0,Jgradient.getv(3*i+1,0));
         Jgradient.setv(3*i+1,0,Jsteps.getv(6*i+2,0)-Jsteps.getv(6*i+3,0));
         Jgradient_prev.setv(3*i+2,0,Jgradient.getv(3*i+2,0));
         Jgradient.setv(3*i+2,0,Jsteps.getv(6*i+4,0)-Jsteps.getv(6*i+5,0));
     end;
end;

//----------------------------------------------------------------------------
//
// calcSteepestDescentStep()
//
//---------------------------------------------------------------------------
// Calculates the new U after a minimization loop iteration (steepest descent
// method)
//----------------------------------------------------------------------------
procedure calcSteepestDescentStep(var Jgradient, Uref : TDmatrix);
var
   i : integer;
begin

     for i:= 0 to (SimParameters.Nu-1) do begin
       Uref.setv(0,i,Uref.getv(0,i) - OptParameters.alpha*Jgradient.getv(0 + 3*i,0));
       Uref.setv(1,i,Uref.getv(1,i) - OptParameters.alpha*Jgradient.getv(1 + 3*i,0));
       Uref.setv(2,i,Uref.getv(2,i) - OptParameters.alpha*Jgradient.getv(2 + 3*i,0));
     end;
end;

//----------------------------------------------------------------------------
//
// loadConfig()
//
//---------------------------------------------------------------------------
// Loads configuration from form
//----------------------------------------------------------------------------
procedure loadConfig();
var
   i : integer;
begin

     //Set model simulation parameters
     SimParameters.N1:=1;
     SimParameters.N2:=10;
     SimParameters.Nu:=2;
     SimParameters.lambda[0] := 2;
     SimParameters.lambda[1] := 1;
     SimParameters.lambda[2] := 0;

     //Set optimizer parameters
     OptParameters.delta:= 0.1;
     OptParameters.MaxIterations:=30;
     OptParameters.Jstop:=0.05;
     OptParameters.alpha :=0.15;          //steepest descent step size

     //Initialize optimizer data structures
     OptData.Jsteps.SetSize(6*SimParameters.Nu,1);
     OptData.Jgradient.SetSize(3*SimParameters.Nu,1);
     OptData.Jgradient_prev.SetSize(3*SimParameters.Nu,1);
     OptData.Jstep_prev.SetSize(3*SimParameters.Nu,1);
     OptData.iterationCount:= 0;
end;

//----------------------------------------------------------------------------
//
// calcRefTraj()
//
//---------------------------------------------------------------------------
// Calculates reference trajectory for controller from given trajectory, desired
// speed, and robot parameters
//----------------------------------------------------------------------------
procedure calcRefTraj(var traj,trajPred: TTrajectory; V : double);
var
   teta, dl, dtotal: double;
   dSegments, segmentSize : double;
   tetaInc : double;
   deltaD : double;
   i, locTrajIndex : integer;
begin

     //initializations
     i := 0;
     deltaD := V*0.04;
     locTrajIndex := traj.index;
     dTotal := 0;

     //angle of current segment to world
     teta := atan2(traj.pts[locTrajIndex+1].y-traj.pts[locTrajIndex].y,
                  traj.pts[locTrajIndex+1].x-traj.pts[locTrajIndex].x);

     //distance of robot robot along the current segment
     dl := DistPointInLine(traj.pts[locTrajIndex].x,traj.pts[locTrajIndex].y,
                       traj.pts[locTrajIndex+1].x,traj.pts[locTrajIndex+1].y,
                                            SimRobot.Pose.x,SimRobot.Pose.y);
     segmentSize := dist(traj.pts[locTrajIndex+1].x-traj.pts[locTrajIndex].x,
                        traj.pts[locTrajIndex+1].y-traj.pts[locTrajIndex].y);

     //sum of distances from segments (of main trajectory)
     dSegments := segmentSize - dl;

     //initial point of predictive controller trajectory
     trajPred.pts[i].x:= traj.pts[locTrajIndex].x + dl*cos(teta);
     trajPred.pts[i].y:= traj.pts[locTrajIndex].y + dl*sin(teta);
     trajPred.pts[i].teta := SimRobot.Pose.theta;

     //first tetaInc
     tetaInc := (traj.pts[locTrajIndex+1].teta - SimRobot.Pose.theta)/(dSegments/deltaD);

     //build new trajectory
     for i:=1 to SimParameters.N2 do begin

         //reached the end of the trajectory
         if(locTrajIndex >= traj.count-1) then begin

             trajPred.pts[i].x := traj.pts[traj.count-1].x;
             trajPred.pts[i].y := traj.pts[traj.count-1].y;
             trajPred.pts[i].teta := traj.pts[traj.count-1].teta;

         end else begin

             //add trajectory points along current segment
             trajPred.pts[i].x := trajPred.pts[i-1].x + deltaD*cos(teta);
             trajPred.pts[i].y := trajPred.pts[i-1].y + deltaD*sin(teta);
             trajPred.pts[i].teta := trajPred.pts[i-1].teta+tetaInc;

             dTotal := dTotal + deltaD;

             //change segment of the main trajectory that's being tracked
             if dtotal >= dSegments then begin

                locTrajIndex:=locTrajIndex+1;
                teta := atan2(traj.pts[locTrajIndex+1].y-traj.pts[locTrajIndex].y,
                             traj.pts[locTrajIndex+1].x-traj.pts[locTrajIndex].x);
                segmentSize := dist(traj.pts[locTrajIndex+1].x-traj.pts[locTrajIndex].x,
                                   traj.pts[locTrajIndex+1].y-traj.pts[locTrajIndex].y);

                //prevent division by zero
                if segmentSize = 0 then
                   segmentSize := 0.00000001;

                //add point (already in next segment)
                trajPred.pts[i].x := traj.pts[locTrajIndex].x + (dTotal-dSegments)*cos(teta);
                trajPred.pts[i].y := traj.pts[locTrajIndex].y + (dTotal-dSegments)*sin(teta);
                trajPred.pts[i].teta := traj.pts[locTrajIndex].teta;

                tetaInc := (traj.pts[locTrajIndex+1].teta - traj.pts[locTrajIndex].teta)/(segmentSize/deltaD);
                dSegments := dSegments + segmentSize;

                if i = 1 then
                   traj.index := traj.index+1;
                end;
         end;

     trajPred.count:=i+1;
     end;

   //handle teta references
   for i := 0 to SimParameters.N2 do begin

       if trajPred.pts[i].teta > pi then
              trajPred.pts[i].teta := -2*pi + trajPred.pts[i].teta;

   end;
end;

//-------------------------------------------------------------------------
// ProportionalSat
//
//-------------------------------------------------------------------------
// Keeps vector directions if norms have to be scaled
//-------------------------------------------------------------------------
procedure proportionalSatMPC(var v1,v2,v3: double; vmax: double);
var maxv,minv: double;
    scale,scalemax,scalemin: double;
begin
  maxv:=Max(v1,Max(v2,v3));
  minv:=Min(v1,Min(v2,v3));

  if maxv>vmax then scalemax:=maxv/vmax else scalemax:=1.0;
  if minv<-vmax then scalemin:=minv/(-vmax) else scalemin:=1.0;

  scale:=Max(scalemin,scalemax);

  v1:=v1/scale;
  v2:=v2/scale;
  v3:=v3/scale;

end;

//-------------------------------------------------------------------------
// scaleForSaturation()
//
//-------------------------------------------------------------------------
// Scales V,Vn,W to prevent saturation
//-------------------------------------------------------------------------
procedure scaleForSaturation(var U : TDmatrix);
var
   v,vn,w,v1,v2,v3,vmax : double;
   i: integer;
begin

     for i:= 0 to (SimParameters.Nu-1) do begin
           v:=U.getv(0,i);
           vn:=U.getv(1,i);
           w:=U.getv(2,i);

           vmax:=SimRobot.Vel.vmax;

           IK(v,vn,w,v1,v2,v3);
           proportionalSatMPC(v1,v2,v3,vmax);
           DK(v1,v2,v3,v,vn,w);

           U.setv(0,i,v);
           U.setv(1,i,vn);
           U.setv(2,i,w);
     end;
end;

function DistPointInLine(x1,y1,x2,y2,x3,y3: double): double;
var
teta,alfa,yp1,d1_3: double;
begin

  // Translacao do vector (px,py) segundo o vector (tx,ty)
  // seguida de Rotacao do angulo teta
  // angulo da recta no mundo
  teta:= atan2(y2-y1,x2-x1);

  // angulo entre a recta e o vector que une o ponto (x1,y1) ao ponto (x3,y3)
  alfa:= atan2(y3-y1,x3-x1)-teta;
  d1_3:=Dist(x3-x1,y3-y1);

  // distancia do ponto (x3,y3) para a linha que passa pelos pontos (x1,y1) e (x2,y2)
  yp1:= d1_3*cos(alfa);
  result := yp1;
end;

//-------------------------------------------------------------------------
// GTXYTCcontroller
//
// Calculates the correct v, vn, and w to follow the trajectory calculated
// before. Limits speeds for deceleration when close to the target to arrive
// to it at the desired speed.
//
//GTXYTCcontroller(speed, sot, traj);
//-------------------------------------------------------------------------
procedure GTXYTCcontroller(speed, speed_on_target: double; var RPose: TRobotPose; var RVel: TRobotSpeeds; var traj: TTrajectory);
var statev, statevn, v, vn, dteta, teta_power: double;
    vmax, pw, tx, ty, segteta: double;
    idx: integer;
    algTime : longword;
begin
  if RPose.x=-4 then begin
    algTime := GetTickCount;
  end;
  idx := Min(2, traj.count-1); //target is the 2nd trajectory point from current localization

  //target x,y,dteta
  tx := traj.pts[idx].x;
  ty := traj.pts[idx].y;
  dteta := DiffAngle(traj.pts[idx].teta, RPose.theta);
  teta_power := traj.pts[idx].teta_power;

  //teta of the direction it has to follow
  segteta := ATan2(ty-RPose.y,tx-RPose.x);

  //calculate new V and Vn
  VxyToVVn(segteta, RVel.vx, RVel.vy, statev, statevn);

  if traj.distance < 0.02 then begin
    vmax := 0;

  end else begin
    //vmax := sqr(speed_on_target) +  0.7*0.7*(traj.distance - 0.02);
    vmax := sqr(speed) +  0.7*0.7*(traj.distance - 0.02);
    if vmax > 0 then begin
      vmax := sqrt(vmax);
      if vmax>speedmax then vmax:=SpeedMax;
    end else vmax := 0;
  end;

  v := speed;
  if v > vmax then v := vmax;
  vn := -statevn * 0.75;
  if vn > vmax then vn := vmax;
  if vn < -vmax then vn := -vmax;

  //calculate w for rotation to desired teta
  if abs(dteta) < 3 * Pi / 180 then begin
    pw := 0;
  end else begin
    pw := 4 * dteta - 0.5 * RVel.w;
    pw := pw * teta_power;
  end;

  VVnToVxy(segteta, v, vn, statev, statevn);

  VxyToVVn(RPose.theta, statev, statevn, v, vn);

  if (Dist(tx-RPose.x, ty-RPose.y)<0.05)or((RPose.x=4)and(RPose.y=1.5)) then begin
    SetTacticCommandsInRobotRef(0, 0, 0);
    algTime := GetTickCount-algTime;
       if (Fcontrol.Edit36.Text=inttostr(0)) then begin
          Fcontrol.Edit36.Text:=inttostr(algTime);
       end;
    stop;
    exit;
  end else begin
    SetTacticCommandsInRobotRef(v, vn, pw);
  end;
end;


//----------------------------------------------------------------------------
//
// DK()
//
//----------------------------------------------------------------------------
// Direct Kinematics(v1,v2,v3->v,vn,w)
//
//----------------------------------------------------------------------------
procedure DK(v1, v2, v3: double; var v, vn, w: double);
begin
     v := (sqrt(3)/3)*(v1-v2);
     vn := (1/3)*(v1+v2)-(2/3)*v3;
     w := (1/(0.195*3))*(v1+v2+v3);
end;


end.
