unit Utils;

{$mode objfpc}{$H+}

interface

uses Classes,sysutils,StdCtrls,graphics,Math, Unix, dynmatrix, Main;

type
  TMVEst=record
    Vmean,Vcov: double;
    speed: double;
    n: integer;
  end;

  TLinearReg=record
    Sx,Sy,Sxy,Sx2: double;
    a,b: double;
    N:integer;
  end;

  TCMarkerType=(mtCross,mtCircle,mtSquare,mtXis);


procedure mse(var e:double; m1,m2: TDMatrix; N: integer);
procedure ClearLinearReg(var TL: TLinearReg);
procedure AddXYtoLinearReg(var TL: TLinearReg; x,y: double);
procedure CalcLinearReg(var TL: TLinearReg);

procedure MVEstInit(var MV: TMVEst);
procedure MVEstAddValue(var MV: TMVEst; v: double);

procedure ZeroMemory(Ptr: Pointer; Len: integer);
procedure CopyMemory(DPtr,SPtr: Pointer; Len: integer);

function GetTickCount: LongWord;

function strtofloatDef(s: string; def: double): double;
function EditToFloatDef(edit: TEdit; default: double): double;
procedure ParseString(s,sep: string; sl: TStrings);

function FastATan2(y,x: double): double;

//function Atan2(y,x: double):double;
function FMod(x,d: double): double;
function DiffAngle(a1,a2: double): double;
function AngleThreePoints(x1,y1,x2,y2,x3,y3: double): double;
function DistPoint2Line(x1,y1,x2,y2,x3,y3: double): double;
function DistPointInLine(x1,y1,x2,y2,x3,y3: double): double;
function Dist(x,y: double): double;
function ATan2(y,x: double): double;
function Sign(a: double): double;
function Sat(a,limit: double): double;
function Wcontrol(Tref,TRobot,Lambda: double): double;

function IncWrap(var v: integer; size: integer; step:integer=1): integer;
function DecWrap(var v: integer; size: integer; step:integer=1): integer;
procedure SwapInts(var v1, v2: integer);

//procedure OptimalMean(a,cov_a, b,cov_b: double; var m,cov_m: double);
function Rad(xw: double):double;
function deg(xw: double):double;

//procedure TranslateAndRotateTraj(var traj: TTrajectory; RS: TRobotState);
//procedure RotateAndTranslateTraj(var traj: TTrajectory; RS: TRobotState);
procedure TranslateAndRotate(var rx,ry: double; px,py,tx,ty,teta: double);
procedure TranslateAndRotateInv(var rx,ry: double; px,py,tx,ty,teta: double);
procedure RotateAndTranslate(var rx,ry: double; px,py,tx,ty,teta: double);
procedure RotateAndTranslate1(var rx,ry,rvx,rvy: double; px,py,vx,vy,tx,ty,teta,v,vn: double);
procedure calcNextBallXY(var rx,ry: double; px,py,vx,vy,tx,ty,teta: double;forwardStep:integer);

function InternalProductCosine(v1x,v1y,v2x,v2y: double): double;
function NormalizeAngle(ang: double): double;

function InFrustum(xc,yc,xp,yp,ang,widthAng: double): boolean;
procedure RelocateAbsCoordItem(var x,y: Double; Ox,Oy,Oteta,Nx,Ny,Nteta: double);

procedure DrawCovElipse(x,y,cov_x,cov_y, cov_xy: double; n: integer; CNV: TCanvas);
procedure RotateCov( const incov_x,incov_y,incov_xy: double; out cov_x,cov_y, cov_xy: double; teta: double);

function AverageAngle(ang1,ang2: double): double;
function CheckPoint(xgoal,ygoal,xrobot,yrobot: double;  d: double ): integer;
function CheckPointEnd(xgoal,ygoal,xrobot,yrobot: double): integer;

//function NormalizeAngle(v: double): double;
procedure NormalizeVector(var x,y: double);
function MidAngle(a1,a2: double): double;
function FiltAngle(a1, a2, lamb: double): double;

//manuel
function GetTimeAndDate(tipo:integer) : string;
function innerProduct(x1,y1,x2,y2:double):double;


//AGM02ABR
//procedure ObsAvoid(d,RSx,RSy,RSteta: double; var x,y: double);
//function Toler(value, true_value, toler_perc: double): boolean;
//function SameInc(teta1,teta2,toler:double): boolean;
//function SegLineMatch(x1,y1,d1,w1,x2,y2,d2,w2,teta: double): double;

// andre
//procedure AvoidObstacles(Obsx,Obsy,RSx,RSy,RSteta: double; var traj: TTrajectory);
{function FreeDirection: integer; // verifica com o radar a direcao livre (right,left,ahead)
procedure LinesIntersect(const x1,y1,x2,y2: double; // first line
                         const x3,y3,x4,y4: double; // second line
                         var code : integer; // =0 OK; =1 lines parallel
                         var x,y : double); // intersection point  }

type
  QSortCmpFunc=function (var a,b): integer;
//  pbyte=^byte;

procedure QSort(base: pointer; num_elem,size_elem: integer; func: QSortCmpFunc);

//function CompressString(src: string): string;
//function DecompressString(src: string): string;

implementation

uses {ZLib,Control,Trajec,}Field,Robots;

var FirstTimeValSec: LongInt;

{function NormalizeAngle(v: double): double;
begin
  if v<0 then v:=v+(Int((-v)/(2*pi))+2)*2*Pi;
  result:=fmod(v+Pi,2*Pi)-Pi;
end; }

procedure NormalizeVector(var x,y: double);
var d: double;
begin
  d:=Dist(x,y);
  if abs(d)<1e-6 then begin
    x:=1;
    y:=0;
  end else begin
    x:=x/d;
    y:=y/d;
  end;
end;


{  intersection of the two infinite lines rather than the line segments }
procedure LinesIntersect(const x1,y1,x2,y2: double; { first line}
                         const x3,y3,x4,y4: double; { second line }
                         var code : integer; { =0 OK; =1 lines parallel}
                         var x,y : double); { intersection point }

var
    a1, a2, b1, b2, c1, c2 : double; { Coefficients of line eqns.}
    denom : double;

begin
  a1:= y2-y1;
  b1:= x1-x2;
  c1:= x2*y1 - x1*y2;  { a1*x + b1*y + c1 = 0 is line 1 }

  a2:= y4-y3;
  b2:= x3-x4;
  c2:= x4*y3 - x3*y4;  { a2*x + b2*y + c2 = 0 is line 2 }

  denom:= a1*b2 - a2*b1;
  if denom = 0 then
    begin
      code:=1;
      exit;
    end;

  x:=(b1*c2 - b2*c1)/denom;
  y:=(a2*c1 - a1*c2)/denom;
  code:=0
end;


{function CompressString(src: string): string;
var outbuf: pointer;
    outbytes: integer;
begin
  CompressBuf(pchar(src),length(src),outbuf,outbytes);
  result:=StringofChar(' ',outbytes);
  Move(outbuf^,result[1],outbytes);
  FreeMem(outbuf);
end;

function DecompressString(src: string): string;
var outbuf: pointer;
    outbytes: integer;
begin
  DecompressBuf(pchar(src),length(src),4096,outbuf,outbytes);
  result:=StringofChar(' ',outbytes);
  Move(outbuf^,result[1],outbytes);
  FreeMem(outbuf);
end;}


function strtofloatDef(s: string; def: double): double;
begin
  try
    result:=strtofloat(s);
  except
    result:=def;
  end;
end;

function EditToFloatDef(edit: TEdit; default: double): double;
begin
  if edit.text='*' then begin
    result:=default;
    edit.text:=Format('%.8g',[default]);
    exit;
  end;
  try
    result:=strtofloat(edit.text);
  except
    result:=default;
    edit.text:=Format('%.8g',[default]);
  end;
end;


procedure ParseString(s,sep: string; sl: TStrings);
var p,i,last: integer;
begin
  sl.Clear;
  last:=1;
  for i:=1 to length(s) do begin
    p:=Pos(s[i],sep);
    if p>0 then begin
      if i<>last then
        sl.add(copy(s,last,i-last));
      last:=i+1;
    end;
  end;
  if last<=length(s) then
    sl.add(copy(s,last,length(s)-last+1));
end;

// ---------------------------------------------------------
//     Math functions

function Dist(x,y: double): double;
begin
  result:=sqrt(x*x+y*y);
end;

function CheckPoint(xgoal,ygoal,xrobot,yrobot: double; d: double ): integer;
begin

  // FMain.Memo1.Lines.Add(floattostr( Dist(xgoal-xrobot,ygoal-yrobot)));


     if Dist(xgoal-xrobot,ygoal-yrobot) < d
     then result := 1
     else result := 0;



end;

function CheckPointEnd(xgoal,ygoal,xrobot,yrobot: double): integer;
begin

 //  FMain.Memo1.Lines.Add(floattostr( Dist(xgoal-xrobot,ygoal-yrobot)));


     if Dist(xgoal-xrobot,ygoal-yrobot) < 0.03
     then result := 1
     else result := 0;



end;


function FMod(x,d: double): double;
begin
  result:=Frac(x/d)*d;
end;



function AngleThreePoints(x1,y1,x2,y2,x3,y3: double): double;
var
a,b: double;
begin
  a:= atan2(y3-y1,x3-x1); // distancia
  b:= atan2(y2-y1,x2-x1); // distancia

  result := diffangle(a,b);

end;

function Wcontrol(Tref,TRobot,Lambda: double): double;
begin
  result := sat(lambda*(Tref-TRobot),1.5);
end;

  (*
function DistPoint2Line(x1,y1,x2,y2,x3,y3: double): double;
var
teta,yp1: double;
begin

//procedure TranslateAndRotate(var rx,ry: double; px,py,tx,ty,teta: double);
// Translacao do vector (px,py) segundo o vector (tx,ty)
// seguida de Rotacao do angulo teta


  // angulo da recta
  teta:= atan2(y2-y1,x2-x1);


  // rotacao de um dos pontos da recta de modo à recta ficar horizontal
  yp1:= (x1-x3)*sin(-teta)+(y1-y3)*cos(-teta);

  if diffangle(teta,atan2(y3-y1,x3-x1)) > 0
    then yp1:=abs(yp1)
    else yp1:=-abs(yp1);

  result := yp1;
end;
 *)

function DistPoint2Line(x1,y1,x2,y2,x3,y3: double): double;
var
teta,alfa,yp1,d1_3: double;
begin

//procedure TranslateAndRotate(var rx,ry: double; px,py,tx,ty,teta: double);
// Translacao do vector (px,py) segundo o vector (tx,ty)
// seguida de Rotacao do angulo teta

  // angulo da recta no mundo
  teta:= atan2(y2-y1,x2-x1);

  // angulo entre a recta e o vector que une o ponto (x1,y1) ao ponto (x3,y3)
  alfa:= atan2(y3-y1,x3-x1)-teta;

  d1_3:=Dist(x3-x1,y3-y1);

  // distancia do ponto (x3,y3) para a linha que passa pelos pontos (x1,y1) e (x2,y2)
  yp1:= d1_3*sin(alfa);

  result := yp1;
end;


function DistPointInLine(x1,y1,x2,y2,x3,y3: double): double;
var
teta,alfa,yp1,d1_3: double;
begin

//procedure TranslateAndRotate(var rx,ry: double; px,py,tx,ty,teta: double);
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

function FastArcTan(x: double): double;
begin
  result :=0;
end;

function FastATan2(y,x: double): double;
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
      result:=FastArcTan(y/x)+pi;
      if result>pi then result:=result-2*pi;
    end else begin
      result:=FastArcTan(y/x);
    end;
  end else begin
    if y<0 then begin
      result:=FastArcTan(-x/y)-pi/2
    end else begin
      result:=FastArcTan(-x/y)+pi/2;
    end;
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

function Sign(a: double): double;
begin
  if a<0 then result:=-1 else result:=1;
end;

function Sat(a,limit: double): double;
begin
 if a>limit then a:=limit;
 if a<-limit then a:=-limit;
 result:=a;
end;


function IncWrap(var v: integer; size: integer; step:integer=1): integer;
begin
  inc(v,step);
  if v>=size then v:=v-size;
  Result:=v;
end;

function DecWrap(var v: integer; size: integer; step:integer=1): integer;
begin
  dec(v,step);
  if v<0 then v:=v+size;
  Result:=v;
end;

procedure SwapInts(var v1, v2: integer);
var t: integer;
begin
  t:=v1;
  v1:=v2;
  v2:=t;
end;


{
function Atan2(y,x: double):double;
begin
  if abs(y)<abs(x) then begin
    result:=arctan2(y,x);
  end else if x>0 then begin
    result:=arctan2(-x,y)-pi/2;
  end else begin
    result:=arctan(x,-y)+pi/2;
    if result>pi then result:=result-2*pi;
  end;
end;

procedure OptimalMean(a,cov_a, b,cov_b: double; var m,cov_m: double);
var ca_plus_cb: double;
begin
  ca_plus_cb:=cov_a+cov_b;
  if ca_plus_cb=0 then ca_plus_cb:=1;
  m:=(b*cov_a+a*cov_b)/ca_plus_cb;
  cov_m:=(cov_a*cov_b)/ca_plus_cb;
end;
}

function Rad(xw: double):double;
begin
  result:=xw*(pi/180);
end;

function deg(xw: double):double;
begin
  result:=xw*(180/pi);
end;

// ordena de forma crescente
//function CompCenter(var a,b): integer;
//begin
//  if TRecord(a).value<TRecord(b).value then result:=-1
//  else if TRecord(a).value>TRecord(b).value then result:=1
//  else result:=0;
//end;



procedure QSortSwapElem(a,b: pbyte; size: integer);
var i: integer;
    tmp: byte;
begin
  if a=b then exit;
  for i:=0 to size-1 do begin
    tmp:=a^;
    a^:=b^;
    b^:=tmp;
    inc(a);
    inc(b);
  end;
end;

procedure QSortSub(base: pointer; num_elem,size_elem: integer; func: QSortCmpFunc; iLo, iHi: integer);
var Lo,Hi: integer;
    MidPtr: pbyte;
begin
  Lo := iLo;
  Hi := iHi;
  MidPtr := pbyte(LongWord(base)+LongWord(((iLo + iHi) div 2)*size_elem));
  repeat
    while func(pbyte(LongWord(base)+LongWord(Lo*size_elem))^,MidPtr^)<0 do Inc(Lo);
    while func(pbyte(LongWord(base)+LongWord(Hi*size_elem))^,MidPtr^)>0 do Dec(Hi);
    if Lo <= Hi then
    begin
      QSortSwapElem(
        pbyte(LongWord(base)+LongWord(Lo*size_elem)),
        pbyte(LongWord(base)+LongWord(Hi*size_elem)),
        size_elem);
      Inc(Lo);
      Dec(Hi);
    end;
  until Lo > Hi;
  if Hi > iLo then QSortSub(base,num_elem,size_elem,func, iLo, Hi);
  if Lo < iHi then QSortSub(base,num_elem,size_elem,func, Lo, iHi);
end;

procedure QSort(base: pointer; num_elem,size_elem: integer; func: QSortCmpFunc);
begin
  if num_elem<2 then exit;
  QSortSub(base,num_elem,size_elem,func,0,num_elem-1);
end;



function InternalProductCosine(v1x,v1y,v2x,v2y: double): double;
var d: double;
begin
  d:=dist(v1x,v1y)*dist(v2x,v2y);
  if abs(d)<1e-6 then result:=-1
  else result:=(v1x*v2x+v1y*v2y)/d;
end;

{procedure TranslateAndRotateTraj(var traj: TTrajectory; RS: TRobotState);
var
vx,vy: double;
i: integer;
begin
// Translacao do vector (px,py) segundo o vector (tx,ty)
// seguida de Rotacao do angulo teta

  for i:=0 to traj.Npoints-1 do
  begin
    vx:=traj.x[i]+RS.x;   // translate traj
    vy:=traj.y[i]+RS.y;
  //   traj.teta[i]:= traj.teta[i]+RS.teta;
    traj.x[i]:=vx*cos(RS.teta)-vy*sin(RS.teta);  // rotate traj
    traj.y[i]:=vx*sin(RS.teta)+vy*cos(RS.teta);
  end;


 // vx:=px+tx;
 // vy:=py+ty;
 // rx:=vx*cos(teta)-vy*sin(teta);
  //ry:=vx*sin(teta)+vy*cos(teta);
end;


procedure RotateAndTranslateTraj(var traj: TTrajectory; RS: TRobotState);
var
vx,vy: double;
i: integer;
begin
// Translacao do vector (px,py) segundo o vector (tx,ty)
// seguida de Rotacao do angulo teta

  for i:=0 to traj.Npoints-1 do
  begin
    vx:=traj.x[i]*cos(RS.teta)-traj.y[i]*sin(RS.teta);  // rotate traj
    vy:=traj.x[i]*sin(RS.teta)+traj.y[i]*cos(RS.teta);
    traj.x[i]:=vx+RS.x;   // translate traj
    traj.y[i]:=vy+RS.y;
    traj.teta[i]:= traj.teta[i]+RS.teta;
  end;


 // vx:=px+tx;
 // vy:=py+ty;
 // rx:=vx*cos(teta)-vy*sin(teta);
  //ry:=vx*sin(teta)+vy*cos(teta);
end; }

procedure TranslateAndRotate(var rx,ry: double; px,py,tx,ty,teta: double);
var vx,vy: double;
begin
// Translacao do vector (px,py) segundo o vector (tx,ty)
// seguida de Rotacao do angulo teta

  vx:=px+tx;
  vy:=py+ty;
  rx:=vx*cos(teta)-vy*sin(teta);
  ry:=vx*sin(teta)+vy*cos(teta);
end;


procedure TranslateAndRotateInv(var rx,ry: double; px,py,tx,ty,teta: double);
var vx,vy: double;
begin
// Translacao do vector (px,py) segundo o vector (tx,ty)
// seguida de Rotacao do angulo teta
// com matriz inversa
  vx:=px+tx;
  vy:=py+ty;

  rx:=vx*cos(teta)+vy*sin(teta);
  ry:=vx*-sin(teta)+vy*cos(teta);

end;


procedure RotateAndTranslate(var rx,ry: double; px,py,tx,ty,teta: double);
var vx,vy: double;
begin
// Rotacao do vector (px,py) do angulo teta seguida de
// Translacao segundo o vector (tx,ty)

  vx:=px*cos(teta)-py*sin(teta);
  vy:=px*sin(teta)+py*cos(teta);
  rx:=vx+tx;
  ry:=vy+ty;
end;

procedure RotateAndTranslate1(var rx,ry,rvx,rvy: double; px,py,vx,vy,tx,ty,teta,v,vn: double);
var xrot,yrot: double;
begin
  //posicao
  xrot:=px*cos(teta)-py*sin(teta);
  yrot:=px*sin(teta)+py*cos(teta);
  rx:=xrot+tx;
  ry:=yrot+ty;
  //velocidade
  rvx:=(vx-v)*cos(teta)-(vy-vn)*sin(teta);
  rvy:=(vx-v)*sin(teta)+(vy-vn)*cos(teta);
end;

procedure calcNextBallXY(var rx, ry: double; px, py, vx, vy, tx, ty,
  teta: double; forwardStep: integer);
var xrot,yrot: double;
    x_next,y_next:double;
begin
  x_next:=px+vx*forwardStep*0.025;
  y_next:=py+vy*forwardStep*0.025;
  //posicao
  xrot:=x_next*cos(teta)-y_next*sin(teta);
  yrot:=x_next*sin(teta)+y_next*cos(teta);
  rx:=xrot+tx;
  ry:=yrot+ty;
end;

function NormalizeAngle(ang: double): double;
var a: double;
begin
  a:=FMod(ang+Pi,2*Pi);
  if a<0 then result:=a+Pi
  else result:=a-Pi;
end;

function InFrustum(xc,yc,xp,yp,ang,widthAng: double): boolean;
begin
  result:=2*abs(DiffAngle(atan2(yp-yc,xp-xc),ang))<widthAng;
end;

procedure RelocateAbsCoordItem(var x,y: Double; Ox,Oy,Oteta,Nx,Ny,Nteta: double);
var pdist,pteta: double;
begin
  // update do estado "observado" do item
  // TODO tirar ObsBallState
  with ObsBallState do begin
    pdist:=Dist(x-Ox,y-Oy);
    pteta:=ATan2(y-Oy,x-Ox)-Oteta;
    x:=Nx+cos(pteta+Nteta)*pdist;
    y:=Ny+sin(pteta+Nteta)*pdist;
  end;
end;



procedure DrawCovElipse(x,y,cov_x,cov_y, cov_xy: double; n: integer; CNV: TCanvas);
var i,x1,y1: integer;
//    xr,yr: double;
    alfa,te,ce,se,qxx,qyy,A,phi,xx,yy,wx,wy: double;
begin
  with CNV do begin //tcanvas
      alfa:=0.5;
      te:=0.5*atan2(2*cov_xy,cov_x-cov_y);
      ce:=cos(te);
      se:=sin(te);
      qxx:=sqrt(abs(cov_y*se*se+cov_x*ce*ce+2*cov_xy*se*ce));
      qyy:=sqrt(abs(cov_y*ce*ce+cov_x*se*se-2*cov_xy*se*ce));
  //    qxx:=sqrt(cov_y*se*se+cov_x*ce*ce+cov_xy*se*ce);
  //    qyy:=sqrt(cov_y*ce*ce+cov_x*se*se-cov_xy*se*ce);
      A:=sqrt(-2*ln(1-alfa));
//      n:=10;
      for i:=0 to n do begin
        phi:=i*2*pi/n;
        xx:=A*qxx*cos(phi);
        yy:=A*qyy*sin(phi);
        wx:=ce*xx-se*yy;
        wy:=se*xx+ce*yy;
        WorldToMap(x+wx,y+wy,x1,y1);
        if i=0 then moveto(x1,y1) else lineto(x1,y1);
    end;
  end;
end;


procedure RotateCov( const incov_x,incov_y,incov_xy: double; out cov_x,cov_y, cov_xy: double; teta: double);
var ce,se,t1,t3,t5,t6: double;
begin              // teta=0
  ce:=cos(teta);   // 1
  se:=sin(teta);   // 0
  t1:= ce*ce;      // 1
  t3:= ce*se;      // 0
  t5:= 2.0*t3*incov_xy;  // 0
  t6:= se*se;            // 0
  cov_xy:= t3*incov_x-t6*incov_xy+t1*incov_xy-t3*incov_y;
  cov_x:= t1*incov_x+t5+t6*incov_y;
  cov_y:= t6*incov_x-t5+t1*incov_y;
end;


function RotateCovM( P: TDMatrix): TDMatrix;
begin

end;


procedure ZeroMemory(Ptr: Pointer; Len: integer);
begin
  FillChar(Ptr^,len,0);
end;
//ZeroMemory(@(centers[0,0]),sizeof(centers));

function GetTickCount: LongWord;
var tv: TTimeVal;
begin
  fpGetTimeOfDay(@tv,nil);
  if FirstTimeValSec=0 then FirstTimeValSec:=tv.tv_sec;
  result:=(tv.tv_sec-FirstTimeValSec)*1000+(tv.tv_usec div 1000);
end;

procedure CopyMemory(DPtr,SPtr: Pointer; Len: integer);
begin
  Move(SPtr^,DPtr^,len);
end;


procedure MVEstInit(var MV: TMVEst);
begin
  with MV do begin
    Vmean:=0;
    Vcov:=0;
    speed:=0.95;
    n:=0;
  end;
end;

procedure MVEstAddValue(var MV: TMVEst; v: double);
begin
  with MV do begin
    Vmean:=Vmean*speed+v*(1-speed);
    Vcov:=Vcov*speed+sqr(Vmean-v)*(1-speed);
    inc(n);
  end;
end;

procedure mse(var e:double; m1,m2: TDMatrix; N: integer);
var
i: integer;
aux:double;
begin

  e:=0;

  for i:=0 to N-1 do
  begin
    aux:=m1.getv(i,0)-m2.getv(i,0);
    e := e + aux*aux;
  end;

  e:=e/N;

end;

procedure ClearLinearReg(var TL: TLinearReg);
begin
  with TL do begin
    Sx:=0;
    Sy:=0;
    Sxy:=0;
    Sx2:=0;
    N:=0;
    a:=0;
    b:=0
  end;
end;


procedure AddXYtoLinearReg(var TL: TLinearReg; x,y: double);
begin
  with TL do begin
    Sx:=Sx+x;
    Sy:=Sy+y;
    Sxy:=Sxy+x*y;
    Sx2:=Sx2+x*x;
    inc(N);
  end;
end;

procedure CalcLinearReg(var TL: TLinearReg);
var mx,my,d: double;
begin
  with TL do begin
    if N=0 then exit;
    mx:=Sx/N;
    my:=Sy/N;
    d:=N*Sx2-Sx*Sx;
    if abs(d)<1e-8 then exit;
    b:=(N*Sxy-Sx*Sy)/d;
    a:=my-b*mx;
  end;
end;

function AverageAngle(ang1,ang2: double): double;
begin
  result:=NormalizeAngle(ang1+DiffAngle(ang2,ang1)*0.5);
end;

//AGM02ABR
{procedure ObsAvoid(d,RSx,RSy,RSteta: double; var x,y: double);
var
  teste: double;
  line_test_obst: integer;

begin

  if GetTickCount-TimeObs > 1100 then begin
    if d>0.75  then begin
      //angulo entre o robot e o ponto para onde vai
      teste:=180*(Atan2(y-RSy,x-RSx)-RSteta)/pi;
      // linha par do radar correspondente (aprox.)
      line_test_obst:=2*(7+round(teste/22.5));
      // so testa se estiver fora da zona morta traseira
      if (line_test_obst>1) and (line_test_obst<28) then begin
        line_free:=-1;
        if ((View.Radar[line_test_obst].d<1.3) and (View.Radar[line_test_obst].d < d)) or
           ((View.Radar[line_test_obst+1].d<1.3) and (View.Radar[line_test_obst+1].d < d)) then begin
          // obstaculo a frente
          TimeObs:=GetTickCount;
          if (View.Radar[line_test_obst+2].d>1.5) then line_free:=line_test_obst+2;
          if line_free>0 then begin
            // esquerda livre
            StateObs:=FREE_LEFT;
          end else begin
            if (View.Radar[line_test_obst-2].d>1.5) then line_free:=line_test_obst-2;
            if line_free>0 then begin
              // direita livre
              StateObs:=FREE_RIGHT;
            end else begin
              // tudo tapado
              StateObs:=FREE_NOTHING;
            end;
          end;
        end;
      end;
    end;
  end else begin


    case StateObs of
      FREE_LEFT:begin
        x:=RSx+d*cos(View.Radar[line_free].teta+RSteta+45*pi/180);
        y:=RSy+d*sin(View.Radar[line_free].teta+RSteta+45*pi/180);
      end;
      FREE_RIGHT:begin
        x:=RSx+d*cos(View.Radar[line_free].teta+RSteta-45*pi/180);
        y:=RSy+d*sin(View.Radar[line_free].teta+RSteta-45*pi/180);
      end;
      FREE_NOTHING:begin
        x:=RSx+d*cos(pi/2+RSteta);
        y:=RSy+d*sin(pi/2+RSteta);
      end;
    end;
  end;
end;}


{procedure AvoidObstacles(Obsx,Obsy,RSx,RSy,RSteta: double; var traj: TTrajectory);
var
  teste,d,x1,y1,k1: double;
  line_test_obst: integer;
  splinea: TSpline;
begin
    //ajusta distancia obstaculo
    // d = distancia ao obstaculo
      k1:=strtofloatdef(Fmain.Editobsgain.text,1);
      d:=dist(RSx-Obsx,RSy-Obsy)*k1;

    // Inicia sempre sem obstaculos
    StateObs:=FREE_AHEAD;

    if d>0.75  then begin
      //angulo entre o robot e o ponto para onde vai
      teste:=180*(Atan2(Obsy-RSy,Obsx-RSx)-RSteta)/pi;
      // linha par do radar correspondente (aprox.)
      line_test_obst:=2*(7+round(teste/22.5));
 //     FMain.EditInfo.text := floattostr(teste)+' '+floattostr(line_test_obst);
      // so testa se estiver fora da zona morta traseira
      if (line_test_obst>1) and (line_test_obst<28) then begin
        line_free:=-1;
        if ((View.Radar[line_test_obst].d<1.3) and (View.Radar[line_test_obst].d < d)) or
           ((View.Radar[line_test_obst+1].d<1.3) and (View.Radar[line_test_obst+1].d < d)) then begin
          // obstaculo a frente
          TimeObs:=GetTickCount;
          if (View.Radar[line_test_obst+2].d>1.5) then line_free:=line_test_obst+2;
          if line_free>0 then begin
            // esquerda livre
            StateObs:=FREE_LEFT;
          end else begin
            if (View.Radar[line_test_obst-2].d>1.5) then line_free:=line_test_obst-2;
            if line_free>0 then begin
              // direita livre
              StateObs:=FREE_RIGHT;
            end else begin
              // tudo tapado
              StateObs:=FREE_NOTHING;
            end;
          end;
        end;
      end; // if zona morta
    end; // if d



    case StateObs of
      FREE_LEFT:begin
        x1:=RSx+d*cos(View.Radar[line_free].teta+RSteta+45*pi/180);
        y1:=RSy+d*sin(View.Radar[line_free].teta+RSteta+45*pi/180);
      end;
      FREE_RIGHT:begin
        x1:=RSx+d*cos(View.Radar[line_free].teta+RSteta-45*pi/180);
        y1:=RSy+d*sin(View.Radar[line_free].teta+RSteta-45*pi/180);
      end;
      FREE_NOTHING:begin
        x1:=RSx+d*cos(pi/2+RSteta);
        y1:=RSy+d*sin(pi/2+RSteta);
      end;
    end;

    if (StateObs <> FREE_AHEAD)
    then begin  // caso contrario nao altera a trajectoria

      splinea.X[0]:=RSx;
      splinea.X[1]:=x1;
      splinea.X[2]:=Obsx;

      splinea.Y[0]:=RSy;
      splinea.Y[1]:=y1;
      splinea.Y[2]:=Obsy;

      splinea.teta[0]:=RSteta;
      splinea.teta[1]:=RSteta;
      splinea.teta[2]:=RSteta;

      splinea.qtdPontos:=10;
      splinea.numVertices:=3;
      Ftraj.Calcspline(splinea,traj);

      LastAvoidCount:=GetTickCount;

(*    Ftraj.sgtraj.Cells[0,0]:=format('%.2f',[RSx]);
      Ftraj.sgtraj.Cells[0,1]:=format('%.2f',[x1]);
      Ftraj.sgtraj.Cells[0,2]:=format('%.2f',[Obsx]);

      Ftraj.sgtraj.Cells[1,0]:=format('%.2f',[RSy]);
      Ftraj.sgtraj.Cells[1,1]:=format('%.2f',[y1]);
      Ftraj.sgtraj.Cells[1,2]:=format('%.2f',[Obsy]);
*)
    end;

end;



function FreeDirection: integer;
var
line: integer;
Dist,CentralDist,LeftDist,RightDist: double;
begin

  line:=10;
  RightDist:=0;
  LeftDist:=0;

  CentralDist := (View.Radar[14].d+View.Radar[15].d+View.Radar[13].d+View.Radar[16].d)/4;

  while line<20 do
  begin

     Dist := (View.Radar[line].d+View.Radar[line+1].d)/2;
     // Somatorio de distancias livres a direita
     if line <= 14 then RightDist := RightDist + Dist;
     // Somatorio de distancias livres a esquerda
     if line >= 14 then LeftDist := LeftDist + Dist;

     Inc(line,2);
  end;

  if RightDist >= LeftDist
    then result := FREE_RIGHT
    else result := FREE_LEFT;

  if CentralDist >= RADAR_RAIO then result := FREE_AHEAD;
  if (abs(Field.FieldDims.FieldDepth/2-Robots.RobotState[myNumber].x) <= Field.FieldDims.areadepth)
    then result := FREE_AHEAD;
//  FMain.Memo1.lines.add('Central '+floattostr(CentralDist));
//  FMain.Memo1.lines.add('Left '+floattostr(LeftDist));

end;

//AGM09ABR
function Toler(value, true_value, toler_perc: double): boolean;
begin
  if abs((value-true_value)/true_value) < toler_perc then
    result:=TRUE
  else
    result:=FALSE;
end;

function SameInc(teta1,teta2,toler:double): boolean;
begin
if (abs(teta1-teta2) < toler) or (abs(teta1-pi-teta2) < toler) or (abs(teta1+pi-teta2) < toler) then
  result:=TRUE
else
  result:=FALSE;
end;

function SegLineMatch(x1,y1,d1,w1,x2,y2,d2,w2,teta: double): double;
var
  xr1,yr1,xr2,yr2: double;
  st, ct, pen: double;
begin
  st := sin(-teta);
  ct := cos(-teta);
  yr1:=x1*st + y1*ct;
  yr2:=x2*st + y2*ct;
  xr1:=x1*ct - y1*st;
  xr2:=x2*ct - y2*st;
  result:=abs(yr1-yr2);
//  result:=result+40*abs(w1-w2);
//  result:=result+20*abs(w1-w2);

  pen := (xr1-d1/2)-(xr2-d2/2);
  if pen > 0 then
    result:=result + 1.5 * pen ;

  pen := (xr2+d2/2)-(xr1+d1/2);
  if pen > 0 then
    result:=result + 1.5 * pen;


  //result:=result+abs(xr1-xr2);


  //Robotica2006
//  if ((abs(d1-d2)<0.5) or (d1=1.5) or (d1=0.5)) and (d2<2.5) then begin

  if d2>1.25*d1 then
    result:=result+10*(d2-d1);

  (*
  if (abs(d1-d2)<0.5)  then begin
    result:=result/5;
  end
  *)

  (*
  if ((xr1-d1/2)>(xr2-d2/2)) and ((xr1+d1/2)<(xr2+d2/2)) then
    result:=result+1.5*((xr2+d2/2)-(xr1+d1/2))+1.5*((xr1-d1/2)-(xr2-d2/2));;
  *)

end;    }

function MidAngle(a1,a2: double): double;
var x,y: double;
begin
  x:=cos(a1)+cos(a2);
  y:=sin(a1)+sin(a2);
  result:=ATan2(y,x);
end;


function FiltAngle(a1, a2, lamb: double): double;
var x,y: double;
begin
  x:=lamb*cos(a1)+(1-lamb)*cos(a2);
  y:=lamb*sin(a1)+(1-lamb)*sin(a2);
  result:=ATan2(y,x);
end;

//manuel
function GetTimeAndDate(tipo:integer) : string;
var
  d,t: string;
begin
  DateTimeToString(d, 'ddmmyy',Date);
  DateTimeToString(t, 'hhnnss',Time);
  case tipo of
    0: result:=d;
    1: begin
      DateTimeToString(t, 'hhnnsszzz',Time);
      result:=t;
    end;
    2: result:=d+'_'+t;
  end;
end;

function innerProduct(x1, y1, x2, y2: double): double;
var x1n,x2n,y1n,y2n:double;
begin
  x1n:=x1;
  x2n:=x2;
  y1n:=y1;
  y2n:=y2;
  NormalizeVector(x1n,y1n);
  NormalizeVector(x2n,y2n);
  result:=x1n*x2n+y1n*y2n;
end;


end.
