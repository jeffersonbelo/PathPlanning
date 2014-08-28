unit configSimTwo;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, FileUtil, LResources, Forms, Controls, Graphics, Dialogs,
  StdCtrls, lNetComponents, lNet, robotmainfunctions;

type

  { TFconfig }

  TFconfig = class(TForm)
    Edit1: TEdit;
    Edit2: TEdit;
    Edit3: TEdit;
    Label1: TLabel;
    Label2: TLabel;
    Label3: TLabel;
    Memo1: TMemo;
    UDP: TLUDPComponent;
    procedure Erro(const msg: string; aSocket: TLSocket);
    procedure Receiver(aSocket: TLSocket);
  private
    { private declarations }
  public
    { public declarations }
  end; 

var
  Fconfig: TFconfig;

implementation
uses control;

{ TFconfig }

procedure TFconfig.Erro(const msg: string; aSocket: TLSocket);
begin
  Memo1.Append('Error: ' + msg);
end;

procedure TFconfig.Receiver(aSocket: TLSocket);
var
  msg: string;
begin
  UDP.GetMessage(msg);
  if (msg <> '') and (Fcontrol.CB_SimTwo.Checked) then begin
    //Program Test UDP Communication debug
    Memo1.Append(Format('Msg: %d - ', [Length(msg)]) + msg);
    while Memo1.Lines.Count > 15 do begin
      Memo1.Lines.Delete(0);
    end;
    ProcessSimTwoMsg(msg);
  end else begin
    exit;
  end;
end;

initialization
  {$I configsimtwo.lrs}

end.

