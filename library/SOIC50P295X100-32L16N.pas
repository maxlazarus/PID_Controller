Var
    CurrentLib : IPCB_Library;

Procedure CreateSMDComponentPad(NewPCBLibComp : IPCB_LibComponent, Name : String, Layer : TLayer, X : Real, Y : Real, OffsetX : Real, OffsetY : Real,
                                TopShape : TShape, TopXSize : Real, TopYSize : Real, Rotation: Real, CRRatio : Real, PMExpansion : Real, SMExpansion : Real,
                                PMFromRules : Boolean, SMFromRules : Boolean);
Var
    NewPad                      : IPCB_Pad2;
    PadCache                    : TPadCache;

Begin
    NewPad := PcbServer.PCBObjectFactory(ePadObject, eNoDimension, eCreate_Default);
    NewPad.HoleSize := MMsToCoord(0);
    NewPad.Layer    := Layer;
    NewPad.TopShape := TopShape;
    if TopShape = eRoundedRectangular then
        NewPad.SetState_StackCRPctOnLayer(eTopLayer, CRRatio);
    NewPad.TopXSize := MMsToCoord(TopXSize);
    NewPad.TopYSize := MMsToCoord(TopYSize);
    NewPad.RotateBy(Rotation);
    NewPad.MoveToXY(MMsToCoord(X), MMsToCoord(Y));
    NewPad.Name := Name;

    Padcache := NewPad.GetState_Cache;
    if (PMExpansion <> 0) or (PMFromRules = False) then
    Begin
        Padcache.PasteMaskExpansionValid   := eCacheManual;
        Padcache.PasteMaskExpansion        := MMsToCoord(PMExpansion);
    End;
    if (SMExpansion <> 0) or (SMFromRules = False) then
    Begin
        Padcache.SolderMaskExpansionValid  := eCacheManual;
        Padcache.SolderMaskExpansion       := MMsToCoord(SMExpansion);
    End;
    NewPad.SetState_Cache              := Padcache;

    NewPCBLibComp.AddPCBObject(NewPad);
    PCBServer.SendMessageToRobots(NewPCBLibComp.I_ObjectAddress,c_Broadcast,PCBM_BoardRegisteration,NewPad.I_ObjectAddress);
End;

Procedure CreateComponentTrack(NewPCBLibComp : IPCB_LibComponent, X1 : Real, Y1 : Real, X2 : Real, Y2 : Real, Layer : TLayer, LineWidth : Real);
Var
    NewTrack                    : IPCB_Track;

Begin
    NewTrack := PcbServer.PCBObjectFactory(eTrackObject,eNoDimension,eCreate_Default);
    NewTrack.X1 := MMsToCoord(X1);
    NewTrack.Y1 := MMsToCoord(Y1);
    NewTrack.X2 := MMsToCoord(X2);
    NewTrack.Y2 := MMsToCoord(Y2);
    NewTrack.Layer := Layer;
    NewTrack.Width := MMsToCoord(LineWidth);
    NewPCBLibComp.AddPCBObject(NewTrack);
    PCBServer.SendMessageToRobots(NewPCBLibComp.I_ObjectAddress,c_Broadcast,PCBM_BoardRegisteration,NewTrack.I_ObjectAddress);
End;

Procedure CreateComponentArc(NewPCBLibComp : IPCB_LibComponent, CenterX : Real, CenterY: Real, Radius : Real, StartAngle : Real, EndAngle : Real, Layer : TLayer, LineWidth : Real);
Var
    NewArc                      : IPCB_Arc;

Begin
    NewArc := PCBServer.PCBObjectFactory(eArcObject,eNoDimension,eCreate_Default);
    NewArc.XCenter := MMsToCoord(CenterX);
    NewArc.YCenter := MMsToCoord(CenterY);
    NewArc.Radius := MMsToCoord(Radius);
    NewArc.StartAngle := StartAngle;
    NewArc.EndAngle := EndAngle;
    NewArc.Layer := Layer;
    NewArc.LineWidth := MMsToCoord(LineWidth);
    NewPCBLibComp.AddPCBObject(NewArc);
    PCBServer.SendMessageToRobots(NewPCBLibComp.I_ObjectAddress,c_Broadcast,PCBM_BoardRegisteration,NewArc.I_ObjectAddress);
End;

Procedure CreateComponentSOIC50P295X100_32L16N;
Var
    NewPCBLibComp               : IPCB_LibComponent;
    NewPad                      : IPCB_Pad2;
    NewRegion                   : IPCB_Region;
    NewContour                  : IPCB_Contour;
    STEPmodel                   : IPCB_ComponentBody;
    Model                       : IPCB_Model;

Begin
    NewPCBLibComp := PCBServer.CreatePCBLibComp;
    NewPcbLibComp.Name := 'SOIC50P295X100-32L16N';
    NewPCBLibComp.Description := 'Small Outline IC (SOIC), 0.050 in pitch;32 pin,0.816 in L X 0.296 in W X 0.100 in H body';
    NewPCBLibComp.Height := MMsToCoord(2.54);

    CreateSMDComponentPad(NewPCBLibComp, '1', eTopLayer, -9.525, -3.40072, 0, 0, eRoundedRectangular, 1.53756, 0.59648, 270, 50, 0, 0, True, True);
    CreateSMDComponentPad(NewPCBLibComp, '2', eTopLayer, -8.255, -3.40072, 0, 0, eRoundedRectangular, 1.53756, 0.59648, 270, 50, 0, 0, True, True);
    CreateSMDComponentPad(NewPCBLibComp, '3', eTopLayer, -6.985, -3.40072, 0, 0, eRoundedRectangular, 1.53756, 0.59648, 270, 50, 0, 0, True, True);
    CreateSMDComponentPad(NewPCBLibComp, '4', eTopLayer, -5.715, -3.40072, 0, 0, eRoundedRectangular, 1.53756, 0.59648, 270, 50, 0, 0, True, True);
    CreateSMDComponentPad(NewPCBLibComp, '5', eTopLayer, -4.445, -3.40072, 0, 0, eRoundedRectangular, 1.53756, 0.59648, 270, 50, 0, 0, True, True);
    CreateSMDComponentPad(NewPCBLibComp, '6', eTopLayer, -3.175, -3.40072, 0, 0, eRoundedRectangular, 1.53756, 0.59648, 270, 50, 0, 0, True, True);
    CreateSMDComponentPad(NewPCBLibComp, '7', eTopLayer, -1.905, -3.40072, 0, 0, eRoundedRectangular, 1.53756, 0.59648, 270, 50, 0, 0, True, True);
    CreateSMDComponentPad(NewPCBLibComp, '8', eTopLayer, -0.635, -3.40072, 0, 0, eRoundedRectangular, 1.53756, 0.59648, 270, 50, 0, 0, True, True);
    CreateSMDComponentPad(NewPCBLibComp, '9', eTopLayer, 0.635, -3.40072, 0, 0, eRoundedRectangular, 1.53756, 0.59648, 270, 50, 0, 0, True, True);
    CreateSMDComponentPad(NewPCBLibComp, '10', eTopLayer, 1.905, -3.40072, 0, 0, eRoundedRectangular, 1.53756, 0.59648, 270, 50, 0, 0, True, True);
    CreateSMDComponentPad(NewPCBLibComp, '11', eTopLayer, 3.175, -3.40072, 0, 0, eRoundedRectangular, 1.53756, 0.59648, 270, 50, 0, 0, True, True);
    CreateSMDComponentPad(NewPCBLibComp, '12', eTopLayer, 4.445, -3.40072, 0, 0, eRoundedRectangular, 1.53756, 0.59648, 270, 50, 0, 0, True, True);
    CreateSMDComponentPad(NewPCBLibComp, '13', eTopLayer, 5.715, -3.40072, 0, 0, eRoundedRectangular, 1.53756, 0.59648, 270, 50, 0, 0, True, True);
    CreateSMDComponentPad(NewPCBLibComp, '14', eTopLayer, 6.985, -3.40072, 0, 0, eRoundedRectangular, 1.53756, 0.59648, 270, 50, 0, 0, True, True);
    CreateSMDComponentPad(NewPCBLibComp, '15', eTopLayer, 8.255, -3.40072, 0, 0, eRoundedRectangular, 1.53756, 0.59648, 270, 50, 0, 0, True, True);
    CreateSMDComponentPad(NewPCBLibComp, '16', eTopLayer, 9.525, -3.40072, 0, 0, eRoundedRectangular, 1.53756, 0.59648, 270, 50, 0, 0, True, True);
    CreateSMDComponentPad(NewPCBLibComp, '17', eTopLayer, 9.525, 3.40072, 0, 0, eRoundedRectangular, 1.53756, 0.59648, 90, 50, 0, 0, True, True);
    CreateSMDComponentPad(NewPCBLibComp, '18', eTopLayer, 8.255, 3.40072, 0, 0, eRoundedRectangular, 1.53756, 0.59648, 90, 50, 0, 0, True, True);
    CreateSMDComponentPad(NewPCBLibComp, '19', eTopLayer, 6.985, 3.40072, 0, 0, eRoundedRectangular, 1.53756, 0.59648, 90, 50, 0, 0, True, True);
    CreateSMDComponentPad(NewPCBLibComp, '20', eTopLayer, 5.715, 3.40072, 0, 0, eRoundedRectangular, 1.53756, 0.59648, 90, 50, 0, 0, True, True);
    CreateSMDComponentPad(NewPCBLibComp, '21', eTopLayer, 4.445, 3.40072, 0, 0, eRoundedRectangular, 1.53756, 0.59648, 90, 50, 0, 0, True, True);
    CreateSMDComponentPad(NewPCBLibComp, '22', eTopLayer, 3.175, 3.40072, 0, 0, eRoundedRectangular, 1.53756, 0.59648, 90, 50, 0, 0, True, True);
    CreateSMDComponentPad(NewPCBLibComp, '23', eTopLayer, 1.905, 3.40072, 0, 0, eRoundedRectangular, 1.53756, 0.59648, 90, 50, 0, 0, True, True);
    CreateSMDComponentPad(NewPCBLibComp, '24', eTopLayer, 0.635, 3.40072, 0, 0, eRoundedRectangular, 1.53756, 0.59648, 90, 50, 0, 0, True, True);
    CreateSMDComponentPad(NewPCBLibComp, '25', eTopLayer, -0.635, 3.40072, 0, 0, eRoundedRectangular, 1.53756, 0.59648, 90, 50, 0, 0, True, True);
    CreateSMDComponentPad(NewPCBLibComp, '26', eTopLayer, -1.905, 3.40072, 0, 0, eRoundedRectangular, 1.53756, 0.59648, 90, 50, 0, 0, True, True);
    CreateSMDComponentPad(NewPCBLibComp, '27', eTopLayer, -3.175, 3.40072, 0, 0, eRoundedRectangular, 1.53756, 0.59648, 90, 50, 0, 0, True, True);
    CreateSMDComponentPad(NewPCBLibComp, '28', eTopLayer, -4.445, 3.40072, 0, 0, eRoundedRectangular, 1.53756, 0.59648, 90, 50, 0, 0, True, True);
    CreateSMDComponentPad(NewPCBLibComp, '29', eTopLayer, -5.715, 3.40072, 0, 0, eRoundedRectangular, 1.53756, 0.59648, 90, 50, 0, 0, True, True);
    CreateSMDComponentPad(NewPCBLibComp, '30', eTopLayer, -6.985, 3.40072, 0, 0, eRoundedRectangular, 1.53756, 0.59648, 90, 50, 0, 0, True, True);
    CreateSMDComponentPad(NewPCBLibComp, '31', eTopLayer, -8.255, 3.40072, 0, 0, eRoundedRectangular, 1.53756, 0.59648, 90, 50, 0, 0, True, True);
    CreateSMDComponentPad(NewPCBLibComp, '32', eTopLayer, -9.525, 3.40072, 0, 0, eRoundedRectangular, 1.53756, 0.59648, 90, 50, 0, 0, True, True);

    CreateComponentArc(NewPCBLibComp, 0, 0, 0.3048, 0, 360, eMechanical15, 0.0508);
    CreateComponentTrack(NewPCBLibComp, 0, 0.4064, 0, -0.4064, eMechanical15, 0.0508);
    CreateComponentTrack(NewPCBLibComp, -0.4064, 0, 0.4064, 0, eMechanical15, 0.0508);
    CreateComponentArc(NewPCBLibComp, -9.525, -4.4997, 0.1016, 0, 360, eTopOverlay, 0.2032);
    CreateComponentTrack(NewPCBLibComp, -10.6934, -4.0513, -10.07724, -4.0513, eMechanical15, 0.0508);
    CreateComponentTrack(NewPCBLibComp, -10.07724, -4.0513, -10.07724, -4.4235, eMechanical15, 0.0508);
    CreateComponentTrack(NewPCBLibComp, -10.07724, -4.4235, 10.07724, -4.4235, eMechanical15, 0.0508);
    CreateComponentTrack(NewPCBLibComp, 10.07724, -4.4235, 10.07724, -4.0513, eMechanical15, 0.0508);
    CreateComponentTrack(NewPCBLibComp, 10.07724, -4.0513, 10.6934, -4.0513, eMechanical15, 0.0508);
    CreateComponentTrack(NewPCBLibComp, 10.6934, -4.0513, 10.6934, 4.0513, eMechanical15, 0.0508);
    CreateComponentTrack(NewPCBLibComp, 10.6934, 4.0513, 10.07724, 4.0513, eMechanical15, 0.0508);
    CreateComponentTrack(NewPCBLibComp, 10.07724, 4.0513, 10.07724, 4.4235, eMechanical15, 0.0508);
    CreateComponentTrack(NewPCBLibComp, 10.07724, 4.4235, -10.07724, 4.4235, eMechanical15, 0.0508);
    CreateComponentTrack(NewPCBLibComp, -10.07724, 4.4235, -10.07724, 4.0513, eMechanical15, 0.0508);
    CreateComponentTrack(NewPCBLibComp, -10.07724, 4.0513, -10.6934, 4.0513, eMechanical15, 0.0508);
    CreateComponentTrack(NewPCBLibComp, -10.6934, 4.0513, -10.6934, -4.0513, eMechanical15, 0.0508);
    CreateComponentTrack(NewPCBLibComp, 10.4394, -3.7973, -9.1694, -3.7973, eMechanical11, 0.127);
    CreateComponentTrack(NewPCBLibComp, -9.1694, -3.7973, -10.4394, -2.5273, eMechanical11, 0.127);
    CreateComponentTrack(NewPCBLibComp, -10.4394, -2.5273, -10.4394, 3.7973, eMechanical11, 0.127);
    CreateComponentTrack(NewPCBLibComp, -10.4394, 3.7973, 10.4394, 3.7973, eMechanical11, 0.127);
    CreateComponentTrack(NewPCBLibComp, 10.4394, 3.7973, 10.4394, -3.7973, eMechanical11, 0.127);
    CreateComponentTrack(NewPCBLibComp, -10.01374, -4.106, -10.01374, -3.7973, eTopOverlay, 0.127);
    CreateComponentTrack(NewPCBLibComp, -10.01374, -3.7973, -10.4394, -3.7973, eTopOverlay, 0.127);
    CreateComponentTrack(NewPCBLibComp, -10.4394, -3.7973, -10.4394, 3.7973, eTopOverlay, 0.127);
    CreateComponentTrack(NewPCBLibComp, -10.4394, 3.7973, -10.01374, 3.7973, eTopOverlay, 0.127);
    CreateComponentTrack(NewPCBLibComp, 10.01374, -3.7973, 10.4394, -3.7973, eTopOverlay, 0.127);
    CreateComponentTrack(NewPCBLibComp, 10.4394, -3.7973, 10.4394, 3.7973, eTopOverlay, 0.127);
    CreateComponentTrack(NewPCBLibComp, 10.4394, 3.7973, 10.01374, 3.7973, eTopOverlay, 0.127);

    CurrentLib.RegisterComponent(NewPCBLibComp);
    CurrentLib.CurrentComponent := NewPcbLibComp;
    CurrentLib.Board.ViewManager_FullUpdate;

    Client.SendMessage('PCB:Zoom', 'Action=All' , 255, Client.CurrentView)
End;

Procedure CreateALibrary;
Var
    View     : IServerDocumentView;
    Document : IServerDocument;

Begin
    If PCBServer = Nil Then
    Begin
        ShowMessage('No PCBServer present. This script inserts a footprint into an existing PCB Library that has the current focus.');
        Exit;
    End;

    CurrentLib := PcbServer.GetCurrentPCBLibrary;
    If CurrentLib = Nil Then
    Begin
        ShowMessage('You must have focus on a PCB Library in order for this script to run.');
        Exit;
    End;

    View := Client.GetCurrentView;
    Document := View.OwnerDocument;
    Document.Modified := True;

    CreateComponentSOIC50P295X100_32L16N;

End;

End.
