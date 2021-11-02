MODULE Module1
	CONST robtarget Target_10:=[[0,0,-1230.907],[0,1,0,0],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
	CONST robtarget Target_10_2:=[[-800,0,-1230.907],[0,1,0,0],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    !***********************************************************
    !
    ! Módulo:  Module1
    !
    ! Descripción:
    !   <Introduzca la descripción aquí>
    !
    ! Autor: Supercabb
    !
    ! Versión: 1.0
    !
    !***********************************************************
    
    
    !***********************************************************
    !
    ! Procedimiento Main
    !
    !   Este es el punto de entrada de su programa
    !
    !***********************************************************
    
    VAR clock clk1;
    VAR num clk01;
    
    PROC main()

        rResetClk;
        rStartClk;
        Path_10;
        rStopClk;
        rShowClkTime;
        !Añada aquí su código
    ENDPROC
    
Local PROC rResetClk ( )
        ClkReset clk1;
ENDPROC

Local PROC rStopClk ( )

        ClkStop clk1;

ENDPROC

Local PROC rStartClk ( )

        ClkStart clk1;

ENDPROC

Local PROC rShowClkTime ( )

        clk01 := ClkRead (clk1);
        ErrWrite \I,"Cycle Time Robot is = "+ValToStr(clk01),"Info";
        

ENDPROC    
    
    PROC GenerateTimesAllPositions()
        VAR robtarget newPos:=[[0,0,-1130.907],[0,1,0,0],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
        VAR robtarget newTmp:=[[0,0,-1130.907],[0,1,0,0],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
        VAR num xIni;
        VAR num xFin;
        VAR num radioRobot:=800;        
        VAR num longitudX;
        VAR num bajadamm:=100;
        VAR robtarget InitPos:=[[0,800,-1130.907],[0,1,0,0],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
        VAR iodev LogFile;
        
        Open "HOME:", \File:="log_prueba_1.csv", LogFile \Write;
        Write LogFile, "#######DatosPrueba###########";
        Write LogFile, "Pos Tolva e Inicial:"+ValToStr(InitPos.trans);
        Write LogFile, "bajadamm:"+ValToStr(bajadamm);
        Write LogFile, "radioRobot:"+ValToStr(radioRobot);
        Write LogFile, "##############################";
        Write LogFile, "TypeMove;EndPosX;EndPosY;Time";

        newTmp.trans.z:=newPos.trans.z-bajadamm;
        
        FOR yPos FROM -800 TO 800 STEP 5 DO
            longitudX := Abs(Trunc(Sqrt(Pow(radioRobot,2)-Pow(yPos,2))));
            xIni:=-longitudX;
            xFin:=longitudX;
            
            FOR xpos FROM xini TO xfin STEP 5 DO
                TPWrite "EndPosX="+ValToStr(xpos)+";EndPosY="+ValToStr(ypos);
                newPos.trans.x:=xpos;
                newPos.trans.y:=ypos;
                rResetClk;
                load0.mass:=0.001;
                GripLoad load0;
                MoveL InitPos,vmax,z100,tool0\WObj:=wobj0;
                rStartClk;
                MoveL newPos,vmax,z100,tool0\WObj:=wobj0;
                MoveL newPos,vmax,z100,tool0\WObj:=wobj0;
                load0.mass:=1;
                GripLoad load0;                
                MoveL InitPos,vmax,z100,tool0\WObj:=wobj0;
                WaitRob \InPos;
                rStopClk;
                !rShowClkTime;
                clk01 := ClkRead (clk1);
                Write LogFile, "Pick&Place;"+ValToStr(xpos)+";"+ValToStr(ypos)+";"+ValToStr(clk01);
                !ErrWrite \I,"EndPosX="+ValToStr(xpos)+";EndPosY="+ValToStr(ypos)+";Time="+ValToStr(clk01),"Info";
                
            ENDFOR
            
            
        ENDFOR
        
        Close LogFile;
        
    ENDPROC

    
   PROC GenerateTimesAllPositions2Steps()
        VAR robtarget newPos:=[[0,0,-1130.907],[0,1,0,0],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
        VAR robtarget newTmp:=[[0,0,-1130.907],[0,1,0,0],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
        VAR num xIni;
        VAR num xFin;
        VAR num radioRobot:=800;        
        VAR num longitudX;
        VAR num bajadamm:=100;
        VAR robtarget InitPos:=[[0,800,-1130.907],[0,1,0,0],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
        VAR iodev LogFile;
        
        Open "HOME:", \File:="log_prueba_2_steps_1.csv", LogFile \Write;
        Write LogFile, "#######DatosPrueba 2Steps###########";
        Write LogFile, "Pos Tolva e Inicial:"+ValToStr(InitPos.trans);
        Write LogFile, "bajadamm:"+ValToStr(bajadamm);
        Write LogFile, "radioRobot:"+ValToStr(radioRobot);
        Write LogFile, "##############################";
        Write LogFile, "TypeMove;EndPosX;EndPosY;Time";

        newTmp.trans.z:=newPos.trans.z-bajadamm;
        
        FOR yPos FROM -800 TO 800 STEP 5 DO
            longitudX := Abs(Trunc(Sqrt(Pow(radioRobot,2)-Pow(yPos,2))));
            xIni:=-longitudX;
            xFin:=longitudX;
            
            FOR xpos FROM xini TO xfin STEP 5 DO
                TPWrite "EndPosX="+ValToStr(xpos)+";EndPosY="+ValToStr(ypos);
                newPos.trans.x:=xpos;
                newPos.trans.y:=ypos;
                rResetClk;
                load0.mass:=0.001;
                GripLoad load0;
                MoveL InitPos,vmax,z100,tool0\WObj:=wobj0;
                rStartClk;
                MoveL newPos,vmax,z100,tool0\WObj:=wobj0;
                MoveL newPos,vmax,z100,tool0\WObj:=wobj0;
                WaitRob \InPos;
                rStopClk;
                clk01 := ClkRead (clk1);
                Write LogFile, "Pick;"+ValToStr(xpos)+";"+ValToStr(ypos)+";"+ValToStr(clk01);
                rResetClk;
                load0.mass:=1;
                GripLoad load0;                
                rStartClk;
                MoveL InitPos,vmax,z100,tool0\WObj:=wobj0;
                WaitRob \InPos;
                rStopClk;
                !rShowClkTime;
                clk01 := ClkRead (clk1);
                Write LogFile, "Place;"+ValToStr(xpos)+";"+ValToStr(ypos)+";"+ValToStr(clk01);
                !ErrWrite \I,"EndPosX="+ValToStr(xpos)+";EndPosY="+ValToStr(ypos)+";Time="+ValToStr(clk01),"Info";
                
            ENDFOR
            
            
        ENDFOR
        
        Close LogFile;
        
    ENDPROC    
    
	PROC Path_10()
        GenerateTimesAllPositions2Steps;
        rResetClk;
        rStartClk;
        MoveL Target_10,v1000,z100,tool0\WObj:=wobj0;
		MoveL Target_10_2,v1000,z100,tool0\WObj:=wobj0;
        rStopClk;
        rShowClkTime;
        !rShowClkTime;   
	ENDPROC
ENDMODULE