package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.common.CommonLogic;

/**
 * Base class for FTC Team 8492 defined hardware
 */
public class Sensors extends BaseHardware {

    private ElapsedTime runtime = new ElapsedTime();
    /**
     * The {@link #telemetry} field contains an object in which a user may accumulate data which
     * is to be transmitted to the driver station. This data is automatically transmitted to the
     * driver station on a regular, periodic basis.
     */
    public Telemetry telemetry = null;
    //private ColorRangeSensor IntakeSensor;
    //private DistanceSensor RearLeftSensor
    private DistanceSensor FLDS1;
    private DistanceSensor FRDS1;
    private DistanceSensor SLDS1;
    private DistanceSensor SRDS1;
    private boolean leftSensor = false;
    private boolean rightSensor = false;
    private final double sideTarget = 24; // in inches
    private final double sideTargetTol = 12; // in inches
    private double centerTarget = 29; // in inches
    private double centerTargetTol = 12; // in inches
    ////////private double autonBackWallTarget = 13 // in inches
  //  private double autonBackWallTargetTol = 1 // in inches



    private boolean cmdComplete = true;
    private Mode CurrentMode = Mode.STOP;

    private Color SignalColor = Color.UNKNOWN;


    /**
     * Hardware Mappings
     */
    public HardwareMap hardwareMap = null; // will be set in Child class


    /**
     * BaseHardware constructor
     * <p>
     * The op mode name should be unique. It will be the name displayed on the driver station. If
     * multiple op modes have the same name, only one will be available.
     */
    /*public Swing_Arm_And_Lift() {

    }*/

    /**
     * User defined init method
     * <p>
     * This method will be called once when the INIT button is pressed.
     */
    public void init(){
        //DeliverySensor = hardwareMap.get(ColorRangeSensor.class, "DeliveryS");
       // RearLeftSensor = hardwareMap.get(DistanceSensor.class, "RearLeftS");
        FLDS1 = hardwareMap.get(Rev2mDistanceSensor.class,"FLDS1");
        FRDS1 = hardwareMap.get(Rev2mDistanceSensor.class,"FRDS1");
        SLDS1 = hardwareMap.get(Rev2mDistanceSensor.class,"SLDS1");
        SRDS1 = hardwareMap.get(Rev2mDistanceSensor.class,"SRDS1");
    }

    /**
     * User defined init_loop method
     * <p>
     * This method will be called repeatedly when the INIT button is pressed.
     * This method is optional. By default this method takes no action.
     */
     public void init_loop() {
         telemetry.addData("FLDS1 Pos " , FLDS1.getDistance(DistanceUnit.INCH)) ;
         telemetry.addData("FRDS1 Pos " , FRDS1.getDistance(DistanceUnit.INCH)) ;

         telemetry.addData("SLDS1 Pos " , SLDS1.getDistance(DistanceUnit.INCH)) ;
         telemetry.addData("SRDS1 Pos " , SRDS1.getDistance(DistanceUnit.INCH)) ;

     }

    /**
     * User defined start method.
     * <p>
     * This method will be called once when the PLAY button is first pressed.
     * This method is optional. By default this method takes not action.
     * Example usage: Starting another thread.
     */
    public void start(){

    }

    /**
     * User defined loop method
     * <p>
     * This method will be called repeatedly in a loop while this op mode is running
     */
    public void loop(){
        telemetry.addData("FLDS1 Pos " , FLDS1.getDistance(DistanceUnit.INCH)) ;
        telemetry.addData("FRDS1 Pos " , FRDS1.getDistance(DistanceUnit.INCH)) ;

        telemetry.addData("SLDS1 Pos " , SLDS1.getDistance(DistanceUnit.INCH)) ;
        telemetry.addData("SRDS1 Pos " , SRDS1.getDistance(DistanceUnit.INCH)) ;

    }

    public void doStop(){
        CurrentMode = Mode.STOP;

        cmdComplete = true;
    }



    /**
     * User defined stop method
     * <p>
     * This method will be called when this op mode is first disabled
     * <p>
     * The stop method is optional. By default this method takes no action.
     */
    void stop(){

}
public boolean FLDS1Detect(){
        return CommonLogic.inRange(FLDS1.getDistance(DistanceUnit.INCH),sideTarget,sideTargetTol);
}
 public boolean FRDS1Detect(){
        return CommonLogic.inRange(FRDS1.getDistance(DistanceUnit.INCH),centerTarget,centerTargetTol);

 }

    /*public boolean SLDS1Detect(){
        return CommonLogic.inRange(SLDS1.getDistance(DistanceUnit.INCH),sideTarget,sideTargetTol);
    }
    public boolean SRDS1Detect(){
        return CommonLogic.inRange(SRDS1.getDistance(DistanceUnit.INCH),centerTarget,centerTargetTol);
    }*/

public double GetSensorDistanceLeftFront(){
        return FLDS1.getDistance(DistanceUnit.INCH);
}
    public double GetSensorDistanceRightFront(){
        return FRDS1.getDistance(DistanceUnit.INCH);
}

    public double GetSensorDistanceLeftSide(){
        return SLDS1.getDistance(DistanceUnit.INCH);
    }
    public double GetSensorDistanceRightSide(){
        return SRDS1.getDistance(DistanceUnit.INCH);
    }

private enum Mode{
    STOP,
    READ,
    UP,
    READPOS,
    COLORFOUND
}
    public enum Color{
        UNKNOWN,
        GREEN,
        RED,
        BLUE
    }





}

