package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.common.CommonLogic;

/**
 * Base class for FTC Team 8492 defined hardware
 */
public class Lift extends BaseHardware {

    private ElapsedTime runtime = new ElapsedTime();
    /**
     * The {@link #telemetry} field contains an object in which a user may accumulate data which
     * is to be transmitted to the driver station. This data is automatically transmitted to the
     * driver station on a regular, periodic basis.
     */
    public Telemetry telemetry = null;
    //private ColorRangeSensor IntakeSensor;
    //private DistanceSensor RearLeftSensor;
    private boolean cmdComplete = true;

    private DcMotorEx LF1;
    private DcMotorEx LF2;
    private DcMotorEx ARM1;

    private Servo WRIST1;
    private Servo BOX;
    private ColorRangeSensor BOXS;
    /**
     * Hardware Mappings
     */
    public HardwareMap hardwareMap = null; // will be set in Child class
    public final int minPos = 0;
    public final int maxPos = 747;
    public final int liftDeliveyPos = 320;
    public final int liftDeliveryPosTel = 747;
    public final int startPos = 5;
    public final int carryPos = 15;
    public final int climbStartPos = 747;
    public final int climbEnd = 140;
    private int targetPos = startPos;
    private final double liftSpeed = 1.0;
    private double stagSpeed = 0.30;
    private static final double holdDefalt = 0.30;
    private static  final double holdClimb = 0.40;
    private final static int stagPos  = 100;
    private final static int tol =15;
    private Mode CurrentMode = Mode.START;
    private final double boxOpen = 1;
    private final double boxClose = 0.5;
    private final double wristPickup = 0;
    private final double wristDelivery = 0.6;
    private final double wristIntake = 0.1;
    private final int armPickup = 0;
    private final int armDelivery = 530;
    private final int armMinPos = 0;
    private final int armMaxPos = 624;
    private int ArmTargetPos = armMinPos;
    private final double armSpeed = 0.7;
    private final double armStagSpeed = 0.15;
    private final int armStagPos = 436; //15
    private  double armHoldPow = 0.0;
    private final  double armHoldDeliver = -0.10;
    private  final double armHoldIntake = 0.05;
    private final int armTol = 15;
private final double boxEmpty = 7; // in inches
    private final double boxOne = 3.5;
    private final double boxTwo = 0;
    private final double boxTol = 0.25;
    private final double armTimeout = 800;

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
        LF1 = hardwareMap.get(DcMotorEx.class, "LF1");
        LF2 = hardwareMap.get(DcMotorEx.class, "LF2");
        LF1.setDirection(DcMotor.Direction.REVERSE);
        LF2.setDirection(DcMotor.Direction.FORWARD);

        LF1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LF2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LF1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LF2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        LF1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LF2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


//                  intizilze arm, wrist, and box
//ARM1 = hardwareMap.dcMotor.get("ARM1");
ARM1 =hardwareMap.get(DcMotorEx.class, "ARM1");
ARM1.setDirection(DcMotorSimple.Direction.REVERSE);
//ARM1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
ARM1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
ARM1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
ARM1.setVelocityPIDFCoefficients(18,0,0,0);
WRIST1 = hardwareMap.get(Servo.class,"WRIST1");
BOX = hardwareMap.get(Servo.class,"BOX");
BOXS = hardwareMap.get(ColorRangeSensor.class, "BOXS");

    }

    /**
     * User defined init_loop method
     * <p>
     * This method will be called repeatedly when the INIT button is pressed.
     * This method is optional. By default this method takes no action.
     */
     public void init_loop() {
         telemetry.addData("lift Pos " , Integer.toString(LF1.getCurrentPosition())) ;
         telemetry.addData("Arm Pos " , Integer.toString(ARM1.getCurrentPosition())) ;
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
        telemetry.addData("lift Pos " , Integer.toString(LF1.getCurrentPosition())) ;
        telemetry.addData("Arm Pos " , Integer.toString(ARM1.getCurrentPosition())) ;
        //GoToPos
       //setMPower(CommonLogic.goToPosStagint(LF1.getCurrentPosition(), targetPos,tol,liftSpeed,stagPos,stagSpeed));
       setMPower(CommonLogic.CapValue(CommonLogic.PIDcalc(stagPos, stagSpeed, LF1.getCurrentPosition(), targetPos),-liftSpeed, liftSpeed));
        //ARM1.setPower(CommonLogic.goToPosStag(ARM1.getCurrentPosition(),ArmTargetPos,armTol,armSpeed,armStagPos,armStagSpeed));

        ARM1.setPower(CommonLogic.CapValue( CommonLogic.PIDcalc(armStagPos,armHoldPow,ARM1.getCurrentPosition(),ArmTargetPos),-armSpeed,armSpeed));

        switch (CurrentMode){
            case START:
            targetPos = startPos;
            stagSpeed = 0;
            closeDoor();
            gotoPosWrist(wristPickup);
                break;
            case CARRY:
            targetPos = carryPos;
            stagSpeed = holdDefalt;
            closeDoor();
                break;
            case DELIVERTEL:
                //  openDoor();
                ArmgotoPos(armDelivery);
                gotoPosWrist(wristDelivery);
                liftgotoPos(liftDeliveryPosTel);
                armHoldPow = armHoldDeliver;
                break;
            case CLIMBPREP:
                ArmgotoPos(armPickup);
                gotoPosWrist(wristPickup);
                targetPos = climbStartPos;
                stagSpeed = holdDefalt;
                break;
            case CLIMBEND:
                targetPos = climbEnd;
                stagSpeed = holdClimb;
                break;
            case STOP:

                break;
            case INTAKE:
               // openDoor();
                ArmgotoPos(armPickup);
                gotoPosWrist(wristPickup);
                liftgotoPos(startPos);
                armHoldPow = armHoldIntake;
                //if(CommonLogic.inRange(ARM1.getCurrentPosition(),armPickup,armTol)){
                  if (runtime.milliseconds()> armTimeout){
                    gotoPosWrist(wristIntake);
                    armHoldPow = armHoldDeliver;
                }

                break;
            case  DELIVER:
              //  openDoor();
                ArmgotoPos(armDelivery);
                gotoPosWrist(wristDelivery);
               liftgotoPos(liftDeliveyPos);
                armHoldPow = armHoldDeliver;
                break;
            default:
        }

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
public void resetArmPos(){
       ARM1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       ARM1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       ARM1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
}

public void resetLiftPos(){
    LF1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    LF2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    LF1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    LF2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    LF1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    LF2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
}

public void liftCoast(){
    LF1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    LF2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
}
    private void setMPower(double newPower){
        LF1.setPower(newPower);
        LF2.setPower(newPower);
    }
public void setCurrentMode (Mode newMode){
        CurrentMode = newMode;
        runtime.reset();
}
    private void gotoPosWrist( double newPos ){WRIST1.setPosition(newPos);}
    public void openDoor(){BOX.setPosition(boxOpen);}
    public void closeDoor(){BOX.setPosition(boxClose);}
    private void ArmgotoPos( int newPos){ArmTargetPos = CommonLogic.CapValueint(newPos,armMinPos,armMaxPos);
    }
    private void liftgotoPos( int newPos) {targetPos = CommonLogic.CapValueint(newPos,minPos,maxPos);
    }
    public void liftPosOverRide(int posChange){
        targetPos = targetPos + (posChange * 1);
    }

    public int boxCount(){
        if (CommonLogic.inRange(BOXS.getDistance(DistanceUnit.INCH),boxEmpty, boxTol)){
            return 0;
        }else if (CommonLogic.inRange(BOXS.getDistance(DistanceUnit.INCH),boxOne, boxTol)){
            return 1;
        }else if (CommonLogic.inRange(BOXS.getDistance(DistanceUnit.INCH),boxTwo, boxTol)){
          return 2;
        }else {
            return -1;
        }
    }
public Mode getCurrentMode(){
        return CurrentMode;
}

    public enum Mode{
    STOP,
    START,
    CARRY,
        DELIVERTEL,
    CLIMBPREP,
    CLIMBEND,
        INTAKE,
        DELIVER;


}






}

