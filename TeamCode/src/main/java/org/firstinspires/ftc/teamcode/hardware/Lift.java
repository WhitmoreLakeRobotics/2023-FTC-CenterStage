package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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
    private Mode CurrentMode = Mode.STOP;

    private DcMotor LF1;
    private DcMotor LF2;

    /**
     * Hardware Mappings
     */
    public HardwareMap hardwareMap = null; // will be set in Child class
    public final int minPos = 0;
    public final int maxPos = 200;
    public final int startPos = 5;
    public final int carryPos = 15;
    public final int climbStartPos = 195;
    public final int climbEnd = 100;
    private int targetPos = startPos;
    private final double liftSpeed = 0.65;
    private final static double stagSpeed = 0.45;
    private final static int stagPos  = 30;
    private final static int tol = 10;




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
        LF1.setDirection(DcMotor.Direction.FORWARD);
        LF2.setDirection(DcMotor.Direction.REVERSE);

        LF1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LF2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LF1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LF2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        LF1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LF2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



    }

    /**
     * User defined init_loop method
     * <p>
     * This method will be called repeatedly when the INIT button is pressed.
     * This method is optional. By default this method takes no action.
     */
     public void init_loop() {
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
        //GoToPos
        CommonLogic.goToPosition(LF1.getCurrentPosition(), targetPos,tol,-liftSpeed,liftSpeed,0,stagPos);

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
    private void setMPower(double newPower){
        LF1.setPower(newPower);
        LF2.setPower(newPower);
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

