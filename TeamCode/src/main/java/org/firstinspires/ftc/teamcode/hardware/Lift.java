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

    private DcMotor LF1;
    private DcMotor LF2;

    /**
     * Hardware Mappings
     */
    public HardwareMap hardwareMap = null; // will be set in Child class
    public final int minPos = 0;
    public final int maxPos = 690;
    public final int startPos = 5;
    public final int carryPos = 15;
    public final int climbStartPos = 685;
    public final int climbEnd = 150;
    private int targetPos = startPos;
    private final double liftSpeed = 0.95;
    private final static double stagSpeed = 0.30;
    private final static int stagPos  = 30;
    private final static int tol = 10;
    private Mode CurrentMode = Mode.START;




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
        LF1 = hardwareMap.dcMotor.get("LF1");
        LF2 = hardwareMap.dcMotor.get("LF2");
        LF1.setDirection(DcMotor.Direction.REVERSE);
        LF2.setDirection(DcMotor.Direction.FORWARD);

        LF1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LF2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LF1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LF2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        LF1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LF2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



    }

    /**
     * User defined init_loop method
     * <p>
     * This method will be called repeatedly when the INIT button is pressed.
     * This method is optional. By default this method takes no action.
     */
     public void init_loop() {
         telemetry.addData("lift Pos " , Integer.toString(LF1.getCurrentPosition())) ;
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
       setMPower(CommonLogic.goToPosStagint(LF1.getCurrentPosition(), targetPos,tol,liftSpeed,stagPos,stagSpeed));

        switch (CurrentMode){
            case START:
            targetPos = startPos;
                break;
            case CARRY:
            targetPos = carryPos;
                break;
            case CLIMBPREP:
                targetPos = climbStartPos;
                break;
            case CLIMBEND:
                targetPos = climbEnd;
                break;
            case STOP:

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
    private void setMPower(double newPower){
        LF1.setPower(newPower);
        LF2.setPower(newPower);
    }
public void setCurrentMode (Mode newMode){
        CurrentMode = newMode;
}

public enum Mode{
    STOP,
    START,
    CARRY,
    CLIMBPREP,
    CLIMBEND,


}






}

