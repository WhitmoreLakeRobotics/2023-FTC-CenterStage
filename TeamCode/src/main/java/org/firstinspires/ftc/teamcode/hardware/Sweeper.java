package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.CommonLogic;

/**
 * Base class for FTC Team 8492 defined hardware
 */
public class Sweeper extends BaseHardware {

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

    private DcMotor ITM1;


    /**
     * Hardware Mappings
     */
    public HardwareMap hardwareMap = null; // will be set in Child class
    private final double stopPow = 0.000;
    private final double forwardPow = 0.70;
    private final double reversePOW = -0.45;
    private double targetPow = stopPow;
    private final double liftSpeed = 0.65;
    private final static double stagSpeed = 0.45;
    private final static int stagPos  = 30;
    private final static int tol = 10;
    private Mode CurrentMode = Mode.STOP;




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
        ITM1 = hardwareMap.dcMotor.get("ITM1");
        ITM1.setDirection(DcMotor.Direction.FORWARD);


        ITM1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        ITM1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        ITM1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);




    }

    /**
     * User defined init_loop method
     * <p>
     * This method will be called repeatedly when the INIT button is pressed.
     * This method is optional. By default this method takes no action.
     */
     public void init_loop() {
   //      telemetry.addData("lift Pos " , Integer.toString(ITM1.getCurrentPosition())) ;
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
        //set motor power
         switch (CurrentMode){
            case FORWARD:
                targetPow = forwardPow;
                break;
            case REVERSE:
                targetPow = reversePOW;
                break;
            case STOP:
                targetPow = stopPow;
                break;

            default:
        }
    ITM1.setPower(targetPow);
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
        ITM1.setPower(newPower);
    }
public void setCurrentMode (Mode newMode){
        CurrentMode = newMode;
}

public enum Mode{
    STOP,
    FORWARD,
    REVERSE;




}






}

