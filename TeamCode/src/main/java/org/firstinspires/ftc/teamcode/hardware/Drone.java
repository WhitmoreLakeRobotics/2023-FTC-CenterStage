package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Base class for FTC Team 8492 defined hardware
 */
public class Drone extends BaseHardware {

    private ElapsedTime runtime = new ElapsedTime();
    /**
     * The {@link #telemetry} field contains an object in which a user may accumulate data which
     * is to be transmitted to the driver station. This data is automatically transmitted to the
     * driver station on a regular, periodic basis.
     */
    public Telemetry telemetry = null;
    private boolean cmdComplete = true;

    private Servo DS1;

    /**
     * Hardware Mappings
     */
    public HardwareMap hardwareMap = null; // will be set in Child class
    private final double preLaunch  = 0.000;
    private final double launch  = 1;
    private final double holdTime = 1500; // in milliseconds
    private Mode CurrentMode = Mode.STOP;




    /**
     * BaseHardware constructor
     * <p>
     * The op mode name should be unique. It will be the name displayed on the driver station. If
     * multiple op modes have the same name, only one will be available.
     */

    /**
     * User defined init method
     * <p>
     * This method will be called once when the INIT button is pressed.
     */
    public void init(){
        DS1 = hardwareMap.get(Servo.class,"DS1");



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
        switch (CurrentMode){
            case STOP:

                break;
            case LAUNCH:
              if (runtime.milliseconds() > holdTime){
                  CurrentMode = Mode.RESET;
              }
                break;
            case RESET:
              DS1.setPosition(preLaunch);
              CurrentMode = Mode.STOP;
                break;
        }

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
    public void launchDrone (){
        DS1.setPosition(launch);
        runtime.reset();
        CurrentMode = Mode.LAUNCH;

    }

public enum Mode{
    STOP,
   LAUNCH,
    RESET;




}






}

