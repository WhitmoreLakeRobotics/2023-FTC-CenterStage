package org.firstinspires.ftc.teamcode.hardware;

import android.widget.Switch;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.Tele_Op;
import org.firstinspires.ftc.teamcode.common.CommonLogic;
import org.firstinspires.ftc.teamcode.common.Settings;


/**
 * Base class for FTC Team 8492 defined hardware
 */
public class DriveTrain extends BaseHardware {
    private DcMotor LDM1 ;
    private DcMotor LDM2 ;
    private DcMotor RDM1 ;
    private DcMotor RDM2 ;

    /**
     * The {@link #telemetry} field contains an object in which a user may accumulate data which
     * is to be transmitted to the driver station. This data is automatically transmitted to the
     * driver station on a regular, periodic basis.
     */
    public Telemetry telemetry = null;

    /**
     * Hardware Mappings
     */
    public HardwareMap hardwareMap = null; // will be set in Child class
    public Mode Current_Mode;

    private boolean cmdComplete = false;



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
    public void init() {

        RDM1 = hardwareMap.dcMotor.get("RDM1");
        LDM1 = hardwareMap.dcMotor.get("LDM1");
        LDM2 = hardwareMap.dcMotor.get("LDM2");
        RDM2 = hardwareMap.dcMotor.get("RDM2");

        if (LDM1 == null) {
            telemetry.log().add("LDM1 is null...");
        }
        if (LDM2 == null) {
            telemetry.log().add("LDM2 is null...");
        }
        if (RDM1 == null) {
            telemetry.log().add("RDM1 is null...");
        }
        if (RDM2 == null) {
            telemetry.log().add("RDM2 is null...");
        }

        LDM1.setDirection(DcMotor.Direction.REVERSE);
        LDM2.setDirection(DcMotor.Direction.REVERSE);
        RDM1.setDirection(DcMotor.Direction.REVERSE);
        RDM2.setDirection(DcMotor.Direction.FORWARD);

        LDM1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LDM2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RDM1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RDM2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LDM1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LDM2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RDM1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RDM2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Drive Train", "Initialized");
        Current_Mode = Mode.STOPPED;

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
    public void start() {
    }
    /**
     * User defined loop method
     * <p>
     * This method will be called repeatedly in a loop while this op mode is running
     */
    public void loop() {
        switch(Current_Mode){
            case TELEOP:

                break;
            case STOPPED:

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

    public void cmdTeleOp(double Left_Y, double Left_X, double Right_X, double Current_Speed) {
        cmdComplete = false;
        drivetrain_mode_Current = Mode.TELEOP;
        double Drive = Left_Y * Current_Speed;
        double Strafe = Left_X * Current_Speed;
        double Turn = Right_X * DRIVETRAIN_TURNSPEED;
        double Heading = subGyro.getGyroHeadingRadian();
        double NDrive = Strafe * Math.sin(Heading) + Drive * Math.cos(Heading);
        double NStrafe = Strafe * Math.cos(Heading) - Drive * Math.sin(Heading);
        // Adapted mecanum drive from link below
        // https://github.com/brandon-gong/ftc-mecanum

        LDM1Power = NDrive + NStrafe + Turn;
        RDM1Power = NDrive - NStrafe - Turn;
        LDM2Power = NDrive - NStrafe + Turn;
        RDM2Power = NDrive + NStrafe - Turn;

        RobotLog.aa(TAGChassis, "LDM1Power: " + LDM1Power +" LDM2Power: " + LDM2Power
                + " RDM1Power: " + RDM1Power +" RDM2Power: " + RDM2Power);
        RobotLog.aa(TAGChassis, "Left_X: " + Left_X +" Left_Y: " + Left_Y
                + " Right_X: " + Right_X + " Heading " + Heading);






        doTeleop();
    }
    public void doTeleop() {
        drivetrain_mode_Current = Mode.TELEOP;
        //Cap the power limit for the wheels
        double LDM1P = CommonLogic.CapValue(LDM1Power,
                minPower, maxPower);

        //Cap the power limit for the wheels
        double LDM2P = CommonLogic.CapValue(LDM2Power,
                minPower, maxPower);

        double RDM1P = CommonLogic.CapValue(RDM1Power,
                minPower, maxPower);

        double RDM2P = CommonLogic.CapValue(RDM2Power,
                minPower, maxPower);




        LDM1.setPower(LDM1P);
        RDM1.setPower(RDM1P);
        LDM2.setPower(LDM2P);
        RDM2.setPower(RDM2P);
        RobotLog.aa(TAGChassis, "doTeleop: LDM1Power =" + LDM1P + " RDM1Power =" + RDM1P +
                " LDM2Power =" + LDM2P + " RDM2Power =" + RDM2P);
    }

public enum Mode{

    STOPPED,
    TELEOP;
}
}