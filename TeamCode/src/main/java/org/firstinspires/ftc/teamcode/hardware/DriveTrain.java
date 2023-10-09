package org.firstinspires.ftc.teamcode.hardware;

import android.widget.Switch;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.Tele_Op;
import org.firstinspires.ftc.teamcode.common.CommonLogic;
import org.firstinspires.ftc.teamcode.common.Settings;
import org.firstinspires.ftc.teamcode.hardware.CommonGyro;


/**
 * Base class for FTC Team 8492 defined hardware
 */
public class DriveTrain extends BaseHardware {
    private DcMotor LDM1 ;
    private DcMotor LDM2 ;
    private DcMotor RDM1 ;
    private DcMotor RDM2 ;

    private CommonGyro Gyro = new CommonGyro();
    /**
     * The {@link #telemetry} field contains an object in which a user may accumulate data which
     * is to be transmitted to the driver station. This data is automatically transmitted to the
     * driver station on a regular, periodic basis.
     */
 //   public Telemetry telemetry = null;

    /**
     * Hardware Mappings
     */
    public HardwareMap hardwareMap = null; // will be set in Child class
    public Mode Current_Mode;

    private boolean cmdComplete = true;

    private static double TURNSPEED_TELEOP = 0.5;

    private double LDM1Power;
    private double LDM2Power;
    private double RDM1Power;
    private double RDM2Power;

    private double minPower = -1.0;
    private double maxPower = 1.0;

    private static final String TAGChassis = "8492 ";

    public static final double DTrain_NORMALSPEED = 0.5;
    public static final double DTrain_SLOWSPEED = 0.3;
    public static final double DTrain_FASTSPEED = 0.7;

    private double Drive_Start;  //in inches
    private double Drive_Target;  //in inches
    private static final double Distance_Per_Rev = 2.95*3.14159;
    private static final double Gear_Ratio = 1 / 10.4329;
    private  static final int Gyro_Tol  = 2; //was 3
    private static final double Ticks_Per_Inch = Settings.REV_HD_HEX_MOTOR_TICKS_PER_REV *  Gear_Ratio * Distance_Per_Rev;
    private double bearing_AA = 0;
    private double speed_AA = 0;
    private int Target_Heading;
    private static final double driveTolAA = 0.25; //in inches
    private static final double diaTurnRaid = 23; //in inches //was 23
    private static final double turnDistPerDeg = ((3.14159 * diaTurnRaid)/360) * Ticks_Per_Inch; //inches per deg
    private static final double stagPos = 10;
    private static final double stagPow = 0.60;



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

        Gyro.telemetry = telemetry;
        Gyro.hardwareMap = hardwareMap;
        Gyro.init();

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

        LDM1.setDirection(DcMotor.Direction.FORWARD);
        LDM2.setDirection(DcMotor.Direction.FORWARD);
        RDM1.setDirection(DcMotor.Direction.FORWARD);
        RDM2.setDirection(DcMotor.Direction.REVERSE);

        LDM1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LDM2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RDM1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RDM2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LDM1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LDM2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RDM1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RDM2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
        telemetry.addData("Gyro","Gyro "+Gyro.getGyroHeading());
        switch(Current_Mode){
            case TELEOP:

                break;
            case STOPPED:
                stopMotors();
                break;
            case COMMAND_AA:

                break;
            case DRIVE_AA:
                doDrive();
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
        Current_Mode = Mode.TELEOP;
        double Drive = Left_Y * Current_Speed;
        double Strafe = Left_X * Current_Speed;
        double Turn = Right_X * (1.0 -Left_Y) * TURNSPEED_TELEOP ;
        double Heading = Gyro.getGyroHeadingRadian();
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

        telemetry.addData(TAGChassis, "Left_X: " + Left_X +" Left_Y: " + Left_Y
                + " Right_X: " + Right_X + " Heading " + Heading);






        doTeleop();
    }
    public void doTeleop() {
        Current_Mode = Mode.TELEOP;
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

        telemetry.addData(TAGChassis, "doTeleop: LDM1Power =" + LDM1P + " RDM1Power =" + RDM1P +
                " LDM2Power =" + LDM2P + " RDM2Power =" + RDM2P);
    }
    public void setMaxPower(double newMax) {

        this.maxPower = Math.abs(newMax);
        this.minPower = Math.abs(newMax)* -1;
    }

    private void scaleMotorPower(){
        //figure out what was max
        double MaxValue = 0;
        if (Math.abs(LDM2Power)  > MaxValue){
            MaxValue = LDM2Power;
        }
        if (Math.abs(RDM1Power)  > MaxValue){
            MaxValue = RDM1Power;
        }
        if (Math.abs(RDM2Power)  > MaxValue) {
            MaxValue = RDM2Power;
        }
        if (Math.abs(LDM1Power)  > MaxValue){
            MaxValue = LDM1Power;
        }
        //divide each motor power by max power
        if (maxPower > 1) {
            scalePower(MaxValue);
        }
    }
    private  void scalePower(double ScalePower){
        LDM1Power=LDM1Power/ScalePower;
        LDM2Power=LDM2Power/ScalePower;
        RDM1Power=RDM1Power/ScalePower;
        RDM2Power=RDM2Power/ScalePower;

    }
    private  void updateMotorPower(){
        scaleMotorPower();
        LDM1Power = CommonLogic.CapValue(LDM1Power,
                Settings.REV_MIN_POWER, Settings.REV_MAX_POWER);

        LDM2Power= CommonLogic.CapValue(LDM2Power,
                Settings.REV_MIN_POWER, Settings.REV_MAX_POWER);
        RDM1Power = CommonLogic.CapValue(RDM1Power,
                Settings.REV_MIN_POWER, Settings.REV_MAX_POWER);

        RDM2Power= CommonLogic.CapValue(RDM2Power,
                Settings.REV_MIN_POWER, Settings.REV_MAX_POWER);

        LDM1.setPower(LDM1Power);
        LDM2.setPower(LDM2Power);
        RDM1.setPower(RDM1Power);
        RDM2.setPower(RDM2Power);

    }

    public void CmdDrive(double TargetDist,double Bearing, double speed, int Heading){
        //drive target needs to account turn distance

        Target_Heading = Heading;

    Drive_Target = (TargetDist) + ((Math.abs(Gyro.getGyroHeading() - Target_Heading)*Math.sqrt(2))/turnDistPerDeg);

    // reset encoders
     resetEncoders();
        // store Bearing
        bearing_AA = Bearing;
        //store speed
        speed_AA = speed;

        cmdComplete = false;
        Current_Mode = Mode.DRIVE_AA;
        startDrive();
    }
    public double calcTurn(int tHeading){

       double turn = CommonLogic.goToPosStag(Gyro.getGyroHeading(),tHeading, Gyro_Tol,
               1.0, stagPos, stagPow);
        telemetry.addData(TAGChassis,"turn power " + turn);
        return turn;
    }


    private void startDrive(){
        double Left_Y = Math.cos(Math.toRadians(bearing_AA));
        double Drive = Left_Y * speed_AA;
        double Strafe = Math.sin(Math.toRadians(bearing_AA)) * speed_AA;
        double Turn = calcTurn(Target_Heading) * (1.01 -Drive) * TURNSPEED_TELEOP ;
        double Heading = Gyro.getGyroHeadingRadian();
        double NDrive = Strafe * Math.sin(Heading) + Drive * Math.cos(Heading);
        double NStrafe = Strafe * Math.cos(Heading) - Drive * Math.sin(Heading);
        // Adapted mecanum drive from link below
        // https://github.com/brandon-gong/ftc-mecanum

        LDM1Power = NDrive + NStrafe + Turn;
        RDM1Power = NDrive - NStrafe - Turn;
        LDM2Power = NDrive - NStrafe + Turn;
        RDM2Power = NDrive + NStrafe - Turn;

       /* RobotLog.aa(TAGChassis, "LDM1Power: " + LDM1Power +" LDM2Power: " + LDM2Power
           //     + " RDM1Power: " + RDM1Power +" RDM2Power: " + RDM2Power);
        RobotLog.aa(TAGChassis, "Left_X: " + Left_X +" Left_Y: " + Left_Y
                + " Right_X: " + Right_X + " Heading " + Heading);

        telemetry.addData(TAGChassis, "Left_X: " + Left_X +" Left_Y: " + Left_Y
                + " Right_X: " + Right_X + " Heading " + Heading);

        */

        //Cap the power limit for the wheels
      scaleMotorPower();
       /* double LDM1P = CommonLogic.CapValue(LDM1Power,
                minPower, maxPower);

        //Cap the power limit for the wheels
        double LDM2P = CommonLogic.CapValue(LDM2Power,
                minPower, maxPower);

        double RDM1P = CommonLogic.CapValue(RDM1Power,
                minPower, maxPower);

        double RDM2P = CommonLogic.CapValue(RDM2Power,
                minPower, maxPower);


        */



        LDM1.setPower(LDM1Power);
        RDM1.setPower(RDM1Power);
        LDM2.setPower(LDM2Power);
        RDM2.setPower(RDM2Power);
        RobotLog.aa(TAGChassis, "doTeleop: LDM1Power =" + LDM1Power + " RDM1Power =" + RDM1Power +
                " LDM2Power =" + LDM2Power + " RDM2Power =" + RDM2Power);

        telemetry.addData(TAGChassis, "doTeleop: LDM1Power =" + LDM1Power + " RDM1Power =" + RDM1Power +
                " LDM2Power =" + LDM2Power + " RDM2Power =" + RDM2Power);

    }
    private void doDrive(){
        double distance = getPosInches();
        telemetry.addData(TAGChassis,"distance driven " + distance);
        startDrive();
    //check to see if we have driven the target distance
    if(Drive_Target <= distance) {
        //if we have reached our target distance
        //stop drive
        stopMotors();
        //mark command complete
            cmdComplete = true;
        //set current mode stop
        Current_Mode = Mode.STOPPED;
    }

        //if not keep driving

    }


    private void resetEncoders(){
        LDM1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LDM2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RDM1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RDM2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LDM1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LDM2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RDM1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RDM2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



    }
    private double getPosInches(){
        double values = Math.abs(LDM1.getCurrentPosition());
        values += Math.abs(LDM2.getCurrentPosition());
        values += Math.abs(RDM1.getCurrentPosition());
        values += Math.abs(RDM2.getCurrentPosition());
        values = values/4;

        values = values/Ticks_Per_Inch;

        return values;




    }
    public void QuickAligenment() {
   //The intention of this method is to return a turn value based upon a desired alingment direction
        //this should override the right joystick
    }

    private void stopMotors(){
        LDM1.setPower(0);
        LDM2.setPower(0);
        RDM1.setPower(0);
        RDM2.setPower(0);
    }

    public void ResetGyro(){
        Gyro.GyroInt();
    }


    public boolean getCmdComplete(){


        return cmdComplete;
    }
    public enum Mode{
    DRIVE_AA,
        COMMAND_AA,
    STOPPED,
    TELEOP;

}
}
