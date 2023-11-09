package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.Settings;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.Sweeper;

//@Disabled
@Autonomous(name = "Blue_Backstage_Outer_Backdrop_Place", group = "Auton")
// @Autonomous(...) is the other common choice

public class Blue_Backstage_Outer_Backdrop_Place extends OpMode {

    //RobotComp robot = new RobotComp();
    Robot robot = new Robot();




    private stage currentStage = stage._unknown;
    // declare auton power variables
    //private double AUTO_DRIVE_TURBO_SPEED = DriveTrain.DRIVETRAIN_TURBOSPEED;
    //private double AUTO_DRIVE_SLOW_SPEED = DriveTrain.DRIVETRAIN_SLOWSPEED;
   // private double AUTO_DRIVE_NORMAL_SPEED = DriveTrain.DRIVETRAIN_NORMALSPEED;
   // private double AUTO_TURN_SPEED = DriveTrain.DRIVETRAIN_TURNSPEED;

    private String RTAG = "8492-Auton";

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    private int sweepTime = 1000;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    private Robot.SensorDetect ScanResults = Robot.SensorDetect.UNKNOWN;
    private int PlaceDistance = 0;
    @Override
    public void init() {
        //----------------------------------------------------------------------------------------------
        // These constants manage the duration we allow for callbacks to user code to run for before
        // such code is considered to be stuck (in an infinite loop, or wherever) and consequently
        // the robot controller application is restarted. They SHOULD NOT be modified except as absolutely
        // necessary as poorly chosen values might inadvertently compromise safety.
        //----------------------------------------------------------------------------------------------
        msStuckDetectInit = Settings.msStuckDetectInit;
        msStuckDetectInitLoop = Settings.msStuckDetectInitLoop;
        msStuckDetectStart = Settings.msStuckDetectStart;
        msStuckDetectLoop = Settings.msStuckDetectLoop;
        msStuckDetectStop = Settings.msStuckDetectStop;

        robot.hardwareMap = hardwareMap;
        robot.telemetry = telemetry;
        robot.init();
        telemetry.addData("Test Auton", "Initialized");

        //Initialize Gyro
        robot.driveTrain.ResetGyro();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        // initialize robot
        robot.init_loop();

    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        // start robot
        runtime.reset();
        robot.start();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        telemetry.addData("Auton_Current_Stage ", currentStage);
        robot.loop();

        switch (currentStage){
            case  _unknown:
                currentStage = stage._00_preStart;
                break;


            case _00_preStart:
                currentStage = stage._09_Scan;


                break;

            case _09_Scan:
                ScanResults = robot.CurrentDetect;
                currentStage = stage._10_Drive_Out;
                break;

            case _10_Drive_Out:
                robot.driveTrain.CmdDrive(1,0,0.35,0);
                switch (ScanResults){
                    case LEFT:
                        currentStage = stage._40_DriveTo_spike_left;
                        PlaceDistance = 22;
                        break;
                    case RIGHT:
                        currentStage = stage._30_DriveTo_spike_center;
                        PlaceDistance = 28;
                        break;
                    case NONE:
                        currentStage = stage._20_DriveTo_spike_right;
                        PlaceDistance = 34;
                        break;
                    default:
                        currentStage = stage._30_DriveTo_spike_center;
                        PlaceDistance = 28;
                }



                break;

            case _20_DriveTo_spike_right:
                if(robot.driveTrain.getCmdComplete()){
                    robot.driveTrain.CmdDrive(18,12,0.35,31);
                    currentStage = stage._22_Drive_Back;



                }

                break;
            case _22_Drive_Back:;
                if(robot.driveTrain.getCmdComplete()) {
                    robot.driveTrain.CmdDrive(19, -168, 0.35, 31);
                    currentStage = stage._50_Turn_To_Backdrop;

                }
                break;


            case _30_DriveTo_spike_center:
                if(robot.driveTrain.getCmdComplete()) {
                    robot.driveTrain.CmdDrive(28, 0, 0.35, 0);
                    currentStage = stage._32_Drive_Back;

                }

                break;
            case _32_Drive_Back:
                if(robot.driveTrain.getCmdComplete()) {
                    robot.driveTrain.CmdDrive(28, -175, 0.35, 0);
                    currentStage = stage._50_Turn_To_Backdrop; // already at heading zero


                }
                break;

            case _40_DriveTo_spike_left:
                if(robot.driveTrain.getCmdComplete()) {
                    robot.driveTrain.CmdDrive(14, -15, 0.35, -10);
                    currentStage = stage._42_Drive_Back;

                }

                break;
            case _42_Drive_Back:
                if(robot.driveTrain.getCmdComplete()) {
                    robot.driveTrain.CmdDrive(16, 165, 0.35, -10);
                    currentStage = stage._50_Turn_To_Backdrop;

                }
                break;


            case _50_Turn_To_Backdrop:
                if(robot.driveTrain.getCmdComplete()) {
                    robot.driveTrain.CmdDrive(0, 0, 0, -90);
                    currentStage = stage._60_Strafe_Left;

                }
                break;

            case _60_Strafe_Left:
                if(robot.driveTrain.getCmdComplete()){
                    robot.driveTrain.cmdDriveBySensors(13,-90,0.35,-90);   //.CmdDrive(16,-90,0.35,-90);
                    currentStage = stage._70_Strafe_Left_Wall;

                }
                break;

            case _70_Strafe_Left_Wall:
                if(robot.driveTrain.getCmdComplete()){
                    robot.driveTrain.CmdDrive(4, -179, 0.35, -90);
                    currentStage = stage._80_Strafe_Right;
                }
                break;

            case _80_Strafe_Right:
                  if(robot.driveTrain.getCmdComplete())     {

                    robot.driveTrain.CmdDrive(PlaceDistance,0,0.35,-90);
                    currentStage = stage._90_Place;
                }

                break;
            case _90_Place:




                break;
            case _95_Park_And_Turn:
                break;
            case _100_End:
                if(robot.driveTrain.getCmdComplete()){
                    robot.stop();


                }







                break;
        }



    }  //  loop

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        robot.stop();
    }

    private enum stage {
        _unknown,
        _00_preStart,
        _09_Scan,
        _10_Drive_Out,
        _20_DriveTo_spike_right,
        _22_Drive_Back,
        _30_DriveTo_spike_center,
        _32_Drive_Back,
        _40_DriveTo_spike_left,
        _42_Drive_Back,
        _50_Turn_To_Backdrop,
        _60_Strafe_Left,
        _70_Strafe_Left_Wall,
        _80_Strafe_Right,
        _90_Place,
        _95_Park_And_Turn,
        _100_End


    }
}