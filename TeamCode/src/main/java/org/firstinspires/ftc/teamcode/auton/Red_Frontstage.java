package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.Settings;
import org.firstinspires.ftc.teamcode.hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.hardware.Lift;
import org.firstinspires.ftc.teamcode.hardware.Robot;

//@Disabled
@Autonomous(name = "Red_Frontstage", group = "Auton")
// @Autonomous(...) is the other common choice

public class Red_Frontstage extends OpMode {

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
    private final int LeftDistance =4 ;
    private final int RightDistance =18 ;
    private final int CenterDistance= 13 ;
    private int StrafeDistance = 0;
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
        robot.lift.resetArmPos();
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
        robot.updateDriveSensor();
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
                robot.driveTrain.CmdDrive(3,0,0.35,0);
                switch (ScanResults){
                    case LEFT:
                        currentStage = stage._40_DriveTo_spike_left;
                        PlaceDistance = LeftDistance;

                        break;
                    case RIGHT:
                        currentStage = stage._30_DriveTo_spike_center;
                        PlaceDistance = CenterDistance;
                        break;
                    case NONE:
                        currentStage = stage._20_DriveTo_spike_right;
                        PlaceDistance = RightDistance;
                        break;
                    default:
                        currentStage = stage._30_DriveTo_spike_center;
                        PlaceDistance = CenterDistance;
                }



                break;

            case _20_DriveTo_spike_right:
                if(robot.driveTrain.getCmdComplete()){
                    robot.driveTrain.CmdDrive(15,18,0.35,31);
                    currentStage = stage._22_Drive_Back;



                }

                break;
            case _22_Drive_Back:
                if(robot.driveTrain.getCmdComplete()) {
                    robot.driveTrain.CmdDrive(18, -170, 0.35, 31);
                    currentStage = stage._50_Drive_Straight_To_Pivot;

                }
                break;


            case _30_DriveTo_spike_center:
                if(robot.driveTrain.getCmdComplete()) {
                    robot.driveTrain.CmdDrive(21, 0, 0.30, 0);
                    currentStage = stage._32_Drive_Back;

                }

                break;
            case _32_Drive_Back:
                if(robot.driveTrain.getCmdComplete()) {
                    robot.driveTrain.CmdDrive(7, -175, 0.35, 0);
                    currentStage = stage._48_Strafe_To_Side_Wall; // already at heading zero


                }
                break;

            case _40_DriveTo_spike_left:
                if(robot.driveTrain.getCmdComplete()) {
                    robot.driveTrain.CmdDrive(13, -19, 0.35, -12);
                    currentStage = stage._42_Drive_Back;

                }

                break;
            case _42_Drive_Back:
                if(robot.driveTrain.getCmdComplete()) {
                    robot.driveTrain.CmdDrive(12, 140, 0.35, -10);
                    currentStage = stage._50_Drive_Straight_To_Pivot;

                }
                break;
            case _48_Strafe_To_Side_Wall:
                if(robot.driveTrain.getCmdComplete()) {
                    robot.driveTrain.CmdDrive(12, -90, 0.35, 0);
                    currentStage = stage._50_Drive_Straight_To_Pivot;

                }
                break;

            case _50_Drive_Straight_To_Pivot:
                if(robot.driveTrain.getCmdComplete()) {
                    robot.driveTrain.CmdDrive(41, 0, 0.35, 0);
                    currentStage = stage._55_Turn_to_BackDrop;

                }
                break;

                case _55_Turn_to_BackDrop:
                if(robot.driveTrain.getCmdComplete()) {
                    robot.driveTrain.CmdDrive(2, 90, 0.35, 85);
                    currentStage = stage._57_Drive_Toward_BackDrop;

                }
                break;
            case _57_Drive_Toward_BackDrop:
                if(robot.driveTrain.getCmdComplete()) {
                    robot.driveTrain.CmdDrive(70, 90, 0.45, 90);
                    currentStage = stage._60_Drive_To_Backdrop_Wall;

                }
                break;

            case _60_Drive_To_Backdrop_Wall:
                if(robot.driveTrain.getCmdComplete()){
                    robot.driveTrain.cmdDriveBySensors(17,-90,0.35,90);
                    //robot.driveTrain.CmdDrive(40, -93, 0.35, -90);

                    currentStage = stage._65_Strafe_BackDrop_Edge; //stage._70_Strafe_Left_Wall;
                }
                break;

            case _65_Strafe_BackDrop_Edge:

                if (robot.driveTrain.getCmdComplete()) {
                    robot.driveTrain.CmdDrive(20, -179, 0.35, 90);
                    //robot.driveTrain.cmdDriveBySensors(6,0,0.35,90, DriveTrain.SensorSel.RIGHT_FRONT);
                    currentStage = stage._68_Drive_To_Edge_Of_Backdrop;
                }

                break;
            case _68_Drive_To_Edge_Of_Backdrop:
                if (robot.sensors.GetSensorDistanceRightFront() < 12) {
                    robot.driveTrain.Current_Mode = DriveTrain.Mode.STOPPED;
                    currentStage = stage._70_Strafe_To_Pos;
                }

                break;

            case _70_Strafe_To_Pos:
                if(robot.driveTrain.getCmdComplete()){
                    robot.driveTrain.CmdDrive(PlaceDistance, -179, 0.35, 90);
                    currentStage = stage._85_Drive_To_Backdrop;
                }
                break;

            case _85_Drive_To_Backdrop:
                  if(robot.driveTrain.getCmdComplete())     {
                    //
                    robot.driveTrain.cmdDriveBySensors(2.75,-90,0.35,90);
                    currentStage = stage._87_Arm_To_Position;
                }

                break;

            case _87_Arm_To_Position:
                if (robot.driveTrain.getCmdComplete()) {
                    robot.lift.setCurrentMode(Lift.Mode.DELIVER);
                    runtime.reset();
                    currentStage = stage._90_Place;
                }

                break;

            case _90_Place:
                if (runtime.milliseconds() > 1750) {
                    robot.lift.openDoor();
                    runtime.reset();
                    currentStage = stage._92_Drive_Away_From_Backdrop;
                }

                break;

            case _92_Drive_Away_From_Backdrop:

                if (runtime.milliseconds() > 1000) {
                    robot.driveTrain.CmdDrive(3,-90,0.35,90);
                    currentStage = stage._95_Park_And_Turn;
                }

                break;

            case _95_Park_And_Turn:
                if (robot.driveTrain.getCmdComplete()) {
                    robot.driveTrain.CmdDrive(PlaceDistance-12,0,0.35,0);
                    robot.lift.setCurrentMode(Lift.Mode.INTAKE);
                    currentStage = stage._97_Strafe_To_Wall;
                }
                break;

            case _97_Strafe_To_Wall:

                if (robot.driveTrain.getCmdComplete()) {
                    robot.driveTrain.CmdDrive(4,90,0.35,0);
                    currentStage = stage._100_End;
                }

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
        _48_Strafe_To_Side_Wall,
        _50_Drive_Straight_To_Pivot,
        _55_Turn_to_BackDrop,
        _57_Drive_Toward_BackDrop,
        _60_Drive_To_Backdrop_Wall,
        _65_Strafe_BackDrop_Edge,
        _68_Drive_To_Edge_Of_Backdrop,
        _70_Strafe_To_Pos,
        _80_Strafe_Right,
        _85_Drive_To_Backdrop,
        _87_Arm_To_Position,
        _90_Place,
        _92_Drive_Away_From_Backdrop,
        _95_Park_And_Turn,
        _97_Strafe_To_Wall,
        _100_End


    }
}