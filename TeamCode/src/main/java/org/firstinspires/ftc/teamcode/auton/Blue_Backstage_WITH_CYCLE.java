package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.Settings;
import org.firstinspires.ftc.teamcode.hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.hardware.Lift;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.Sweeper;

//@Disabled
@Autonomous(name = "Blue_Backstage_WITH_CYCLE", group = "Auton")
// @Autonomous(...) is the other common choice

public class Blue_Backstage_WITH_CYCLE extends OpMode {

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
                        PlaceDistance = 23;
                        break;
                    case RIGHT:
                        currentStage = stage._30_DriveTo_spike_center;
                        PlaceDistance = 29;
                        break;
                    case NONE:
                        currentStage = stage._20_DriveTo_spike_right;
                        PlaceDistance = 34;
                        break;
                    default:
                        currentStage = stage._30_DriveTo_spike_center;
                        PlaceDistance = 29;
                }



                break;

            case _20_DriveTo_spike_right:
                if(robot.driveTrain.getCmdComplete()){
                    robot.driveTrain.CmdDrive(16,14,0.35,31);
                    currentStage = stage._22_Drive_Back;



                }

                break;
            case _22_Drive_Back:
                if(robot.driveTrain.getCmdComplete()) {
                    robot.driveTrain.CmdDrive(14.5, -166, 0.35, 31);
                    currentStage = stage._50_Turn_To_Backdrop;

                }
                break;


            case _30_DriveTo_spike_center:
                if(robot.driveTrain.getCmdComplete()) {
                    robot.driveTrain.CmdDrive(22, 0, 0.35, 0);
                    currentStage = stage._32_Drive_Back;

                }

                break;
            case _32_Drive_Back:
                if(robot.driveTrain.getCmdComplete()) {
                    robot.driveTrain.CmdDrive(18, -175, 0.35, 0);
                    currentStage = stage._50_Turn_To_Backdrop; // already at heading zero


                }
                break;

            case _40_DriveTo_spike_left:
                if(robot.driveTrain.getCmdComplete()) {
                    robot.driveTrain.CmdDrive(13, -15, 0.35, -10);
                    currentStage = stage._42_Drive_Back;

                }

                break;
            case _42_Drive_Back:
                if(robot.driveTrain.getCmdComplete()) {
                    robot.driveTrain.CmdDrive(12, 165, 0.35, -10);
                    currentStage = stage._50_Turn_To_Backdrop;

                }
                break;


            case _50_Turn_To_Backdrop:
                if(robot.driveTrain.getCmdComplete()) {
                    robot.driveTrain.CmdDrive(1, -90, 0.35, -90);
                    currentStage = stage._60_Strafe_Left;

                }
                break;

            case _60_Strafe_Left:
                if(robot.driveTrain.getCmdComplete()){
                    robot.driveTrain.cmdDriveBySensors(17,90,0.35,-90);
                    //robot.driveTrain.CmdDrive(40, -93, 0.35, -90);

                    currentStage = stage._65_Drive_Back; //stage._70_Strafe_Left_Wall;
                }
                break;

            case _65_Drive_Back:

                if (robot.driveTrain.getCmdComplete()) {
                    robot.driveTrain.cmdDriveBySensors(PlaceDistance, 0, 0.35, -90,
                            DriveTrain.SensorSel.LEFT_SIDE);
                    currentStage = stage._85_Arm_To_Position;
                }

                break;


            case _85_Arm_To_Position:
                if(robot.driveTrain.getCmdComplete())     {
                    robot.lift.setCurrentMode(Lift.Mode.DELIVER);
                    runtime.reset();
                    currentStage = stage._87_Drive_To_Backdrop;
                }

                break;

            case _87_Drive_To_Backdrop:
                if (runtime.milliseconds() > 1750) {
                    robot.driveTrain.CmdDrive(3,-90,0.35,-90);
                    currentStage = stage._90_Place;
                }

                break;

            case _90_Place:
                if (robot.driveTrain.getCmdComplete()) {
                    robot.lift.openDoor();
                    runtime.reset();
                    currentStage = stage._92_Drive_Away_From_Backdrop;
                }

                break;

            case _92_Drive_Away_From_Backdrop:

                if (runtime.milliseconds() > 1000) {
                    robot.driveTrain.CmdDrive(2.5,90,0.35,-90);
                    currentStage = stage._95_Park_And_Turn;
                }

                break;

            case _95_Park_And_Turn:
                if (robot.driveTrain.getCmdComplete()) {
                    robot.driveTrain.cmdDriveBySensors(5,0,0.35,-90, DriveTrain.SensorSel.LEFT_SIDE);
                    robot.lift.setCurrentMode(Lift.Mode.INTAKE);
                    currentStage = stage._100_Drive_To_Wing;
                }
                break;

            case _100_Drive_To_Wing:
                if (robot.driveTrain.getCmdComplete()) {
                    robot.driveTrain.CmdDrive(84,90,0.35,-90);
                    currentStage = stage._110_Drive_By_Sensor_To_Wing;
                }


                break;


            case _110_Drive_By_Sensor_To_Wing:
                if (robot.driveTrain.getCmdComplete()) {
                    robot.driveTrain.cmdDriveBySensors(5.5,-90,0.35,-90,
                    DriveTrain.SensorSel.REAR);
                    currentStage = stage._120_Drive_By_Sensor_To_Stack;
                }

                break;

            case _120_Drive_By_Sensor_To_Stack:
                if (robot.driveTrain.getCmdComplete()) {
                    robot.driveTrain.cmdDriveBySensors(27,0,0.35,-90, DriveTrain.SensorSel.LEFT_SIDE);
                    currentStage = stage._130_Open_And_Start_Intake;
                }

                break;

            case _130_Open_And_Start_Intake:
                if (robot.driveTrain.getCmdComplete()) {
                    robot.lift.openDoor();
                    robot.sweeper.setCurrentMode(Sweeper.Mode.FORWARD);
                    runtime.reset();
                //    robot.driveTrain.cmdDriveBySensors(29,0,0.35,-90);
                    currentStage = stage._140_Drive_To_Stack;
                }

                break;

            case _140_Drive_To_Stack:
                if (runtime.milliseconds() > 250 ) {

                    robot.driveTrain.cmdDriveBySensors(3,-90,0.35,-90,
                            DriveTrain.SensorSel.REAR);
                    runtime.reset();
                    currentStage = stage._150_Close_And_Stop_Intake;
                }

                break;
            case _150_Close_And_Stop_Intake:

                if ((robot.lift.boxCount() == 2) || (runtime.milliseconds() > 4000)) {
                    robot.lift.closeDoor();
                    robot.sweeper.setCurrentMode(Sweeper.Mode.STOP);
                    runtime.reset();
                    currentStage = stage._160_Drive_To_Side_Wall;
                }

                break;
            case _160_Drive_To_Side_Wall:

                if(runtime.milliseconds() > 250){
                    robot.driveTrain.cmdDriveBySensors(5,0,0.35,-90, DriveTrain.SensorSel.LEFT_SIDE);
                    currentStage = stage._170_Start_Backstage_Drive;
                }

                break;
            case _170_Start_Backstage_Drive:
                 if(robot.driveTrain.getCmdComplete()){
                     robot.driveTrain.CmdDrive(72, -90,0.35,-90);
                     currentStage = stage._260_Strafe_Left;
                 }

                break;

            case _260_Strafe_Left:
                if(robot.driveTrain.getCmdComplete()){
                    robot.driveTrain.cmdDriveBySensors(17,90,0.35,-90);
                    //robot.driveTrain.CmdDrive(40, -93, 0.35, -90);

                    currentStage = stage._265_Drive_Back; //stage._70_Strafe_Left_Wall;
                }
                break;

            case _265_Drive_Back:

                if (robot.driveTrain.getCmdComplete()) {
                    robot.driveTrain.cmdDriveBySensors(PlaceDistance, 0, 0.35, -90,
                            DriveTrain.SensorSel.LEFT_SIDE);
                    currentStage = stage._285_Arm_To_Position;
                }

                break;


            case _285_Arm_To_Position:
                if(robot.driveTrain.getCmdComplete())     {
                    robot.lift.setCurrentMode(Lift.Mode.DELIVER);
                    runtime.reset();
                    currentStage = stage._287_Drive_To_Backdrop;
                }

                break;

            case _287_Drive_To_Backdrop:
                if (runtime.milliseconds() > 1750) {
                    robot.driveTrain.CmdDrive(3,-90,0.35,-90);
                    currentStage = stage._290_Place;
                }

                break;

            case _290_Place:
                if (robot.driveTrain.getCmdComplete()) {
                    robot.lift.openDoor();
                    runtime.reset();
                    currentStage = stage._292_Drive_Away_From_Backdrop;
                }

                break;

            case _292_Drive_Away_From_Backdrop:

                if (runtime.milliseconds() > 1000) {
                    robot.driveTrain.CmdDrive(2.5,90,0.35,-90);
                    currentStage = stage._295_Park_And_Turn;
                }

                break;

            case _295_Park_And_Turn:
                if (robot.driveTrain.getCmdComplete()) {
                    robot.driveTrain.cmdDriveBySensors(5,0,0.35,0);
                    robot.lift.setCurrentMode(Lift.Mode.INTAKE);
                    currentStage = stage._300_End;
                }
                break;





            case _300_End:
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
        _65_Drive_Back,

        _85_Arm_To_Position,
        _87_Drive_To_Backdrop,
        _90_Place,
        _92_Drive_Away_From_Backdrop,
        _95_Park_And_Turn,
        _97_Drive_To_Corner,
        _100_Drive_To_Wing,
        _110_Drive_By_Sensor_To_Wing,
        _120_Drive_By_Sensor_To_Stack,
        _130_Open_And_Start_Intake,
        _140_Drive_To_Stack,
        _150_Close_And_Stop_Intake,
        _160_Drive_To_Side_Wall,
        _170_Start_Backstage_Drive,
        _180_Drive_By_Sensor_To_Backstage,
        _260_Strafe_Left,
        _265_Drive_Back,

        _285_Arm_To_Position,
        _287_Drive_To_Backdrop,
        _290_Place,
        _292_Drive_Away_From_Backdrop,
        _295_Park_And_Turn,
        _300_End


    }
}