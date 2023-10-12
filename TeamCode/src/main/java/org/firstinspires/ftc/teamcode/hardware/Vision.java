package org.firstinspires.ftc.teamcode.hardware;
import static java.lang.System.currentTimeMillis;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.auton.AutonVisionTest;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.List;

/**
 * Base class for FTC Team 8492 defined hardware
 */
public class Vision extends BaseHardware {
    private long desiredTagLastSeen_mSec = 0;   //system time stamp for when the desired tag is seen
    private ElapsedTime runtime = new ElapsedTime();
    private boolean targetFound = false;
    private int desiredTagId = -1;
    private boolean USE_WEBCAM = true;
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag
    private double drive = 0.0;
    private double strafe = 0.0;
    private double turn = 0.0;
    public final double SPEED_GAIN = 0.02;
    public final double TURN_GAIN = 0.02;
    public final double STRAFE_GAIN = 0.02;


    // This value is a guess of how far back the camera is from the front of the robot
    private double desiredDistanceInches = 8.5;

    private Mode  Current_Mode = Mode.VISION_ON;
    List<AprilTagDetection> currentDetections =  null;
    /**
     * The {@link #telemetry} field contains an object in which a user may accumulate data which
     * is to be transmitted to the driver station. This data is automatically transmitted to the
     * driver station on a regular, periodic basis.
     */
    public Telemetry telemetry = null;
    private boolean cmdComplete = true;

    /**
     * Hardware Mappings
     */
    public HardwareMap hardwareMap = null; // will be set in Child class
    /**
     * User defined init method
     * This method will be called once when the INIT button is pressed.
     */
    public void init(){
       initAprilTag();
    }
    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Create the vision portal by using a builder.
        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }
    /**
     * User defined init_loop method
     * This method will be called repeatedly when the INIT button is pressed.
     * This method is optional. By default this method takes no action.
     */
     public void init_loop() {
     }

    /**
     * User defined start method.
     * This method will be called once when the PLAY button is first pressed.
     * This method is optional. By default this method takes not action.
     * Example usage: Starting another thread.
     */
    public void start(){
        visionOn();
    }

    /**
     * User defined loop method
     * This method will be called repeatedly in a loop while this op mode is running
     */
    public void loop(){

        switch(Current_Mode) {
            case VISION_ON: {
                // Step through the list of detected tags and look for a matching tag
                List<AprilTagDetection> currentDetections = aprilTag.getDetections();
                targetFound = false;
                if (canSeeDesiredTag()) {
                    /* desiredTag is stored by canSeeTag */
                    double rangeError = (desiredTag.ftcPose.range - desiredDistanceInches);
                    double headingError = desiredTag.ftcPose.bearing;
                    double yawError = desiredTag.ftcPose.yaw;
                    // Use the speed and turn "gains" to calculate how we want the robot to move.

                    //TODO: define SPEED_GAIN, TURN_GAIN, STRAFE_GAIN in drivetrain class and make them public
                   // TODO: get minPower and maxPower from drivetrain

                    drive = rangeError * SPEED_GAIN;
                    turn = headingError * TURN_GAIN;
                    strafe = -yawError * STRAFE_GAIN;

                   // TODO: create setter for drive, turn, and strafe for drive train to call and get in loop
                    //TODO:

                }
                break;
            }
            default: {
                // if it is not ON then it must be off
                break;
            }
        }

    }

    public void doStop(){
        visionOff();
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
        visionOff();
    }

    /* turns on the vision system */
    public void visionOn() {
        Current_Mode = Mode.VISION_ON;
    }
    /* turns off the vision system (save cpu cycles when vision is not needed like TeleOp) */
    public void visionOff () {
        Current_Mode = Mode.VISION_OFF;
    }
    /* set the tag ID that we are looking for and desired distance from said tag */
    public void setTagAndDistance (int tagID, double inchesFromTag) {
        desiredDistanceInches = inchesFromTag;
        desiredTagId = tagID;
    }
    /*
     * returns true if the desired tag is seen by the camera It does store both the last seen time
     * and allows for computation of the distances and drive, strafe and turn values
     */
    private boolean canSeeDesiredTag () {
        boolean targetFound = false;
        /* loop through and find out if we can see it */
        for (AprilTagDetection detection : currentDetections) {
            if ((detection.metadata != null) &&
                    ((desiredTagId < 0) || (detection.id == desiredTagId))  ){
                targetFound = true;
                /* store the full detection for later use */
                desiredTag = detection;
                desiredTagLastSeen_mSec = currentTimeMillis();
                break;  // for loop don't look any further.
            } else {
                telemetry.addData("Unknown Target", "Tag ID %d is not in TagLibrary\n", detection.id);
            }
        }
        return (targetFound);
    }

    /*
    Way to tell just how long since the last time the desired tag was seen.   Normally this should
    be less than 1 second or 1000 mSec.
     */
    public long getDesiredTag_staleTime_mSec () {
        return (currentTimeMillis() - desiredTagLastSeen_mSec);
    }

    /*
    * returns true if the tag is seen by the camera does not store anything
    * useful to test and see what is visible
    */
    public boolean canSeeTag (int tagToLookFor) {
        boolean targetFound = false;
        /* loop through and find out if we can see it */
        for (AprilTagDetection detection : currentDetections) {
            if ((detection.metadata != null) &&
                    ((tagToLookFor < 0) || (detection.id == tagToLookFor))  ){
                targetFound = true;
                /* store the full detection for later use */
                break;  // for loop don't look any further.
            } else {
                telemetry.addData("Unknown Target", "Tag ID %d is not in TagLibrary\n", detection.id);
            }
        }
        return (targetFound);
    }
    public enum Mode{
        VISION_ON,
        VISION_OFF
    }
    public double getDrive(){
        return drive;
    }
    public double getStrafe(){
        return strafe;
    }
    public double getTurn(){
        return turn;
    }
}








