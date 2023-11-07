package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import org.firstinspires.ftc.teamcode.common.CommonLogic;

public class Robot extends BaseHardware {

    public DriveTrain driveTrain = new DriveTrain();
    public Lighting lighting = new Lighting();
    public Sensors sensors = new Sensors();
    public Lift lift = new Lift();
    public Sweeper sweeper = new Sweeper();
    public SensorDetect CurrentDetect = SensorDetect.UNKNOWN;
    public Drone drone = new Drone();

    @Override
    public void init() {
        // Must set Hardware Map and telemetry before calling init
        driveTrain.hardwareMap = this.hardwareMap;
        driveTrain.telemetry = this.telemetry;
        driveTrain.init();

           lighting.hardwareMap = this.hardwareMap;
        lighting.telemetry = this.telemetry;
        lighting.init();

        sensors.hardwareMap = this.hardwareMap;
        sensors.telemetry = this.telemetry;
        sensors.init();

        lift.hardwareMap = this.hardwareMap;
        lift.telemetry = this.telemetry;
        lift.init();

        sweeper.hardwareMap = this.hardwareMap;
        sweeper.telemetry = this.telemetry;
        sweeper.init();

        drone.hardwareMap = this.hardwareMap;
        drone.telemetry = this.telemetry;
        drone.init();

    }

    @Override
    public void init_loop() {
        driveTrain.init_loop();
        lighting.init_loop();
        sensors.init_loop();
        lift.init_loop();
        sweeper.init_loop();
        drone.init_loop();

        propCheck();
    }

    @Override
    public void start() {
        driveTrain.start();
        lighting.start();
        sensors.start();
        lift.start();
        sweeper.start();
        drone.start();

        lighting.UpdateBaseColor(RevBlinkinLedDriver.BlinkinPattern.WHITE);
    }

    @Override
    public void loop() {
        driveTrain.loop();
        lighting.loop();
        sensors.loop();
        lift.loop();
        sweeper.loop();
        drone.loop();

        updateDriveSensor();
    }


    @Override
    public void stop() {
        driveTrain.stop();
        lighting.stop();
        sensors.stop();
        lift.stop();
        sweeper.stop();
        drone.stop();

        lighting.UpdateBaseColor(RevBlinkinLedDriver.BlinkinPattern.WHITE);
    }
    public void propCheck(){

        boolean propISLeft = sensors.FLDS1Detect();
        boolean propIsRight = sensors.FRDS1Detect();

       if (propISLeft && ! propIsRight){
           lighting.UpdateBaseColor(RevBlinkinLedDriver.BlinkinPattern.RED);//detaced prop on left
            CurrentDetect = SensorDetect.LEFT;
       } else if (propIsRight && ! propISLeft) {
           lighting.UpdateBaseColor(RevBlinkinLedDriver.BlinkinPattern.BLUE);//detaced prop on right
           CurrentDetect = SensorDetect.RIGHT;
       }
       else if (propIsRight && propISLeft) {
           lighting.UpdateBaseColor(RevBlinkinLedDriver.BlinkinPattern.VIOLET);//detaced prop on center
           CurrentDetect = SensorDetect.BOTH;
       }
       else if (! propIsRight  && ! propISLeft) {
           lighting.UpdateBaseColor(RevBlinkinLedDriver.BlinkinPattern.GREEN);//detaced prop on
           CurrentDetect = SensorDetect.NONE;
       }
    }

    public void updateDriveSensor(){
        driveTrain.updateRange(sensors.GetSensorDistance());

    }
    public enum SensorDetect{
        LEFT,
        RIGHT,
        BOTH,
        NONE,
        UNKNOWN;
    }

}
