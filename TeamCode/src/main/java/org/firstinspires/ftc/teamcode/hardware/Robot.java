package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import org.firstinspires.ftc.teamcode.common.CommonLogic;

public class Robot extends BaseHardware {

    public DriveTrain driveTrain = new DriveTrain();
    public Lighting lighting = new Lighting();
    public Sensors sensors = new Sensors();
    public Lift lift = new Lift();
    public Sweeper sweeper = new Sweeper();
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

    }

    @Override
    public void init_loop() {
        driveTrain.init_loop();
        lighting.init_loop();
        sensors.init_loop();
        lift.init_loop();
        sweeper.init_loop();
    }

    @Override
    public void start() {
        driveTrain.start();
        lighting.start();
        sensors.start();
        lift.start();
        sweeper.start();
    }

    @Override
    public void loop() {
        driveTrain.loop();
        lighting.loop();
        sensors.loop();
        lift.loop();
        sweeper.loop();
    }


    @Override
    public void stop() {
        driveTrain.stop();
        lighting.stop();
        sensors.stop();
        lift.stop();
        sweeper.stop();
    }


}
