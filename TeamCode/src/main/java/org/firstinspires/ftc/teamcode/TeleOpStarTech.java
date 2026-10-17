package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.ManualDrive;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp(name = "Example TeleOp")
public class TeleOpStarTech extends OpMode {

    HardwareBox robot = new HardwareBox();
    private Follower follower;
    private double SLOW_DOWN_FACTOR = 1.0;
    private boolean slowMode = false;


    Gamepad currentGamepad1 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        robot.init(hardwareMap);
    }

    @Override
    public void loop() {

        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);

        handleManualDrive();
        handleGamepadControls();


        follower.update();
        Pose robotPose = follower.pose(); // returns a Pose object
        telemetry.addData("Robot X", robotPose.x());
        telemetry.addData("Robot Y", robotPose.y());
        telemetry.addData("Robot Heading", Math.toDegrees(robotPose.heading()));
        // Math.toDegrees() is a built-in java method
    }

    private void handleManualDrive() {
        SLOW_DOWN_FACTOR = slowMode ? 0.3 : 1.0;
        // This command is ignored by the follower when a path is active.
        DrivePowers powers = ManualDrive.fieldCentric(
                -gamepad1.left_stick_y * SLOW_DOWN_FACTOR,
                gamepad1.left_stick_x * SLOW_DOWN_FACTOR,
                gamepad1.right_stick_x * SLOW_DOWN_FACTOR,
                follower.pose().heading()
        );
        follower.manual(powers);
    }

    private void handleGamepadControls(){
        if (currentGamepad1.a && !previousGamepad1.a) {
            slowMode = !slowMode;
        };

        if (gamepad1.startWasPressed()) {
            Pose cornerPose = new Pose(10.5, 10.5, Math.toRadians(90));
            // On the fly Pose creation, we dont recommend this for Autonomous. Only accepts radians for heading
            follower.setPose(cornerPose); // overrides our pose
        }

    }
}
