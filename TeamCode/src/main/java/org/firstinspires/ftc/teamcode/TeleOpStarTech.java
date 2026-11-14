package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.ManualDrive;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp(name = "Example TeleOp")
public class TeleOpStarTech extends OpMode {

    private DcMotorEx leftFront;
    private DcMotorEx rightFront;
    private DcMotorEx leftBack;
    private DcMotorEx rightBack;

    private double SLOW_DOWN_FACTOR = 1.0;
    private boolean slowMode = false;


    Gamepad currentGamepad1 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();

    @Override
    public void init() {
        //follower = Constants.create(hardwareMap);
        //robot.init(hardwareMap);
        // 1. HARDVERES INICIALIZÁLÁS: Lekérjük a motorokat a portnevek alapján
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");   // Bal dead wheel pod is ide van kötve
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront"); // Oldalsó (Strafe) dead wheel pod is ide van kötve
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");   // Jobb dead wheel pod is ide van kötve

        // 2. FORGÁSIRÁNYOK: Beállítjuk a motorok irányát a Constants konfigurációd alapján
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        // 3. ENKÓDER ÜZEMMÓD: Nullázzuk a portok enkóder számlálóit, majd RUN_WITHOUT_ENCODER módra váltunk
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // 4. SZABADONFUTÁS: Szabadonfutóra (FLOAT) állítjuk őket, hogy kézzel könnyen tolható legyen a robot teszt közben
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry.addLine("Hardver sikeresen inicializálva!");
        telemetry.update();
    }

    @Override
    public void loop() {

        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);

        handleManualDrive();
        handleGamepadControls();


        // --- ENKÓDER LIVE VISSZAJELZÉS ---
        // Kiolvassuk az aktuális nyers számláló állásokat az adott portokról
        int leftPodTicks = leftFront.getCurrentPosition();   // Bal enkóder pod
        int rightPodTicks = rightBack.getCurrentPosition();  // Jobb enkóder pod
        int strafePodTicks = rightFront.getCurrentPosition(); // Oldalsó (Strafe) enkóder pod

        // Telemetria adatok küldése a Driver Station / Drive Hub kijelzőjére
        telemetry.addLine("=== DIAGNOSZTIKA: DEAD WHEEL TICK-EK ===");
        telemetry.addData("BAL Pod (leftFront port)", leftPodTicks);
        telemetry.addData("JOBB Pod (rightBack port)", rightPodTicks);
        telemetry.addData("OLDAL Pod (rightFront port)", strafePodTicks);

        telemetry.addLine("\n=== PEDRO PATHING IRÁNY ELLENŐRZÉS ===");
        telemetry.addLine("• Kézzel ELŐRE tolva: BAL és JOBB értéknek NÖVEKEDNIE (+) kell.");
        telemetry.addLine("• Kézzel BALRA tolva: OLDAL értéknek NÖVEKEDNIE (+) kell.");

        telemetry.addLine("\n=== TELEOP ÁLLAPOT ===");
        telemetry.addData("Slow Mode", slowMode ? "AKTÍV (0.3)" : "KIKAPCSOLVA (1.0)");

        telemetry.update();
        /*follower.update();
        Pose robotPose = follower.pose(); // returns a Pose object
        telemetry.addData("Robot X", robotPose.x());
        telemetry.addData("Robot Y", robotPose.y());
        telemetry.addData("Robot Heading", Math.toDegrees(robotPose.heading()));*/
        // Math.toDegrees() is a built-in java method
    }

    private void handleManualDrive() {
        //HardwareBox robot = new HardwareBox();
        //private Follower follower;
        double SLOW_DOWN_FACTOR = slowMode ? 0.3 : 1.0;
        // This command is ignored by the follower when a path is active.
        /*DrivePowers powers = ManualDrive.fieldCentric(
                -gamepad1.left_stick_y * SLOW_DOWN_FACTOR,
                gamepad1.left_stick_x * SLOW_DOWN_FACTOR,
                gamepad1.right_stick_x * SLOW_DOWN_FACTOR,
                follower.pose().heading()
        );
        follower.manual(powers);*/
        // Joystick értékek beolvasása (az FTC-ben a Y tengely felprogramozva fordított, ezért kell a negatív előjel)
        double y = -currentGamepad1.left_stick_y;  // Előre-hátra mozgás
        double x = currentGamepad1.left_stick_x;   // Oldalazás (Strafe)
        double rx = currentGamepad1.right_stick_x; // Fordulás (Rotation)

        // Mecanum hajtásképlet a motorok erejének kiszámításához
        double leftFrontPower  = y + x + rx;
        double leftBackPower   = y - x + rx;
        double rightFrontPower = y - x - rx;
        double rightBackPower  = y + x - rx;

        // Értékek normalizálása, hogy ne lépjék túl az 1.0 / -1.0 tartományt
        double max = Math.max(Math.max(Math.abs(leftFrontPower), Math.abs(leftBackPower)),
                Math.max(Math.abs(rightFrontPower), Math.abs(rightBackPower)));


        if (max > 1.0) {
            leftFrontPower  /= max;
            leftBackPower   /= max;
            rightFrontPower /= max;
            rightBackPower  /= max;
        }

        // Motorok meghajtása a lassítási faktor figyelembevételével
        leftFront.setPower(leftFrontPower * SLOW_DOWN_FACTOR);
        leftBack.setPower(leftBackPower * SLOW_DOWN_FACTOR);
        rightFront.setPower(rightFrontPower * SLOW_DOWN_FACTOR);
        rightBack.setPower(rightBackPower * SLOW_DOWN_FACTOR);
    }

    private void handleGamepadControls(){
        if (currentGamepad1.a && !previousGamepad1.a) {
            slowMode = !slowMode;
        };

        /*if (gamepad1.startWasPressed()) {
            Pose cornerPose = new Pose(10.5, 10.5, Math.toRadians(90));
            // On the fly Pose creation, we dont recommend this for Autonomous. Only accepts radians for heading
            follower.setPose(cornerPose); // overrides our pose
        }*/


    }
}
