package org.firstinspires.ftc.teamcode;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.groups.Groups;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.api.Paths.*;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "STARTECH_MASTER_BIOBUZZ_V13", group = "Opmode")
public class Autonom extends OpMode {

    HardwareBox robot = new HardwareBox();

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();
    private final ElapsedTime autonomousTimer = new ElapsedTime();

    // Környezeti és stratégiai állapotváltozók
    private enum Alliance { RED, BLUE, UNDETERMINED }
    private enum StartStrategy {
        GARDEN_SIDE_CROSSOVER,   // 1. ESET: Saját oldal, Gardenből indul -> Átmegy a túloldalra
        SHOOTING_SIDE_LOCAL,     // 2. ESET: Saját oldal, helyben marad -> Csak az oldalsó Flower-t üríti
        BLIND_OPPONENT_START     // 3. ESET: Vak indulás -> Vár 5 másodpercet az ellenfél miatt
    }

    private Alliance currentAlliance = Alliance.UNDETERMINED;
    private StartStrategy currentStrategy = StartStrategy.BLIND_OPPONENT_START; // Alapértelmezetten vak
    private boolean positionCalibrated = false;
    private boolean autoSequenceFinished = false; // Alapból hamis, a meccs végén igaz lesz


    // Szoftveres kapcsoló a Garden utáni döntéshez (Meccs előtt állítható)
    private final boolean crossOverAfterGarden = false;

    // Globális Pedro Pathing koordináták
    private Pose startPose;
    private Pose gardenPose;
    private Pose scorePose1;
    private Pose scorePose2;
    private Pose flowerPose;
    private Pose parkPose;

    // --- ÚTVONALTERVEK (Tiszta Pedro v3 Path) ---
    private Path toInitialShoot() { return line(startPose, scorePose1).linear(startPose, scorePose1); }
    private Path toGarden() { return line(scorePose1, gardenPose).linear(scorePose1, gardenPose); }
    private Path backToLocalShoot() { return line(gardenPose, scorePose1).linear(gardenPose, scorePose1); }
    private Path toLocalFlower() { return line(scorePose1, flowerPose).linear(scorePose1, flowerPose); }
    private Path flowerBackToShoot() { return line(flowerPose, scorePose1).linear(flowerPose, scorePose1); }
    private Path parkFromLocal() { return line(scorePose1, parkPose).linear(scorePose1, parkPose); }

    private Path crossOverPath() {
        return path(
                line(gardenPose, poseFactory.of(90, 48, 90)),
                line(poseFactory.of(90, 48, 90), scorePose2)
        );
    }
    private Path parkFromOpponent() { return line(scorePose2, parkPose).linear(scorePose2, parkPose); }

    // =====================================================================
    // DINAMIKUS IVY FORGATÓKÖNYVEK (DECISION TREE)
    // =====================================================================
    // AZ AUTONÓM LOGIKAI DÖNTÉSI FA (JAVÍTVA ÉS TÍPUS-BIZTOSÍTVA)
    private Command autoRoutine() {
        boolean isBlue = (currentAlliance == Alliance.BLUE);

        // A befejezést jelző parancsot tiszta instant-ként kezeljük
        Command signalFinished =  Command.build().setStart(() -> autoSequenceFinished = true);

        // Létrehozunk egy tiszta, explicit módon típusos Java listát a parancsoknak
        java.util.List<Command> routineSteps = new java.util.ArrayList<>();

        // ---------------------------------------------------------------------
        // 4/a FORGATÓKÖNYV: SAJÁT OLDAL (LÁTHATÓ AP_TAG INDULÁS)
        // ---------------------------------------------------------------------
        if (currentStrategy == StartStrategy.SHOOTING_SIDE_LOCAL) {

            // 1. Azonnali gépfegyver lövés a 4 pre-load-ból
            routineSteps.add(robot.setTargetToAudienceCell(isBlue));
            routineSteps.add(robot.enableTuretTracking());
            routineSteps.add(follow(follower, toInitialShoot()));
            routineSteps.add(waitMs(200));
            routineSteps.add(robot.openGate());
            routineSteps.add(robot.startIntake());
            routineSteps.add(waitMs(1200));
            routineSteps.add(robot.closeGate());
            routineSteps.add(robot.stopIntake());
            routineSteps.add(robot.disableTuretTracking());

            // 2. Elmegyünk a Gardenhez, felszedjük a labdákat
            routineSteps.add(parallel(follow(follower, toGarden()), robot.startIntake()));
            routineSteps.add(waitMs(800));
            routineSteps.add(robot.stopIntake());

            // 3. Szövetséges alapú döntési ág felépítése tiszta listákkal
            if (crossOverAfterGarden) {
                // ÁTMENET A TÚLOLDALRA:
                java.util.List<Command> crossOverSteps = new java.util.ArrayList<>();
                crossOverSteps.add(checkTimeAndPark(5.0, parkFromLocal()));
                crossOverSteps.add(robot.setTargetToBackstageCell(isBlue));
                crossOverSteps.add(robot.enableTuretTracking());
                crossOverSteps.add(follow(follower, crossOverPath()));
                crossOverSteps.add(waitForValidTagAndShoot(5.0, parkFromOpponent()));
                crossOverSteps.add(follow(follower, parkFromOpponent()));
                crossOverSteps.add(signalFinished);

                routineSteps.add(Groups.sequential(crossOverSteps.toArray(new Command[0])));
            } else {
                // HELYBEN MARADÁS (Vissza a kezdőpontra):
                java.util.List<Command> localSteps = new java.util.ArrayList<>();
                localSteps.add(checkTimeAndPark(5.0, parkFromLocal()));
                localSteps.add(robot.setTargetToAudienceCell(isBlue));
                localSteps.add(robot.enableTuretTracking());
                localSteps.add(follow(follower, backToLocalShoot()));
                localSteps.add(waitForValidTagAndShoot(5.0, parkFromLocal()));

                // 4. Elmegyünk a falon lévő Flower-höz labdákért
                localSteps.add(checkTimeAndPark(5.0, parkFromLocal()));
                localSteps.add(robot.disableTuretTracking());
                localSteps.add(parallel(follow(follower, toLocalFlower()), robot.startIntake()));
                localSteps.add(waitMs(800));
                localSteps.add(robot.stopIntake());

                // Visszatérés a biztos lőhelyre és utolsó sorozat kilövése
                localSteps.add(checkTimeAndPark(5.0, parkFromLocal()));
                localSteps.add(robot.setTargetToAudienceCell(isBlue));
                localSteps.add(robot.enableTuretTracking());
                localSteps.add(follow(follower, flowerBackToShoot()));
                localSteps.add(waitMs(200));
                localSteps.add(robot.openGate());
                localSteps.add(robot.startIntake());
                localSteps.add(waitMs(1200));
                localSteps.add(robot.closeGate());
                localSteps.add(robot.stopIntake());
                localSteps.add(robot.disableTuretTracking());
                localSteps.add(follow(follower, parkFromLocal()));
                localSteps.add(signalFinished);

                routineSteps.add(Groups.sequential(localSteps.toArray(new Command[0])));
            }

            // Visszaadjuk a becsomagolt tiszta tömböt
            return Groups.sequential(routineSteps.toArray(new Command[0]));
        }

        // ---------------------------------------------------------------------
        // 4/b FORGATÓKÖNYV: ELLENFÉL OLDAL (VAK INDULÁS)
        // ---------------------------------------------------------------------
        else {
            routineSteps.add(follow(follower, toInitialShoot()));
            routineSteps.add(robot.setTargetToAudienceCell(isBlue));
            routineSteps.add(robot.enableTuretTracking());
            routineSteps.add(waitForValidTagAndShoot(5.0, parkFromLocal()));

            routineSteps.add(checkTimeAndPark(5.0, parkFromLocal()));
            routineSteps.add(robot.openGate());
            routineSteps.add(parallel(
                    follow(follower, toLocalFlower()),
                    robot.startIntake()
            ));
            routineSteps.add(waitMs(3500));

            routineSteps.add(robot.closeGate());
            routineSteps.add(robot.stopIntake());
            routineSteps.add(robot.disableTuretTracking());
            routineSteps.add(follow(follower, parkFromLocal()));
            routineSteps.add(signalFinished);

            return Groups.sequential(routineSteps.toArray(new Command[0]));
        }
    }


    // --- OKOS IDŐFIGYELŐ ÉS MENEKÜLŐ PARANCS ---
    private Command checkTimeAndPark(double safeMarginSeconds, Path parkPath) {
        return Command.build().setStart(() -> {
            if (autonomousTimer.seconds() > (25.0 - safeMarginSeconds)) {
                telemetry.log().add("IDŐKORLÁT! Azonnali menekülés a parkolóba!");
                Scheduler.reset(); // Törli a hátralévő lövési parancsokat
                schedule(sequential(robot.disableTuretTracking(), robot.stopIntake(), robot.closeGate(), follow(follower, parkPath)));
            }
        });
    }

    // --- JEL- ÉS CELLAMAGASSÁG-KÖVETŐ BLOKKOLÓ PARANCS ---
    private Command waitForValidTagAndShoot(double safeMarginSeconds, Path parkPath) {

        final ElapsedTime commandTimer = new ElapsedTime();

        return Command.build()
                .setStart(() -> {
                    robot.outtake.setPower(0.85);
                    commandTimer.reset(); // Elindítjuk a parancs belső óráját
                })
                .setExecute(() -> {
// Ha közeledik a 30s vége, megszakítjuk a várakozást és menekülünk parkolni
                    if (autonomousTimer.seconds() > (25.0 - safeMarginSeconds)) {
                        return;
                    }
// Megnézzük, hogy a lőtorony látja-e a célt (ha az updateTuret kiszámolta a pozíciót)
                    if (aprilTag.getDetections().size() > 0) {
// Ha látunk érvényes taget, kinyitjuk a zárat és tüzelünk
                        robot.openGate();
                        robot.startIntake();
                    }
                })
                .setDone(() -> commandTimer.seconds() >= 1.5); // Maximum 1.5 másodpercig engedélyezi a burst fire-t ebben a zónában
    }
    @Override
    public void init() {
        Scheduler.reset();
        robot.init(hardwareMap);
        initVision();
        follower = Constants.create(hardwareMap);
// Szigorú 18x18-as mérethez igazított alapértelmezett koordináták (Kék szövetség alap)
        startPose = poseFactory.of(9.0, 24.0, 0);
        scorePose1 = poseFactory.of(48.0, 112.0, 90);
        scorePose2 = poseFactory.of(48.0, 40.0, 90);
        gardenPose = poseFactory.of(24.0, 11.0, 0);
        flowerPose = poseFactory.of(24.0, 133.0, 90);
        parkPose   = poseFactory.of(11.0, 24.0, 0);
        follower.setPose(startPose);
        follower.update();
    }
    @Override
    public void init_loop() {
        ArrayList<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection instanceof AprilTagSingleDetection) {
                AprilTagSingleDetection singleDet = (AprilTagSingleDetection) detection;
                if (singleDet.metadata != null && singleDet.robotPose != null) {
                    int id = singleDet.id;
                    if (id >= 30 && id <= 33) {
                        currentAlliance = Alliance.RED;
                        currentStrategy = StartStrategy.SHOOTING_SIDE_LOCAL;
                        validator(singleDet.robotPose);
                    } else if (id >= 34 && id <= 37) {
                        currentAlliance = Alliance.RED;
                        currentStrategy = StartStrategy.GARDEN_SIDE_CROSSOVER;
                        validator(singleDet.robotPose);
                    } else if (id >= 38 && id <= 41) {
                        currentAlliance = Alliance.BLUE;
                        currentStrategy = StartStrategy.GARDEN_SIDE_CROSSOVER;
                        validator(singleDet.robotPose);
                    } else if (id >= 42 && id <= 45) {
                        currentAlliance = Alliance.BLUE;
                        currentStrategy = StartStrategy.SHOOTING_SIDE_LOCAL;
                        validator(singleDet.robotPose);
                    }
                }
            }
            if (positionCalibrated) break;
        }
        follower.update();
        telemetry.addData("Szövetség", currentAlliance);
        telemetry.addData("Stratégia", currentStrategy);
        telemetry.addData("Kalibráció", positionCalibrated ? "SIKERES!" : "KERESÉS...");
        telemetry.update();
    }
    private void validator(Pose3D robotPose) {
        double calcX = robotPose.getPosition().x;
        double calcY = robotPose.getPosition().y;
        double calcHeadingRad = robotPose.getOrientation().getYaw(AngleUnit.RADIANS);
        if (calcX >= 0 && calcX <= 144 && calcY >= 0 && calcY <= 144) {
// A ti egyedi 2/a pontotok szerint: a robot pontosan a cellával szemben, a tag középvonalán áll az Y tengelyen
            startPose = poseFactory.of(9.0, calcY, Math.toDegrees(calcHeadingRad));
            follower.setPose(startPose);
            boolean isBlue = (currentAlliance == Alliance.BLUE);
            if (isBlue) {
                gardenPose = poseFactory.of(24.0, 11.0, 0);
                scorePose1 = poseFactory.of(48.0, 112.0, 90);
                scorePose2 = poseFactory.of(48.0, 40.0, 90);
                flowerPose = poseFactory.of(24.0, 133.0, 90);
                parkPose   = poseFactory.of(11.0, 24.0, 0);
            } else {
                gardenPose = poseFactory.of(24.0, 133.0, 180);
                scorePose1 = poseFactory.of(48.0, 32.0, -90);
                scorePose2 = poseFactory.of(48.0, 104.0, -90);
                flowerPose = poseFactory.of(24.0, 11.0, -90);
                parkPose   = poseFactory.of(11.0, 120.0, 180);
            }
            positionCalibrated = true;
        }
    }
    @Override
    public void start() {
        autonomousTimer.reset(); // Elindítjuk a 30 másodperces órát
        if (visionPortal != null) {
            visionPortal.stopStreaming();
        }
        follower.setPose(startPose);
        follower.update();
        schedule(autoRoutine());
    }
    @Override
    public void loop() {
        follower.update();    // Pedro Pathing lokalizáció élő frissítése
        Scheduler.execute();   // Ivy parancsok végrehajtása

        // JAVÍTVA: getQueue().isEmpty() helyet a hivatalos !Scheduler.isRunning() hívást használjuk!
        // Ha az autonóm parancsok véget értek, VAGY ha már csak 0.5 másodperc van hátra a meccsből:
        if (autoSequenceFinished || autonomousTimer.seconds() > 28.0) {
            robot.turet.setTargetPosition(0); // Visszaforgatja a tornyot pontosan a kezdő (nulla) pontra
            robot.turet.setPower(0.5);        // Biztonságos sebességgel tartja a pozíciót
        } else {
            // Ha még fut az autonóm, a torony folyamatosan követi az élő Pedro koordinátákat
            robot.updateTuret(follower.pose().x(), follower.pose().y(), follower.pose().heading());
        }

        telemetry.addData("Idő", autonomousTimer.seconds());
        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.update();
    }


    private void initVision() {
        try {
            aprilTag = new AprilTagProcessor.Builder()
                    .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                    .build();

            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();
        } catch (Exception e) {
            aprilTag = null;
            visionPortal = null;
        }
    }
}