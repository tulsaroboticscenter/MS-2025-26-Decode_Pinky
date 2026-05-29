
    package org.firstinspires.ftc.teamcode.OpModes; // make sure this aligns with class location

    import static java.lang.Thread.sleep;

    import com.pedropathing.follower.Follower;
    import com.pedropathing.geometry.BezierLine;
    import com.pedropathing.geometry.Pose;
    import com.pedropathing.paths.Path;
    import com.pedropathing.paths.PathChain;
    import com.pedropathing.util.Timer;
    import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
    import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
    import com.qualcomm.robotcore.eventloop.opmode.OpMode;
    import com.qualcomm.robotcore.hardware.DcMotor;
    import com.qualcomm.robotcore.hardware.HardwareMap;
    import com.qualcomm.robotcore.util.ElapsedTime;

    import org.firstinspires.ftc.teamcode.Hardware.HWProfile2;
    import org.firstinspires.ftc.teamcode.Hardware.MSParams;
    import org.firstinspires.ftc.teamcode.Libs.CTSMechOps;
    import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

    //@Autonomous(name = "PedroBlue", group = "Robot")


    public class PedroLinearBlue extends OpMode {

        public DcMotor motorShooter = null;
        public DcMotor motorShooterTop = null;
        public DcMotor motorIntake;
        public DcMotor motorFeeder;

        HardwareMap hwMap;


        private HWProfile2 robot = new HWProfile2();
        public final static MSParams params = new MSParams();
        private OpMode myOpMode = this;
        public HardwareMap hwmap;
        private CTSMechOps mechOps;
        private Follower follower;
        private Timer pathTimer, actionTimer, opmodeTimer;

        private int pathState;

        private final Pose startPose = new Pose(28.5, 128, Math.toRadians(180)); // Start Pose of our robot.
        private final Pose scorePose = new Pose(60, 85, Math.toRadians(135)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
        private final Pose pickup1PoseEnd = new Pose(25, 82.5, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
        private final Pose pickup1PoseBegin= new Pose(25, 82.5, Math.toRadians(180));
        private final Pose pickup2Pose = new Pose(25, 59, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
        private final Pose pickup3Pose = new Pose(25, 35, Math.toRadians(180)); // Lowest (Third Set) of Artifacts from the Spike Mark.
        private final Pose endPose = new Pose(50, 53, Math.toRadians(135)); //  End Position of the Robot

        private ElapsedTime ctsTimer = new ElapsedTime();
        private Path scorePreload;
        private PathChain scorePickup1, grabPickup1Begin,grabPickup1End, grabPickup2, scorePickup2, grabPickup3, scorePickup3,endingPose;

        public void buildPaths() {
            /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
            scorePreload = new Path(new BezierLine(startPose, scorePose));
            scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */

            /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
            grabPickup1Begin = follower.pathBuilder()
                    .addPath(new BezierLine(scorePose, pickup1PoseBegin))
                    .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1PoseBegin.getHeading())

                    .build();

            grabPickup1End = follower.pathBuilder()
                    .addPath(new BezierLine(scorePose, pickup1PoseBegin))
                    .setLinearHeadingInterpolation(pickup1PoseBegin.getHeading(), pickup1PoseEnd.getHeading())

                    .build();

            /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
            scorePickup1 = follower.pathBuilder()
                    .addPath(new BezierLine(pickup1PoseEnd, scorePose))
                    .setLinearHeadingInterpolation(pickup1PoseEnd.getHeading(), scorePose.getHeading())
                    .build();

            /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
            grabPickup2 = follower.pathBuilder()
                    .addPath(new BezierLine(scorePose, pickup2Pose))
                    .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                    .build();

            /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
            scorePickup2 = follower.pathBuilder()
                    .addPath(new BezierLine(pickup2Pose, scorePose))
                    .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                    .build();

            /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
            grabPickup3 = follower.pathBuilder()
                    .addPath(new BezierLine(scorePose, pickup3Pose))
                    .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                    .build();

            /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
            scorePickup3 = follower.pathBuilder()
                    .addPath(new BezierLine(pickup3Pose, scorePose))
                    .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                    .build();
            /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
            endingPose = follower.pathBuilder()
                    .addPath(new BezierLine(scorePose, endPose))
                    .setLinearHeadingInterpolation(scorePose.getHeading(), endPose.getHeading())
                    .build();
        }

        public void autonomousPathUpdate() {
            switch (pathState) {
                case 0:
//                    mechOps.shooterControl(3800);
                 //   robot.motorShooter.setPower(.85);;
                 //   robot.motorShooterTop.setPower(.85);

                    ctsTimer.reset();
                    while(ctsTimer.time() < 1){

                    }

                    follower.followPath(scorePreload);

                 //   robot.motorIntake.setPower(1);
                 //   robot.motorFeeder.setPower(1);

                    follower.holdPoint(scorePose);

                    ctsTimer.reset();
                    while(ctsTimer.time() < 3){

                    }
                    setPathState(1);
                    break;
                case 1:

            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */

                    /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                    if (!follower.isBusy()) {
                        /* Score Preload */

                        /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                        follower.followPath(grabPickup1Begin, true);
                        //Run the motor
                        follower.followPath(grabPickup1End, true);
                        setPathState(2);
                    }
                    break;
                case 2:
                    /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                    if (!follower.isBusy()) {
                        /* Grab Sample */

                        /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                        follower.followPath(scorePickup1, true);
                        setPathState(3);
                    }
                    break;
                case 3:
                    /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                    if (!follower.isBusy()) {
                        /* Score Sample */

                        /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                        follower.followPath(grabPickup2, true);
                        setPathState(4);
                    }
                    break;
                case 4:
                    /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                    if (!follower.isBusy()) {
                        /* Grab Sample */

                        /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                        follower.followPath(scorePickup2, true);
                        setPathState(5);
                    }
                    break;
                case 5:
                    /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                    if (!follower.isBusy()) {
                        /* Score Sample */

                        /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                        follower.followPath(grabPickup3, true);
                        setPathState(6);
                    }
                    break;
                case 6:
                    /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                    if (!follower.isBusy()) {
                        /* Grab Sample */

                        /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                        follower.followPath(scorePickup3, true);
                        setPathState(7);
                    }
                    break;
                case 7:
                    /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                    if (!follower.isBusy()) {
                        /* Grab Sample */

                        /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                        follower.followPath(endingPose, true);
                        setPathState(8);
                    }
                    break;
                case 8:
                    /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                    if (!follower.isBusy()) {
                        /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                        setPathState(-1);
                    }
                    break;
            }
        }

        /**
         * These change the states of the paths and actions. It will also reset the timers of the individual switches
         **/
        public void setPathState(int pState) {
            pathState = pState;
            pathTimer.resetTimer();
        }

        /**
         * This is the main loop of the OpMode, it will run repeatedly after clicking "Play".
         **/
        @Override
        public void loop() {

            // These loop the movements of the robot, these must be called continuously in order to work
            follower.update();
            autonomousPathUpdate();

            // Feedback to Driver Hub for debugging
            telemetry.addData("path state", pathState);
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());
            telemetry.update();
        }



        /**
         * This method is called once at the init of the OpMode.
         **/
        @Override
        public void init() {





            motorShooter  = hardwareMap.get(DcMotor.class, "motorShooter");
            motorShooter.setDirection(DcMotor.Direction.REVERSE);
            motorShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorShooter.setPower(0);

            motorShooterTop = hardwareMap.get(DcMotor.class, "motorShooterTop");
            motorShooterTop.setDirection(DcMotor.Direction.FORWARD);
            motorShooterTop.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorShooterTop.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorShooterTop.setPower(0);


            motorIntake = hardwareMap.get(DcMotor.class, "motorIntake");
            motorIntake.setDirection(DcMotor.Direction.FORWARD);
            motorIntake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorIntake.setPower(0);
            motorIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motorIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            motorFeeder = hardwareMap.get(DcMotor.class, "motorFeeder");
            motorFeeder.setDirection(DcMotor.Direction.REVERSE);
            motorFeeder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorFeeder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motorFeeder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorFeeder.setPower(0);




            pathTimer = new Timer();
            opmodeTimer = new Timer();
            opmodeTimer.resetTimer();


            follower = Constants.createFollower(hardwareMap);
            buildPaths();
            follower.setStartingPose(startPose);

        }

        /**
         * This method is called continuously after Init while waiting for "play".
         **/
        @Override
        public void init_loop() {
        }

        /**
         * This method is called once at the start of the OpMode.
         * It runs all the setup actions, including building paths and starting the path system
         **/
        @Override
        public void start() {
            opmodeTimer.resetTimer();
            setPathState(0);
        }

        /**
         * We do not use this because everything should automatically disable
         **/
        @Override
        public void stop() {
        }

    }
