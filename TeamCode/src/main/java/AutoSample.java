import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class AutoSample extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor pivotMotor = null;
    private DcMotor lslidesMotor = null;
    private DcMotor rslidesMotor = null;

    private Servo rpivot = null;
    private Servo lpivot = null;
    private Servo elbowM = null;
    private Servo claw = null;
    private Servo wrist = null;
    private Servo larmM = null;
    private Servo rarmM = null;
    @Override
    public void runOpMode(){
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "fl");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "bl");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "fr");
        rightBackDrive = hardwareMap.get(DcMotor.class, "br");
        rpivot = hardwareMap.get(Servo.class, "rpivot");
        lpivot = hardwareMap.get(Servo.class, "lpivot");
        lslidesMotor  = hardwareMap.get(DcMotor.class, "lslides");
        rslidesMotor  = hardwareMap.get(DcMotor.class, "rslides");
        elbowM = hardwareMap.get(Servo.class, "elbow");
        wrist = hardwareMap.get(Servo.class, "wrist");
        claw = hardwareMap.get(Servo.class, "claw");
        larmM = hardwareMap.get(Servo.class, "larm");
        rarmM = hardwareMap.get(Servo.class, "rarm");
        //initialize motor directions
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rslidesMotor.setDirection(DcMotor.Direction.FORWARD);
        lslidesMotor.setDirection(DcMotor.Direction.FORWARD);
        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();
        double speed = 0.5;
        double phase2Offset = 0;
        while(opModeIsActive()) {
            //strafe left
//            leftFrontDrive.setPower(-speed);
//            rightBackDrive.setPower(-speed);
//            leftBackDrive.setPower(speed);
//            rightFrontDrive.setPower(speed);
            elbowM.setPosition(0);
            claw.setPosition(50);
            wrist.setPosition(0);
            if (runtime.seconds() < 0.9) {
                leftFrontDrive.setPower(speed);
                rightBackDrive.setPower(speed);
                leftBackDrive.setPower(speed);
                rightFrontDrive.setPower(speed);
            }
            telemetry.addData("Elapsed Time", runtime.seconds());
            telemetry.update();
            if (runtime.seconds() > 0.9 && runtime.seconds() < 2) {
                leftFrontDrive.setPower(0);
                rightBackDrive.setPower(0);
                leftBackDrive.setPower(0);
                rightFrontDrive.setPower(0);
                lslidesMotor.setPower(0.6);
                rslidesMotor.setPower(0.6);
            }
        }
    }
}
