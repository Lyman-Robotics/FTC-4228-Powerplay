package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Omnirive")
public class Omnidrive extends LinearOpMode {

  // Declare OpMode members for each of the 4 motors.
  private ElapsedTime runtime = new ElapsedTime();
  private DcMotor leftFrontDrive = null;
  private DcMotor leftBackDrive = null;
  private DcMotor rightFrontDrive = null;
  private DcMotor rightBackDrive = null;

  @Override
  public void runOpMode() {
    // Initialize the hardware variables. Note that the strings used here as
    // parameters
    RobotClass robot = new RobotClass();
    robot.init();

    // Wait for the game to start (driver presses PLAY)
    telemetry.addData("Status", "Initialized");
    telemetry.update();
    waitForStart();
    runtime.reset();

    // Initialize drive variables
    float vertical;
    float horizontal;
    float pivot;
    double speedScalar;

    // Runs until the end of the match
    while (opModeIsActive()) {
      double max;

      vertical = gamepad1.left_stick_y;
      horizontal = -gamepad1.left_stick_x;
      pivot = -gamepad1.right_stick_x;

      double FRPower = ((-pivot + (vertical - horizontal)) * speedScalar);
      double BRPower = ((-pivot + vertical + horizontal) * speedScalar);
      double FLPower = ((pivot + vertical + horizontal) * speedScalar);
      double BLPower = ((pivot + (vertical - horizontal)) * speedScalar);

      // Normalize the values so no wheel power exceeds 100%
      // This ensures that the robot maintains the desired motion.
      max = Math.max(Math.abs(FLPower), Math.abs(FRPower));
      max = Math.max(max, Math.abs(BLPower));
      max = Math.max(max, Math.abs(BRPower));

      if (max > 1.0) {
        FLPower /= max;
        FRPower /= max;
        BLPower /= max;
        BRPower /= max;
      }

      // This is test code:

      // Uncomment the following code to test your motor directions.
      // Each button should make the corresponding motor run FORWARD.
      // 1) First get all the motors to take to correct positions on the robot
      // by adjusting your Robot Configuration if necessary.
      // 2) Then make sure they run in the correct direction by modifying the
      // the setDirection() calls above.
      // Once the correct motors move in the correct direction re-comment this code.

      /*
       * FLPower = gamepad1.x ? 1.0 : 0.0; // X gamepad
       * BLPower = gamepad1.a ? 1.0 : 0.0; // A gamepad
       * FRPower = gamepad1.y ? 1.0 : 0.0; // Y gamepad
       * BRPower = gamepad1.b ? 1.0 : 0.0; // B gamepad
       */

      // Send calculated power to wheels
      robot.setDrivePower(FLPower, FRPower, BLPower, BRPower);

      // Show the elapsed game time and wheel power.
      telemetry.addData("Status", "Run Time: " + runtime.toString());
      telemetry.addData("Front left/Right", "%4.2f, %4.2f", FLPower, FRPower);
      telemetry.addData("Back  left/Right", "%4.2f, %4.2f", BLPower, BRPower);
      telemetry.addData("Elapsed Time: ", robot.timeElapsed);
      telemetry.update();
    }
  }
}
