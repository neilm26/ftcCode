/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="SwatRed_Tele_R5", group="Linear Opmode")
//@Disabled
public class SwatRed_Tele_R5 extends LinearOpMode {

    // Declare OpMode members.

    private TeamRobot_R5 robot = new TeamRobot_R5();

    @Override
    public void runOpMode() {

        robot.init(this.hardwareMap);

        // Wait for the game to start (driver presses PLAY)
        robot.fl_Drive.setPower(0);
        robot.fr_Drive.setPower(0);
        robot.bl_Drive.setPower(0);
        robot.br_Drive.setPower(0);

        waitForStart();
        //runtime.reset();

        double drive = 0;
        double turn  = 0;
        boolean arm_state = false;
        boolean clamp_state = false;
        boolean auto_state = false;
        boolean foundation_state = false;
        boolean CAPPING          = true;
        boolean clamp_complete = true;
        boolean arm_complete = true;
        robot.l_gripper.setPosition(0.50);
        robot.r_gripper.setPosition(0.3);
        sleep(450);
        robot.l_arm.setPosition(0.5);
        robot.l_gripper.setPosition(0.00);
        robot.r_arm.setPosition(0.66);
        robot.r_gripper.setPosition(0.70);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Setup a variable for each drive wheel to save power level for telemetry
            double drive_power;
            double turn_power;
            double strafe_power;
            double strafe_left = 0;
            double strafe_right = 0;


            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            if (Math.abs(gamepad1.left_stick_y) > 0.1) {
                drive = gamepad1.left_stick_y * 0.6; //Normal Drive Motion
                if (gamepad1.b) {
                    drive *= 2;                     // Turbo Drive Motion
                }
                drive_power = Range.clip(drive, -1.0, 1.0);
                telemetry.addData("drive:", drive_power);
                telemetry.update();
                robot.drive(TeamRobot_R5.DRIVE_OPTION.STRAIGHT, drive_power);
            }

            if (Math.abs(gamepad1.right_stick_x) > 0.1) {
                turn = gamepad1.right_stick_x * 0.6;        //Normal Rotate Motion
                if (gamepad1.b) {
                    turn *= 2;                     // Turbo Rotate Motion
                }
                turn_power = Range.clip(turn, -1.0, 1.0);
                robot.drive(TeamRobot_R5.DRIVE_OPTION.TURN, -turn_power);
            }

            //Strafe operation
            if (Math.abs(gamepad1.right_trigger) > 0.1) {
                strafe_right = gamepad1.right_trigger;
                strafe_power = Range.clip(strafe_right, -1.0, 1.0);
                robot.drive(TeamRobot_R5.DRIVE_OPTION.RIGHTSTRAFE, strafe_power);
            } else if (Math.abs(gamepad1.left_trigger) > 0.1) {
                strafe_left = gamepad1.left_trigger;
                strafe_power = Range.clip(strafe_left, -1.0, 1.0);
                robot.drive(TeamRobot_R5.DRIVE_OPTION.LEFTSTRAFE, strafe_power);
            }

            if (Math.abs(gamepad1.right_stick_x) < 0.1 && Math.abs(gamepad1.left_stick_y) < 0.1 && gamepad1.left_trigger < 0.1 && gamepad1.right_trigger < 0.1) {
                robot.stop();
            }

            if(gamepad1.a) {
                robot.intake(TeamRobot_R5.INTAKE_OPTION.INTAKE,-0.85);
            } else if (gamepad1.y) {
                robot.intake(TeamRobot_R5.INTAKE_OPTION.OUTTAKE,0.85);
            }

            if(!gamepad1.x&&!gamepad1.b) {
                robot.intake(TeamRobot_R5.INTAKE_OPTION.INTAKE,0);
            }
            if (gamepad2.back) {
                robot.capping_servo.setPosition(0.1);
            }
            if (gamepad2.start) {
                robot.capping_servo.setPosition(0.5);
            }
            /**if (gamepad2.y){
                runtime.reset();
                while (runtime.milliseconds()<1600) {
                    robot.linear_lift.setPower(-0.7);
                }
                robot.linear_lift.setPower(0);
                robot.linear_arm.setPosition(0);
                sleep(200);
            }*/
            if (gamepad2.dpad_up) {
                robot.tape_measurer.setPower(1);
            }
            if (gamepad2.dpad_down) {
                robot.tape_measurer.setPower(-1);
            }
            if (!gamepad2.dpad_up&&!gamepad2.dpad_down) {
                robot.tape_measurer.setPower(0);
            }
            if (gamepad2.right_trigger>0.1) {
                robot.linear_lift.setPower(-gamepad2.right_trigger);
            }
            if (gamepad2.left_trigger>0.1){
                robot.linear_lift.setPower(gamepad2.left_trigger);
            }
            if(gamepad2.left_trigger <0.1 && gamepad2.right_trigger <0.1){
                robot.linear_lift.setPower(0);
            }

            //linear_arm operations

            if (gamepad2.left_bumper){
                if (arm_state == false&&gamepad2.left_bumper&&arm_complete) {
                    arm_complete=false;
                    robot.linear_arm.setPosition(1);
                    arm_complete=true;
                    arm_state = true;
                    sleep(200);

                } else if (arm_state == true&&gamepad2.left_bumper&&arm_complete) {
                    arm_complete=false;
                    robot.linear_arm.setPosition(0);
                    arm_complete=true;
                    arm_state = false;
                    sleep(200);
                }
            }
            // clamp operations
            if (gamepad2.b){
                if (clamp_state == false&&gamepad2.b&&clamp_complete) {
                    clamp_complete=false;
                    robot.clamp.setPosition(0.8);
                    clamp_state = true;
                    clamp_complete=true;
                    sleep(200);

                } else if (clamp_state == true&&gamepad2.b&&clamp_complete) {
                    clamp_complete=false;
                    robot.clamp.setPosition(0);
                    clamp_state = false;
                    clamp_complete=true;
                    sleep(200
                    );
                }
            }
            // semi auto button operation
            if (gamepad2.a){
                if (foundation_state == false) {
                    foundation_state = true;
                    robot.foundation_clamp.setPosition(1.0);
                    sleep(200);

                } else if (foundation_state == true) {
                    foundation_state = false;
                    robot.foundation_clamp.setPosition(0);
                    sleep(200);
                }
            }

        }
        //Linear lift operations

    }

        // Gripper operations
    }

/**
 * Method to perfmorm a relative move, based on encoder counts.
 * Encoders are not reset as the move is based on the current position.
 * Move will stop if any of three conditions occur:
 * 1) Move gets to the desired position
 * 2) Move runs out of time
 * 3) Driver stops the opmode running.
 */