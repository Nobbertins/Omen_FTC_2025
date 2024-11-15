package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

public class ToggleServo{
    private Servo servo;
    private int angle1;
    private int angle2;
    private boolean pos;
    public ToggleServo(Servo s, int angle1, int angle2, Servo.Direction direction){
        this.servo = s;
        this.angle1 = angle1;
        this.angle2 = angle2;
        this.pos = false;
        this.servo.setDirection(direction);
    }
    public void toggle(){
        if(this.pos) {
            this.servo.setPosition(angle1 / 355.0);
        }
        else{
            this.servo.setPosition(angle2 / 355.0);
        }
        this.pos = !this.pos;
    }
    public Servo getServo(){return this.servo;}
}