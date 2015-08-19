#include <Servo.h>
#define BAUD_RATE 9600
#define MIN_BYTE_AVAILABLE_TO_READ 5
// ServoNum(1Byte) + delemiter(1Byte) + int for pos(4Bytes)

#define ROT_SERVO 2
#define CLAW_SERVO 3

#define SERVO_EXT 90
#define SERVO_NUT 45
#define SERVO_RET 0

#define OPEN  0
#define STOP  1
#define CLOSE 2

Servo rot, claw, dest;
int servo, state, valid;
void 
setup() 
{
    rot.attach(ROT_SERVO,SERVO_RET,SERVO_EXT);
	rot.write(SERVO_NUT);
    claw.attach(CLAW_SERVO,SERVO_RET,SERVO_EXT);
	claw.write(SERVO_NUT);
    Serial.begin(BAUD_RATE);

    servo = 0;
    state = STOP;
    valid = 0;
}

void
loop()
{
    if(Serial.available() >= MIN_BYTE_AVAILABLE_TO_READ){
        servo = Serial.parseInt();
        state = Serial.parseInt();
        valid = 0;
        
        if (servo == ROT_SERVO) {
            dest = rot;
            valid = 1;
        } else if (servo == CLAW_SERVO) {
            dest = claw;
            valid = 1;
        }

        if (valid) {
            int pos = SERVO_NUT;
            if (state == OPEN) {
                pos = SERVO_RET;
            }else if (state == CLOSE) {
                pos = SERVO_EXT;   
            }
            dest.write(pos);
        }
    }
}
