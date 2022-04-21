#define MAX_LEN 16
#define STEPPER_ADDR 8
#define UI_ADDR 7
#define FAIL_HEADER 0xFE
#define STEP_MOVE 0xE0
#define STEP_STOP 0xEF
#define STEP_ORBIT 0xEB
#define STEP_CLIMB 0xEC
#define STEP_DESC 0xED
#define UI_CMD_MOVE_HEADER 0xC0
#define STATUS_HEADER 0x00

struct stepPos{
  int x;
  int y;
  int z;
};
