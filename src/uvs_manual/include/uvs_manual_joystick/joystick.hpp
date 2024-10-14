#ifndef JOYSTICK_HPP
#define JOYSTICK_HPP

#define JOYSTICK_TYPE_BUTTON  (0X01)
#define JOYSTICK_TYPE_AXIS    (0X02)

#define JOYSTICK_BUTTON_A     (0X00)
#define JOYSTICK_BUTTON_B     (0X01)
#define JOYSTICK_BUTTON_X     (0X02)
#define JOYSTICK_BUTTON_Y     (0X03)
#define JOYSTICK_BUTTON_L1    (0X04)
#define JOYSTICK_BUTTON_R1    (0X05)
#define JOYSTICK_BUTTON_SELECT (0X06)
#define JOYSTICK_BUTTON_START (0X07)
#define JOYSTICK_BUTTON_MODE  (0X08)
#define JOYSTICK_BUTTON_LTHUMB (0X09)
#define JOYSTICK_BUTTON_RTHUMB (0X0A)

#define JOYSTICK_BUTTON_ON   (0X01)
#define JOYSTICK_BUTTON_OFF   (0X00)

#define JOYSTICK_AXIS_L_LR    (0X00)
#define JOYSTICK_AXIS_L_UD    (0X01)
#define JOYSTICK_AXIS_L2      (0X02)
#define JOYSTICK_AXIS_R_LR    (0X03)
#define JOYSTICK_AXIS_R_UD    (0X04)
#define JOYSTICK_AXIS_R2      (0X05)
#define JOYSTICK_AXIS_CROSS_LR (0X06)
#define JOYSTICK_AXIS_CROSS_UD (0X07)

#define JOYSTICK_AXIS_UPMAX (-32767)
#define JOYSTICK_AXIS_DOWNMAX (32767)
#define JOYSTICK_AXIS_LEFTMAX (-32767)
#define JOYSTICK_AXIS_RIGHTMAX (32767)

#define JOYSTICK_AXIS_MIN (-32767)
#define JOYSTICK_AXIS_MAX (32767)
#define JOYSTICK_AXIS_MID (0)

typedef struct JoystickMap
{
  int time;
  // button
  int button_a;
  int button_b;
  int button_x;
  int button_y;
  int button_l1;
  int button_r1;
  int button_select;
  int button_start;
  int button_mode;
  int button_lthumb;
  int button_rthumb;
  // axis
  int axis_l_lr;
  int axis_l_ud;
  int axis_l2;
  int axis_r_lr;
  int axis_r_ud;
  int axis_r2;
  int axis_cross_lr;
  int axis_cross_ud;

  JoystickMap()
  {
    time = 0;
    button_a = 0;
    button_b = 0;
    button_x = 0;
    button_y = 0;
    button_l1 = 0;
    button_r1 = 0;
    button_select = 0;
    button_start = 0;
    button_mode = 0;
    button_lthumb = 0;
    button_rthumb = 0;
    axis_l_lr = 0;
    axis_l_ud = 0;
    axis_l2 = JOYSTICK_AXIS_MIN;
    axis_r_lr = 0;
    axis_r_ud = 0;
    axis_r2 = JOYSTICK_AXIS_MIN;
    axis_cross_lr = 0;
    axis_cross_ud = 0;
  }
}JoystickMap;

class Joystick
{
public:
  Joystick(int id = 0);
  ~Joystick();
  bool isFound();
  bool sample(JoystickMap *joystickMap);
private:
  int fd;
  int button[11];
  int axis[8];
  int time;
};


#endif // UVS_MANUAL_JOYSTICK_HPP