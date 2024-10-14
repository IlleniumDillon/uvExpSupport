#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <linux/input.h>
#include <linux/joystick.h>

#include <iostream>
#include <string>
#include <sstream>

#include "joystick.hpp"

Joystick::Joystick(int id)
{
  std::stringstream sstm;
  sstm << "/dev/input/js" << id;
  fd = open(sstm.str().c_str(), O_RDONLY | O_NONBLOCK);
}

Joystick::~Joystick()
{
  if (fd >= 0)
  {
    close(fd);
  }
}

bool Joystick::isFound()
{
  return fd >= 0;
}

bool Joystick::sample(JoystickMap *joystickMap)
{
  struct js_event js;
  int len = read(fd, &js, sizeof(struct js_event));
  if (len != sizeof(struct js_event))
  {
    return false;
  }
  joystickMap->time = js.time;
  if (js.type == JS_EVENT_BUTTON)
  {
    switch (js.number)
    {
    case JOYSTICK_BUTTON_A:
      joystickMap->button_a = js.value;
      break;
    case JOYSTICK_BUTTON_B:
      joystickMap->button_b = js.value;
      break;
    case JOYSTICK_BUTTON_X:
      joystickMap->button_x = js.value;
      break;
    case JOYSTICK_BUTTON_Y:
      joystickMap->button_y = js.value;
      break;
    case JOYSTICK_BUTTON_L1:
      joystickMap->button_l1 = js.value;
      break;
    case JOYSTICK_BUTTON_R1:
      joystickMap->button_r1 = js.value;
      break;
    case JOYSTICK_BUTTON_SELECT:
      joystickMap->button_select = js.value;
      break;
    case JOYSTICK_BUTTON_START:
      joystickMap->button_start = js.value;
      break;
    case JOYSTICK_BUTTON_MODE:
      joystickMap->button_mode = js.value;
      break;
    case JOYSTICK_BUTTON_LTHUMB:
      joystickMap->button_lthumb = js.value;
      break;
    case JOYSTICK_BUTTON_RTHUMB:
      joystickMap->button_rthumb = js.value;
      break;
    }
  }
  else if (js.type == JS_EVENT_AXIS)
  {
    switch (js.number)
    {
    case JOYSTICK_AXIS_L_LR:
      joystickMap->axis_l_lr = js.value;
      break;
    case JOYSTICK_AXIS_L_UD:
      joystickMap->axis_l_ud = js.value;
      break;
    case JOYSTICK_AXIS_L2:
      joystickMap->axis_l2 = js.value;
      break;
    case JOYSTICK_AXIS_R_LR:
      joystickMap->axis_r_lr = js.value;
      break;
    case JOYSTICK_AXIS_R_UD:
      joystickMap->axis_r_ud = js.value;
      break;
    case JOYSTICK_AXIS_R2:
      joystickMap->axis_r2 = js.value;
      break;
    case JOYSTICK_AXIS_CROSS_LR:
      joystickMap->axis_cross_lr = js.value;
      break;
    case JOYSTICK_AXIS_CROSS_UD:
      joystickMap->axis_cross_ud = js.value;
      break;
    }
  }
  return true;
}
