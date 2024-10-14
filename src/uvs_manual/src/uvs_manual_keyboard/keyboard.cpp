#include "keyboard.hpp"

#include <iostream>
#include <string>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <linux/input.h>

Keyboard::Keyboard(int id)
{
    if (id < 0)
    {
        FILE* fp = popen("cat /proc/bus/input/devices | grep -E \"sysrq kbd event[0-9]+ leds\" | grep -E -o \"[0-9]+\"", "r");
        if (fp == NULL)
        {
            fprintf(stderr, "Failed to run command\n");
            return;
        }
        char path[256];
        while (fgets(path, sizeof(path), fp) != NULL)
        { }
        pclose(fp);

        id = atoi(path);
    }

    std::string devPath = "/dev/input/event" + std::to_string(id);

    fd = open(devPath.c_str(), O_RDONLY | O_NONBLOCK);
    if (fd < 0) return;
}

Keyboard::~Keyboard()
{
    if (fd >= 0)
    {
        close(fd);
    }
}

bool Keyboard::isFound()
{
    return fd >= 0;
}

bool Keyboard::sample(KeyboardEvent* event)
{
    struct input_event ev;
    int len = read(fd, &ev, sizeof(struct input_event));
    if (len != sizeof(struct input_event))
    {
        return false;
    }
    if (ev.type == EV_KEY)
    {
        switch (ev.code)
        {
        case KEY_W:
            event->w = ev.value;
            break;
        case KEY_S:
            event->s = ev.value;
            break;
        case KEY_A:
            event->a = ev.value;
            break;
        case KEY_D:
            event->d = ev.value;
            break;
        case KEY_U:
            event->u = ev.value;
            break;
        case KEY_O:
            event->o = ev.value;
            break;
        case KEY_J:
            event->j = ev.value;
            break;
        case KEY_L:
            event->l = ev.value;
            break;
        case KEY_SPACE:
            event->space = ev.value;
            break;
        }
    }
    return true;
}