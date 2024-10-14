#ifndef KEYBOARD_HPP
#define KEYBOARD_HPP

typedef struct KeyboardEvent
{
    int time;

    int w;
    int s;
    int a;
    int d;
    int u;
    int o;
    int j;
    int l;
    int space;

    KeyboardEvent()
    {
        time = 0;

        w = 0;
        s = 0;
        a = 0;
        d = 0;
        u = 0;
        o = 0;
        j = 0;
        l = 0;
        space = 0;
    }
}KeyboardEvent;

class Keyboard
{
public:
    Keyboard(int id = -1);
    ~Keyboard();

    bool isFound();
    bool sample(KeyboardEvent* event);
private:
    int fd;
};

#endif //KEYBOARD_HPP