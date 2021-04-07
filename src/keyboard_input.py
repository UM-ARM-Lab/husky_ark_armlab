"""
A Python class implementing KBHIT, the standard keyboard-interrupt poller.
Works transparently on Windows and Posix (Linux, Mac OS X).  Doesn't work
with IDLE.

Original source: https://simondlevy.academic.wlu.edu/files/software/kbhit.py
Stackoverflow source: https://stackoverflow.com/questions/2408560/python-nonblocking-console-input
"""

import os
import time

# Windows
if os.name == "nt":
    import msvcrt

# Posix (Linux, OS X)
else:
    import sys
    import termios
    import atexit
    from select import select


class KeyboardInput:
    def __init__(self):
        """Creates a KBHit object that you can call to do various keyboard things."""

        if os.name == "nt":
            pass

        else:

            # Save the terminal settings
            self.fd = sys.stdin.fileno()
            self.new_term = termios.tcgetattr(self.fd)
            self.old_term = termios.tcgetattr(self.fd)

            # New terminal setting unbuffered
            self.new_term[3] = self.new_term[3] & ~termios.ICANON & ~termios.ECHO
            termios.tcsetattr(self.fd, termios.TCSAFLUSH, self.new_term)

            # Support normal-terminal reset at exit
            atexit.register(self.set_normal_term)

    def set_normal_term(self):
        """Resets to normal terminal.  On Windows this is a no-op."""

        if os.name == "nt":
            pass

        else:
            termios.tcsetattr(self.fd, termios.TCSAFLUSH, self.old_term)

    def get_char(self):
        """Returns a keyboard character after kbhit() has been called.
        Should not be called in the same program as getarrow().
        """

        s = ""

        if os.name == "nt":
            return msvcrt.getch().decode("utf-8")

        else:
            return sys.stdin.read(1)

    def get_arrow(self):
        """Returns an arrow-key code after kbhit() has been called. Codes are
        0 : up
        1 : right
        2 : down
        3 : left
        Should not be called in the same program as getch().
        """

        if os.name == "nt":
            msvcrt.getch()  # skip 0xE0
            c = msvcrt.getch()
            vals = [72, 77, 80, 75]

        else:
            c = sys.stdin.read(3)[2]
            vals = [65, 67, 66, 68]

        return vals.index(ord(c.decode("utf-8")))

    def check_hit(self):
        """Returns True if keyboard character was hit, False otherwise."""
        if os.name == "nt":
            return msvcrt.kbhit()

        else:
            dr, dw, de = select([sys.stdin], [], [], 0)
            return dr != []


if __name__ == "__main__":

    keyboard_input = KeyboardInput()

    print("Hit any key, or ESC to exit")

    while True:
        time.sleep(0.2)
        if keyboard_input.check_hit():
            c = keyboard_input.get_char()
            if ord(c) == 27:  # ESC
                break
            print(c, ord(c))

    keyboard_input.set_normal_term()