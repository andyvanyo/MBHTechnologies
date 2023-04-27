import curses

#set up curses for keyboard control
screen = curses.initscr()
curses.noecho()
curses.cbreak()
screen.keypad(True)

try:
    while True:
        char = screen.getch()
        if char == ord('q'):
            break
        elif char == curses.KEY_UP:
            print("F")
        elif char == curses.KEY_DOWN:
            print("B")
        elif char == curses.KEY_LEFT:
            print("L")
        elif char == curses.KEY_RIGHT:
            print("R")
        elif char == curses.KEY_BACKSPACE:
            print("S")
finally:
    curses.nocbreak(); screen.keypad(0); curses.echo();
    curses.endwin()