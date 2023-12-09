import sys, select, os
import tty, termios

settings = termios.tcgetattr(sys.stdin)

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

while(1):
    # 多线程，按键一直在监听状态
    key = getKey()
    print(f'按下了{key}--')
    if key=='q':
        break

