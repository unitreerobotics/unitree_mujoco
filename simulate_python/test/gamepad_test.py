import pygame
import sys

pygame.init()
pygame.joystick.init()

joystick_count = pygame.joystick.get_count()
if joystick_count > 0:
    joystick = pygame.joystick.Joystick(0) 
    joystick.init()
else:
    sys.exit()

while True:
    # for event in pygame.event.get():
    #     if event.type == pygame.JOYBUTTONDOWN:
    #         print(f"Button pressed： {event.button}")
    #     elif event.type == pygame.JOYBUTTONUP:
    #         print(f"Button released： {event.button}")
    pygame.event.get()
        # 读取轴位置
    axes = joystick.get_button(0)
    print(joystick.get_numaxes(), joystick.get_numhats(),joystick.get_numbuttons(),joystick.get_numballs())
    print(f"axis： {axes}")


    pygame.time.wait(100)

