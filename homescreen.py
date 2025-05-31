import pygame
import sys
from flappy import start_flappy_bird
from semg_app2v2 import fight_machine
from voicecontrolled import start_voice_control

# initialize pygame
pygame.init()
screen = pygame.display.set_mode((800, 600))
pygame.display.set_caption("My Game")
font = pygame.font.SysFont(None, 48)

# list of game modes, by default select the first one
options = ["Flappy Bird", "vs Machine (EMG based)", "Constant Torque (Voice Controlled)"]
selected = 0

# Function to draw the home screen with options
def draw_home_screen():
    # Clear the screen and draw the title and options
    screen.fill((30, 30, 30))
    title = font.render("Select Game Mode", True, (255, 255, 255))
    screen.blit(title, (250, 100))

    # draw each option
    for i, option in enumerate(options):
        # highlight the selected option
        color = (235, 128, 52) if i == selected else (200, 200, 200)
        text = font.render(option, True, color)
        screen.blit(text, (300, 200 + i * 60))

    pygame.display.flip()

# function to handle input and update the selected option
def handle_input():
    global selected
    # check for events
    for event in pygame.event.get():
        # if user quits, exit the game
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()

        # check for key presses
        elif event.type == pygame.KEYDOWN:
            # if users pressed down, go to the next option
            if event.key == pygame.K_DOWN:
                selected = (selected + 1) % len(options)
            # if users pressed up, go to the previous option
            elif event.key == pygame.K_UP:
                selected = (selected - 1) % len(options)
            # if users pressed enter, return the current option
            elif event.key == pygame.K_RETURN:
                return options[selected]

    return None

# Main loop for the home screen
# draw the home screen initially
draw_home_screen()
while True:
    # check for a selected mode, loop until one is selected
    mode = handle_input()
    if mode:
        print(f"Starting {mode}...")
        # start Flappy Bird game
        if mode == "Flappy Bird":
            start_flappy_bird()
            # reset screen size when game ends
            screen = pygame.display.set_mode((800, 600))
        # start the machine game
        elif mode == "vs Machine (EMG based)":
            fight_machine()
        # start the voice-controlled game
        elif mode == "Constant Torque (Voice Controlled)":
            start_voice_control()
    # redraw the home screen after handling input
    draw_home_screen()

    # input torque doesn't take .
    # torque displays as negative
    # add quit voice command
    # flappy bird setup (let the arm fall)