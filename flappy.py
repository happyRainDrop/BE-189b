import pygame, random, time, odrive
from pygame.locals import *
from odrive.enums import *
from utility import set_torque

# Parameters

# screen size
SCREEN_WIDTH = 400
SCREEN_HEIGHT = 600

# speed in x direction
GAME_SPEED = 2

# width and height of the ground, width mainly used to scale image
GROUND_WIDTH = 2 * SCREEN_WIDTH
GROUND_HEIGHT= 100

# buffer at the top and bottom so the bird doesn't crash into the top or bottom
TOP_BUFFER = 10
BOTTOM_BUFFER = 50

# width and height of the pipes, used to scale image
PIPE_WIDTH = 80
PIPE_HEIGHT = 500

# size of space between a pair of top and bottom pipes
PIPE_GAP = 150


class Bird(pygame.sprite.Sprite):

    def __init__(self):
        # call the parent class constructor
        pygame.sprite.Sprite.__init__(self)

        # load the images for the bird, each image is used for a different frame of the flapping animation
        # each image is repeated 4 times to make the animation smoother
        up = 4 * [pygame.image.load('assets/sprites/bluebird-upflap.png').convert_alpha()]
        mid = 4 * [pygame.image.load('assets/sprites/bluebird-midflap.png').convert_alpha()]
        down = 4 * [pygame.image.load('assets/sprites/bluebird-downflap.png').convert_alpha()]
        self.images = up + mid + down

        # use an index to keep track of the current frame of the bird
        self.current_image = 0

        # convert the images to a format that pygame can use
        self.image = pygame.image.load('assets/sprites/bluebird-upflap.png').convert_alpha()

        # create a mask for the bird to detect collisions
        self.mask = pygame.mask.from_surface(self.image)

        # set the size of the bird
        self.rect = self.image.get_rect()

        # starting position of the bird, 1/6 of the screen width and in the middle of the screen height
        self.rect[0] = SCREEN_WIDTH / 6
        self.rect[1] = SCREEN_HEIGHT / 2

    # update the frame of the bird for flapping animation
    def update(self):
        self.current_image = (self.current_image + 1) % 12
        self.image = self.images[self.current_image]

    # start the bird at the initial frame
    def begin(self):
        self.image = self.images[self.current_image]


class Pipe(pygame.sprite.Sprite):

    def __init__(self, inverted, xpos, ysize):
        # call the parent class constructor
        pygame.sprite.Sprite.__init__(self)

        # load the image for the pipe, convert it to a format that pygame can use
        self.image = pygame.image.load('assets/sprites/pipe-green.png').convert_alpha()
        self.image = pygame.transform.scale(self.image, (PIPE_WIDTH, PIPE_HEIGHT))

        # create a mask for the pipe to detect collisions
        self.mask = pygame.mask.from_surface(self.image)

        # set the size of the pipe
        self.rect = self.image.get_rect()
        self.rect[0] = xpos

        # set the height of the pipe, if inverted, the pipe is at the top of the screen
        if inverted:
            self.image = pygame.transform.flip(self.image, False, True)
            self.rect[1] = - (self.rect[3] - ysize)
        else:
            self.rect[1] = SCREEN_HEIGHT - ysize

    # update the position of the pipe, moving it to the left
    def update(self):
        self.rect[0] -= GAME_SPEED

        

class Ground(pygame.sprite.Sprite):
    
    def __init__(self, xpos):
        # call the parent class constructor
        pygame.sprite.Sprite.__init__(self)

        # load the image for the ground, convert it to a format that pygame can use
        self.image = pygame.image.load('assets/sprites/base.png').convert_alpha()
        self.image = pygame.transform.scale(self.image, (GROUND_WIDTH, GROUND_HEIGHT))

        # create a mask for the ground to detect collisions
        self.mask = pygame.mask.from_surface(self.image)

        # set the size of the ground
        self.rect = self.image.get_rect()
        self.rect[0] = xpos
        self.rect[1] = SCREEN_HEIGHT - GROUND_HEIGHT

    # update the position of the ground, moving it to the left
    def update(self):
        self.rect[0] -= GAME_SPEED

# check if the sprite is off the screen
def is_off_screen(sprite):
    return sprite.rect[0] < -(sprite.rect[2])

# get a pair of pipes at a random height, one inverted and one not
def get_random_pipes(xpos):
    y = random.randint(100, 300)
    pipe = Pipe(False, xpos, y)
    pipe_inverted = Pipe(True, xpos, SCREEN_HEIGHT - y - PIPE_GAP)
    return pipe, pipe_inverted

# main function to start the Flappy Bird game
def start_flappy_bird():
    # connect to the ODrive motor controller and set it up for torque control
    odrv0 = odrive.find_any()
    odrv0.axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL
    odrv0.clear_errors()
    odrv0.axis0.config.motor.current_soft_max = 10
    odrv0.axis0.controller.config.control_mode = ControlMode.TORQUE_CONTROL

    # initialize pygame and set up the screen
    pygame.init()
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    pygame.display.set_caption('Flappy Bird')

    # load the background and begin image, scale them to fit the screen
    BACKGROUND = pygame.image.load('assets/sprites/background-day.png')
    BACKGROUND = pygame.transform.scale(BACKGROUND, (SCREEN_WIDTH, SCREEN_HEIGHT))
    BEGIN_IMAGE = pygame.image.load('assets/sprites/message.png').convert_alpha()

    # create sprite group for the bird
    bird_group = pygame.sprite.Group()
    bird = Bird()
    bird_group.add(bird)

    # create sprite group for the ground
    ground_group = pygame.sprite.Group()

    # create 2 ground sprites, one on screen and one queued up off screen
    for i in range(2):
        ground = Ground(GROUND_WIDTH * i)
        ground_group.add(ground)

    # create sprite group for the pipes
    pipe_group = pygame.sprite.Group()

    # create 2 pairs of random pipes
    for i in range (2):
        pipes = get_random_pipes(SCREEN_WIDTH * i + 800)
        pipe_group.add(pipes[0])
        pipe_group.add(pipes[1])


    # create a clock to control the frame rate
    clock = pygame.time.Clock()

    # display the beginning screen and wait for user input to start the game
    begin = True
    while begin:
        # set the game clock to 60 Hz
        clock.tick(60)

        # check for events, if the user quits or presses a key
        for event in pygame.event.get():
            # if the user quits, exit the game
            if event.type == QUIT:
                pygame.quit()
            # if the user presses a key, start the game
            if event.type == KEYDOWN:
                # to initialize, pull the arm down and reference the starting position as ground level
                set_torque(odrv0, 0.3)
                time.sleep(3)
                starting_pos = odrv0.axis0.pos_estimate
                begin = False

        # draw the background and the begin image
        screen.blit(BACKGROUND, (0, 0))
        screen.blit(BEGIN_IMAGE, (120, 150))

        # if the ground is off the screen, remove it and add a new one
        if is_off_screen(ground_group.sprites()[0]):
            ground_group.remove(ground_group.sprites()[0])

            new_ground = Ground(GROUND_WIDTH - 20)
            ground_group.add(new_ground)

        # update the ground and bird groups, draw them on the screen
        bird.begin()
        ground_group.update()

        bird_group.draw(screen)
        ground_group.draw(screen)

        pygame.display.update()


    # start the game loop
    while True:
        # set the game clock to 60 Hz
        clock.tick(60)

        # if the user quits, exit the game
        for event in pygame.event.get():
            if event.type == QUIT:
                pygame.quit()
        
        # calculate the bird height based on the motor position
        bird.rect[1] = int((TOP_BUFFER - SCREEN_HEIGHT + GROUND_HEIGHT + BOTTOM_BUFFER)*((- odrv0.axis0.pos_estimate + starting_pos)/3.8 - 1))

        # draw the background
        screen.blit(BACKGROUND, (0, 0))

        # if a ground is off the screen, remove it and add a new one
        if is_off_screen(ground_group.sprites()[0]):
            ground_group.remove(ground_group.sprites()[0])

            new_ground = Ground(GROUND_WIDTH - 20)
            ground_group.add(new_ground)

        # if a pipe is off the screen, remove it and add a new pair of pipes
        if is_off_screen(pipe_group.sprites()[0]):
            pipe_group.remove(pipe_group.sprites()[0])
            pipe_group.remove(pipe_group.sprites()[0])

            pipes = get_random_pipes(SCREEN_WIDTH * 2)

            pipe_group.add(pipes[0])
            pipe_group.add(pipes[1])

        # update and draw the bird, ground, and pipe groups
        bird_group.update()
        ground_group.update()
        pipe_group.update()

        bird_group.draw(screen)
        pipe_group.draw(screen)
        ground_group.draw(screen)

        # update the display
        pygame.display.update()

        # check for collisions between the bird and the ground or pipes
        if (pygame.sprite.groupcollide(bird_group, ground_group, False, False, pygame.sprite.collide_mask) or
                pygame.sprite.groupcollide(bird_group, pipe_group, False, False, pygame.sprite.collide_mask)):
            # if a collision is detected, stop the motor and exit the game loop
            time.sleep(1)
            odrv0.axis0.controller.input_torque = 0
            odrv0.axis0.requested_state = AxisState.IDLE
            break