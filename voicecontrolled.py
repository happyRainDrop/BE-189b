import pygame
import speech_recognition as sr
import threading
from difflib import SequenceMatcher
import odrive
from odrive.enums import *
from utility import set_torque

# Initialize pygame
pygame.init()
screen = pygame.display.set_mode((600, 400))
pygame.display.set_caption("Voice Torque Control")
font = pygame.font.Font(None, 74)
clock = pygame.time.Clock()

# initialize speech recognizer
recognizer = sr.Recognizer()
# initialize microphone
mic = sr.Microphone()

# Connect to ODrive
print("Searching for ODrive...")
odrv0 = odrive.find_sync()
print(f"ODrive found! Firmware Version: {odrv0.fw_version_major}.{odrv0.fw_version_minor}.{odrv0.fw_version_revision}")

# Set up ODrive
print("Setting up axis...")
odrv0.clear_errors()
odrv0.axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL
odrv0.axis0.controller.config.control_mode = ControlMode.TORQUE_CONTROL
odrv0.axis0.config.motor.current_soft_max = 10  # Amps

# get the similarity between two words
def similarity(a, b):
    return SequenceMatcher(None, a.lower(), b.lower()).ratio()

# classify the voice command word into "stronger" or "relax"
def classify(command):
    # get the similarity between the voice command and "stronger" / "relax"
    up_sim = similarity(command, "stronger")
    down_sim = similarity(command, "relax")

    # threshold for similarity is 0.4, if not met then return "white noise"
    if up_sim > down_sim and up_sim > 0.4:
        return "stronger"
    elif down_sim > up_sim and down_sim > 0.4:
        return "relax"
    return "White Noise"

# listening for voice commands
def listen_for_commands():
    global torque
    while listening:
        try:
            with mic as source:
                # listen for a word and use google's speech recognizer
                print("Listening...")
                audio = recognizer.listen(source)
                spoken = recognizer.recognize_google(audio)
                print(f"Heard: {spoken}")

                # classify the word heard
                word = classify(spoken)
                print(f"Classified as: {word}")

                # if the word heard matches our two voice commands, increase or decrease torque
                if word == "stronger":
                    # our torque is negative, so this is increasing it
                    torque -= 0.1
                elif word == "relax":
                    torque = min(0.0, torque + 0.1)

                # update the torque based on the voice command
                set_torque(odrv0, torque)

        # handle errors
        except sr.WaitTimeoutError:
            continue
        except sr.UnknownValueError:
            continue
        except sr.RequestError as e:
            print(f"Speech Recognition error: {e}")

# spawn a thread for listening for voice commands
def start_listening():
    global listening
    listening = True
    threading.Thread(target=listen_for_commands, daemon=True).start()

# main loop
def start_voice_control():
    # start listening for voice commands
    start_listening()

    # initialize torque at 0
    global torque
    torque = 0.0

    # boolean indicating we're waiting for user input_text of the initial torque
    input_mode = True
    input_text = ""
    # boolean indicating the mic is listening for keywords
    listening = False
    
    # start looping
    running = True
    while running:

        # set the background
        screen.fill((30, 30, 30))

        # if user quits, break out of the loop and stop looping
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
                break

            # handle key presses
            elif event.type == pygame.KEYDOWN and input_mode:
                # if retun was pressed
                if event.key == pygame.K_RETURN:
                    # try setting the torque from the input string
                    try:
                        # torque is negative due to the way we wind the motor
                        torque = -float(input_text)
                        input_mode = False
                        set_torque(odrv0, torque)
                        start_listening()

                    # if failed, clear it
                    except ValueError:
                        input_text = ""

                # handle delete
                elif event.key == pygame.K_BACKSPACE:
                    input_text = input_text[:-1]
                
                # ignore invalid characters
                elif event.unicode.isdigit() or (event.unicode == '.' and len(input_text) == 0):
                    input_text += event.unicode

        # update torque
        set_torque(odrv0, torque)

        # render torque text
        if input_mode:
            label = font.render("Enter a number: " + input_text, True, (235, 128, 52))
        else:
            label = font.render("Torque: " + str(torque), True, (235, 128, 52))

        # draw the game screen 
        screen.blit(label, (50, 150))
        pygame.display.flip()

        # clock the game at 1 Hz
        clock.tick(1)

    # On quit, stop the motor and set to idle state
    print("Stopping motor...")
    odrv0.axis0.controller.input_torque = 0
    odrv0.axis0.requested_state = AxisState.IDLE
    pygame.quit()
