# Import Bleak
# Create Timer
import datetime

import pygame
from bleak import BleakClient

# BLE Parameters
BLE_ADDRESS = "34:08:E1:19:51:E7"
BLE_UUID = "0000ffe1-0000-1000-8000-00805f9b34fb"

# Initialize Pygame
pygame.init()

# Set up the window
screen_width = 800
screen_height = 500
screen = pygame.display.set_mode((screen_width, screen_height))
pygame.display.set_caption("ECE 2804 Autonomous Tractor")

# Set up the font
font = pygame.font.Font(None, 32)

# Add text box for Autonomous Tractor GUI
text = font.render("Autonomous Tractor GUI", True, (0, 0, 0))
text_rect = text.get_rect()
text_rect.topleft = (20, 20)

# Set up the buttons
start_button = pygame.Rect(20, 450, 100, 40)
stop_button = pygame.Rect(130, 450, 100, 40)

# Define the timer textbook's dimensions and position
timer_textbox_width = 280
timer_textbox_height = 30
timer_textbox_x = 500
timer_textbox_y = 220
timer_textbox = pygame.Rect(timer_textbox_x, timer_textbox_y, timer_textbox_width, timer_textbox_height)

# Define the dimensions and position of the black tape counter textbox
black_tape_textbox = pygame.Rect(20, 220, 330, 30)

# Load the Tractor image
image = pygame.image.load("pngwing.com.png")
image = pygame.transform.scale(image, (240, 300))

# Get the image dimensions
image_width = image.get_width()
image_height = image.get_height()

# Calculate the position of the image
image_x = screen_width / 1.49 - image_width
image_y = 100

# Load the Grass image
image2 = pygame.image.load("grass.png")
image2 = pygame.transform.scale(image2, (1000, 180))

# Calculate the position of the image
image2_x = -100
image2_y = 240

# Load the cloud  image
image3 = pygame.image.load("cloud.png")
image3 = pygame.transform.scale(image3, (220, 220))
# Calculate the position of the image
image3_x = screen_width / 1.6
image3_y = 5

# Load the cloud image
image4 = pygame.image.load("cloud.png")
image4 = pygame.transform.scale(image4, (220, 220))
# Calculate the position of the image
image4_x = screen_width / 9
image4_y = 5

# Initialize the timer
start_time = None
elapsed_time = datetime.timedelta()


# Connect to Bluetooth
async def connect_bluetooth():
    async with BleakClient(BLE_ADDRESS) as client:
        # Connect to the Bluetooth device
        await client.connect()

        # Return the client object so it can be used to send data later
        return client


# Function that sends data using bluetooth
async def send_data_bluetooth(client, data):
    # Encode the data as ASCII
    encoded_data = data.encode('ascii')
    # Write data to the characteristic
    await client.write_gatt_char(BLE_UUID, encoded_data, response=True)


# Receive data from bluetooth
async def receive_data_bluetooth(client):
    # Read data from the characteristic
    data = await client.read_gatt_char(BLE_UUID)
    # Decode the data from ASCII to a string
    decoded_data = data.decode('ascii')

    # Check if the received data matches "BLACK TAPE"
    if decoded_data == "BLACK TAPE":
        # Increment the counter by 1
        counter += 1
        print("Counter incremented to", counter)


# define counter
counter = 0

# Run the game loop
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.MOUSEBUTTONDOWN:
            if start_button.collidepoint(event.pos):
                # TURN ON THROUGH BLUETOOTH
                # asyncio.run(send_data("START"))
                # Start button pressed, start the timer
                start_time = datetime.datetime.now()
            elif stop_button.collidepoint(event.pos):
                # TURN OFF THROUGH BLUETOOTH
                data = data.encode("ASCII")
                # Stop button pressed, stop the timer
                if start_time is not None:
                    elapsed_time = datetime.datetime.now() - start_time
                    start_time = None

    bg_color = (102, 178, 255)  # slightly darker shade of blue
    screen.fill(bg_color)

    # Draw the buttons
    pygame.draw.rect(screen, (0, 255, 0), start_button)
    pygame.draw.rect(screen, (255, 0, 0), stop_button)

    # Draw the button labels
    start_label = font.render("Start", True, (255, 255, 255))
    stop_label = font.render("Stop", True, (255, 255, 255))
    screen.blit(start_label, (start_button.x + 10, start_button.y + 10))
    screen.blit(stop_label, (stop_button.x + 20, stop_button.y + 10))

    # Draw the black tape counter textbox
    pygame.draw.rect(screen, (200, 200, 200), black_tape_textbox)
    black_tape_label = font.render("Number of Black Tapes: {}".format(counter), True, (0, 0, 0))
    screen.blit(black_tape_label, (black_tape_textbox.x, black_tape_textbox.y))

    # Blit the image to the screen
    screen.blit(image, (image_x, image_y))
    screen.blit(image3, (image3_x, image3_y))

    # Draw the timer textbox
    pygame.draw.rect(screen, (200, 200, 200), timer_textbox)
    if start_time is not None:
        elapsed_time = datetime.datetime.now() - start_time
    total_seconds = elapsed_time.total_seconds()
    minutes = int(total_seconds // 60)
    seconds = int(total_seconds % 60)
    timer_text = font.render("Trip Time: {:02d}:{:02d}".format(minutes, seconds), True, (0, 0, 0))
    screen.blit(timer_text, (timer_textbox.x + 10, timer_textbox.y + 5))

    # Blit the image to the screen
    screen.blit(image2, (image2_x, image2_y))
    screen.blit(image4, (image4_x, image4_y))

    # Blit the text box for Autonomous Tractor GUI title onto the screen
    screen.blit(text, text_rect)

    # Update the display
    pygame.display.flip()

# Quit Pygame
pygame.quit()
