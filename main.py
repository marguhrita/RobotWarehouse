import pygame
import threading
from nav_controller import Bot
from PIL import Image
from util import RobotStatePublisher
from state_pubsub.state_pub import RobotState
from state_pubsub.state_sub import RobotStateSub
from typing import List, TypedDict
import rclpy


#region pygame init
pygame.init()

# Screen dimensions
SCREEN_WIDTH, SCREEN_HEIGHT = 1200, 800
screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
pygame.display.set_caption("Robot Warehouse")
def pgm():
    # Load PGM image using Pillow
    pgm_path = "map.pgm"  # Replace with your PGM file path
    image = Image.open(pgm_path)

    # Convert grayscale to RGB (Pygame requires RGB format)
    image = image.convert("RGB")

    # Resize image to 350x350 (using LANCZOS for high-quality scaling)
    image = image.resize((350, 350), Image.LANCZOS)

    # Convert Pillow image to Pygame surface
    pgm_surface = pygame.image.fromstring(image.tobytes(), image.size, "RGB")
    
    return pgm_surface

pgm_surface = pgm()

# constants
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
GRAY = (200, 200, 200)
DARK_GRAY = (150, 150, 150)
RED = (255,0,0)
GREEN = (0,255,0)
ALGAE = (96, 108, 56)
DARK_GREEN = (40, 54, 24)
CREAM = (254, 250, 224)
font = pygame.font.Font(None, 40)


class Button():
    def __init__(self, x, y, width, height, text, font, base_color, hover_color):
        
        self.rect = pygame.Rect(x, y, width, height)
        self.text = text
        self.font = font
        self.base_color = base_color
        self.hover_color = hover_color

    def draw(self, screen):
        # Check if the mouse is hovering over the button
        mouse_pos = pygame.mouse.get_pos()
        if self.rect.collidepoint(mouse_pos):
            pygame.draw.rect(screen, self.hover_color, self.rect)
        else:
            pygame.draw.rect(screen, self.base_color, self.rect)

        # Draw the text
        text_surface = self.font.render(self.text, True, BLACK)
        text_rect = text_surface.get_rect(center=self.rect.center)
        screen.blit(text_surface, text_rect)

    def is_clicked(self, event):
        if event.type == pygame.MOUSEBUTTONDOWN:
            if self.rect.collidepoint(event.pos):
                return True
        return False

#endregion


class StatusBar:
    def __init__(self, x, y, width, height, name="Robot", battery=100, status="Idle"):
        self.x, self.y = x, y
        self.width, self.height = width, height
        self.name = name
        self.battery = battery  # Battery percentage (1-100)
        self.status = status

    def draw(self, surface):
        # Draw status bar background
        pygame.draw.rect(surface, CREAM, (self.x, self.y, self.width, self.height), border_radius=10)

        # Draw name
        name_text = font.render(f"Name: {self.name}", True, BLACK)
        surface.blit(name_text, (self.x + 10, self.y + 10))

        # Draw battery bar outline
        battery_x = self.x + 10
        battery_y = self.y + 40
        battery_width = 200
        battery_height = 25
        pygame.draw.rect(surface, BLACK, (battery_x, battery_y, battery_width, battery_height), 2)

        # Fill battery level
        battery_fill_width = int((self.battery / 100) * (battery_width - 4))
        battery_fill_color = GREEN if self.battery > 30 else RED
        pygame.draw.rect(surface, battery_fill_color, (battery_x + 2, battery_y + 2, battery_fill_width, battery_height - 4))

        # Display battery percentage
        battery_text = font.render(f"Battery: {self.battery}%", True, BLACK)
        surface.blit(battery_text, (battery_x + battery_width + 10, battery_y))

        # Draw status text
        status_text = font.render(f"Status: {self.status}", True, BLACK)
        surface.blit(status_text, (self.x + 10, self.y + 75))

    def update_battery(self, value):
        self.battery = max(1, min(100, value))  # Clamp battery between 1 and 100

    def update_status(self, new_status):
        self.status = new_status





class RobotManager():
    """
    Stores a list of robots, as well as their current state.\n
    Updates their current state by observing robot_state topic as well as other topics.

    Class Variables:
    - `bots` (list[BotEntry]): Tracks available robots and holds relevant information about them (state)
    """
    def __init__(self):

        rclpy.init()
        self.sub = RobotStateSub()

        #start subscriber in different thread
        ros_thread = threading.Thread(target=self.start_subscriber, args=(self.sub,), daemon=True)
        ros_thread.start()

    def start_subscriber(self, node):
        rclpy.spin(node)

    def stop_subscriber(self):
        # Cleanup
        self.sub.destroy_node()
        rclpy.shutdown()


#region commander init
#bot = Bot((1.7,0.222,0),"tb3_0")

bot_manager = RobotManager()



def navigate(x,y,z):
    nav_thread = threading.Thread(target=bot.navigate_to_position, args = (x,y,z))
    nav_thread.start()

#state_pub = RobotStatePublisher()

# Main loop
def main():
    clock = pygame.time.Clock()
    running = True
    mainpage=True
    button_list = []

    #region navbarinit
    nav_pad = 20
    nav_x, nav_y, nav_width, nav_height = 0, 0, SCREEN_WIDTH, 120
    button_nav_main = Button(nav_pad, nav_pad, 150, 80, "Home", font, CREAM, DARK_GRAY)

    button_list.append(button_nav_main)


    #region mainpageinit

    offset_y = 100
    offset_x = 175
    start_y = 150
    start_x = 25
    button_goal_a = Button(start_x, start_y, 150, 80, "Goal A", font, GRAY, DARK_GRAY)
    button_goal_b = Button(start_x + offset_x, start_y, 150, 80, "Goal B", font, GRAY, DARK_GRAY)
    button_goal_c = Button(start_x, + start_y + offset_y, 150, 80, "Goal C", font, GRAY, DARK_GRAY)
    button_goal_d = Button(start_x + offset_x, start_y + offset_y, 150, 80, "Goal D", font, GRAY, DARK_GRAY)
    button_start = Button(start_x, start_y + offset_y * 2, 150, 80, "START", font, GREEN, DARK_GRAY)
    button_stop = Button(start_x + offset_x, start_y + offset_y * 2, 150, 80, "STOP", font, RED, DARK_GRAY)


    # button list
    #button_list.append(button_goal_a)
    #button_list.append(button_goal_b)
    button_list.append(button_stop)
    button_list.append(button_start)
    button_list.append(button_goal_c)
    button_list.append(button_goal_d)
    #endregion

    #region robot status
    #status_x, status_y, status_width, status_height = 25, 150, 600, 100
    status_bar = StatusBar(25, 150, 600, 100)

    #endregion
    
 
    while running:
        screen.fill(ALGAE)

        #events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            #region mainpage
            if button_goal_a.is_clicked(event):
                print("Navigating to Goal A!!")
                #bot.navigate_to_position((0.056,0.01,0))
                #navigate(0.056,0.01,0)

            if button_goal_b.is_clicked(event):
                print("Goal B clicked!")
                #bot.navigate_to_position((1.43,0.01,0))
                #navigate(1.43,0.01,0)

            if button_goal_c.is_clicked(event):
                print("Goal C clicked!")
                #bot.navigate_to_position((-0.9,1.44,0))
                #navigate(-0.9,1.44,0)


            if button_goal_d.is_clicked(event):
                print("Goal D clicked!")
                #bot.navigate_to_position((-0.9,-0.8,0))
                #navigate(-0.9,-0.8,0)

            if button_start.is_clicked(event):
                print("start")
                #state_pub.publish("tb3", RobotState.ONLINE)

            #endregion

        #Nav bar
        pygame.draw.rect(screen, DARK_GREEN, (nav_x, nav_y, nav_width, nav_height))

        
        if mainpage:
            # drawing
            #screen.blit(pgm_surface, (400, 150))

            for button in button_list:
                button.draw(screen)

            #status
            #pygame.draw.rect(screen, CREAM, (status_x, status_y, status_width, status_height))
            status_bar.draw(screen)

            
        pygame.display.flip()
        clock.tick(60)

    #bot.end_nav2_process()
    #print("Shutting down nav2")
    pygame.quit()

if __name__ == "__main__":
    main()
