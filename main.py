import pygame
import threading
from nav_controller import Bot
from util import RobotStatePublisher, RobotState
from state_pubsub.state_sub import RobotStateSub, BotEntry
import rclpy
from dataclasses import dataclass
import time
import csv


class RobotManager():
    """
    Stores a list of robots, as well as their current state.\n
    Updates their current state by observing robot_state topic as well as other topics.

    Class Variables:
    - `bots` (list[BotEntry]): Tracks available robots and holds relevant information about them (state)
    """
    def __init__(self):

        self.sub = RobotStateSub()
        self.pub = RobotStatePublisher()

        #start subscriber in different thread
        ros_thread = threading.Thread(target=self.start_subscriber, args=(self.sub,), daemon=True)
        ros_thread.start()

        #Create nav instances if they do not exist
        self.timer_thread = threading.Thread(target=self.update_bots, daemon=True)
        self.timer_thread.start()

        # Load robot information into robot_details dictionary
        self.robots_config = [dict]
        self.load_config()

    def load_config(self):
        with open("warehouse_robots_config.csv", newline="", encoding="utf-8") as csvfile:

            # Add each robot in csv file and add to robot_entry
            reader = list(csv.reader(csvfile))
            for row in reader[1:]:
                self.robots_config.append(
                    {
                        "name" : row[0],
                        "start_pos" : (row[1],row[2],row[3]),
                        "delivery_pos" : (row[4],row[5],row[6])
                    }
                )
        print(self.robots_config)

    def search_config(self, name : str) -> dict:
        print(self.robots_config)
        for c in self.robots_config:
            if c["name"] == name:
                return c
        raise Exception("Robot name must match a robot in warehouse robot config file!")

    def start_subscriber(self, node):
        rclpy.spin(node)
    

    def stop_subscriber(self):
        self.sub.destroy_node()
        rclpy.shutdown()

    def print_bots(self):
        print(self.sub.bots)

    def update_bots(self):
        while True:
            for bot in self.sub.bots:
                if bot.nav_manager == None:
                    bot_details = self.search_config(bot.name)
                    bot.nav_manager = Bot(bot_details["start_pos"], bot_details["delivery_pos"], f"{bot.name}", self.pub)
                    
            time.sleep(1)

    def navigate_bot(self, name : str, pos : tuple[float, float, float]):
        b = self.search_bot(name)
        print(f"Navigating bot {b.name}")
        nav_thread = threading.Thread(target=b.nav_manager.navigate_to_position, args = (pos[0],pos[1],0))
        nav_thread.start()

    

    def search_bot(self, name : str) -> BotEntry:
        bot : BotEntry
        for b in self.sub.bots:
            if b.name == name:
                return b
        
        print(f"Bot {name} not found!")
        return None
        




#region pygame init
pygame.init()

# Screen dimensions
SCREEN_WIDTH, SCREEN_HEIGHT = 1200, 1000
screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
pygame.display.set_caption("Robot Warehouse")

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




class StatusBar:
    def __init__(self, x, y, width = 500, height = 100, name="Robot", battery=0, bot = BotEntry):
        self.x, self.y = x, y
        self.width, self.height = width, height
        self.name = name
        self.battery = battery 
        self.bot = bot

    def draw(self, surface):
        # Draw status bar background
        pygame.draw.rect(surface, CREAM, (self.x, self.y, self.width, self.height), border_radius=10)

        # Draw name
        name_text = font.render(f"Name: {self.name}", True, BLACK)
        surface.blit(name_text, (self.x + 10, self.y + 10))


        # Draw status text
        status_text = font.render(f"Status: {RobotState(self.bot.state).name}", True, BLACK)
        surface.blit(status_text, (self.x + 10, self.y + 40))

        # Draw battery bar outline
        battery_x = self.x + 10
        battery_y = self.y + 70
        battery_width = 200
        battery_height = 25
        pygame.draw.rect(surface, BLACK, (battery_x, battery_y, battery_width, battery_height), 2)

        # Fill battery level
        battery_fill_width = int((self.battery / 100) * (battery_width - 4))
        battery_fill_color = GREEN if self.battery > 20 else RED
        pygame.draw.rect(surface, battery_fill_color, (battery_x + 2, battery_y + 2, battery_fill_width, battery_height - 4))

        # Display battery percentage
        battery_text = font.render(f"Battery: {self.battery}%", True, BLACK)
        surface.blit(battery_text, (battery_x + battery_width + 10, battery_y))


    def update_battery(self, value):
        self.battery = max(1, min(100, value))


    
class Console:
    def __init__(self, x, y, width, height, bot_manager : RobotManager):
        self.x, self.y, self.width, self.height = x, y, width, height
        self.text = ""
        self.output_text = ""
        self.active = False
        self.bot_manager = bot_manager

    def draw(self, surface):
        pygame.draw.rect(surface, BLACK, (self.x, self.y, self.width, self.height))
        pygame.draw.rect(surface, WHITE, (self.x + 2, self.y + 2, self.width - 4, self.height - 4))
        
        # Draw text
        text_surface = font.render(self.text, True, BLACK)
        error_text_surface = font.render(self.output_text, True, RED)
        surface.blit(text_surface, (self.x + 5, self.y + 5))
        surface.blit(error_text_surface, (self.x + 5, self.y + 50))


    def handle_event(self, event):
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_RETURN:
                self.process_command(self.text)
                self.text = "" #clear input

            elif event.key == pygame.K_BACKSPACE:
                self.text = self.text[:-1]

            # Add key (which the user pressed)
            else:
                self.text += event.unicode

    def process_command(self, command):
        if command.lower().startswith("navigate"):
            try:
                split = command.lower().split(" ")
                
                if len(split) < 3:
                    self.output_text = "navigate command takes 2 arguments! use the format navigate <bot_name> <(x,y,z)>"
                    return
                
                bot = self.bot_manager.search_bot(split[1])

                if not bot:
                    self.output_text = f"Robot {split[1]} could not be found!"
                    return

                pos : tuple[float, float, float] = split[2]
                pos_tuple = tuple(float(x) for x in pos[1:-1].split(','))

                self.bot_manager.navigate_bot(bot.name, pos_tuple)
                self.output_text = f"Navigating robot {split[1]} to position {split[2]}"
            except:
                raise Exception("oops!")
        
        else:
            print("unrecognised command")

 
#endregion


# Main loop
def main():
    clock = pygame.time.Clock()
    running = True
    mainpage = True
    button_list = []
    fps = 0
    rclpy.init()

    bot_manager = RobotManager()

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

    button_refresh = Button(200, SCREEN_HEIGHT - 200, 150, 80, "RESET", font, GREEN, DARK_GRAY)
    button_stop = Button(50, SCREEN_HEIGHT - 200, 150, 80, "STOP", font, RED, DARK_GRAY)


    # button list
    button_list.append(button_stop)
    button_list.append(button_refresh)
    
    #endregion

    #region robot status

    # Declare positions for consecutive status bars to go
    y_lim : int = SCREEN_HEIGHT - 200
    x_lim : int = SCREEN_WIDTH
    status_pos : tuple[int, int] = []
    status_list : StatusBar = []
    status_gap = 30
    status_width = 500
    status_height = 100
    y_app = status_height + status_gap
    x_app = status_width + status_gap

    s_x = 80
    s_y = 150
    status_pos.append((s_x, s_y))
    s_y += y_app
    # Populate status positions
    while not s_y + status_gap + status_height > y_lim and not s_x + status_gap + status_width > x_lim:
        # add position to list, and increment row
        status_pos.append((s_x, s_y))
        s_y += y_app

        # If we have reached the bottom of current column, increment column and reset y pos
        if s_y + status_gap + status_height > y_lim:
            s_x += x_app
            s_y = 150
            print(not s_x + status_gap + status_width > x_lim)
            
    #console
    console = Console(0, SCREEN_HEIGHT-100, SCREEN_WIDTH, 100, bot_manager)
    #endregion
    
    
    while running:
        screen.fill(ALGAE)

        #events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            #region mainpage
            if button_stop.is_clicked(event):
                print("STOPPING")
                bot_manager.print_bots()

            if event.type == pygame.KEYDOWN:
                console.handle_event(event)

            #endregion

        if fps % 10 == 0:
            fps = 0
            bots = bot_manager.sub.bots
            for i in range(len(bots)):
                
                status_list.append(StatusBar(status_pos[i][0], status_pos[i][1], name = bots[i].name, bot=bots[i]))


        #Nav bar
        pygame.draw.rect(screen, DARK_GREEN, (nav_x, nav_y, nav_width, nav_height))

        
        if mainpage:
            # drawing
            #screen.blit(pgm_surface, (400, 150))

            for button in button_list:
                button.draw(screen)

            #status
            #pygame.draw.rect(screen, CREAM, (status_x, status_y, status_width, status_height))
            for s in status_list:
                s.draw(screen)

            #console
            console.draw(screen)
            
        pygame.display.flip()
        fps += 1
        clock.tick(60)

    #bot.end_nav2_process()
    #print("Shutting down nav2")
    pygame.quit()

if __name__ == "__main__":
    main()
