import pygame
from nav_controller import Bot

#region pygame init
pygame.init()

# Screen dimensions
SCREEN_WIDTH, SCREEN_HEIGHT = 800, 400
screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
pygame.display.set_caption("Robot Warehouse")

# constants
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
GRAY = (200, 200, 200)
DARK_GRAY = (150, 150, 150)
RED = (255,0,0)
GREEN = (0,255,0)

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

#region commander init
bot = Bot((1.43,0.01,0), "bot1")
print("Bot Created!")


# Main loop
def main():
    clock = pygame.time.Clock()
    running = True
    font = pygame.font.Font(None, 40)  # Default Pygame font
    offset_y = 100
    offset_x = 175
    start_y = 20
    start_x = 25
    button_goal_a = Button(start_x, start_y, 150, 80, "Goal A", font, GRAY, DARK_GRAY)
    button_goal_b = Button(start_x + offset_x, start_y, 150, 80, "Goal B", font, GRAY, DARK_GRAY)
    button_goal_c = Button(start_x, + start_y + offset_y, 150, 80, "Goal C", font, GRAY, DARK_GRAY)
    button_goal_d = Button(start_x + offset_x, start_y + offset_y, 150, 80, "Goal D", font, GRAY, DARK_GRAY)
    button_stop = Button(start_x, start_y + offset_y * 2, 150, 80, "START", font, GREEN, DARK_GRAY)
    button_start = Button(start_x + offset_x, start_y + offset_y * 2, 150, 80, "STOP", font, RED, DARK_GRAY)


    # button list
    button_list = []
    button_list.append(button_goal_a)
    button_list.append(button_goal_b)
    button_list.append(button_stop)
    button_list.append(button_start)
    button_list.append(button_goal_c)
    button_list.append(button_goal_d)

    map = pygame.Rect(400, 20, 350, 350)



    while running:
        screen.fill(WHITE)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            if button_goal_a.is_clicked(event):
                print("Navigating to Goal A!!")
                bot.navigate_to_position((0.056,0.01,0))

            if button_goal_b.is_clicked(event):
                print("Goal B clicked!")
                bot.navigate_to_position((1.43,0.01,0))

        # draw map
        pygame.draw.rect(screen, GRAY, map)

        for button in button_list:
            button.draw(screen)
        pygame.display.flip()
        clock.tick(60)

    bot.end_nav2_process()
    print("Shutting down nav2")
    pygame.quit()

if __name__ == "__main__":
    main()
