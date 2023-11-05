import pygame
from math import sin,cos
from numpy import deg2rad

#parameters
BLACK = (0,0,0)
WHITE = (255,255,255)
ROBOT_COLOUR1 = (179, 229, 190)
ROBOT_COLOUR2 = (168, 100, 100)

pygame.init()

pose_font = pygame.font.SysFont('Helvetica', 25)
Window_width = 500
Robot_size = 40

class Robot:
    def __init__(self) -> None:
        self.pos = (0,0)
        self.theta = 0

    def updatePose(self,pose):
        self.pos = (pose[0],pose[1])
        self.theta = pose[2]
        
    def draw(self, surface):
        pygame.draw.circle(surface,ROBOT_COLOUR1,self.pos,Robot_size)
        pygame.draw.line(surface,ROBOT_COLOUR2,self.pos,(self.pos[0]+Robot_size*cos(self.theta),self.pos[1]+Robot_size*sin(self.theta)),4)

    def run_sim(self,surface,pose):
        ''' Run simulation'''
        surface.fill(BLACK) # Background color = black
        self.updatePose(pose)
        self.draw(surface)
        pygame.display.update()

        
        

def main():
    r = Robot()
    win = pygame.display.set_mode((Window_width,Window_width))
    win.fill(BLACK)
    pos = (250,250,0)
    while(True):
        r.run_sim(win,pos)
        events = pygame.event.get()
        for e in events:
            # Close the window
            if e.type == pygame.QUIT:
                pygame.quit()

            if e.type == pygame.KEYDOWN:
                #UP
                if e.key == pygame.K_w:
                    pos = (pos[0],pos[1]-10,pos[2])
                    if(pos[1]<0):
                        pos = (pos[0],0,pos[2])

                #DOWN    
                if e.key == pygame.K_s:
                    pos = (pos[0],pos[1]+10,pos[2])
                    if(pos[1]>Window_width):
                        pos = (pos[0],Window_width-1,pos[2])
                
                #LEFT
                if e.key == pygame.K_a:
                    pos = (pos[0]-10,pos[1],pos[2])
                    if(pos[0]<0):
                        pos = (0,pos[1],pos[2])
                
                #RIGHT
                if e.key == pygame.K_d:
                    pos = (pos[0]+10,pos[1],pos[2])
                    if(pos[0]>Window_width):
                        pos = (Window_width-1,pos[1],pos[2])
                
                #CLOCKWISE ROTATE
                if e.key == pygame.K_t:
                    pos = (pos[0],pos[1],pos[2]+deg2rad(10))
                    

                #ANTICLOCKWISE ROTATE  
                if e.key == pygame.K_g:
                    pos = (pos[0],pos[1],pos[2]-deg2rad(10))
                    
                


if __name__ == "__main__":
    main()
