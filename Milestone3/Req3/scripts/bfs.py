#!/usr/bin/env python3

import rospy
import numpy as np
import cv2

def Draw_Horizontal(Array,Index1,Index2):   #Draws A horizontal Line from index1 to index2
    Local_Counter = Index1[1]
    while(Local_Counter <= Index2[1]):
        Array[Index1[0],Local_Counter]= 0
        Local_Counter+=1
        
def Draw_Vertical(Array,Index1,Index2):    #Draws A Vertical Line from index1 to index2
    Local_Counter = Index1[0]
    while(Local_Counter <= Index2[0]):
        Array[Local_Counter,Index1[1]]= 0
        Local_Counter+=1

def Draw(Map):    #Draws a Map

    Draw_Horizontal(Map,[0,0],[0,10])
    Draw_Horizontal(Map,[10,0],[10,10])
    Draw_Vertical(Map,[0,0],[10,0])        # EDGES
    Draw_Vertical(Map,[0,10],[10,10])


def BFS(Map,Start,Goal,Direction):  # Direction for CCW -1  for CW 1

    Flag = False # Flag indicating if goal node is found
    
    # Initializing a queue 
    queue = []
    queue.append(Start)

    # Visited Nodes
    Visited = []

    # Parent Node Mapping
    Parents = np.zeros([12,12,2],dtype = int)


    while queue:   # If queue is not empty 

        Node = queue.pop(0)  # Pop first element in the queue

        if Node==Goal:   # Did we reach the goal?
            Flag = True
            break

        Neighbours = get_Neighbours(Node,Direction) # Get neighbours of this node

        for N in Neighbours: # For each neighbour of this node
            if not N in Visited and Map[N[0],N[1]]!=0:   # if this neighbour wasnt visited before and doesnt equal 0 (ie not an obstacle)
                queue.append(N)      # add to the queue
                Visited.append(N)     # add it to the visited
                Parents[N[0],N[1]] = Node                 # add its parent node 
    
    
    if Flag == False:
        print('Mafesh path :(')

    if Flag == True:
        path = get_Path(Parents,Goal,Start) # this function returns the path to the goal 
        return path


        
    

def get_Path(Parents,Goal,Start):

    path = []
    Node = Goal  # start backward 
    path.append(Goal)
   
    while not np.array_equal(Node, Start):

       path.append(Parents[Node[0],Node[1]])  # add the parent node 
       Node = Parents[Node[0],Node[1]]        # search for the parent of that node 
     
    path.reverse()
    return path

    

def get_Neighbours(Node,Direction):    # Returns neighbours of the parent node   Direction = 1 for Clockwise/ Direction = -1 for CCW 

    Neighbours = []
    
    if Direction==1:
        Neighbours.append([Node[0],Node[1]+1])
        Neighbours.append([Node[0]+1,Node[1]+1])
        Neighbours.append([Node[0]+1,Node[1]])
        Neighbours.append([Node[0]+1,Node[1]-1])
        Neighbours.append([Node[0],Node[1]-1])
        Neighbours.append([Node[0]-1,Node[1]-1])
        Neighbours.append([Node[0]-1,Node[1]])
        Neighbours.append([Node[0]-1,Node[1]+1])
                        
    if Direction==-1:
        Neighbours.append([Node[0],Node[1]+1])
        Neighbours.append([Node[0]-1,Node[1]+1])
        Neighbours.append([Node[0]-1,Node[1]])
        Neighbours.append([Node[0]-1,Node[1]-1])
        Neighbours.append([Node[0],Node[1]-1])
        Neighbours.append([Node[0]+1,Node[1]-1])
        Neighbours.append([Node[0]+1,Node[1]])
        Neighbours.append([Node[0]+1,Node[1]+1])

    return Neighbours
    
if __name__ == '__main__':

    img = cv2.imread('/home/teleb/catkin_ws/src/milestone3Autonomous/scripts/path.png')
    grayImage = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret, bw_img = cv2.threshold(grayImage,30,255,cv2.THRESH_BINARY)

    Draw(bw_img)
    path = BFS(bw_img, [1,1],[4,9],-1)
    print(path)
    print(bw_img)
    
    cv2.waitKey(0)
    cv2.destroyAllWindows()


