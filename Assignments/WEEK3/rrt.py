import numpy as np
import matplotlib.pyplot as plt
import random
import shapely.geometry
import descartes

class Node:
    def __init__(self, x, y, parent = None ):
        self.x, self.y = x, y
        self.parent = parent 
        self.distance = 0

    def __eq__(self, o: object) -> bool:
        return self.x == o.x and self.y == o.y


# copied from the ContinuousWorld file given :p
class Obstacle:
    def __init__(self, start= [0,0], goal = [10,10], X=10, Y=10 ) -> None:
        self.start = start
        self.goal = goal
        self.X = X
        self.Y = Y
        self.obstacle_level=0.10
        self.obstacles = []

    def make_obstacle(self):
        X_val = [i for i in range(0, self.X - 1) if i not in [self.start[0], self.goal[0]] ]
        Y_val = [i for i in range(0, self.Y - 1) if i not in [self.start[1], self.goal[1]] ]
        for i in range(0, int(self.obstacle_level * self.X * self.Y)):
            # randomly select a corner in X, Y
            x, y = np.random.choice(X_val), np.random.choice(Y_val)
            height, width = np.random.uniform(0, 1.5), np.random.uniform(0, 1.5)
            rect = shapely.geometry.Polygon([(x, y), (x, y + height), (x + width, y + height), (x + width, y)])
            self.obstacles.append(rect)
        
        return self.obstacles

    def plot_obstacles(self, ax):
        for obst in self.obstacles:
            ax.add_patch(descartes.PolygonPatch(obst, fc='black', alpha=0.75))
        ax.scatter([self.start[0], self.goal[0]], [self.start[1], self.goal[1]], color="green", alpha=0.9)

class RRT():
    def __init__(self, start, goal, threshold, max_iter=500):
        self.start = start
        self.goal = goal
        self.threshold = threshold
        self.nodes = []
        self.reached = False
        self.max_iter = max_iter

        ob = Obstacle()
        ob.make_obstacle()
        
        self.X = ob.X
        self.Y = ob.Y
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(1, 1, 1)
        self.ax.set_xlim([-2.5, int(self.X) + 2.5])
        self.ax.set_ylim([-2.5, int(self.Y) + 2.5])

        ob.plot_obstacles(self.ax)  
        self.obstacle_list = ob.make_obstacle()

        self.nodes= []
        

    def distance(self, point1, point2):
        distance = np.sqrt((point1.x - point2.x)**2 + (point1.y - point2.y)**2)
        return distance

    def collisionCheck(self, point1, point2):
        line = shapely.geometry.LineString([[point1.x, point1.y], [point2.x, point2.y]])
        collision = False
        for obs in self.obstacle_list:
            if line.intersects(obs):
                collision = True
                # print("Collision")
        return collision

    def get_random_node(self):
        new_node = Node(random.random() * 10, random.random()*10)
        if self.distance(new_node, self.goal) < self.threshold:     # if the random point is in threshold region around goal
            new_node = self.goal
            self.goal.parent = new_node
        return new_node

    def search(self):
        self.nodes.append(self.start)
        count = 0
        while count < self.max_iter:
            new_node = self.get_random_node()
            for node in self.nodes:
                node.distance = self.distance(node, new_node)
            self.nodes = sorted(self.nodes, key=lambda node: node.distance)
            nearby_node = self.nodes[0]
            for node in self.nodes:
                node.distance = 0 

            if self.collisionCheck(nearby_node, new_node) == False:
                new_node.parent = nearby_node
                self.nodes.append(new_node)
            else: continue

            # if new_node.x == self.goal.x and new_node.y == self.goal.y:
            if new_node == self.goal:
                self.reached == True
                print(count)
                print('Reached')
                break
            count = count + 1
        if count == self.max_iter:
            print("Not reached")

    def get_path(self):
        self.search()
        path = [self.nodes[-1]]
        current = self.nodes[-1]
        while True:
            if current.x == self.start.x and current.y == self.start.y:
                path.append(current)
                break
            else:
                path.append(current.parent)
                current = current.parent
        path_planned = [(p.x, p.y) for p in path]
        return path_planned

    def plot_path(self, path):
        self.ax.scatter([p[0] for p in path], [p[1] for p in path], color="r", marker="x")
        self.ax.plot([p[0] for p in path], [p[1] for p in path], color="green", alpha=0.75)
        plt.show()

if __name__ == "__main__" :
    start = Node(0,0)
    goal = Node(10,10)
    rrt = RRT(start, goal, 1)
    path = rrt.get_path()
    # print(path)
    rrt.plot_path(path)