import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry import Point, LineString, Polygon
from sklearn.neighbors import KDTree
from descartes import PolygonPatch

PLOT_POINTS = False
POINT_DIST = 0.5
PLOT_START_AND_GOAL = True


class Node:
    def __init__(self, x, y):
        self.x, self.y = x, y


class Obstacle:
    def __init__(self, obj, type_):
        self.obj = obj
        self.patch = PolygonPatch(obj, fc="black", alpha=0.75)
        self.type = type_

    def plot_obstacle(self, ax):
        ax.add_patch(self.patch)

    def check_collision(self, point):
        if point.within(self.obj):
            return True
        else:
            return False

    def __str__(self):
        print(
            f"Box at : {self.obj.exterior.coords.xy}"
        ) if self.type == "box" else print(
            f"Circle at : {self.obj.centroid.x}, {self.obj.centroid.y}"
        )

    def __repr__(self):
        return (
            "Box : {}\n".format(self.obj.exterior.coords.xy)
            if self.type == "box"
            else "Circle : {:1f}, {:1f}\n".format(
                self.obj.centroid.x, self.obj.centroid.y
            )
        )


class GridWorld:
    """Basic World that consisits of Grid
    Args:
        X : Maximum Width of the World
        Y : Maximum length of the World
        obstacle_level (0 to 1) : level of obstacles
        obst_type : list of strings. Supported "circles" and "boxes"
    """

    def __init__(
        self,
        X=10,
        Y=10,
        obstacle_level=0.9,
        obst_type=["circles", "boxes"],
        start=[0, 0],
        goal=[10, 10],
    ):
        self.X = X
        self.Y = Y
        self.obstacle_level = obstacle_level
        self.obst_type = obst_type
        self.start = start
        self.goal = goal

        # List of all the nodes and obstacles
        self.nodes = []
        self.tree = None
        self.obstacles = []

        # Make the obstacles
        self.make_obstacles()

        # Method to make the nodes
        # Nodes are stored in a KDTree data structure for fast nearest neighbour query
        self.make_nodes()

        # Plotting information
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(1, 1, 1)
        self.ax.set_xlim([-2.5, int(self.X) + 2.5])
        self.ax.set_ylim([-2.5, int(self.Y) + 2.5])

    def __len__(self):
        return len(self.nodes)

    def __str__(self):
        return f"``Grid World``\nDimensions : {self.X}, {self.Y}\nObstacle level : {self.obstacle_level}\nObstacle Types : {self.obst_type}"

    def make_obstacles(self):
        X = [i for i in range(0, self.X - 1) if i not in [self.start[0], self.goal[0]]]
        Y = [i for i in range(0, self.Y - 1) if i not in [self.start[1], self.goal[1]]]

        for i in range(0, int(self.obstacle_level * self.X * self.Y)):
            # randomly select a corner in X, Y
            x, y = np.random.choice(X), np.random.choice(Y)

            # choose between a square and a rectangle
            obst_type = np.random.choice(self.obst_type)

            if obst_type == "boxes":
                height, width = np.random.uniform(1, 3), np.random.uniform(1, 3)
                self.obstacles.append(
                    Obstacle(
                        Polygon(
                            [
                                (x, y),
                                (x, y + height),
                                (x + width, y + height),
                                (x + width, y),
                            ]
                        ),
                        "box",
                    )
                )
            else:
                self.obstacles.append(
                    Obstacle(Point(x, y).buffer(np.random.uniform(0.5, 1.5)), "circle")
                )

    def make_nodes(self):
        # make all the nodes and store them in a KDTree
        self.nodes = []
        for x in np.arange(-2, self.X + 2, POINT_DIST):
            for y in np.arange(-2, self.Y + 12, POINT_DIST):
                self.nodes.append([x, y])
        self.nodes = np.array(self.nodes)
        self.tree = KDTree(self.nodes, leaf_size=2)

    # --- Plotting functions
    def plot_obstacles(self):
        for obst in self.obstacles:
            obst.plot_obstacle(self.ax)
        if PLOT_POINTS:
            self.plot_points()
        if PLOT_START_AND_GOAL:
            self.plot_start_and_goal()

    def plot_points(self):
        for x in np.arange(-2, self.X + 2, POINT_DIST):
            for y in np.arange(-2, self.Y + 2, POINT_DIST):
                plt.scatter(x, y, color="red", alpha=0.5)

    def plot_start_and_goal(self):
        self.ax.scatter(
            [self.start[0], self.goal[0]],
            [self.start[1], self.goal[1]],
            color="green",
            alpha=0.9,
        )

    def plot_path(self, path):
        self.ax.scatter(
            [p[0] for p in path], [p[1] for p in path], color="r", marker="x"
        )
        self.ax.plot(
            [p[0] for p in path], [p[1] for p in path], color="green", alpha=0.75
        )
        plt.show()

    def plot_world(self, path=None):
        self.plot_obstacles()
        if path is not None:
            self.plot_path(path)

    # --- Functions for checking collisions
    def check_collision_of_point(self, point):
        for obst in self.obstacles:
            if obst.check_collision(Point(point[0], point[1])) is True:
                return True
        # else:
        return False

    def check_collision_of_path(self, path):
        for obst in self.obstacles:
            if LineString(path).intersects(obst.obj):
                return True
        # else
        return False

    # --- Functions for nearest neighbour search
    def obtain_nearest_n(self, point, n=1, sort_results=False, return_distance=False):
        # use KDTree to store all the nodes
        # and query the nearest n neighbours
        if not sort_results:
            inds = self.tree.query(
                np.array([[point.x, point.y]]), k=n, return_distance=return_distance
            )[0]
            return [Point(x[0], x[1]) for x in self.nodes[inds]]
        elif sort_results and not return_distance:
            dis, inds = self.tree.query(
                np.array([[point.x, point.y]]), k=n, sort_results=sort_results
            )
            inds = inds[0]
            return [Point(x[0], x[1]) for x in self.nodes[inds]]
        elif sort_results and return_distance:
            dis, inds = self.tree.query(
                np.array([[point.x, point.y]]),
                k=n,
                sort_results=sort_results,
                return_distance=return_distance,
            )
            inds = inds[0]
            return [Point(x[0], x[1]) for x in self.nodes[inds]], dis[0]

    def obtain_nearest_in_r(
        self, point, r=2, sort_results=False, return_distance=False
    ):
        # use KDTree to store all the nodes
        # and query the nearest neighbours in r radius
        if not sort_results:
            inds = self.tree.query_radius(
                np.array([[point.x, point.y]]), r=r, sort_results=sort_results
            )[0]
            return [Point(x[0], x[1]) for x in self.nodes[inds]]
        elif sort_results:
            inds, dis = self.tree.query_radius(
                np.array([[point.x, point.y]]),
                r=r,
                sort_results=sort_results,
                return_distance=sort_results,
            )
            inds = inds[0]
            return [Point(x[0], x[1]) for x in self.nodes[inds]], dis[0]