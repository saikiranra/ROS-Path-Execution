import math
import matplotlib.pyplot as plt

class robot:
    def __init__(self , width , maxSpeed , maxAcc , maxJerk):
        self.width = width
        self.maxSpeed = maxSpeed
        self.maxAcc = maxAcc
        self.maxJerk = maxJerk

    def generateXYFromLR(self, startXYA, TLRPoints):
        """
        Not finished
        startXYA - (xPos , yPos , angleInRad)
        LRPoints - [(LSpeed1 , RSpeed1) , ...]
        """
        if type(startXY) != tuple:
            raise Exception("Not tuple")
        if len(startXY) != 3:
            raise Exception("XYA not 2 points")
        out = [(0 , startXYA[0] , startXYA[1] , startXYA[2])]
        for LR in TLRPoints:
            t = out[-1][0]
            x = out[-1][1]
            y = out[-1][2]
            a = out[-1][3] #in radians
            nt = LR[0]
            r = (LR[1] + LR[2])/2 #units per second
            theta = (LR[1] - LR[2]) / self.width #radians per second
            dt = nt-t

    def getTXYoints(self, TXYPoints):
        """
        Save a dict of TXY points to the object
        NOTE: T is really dt

        so self.points["dt"] = [list of dt]
        self.points["X"] - [list of X]
        and so on
        """
        self.points = TXYPoints

    def loadTXYPointsFromFile(self , filename):
        """
        In
            filename - Name of file to parse
        Out
            Dictionary, keys being column name and val being list of points

        File Format - CSV style
        First line is column headers
        # denote lines that are comments
        """
        fp = open(filename , 'r')
        out = {}
        indexDict = {}
        first = True
        for line in fp:
            line=line.strip()
            splitline = line.split(",")
            if line[0] == "#":
                pass
            elif first:
                first = False
                for i in range(len(splitline)):
                    indexDict[i] = splitline[i]
                    out[splitline[i]] = []
            else:
                for i in range(len(splitline)):
                    out[indexDict[i]].append(float(splitline[i]))
        
        fp.close()
        self.getTXYoints(out)

    def getPosVelAccJerkHeading(self, startVel):
        length = len(self.points["X"])
        self.points["Pos"] = [0 for x in range(length)]
        self.points["Vel"] = [startVel] + [0 for x in range(length-1)]
        self.points["Acc"] = [0 for x in range(length)]
        self.points["Jerk"] = [0 for x in range(length)]
        self.points["Heading"] = [0 for x in range(length)]

        position = self.points["Pos"][0]

        for i in range(1, length):
            j = i - 1 #previous point

            newX = self.points["X"][i]
            newY = self.points["Y"][i]
            oldX = self.points["X"][j]
            oldY = self.points["Y"][j]

            dt = self.points["dt"][j]

            position +=  math.sqrt(((newX - oldX)**2) + ((newY - oldY)**2))
            self.points["Pos"][i] = position 

            vf = (position - self.points["Pos"][j])/dt #(2/dt)*(position - self.points["Pos"][j]) - self.points["Vel"][j]
            self.points["Vel"][i] = vf

            a = (vf - self.points["Vel"][j])/dt
            self.points["Acc"][i] = a

            j = (a - self.points["Acc"][j])/dt
            self.points["Jerk"][i] = j

    def adjustForwardVelAccJerk(self):
        length = len(self.points["X"])

        for i in range(1, length):
            j = i - 1 #previous point

            self.points["Vel"][i] = min(self.maxSpeed , self.points["Vel"])
            if self.points["Vel"][i] != 0:
                newdt = (self.points["Pos"][i] - self.points["Pos"][j])/self.points["Vel"][i]
            #newdt = (2*(self.points["Pos"][i] - self.points["Pos"][j]) - self.points["Vel"][j]) / self.points["Vel"][i]

            self.points["Acc"][i] = min(self.maxAcc , (self.points["Vel"][i] - self.points["Vel"][j])/newdt)
            if self.points["Acc"][i] != 0:
                newdt = (self.points["Vel"][i] - self.points["Vel"][j])/self.points["Acc"][i]

            self.points["Jerk"][i] = min(self.maxJerk , (self.points["Acc"][i] - self.points["Acc"][j])/newdt)
            if self.points["Jerk"][i] != 0:
                newdt = (self.points["Acc"][i] - self.points["Acc"][j])/self.points["Jerk"][i]

            self.points["dt"][i] = newdt

    def graph(self , length , xLambda , yLambda , name):
        plt.plot([x for x in xLambda(length)] , [y for y in yLambda(length)] , label = name)
        plt.legend()
        

    def pointGen(self , name):
        def lamb(length):
            t = 0
            for i in range(length):
                if name == "time":
                    yield(t)
                    t += self.points['dt'][i]
                else:
                    yield self.points[name][i]
        return lamb



if __name__ == "__main__":
    clyde = robot(2 , 0.65 , 1 , 1)
    clyde.loadTXYPointsFromFile("points.txt")
    clyde.getPosVelAccJerkHeading(0)
    clyde.graph(len(clyde.points["X"])  , clyde.pointGen("time") , clyde.pointGen("Vel") , "Unadjusted Velocity over Time")
    clyde.adjustForwardVelAccJerk()
    plt.figure()
    clyde.graph(len(clyde.points["X"])  , clyde.pointGen("time") , clyde.pointGen("Vel") , "Adjusted Velocity over Time")

    plt.show()





