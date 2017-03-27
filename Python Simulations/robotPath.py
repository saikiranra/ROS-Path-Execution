
class robot:
    def __init__(self , width , maxSpeedR , maxSpeedL):
        self.width = width
        self.maxSpeedR = maxSpeedR
        self.maxSpeedL = maxSpeedL

    def generateXYFromLR(self, startXYA, TLRPoints):
        """
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
