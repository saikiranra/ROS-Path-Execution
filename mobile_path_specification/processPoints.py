import matplotlib.pyplot as plt
import numpy as np

def processFile(filename , names , headerIncluded):
	fp = open(filename)
	outx = {}
	outy = []
	index = 0
	first = True
	for name in names:
		outx[name] = []
	for line in fp:
		line = line.strip()
		splitLine = line.split(",")
		if not(headerIncluded and first):
			for i in range(len(names)):
				outx[names[i]].append(float(splitLine[i]))
			outy.append(index)	
			index += 1
		if headerIncluded:
			if first:
				names = []
				for name in splitLine:
					outx[name] = []
					names.append(name)
				first = False
		
		
	fp.close()
	return((outx , outy))

def drawGraph(outx , outy):
	if type(outx) == dict:
		for name in outx.keys():
			plt.plot(outy , outx[name] , label = name)
	if type(outx) == tuple:
		plt.plot(outy , outx[0] , label = outx[1])
	plt.legend()
	

if __name__ == "__main__":
	#data = processFile("sensorLog.txt" , ["Right" , "Left" , "Mid"] , False)
	#drawGraph(data[0] , data[1])
	data = processFile("../../../.ros/pathLog.txt" , [] , True)
	print(data[0].keys())
	drawGraph((data[0]["Robot Y"] , "Robot") , data[0]["Robot X"])
	drawGraph((data[0]["Close Y"] , "Close") , data[0]["Close X"])
	drawGraph((data[0]["Goal Y"] , "Goal") , data[0]["Goal X"])
	plt.figure()
	try:
		data = processFile("../../../.ros/speedLog.txt" , [] , True)
		drawGraph((data[0]["Linear"] , "Linear Speed") , data[0]["time"])
		drawGraph((data[0]["Angular"] , "Angluar Speed") , data[0]["time"])
	except:
		pass
	plt.show()
