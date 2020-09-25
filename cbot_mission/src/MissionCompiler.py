import json
	
fileName = "Mission.txt"
f = open(fileName,"r")

MissionTypes = {"guidance": ["wpt", "lfw", "arc", "dock"],
				"guidance2": ["constDepth", "constHeading", "constPitch", "constSpeed", "surfacing"],
				"behaviour":["loiter"]}

params = {"wpt": ["position", "depth", "speed", "heading", "captureRadius", "slipRadius", "timeout"],
		  "lfw": ["position1", "position2", "depth", "speed", "heading", "captureRadius", "timeout"],
		  "arc": ["centerCoord", "depth", "radius","speed", "heading", "captureRadius", "direction", "start", "timeout"],
		  "dock": ["position", 	"depth", "heading", "runwayLength"],
		  "constDepth": ["depth"],
		  "constSpeed": ["speed"],
		  "constHeading": ["heading"],
		  "constPitch": ["pitch"],
		  "loiter": ["timeout"]}

overrideParams = {"constDepth": ["depth"],
				  "constSpeed": ["speed"],
				  "constHeading": ["heading"],
				  "constPitch": ["pitch"],
				  "loiter": ["timeout"]}

commentTag = "#"

MissionDict = {}
MissionNameTable = {}
BHVNameTable = {}
lineCount = 0

def readNewLine(splitStr = ' '):
	global lineCount
	line = f.readline()
	try:
		commentIndex = line.index(commentTag)
		line = line[0:commentIndex].strip()
	except:
		line = line.strip()
	lineCount +=1

	while(line==''):
		line = f.readline()
		try:
			commentIndex = line.index(commentTag)
			line = line[0:commentIndex].strip()
		except:
			line = line.strip()
		lineCount +=1
	line = line.split(splitStr)
	line = [x.strip() for x in line]
	return line

def addData(mission,data,override):
	global MissionNameTable

	missionType = mission[0]
	line = readNewLine(splitStr = ':')	
	if(missionType in MissionNameTable.keys()):
		return MissionNameTable[line[0]][data]
		
	while(line[0] != "end"):
		if(line[0] in params[missionType]):
			data[line[0]] = ','.join(line[1:])
		line = readNewLine(splitStr = ':')

	for key in override.keys():
		data[key] = override[key]

def updateOverride(missionType,override):
	line = readNewLine(splitStr = ':')
	if(line[0]=="vars"):
		while(line[0]!="end"):
			line = readNewLine(splitStr = ':')
			if(line[0] in overrideParams[missionType]):
				override[line[0]] = ','.join(line[1:])
			elif(line[0]!="end"):
				err = "Expected a \"end\" statement before line " + str(lineCount)
				raise SyntaxError(err)
	return override


def parseMission(line,count, override, suffix, singleMission):
	global MissionTypes, MissionNameTable, MissionDict, BHVNameTable
	m = line[0]
	if(m in BHVNameTable.keys()):
		singleMission.append(m)
	elif(m in MissionNameTable.keys()):
		singleMission.append(m + suffix)
		if(suffix!=''):
			MissionNameTable[m + suffix] = {"type": MissionNameTable[m]["type"], "data": {}}
			for key in MissionNameTable[m]["data"].keys():
				MissionNameTable[m + suffix]["data"][key] = MissionNameTable[m]["data"][key]
			for key in override.keys():
				MissionNameTable[m + suffix]["data"][key] = override[key]

	elif (m in MissionTypes["guidance"]):
		singleMission.append(line[1] + suffix)
		MissionNameTable[line[1] + suffix] = {"type": line[0], "data": {}}
		addData(line,MissionNameTable[line[1] + suffix]["data"],override)

	elif(m in MissionTypes["guidance2"]):
		suffix += line[1]
		override = updateOverride(m,override)
		line = readNewLine()
		while(line[0]!='end'):
			singleMission = parseMission(line,count,override,suffix, singleMission)
			line = readNewLine()

	elif(m in MissionTypes["behaviour"]):
		tout = {}
		bhvMission = []
		tout = updateOverride(m,tout)
		BHVNameTable[line[1]] = {"type":m}
		for key in tout.keys():
			BHVNameTable[line[1]][key] = tout[key]
		line2 = readNewLine()
		while(line2[0]!='end'):
			singleMission = []
			singleMission = parseMission(line2,count,override,suffix,singleMission)
			if(line2[0] in BHVNameTable.keys()):
				bhvMission.append(line2[0]) 
			elif(line2[0] in MissionTypes["behaviour"] or line2[0] in MissionTypes["guidance"]):
				bhvMission.append(line2[1])
			elif(line2[0] in MissionTypes["guidance2"]):
				bhvMission.append(singleMission[0])
			line2 = readNewLine()
		BHVNameTable[line[1]]["names"] = bhvMission
		return bhvMission

	return singleMission

def main():
	line = readNewLine()
	count = 0

	while(line[0]!='END'):
		override = {}
		singleMission = []
		suffix = ""
		count += 1

		if(line[0] in MissionNameTable.keys() or line[0] in BHVNameTable.keys()):
			MissionDict['M'+str(count)] = {}
			MissionDict['M'+str(count)]["names"] = [line[0]]

		elif((line[0] in MissionTypes["guidance"]) or (line[0] in MissionTypes["guidance2"])):
			MissionDict['M'+str(count)] = {}
			singleMission = parseMission(line,count,override,suffix,singleMission)
			MissionDict['M'+str(count)]["names"] = singleMission

		elif(line[0] in MissionTypes["behaviour"]):
			MissionDict['M'+str(count)] = {}
			MissionDict['M'+str(count)]["names"] = [line[1]]
			singleMission = parseMission(line,count,override,suffix,singleMission)
			BHVNameTable[line[1]]["names"] = singleMission
		
		line = readNewLine()

	Mission = {}
	Mission["Missions"] = MissionDict
	Mission["GuidanceNameTable"] = MissionNameTable
	Mission["BHVNameTable"] = BHVNameTable

	f.close()

	with open('result.json', 'w') as fp:
		json.dump(Mission,fp,indent=4)
		fp.close()

	return Mission

def readMission(filename):
	global fileName,f
	fileName = filename
	f = open(fileName,"r")
	return main()

if __name__=="__main__":
	main()