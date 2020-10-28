########################################################
# Mission Compiler made for NIO
# Author: Mohit Gupta, BITS Goa
###################################################

writeToFile = 1 #Write mission to json file for check

try:
	import json
except:
	print("[Import Error] Cannot write to JSON file.")
	writeToFile = 0

fileName = ""

MissionTypes = {"guidance": ["wpt", "wptFA", "lfw", "lfwFA", "arc", "arcFA", "dock"],
				"guidance2": ["constDepth", "constHeading", "constPitch", "constSpeed","constThrust", "surfacing"],
				"behaviour":["loiter"]}

# List of all possible paramerters for the guidance missions
params = {"wpt": ["position", "depth", "speed", "captureRadius", "slipRadius", "timeout"],
		  "wptFA": ["position", "depth", "speed", "heading","pitch","roll", "captureRadius", "slipRadius", "timeout"],
		  "lfw": ["position1", "position2", "depth", "speed", "captureRadius", "timeout"],
		  "lfwFA": ["position1", "position2", "depth", "speed", "heading","pitch","roll", "captureRadius", "timeout"],
		  "arc": ["centerCoord", "depth", "radius","speed", "captureRadius", "direction", "start", "timeout"],
		  "arcFA": ["centerCoord", "depth", "radius","speed", "heading","pitch","roll", "captureRadius", "direction", "start", "timeout"],
		  "dock": ["position", 	"depth", "heading", "runwayLength"]}

# List of parameters without which the mission would be invalid
impParams = {"wpt": ["position","speed"],
			 "wptFA": ["position","speed"],
			 "lfw": ["position1", "position2", "speed"],
			 "lfwFA": ["position1", "position2", "speed"],
			 "arc": ["centerCoord", "radius", "speed", "start"],
			 "arcFA": ["centerCoord", "radius", "speed", "start"],
			 "dock": ["position", 	"depth", "heading", "runwayLength"]}

# List of parameters which can override mission parameters when specified
overrideParams = {"constDepth": ["depth","timeout"],
				  "constSpeed": ["speed","timeout"],
				  "constHeading": ["heading","timeout"],
				  "constPitch": ["pitch","timeout"],
				  "constThrust": ["cmf","dmf","cmv","dmv", "timeout"],
				  "loiter": ["timeout"]}

safetyParams = ["maxDepth", "maxPitch", "maxRoll", "maxSpeed"]

commentTag = "#"		# Comment tag to be used in the mission file 
missionEndTag = "END"	# Defines the end of Mission file

MissionDict = {}
GuidanceTable = {}
BHVNameTable = {}
Safety = {}

lineCount = 0 # Variable to store line number to be used in case of error 


def parseSafetyParams():
	line = readNewLine(splitStr=":")
	while(line[0]!="end"):
		if(line[0] in safetyParams):
			Safety[line[0]] = line[1]
		else:
			err = "Invalid Syntax or missing \"end\" tag while parsing Safety Parameters before Line: " + str(lineCount)
			raise SyntaxError(err)
		line = readNewLine(splitStr=":")

def readNewLine(splitStr = ' '):
	global lineCount
	line = f.readline()
	
	# Check and remove comments in the line
	try:
		commentIndex = line.index(commentTag) # Get the index position of comment tag
		line = line[0:commentIndex].strip() # Remove blank space from start and end
	except:
		line = line.strip() # Remove blank space from start and end
	lineCount +=1 #Increment the line count 

	# Loop to skip any empty lines read
	while(line==''):
		line = f.readline()
		# Check and remove comments in the line
		try:
			commentIndex = line.index(commentTag)
			line = line[0:commentIndex].strip()
		except:
			line = line.strip()
		lineCount +=1

	line = line.split(splitStr) # Split the line read based on the splitting variable
	line = [x.strip() for x in line] # Remove start and end white space from every element in list
	return line

def addData(mission,data,override):
	global GuidanceTable

	missionType = mission[0]
	line = readNewLine(splitStr = ':')

	# Check if the mission is defined before
	if(missionType in GuidanceTable.keys()):
		return GuidanceTable[line[0]][data]
	
	countImpParams = {}
	for key in impParams[missionType]:
		countImpParams[key] = 0

	missionName = mission[1]
	# Add data 
	while(line[0] != "end"):
		if(line[0] in params[missionType]):
			data[line[0]] = ','.join(line[1:])
		else:
			err = "Invalid syntax or a missing \"end\" statement.\n Mission Name: "+ missionName+"\n Mission Type: " + missionType + "\n Line Numer: " + str(lineCount)
			raise SyntaxError(err)
		if(line[0] in impParams[missionType]):
			countImpParams[line[0]] +=1
		line = readNewLine(splitStr = ':')

	# Check for all compulsary parameters in mission
	for key in countImpParams.keys():
		if(countImpParams[key]==0):
			err = "Incomplete mission\n Mission Name: " + str(missionName) + "\n Type: " + missionType + "\n Line Numer: " + str(lineCount) + "\n Missing Parameter: " + key 
			raise Exception(err)

	# Add override data
	for key in override.keys():
		data[key] = override[key]

def updateOverride(missionType,override):
	line = readNewLine(splitStr = ':')
	if(line[0]=="vars"):
		while(line[0]!="end"):
			line = readNewLine(splitStr = ':')
			if(line[0] in overrideParams[missionType]):
				override[line[0]] = ','.join(line[1:])
			elif(line[0]!="end" or line[0]==missionEndTag):
				err = "Invalid syntax or missing \"end\" statement before line " + str(lineCount)
				raise SyntaxError(err)
	else:
		err = "Missing override Parameter \"vars\" at Line: " + str(lineCount) + "\n Mission Type: " + missionType 
		raise Exception(err)
	return override


def parseMission(line,count, override, suffix, singleMission):
	global MissionTypes, GuidanceTable, MissionDict, BHVNameTable
	m = line[0]
	# Check if the name is parsed previously as a behaviour
	if(m in BHVNameTable.keys()):
		singleMission.append(m)
		
	# Check if the name is parsed previously as a guidance
	elif(m in GuidanceTable.keys()):
		singleMission.append(suffix + m)
		if(suffix!=''):
			GuidanceTable[suffix + m] = {"type": GuidanceTable[m]["type"], "data": {}}
			for key in GuidanceTable[m]["data"].keys():
				GuidanceTable[suffix + m]["data"][key] = GuidanceTable[m]["data"][key]
			for key in override.keys():
				GuidanceTable[suffix + m]["data"][key] = override[key]

	# Parse the new guidance mission
	elif (m in MissionTypes["guidance"]):
		singleMission.append(suffix + line[1])
		GuidanceTable[suffix + line[1]] = {"type": line[0], "data": {}}
		addData(line,GuidanceTable[suffix + line[1]]["data"],override)

	# Check if the mission is for overriding data
	elif(m in MissionTypes["guidance2"]):
		suffix += line[1]
		override = updateOverride(m,override)
		line = readNewLine()
		if(line[0]=="end"):
			singleMission.append(suffix)
			GuidanceTable[suffix] = {"type": m, "data": {}}
			for key in override.keys():
				GuidanceTable[suffix]["data"][key] = override[key]

		else:
			while(line[0]!='end'):
				temp_override = {}
				for param in override:
					temp_override[param] = override[param]
				singleMission = parseMission(line,count,temp_override,suffix, singleMission)
				line = readNewLine()


	# Check if the mission type is a new behavior
	elif(m in MissionTypes["behaviour"]):
		tout = {}
		bhvMission = []
		missionName = line[1]
		tout = updateOverride(m,tout) # get the additional behaviour variables 
		BHVNameTable[line[1]] = {"type":m} # Add behaviour type
		for key in tout.keys():	# Add behaviour variables to mission data
			BHVNameTable[line[1]][key] = tout[key]
		line2 = readNewLine()
		while(line2[0]!='end'):
			if(line2[0]==missionEndTag):
				err = "Reached the end of mission file \n Missing \"end\" tag.\n Mission Name:" + missionName +"\n Mission Type: " + m + "\n Line: " + str(lineCount)
				raise SyntaxError(err)
			singleMission = []
			singleMission = parseMission(line2,count,override,suffix,singleMission)
			if(line2[0] in BHVNameTable.keys() or line2[0] in GuidanceTable.keys()):
				bhvMission.append(line2[0]) 
			elif(line2[0] in MissionTypes["behaviour"] or line2[0] in MissionTypes["guidance"]):
				bhvMission.append(line2[1])
			elif(line2[0] in MissionTypes["guidance2"]):
				bhvMission.append(singleMission[0])
			else:
				err = "Invalid Syntax at Line: " + str(lineCount)
				raise SyntaxError(err)
			line2 = readNewLine()
		BHVNameTable[line[1]]["names"] = bhvMission
		return bhvMission

	return singleMission

def main():
	global MissionDict, GuidanceTable, BHVNameTable, Safety
	line = readNewLine()
	count = 0 # To store the mission number

	MissionDict = {}
	GuidanceTable = {}
	BHVNameTable = {}
	Safety = {}

	while(line[0]!=missionEndTag):
		override = {}
		singleMission = []
		suffix = ""
		count += 1

		# Parse Safety params
		if(line[0]=="Safety"):
			count-=1
			parseSafetyParams()

		# Check if the read line is name of pre-parsed behavior name
		elif(line[0] in GuidanceTable.keys() or line[0] in BHVNameTable.keys()):
			MissionDict['M'+str(count)] = {}
			MissionDict['M'+str(count)]["names"] = [line[0]]

		# Check if the mission is a guidance
		elif((line[0] in MissionTypes["guidance"]) or (line[0] in MissionTypes["guidance2"])):
			MissionDict['M'+str(count)] = {}
			singleMission = parseMission(line,count,override,suffix,singleMission)
			MissionDict['M'+str(count)]["names"] = singleMission

		# Check if the mission is a behaviour
		elif(line[0] in MissionTypes["behaviour"]):
			MissionDict['M'+str(count)] = {}
			MissionDict['M'+str(count)]["names"] = [line[1]]
			singleMission = parseMission(line,count,override,suffix,singleMission)
			BHVNameTable[line[1]]["names"] = singleMission
		
		line = readNewLine()

	# Create the final dictionary to be returned
	Mission = {}
	Mission["Missions"] = MissionDict
	Mission["GuidanceTable"] = GuidanceTable
	Mission["BHVTable"] = BHVNameTable
	Mission["Safety"] = Safety

	f.close() # Close the mission file

	# Dump the mission dictionary as JSON file for the purpose of testing
	# This can be removed for final implementation
	if(writeToFile):
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
	readMission(filename="Mission.txt")