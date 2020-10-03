from pykml import parser
from os import path
import json


Mission = {"GuidanceNameTable": {}, "Missions": {}, "BHVNameTable": {}}

def addWPT(coords, names, missionType):
	Mission["GuidanceNameTable"][names[0]] = {"data":{}, "type":missionType}
	Mission["GuidanceNameTable"][names[0]]["data"]["position"] = "(" + str(coords[1]) + "," + str(coords[0]) + ")"
	Mission["GuidanceNameTable"][names[0	]]["data"]["speed"] = "0.5"
 
def addLFW(coords, names, missionType):
	for i in (range(len(names)-1)):
		Mission["GuidanceNameTable"][names[i]] = {"data":{}, "type": missionType}
		Mission["GuidanceNameTable"][names[i]]["data"]["position1"] = "(" + coords[i][1] + "," + coords[i][0] + ")"
		Mission["GuidanceNameTable"][names[i]]["data"]["position2"] = "(" + coords[i+1][1] + "," + coords[i+1][0] + ")"
		Mission["GuidanceNameTable"][names[i]]["data"]["speed"] = "0.5"

def read(filename="test.kml"):
	kml_file = path.join(filename)
	with open(kml_file) as f:
		doc = parser.parse(f).getroot()
		count=0
		# Parse each mission
		for e in doc.Document.Placemark:
			count += 1
			coords = []
			name = []
			missionType = ""
			try:
				missionType = "lfw"
				t = e.LineString.coordinates.text.strip(" \n\t").replace(" ",",").split(',')
				i = 0
				while(i<len(t)):
					name.append(e.name.text + str(int(i/3)))
					l = t[i:i+3]
					coords.append(l)
					i +=3
			except:
				name.append(e.name.text)
				missionType = "wpt"
				coords = e.Point.coordinates.text.strip(" \n\t").split(',')
				
			# Add mission to dictionary
			if(len(coords)!=0):
				Mission["Missions"]["M"+str(count)] = {"names": name}
				if (missionType=="wpt"):
					addWPT(coords,name,missionType)
				elif(missionType=="lfw"):
					addLFW(coords,name,missionType)

		with open('result2.json', 'w') as fp:
			json.dump(Mission,fp,indent=4)
			fp.close()

		return Mission

if __name__=="__main__":
	read()