#!/usr/bin/env python
import rospy
import PySimpleGUI27 as sg
import os.path
from PIL import Image

rospy.init_node('cbotManager')

sizeImage = (500,500)
imageName = "/home/mohit/nio/src/ROS-Cbot-master/cbot_manager/src/cbot_image.png"
newImageName = "/home/mohit/nio/src/ROS-Cbot-master/cbot_manager/src/cbot_image_resized.png"
image = Image.open(imageName)
image = image.resize((sizeImage))
image.save(newImageName)

# file_list_column = [
    # [
    #     sg.Text("Image Folder"),
    #     sg.In(size=(25, 1), enable_events=True, key="-FOLDER-"),
    #     sg.FolderBrowse(),
    # ],
    # [
    #     sg.Listbox(
    #         values=[], enable_events=True, size=(40, 20), key="-FILE LIST-"
    #     )
    # ],
# ]
buttons = [
    [sg.Frame(layout=[
        [sg.Checkbox('AUV Mode', default=False, key='AUV_mode'),
         sg.Checkbox('Hybrid Mode', default=False, key='Hybrid_mode'),
         sg.Checkbox('ROV Mode', default=False, key='ROV_mode')]],
                     title='Select Mode',title_color='yellow')],
    [sg.Frame(layout=[
        [sg.Checkbox('Drive', default=False, key='Drive'),
         sg.Checkbox('Park', default=False, key='Park')]],
                     title='AUV Mission Status',title_color='yellow')],
    [sg.Frame(layout=[
        [sg.Checkbox('Guidance', default=False, key='GUIDANCE_ON'),
         sg.Checkbox('Controller', default=False, key='CONTROLLER_ON'),
         sg.Checkbox('HIL', default=True, key='HIL_ON')]],
                     title='Autoset by Mode',title_color='yellow')],
    [sg.Submit('Update')],
]

image_viewer_column = [
    [sg.Image(filename=newImageName,size=sizeImage, key="-IMAGE-")],
]

layout = [
    [sg.Column(buttons),   
    sg.VSeperator(),
    sg.Column(image_viewer_column)],
]

# sg.theme("DarkAmber")# or event == sg.WIN_CLOSED):
window = sg.Window("CBOT Control", layout)

while not rospy.is_shutdown():
    event, values = window.read()
    print(event,values)
    if(event == "Exit" or event == None):
        break
    if(event=="Update"):
        if(values['ROV_mode']):
            rospy.set_param('/Mode', "ROV")
            window.Element('AUV_mode').Update(False)
            window.Element('Hybrid_mode').Update(False)
            window.Element('Drive').Update(False)
            window.Element('Park').Update(False)
            window.Element('CONTROLLER_ON').Update(False)
            window.Element('GUIDANCE_ON').Update(False)
            rospy.set_param('/GUIDANCE_ON', 0)
            rospy.set_param('/Controller_ON', 0)

        elif(values['Hybrid_mode']):
            rospy.set_param('/Mode', "Hybrid")
            window.Element('ROV_mode').Update(False)
            window.Element('AUV_mode').Update(False)
            rospy.set_param('/Controller_ON', 1)
            window.Element('CONTROLLER_ON').Update(True)
            rospy.set_param('/GUIDANCE_ON', 0)
            window.Element('GUIDANCE_ON').Update(False)

        elif(values['AUV_mode']):
            rospy.set_param('/Mode', "AUV")
            window.Element('ROV_mode').Update(False)
            rospy.set_param('/Controller_ON', 1)
            window.Element('CONTROLLER_ON').Update(True)
            rospy.set_param('/GUIDANCE_ON', 1)
            window.Element('GUIDANCE_ON').Update(True)

        if(values['Park']):
            rospy.set_param("Status","Park")
            window.Element('Drive').Update(False)
        elif(values['Drive']):
            rospy.set_param("Status","Drive")
            window.Element('Park').Update(False)

        else:
            rospy.set_param('/Mode', "Invalid")
            rospy.set_param('/Status', "Invalid")
            rospy.set_param('/Controller_ON', 0)
            rospy.set_param('/GUIDANCE_ON', 0)
            window.Element('Drive').Update(False)
            window.Element('Park').Update(False)
            # window.Element('AUV_pause').Update(False)
            window.Element('CONTROLLER_ON').Update(False)
            window.Element('GUIDANCE_ON').Update(False)

        rospy.set_param('/HIL_ON', 1 if(values['HIL_ON']) else 0)

window.close()