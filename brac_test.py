import PySimpleGUI as sg

# Define the layout
layout = [
    [
        sg.Column([
            [sg.Button('Button 1'), sg.Button('Button 2'), sg.Button('Button 3')],
        ]),
        sg.Column([
            [sg.Multiline(size=(50, 10))],
        ]),
    ],
    [sg.Multiline(size=(50, 10))],
    [sg.Button('Exit')]
]

# Create the window
window = sg.Window('Two Columns Example', layout)

while True:
    event, values = window.read()

    if event in (sg.WIN_CLOSED, 'Exit'):
        break

window.close()
