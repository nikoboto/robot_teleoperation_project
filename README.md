To test this thing just:

GUI Test:
1. Run gui_commander.py
2. Click buttons and verify messages on /comandos using rostopic echo /comandos
   
Joystick Test:
1. Connect a joystick (if available, otherwise mock /joy topic)
2. Run joy_commander
3. Publish to /joy manually or use a real joystick
4. Verify messages on /comandos.
