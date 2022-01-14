def grab_piece(x, y):
    above_piece, rx', ry', rz' =-0.1502, -2.1890, 2.1920,=-0.0111
    at_piece, rx, ry, rz = -0.2025, -2.1890,2.1910,-0.0110
    gripper.open_gripper()
    rob.movel([x, y, above_piece, rx', ry', rz'],a,v)
    rob.movel([x, y, at_piece, rx, ry, rz],a,v)
    gripper.close_gripper()
    rob.movel([x, y, above_piece, rx, ry, rz],a,v)
def place_piece(x, y):
    rob.movel([x, y, above_piece, rx', ry', rz'],a,v)
    rob.movel([x, y, at_piece, rx, ry, rz],a,v)
    gripper.open_gripper()
    rob.movel([x, y, above_piece, rx', ry', rz'],a,v)
