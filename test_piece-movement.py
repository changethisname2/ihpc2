a, v = 0.5, 0.3
pos_dice_drop = [0.707, 0.219, 0.030, -2.205, 2.205, -0.043]
pos_dice_grab = [0.719, 0.219, -0.200, -2.205, 2.205, -0.043]
pos_dice_pic = [0.653, 0.187, -0.143, -2.189, 2.191, -0.011]
pos_board = [0.707, -0.053, 0.035, -2.042, 2.027, -0.376]
pos_boardj = [0.234, -1.847, -1.965, -0.558, 1.478, 0.208]

def grab_piece(x, y):
    above_piece, rxb, ryb, rzb = -0.1502, -2.1890, 2.1920, -0.0111
    at_piece, rx, ry, rz = -0.2025, -2.1890, 2.1910, -0.0110
    rob.movel([x, y, above_piece, rxb, ryb, rzb], a, v)
    rob.movel([x, y, at_piece, rx, ry, rz], a, v)
    gripper.close_gripper()
    rob.movel([x, y, above_piece, rx, ry, rz], a, v)

def place_piece(x, y):
    above_piece, rxb, ryb, rzb = -0.1502, -2.1890, 2.1920, -0.0111
    at_piece, rx, ry, rz = -0.2025, -2.1890, 2.1910, -0.0110
    rob.movel([x, y, above_piece, rxb, ryb, rzb], a, v)
    rob.movel([x, y, at_piece, rx, ry, rz],a,v)
    gripper.gripper_action(150)
    rob.movel([x, y, above_piece, rxb, ryb, rzb], a, v)

def move_piece(markerCenter_urx, steps, dice_num):
    for step in steps:
        if ((markerCenter_urx[0][0] >= (steps[step][0] - 0.01)) and (markerCenter_urx[0][0] <= (steps[step][0] + 0.01))) and ((markerCenter_urx[0][1] >= (steps[step][1] - 0.01)) and (markerCenter_urx[0][1] <= (steps[step][1] + 0.01))): 
            marker_pos = step
    diff = (57 - marker_pos)
    if diff < 6:
        if diff < dice_num:
            key = 57 - (dice_num - diff)
            grab_piece(markerCenter_urx[0][0], markerCenter_urx[0][1])
            place_piece(steps[key][0], steps[key][1])
        else:
            for step in steps:
                if ((markerCenter_urx[0][0] >= (steps[step][0] - 0.01)) and (markerCenter_urx[0][0] <= (steps[step][0] + 0.01))) and ((markerCenter_urx[0][1] >= (steps[step][1] - 0.01)) and (markerCenter_urx[0][1] <= (steps[step][1] + 0.01))):
                    next_step_num = step + dice_num
                    for nextStep in steps:
                        if next_step_num == nextStep:
                            grab_piece(markerCenter_urx[0][0], markerCenter_urx[0][1])
                            place_piece(steps[nextStep][0], steps[nextStep][1])
    else:
        for step in steps:
            if ((markerCenter_urx[0][0] >= (steps[step][0] - 0.01)) and (markerCenter_urx[0][0] <= (steps[step][0] + 0.01))) and ((markerCenter_urx[0][1] >= (steps[step][1] - 0.01)) and (markerCenter_urx[0][1] <= (steps[step][1] + 0.01))):
                next_step_num = step + dice_num
                for nextStep in steps:
                    if next_step_num == nextStep:
                        grab_piece(markerCenter_urx[0][0], markerCenter_urx[0][1])
                        place_piece(steps[nextStep][0], steps[nextStep][1])
