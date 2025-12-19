import math
import win32com.client
import time
import numpy as np

def get_board_coordinates_v3(square_name):
    # Setup
    square_size = 25
    board_start_distance = 60  # X axis e board shuru

    file_map = {'a': 0, 'b': 1, 'c': 2, 'd': 3, 'e': 4, 'f': 5, 'g': 6, 'h': 7}

    # Input Validation
    square_name = square_name.lower()
    if len(square_name) != 2: return None

    file_char, rank_char = square_name[0], square_name[1]
    col_idx = file_map[file_char]  # 0 to 7
    row_idx = int(rank_char) - 1  # 0 to 7

    # --- Calculation ---
    # Row 1 is closest (Index 0)
    x = board_start_distance + (row_idx * square_size) + (square_size / 2)

    # Y Axis (Side): Left/Right Center
    # a-d: Negative Y
    # e-h: Positive Y
    y = (col_idx - 3.5) * square_size

    return x, y

def ik_4dof_vertical(x, y, z, L1, L2, L3, L4):
    theta1 = math.atan2(y, x)

    r = math.sqrt(x**2 + y**2)

    R = math.sqrt((L1-z-L4)**2 + r**2)

    theta3 = math.acos( (L2**2 + L3**2 - R**2)/(2*L2*L3) )

    theta2 = math.atan2(r,(L1-z-L4)) + math.acos( (R**2 + L2**2 - L3**2)/(2*L2*R) )

    theta4 = math.pi/2 + math.atan2( (L1 - z - L4),r ) + math.acos((R ** 2 + L3 ** 2 - L2 ** 2) / (2 * L3 * R))

    return [
        math.degrees(theta1),
        math.degrees(theta2),
        math.degrees(theta3),
        math.degrees(theta4)
    ]

# Function to get current angle (in rad)
def get_current_angle(model,param_name):
    param = model.Parameter(param_name)
    if param is None:
        raise ValueError(f"Parameter {param_name} not found")
    return param.SystemValue

# Function to generate interpolation list
def generate_steps(current, target, step_rad):
    if current < target:
        steps = np.arange(current, target, step_rad).tolist()
        steps.append(target)  # ensure exact final
    else:
        steps = np.arange(current, target, -step_rad).tolist()
        steps.append(target)
    return steps

swSuppress = 0
swUnsuppress = 1

def set_mate_state(model, mate_name, suppress=True):
    feat = model.FeatureByName(mate_name)
    if feat is None:
        raise ValueError("Mate not found")

    state = swSuppress if suppress else swUnsuppress
    feat.SetSuppression2(state, 2, None)
    model.ForceRebuild3(False)

def grip_on(model):
    set_mate_state(model, "Concentric5", suppress=False)
    set_mate_state(model, "Coincident6", suppress=False)

def grip_off(model):
    # Drop piece
    set_mate_state(model, "Concentric5", suppress=True)
    set_mate_state(model, "Coincident6", suppress=True)

def arm_movement(model,square,z_height,joint_order):
    # Joint angle parameters in order
    param_names = [
        "D1@Angle1",
        "D1@Angle2",
        "D1@Angle3",
        "D1@Angle4"
    ]

    x1, y1 = get_board_coordinates_v3(square)
    # Target angles in degrees (from IK or desired)
    t1, t2, t3, t4 = ik_4dof_vertical(x=x1, y=y1, z=z_height, L1=90, L2=150, L3=150, L4=55)
    ang2_out = 270 - t2
    target_angles_deg = [90 + t1, ang2_out, t3,
                         t4]  # Example targets, for angle2 output theta = 270 - input theta

    step_deg = 10  # increment step in degrees
    # Convert step to radians
    step_rad = math.radians(step_deg)

    # Loop over each joint sequentially
    for i in joint_order:
        param_name = param_names[i]
        target_rad = math.radians(target_angles_deg[i])
        current_rad = get_current_angle(model, param_name)

        angle_list = generate_steps(current_rad, target_rad, step_rad)

        for ang in angle_list:
            model.Parameter(param_name).SystemValue = ang
            model.ForceRebuild3(False)
            time.sleep(0.05)  # adjust for smoothness


def main():
    swApp = win32com.client.Dispatch("SldWorks.Application")
    swApp.Visible = True

    model = swApp.ActiveDoc

    move = input("EnterSquareName:")

    APPROACH_SEQ = [0, 3, 2, 1]  # t1, t4, t3, t2
    RETRACT_SEQ = [1, 2, 3, 0]  # t2, t3, t4, t1

    # ---- PICK ----
    arm_movement(model, square=move[:2], z_height=60, joint_order=APPROACH_SEQ)
    arm_movement(model, square=move[:2], z_height=20, joint_order=APPROACH_SEQ)
    grip_on(model)

    # ---- LIFT STRAIGHT UP ----
    arm_movement(model, square=move[:2], z_height=80, joint_order=RETRACT_SEQ)

    # ---- TRAVEL HIGH ----
    arm_movement(model, square=move[2:], z_height=80, joint_order=APPROACH_SEQ)

    # ---- PLACE ----
    arm_movement(model, square=move[2:], z_height=20, joint_order=APPROACH_SEQ)
    grip_off(model)

    # ---- RETRACT ----
    arm_movement(model, square=move[2:], z_height=80, joint_order=RETRACT_SEQ)


if __name__ == "__main__":
    main()